import math
import numpy as np
import rasterio
from rasterio.enums import Resampling
from rasterio.windows import Window
from affine import Affine

_LAT_M = 111139.0  # metres per degree latitude (WGS-84 approximation)

# Above this many cells we never hold the whole raster in RAM at once. A giga-pixel
# DTM at native resolution can be many GB (e.g. 54591x42838 float32 ~ 9 GB). Small
# rasters (<= this) are read once at native resolution — fast and exact. Larger ones
# keep `array is None` and are read through native WINDOWS on demand (elevation_at_many
# / read_window): the terrain planning sees is never coarsened, only ever a bounded
# window is materialised. So a large DTM behaves the same as a small one, it just
# streams. Nothing is ever written to disk — the source raster is left untouched.
_MAX_CELLS = 64_000_000

# Whole-extent display overview budget (cells). The map is a decimated overview of at
# most this many pixels (read once at load); planning never uses it.
_DISP_CELLS = 9_000_000


class DTM:
    """Elevation model with size-transparent, native-resolution access.

    `transform` / `nodata` are native and `src.res` is the native resolution whatever
    the raster's size. `elevation_at[_many]` return native-resolution terrain: a small
    raster is sampled from the in-RAM `array`; a large raster (`array is None`) reads
    the native window covering the query points on demand (a recent native window is
    cached). `read_window` materialises a native AOI view (WITH `array`) for consumers
    that need a contiguous block + gradient (planner, density estimate, HELIOS);
    `overview_array` gives a bounded decimated whole-extent grid for the map only.
    """

    def __init__(self, path):
        self.src = rasterio.open(path)
        self.transform = self.src.transform            # NATIVE, always
        self.nodata = self.src.nodata
        self.width, self.height = self.src.width, self.src.height
        self._inv = ~self.transform                    # world -> pixel
        self._win = None                               # cached native window (point queries)
        self._ov = None                                # cached whole-extent display overview
        if self.width * self.height <= _MAX_CELLS:
            self.array = self.src.read(1)              # small: whole native grid in RAM
        else:
            self.array = None                          # large: native windows on demand

    # ---- bilinear sampling, shared by the in-RAM and windowed paths ----------
    @staticmethod
    def _bilinear(arr, inv, xs, ys, nodata):
        """Vectorised bilinear sample of `arr` (world->pixel affine `inv`) at world
        coords xs, ys. NaN where the 2x2 stencil falls off `arr`, and NaN where all
        four surrounding cells are nodata (otherwise the raw bilinear value)."""
        out = np.full(np.broadcast(xs, ys).shape, np.nan, dtype=float)
        if out.size == 0:
            return out
        xf = np.broadcast_to(xs, out.shape).ravel()
        yf = np.broadcast_to(ys, out.shape).ravel()
        col = inv.a * xf + inv.b * yf + inv.c
        row = inv.d * xf + inv.e * yf + inv.f
        j = np.floor(col).astype(np.int64)
        i = np.floor(row).astype(np.int64)
        h, w = arr.shape
        ok = (i >= 0) & (j >= 0) & (i + 1 < h) & (j + 1 < w)
        flat = out.ravel()
        if np.any(ok):
            ii, jj = i[ok], j[ok]
            dx = col[ok] - jj
            dy = row[ok] - ii
            z00 = arr[ii, jj].astype(float)
            z10 = arr[ii, jj + 1].astype(float)
            z01 = arr[ii + 1, jj].astype(float)
            z11 = arr[ii + 1, jj + 1].astype(float)
            z = (z00 * (1 - dx) + z10 * dx) * (1 - dy) + \
                (z01 * (1 - dx) + z11 * dx) * dy
            if nodata is not None:
                allnd = ((z00 == nodata) & (z10 == nodata)
                         & (z01 == nodata) & (z11 == nodata))
                z = np.where(allnd, np.nan, z)
            flat[ok] = z
        return flat.reshape(out.shape)

    def elevation_at(self, x, y):
        """Bilinearly-interpolated elevation at world coords (x, y), same CRS as the
        raster. NaN outside the raster / all-nodata stencil."""
        return float(self.elevation_at_many(np.asarray([x], dtype=float),
                                             np.asarray([y], dtype=float))[0])

    def elevation_at_many(self, xs, ys):
        """Vectorised native-resolution bilinear elevation for arrays of world coords.
        Samples the in-RAM `array` directly when present (the hot path); otherwise reads
        the native window covering the query points (reusing a cached window) — same
        numbers, bounded RAM."""
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        if self.array is not None:
            return self._bilinear(self.array, self._inv, xs, ys, self.nodata)
        return self._sample_windowed(xs, ys)

    # ---- native windowed point sampling (large rasters) ----------------------
    def _sample_windowed(self, xs, ys):
        out = np.full(np.broadcast(xs, ys).shape, np.nan, dtype=float)
        if out.size == 0:
            return out
        xf = np.broadcast_to(xs, out.shape).ravel()
        yf = np.broadcast_to(ys, out.shape).ravel()
        cols = self._inv.a * xf + self._inv.b * yf + self._inv.c
        rows = self._inv.d * xf + self._inv.e * yf + self._inv.f
        finite = np.isfinite(cols) & np.isfinite(rows)
        if not finite.any():
            return out
        # Pixel bbox of the query, padded 1 so interior points keep their 2x2 stencil.
        c0 = max(0, int(math.floor(float(np.min(cols[finite])))) - 1)
        r0 = max(0, int(math.floor(float(np.min(rows[finite])))) - 1)
        c1 = min(self.width, int(math.ceil(float(np.max(cols[finite])))) + 1)
        r1 = min(self.height, int(math.ceil(float(np.max(rows[finite])))) + 1)
        if c1 <= c0 or r1 <= r0:
            return out
        arr, inv = self._window_array(r0, c0, r1, c1)
        return self._bilinear(arr, inv, xs, ys, self.nodata)

    def _window_array(self, r0, c0, r1, c1):
        """(array, inv_transform) for a native window covering pixel bbox [r0:r1, c0:c1],
        reusing the cached window when it already covers the bbox. Reads a padded block
        so nearby follow-up queries (a moving cursor) stay in cache. A single query wider
        than the cap returns a bounded decimated read but is NOT cached (so a later local
        query always reads native)."""
        c = self._win
        if c is not None and c[0] <= r0 and c[1] <= c0 and r1 <= c[2] and c1 <= c[3]:
            return c[4], c[5]
        pad = 256
        rr0 = max(0, r0 - pad); cc0 = max(0, c0 - pad)
        rr1 = min(self.height, r1 + pad); cc1 = min(self.width, c1 + pad)
        h, w = rr1 - rr0, cc1 - cc0
        win = Window(cc0, rr0, w, h)
        wt = self.src.window_transform(win)
        stride = max(1, int(math.ceil(math.sqrt((h * w) / float(_MAX_CELLS)))))
        if stride > 1:
            oh, ow = max(1, h // stride), max(1, w // stride)
            arr = self.src.read(1, window=win, out_shape=(oh, ow),
                                resampling=Resampling.nearest)
            return arr, ~(wt * Affine.scale(w / ow, h / oh))
        arr = self.src.read(1, window=win)
        inv = ~wt
        self._win = (rr0, cc0, rr1, cc1, arr, inv)     # cache native windows only
        return arr, inv

    @classmethod
    def _view(cls, src, array, transform, nodata):
        """A DTM-like view (same sampling interface) over a given array + transform,
        sharing the parent's open dataset. `array` is set, so it samples in-RAM."""
        obj = cls.__new__(cls)
        obj.src = src
        obj.array = array
        obj.transform = transform
        obj.nodata = nodata
        obj.width, obj.height = array.shape[1], array.shape[0]
        obj._inv = ~transform
        obj._win = None
        obj._ov = None
        return obj

    def overview_array(self, max_px=_DISP_CELLS):
        """Bounded decimated whole-extent grid for the map — read once and cached.
        Returns (array, transform, stride); native (stride 1) when the raster fits. This
        single whole-extent read (at load) is the only time the full file is scanned for
        display."""
        if self._ov is not None:
            return self._ov
        stride = max(1, int(math.ceil(math.sqrt(
            (self.width * self.height) / float(max_px)))))
        if stride <= 1:
            arr = self.array if self.array is not None else self.src.read(1)
            self._ov = (arr, self.transform, 1)
            return self._ov
        if self.array is not None:
            # In RAM already — decimate the array (instant, no disk read).
            arr = self.array[::stride, ::stride]
            self._ov = (arr, self.transform * Affine.scale(stride), stride)
            return self._ov
        # Large raster: the one whole-extent disk read. Slow enough on a giga-pixel file
        # that the map runs it on a worker thread (see CanvasMap._OverviewWorker).
        ow = max(1, self.width // stride)
        oh = max(1, self.height // stride)
        arr = self.src.read(1, out_shape=(oh, ow), resampling=Resampling.average)
        t = self.transform * Affine.scale(self.width / ow, self.height / oh)
        self._ov = (arr, t, stride)
        return self._ov

    def read_window(self, bounds, margin_m=0.0):
        """A DTM view holding the NATIVE-resolution array over `bounds`
        ((minx, miny, maxx, maxy) in this raster's CRS), padded by `margin_m`. Lets
        planning/estimation run at full detail on an AOI with RAM bounded by the window,
        not the whole raster. If the window alone still exceeds the cell cap (a huge
        AOI) it too is decimated. Clipped to the raster; returns self if empty."""
        minx, miny, maxx, maxy = bounds
        crs = self.src.crs
        if crs is not None and crs.is_geographic:
            latc = (miny + maxy) / 2.0
            px = margin_m / (_LAT_M * max(math.cos(math.radians(latc)), 1e-6))
            py = margin_m / _LAT_M
        else:
            px = py = margin_m
        c0f, r0f = ~self.src.transform * (minx - px, maxy + py)   # upper-left
        c1f, r1f = ~self.src.transform * (maxx + px, miny - py)   # lower-right
        col_off = max(0, int(math.floor(min(c0f, c1f))))
        row_off = max(0, int(math.floor(min(r0f, r1f))))
        col_end = min(self.src.width, int(math.ceil(max(c0f, c1f))))
        row_end = min(self.src.height, int(math.ceil(max(r0f, r1f))))
        if col_end <= col_off or row_end <= row_off:
            return self
        win = Window(col_off, row_off, col_end - col_off, row_end - row_off)
        wt = self.src.window_transform(win)
        wh, ww = row_end - row_off, col_end - col_off
        stride = max(1, int(math.ceil(math.sqrt((wh * ww) / float(_MAX_CELLS)))))
        if stride > 1:
            oh, ow = max(1, wh // stride), max(1, ww // stride)
            arr = self.src.read(1, window=win, out_shape=(oh, ow),
                                resampling=Resampling.nearest)
            wt = wt * Affine.scale(ww / ow, wh / oh)
        else:
            arr = self.src.read(1, window=win)
        return DTM._view(self.src, arr, wt, self.nodata)
