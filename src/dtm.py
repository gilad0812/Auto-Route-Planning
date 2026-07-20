import numpy as np
import rasterio

class DTM:
    def __init__(self, path):
        self.src = rasterio.open(path)
        self.array = self.src.read(1)
        self.transform = self.src.transform
        self.nodata = self.src.nodata

    def elevation_at(self, x, y):
        """Return bilinearly-interpolated elevation at geographic coords (x,y).
        x,y should be in the same CRS as the raster (usually lon/lat or projected).
        """
        # Convert world coords to raster row/col
        col, row = ~self.transform * (x, y)
        # indices
        i = int(np.floor(row))
        j = int(np.floor(col))
        # bounds check
        if i < 0 or j < 0 or i + 1 >= self.array.shape[0] or j + 1 >= self.array.shape[1]:
            return float('nan')
        # fractional
        dy = row - i
        dx = col - j
        z00 = self.array[i, j]
        z10 = self.array[i, j+1]
        z01 = self.array[i+1, j]
        z11 = self.array[i+1, j+1]
        # handle nodata
        vals = np.array([z00, z10, z01, z11], dtype=float)
        if np.all(vals == self.nodata):
            return float('nan')
        # bilinear interpolation
        z0 = z00*(1-dx) + z10*dx
        z1 = z01*(1-dx) + z11*dx
        z = z0*(1-dy) + z1*dy
        return float(z)

    def elevation_at_many(self, xs, ys):
        """Vectorised bilinear elevation for arrays of world coords (same CRS as the
        raster). Mirrors elevation_at exactly: NaN where the 2x2 stencil falls off the
        raster, and NaN where all four surrounding cells are nodata (otherwise the raw
        bilinear value, matching the scalar path). Returns a float ndarray shaped like
        the broadcast of xs, ys.

        Route planning samples thousands of terrain points per plan; batching them
        through one array lookup instead of a Python call per point is the hot-path
        speedup (same numbers, far fewer interpreter round-trips).
        """
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        out = np.full(np.broadcast(xs, ys).shape, np.nan, dtype=float)
        if out.size == 0:
            return out
        xf = np.broadcast_to(xs, out.shape).ravel()
        yf = np.broadcast_to(ys, out.shape).ravel()
        inv = ~self.transform                      # affine world->pixel (handles rotation)
        col = inv.a * xf + inv.b * yf + inv.c
        row = inv.d * xf + inv.e * yf + inv.f
        j = np.floor(col).astype(np.int64)
        i = np.floor(row).astype(np.int64)
        h, w = self.array.shape
        ok = (i >= 0) & (j >= 0) & (i + 1 < h) & (j + 1 < w)
        flat = out.ravel()
        if np.any(ok):
            ii, jj = i[ok], j[ok]
            dx = col[ok] - jj
            dy = row[ok] - ii
            arr = self.array
            z00 = arr[ii, jj].astype(float)
            z10 = arr[ii, jj + 1].astype(float)
            z01 = arr[ii + 1, jj].astype(float)
            z11 = arr[ii + 1, jj + 1].astype(float)
            z = (z00 * (1 - dx) + z10 * dx) * (1 - dy) + \
                (z01 * (1 - dx) + z11 * dx) * dy
            if self.nodata is not None:
                allnd = ((z00 == self.nodata) & (z10 == self.nodata)
                         & (z01 == self.nodata) & (z11 == self.nodata))
                z = np.where(allnd, np.nan, z)
            flat[ok] = z
        return flat.reshape(out.shape)
