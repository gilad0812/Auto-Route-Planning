"""
Convert a GeoTIFF DTM raster to a Wavefront OBJ mesh for HELIOS++ scenes.

The output mesh uses the same flat-Earth metric coordinate system as
helios_integration._route_to_metric(): vertices are expressed in metres
relative to a reference (ref_lon, ref_lat) point — typically the route
centroid — so the mesh and the survey legs are aligned without a full
re-projection.
"""

from __future__ import annotations

import io as _io
import math
import os
from pathlib import Path
from typing import Tuple

import numpy as np

try:
    import rasterio
    _RASTERIO_OK = True
except ImportError:
    _RASTERIO_OK = False

_LAT_M = 111_139.0  # metres per degree latitude (WGS-84 approximation)


def dtm_to_obj(
    dtm_path: str,
    output_obj_path: str,
    step_m: float = 2.0,
    ref_lon: float = 0.0,
    ref_lat: float = 0.0,
) -> str:
    """
    Generate a Wavefront OBJ terrain mesh from a GeoTIFF DTM.

    For geographic rasters (lon/lat), each vertex is projected to a flat-Earth
    metric system centred at (ref_lon, ref_lat) — pass the route centroid here
    so the mesh and the HELIOS++ survey legs share the same coordinate origin.

    For projected rasters (already in metres) the vertex coordinates are used
    as-is; ref_lon and ref_lat are ignored.

    Args:
        dtm_path:        Path to the source GeoTIFF.
        output_obj_path: Destination .obj file path.
        step_m:          Approximate vertex spacing in metres.
                         Larger values → coarser but faster mesh.
        ref_lon:         Reference longitude (degrees) for the local projection.
        ref_lat:         Reference latitude  (degrees) for the local projection.

    Returns:
        Absolute path of the written .obj file.
    """
    if not _RASTERIO_OK:
        raise ImportError("rasterio is required: pip install rasterio")

    with rasterio.open(dtm_path) as src:
        is_geo = src.crs.is_geographic if src.crs else True
        data = src.read(1).astype(np.float64)
        nodata = src.nodata
        transform = src.transform
        pixel_w = abs(transform.a)
        pixel_h = abs(transform.e)

    # Replace no-data with NaN so the fill step can handle it
    if nodata is not None:
        data[data == nodata] = np.nan

    # Subsample to the requested mesh resolution
    step_x = max(1, int(round(step_m / (pixel_w * (_LAT_M if is_geo else 1.0)))))
    step_y = max(1, int(round(step_m / (pixel_h * (_LAT_M if is_geo else 1.0)))))
    data = data[::step_y, ::step_x]
    nrows, ncols = data.shape

    # Fill any remaining NaN holes with nearest-neighbour propagation
    data = _fill_nodata(data)

    # Top-left vertex world coordinates (before projection)
    # transform.c = left edge x,  transform.f = top edge y
    origin_x = transform.c
    origin_y = transform.f
    eff_dx = step_x * abs(transform.a)   # effective pixel width after subsampling
    eff_dy = step_y * abs(transform.e)   # effective pixel height

    # ── Compute all vertex coordinates via numpy (no Python loop) ────────────
    col_idx = np.arange(ncols, dtype=np.float64)
    row_idx = np.arange(nrows, dtype=np.float64)

    raw_x = origin_x + col_idx * eff_dx   # (ncols,)
    raw_y = origin_y - row_idx * eff_dy   # (nrows,)  top→bottom

    if is_geo:
        lon_m = _LAT_M * math.cos(math.radians(ref_lat))
        vx = (raw_x - ref_lon) * lon_m          # (ncols,)
        vy = (raw_y - ref_lat) * _LAT_M         # (nrows,)
    else:
        vx = raw_x
        vy = raw_y

    VX = np.tile(vx[np.newaxis, :], (nrows, 1))   # (nrows, ncols)
    VY = np.tile(vy[:, np.newaxis], (1, ncols))    # (nrows, ncols)
    coords = np.column_stack([VX.ravel(), VY.ravel(), data.ravel()])  # (N, 3)

    # ── Build face index arrays (vectorised, 1-based) ─────────────────────────
    r = np.arange(nrows - 1)
    c = np.arange(ncols - 1)
    R, C = np.meshgrid(r, c, indexing="ij")
    R = R.ravel(); C = C.ravel()
    TL = (R * ncols + C + 1).astype(np.int64)
    TR = TL + 1
    BL = TL + ncols
    BR = BL + 1
    all_faces = np.empty((2 * len(TL), 3), dtype=np.int64)
    all_faces[0::2] = np.column_stack([TL, BL, BR])   # triangle 1
    all_faces[1::2] = np.column_stack([TL, BR, TR])   # triangle 2

    output_obj_path = os.path.abspath(output_obj_path)
    Path(output_obj_path).parent.mkdir(parents=True, exist_ok=True)

    with open(output_obj_path, "w", encoding="utf-8") as fh:
        fh.write("# Terrain mesh — auto-generated from DTM\n\n")

        v_buf = _io.StringIO()
        np.savetxt(v_buf, coords, fmt="v %.3f %.3f %.3f")
        fh.write(v_buf.getvalue())
        fh.write("\n")

        f_buf = _io.StringIO()
        np.savetxt(f_buf, all_faces, fmt="f %d %d %d")
        fh.write(f_buf.getvalue())

    return output_obj_path


def _fill_nodata(data: np.ndarray, iterations: int = 20) -> np.ndarray:
    """Fill NaN values by iterative 4-connectivity nearest-neighbour propagation."""
    arr = data.copy()
    for _ in range(iterations):
        nan_mask = np.isnan(arr)
        if not nan_mask.any():
            break
        padded = np.pad(arr, 1, constant_values=np.nan)
        neighbors = np.stack([
            padded[:-2, 1:-1],   # north
            padded[2:,  1:-1],   # south
            padded[1:-1, :-2],   # west
            padded[1:-1, 2:],    # east
        ])
        neighbor_mean = np.nanmean(neighbors, axis=0)
        arr = np.where(nan_mask, neighbor_mean, arr)
    arr[np.isnan(arr)] = 0.0
    return arr
