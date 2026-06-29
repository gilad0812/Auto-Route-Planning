"""Fast analytical point-density estimate for a planned route.

Expected LiDAR density per ground cell from scan geometry alone (no ray tracing),
so a route iterates in ~a second instead of a HELIOS++ run.

Model: a single pass deposits ρ(θ) = pulse_freq·cos²θ / (speed·h·FOV) at a point
seen under scan angle θ (h = AGL there, FOV = 2·half-angle). scan_freq cancels in
the derivation, so it doesn't affect average density. Per-cell density sums ρ over
every pass whose swath covers it — capturing swath-edge thinning (cos²θ), range²
thinning over valleys, coverage gaps, and the FOV cut-off.

Occlusion is modelled by a line-of-sight march; a CHM thins vegetated cells by
`veg_penetration`. Multiple returns are not modelled — an estimator for iterating,
confirmed by one HELIOS++ run.
"""

import math
import numpy as np

try:
    from matplotlib.path import Path as _MplPath
    _MPL_OK = True
except ImportError:
    _MPL_OK = False

_LAT_M = 111139.0  # metres per degree latitude (WGS-84 approximation)


def _group_passes(route):
    passes = {}
    for wp in route:
        passes.setdefault(wp.get("pass_id", 0), []).append(wp)
    return list(passes.values())


def estimate_density_grid(
    route, dtm, region, *,
    pulse_freq_hz, scan_freq_hz, scan_half_angle_deg, speed_ms, min_points,
    is_geo=True, cell_size_m=1.0, max_cells=3_000_000,
    occlusion=True, occ_margin_m=2.0,
    chm=None, veg_penetration=0.4,
):
    """Estimate per-cell point density for `route` over `dtm`.

    Args:
        route:       list of waypoint dicts {x(lon), y(lat), z(alt), pass_id}.
        dtm:         DTM object exposing .array, .src.transform, .nodata.
        region:      AOI as a list of (lon, lat) vertices, or None (whole bbox).
        scan_freq_hz: accepted for signature symmetry; cancels out of the model.
        cell_size_m: grid resolution (auto-coarsened to stay under max_cells).
        chm:         optional binary vegetation mask (same interface as `dtm`);
                     cells with value > 0 are thinned by `veg_penetration`.
        veg_penetration: ground-return fraction through canopy (thumb rule 0.4).

    Returns a dict mirroring the HELIOS result shape so the same map overlay/
    summary can render it:
        {passed, failing_cells_geo, n_fail, n_cells, median_density,
         min_density, cell_size_m, estimate=True}
    """
    fov = 2.0 * math.radians(scan_half_angle_deg)
    tan_half = math.tan(math.radians(scan_half_angle_deg))

    passes = [p for p in _group_passes(route)
              if len(p) >= 2 and not math.isnan(float(p[0]["z"]))]
    if not passes:
        return {"passed": False, "failing_cells_geo": [], "n_fail": 0,
                "n_cells": 0, "median_density": 0.0, "min_density": 0.0,
                "cell_size_m": cell_size_m, "estimate": True,
                "error": "No valid passes in route."}

    # ── AOI bounding box (lon/lat) ───────────────────────────────────────────
    if region:
        rs = np.asarray(region, dtype=float)
        minlon, minlat = rs[:, 0].min(), rs[:, 1].min()
        maxlon, maxlat = rs[:, 0].max(), rs[:, 1].max()
    else:
        xs = [wp["x"] for wp in route]; ys = [wp["y"] for wp in route]
        minlon, maxlon = min(xs), max(xs)
        minlat, maxlat = min(ys), max(ys)

    lat0 = (minlat + maxlat) / 2.0
    lon_m = _LAT_M * math.cos(math.radians(lat0)) if is_geo else 1.0
    lat_m = _LAT_M if is_geo else 1.0

    width_m = max((maxlon - minlon) * lon_m, 1.0)
    height_m = max((maxlat - minlat) * lat_m, 1.0)

    # Coarsen the cell to keep the grid under the work budget.
    cell = float(cell_size_m)
    while (width_m / cell) * (height_m / cell) > max_cells:
        cell *= 1.5
    nx = max(1, int(math.ceil(width_m / cell)))
    ny = max(1, int(math.ceil(height_m / cell)))

    # Cell-centre lon/lat grids.
    lon = minlon + (np.arange(nx) + 0.5) * (cell / lon_m)
    lat = minlat + (np.arange(ny) + 0.5) * (cell / lat_m)
    LON, LAT = np.meshgrid(lon, lat)               # (ny, nx)

    # Metric coords relative to the AOI centre.
    E = (LON - (minlon + maxlon) / 2.0) * lon_m
    N = (LAT - (minlat + maxlat) / 2.0) * lat_m

    # ── Terrain elevation per cell (vectorised raster lookup) ────────────────
    arr = np.asarray(dtm.array, dtype=float)
    t = dtm.src.transform
    col = ((LON - t.c) / t.a)
    row = ((LAT - t.f) / t.e)
    col = np.clip(col.astype(int), 0, arr.shape[1] - 1)
    row = np.clip(row.astype(int), 0, arr.shape[0] - 1)
    terr = arr[row, col]
    if dtm.nodata is not None:
        terr = np.where(terr == dtm.nodata, np.nan, terr)

    # Surface normal per cell from local slope — for the back-facing test (cos_i ≤ 0)
    # and per-surface density. Differentiate the DTM at NATIVE pixel resolution, not
    # on the fine cell grid: a coarse DTM upsampled to 1 m is a staircase that
    # invents cliffs at the pixel risers. (t.a>0 east per col, t.e<0 north per row.)
    arr_f = np.where(arr == dtm.nodata, np.nan, arr) if dtm.nodata is not None else arr
    arr_f = np.where(np.isnan(arr_f), np.nanmean(arr_f), arr_f)
    g_e = (np.gradient(arr_f, axis=1) / (t.a * lon_m))[row, col]   # ∂z/∂east  per m
    g_n = (np.gradient(arr_f, axis=0) / (t.e * lat_m))[row, col]   # ∂z/∂north per m
    nrm = np.sqrt(1.0 + g_e * g_e + g_n * g_n)

    cu = (minlon + maxlon) / 2.0
    cvv = (minlat + maxlat) / 2.0

    def _terr_EN(Em, Nm):
        """Terrain elevation at metric points (E, N) — for the occlusion march."""
        cq = np.clip(((cu + Em / lon_m - t.c) / t.a).astype(int), 0, arr.shape[1] - 1)
        rq = np.clip(((cvv + Nm / lat_m - t.f) / t.e).astype(int), 0, arr.shape[0] - 1)
        return arr[rq, cq]

    # ── Accumulate density from every pass ───────────────────────────────────
    density = np.zeros((ny, nx), dtype=float)
    for pts in passes:
        z_pass = float(pts[0]["z"])
        ax = (pts[0]["x"] - cu) * lon_m
        ay = (pts[0]["y"] - cvv) * lat_m
        bx = (pts[-1]["x"] - cu) * lon_m
        by = (pts[-1]["y"] - cvv) * lat_m
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-9:
            fx, fy = ax, ay
        else:
            tt = np.clip(((E - ax) * dx + (N - ay) * dy) / L2, 0.0, 1.0)
            fx, fy = ax + tt * dx, ay + tt * dy
        ox, oy = E - fx, N - fy                     # horizontal aircraft→cell offset
        d = np.hypot(ox, oy)
        h = z_pass - terr                           # AGL above each cell
        with np.errstate(invalid="ignore"):
            R = np.sqrt(d * d + h * h)              # slant range
            # cos(incidence) of ray vs surface normal; flat ground → h/R.
            cos_i = (h + ox * g_e + oy * g_n) / (np.maximum(R, 1e-6) * nrm)
            covered = (np.isfinite(h) & (h > 1.0)
                       & (d <= h * tan_half) & (cos_i > 0.0))
            # cos_i / R → points per tilted SURFACE m² (survey-quality metric);
            # HELIOS normalises the same way, so estimate and sim are comparable.
            contrib = np.where(
                covered,
                pulse_freq_hz * cos_i / (speed_ms * np.maximum(R, 1.0) * fov),
                0.0,
            )
        # Occlusion: march the sight-line from the pass down to the cell; terrain
        # rising above it blocks the beam (shadowed gully floor / lee face). The
        # dominant effect HELIOS sees that pure scan geometry misses.
        if occlusion:
            with np.errstate(invalid="ignore"):
                blocked = np.zeros_like(h, dtype=bool)
                for tf in (0.25, 0.45, 0.65, 0.82, 0.93):
                    Em = E - (1.0 - tf) * ox        # march point foot→cell
                    Nm = N - (1.0 - tf) * oy
                    los = z_pass - tf * h           # straight sight-line altitude
                    blocked |= covered & (_terr_EN(Em, Nm) > los + occ_margin_m)
            contrib = np.where(blocked, 0.0, contrib)
        density += contrib

    # Canopy: vegetated cells keep only `veg_penetration` of the bare-earth density
    # (the fraction of pulses reaching the ground through the canopy).
    if chm is not None:
        ca = np.asarray(chm.array, dtype=float)
        ct = chm.src.transform
        ccol = np.clip(((LON - ct.c) / ct.a).astype(int), 0, ca.shape[1] - 1)
        crow = np.clip(((LAT - ct.f) / ct.e).astype(int), 0, ca.shape[0] - 1)
        mask = ca[crow, ccol]
        if chm.nodata is not None:
            mask = np.where(mask == chm.nodata, 0.0, mask)
        veg = np.isfinite(mask) & (mask > 0)
        density = np.where(veg, density * float(veg_penetration), density)

    # ── Region mask + failure detection ──────────────────────────────────────
    if region and _MPL_OK:
        inside = _MplPath(np.asarray(region, dtype=float)).contains_points(
            np.column_stack([LON.ravel(), LAT.ravel()])
        ).reshape(LON.shape)
    else:
        inside = np.ones_like(density, dtype=bool)

    fail_mask = inside & (density < float(min_points))
    rows, cols = np.where(fail_mask)
    failing_geo = list(zip(LON[rows, cols].tolist(), LAT[rows, cols].tolist()))

    in_vals = density[inside]
    return {
        "passed": len(failing_geo) == 0,
        "failing_cells_geo": failing_geo,
        "n_fail": len(failing_geo),
        "n_cells": int(inside.sum()),
        # Voids = reached cells with ~zero points (occlusion shadows / gaps).
        "n_void": int((in_vals <= 0.0).sum()),
        "median_density": float(np.median(in_vals)) if in_vals.size else 0.0,
        "min_density": float(in_vals.min()) if in_vals.size else 0.0,
        "cell_size_m": cell,
        "estimate": True,
    }
