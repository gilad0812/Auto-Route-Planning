"""Numba-accelerated point-density estimate — a drop-in fast path for estimate_density_grid.

Same analytical model as density_estimate.estimate_density_grid, but the whole hot path
is one fused, multithreaded @njit kernel: per output cell it computes the terrain
(bilinear), the slope/normal (native-pixel gradient) and the scan geometry over every
pass — NFB looks + occlusion march — then sums. Cell-major, so it parallelises with
prange and never scatters into shared memory. Only the grid setup (1-D lon/lat, the
nodata-masked DTM views) and the finalisation (canopy thinning + failure
categorisation) stay in NumPy.

Correctness is validated bit-for-bit against the NumPy estimator
(tests/test_estimator_numba.py); this module is only imported when numba is available,
and the caller falls back to the NumPy path otherwise.
"""
import math

import numpy as np
from numba import njit, prange

from scanner import max_range_m as _scanner_max_range

try:
    from matplotlib.path import Path as _MplPath
    _MPL_OK = True
except ImportError:
    _MPL_OK = False

_LAT_M = 111139.0


def _group_passes(route):
    passes = {}
    for wp in route:
        passes.setdefault(wp.get("pass_id", 0), []).append(wp)
    return list(passes.values())


@njit(parallel=True, cache=True, fastmath=False)
def _fused_kernel(lon, lat, am, arr_f, arr,
                  ia, ib, ic, id_, ie, if_, lon_m, lat_m, cu, cvv, gdiv_e, gdiv_n,
                  t_a, t_c, t_e, t_f,
                  pax, pay, pbx, pby, pz,
                  tan_half, fov, rng_max, prr, speed, occ_margin,
                  look_sgn, look_wgt, occlusion):
    """Per-cell density + coverage flags. Fuses the scaffold (terrain bilinear, native
    gradient, normal) with the scan over all passes — one parallel pass over the grid,
    no intermediate full-grid arrays. Mirrors the NumPy estimator's arithmetic exactly."""
    ny = lat.shape[0]
    nx = lon.shape[0]
    a_rows, a_cols = arr.shape
    density = np.zeros((ny, nx))
    range_hit = np.zeros((ny, nx), dtype=np.bool_)
    any_fov = np.zeros((ny, nx), dtype=np.bool_)
    any_cov = np.zeros((ny, nx), dtype=np.bool_)
    npass = pax.shape[0]
    nlook = look_sgn.shape[0]
    for idx in prange(ny * nx):
        i = idx // nx
        j = idx % nx
        LON = lon[j]; LAT = lat[i]
        Ec = (LON - cu) * lon_m
        Nc = (LAT - cvv) * lat_m

        # ── scaffold: bilinear terrain (am) + native-pixel gradient (arr_f) ──
        cf = ia * LON + ib * LAT + ic
        rf = id_ * LON + ie * LAT + if_
        col = int(cf)
        col = 0 if col < 0 else (a_cols - 1 if col > a_cols - 1 else col)
        row = int(rf)
        row = 0 if row < 0 else (a_rows - 1 if row > a_rows - 1 else row)
        j0 = int(math.floor(cf))
        j0 = 0 if j0 < 0 else (a_cols - 2 if j0 > a_cols - 2 else j0)
        i0 = int(math.floor(rf))
        i0 = 0 if i0 < 0 else (a_rows - 2 if i0 > a_rows - 2 else i0)
        dx = cf - j0
        dx = 0.0 if dx < 0.0 else (1.0 if dx > 1.0 else dx)
        dy = rf - i0
        dy = 0.0 if dy < 0.0 else (1.0 if dy > 1.0 else dy)
        tz = ((am[i0, j0] * (1 - dx) + am[i0, j0 + 1] * dx) * (1 - dy)
              + (am[i0 + 1, j0] * (1 - dx) + am[i0 + 1, j0 + 1] * dx) * dy)
        if not np.isfinite(tz):
            continue                                       # void / off-DTM cell

        if col == 0:
            gx = arr_f[row, 1] - arr_f[row, 0]
        elif col == a_cols - 1:
            gx = arr_f[row, a_cols - 1] - arr_f[row, a_cols - 2]
        else:
            gx = (arr_f[row, col + 1] - arr_f[row, col - 1]) * 0.5
        if row == 0:
            gy = arr_f[1, col] - arr_f[0, col]
        elif row == a_rows - 1:
            gy = arr_f[a_rows - 1, col] - arr_f[a_rows - 2, col]
        else:
            gy = (arr_f[row + 1, col] - arr_f[row - 1, col]) * 0.5
        ge = gx / gdiv_e
        gn = gy / gdiv_n
        nm = math.sqrt(1.0 + ge * ge + gn * gn)

        # ── scan every pass ──
        acc = 0.0
        rh = False; af = False; ac = False
        for k in range(npass):
            z = pz[k]
            h = z - tz
            if h <= 1.0:
                continue
            ax = pax[k]; ay = pay[k]; bx = pbx[k]; by = pby[k]
            dxp = bx - ax; dyp = by - ay
            L2 = dxp * dxp + dyp * dyp
            if L2 < 1e-9:
                fx = ax; fy = ay; ux = 1.0; uy = 0.0
            else:
                tt = ((Ec - ax) * dxp + (Nc - ay) * dyp) / L2
                if tt < 0.0:
                    tt = 0.0
                elif tt > 1.0:
                    tt = 1.0
                fx = ax + tt * dxp; fy = ay + tt * dyp
                pl = math.sqrt(L2); ux = dxp / pl; uy = dyp / pl
            ox = Ec - fx; oy = Nc - fy
            d = math.sqrt(ox * ox + oy * oy)
            if d > h * tan_half:
                continue
            frac = d / (h * tan_half) if h * tan_half > 1e-6 else 0.0
            if frac > 1.0:
                frac = 1.0
            a_f = h * math.tan(math.radians(10.0 + 5.0 * frac))
            for L in range(nlook):
                sgn = look_sgn[L]; wgt = look_wgt[L]
                px = ox + sgn * a_f * ux
                py = oy + sgn * a_f * uy
                R = math.sqrt(px * px + py * py + h * h)
                cos_i = (h + px * ge + py * gn) / (max(R, 1e-6) * nm)
                if cos_i <= 0.0:
                    continue
                af = True
                if R > rng_max:
                    rh = True
                    continue
                ac = True
                blocked = False
                if occlusion:
                    for tf in (0.25, 0.45, 0.65, 0.82, 0.93):
                        Em = Ec - (1.0 - tf) * px
                        Nm = Nc - (1.0 - tf) * py
                        cq = int((cu + Em / lon_m - t_c) / t_a)
                        rq = int((cvv + Nm / lat_m - t_f) / t_e)
                        cq = 0 if cq < 0 else (a_cols - 1 if cq > a_cols - 1 else cq)
                        rq = 0 if rq < 0 else (a_rows - 1 if rq > a_rows - 1 else rq)
                        if arr[rq, cq] > z - tf * h + occ_margin:
                            blocked = True
                            break
                if not blocked:
                    acc += wgt * prr * cos_i / (speed * max(R, 1.0) * fov)
        density[i, j] = acc
        range_hit[i, j] = rh
        any_fov[i, j] = af
        any_cov[i, j] = ac
    return density, range_hit, any_fov, any_cov


def estimate_density_grid_nb(
    route, dtm, region, *,
    pulse_freq_hz, scan_freq_hz, scan_half_angle_deg, speed_ms, min_points,
    is_geo=True, cell_size_m=1.0, max_cells=3_000_000,
    occlusion=True, occ_margin_m=2.0, nfb=True,
    chm=None, veg_penetration=0.4,
):
    """Numba fast path with the SAME signature and result dict as
    density_estimate.estimate_density_grid."""
    passes = [p for p in _group_passes(route)
              if len(p) >= 2 and not math.isnan(float(p[0]["z"]))]
    if not passes:
        return {"passed": False, "failing_cells_geo": [], "n_fail": 0,
                "n_cells": 0, "median_density": 0.0, "min_density": 0.0,
                "cell_size_m": cell_size_m, "estimate": True,
                "error": "No valid passes in route."}

    fov = 2.0 * math.radians(scan_half_angle_deg)
    tan_half = math.tan(math.radians(scan_half_angle_deg))
    rng_max = _scanner_max_range(pulse_freq_hz)

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

    cell = float(cell_size_m)
    while (width_m / cell) * (height_m / cell) > max_cells:
        cell *= 1.5
    nx = max(1, int(math.ceil(width_m / cell)))
    ny = max(1, int(math.ceil(height_m / cell)))

    lon = np.ascontiguousarray(minlon + (np.arange(nx) + 0.5) * (cell / lon_m))
    lat = np.ascontiguousarray(minlat + (np.arange(ny) + 0.5) * (cell / lat_m))
    cu = (minlon + maxlon) / 2.0
    cvv = (minlat + maxlat) / 2.0

    arr = np.ascontiguousarray(np.asarray(dtm.array, dtype=float))
    t = dtm.transform
    inv = ~t
    am = (np.where(arr == dtm.nodata, np.nan, arr) if dtm.nodata is not None else arr)
    am = np.ascontiguousarray(am)
    arr_f = np.where(arr == dtm.nodata, np.nan, arr) if dtm.nodata is not None else arr
    arr_f = np.ascontiguousarray(np.where(np.isnan(arr_f), np.nanmean(arr_f), arr_f))

    pax = np.empty(len(passes)); pay = np.empty(len(passes))
    pbx = np.empty(len(passes)); pby = np.empty(len(passes)); pz = np.empty(len(passes))
    for k, pts in enumerate(passes):
        pax[k] = (pts[0]["x"] - cu) * lon_m
        pay[k] = (pts[0]["y"] - cvv) * lat_m
        pbx[k] = (pts[-1]["x"] - cu) * lon_m
        pby[k] = (pts[-1]["y"] - cvv) * lat_m
        pz[k] = float(pts[0]["z"])

    if nfb:
        look_sgn = np.array([0.0, 1.0, -1.0]); look_wgt = np.array([1.0 / 3.0] * 3)
    else:
        look_sgn = np.array([0.0]); look_wgt = np.array([1.0])

    density, range_hit, any_fov, any_covered = _fused_kernel(
        lon, lat, am, arr_f, arr,
        inv.a, inv.b, inv.c, inv.d, inv.e, inv.f, lon_m, lat_m, cu, cvv,
        t.a * lon_m, t.e * lat_m, t.a, t.c, t.e, t.f,
        pax, pay, pbx, pby, pz,
        tan_half, fov, rng_max, float(pulse_freq_hz), float(speed_ms), float(occ_margin_m),
        look_sgn, look_wgt, bool(occlusion))

    # ── finalise (NumPy): canopy thinning + failure categorisation ──────────
    LON, LAT = np.meshgrid(lon, lat)
    if chm is not None:
        ca = np.asarray(chm.array, dtype=float)
        ct = chm.transform
        ccol = np.clip(((LON - ct.c) / ct.a).astype(int), 0, ca.shape[1] - 1)
        crow = np.clip(((LAT - ct.f) / ct.e).astype(int), 0, ca.shape[0] - 1)
        mask = ca[crow, ccol]
        if chm.nodata is not None:
            mask = np.where(mask == chm.nodata, 0.0, mask)
        veg = np.isfinite(mask) & (mask > 0)
        density = np.where(veg, density * float(veg_penetration), density)

    if region and _MPL_OK:
        inside = _MplPath(np.asarray(region, dtype=float)).contains_points(
            np.column_stack([LON.ravel(), LAT.ravel()])
        ).reshape(LON.shape)
    else:
        inside = np.ones_like(density, dtype=bool)

    fail_mask = inside & (density < float(min_points))
    rows, cols = np.where(fail_mask)
    failing_geo = list(zip(LON[rows, cols].tolist(), LAT[rows, cols].tolist()))

    _thin = fail_mask & (density > 0.0)
    _void = fail_mask & ~_thin
    _shadow = _void & any_covered
    _range = _void & ~any_covered & range_hit
    _gap = _void & ~any_covered & ~range_hit

    def _geo(m):
        r, c = np.where(m)
        return list(zip(LON[r, c].tolist(), LAT[r, c].tolist()))

    _under = _thin | _gap
    _ur, _uc = np.where(_under)
    _ufrac = np.clip(density[_ur, _uc] / max(float(min_points), 1e-9), 0.0, 1.0)
    by_reason = {"range": _geo(_range), "shadow": _geo(_shadow),
                 "thin": list(zip(LON[_ur, _uc].tolist(), LAT[_ur, _uc].tolist(),
                                  _ufrac.tolist()))}

    in_vals = density[inside]
    return {
        "failing_cells_by_reason": by_reason,
        "n_thin": int(_thin.sum()), "n_shadow": int(_shadow.sum()),
        "n_beyond_range": int(_range.sum()), "n_gap": int(_gap.sum()),
        "passed": len(failing_geo) == 0,
        "failing_cells_geo": failing_geo,
        "n_fail": len(failing_geo),
        "n_cells": int(inside.sum()),
        "n_void": int((in_vals <= 0.0).sum()),
        "n_range_limited": int((range_hit & fail_mask).sum()),
        "median_density": float(np.median(in_vals)) if in_vals.size else 0.0,
        "mean_density": float(in_vals.mean()) if in_vals.size else 0.0,
        "min_density": float(in_vals.min()) if in_vals.size else 0.0,
        "in_region_density": in_vals,   # per-cell densities inside the AOI (for band stats)
        "cell_size_m": cell,
        "estimate": True,
    }
