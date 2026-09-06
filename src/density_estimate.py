"""Fast analytical point-density estimate for a planned route.

Expected LiDAR density per ground cell from scan geometry alone (no ray tracing),
so a route iterates in ~a second instead of a HELIOS++ run.

Model: a single pass deposits ρ(θ) = pulse_freq·cos²θ / (speed·h·FOV) at a point
seen under scan angle θ (h = AGL there, FOV = 2·half-angle). scan_freq cancels in
the derivation, so it doesn't affect average density. Per-cell density sums ρ over
every pass whose swath covers it — capturing swath-edge thinning (cos²θ), range²
thinning over valleys, coverage gaps, and the FOV cut-off.

VUX-120 "NFB": the beam alternates nadir / +forward / −backward (datasheet: ±10° at
swath centre, up to ±15° at the swath edges), transverse to the cross-track scan, so
each pass is three looks sharing the pulse budget (~1/3 each). Modelling all three
lets steep/occluded faces that the nadir look misses be covered from the fore or aft
angle — the sensor's whole point on "vertical surfaces and narrow canyons". Total
density is conserved (pulses split, not added). Toggle with `nfb`.

Occlusion is modelled by a line-of-sight march (per look); a CHM thins vegetated
cells by `veg_penetration`. Multiple returns are not modelled — an estimator for
iterating, confirmed by one HELIOS++ run.
"""

import math
import numpy as np

from scanner import max_range_m as _scanner_max_range

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
    occlusion=True, occ_margin_m=2.0, nfb=True,
    chm=None, veg_penetration=0.4,
):
    """Estimate per-cell point density for `route` over `dtm`.

    Args:
        route:       list of waypoint dicts {x(lon), y(lat), z(alt), pass_id}.
        dtm:         DTM object exposing .array, .transform, .nodata.
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
    # Scanner range envelope (VUX-120-23 datasheet, >=20 % reflectivity, MTA
    # resolved): pulses are still EMITTED across the full FOV — the `fov` divisor
    # in the density formula is unchanged — but beyond this slant range they
    # return nothing, so those cells get no contribution.
    rng_max = _scanner_max_range(pulse_freq_hz)

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

    # ── Terrain elevation per cell — BILINEAR at the DTM's native resolution ──
    # Use dtm.transform (not dtm.src.transform): `dtm` may be a native AOI window whose
    # array is offset from the whole raster, so the parent transform would mis-index it.
    # Bilinear (not nearest-pixel) removes the staircase in the per-cell AGL; a nodata
    # corner propagates to NaN so void-edge cells are treated as unsurveyed.
    arr = np.asarray(dtm.array, dtype=float)
    t = dtm.transform
    inv = ~t
    cf = inv.a * LON + inv.b * LAT + inv.c            # fractional pixel column
    rf = inv.d * LON + inv.e * LAT + inv.f            # fractional pixel row
    h_, w_ = arr.shape
    col = np.clip(cf.astype(int), 0, w_ - 1)          # nearest index — for the slope lookup
    row = np.clip(rf.astype(int), 0, h_ - 1)
    am = np.where(arr == dtm.nodata, np.nan, arr) if dtm.nodata is not None else arr
    j0 = np.clip(np.floor(cf).astype(int), 0, w_ - 2)
    i0 = np.clip(np.floor(rf).astype(int), 0, h_ - 2)
    dx = np.clip(cf - j0, 0.0, 1.0); dy = np.clip(rf - i0, 0.0, 1.0)
    terr = ((am[i0, j0] * (1 - dx) + am[i0, j0 + 1] * dx) * (1 - dy)
            + (am[i0 + 1, j0] * (1 - dx) + am[i0 + 1, j0 + 1] * dx) * dy)

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
    # NFB looks: (along-track sign, pulse-budget fraction). The beam alternates
    # nadir/forward/backward (datasheet), so each carries ~1/3 of the pulses — a
    # cell seen by all three gets 3·(1/3) = the same total (density conserved).
    looks = ((0.0, 1.0 / 3.0), (1.0, 1.0 / 3.0), (-1.0, 1.0 / 3.0)) if nfb \
        else ((0.0, 1.0),)
    density = np.zeros((ny, nx), dtype=float)
    range_hit = np.zeros((ny, nx), dtype=bool)   # in-FOV but beyond max range
    any_fov = np.zeros((ny, nx), dtype=bool)     # in some look's FOV (any range)
    any_covered = np.zeros((ny, nx), dtype=bool)  # in-FOV AND in-range (pre-occlusion)
    # Lowest terrain on the grid → the largest possible AGL, hence the widest possible
    # cross-track swath any pass can reach. Used to bound each pass to a sub-window.
    gmin_terr = float(np.nanmin(terr)) if np.isfinite(terr).any() else 0.0
    for pts in passes:
        z_pass = float(pts[0]["z"])
        # ── Restrict this pass to its swath's bounding box ───────────────────
        # A cell is only ever reached if d ≤ h·tan_half (cross-track FOV). With h
        # bounded by z_pass − gmin_terr, no cell farther than `reach` from the pass
        # segment can pass that test, so cells outside this box contribute exactly 0.
        # Working on the sub-window is identical maths on far fewer cells.
        max_h = z_pass - gmin_terr
        if max_h <= 1.0:
            continue                                # aircraft below/at the terrain — no swath
        reach = max_h * tan_half
        x0, x1 = pts[0]["x"], pts[-1]["x"]
        y0, y1 = pts[0]["y"], pts[-1]["y"]
        c0 = max(0, int(np.searchsorted(lon, min(x0, x1) - reach / lon_m, "left")))
        c1 = min(nx, int(np.searchsorted(lon, max(x0, x1) + reach / lon_m, "right")))
        r0 = max(0, int(np.searchsorted(lat, min(y0, y1) - reach / lat_m, "left")))
        r1 = min(ny, int(np.searchsorted(lat, max(y0, y1) + reach / lat_m, "right")))
        if c0 >= c1 or r0 >= r1:
            continue
        rs, cs = slice(r0, r1), slice(c0, c1)
        Ew, Nw = E[rs, cs], N[rs, cs]
        g_ew, g_nw, nrmw = g_e[rs, cs], g_n[rs, cs], nrm[rs, cs]

        ax = (x0 - cu) * lon_m
        ay = (y0 - cvv) * lat_m
        bx = (x1 - cu) * lon_m
        by = (y1 - cvv) * lat_m
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-9:
            fx, fy = ax, ay
            ux, uy = 1.0, 0.0
        else:
            tt = np.clip(((Ew - ax) * dx + (Nw - ay) * dy) / L2, 0.0, 1.0)
            fx, fy = ax + tt * dx, ay + tt * dy
            _pl = math.sqrt(L2); ux, uy = dx / _pl, dy / _pl   # along-track unit vector
        ox, oy = Ew - fx, Nw - fy                   # cross-track aircraft→cell offset
        d = np.hypot(ox, oy)
        h = z_pass - terr[rs, cs]                   # AGL above each cell
        with np.errstate(invalid="ignore"):
            in_swath = np.isfinite(h) & (h > 1.0) & (d <= h * tan_half)   # cross-track FOV
        # Only the cells actually in the swath do any per-look / occlusion work — the
        # rest of the window fails the FOV test and would contribute exactly 0. Gather
        # them to 1-D so the expensive inner loop runs on a fraction of the cells (this
        # is where the reach-bloated window on steep terrain gets its cells back).
        wr, wc = np.where(in_swath)
        if wr.size == 0:
            continue
        gr, gc = wr + r0, wc + c0                    # indices back into the full grid
        ox_s, oy_s, d_s, h_s = ox[wr, wc], oy[wr, wc], d[wr, wc], h[wr, wc]
        Es, Ns = Ew[wr, wc], Nw[wr, wc]
        g_es, g_ns, nrm_s = g_ew[wr, wc], g_nw[wr, wc], nrmw[wr, wc]
        with np.errstate(invalid="ignore"):
            # Fore/aft along-track tilt grows 10° (swath centre) → 15° (edge); the
            # along-track ground reach of that tilt at this AGL is a_f.
            _frac = np.clip(d_s / np.maximum(h_s * tan_half, 1e-6), 0.0, 1.0)
            a_f = h_s * np.tan(np.radians(10.0 + 5.0 * _frac))
            for sgn, wgt in looks:
                px = ox_s + sgn * a_f * ux           # this look's horizontal offset to cell
                py = oy_s + sgn * a_f * uy
                R = np.sqrt(px * px + py * py + h_s * h_s)   # slant range for this look
                # cos(incidence) of THIS look's ray vs surface normal; flat ground → h/R.
                cos_i = (h_s + px * g_es + py * g_ns) / (np.maximum(R, 1e-6) * nrm_s)
                facing = cos_i > 0.0                 # in_swath already holds for every cell
                beyond = facing & (R > rng_max)      # emitted, but no return
                covered = facing & (R <= rng_max)
                # cos_i / R → points per tilted SURFACE m² (survey-quality metric);
                # HELIOS normalises the same way, so estimate and sim are comparable.
                contrib = np.where(
                    covered,
                    wgt * pulse_freq_hz * cos_i / (speed_ms * np.maximum(R, 1.0) * fov),
                    0.0,
                )
                # Occlusion: march THIS look's sight-line from the sensor down to the
                # cell; terrain rising above it blocks the beam. Fore/aft looks see under
                # ridges the nadir look can't — the recovery NFB is meant to give.
                if occlusion:
                    blocked = np.zeros(wr.size, dtype=bool)
                    for tf in (0.25, 0.45, 0.65, 0.82, 0.93):
                        Em = Es - (1.0 - tf) * px   # march point sensor→cell
                        Nm = Ns - (1.0 - tf) * py
                        los = z_pass - tf * h_s     # straight sight-line altitude
                        blocked |= covered & (_terr_EN(Em, Nm) > los + occ_margin_m)
                    contrib = np.where(blocked, 0.0, contrib)
                # scatter the per-cell results back (indices are unique within a look).
                if beyond.any():
                    range_hit[gr[beyond], gc[beyond]] = True
                if facing.any():
                    any_fov[gr[facing], gc[facing]] = True   # any_fov == in_fov here
                if covered.any():
                    any_covered[gr[covered], gc[covered]] = True
                density[gr, gc] += contrib

    # Canopy: vegetated cells keep only `veg_penetration` of the bare-earth density
    # (the fraction of pulses reaching the ground through the canopy).
    if chm is not None:
        ca = np.asarray(chm.array, dtype=float)
        ct = chm.transform                           # matches chm.array (native AOI window)
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

    # Categorise each failing cell by CAUSE, so the map colours them and the
    # operator sees the lever, not just "orange":
    #   thin   — reached but under target (AGL over low ground, swath-edge cos²).
    #   shadow — a pass had it in-range & in-FOV, but occlusion blocked the beam.
    #   range  — only ever seen beyond the scanner's max range (AGL/PRR mismatch).
    #   gap    — never fell in any pass's swath (spacing / AOI-edge coverage gap).
    _thin = fail_mask & (density > 0.0)
    _void = fail_mask & ~_thin
    _shadow = _void & any_covered
    _range = _void & ~any_covered & range_hit
    _gap = _void & ~any_covered & ~range_hit

    def _geo(m):
        r, c = np.where(m)
        return list(zip(LON[r, c].tolist(), LAT[r, c].tolist()))

    # One coverage/density gradient for the cells whose problem IS low coverage — thin
    # (reached but sparse) AND gap (never in a swath): density / target in [0, 1], so
    # 0 (empty / uncovered) draws red and target draws yellow. `range` and `shadow` stay
    # their own colours — their fix isn't more coverage (scanner range / occlusion).
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
        # Voids = reached cells with ~zero points (occlusion shadows / gaps).
        "n_void": int((in_vals <= 0.0).sum()),
        # Failing cells that sat beyond the scanner's max range from >=1 pass —
        # the tell that the AGL/PRR combo, not the geometry, is what's starving them.
        "n_range_limited": int((range_hit & fail_mask).sum()),
        "median_density": float(np.median(in_vals)) if in_vals.size else 0.0,
        "mean_density": float(in_vals.mean()) if in_vals.size else 0.0,
        "min_density": float(in_vals.min()) if in_vals.size else 0.0,
        "in_region_density": in_vals,   # per-cell densities inside the AOI (for band stats)
        "cell_size_m": cell,
        "estimate": True,
    }
