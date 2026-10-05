"""Density-estimator geometry: line ends and surfaces facing away from the scanner.

1. Line ends — a look only sees a cell while the aircraft is ON the line. Past the END
   only the fore look reaches (≈ a_f = AGL·tan 10–15°); before the START only the aft
   look; beyond that, nothing. (The old clamp-to-segment credited a half-disc "cap" of
   radius AGL·tan 50° past every end — cover no scan line actually provides.)
2. Facing away — a void cell whose surface faces away from every line (cos_i ≤ 0) is
   categorised as 'shadow' (fix: a cross-line), not as an under-target gap (whose advice,
   lower AGL / tighter spacing, can't help).

Each case runs against the NumPy reference AND the numba fast path (when installed), and
checks the two agree. Runs standalone (`python tests/test_estimator_geometry.py`) and
under pytest.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

import numpy as np                                          # noqa: E402
import rasterio                                             # noqa: E402
from rasterio.transform import from_origin                  # noqa: E402
from density_estimate import estimate_density_grid          # noqa: E402
from dtm import DTM                                         # noqa: E402

try:
    from density_estimate_nb import estimate_density_grid_nb
    _ESTIMATORS = [('numpy', estimate_density_grid), ('numba', estimate_density_grid_nb)]
except Exception:                                           # numba not installed
    _ESTIMATORS = [('numpy', estimate_density_grid)]

_N = 500
_KW = dict(pulse_freq_hz=600_000, scan_freq_hz=224.4, scan_half_angle_deg=50.0,
           speed_ms=6.0, is_geo=False, cell_size_m=1.0)
_ALL_FAIL = 1e9          # min_points so high every cell "fails" → per-cell density readable


def _dtm(tmp, z):
    path = os.path.join(tmp, 'd.tif')
    with rasterio.open(path, 'w', driver='GTiff', height=_N, width=_N, count=1,
                       dtype='float32', crs='EPSG:32633',
                       transform=from_origin(0, _N, 1, 1), nodata=-9999.0) as d:
        d.write(z.astype('float32'), 1)
    return DTM(path)


def _box(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]


def _densities(res):
    """{(x, y): density} for every reached-or-gap cell, read back from the gradient list
    (frac = density / min_points, so density = frac · min_points)."""
    return {(x, y): f * _ALL_FAIL for x, y, f in res['failing_cells_by_reason']['thin']}


# ── 1. line ends ──────────────────────────────────────────────────────────────────────
# Flat ground at 300 m; one N-S line x=250, y 150→350, flown at 400 m (100 m AGL), so
# a_f ≈ 17.6 m near nadir. Cell centres sit on .5.
_ROUTE = [{'x': 250.0, 'y': 150.0, 'z': 400.0, 'pass_id': 0},
          {'x': 250.0, 'y': 350.0, 'z': 400.0, 'pass_id': 0}]


def _end_run(est, **over):
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm = _dtm(tmp, np.full((_N, _N), 300.0))
        res = est(_ROUTE, dtm, _box(150, 50, 350, 450),
                  **dict(_KW, min_points=_ALL_FAIL, **over))
    return _densities(res)


def test_line_ends_have_no_cap():
    for name, est in _ESTIMATORS:
        d = _end_run(est)
        mid = d[(250.5, 250.5)]
        assert mid > 0, name
        # 10 m past the END: only the fore look reaches → about a third of mid-line.
        past_end = d[(250.5, 360.5)]
        assert 0.2 * mid < past_end < 0.5 * mid, (name, past_end / mid)
        # 10 m before the START: only the aft look reaches.
        before = d[(250.5, 140.5)]
        assert 0.2 * mid < before < 0.5 * mid, (name, before / mid)
        # Inside, within a_f of the end: nadir + fore only (aft would need the aircraft
        # past the end) → about two thirds.
        near_end = d[(250.5, 345.5)]
        assert 0.5 * mid < near_end < 0.85 * mid, (name, near_end / mid)
        # 40 m past the end is beyond every look's reach → no cover at all. (The old
        # clamp gave this cell full nadir cover from a ~119 m-radius cap.)
        assert d[(250.5, 390.5)] == 0.0, name
        assert d[(250.5, 109.5)] == 0.0, name


def test_line_ends_nadir_only_stops_at_the_end():
    for name, est in _ESTIMATORS:
        d = _end_run(est, nfb=False)
        assert d[(250.5, 250.5)] > 0, name
        assert d[(250.5, 350.5)] == 0.0 and d[(250.5, 149.5)] == 0.0, name


# ── 2. surfaces facing away ───────────────────────────────────────────────────────────
# Flat 300 m west of x=200, then a steep (≈72°) drop to 120 m by x=260. A line at x=170,
# 60 m AGL, looks over the brink: every cell on the drop faces away from it (cos_i < 0).
def _cliff():
    xx = np.tile(np.arange(_N, dtype=float), (_N, 1))
    return np.where(xx < 200, 300.0, np.where(xx > 260, 120.0, 300.0 - 3.0 * (xx - 200)))


def test_faces_away_is_shadow_not_gap():
    route = [{'x': 170.0, 'y': 100.0, 'z': 360.0, 'pass_id': 0},
             {'x': 170.0, 'y': 400.0, 'z': 360.0, 'pass_id': 0}]
    results = {}
    for name, est in _ESTIMATORS:
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            dtm = _dtm(tmp, _cliff())
            res = est(route, dtm, _box(180, 150, 280, 350), **dict(_KW, min_points=50))
        shadow = set(res['failing_cells_by_reason']['shadow'])
        under = {(x, y) for x, y, _ in res['failing_cells_by_reason']['thin']}
        slope = [(x + 0.5, y + 0.5) for x in range(203, 257, 5) for y in range(160, 340, 15)]
        for c in slope:
            assert c in shadow, f'{name}: drop cell {c} not labelled shadow'
            assert c not in under, f'{name}: drop cell {c} labelled under-target/gap'
        assert res['n_backface'] > 0, name
        results[name] = res
    if 'numba' in results:                     # the two paths must agree exactly
        a, b = results['numpy'], results['numba']
        for k in ('n_shadow', 'n_backface', 'n_gap', 'n_thin', 'n_fail'):
            assert a[k] == b[k], (k, a[k], b[k])


if __name__ == '__main__':
    print('estimators:', ', '.join(n for n, _ in _ESTIMATORS))
    ok = True
    for fn in (test_line_ends_have_no_cap, test_line_ends_nadir_only_stops_at_the_end,
               test_faces_away_is_shadow_not_gap):
        try:
            fn(); print(f'PASS  {fn.__name__}')
        except AssertionError as e:
            ok = False; print(f'FAIL  {fn.__name__}: {e}')
    sys.exit(0 if ok else 1)
