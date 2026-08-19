"""The numba fast path (density_estimate_nb) must be bit-for-bit identical to the NumPy
reference estimator (density_estimate.estimate_density_grid).

The numba build is only a speed optimisation, so the contract is EXACT agreement — same
density, same failure categorisation — on the same route. Skips cleanly when numba isn't
installed. Runs standalone (`python tests/test_estimator_numba.py`) and under pytest.
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
    _HAVE_NB = True
except Exception:                                           # numba not installed
    _HAVE_NB = False

_KW = dict(pulse_freq_hz=600_000, scan_freq_hz=224.4, scan_half_angle_deg=50.0,
           speed_ms=6.0, min_points=50, is_geo=False, cell_size_m=1.0)


def _setup(tmp, terr_fn, n=700):
    yy, xx = np.mgrid[0:n, 0:n]
    z = terr_fn(xx, yy).astype('float32')
    path = os.path.join(tmp, 'd.tif')
    with rasterio.open(path, 'w', driver='GTiff', height=n, width=n, count=1, dtype='float32',
                       crs='EPSG:32633', transform=from_origin(500000, 3900000 + n, 1, 1),
                       nodata=-9999.0) as d:
        d.write(z, 1)
    dtm = DTM(path)
    x0, y0, x1, y1 = 500120, 3900120, 500120 + (n - 240), 3900120 + (n - 240)
    region = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    zmax = float(z.max())
    route = []
    for i, x in enumerate(range(x0, x1, 45)):
        route += [{'x': float(x), 'y': float(y0), 'z': zmax + 60.0, 'pass_id': i},
                  {'x': float(x), 'y': float(y1), 'z': zmax + 60.0, 'pass_id': i}]
    return dtm, region, route


def _assert_identical(a, b):
    for k in ('n_fail', 'n_cells', 'n_thin', 'n_shadow', 'n_beyond_range',
              'n_gap', 'n_void', 'n_range_limited', 'passed', 'cell_size_m'):
        assert a[k] == b[k], f'{k}: numpy={a[k]} numba={b[k]}'
    assert a['median_density'] == b['median_density'], \
        f"median: {a['median_density']} vs {b['median_density']}"
    assert a['min_density'] == b['min_density'], \
        f"min: {a['min_density']} vs {b['min_density']}"
    # exact agreement on WHICH cells fail and the per-cell density gradient, not just counts
    assert (set(map(tuple, a['failing_cells_geo']))
            == set(map(tuple, b['failing_cells_geo']))), 'failing-cell set differs'
    assert (sorted(a['failing_cells_by_reason']['thin'])
            == sorted(b['failing_cells_by_reason']['thin'])), 'thin gradient list differs'


def _run(terr_fn, **over):
    kw = dict(_KW, **over)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm, region, route = _setup(tmp, terr_fn)
        _assert_identical(estimate_density_grid(route, dtm, region, **kw),
                          estimate_density_grid_nb(route, dtm, region, **kw))


def test_numba_matches_numpy_gentle():
    if not _HAVE_NB:
        print('SKIP  numba not installed'); return
    _run(lambda x, y: 300 + 0.02 * x + 0.01 * y + 8 * np.sin(x / 90.0))


def test_numba_matches_numpy_steep():
    if not _HAVE_NB:
        print('SKIP  numba not installed'); return
    # steep terrain exercises the occlusion march + back-facing culling
    _run(lambda x, y: 300 + 0.18 * x + 20 * np.sin(x / 35.0) * np.cos(y / 50.0))


def test_numba_matches_numpy_nfb_off():
    if not _HAVE_NB:
        print('SKIP  numba not installed'); return
    _run(lambda x, y: 300 + 0.05 * x + 10 * np.sin(x / 60.0), nfb=False)


def _write(path, Z, crs, tr, nodata=-9999.0):
    with rasterio.open(path, 'w', driver='GTiff', height=Z.shape[0], width=Z.shape[1],
                       count=1, dtype='float32', crs=crs, transform=tr, nodata=nodata) as d:
        d.write(Z.astype('float32'), 1)
    return DTM(path)


def test_numba_matches_numpy_geographic():
    """WGS84 lon/lat DTM (is_geo=True) — the lon_m = 111139·cos(lat) path."""
    if not _HAVE_NB:
        print('SKIP  numba not installed'); return
    n = 400
    yy, xx = np.mgrid[0:n, 0:n]
    Z = 300 + 0.05 * xx + 0.03 * yy + 12 * np.sin(xx / 40.0)
    px = 1.0 / 111139.0
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm = _write(os.path.join(tmp, 'geo.tif'), Z, 'EPSG:4326',
                     from_origin(34.0, 31.0 + n * px, px, px))
        reg = [(34.0 + 50 * px, 31.0 + 50 * px), (34.0 + 350 * px, 31.0 + 50 * px),
               (34.0 + 350 * px, 31.0 + 350 * px), (34.0 + 50 * px, 31.0 + 350 * px),
               (34.0 + 50 * px, 31.0 + 50 * px)]
        route = []
        for i, c in enumerate(range(70, 350, 40)):
            x = 34.0 + c * px
            route += [{'x': x, 'y': 31.0 + 60 * px, 'z': 400.0, 'pass_id': i},
                      {'x': x, 'y': 31.0 + 340 * px, 'z': 400.0, 'pass_id': i}]
        kw = dict(_KW, is_geo=True)
        _assert_identical(estimate_density_grid(route, dtm, reg, **kw),
                          estimate_density_grid_nb(route, dtm, reg, **kw))


def _proj_route_region(n=400):
    reg = [(500050, 3900050), (500350, 3900050), (500350, 3900350),
           (500050, 3900350), (500050, 3900050)]
    route = []
    for i, c in enumerate(range(70, 350, 40)):
        route += [{'x': 500000.0 + c, 'y': 3900060.0, 'z': 400.0, 'pass_id': i},
                  {'x': 500000.0 + c, 'y': 3900340.0, 'z': 400.0, 'pass_id': i}]
    return route, reg


def test_numba_matches_numpy_chm():
    """CHM canopy thinning path."""
    if not _HAVE_NB:
        print('SKIP  numba not installed'); return
    n = 400
    yy, xx = np.mgrid[0:n, 0:n]
    Z = 300 + 0.05 * xx + 0.03 * yy + 12 * np.sin(xx / 40.0)
    tr = from_origin(500000, 3900000 + n, 1, 1)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm = _write(os.path.join(tmp, 'p.tif'), Z, 'EPSG:32633', tr)
        chm = _write(os.path.join(tmp, 'chm.tif'),
                     np.where(xx > n // 2, 10.0, 0.0), 'EPSG:32633', tr)
        route, reg = _proj_route_region(n)
        kw = dict(_KW, chm=chm, veg_penetration=0.4)
        _assert_identical(estimate_density_grid(route, dtm, reg, **kw),
                          estimate_density_grid_nb(route, dtm, reg, **kw))


def test_numba_matches_numpy_nodata():
    """nodata voids in the DTM (NaN terrain + gradient mean-fill)."""
    if not _HAVE_NB:
        print('SKIP  numba not installed'); return
    n = 400
    yy, xx = np.mgrid[0:n, 0:n]
    Z = 300 + 0.05 * xx + 0.03 * yy + 12 * np.sin(xx / 40.0)
    Z[100:150, 100:150] = -9999.0                          # a nodata block
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm = _write(os.path.join(tmp, 'nd.tif'), Z, 'EPSG:32633',
                     from_origin(500000, 3900000 + n, 1, 1))
        route, reg = _proj_route_region(n)
        _assert_identical(estimate_density_grid(route, dtm, reg, **_KW),
                          estimate_density_grid_nb(route, dtm, reg, **_KW))


if __name__ == '__main__':
    if not _HAVE_NB:
        print('SKIP  numba not installed — nothing to validate'); sys.exit(0)
    ok = True
    for fn in (test_numba_matches_numpy_gentle, test_numba_matches_numpy_steep,
               test_numba_matches_numpy_nfb_off, test_numba_matches_numpy_geographic,
               test_numba_matches_numpy_chm, test_numba_matches_numpy_nodata):
        try:
            fn(); print(f'PASS  {fn.__name__}')
        except AssertionError as e:
            ok = False; print(f'FAIL  {fn.__name__}: {e}')
    sys.exit(0 if ok else 1)
