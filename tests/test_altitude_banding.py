"""band_pass_altitudes groups passes onto the FEWEST distinct altitudes (minimum
interval stabbing), globally — non-adjacent passes that fit the same AGL band share a
height — while only ever raising a pass and keeping every pass inside its band ceiling.

Runs standalone (`python tests/test_altitude_banding.py`) and under pytest.
"""
import os
import sys
import tempfile

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, 'src'))
sys.path.insert(0, os.path.join(_ROOT, 'ui'))

import numpy as np                                          # noqa: E402
import rasterio                                             # noqa: E402
from rasterio.transform import from_origin                  # noqa: E402
from route_planner import band_pass_altitudes               # noqa: E402
from dtm import DTM                                         # noqa: E402


def _dtm(tmp):
    """Terrain in three bands by y: low (300) / ridge (400) / mid (320)."""
    n = 300
    z = np.empty((n, n), 'float32')
    z[200:, :] = 300.0        # rows>=200 -> y in [0,100)    low
    z[100:200, :] = 400.0     # rows 100-199 -> y in [100,200) ridge
    z[:100, :] = 320.0        # rows<100 -> y in [200,300)   mid
    path = os.path.join(tmp, 'd.tif')
    with rasterio.open(path, 'w', driver='GTiff', height=n, width=n, count=1,
                       dtype='float32', crs='EPSG:32633', transform=from_origin(0, n, 1, 1),
                       nodata=-9999.0) as d:
        d.write(z, 1)
    return DTM(path)


def _route():
    # program-assigned floors: pass0=360 (low+60), pass1=460 (ridge+60), pass2=380 (mid+60)
    return [{'x': 10.0, 'y': 50.0, 'z': 360.0, 'pass_id': 0},
            {'x': 290.0, 'y': 50.0, 'z': 360.0, 'pass_id': 0},
            {'x': 10.0, 'y': 150.0, 'z': 460.0, 'pass_id': 1},
            {'x': 290.0, 'y': 150.0, 'z': 460.0, 'pass_id': 1},
            {'x': 10.0, 'y': 250.0, 'z': 380.0, 'pass_id': 2},
            {'x': 290.0, 'y': 250.0, 'z': 380.0, 'pass_id': 2}]


def test_global_banding_minimizes_distinct_altitudes():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm = _dtm(tmp)
        route = _route()
        before = {w['pass_id']: w['z'] for w in route}
        band_pass_altitudes(route, dtm, agl=60.0, is_geo=False, band_half_m=50.0)
        after = {w['pass_id']: w['z'] for w in route}

        # non-adjacent passes 0 & 2 share a height -> only 2 distinct altitudes
        # (consecutive-only banding would leave 3: 360, 460, 380)
        assert len(set(after.values())) == 2, after
        assert after[0] == after[2], 'passes 0 & 2 banded together (not adjacent)'
        assert after[1] == 460.0, 'the ridge pass stays on its own height'
        # only ever raises; never lowers below the planned floor
        for pid in before:
            assert after[pid] >= before[pid], f'pass {pid} was lowered'
        # every pass stays under its band ceiling (valley + AGL + band_half)
        assert after[0] <= 410.0 and after[2] <= 430.0 and after[1] <= 510.0


def test_reband_rebases_no_ratchet_and_is_idempotent():
    """band_route_altitudes (the edit re-band path) re-derives each pass's base altitude
    from terrain first, so altitudes don't ratchet up over edits, then groups; re-running
    it changes nothing."""
    from planning import band_route_altitudes, PlanParams
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm = _dtm(tmp)
        params = PlanParams(altitude_m=60.0, min_peak_clearance_m=50.0)
        # start from ARTIFICIALLY INFLATED altitudes (as if raised by earlier banding)
        route = _route()
        for w in route:
            w['z'] = 600.0
        band_route_altitudes(dtm, route, params, is_geo=False)
        a1 = {w['pass_id']: w['z'] for w in route}
        assert max(a1.values()) < 600.0, 'rebased down from inflated (no ratchet)'
        assert len(set(a1.values())) == 2 and a1[0] == a1[2], a1
        band_route_altitudes(dtm, route, params, is_geo=False)
        a2 = {w['pass_id']: w['z'] for w in route}
        assert a2 == a1, f'not idempotent: {a1} -> {a2}'


def test_banding_is_a_noop_for_a_single_pass():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm = _dtm(tmp)
        route = [{'x': 10.0, 'y': 50.0, 'z': 360.0, 'pass_id': 0},
                 {'x': 290.0, 'y': 50.0, 'z': 360.0, 'pass_id': 0}]
        band_pass_altitudes(route, dtm, agl=60.0, is_geo=False)
        assert all(w['z'] == 360.0 for w in route)


if __name__ == '__main__':
    ok = True
    for fn in (test_global_banding_minimizes_distinct_altitudes,
               test_reband_rebases_no_ratchet_and_is_idempotent,
               test_banding_is_a_noop_for_a_single_pass):
        try:
            fn(); print(f'PASS  {fn.__name__}')
        except AssertionError as e:
            ok = False; print(f'FAIL  {fn.__name__}: {e}')
    sys.exit(0 if ok else 1)
