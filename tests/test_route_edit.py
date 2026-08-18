"""Route-edit recompute pipeline (headless, no Qt).

Exercises the exact data operations the Edit Route handlers run in ui/main_window.py:
deleting a pass (filter the survey by pass_id) and moving a pass endpoint (rebuild via
build_manual_pass + splice in place) must each re-estimate to a valid result, preserve
the rest of the route, and keep pass ids stable. Uses a small synthetic GeoTIFF so it's
self-contained.

Runs standalone (`python tests/test_route_edit.py`, non-zero exit on failure) and under
pytest.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'ui'))

import numpy as np                                          # noqa: E402
import rasterio                                             # noqa: E402
from rasterio.transform import from_origin                  # noqa: E402
from planning import (load_dtm, compute_plan, estimate_for_route,   # noqa: E402
                      build_manual_pass, centered_box, PlanParams)


def _pass_ids(route):
    return sorted({w['pass_id'] for w in route if isinstance(w.get('pass_id'), int)})


def _build_route(tmpdir):
    """A 700 m tilted-plane DTM (projected CRS, is_geo=False) planned into >=3 passes."""
    n = 700
    yy, xx = np.mgrid[0:n, 0:n]
    z = (300.0 + 0.15 * xx + 0.05 * yy).astype('float32')   # gentle NE slope
    path = os.path.join(tmpdir, 'edit_dtm.tif')
    tr = from_origin(500000.0, 3900000.0 + n, 1.0, 1.0)
    with rasterio.open(path, 'w', driver='GTiff', height=n, width=n, count=1,
                       dtype='float32', crs='EPSG:32633', transform=tr,
                       nodata=-9999.0) as d:
        d.write(z, 1)
    dtm = load_dtm(path)
    poly = centered_box(dtm, frac=0.85)
    params = PlanParams(altitude_m=80.0, speed_ms=6.0, min_points=50)
    res = compute_plan(dtm, poly, params, is_geo=False)
    return dtm, poly, params, res.route


def test_delete_pass_reestimates():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm, poly, params, route = _build_route(tmp)
        pids = _pass_ids(route)
        assert len(pids) >= 3, f'need several passes to test (got {len(pids)})'

        victim = pids[len(pids) // 2]
        remaining = [w for w in route if w.get('pass_id') != victim]
        res = estimate_for_route(dtm, poly, remaining, params, is_geo=False)

        assert victim not in _pass_ids(remaining)
        assert _pass_ids(remaining) == [p for p in pids if p != victim]
        assert res.estimate and res.n_waypoints > 0


def test_move_pass_endpoint_reestimates():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dtm, poly, params, route = _build_route(tmp)
        pids = _pass_ids(route)
        edit_pid = pids[0]
        ends = [(w['x'], w['y']) for w in route if w.get('pass_id') == edit_pid]
        p0, p1 = ends[0], ends[-1]
        p1_moved = (p1[0] + 15.0, p1[1] + 10.0)             # shove the end 15 m E / 10 m N

        new_pass = build_manual_pass(dtm, p0, p1_moved, params, is_geo=False,
                                     pass_id=edit_pid)
        assert new_pass, 'rebuilt pass should find terrain'

        new_survey, inserted = [], False
        for w in route:
            if w.get('pass_id') == edit_pid:
                if not inserted:
                    new_survey.extend(new_pass); inserted = True
            else:
                new_survey.append(w)
        res = estimate_for_route(dtm, poly, new_survey, params, is_geo=False)
        new_ends = [(w['x'], w['y']) for w in new_survey if w.get('pass_id') == edit_pid]

        assert _pass_ids(new_survey) == pids, 'no pass lost, id preserved'
        assert abs(new_ends[-1][0] - p1_moved[0]) < 1e-6, 'moved endpoint applied'
        assert res.estimate and res.n_waypoints > 0


def test_wkt_route_roundtrip():
    """Save route (.wkt) writes each pass as a LINESTRING Z; the loader reads the same
    geometry + altitude back. Guards the format contract behind edit -> Save -> Load."""
    from shapely.geometry import LineString, MultiLineString
    from shapely.wkt import loads as wkt_loads

    # two edited passes with distinct altitudes (endpoint waypoints, as survey_route holds)
    survey = [
        {'x': 34.10, 'y': 31.20, 'z': 612.0, 'pass_id': 0},
        {'x': 34.10, 'y': 31.24, 'z': 612.0, 'pass_id': 0},
        {'x': 34.11, 'y': 31.20, 'z': 615.5, 'pass_id': 1},
        {'x': 34.11, 'y': 31.24, 'z': 615.5, 'pass_id': 1},
    ]
    groups, order = {}, []
    for w in survey:
        pid = w['pass_id']
        if pid not in groups:
            order.append(pid)
        groups.setdefault(pid, []).append((w['x'], w['y'], w['z']))
    lines = [LineString([groups[p][0], groups[p][-1]]) for p in order]
    text = MultiLineString(lines).wkt
    assert text.startswith('MULTILINESTRING Z'), text[:25]

    # read back the way the loader does: MultiLineString -> per-segment passes, Z = altitude
    g = wkt_loads(text)
    segs = []
    for ln in (g.geoms if g.geom_type == 'MultiLineString' else [g]):
        cs = list(ln.coords)
        for a, b in zip(cs, cs[1:]):
            segs.append((tuple(round(c, 6) for c in a), tuple(round(c, 6) for c in b)))
    assert len(segs) == 2, f'two passes round-trip (got {len(segs)})'
    assert segs[0] == ((34.1, 31.2, 612.0), (34.1, 31.24, 612.0))
    assert segs[1] == ((34.11, 31.2, 615.5), (34.11, 31.24, 615.5))


if __name__ == '__main__':
    ok = True
    for fn in (test_delete_pass_reestimates, test_move_pass_endpoint_reestimates,
               test_wkt_route_roundtrip):
        try:
            fn()
            print(f'PASS  {fn.__name__}')
        except AssertionError as e:
            ok = False
            print(f'FAIL  {fn.__name__}: {e}')
    sys.exit(0 if ok else 1)
