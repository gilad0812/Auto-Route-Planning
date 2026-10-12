"""Several polygons from one file: pick one to work on; switching keeps each polygon's
route, results, edits and undo history.

Drives the real main window offscreen with the file / message dialogs patched. Runs
standalone (`python tests/test_multi_polygon.py`) and under pytest.
"""
import csv
import os
import sys
import tempfile
from unittest import mock

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [_ROOT, os.path.join(_ROOT, 'src')]
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import numpy as np                                          # noqa: E402
import rasterio                                             # noqa: E402
from rasterio.transform import from_origin                  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox     # noqa: E402

_app = QApplication.instance() or QApplication([])
from ui.main_window import MainWindow, _read_wkt_geoms     # noqa: E402

N = 1200
POLYS = [('north', 'POLYGON ((100 650, 550 650, 550 1100, 100 1100, 100 650))'),
         ('south', 'POLYGON ((100 100, 550 100, 550 550, 100 550, 100 100))'),
         ('east', 'POLYGON ((700 300, 1100 300, 1100 900, 700 900, 700 300))'),
         ('far', 'POLYGON ((50000 50000, 50300 50000, 50300 50300, 50000 50000))'),
         ('pin', 'POINT (10 10)')]


def _dtm(tmp):
    yy, xx = np.mgrid[0:N, 0:N]
    p = os.path.join(tmp, 'd.tif')
    with rasterio.open(p, 'w', driver='GTiff', height=N, width=N, count=1, dtype='float32',
                       crs='EPSG:32636', transform=from_origin(0, N, 1, 1), nodata=-9999.0) as d:
        d.write((300 + 0.04 * xx + 12 * np.sin(yy / 90.0)).astype('float32'), 1)
    return p


def _csv(tmp, rows, delim=','):
    p = os.path.join(tmp, 'blocks.csv')
    with open(p, 'w', newline='', encoding='utf-8') as f:
        wr = csv.writer(f, delimiter=delim)
        wr.writerow(['name', 'wkt'])
        wr.writerows(rows)
    return p


def _open(w, path):
    with mock.patch('ui.main_window.QFileDialog.getOpenFileName', return_value=(path, '')):
        if path.endswith('.tif'):
            w._open_dtm()
        else:
            w._load_wkt_aoi()
    _app.processEvents()


def _ids(route):
    return [(round(p['x'], 3), round(p['y'], 3), round(p['z'], 3), p['pass_id']) for p in route]


def test_switching_keeps_each_polygons_work():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        w = MainWindow(); w.resize(1400, 900); w.show(); _app.processEvents()
        _open(w, _dtm(tmp))
        with mock.patch('ui.main_window.QMessageBox.warning') as warn:
            _open(w, _csv(tmp, POLYS))
        msg = warn.call_args[0][2]
        assert 'far' in msg and 'pin' in msg, msg               # unusable rows reported
        names = [w.cmb_aoi.itemText(i) for i in range(w.cmb_aoi.count())]
        assert names == ['north', 'south', 'east'], names
        assert w.aoi_row.isVisibleTo(w) and w._aoi_active == 0
        assert w.mapview._other_aoi_item is not None              # the others are outlined

        # plan + edit "north"
        w._compute(); _app.processEvents()
        assert w.result and w.result.route
        pts = [q for q in w.survey_route if q['pass_id'] == 1]
        a, b = (pts[0]['x'], pts[0]['y']), (pts[-1]['x'], pts[-1]['y'])
        w._on_edit_geom([(1, (a[0] + 25, a[1]), (b[0] + 25, b[1]))]); _app.processEvents()
        north_route = _ids(w.survey_route)
        assert len(w._edit_undo) == 1
        assert '✓' in w.cmb_aoi.itemText(0)

        # switch to "south": a clean slate
        w.cmb_aoi.setCurrentIndex(1); _app.processEvents()
        assert w._aoi_active == 1 and w.result is None and w.survey_route == []
        assert not w.act_undo.isEnabled() and not w.btn_geojson.isEnabled()
        assert w.drawn_polygon.bounds == (100.0, 100.0, 550.0, 550.0)
        w._compute(); _app.processEvents()
        south_route = _ids(w.survey_route)
        assert south_route and south_route != north_route

        # back to "north": its edited route, results and undo history are all back
        w.cmb_aoi.setCurrentIndex(0); _app.processEvents()
        assert _ids(w.survey_route) == north_route
        assert w.result and w.result.route and w.btn_geojson.isEnabled()
        assert len(w._edit_undo) == 1 and w.act_undo.isEnabled()
        assert 'north' in w.lbl_summary.text()                    # results name the polygon
        assert w._export_name('route.csv') == 'north_route.csv'
        w._undo_edit(); _app.processEvents()                      # undo still works here
        assert _ids(w.survey_route) != north_route

        # and "south" is untouched by all that
        w.cmb_aoi.setCurrentIndex(1); _app.processEvents()
        assert _ids(w.survey_route) == south_route

        # clearing asks first (routes would be lost); No keeps everything
        with mock.patch('ui.main_window.QMessageBox.question', return_value=QMessageBox.No):
            w._clear_aoi()
        assert len(w._aoi_set) == 3 and w.drawn_polygon is not None
        with mock.patch('ui.main_window.QMessageBox.question', return_value=QMessageBox.Yes):
            w._clear_aoi()
        _app.processEvents()
        assert w._aoi_set == [] and not w.aoi_row.isVisibleTo(w)
        assert w.mapview._other_aoi_item is None
        w.close()


def test_single_polygon_file_has_no_selector():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        w = MainWindow(); w.resize(1400, 900); w.show(); _app.processEvents()
        _open(w, _dtm(tmp))
        _open(w, _csv(tmp, POLYS[:1]))
        assert w._aoi_set == [] and not w.aoi_row.isVisibleTo(w)
        assert w.drawn_polygon is not None
        w.close()


def test_polygons_loaded_before_the_dtm():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        w = MainWindow(); w.resize(1400, 900); w.show(); _app.processEvents()
        _open(w, _csv(tmp, POLYS[:3]))                            # no DTM yet
        assert w._pending_aoi_set and 'open a DTM' in w.lbl_aoi.text()
        _open(w, _dtm(tmp))
        assert [e['name'] for e in w._aoi_set] == ['north', 'south', 'east']
        assert w._aoi_active == 0 and w.drawn_polygon is not None and w.btn_compute.isEnabled()
        w.close()


def test_clip_segment():
    from shapely.geometry import Polygon
    from ui.main_window import _clip_segment
    ell = Polygon([(0, 0), (100, 0), (100, 40), (40, 40), (40, 100), (0, 100)])   # an L
    # a horizontal 3D segment from x=-50 to x=150 at y=20 is cut to x 0..100, z interpolated
    (a, b), = _clip_segment((-50, 20, 400.0), (150, 20, 500.0), ell, 1.0)
    assert (a[0], b[0]) == (0.0, 100.0) and abs(a[2] - 425.0) < 1e-9 and abs(b[2] - 475.0) < 1e-9
    # direction kept even when the segment runs right-to-left
    (a, b), = _clip_segment((150, 20), (-50, 20), ell, 1.0)
    assert (a[0], b[0], a[2]) == (100.0, 0.0, None)
    # a vertical segment at x=70 passes only through the L's lower arm -> one piece
    assert len(_clip_segment((70, -10), (70, 150), ell, 1.0)) == 1
    # a diagonal through both arms, crossing the notch between them -> two pieces
    pieces = _clip_segment((10, 90), (90, 10), ell, 1.0)
    assert [(round(a[0]), round(b[0])) for a, b in pieces] == [(10, 40), (60, 90)], pieces
    # slivers shorter than the minimum are dropped; fully outside -> nothing
    assert _clip_segment((99, 20), (130, 20), ell, 5.0) == []
    assert _clip_segment((200, 0), (300, 0), ell, 1.0) == []


def _lines_csv(tmp, rows):
    p = os.path.join(tmp, 'lines.csv')
    with open(p, 'w', newline='', encoding='utf-8') as f:
        wr = csv.writer(f); wr.writerow(['name', 'wkt']); wr.writerows(rows)
    return p


def _load_route(w, path):
    with mock.patch('ui.main_window.QFileDialog.getOpenFileName', return_value=(path, '')):
        w._load_wkt_route()
    _app.processEvents()


def test_lines_cropped_and_shared_between_polygons():
    from shapely.geometry import Point
    from ui.main_window import _route_pass_pts
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        w = MainWindow(); w.resize(1400, 900); w.show(); _app.processEvents()
        _open(w, _dtm(tmp))
        _open(w, _csv(tmp, POLYS[:3]))                           # north, south, east
        lines = _lines_csv(tmp, [
            ('across-north', 'LINESTRING (20 800, 1180 800)'),   # north only (east is y<=900... and x>=700)
            ('across-both', 'LINESTRING (20 450, 1180 450)'),    # south and east
            ('nowhere', 'LINESTRING (600 20, 650 60)')])         # outside every polygon
        _load_route(w, lines)
        # the current polygon (north) shows only its cropped passes for selection
        north = w.drawn_polygon.buffer(1e-6)
        pp = _route_pass_pts(w.loaded_route)
        assert pp and all(north.covers(Point(*q)) for _, s in pp for q in s), pp
        assert w.mapview.selecting_passes
        assert 'outside every polygon ignored' in w.statusBar().currentMessage()
        # the others got their own pieces, waiting
        south_lines = _route_pass_pts(w._aoi_set[1]['slot']['loaded_route'])
        east_lines = _route_pass_pts(w._aoi_set[2]['slot']['loaded_route'])
        assert south_lines and east_lines
        assert all(w._aoi_set[1]['poly'].buffer(1e-6).covers(Point(*q))
                   for _, s in south_lines for q in s)
        assert 'lines to confirm' in w.cmb_aoi.itemText(1)
        # switching to south shows its waiting lines; select all + confirm estimates them
        w.cmb_aoi.setCurrentIndex(1); _app.processEvents()
        assert w.mapview.selecting_passes and w.loaded_route
        w.mapview._toggle_select_all(); w._confirm_selected_passes(); _app.processEvents()
        assert w.result and w.result.route and '✓' in w.cmb_aoi.itemText(1)
        # loading lines again asks before replacing south's route; No changes nothing
        w.cmb_aoi.setCurrentIndex(0); _app.processEvents()
        before = _route_pass_pts(w._aoi_set[1]['slot']['survey_route'])
        with mock.patch('ui.main_window.QMessageBox.question',
                        return_value=QMessageBox.No) as q:
            _load_route(w, lines)
        assert q.called and _route_pass_pts(w._aoi_set[1]['slot']['survey_route']) == before
        w.close()


def test_other_outlines_only_what_is_in_view():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        w = MainWindow(); w.resize(1400, 900); w.show(); _app.processEvents()
        _open(w, _dtm(tmp))
        far = ('remote', 'POLYGON ((1000 1000, 1150 1000, 1150 1150, 1000 1150, 1000 1000))')
        _open(w, _csv(tmp, [POLYS[1], far]))                     # south (active) + remote
        item = w.mapview._other_aoi_item                          # focused on south
        assert item is None or not item.childItems(), 'remote is out of view: not drawn'
        w._render_map(focus=False); _app.processEvents()          # full extent: drawn
        assert w.mapview._other_aoi_item is not None and w.mapview._other_aoi_item.childItems()
        w.close()


def test_reader_names_and_semicolons():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        p = _csv(tmp, POLYS[:2], delim=';')                       # Excel ';' CSV
        assert [n for n, _ in _read_wkt_geoms(p, with_names=True)] == ['north', 'south']
        assert len(_read_wkt_geoms(p)) == 2                       # the old call still works


if __name__ == '__main__':
    ok = True
    for fn in (test_switching_keeps_each_polygons_work, test_single_polygon_file_has_no_selector,
               test_polygons_loaded_before_the_dtm, test_clip_segment,
               test_lines_cropped_and_shared_between_polygons,
               test_other_outlines_only_what_is_in_view, test_reader_names_and_semicolons):
        try:
            fn(); print(f'PASS  {fn.__name__}')
        except AssertionError as e:
            ok = False; print(f'FAIL  {fn.__name__}: {e}')
    os._exit(0 if ok else 1)
