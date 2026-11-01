"""Export CSV of a route loaded from a CSV writes the ORIGINAL file back — every row and
column kept — with your edits applied to the lines: X/Y = the new midpoint, deleted lines'
rows removed, Add Pass lines as new rows copied from the nearest line (fresh ID). Cropping
to the polygon is not an edit: parts outside it stay, and unedited rows come back verbatim.

Drives the real main window offscreen with the dialogs patched. Runs standalone
(`python tests/test_route_csv_export.py`) and under pytest.
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
from shapely.geometry import box, mapping                   # noqa: E402
from shapely.wkt import loads as wkt_loads                  # noqa: E402
from PySide6.QtWidgets import QApplication                  # noqa: E402

_app = QApplication.instance() or QApplication([])
from ui.main_window import MainWindow                      # noqa: E402

N = 1200
HEADER = ['ID', 'X', 'Y', 'WKT', 'שם', 'תאור', 'תאריך יצירה', 'תאריך עדכון']


def _dtm(tmp):
    yy, xx = np.mgrid[0:N, 0:N]
    p = os.path.join(tmp, 'd.tif')
    with rasterio.open(p, 'w', driver='GTiff', height=N, width=N, count=1, dtype='float32',
                       crs='EPSG:32636', transform=from_origin(0, N, 1, 1), nodata=-9999.0) as d:
        d.write((300 + 0.04 * xx + 12 * np.sin(yy / 90.0)).astype('float32'), 1)
    return p


def _row(i, wkt, name):
    g = wkt_loads(wkt)
    a, b = g.coords[0] if g.geom_type == 'LineString' else g.geoms[0].coords[0], \
        g.coords[-1] if g.geom_type == 'LineString' else g.geoms[-1].coords[-1]
    return [str(i), f'{(a[0] + b[0]) / 2:.2f}', f'{(a[1] + b[1]) / 2:.2f}', wkt, name,
            f'תיאור {i}', '01/03/2026', '05/03/2026']


def _write_csv(tmp, rows):
    p = os.path.join(tmp, 'lines.csv')
    with open(p, 'w', encoding='utf-8-sig', newline='') as f:     # Excel-style: BOM + CRLF
        csv.writer(f, lineterminator='\r\n').writerows([HEADER] + rows)
    return p


def _read_csv(p):
    raw = open(p, 'rb').read()
    with open(p, encoding='utf-8-sig', newline='') as f:
        return raw, list(csv.reader(f))


def _window(tmp, csv_path, poly=box(100, 100, 900, 900)):
    w = MainWindow(); w.resize(1400, 900); w.show(); _app.processEvents()
    with mock.patch('ui.main_window.QFileDialog.getOpenFileName',
                    return_value=(_dtm(tmp), '')):
        w._open_dtm()
    w._on_polygon_drawn(mapping(poly))
    with mock.patch('ui.main_window.QFileDialog.getOpenFileNames',
                    return_value=([csv_path], '')):
        w._load_wkt_route()
    _app.processEvents()
    return w


def _pids(w, row, seg=None):
    """Pass ids that came from CSV row `row` (0-based data row), optionally one segment."""
    return [pid for pid, key in w._route_src.items()
            if key and key[2] == row and (seg is None or key[3] == seg)]


def _confirm(w, pids):
    for pid, d in w.mapview._route_passes.items():
        d['selected'] = pid in pids
    w._confirm_selected_passes(); _app.processEvents()


def _export(w, out):
    with mock.patch('ui.main_window.QFileDialog.getSaveFileName', return_value=(out, '')):
        w._export_csv()


def _xy(r):
    return float(r[1]), float(r[2])


def test_export_writes_original_csv_with_edits():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        rows = [
            _row(1, 'LINESTRING (200 200, 800 200)', 'edited'),         # moved
            _row(2, 'LINESTRING (300 50, 300 700)', 'cropped'),         # starts outside
            _row(3, 'LINESTRING (400 200, 400 800)', 'deleted'),
            _row(4, 'LINESTRING (500 200, 500 800)', 'not selected'),
            _row(5, 'LINESTRING (1000 100, 1100 200)', 'outside'),
            _row(6, 'LINESTRING (600 200, 600 500, 700 800)', 'half deleted'),
        ]
        src = _write_csv(tmp, rows)
        w = _window(tmp, src)
        picked = _pids(w, 0) + _pids(w, 1) + _pids(w, 2) + _pids(w, 5)
        assert len(picked) == 5, w._route_src                      # row 6 = 2 segments
        _confirm(w, picked)
        # edits: move line 1's end, delete line 3 and line 6's 2nd segment, add a line
        w._on_edit_geom([(_pids(w, 0)[0], (200, 200), (800, 260))])
        w._on_edit_delete(_pids(w, 2) + _pids(w, 5, seg=1))
        w._on_pass_drawn(((150, 850), (250, 850)))
        _app.processEvents()

        out = os.path.join(tmp, 'lines_edited.csv')
        _export(w, out)
        raw, got = _read_csv(out)
        assert raw.startswith(b'\xef\xbb\xbf') and b'\r\n' in raw   # same file format
        assert got[0] == HEADER                                     # same columns
        by_id = {r[0]: r for r in got[1:]}
        assert '3' not in by_id                                     # deleted line's row
        assert by_id['4'] == rows[3] and by_id['5'] == rows[4]      # untouched, verbatim

        r1 = by_id['1']                                             # moved end point
        assert wkt_loads(r1[3]).equals(wkt_loads('LINESTRING (200 200, 800 260)')), r1[3]
        assert _xy(r1) == (500.0, 230.0) and r1[1] == '500.00'      # midpoint, same precision
        assert r1[4:] == rows[0][4:]                                # other columns kept

        assert by_id['2'] == rows[1]                # cropped for planning only: not cut
        assert not wkt_loads(r1[3]).has_z                           # 2D stays 2D

        r6 = by_id['6']                                             # 2nd segment deleted
        assert wkt_loads(r6[3]).equals(wkt_loads('LINESTRING (600 200, 600 500)')), r6[3]
        assert _xy(r6) == (600.0, 350.0)

        new = by_id['7']                                            # Add Pass: fresh ID
        assert wkt_loads(new[3]).equals(wkt_loads('LINESTRING (150 850, 250 850)')), new[3]
        assert _xy(new) == (200.0, 850.0)
        assert new[4:] == rows[1][4:]                               # copied from nearest (2)
        assert [r[0] for r in got[1:]] == ['1', '2', '4', '5', '6', '7']   # order kept
        w.close()


def test_3d_altitude_change_is_exported():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        rows = [_row(1, 'LINESTRING Z (200 300 520.5, 800 300 520.5)', 'a'),
                _row(2, 'LINESTRING Z (200 600 530.5, 800 600 530.5)', 'b')]
        w = _window(tmp, _write_csv(tmp, rows))
        _confirm(w, _pids(w, 0) + _pids(w, 1))
        w._on_pass_altitude_changed(_pids(w, 0)[0], 650.0)          # profile drag
        _app.processEvents()
        out = os.path.join(tmp, 'z.csv')
        _export(w, out)
        _, got = _read_csv(out)
        g1 = wkt_loads(got[1][3])
        assert g1.has_z and {c[2] for c in g1.coords} == {650.0}, got[1][3]
        assert got[1][3].startswith('LINESTRING Z (200 300 650.0'), got[1][3]  # same precision
        assert got[2] == rows[1]                                    # unchanged: verbatim
        w.close()


def test_outside_parts_are_kept_with_the_edits():
    """Parts of a line outside the polygon are not cropped from the export: an edited
    stretch is swapped in and the outside / unselected parts stay as they were."""
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        rows = [_row(1, 'LINESTRING (20 300, 80 300, 400 300)', 'edge 1 outside'),
                _row(2, 'LINESTRING (200 500, 400 500, 600 500)', 'edge 2 not selected'),
                _row(3, 'LINESTRING (300 50, 300 700)', 'starts outside, deleted'),
                _row(4, 'LINESTRING (200 700, 400 750, 600 700)', 'untouched')]
        w = _window(tmp, _write_csv(tmp, rows))
        assert _pids(w, 0, seg=0) == []                              # edge 1: no pass at all
        _confirm(w, _pids(w, 0) + _pids(w, 1, seg=0) + _pids(w, 2) + _pids(w, 3))
        # move the inside end of line 1 (its flown stretch is 100 300 -> 400 300)
        w._on_edit_geom([(_pids(w, 0)[0], (100, 300), (400, 350))])
        w._on_edit_delete(_pids(w, 2))
        out = os.path.join(tmp, 'multi.csv')
        _export(w, out)
        _, got = _read_csv(out)
        by_id = {r[0]: r for r in got[1:]}
        assert wkt_loads(by_id['1'][3]).equals(
            wkt_loads('LINESTRING (20 300, 80 300, 100 300, 400 350)')), by_id['1'][3]
        assert _xy(by_id['1']) == (210.0, 325.0)
        assert by_id['2'] == rows[1]                                 # unselected edge kept
        assert '3' not in by_id                                      # deleted line: row gone
        assert by_id['4'] == rows[3]                                 # verbatim
        msg = w.statusBar().currentMessage()
        assert '1 changed, 1 removed' in msg, msg
        w.close()


def test_3d_edges_at_different_altitudes_split():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        rows = [_row(1, 'LINESTRING Z (200 300 520, 400 300 520, 600 300 520)', 'a')]
        w = _window(tmp, _write_csv(tmp, rows))
        _confirm(w, _pids(w, 0))
        w._on_pass_altitude_changed(_pids(w, 0, seg=1)[0], 650.0)     # 2nd edge higher
        _app.processEvents()
        out = os.path.join(tmp, 'z2.csv')
        _export(w, out)
        _, got = _read_csv(out)
        g = wkt_loads(got[1][3])
        assert g.equals(wkt_loads('MULTILINESTRING Z ((200 300 520, 400 300 520), '
                                  '(400 300 650, 600 300 650))')), got[1][3]
        assert [c[2] for p in g.geoms for c in p.coords] == [520, 520, 650, 650]
        assert _xy(got[1]) == (400.0, 300.0)
        w.close()


def test_line_across_two_polygons_keeps_both_parts():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        polys = os.path.join(tmp, 'polys.csv')
        with open(polys, 'w', encoding='utf-8', newline='') as f:
            csv.writer(f).writerows([['name', 'wkt'],
                                     ['west', 'POLYGON ((100 100, 500 100, 500 900, 100 900, 100 100))'],
                                     ['east', 'POLYGON ((700 100, 1100 100, 1100 900, 700 900, 700 100))']])
        rows = [_row(1, 'LINESTRING (200 500, 1000 500)', 'across both')]
        w = MainWindow(); w.resize(1400, 900); w.show(); _app.processEvents()
        with mock.patch('ui.main_window.QFileDialog.getOpenFileName',
                        return_value=(_dtm(tmp), '')):
            w._open_dtm()
        with mock.patch('ui.main_window.QFileDialog.getOpenFileNames',
                        return_value=([polys], '')):
            w._load_wkt_aoi()
        with mock.patch('ui.main_window.QFileDialog.getOpenFileNames',
                        return_value=([_write_csv(tmp, rows)], '')):
            w._load_wkt_route()
        _app.processEvents()
        _confirm(w, _pids(w, 0))                                    # west part
        w.cmb_aoi.setCurrentIndex(1); _app.processEvents()
        _confirm(w, _pids(w, 0))                                    # east part
        out = os.path.join(tmp, 'both.csv')
        _export(w, out)
        _, got = _read_csv(out)
        g = wkt_loads(got[1][3])
        assert got[1] == rows[0], got[1]            # flown in both, unedited: as it was
        # move the east part: the gap between the polygons stays original
        w._on_edit_geom([(_pids(w, 0)[0], (700, 500), (1000, 560))])
        _export(w, out)
        _, got = _read_csv(out)
        g = wkt_loads(got[1][3])
        assert g.equals(wkt_loads('LINESTRING (200 500, 500 500, 700 500, 1000 560)')), \
            got[1][3]
        assert _xy(got[1]) == (600.0, 530.0)                        # start-to-end midpoint
        w.close()


def _drag_body(w, at, to):
    """Edit Route mouse drag on the map, in world coordinates: press at `at`, move, release."""
    m = w.mapview
    if not m.editing:
        m.btn_edit.setChecked(True); m._toggle_edit(True)
    a, b = m._scene(*at), m._scene(*to)
    assert m.edit_press(a)
    m.edit_drag_move(b); m.edit_drag_release(b)
    _app.processEvents()


def _passes(w):
    out = {}
    for p in w.survey_route:
        out.setdefault(p['pass_id'], []).append((round(p['x'], 6), round(p['y'], 6)))
    return {pid: (pts[0], pts[-1]) for pid, pts in out.items()}


def test_dragging_a_line_body_bends_it_at_a_new_joint():
    """Drag the middle of a line: it splits into two lines meeting at the new joint (the
    ends stay put); export keeps it ONE row with the extra vertex. A click doesn't split."""
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        rows = [_row(1, 'LINESTRING (200 300, 800 300)', 'bent'),
                _row(2, 'LINESTRING (200 600, 800 600)', 'clicked only')]
        w = _window(tmp, _write_csv(tmp, rows))
        _confirm(w, _pids(w, 0) + _pids(w, 1))
        before = _passes(w)
        _drag_body(w, (500, 600), (500, 600))                       # click: no split
        assert _passes(w) == before
        pid = _pids(w, 0)[0]
        _drag_body(w, (400, 300), (450, 400))                       # bend line 1 at (450, 400)
        after = _passes(w)
        assert len(after) == len(before) + 1, after
        joint = (450.0, 400.0)
        halves = sorted(v for k, v in after.items() if k not in before or k == pid)
        assert halves == [((200.0, 300.0), joint), (joint, (800.0, 300.0))], halves
        assert w.result is not None and w._edit_undo                 # re-estimated, undoable

        out = os.path.join(tmp, 'bent.csv')
        _export(w, out)
        _, got = _read_csv(out)
        assert len(got) == 3, got                                   # header + 2 rows, no new row
        assert wkt_loads(got[1][3]).equals(
            wkt_loads('LINESTRING (200 300, 450 400, 800 300)')), got[1][3]
        assert _xy(got[1]) == (500.0, 300.0)                        # start-to-end midpoint
        assert got[2] == rows[1]

        _drag_body(w, (300, 350), (300, 450))                       # bend the 1st half again
        _export(w, out)
        _, got = _read_csv(out)
        g = wkt_loads(got[1][3])
        assert len(got) == 3 and len(g.coords) == 4, got[1][3]
        assert list(g.coords)[-2:] == [(450, 400), (800, 300)]      # order along the line kept

        w._undo_edit(); w._undo_edit(); _app.processEvents()        # back to the straight line
        assert _passes(w) == before
        _export(w, out)
        _, got = _read_csv(out)
        assert got[1] == rows[0]
        w.close()


def test_original_file_is_never_overwritten():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        src = _write_csv(tmp, [_row(1, 'LINESTRING (200 200, 800 200)', 'a')])
        before = open(src, 'rb').read()
        w = _window(tmp, src)
        _confirm(w, _pids(w, 0))
        with mock.patch('ui.main_window.QMessageBox.warning') as warn:
            _export(w, src)
        assert warn.called and open(src, 'rb').read() == before
        w.close()


if __name__ == '__main__':
    ok = True
    for fn in (test_export_writes_original_csv_with_edits, test_3d_altitude_change_is_exported,
               test_outside_parts_are_kept_with_the_edits,
               test_3d_edges_at_different_altitudes_split,
               test_line_across_two_polygons_keeps_both_parts,
               test_dragging_a_line_body_bends_it_at_a_new_joint,
               test_original_file_is_never_overwritten):
        try:
            fn(); print(f'PASS  {fn.__name__}')
        except AssertionError as e:
            ok = False; print(f'FAIL  {fn.__name__}: {e}')
    os._exit(0 if ok else 1)
