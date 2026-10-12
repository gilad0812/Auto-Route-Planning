"""The window must fit small / high-scaling screens: the map toolbar wraps instead of
forcing the window wide, every visible toolbar button stays reachable, and the window is
fitted to the screen's free area on first show (also pulled back if off-screen).

Runs standalone (`python tests/test_window_fits.py`) and under pytest (offscreen Qt).
"""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [_ROOT, os.path.join(_ROOT, 'src')]
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PySide6.QtCore import QRect                         # noqa: E402
from PySide6.QtWidgets import QApplication              # noqa: E402

_app = QApplication.instance() or QApplication([])
from ui.main_window import MainWindow                    # noqa: E402


def _toolbar_ok(w):
    """Every visible toolbar widget lies inside the toolbar and none overlap."""
    m = w.mapview
    bar = m.btn_draw.parentWidget()
    items = [b for b in (m.btn_draw, m.btn_edit, m.btn_pass, m.btn_del, m.btn_point, m.btn_fit,
                         m.btn_focus, m.btn_all, m.btn_confirm) if b.isVisibleTo(w)]
    inside = all(bar.rect().contains(b.geometry()) for b in items)
    overlap = any(a.geometry().intersects(b.geometry())
                  for i, a in enumerate(items) for b in items[i + 1:])
    rows = len({b.geometry().y() // 10 for b in items})
    return inside and not overlap, rows


def _floor(w):
    """Narrowest the window could be: the side panels' minimums plus the widest single
    toolbar item (font metrics differ per platform, so no fixed pixel number)."""
    m = w.mapview
    widest = max(b.sizeHint().width() for b in (m.btn_draw, m.btn_edit, m.btn_pass, m.btn_del,
                 m.btn_point, m.btn_fit, m.btn_focus, m.btn_all, m.btn_confirm))
    sp = m.parentWidget()                                 # sidebar | map | results
    return sum(sp.widget(i).minimumWidth() for i in range(sp.count())) + widest


def test_minimum_width_fits_small_screens():
    w = MainWindow(); w.show(); _app.processEvents()
    # the toolbar no longer adds up all its buttons into the window's minimum width
    assert w.minimumSizeHint().width() <= _floor(w) + 60, (w.minimumSizeHint(), _floor(w))
    w.close()


def test_toolbar_wraps_and_stays_reachable():
    w = MainWindow(); w.show(); _app.processEvents()
    lo = w.minimumSizeHint().width()
    room = max(w.screen().availableGeometry().width() - 20, lo)   # a real screen caps it
    wide = min(lo + 900, room)
    for width in (wide, min(lo + 300, room), lo):
        w.resize(width, 700); _app.processEvents()
        assert w.width() == width, (width, w.width())
        ok, rows = _toolbar_ok(w)
        assert ok, f'toolbar items clipped/overlapping at {width}px'
    _, rows_wide = (w.resize(wide, 700), _app.processEvents(), _toolbar_ok(w))[2]
    bar = w.mapview.btn_draw.parentWidget()
    if bar.layout().sizeHint().width() <= bar.width():    # room for one row: one row, as before
        assert rows_wide == 1, rows_wide
    m = w.mapview
    # Edit Route mode shows extra buttons: still all inside at a narrow width
    m.btn_edit.setEnabled(True); m.btn_edit.setChecked(True); m._toggle_edit(True)
    w.resize(lo, 700); _app.processEvents()
    ok, rows = _toolbar_ok(w)
    assert ok, rows
    w.close()


def test_window_fitted_to_screen():
    w = MainWindow(); w.show(); _app.processEvents()
    w.resize(1600, 1000); w.move(3000, 2000); _app.processEvents()   # too big, off-screen
    avail = QRect(0, 0, 1280, 680)                                   # e.g. 1920@150 %, taskbar
    w._fit_to_screen(avail); _app.processEvents()
    f = w.frameGeometry()
    assert avail.contains(f), (f, avail)
    w.close()


if __name__ == '__main__':
    ok = True
    for fn in (test_minimum_width_fits_small_screens, test_toolbar_wraps_and_stays_reachable,
               test_window_fitted_to_screen):
        try:
            fn(); print(f'PASS  {fn.__name__}')
        except AssertionError as e:
            ok = False; print(f'FAIL  {fn.__name__}: {e}')
    os._exit(0 if ok else 1)
