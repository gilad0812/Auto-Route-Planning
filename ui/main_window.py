"""Main window for the desktop planner.

Left: parameter sidebar + Compute. Right: tabbed views — Summary and the 2D
Leaflet Map (draw the AOI, see the route + under-density overlay). Compute runs
the existing model and fills the Summary panel.
"""
import csv
import json
import math
import os
import re

from PySide6.QtCore import Qt, QSettings
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QFormLayout, QDoubleSpinBox, QSpinBox, QComboBox, QGroupBox,
    QSplitter, QScrollArea, QFileDialog, QMessageBox, QFrame, QProgressBar,
    QApplication, QAbstractSpinBox, QToolButton,
)

from .planning import (PlanParams, compute_plan, load_dtm,
                       scan_lines_for_square_pattern, build_manual_pass,
                       estimate_for_route, polygon_area_m2, MAX_AOI_M2,
                       _pass_altitude, band_pass_altitudes, band_route_altitudes,
                       _LAT_M)
from .geo import from_utm

try:
    from .canvasmap import CanvasMap, FAILURE_REASON_STYLE, FAILURE_REASON_LABEL
    _MAPVIEW_ERR = None
except Exception as _e:
    CanvasMap = None
    _MAPVIEW_ERR = str(_e)
    # Qt-free fallback so the summary legend still renders without the map backend.
    FAILURE_REASON_STYLE = {
        "range": ("#8c959f", 130), "shadow": ("#8250df", 120), "thin": ("#ff9900", 95),
    }
    FAILURE_REASON_LABEL = {
        "range": ("Beyond scanner range", "lower AGL or PRR"),
        "shadow": ("Shadow / faces away", "needs a cross-line, or accept"),
        "thin": ("Under target", "lower AGL / tighter spacing"),
    }

from shapely.geometry import (shape as shapely_shape, box as shapely_box,
                              LineString, MultiLineString, Point)
from shapely.wkt import loads as wkt_loads

PULSE_FREQS = (150_000, 300_000, 600_000, 1_200_000, 1_800_000, 2_400_000)

# Passes are always floored at least this far above their highest point. Fixed (not
# exposed in the UI) — it's a safety clearance, not a routine tuning knob.
_MIN_PEAK_CLEARANCE_M = 50.0

# Column names an exported CSV might use for the WKT geometry.
_WKT_COLUMNS = {'wkt', 'geometry', 'geom', 'the_geom', 'wkt_geom', 'wkt_geometry',
                'shape', 'well_known_text'}


def _looks_like_wkt(s):
    return s.upper().lstrip().startswith((
        'POLYGON', 'MULTIPOLYGON', 'LINESTRING', 'MULTILINESTRING',
        'POINT', 'MULTIPOINT', 'GEOMETRYCOLLECTION', 'LINEARRING'))


def _locate_wkt_column(rows):
    """(column index, data rows) for the WKT column in parsed CSV `rows`, or (None, rows).
    Prefers a known header name; otherwise finds a column whose values look like WKT
    (treating the first row as data if it is itself WKT — a header-less file)."""
    header = rows[0]
    for i, h in enumerate(header):
        if (h or '').strip().lower() in _WKT_COLUMNS:
            return i, rows[1:]
    ncol = max(len(r) for r in rows)
    for i in range(ncol):
        cell = lambda r: (r[i] if i < len(r) else '').strip()
        if _looks_like_wkt(cell(header)):
            return i, rows                      # header row was actually data
        if any(_looks_like_wkt(cell(r)) for r in rows[1:3]):
            return i, rows[1:]
    return None, rows


def _densify_polyline(xy, step):
    """Sample points along a polyline `xy` [(x,y)] at ~`step` spacing (map units), so a
    two-vertex pass gets enough terrain samples for the altitude rule."""
    if len(xy) < 2 or step <= 0:
        return list(xy)
    out = [xy[0]]
    for (x0, y0), (x1, y1) in zip(xy, xy[1:]):
        n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
        out.extend((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n)
                   for i in range(1, n + 1))
    return out


# Column names a CSV might use to name each polygon.
_NAME_COLUMNS = ('name', 'id', 'fid', 'label', 'title', 'polygon', 'poly', 'area')


def _read_wkt_geoms(path, with_names=False):
    """Geometries from a `.wkt` (plain text, one geometry) or a `.csv` with a WKT column
    (one geometry per row). Raises ValueError with a clear message on failure.

    with_names=True returns [(name, geometry)] instead: the name comes from a CSV column
    such as 'name'/'id' (see _NAME_COLUMNS) when there is one, else None; a .wkt file is
    named after the file."""
    if os.path.splitext(path)[1].lower() != '.csv':
        with open(path, 'r', encoding='utf-8') as f:
            txt = f.read().strip()
        if not txt:
            raise ValueError('File is empty.')
        g = wkt_loads(txt)
        return [(os.path.splitext(os.path.basename(path))[0], g)] if with_names else [g]
    with open(path, 'r', encoding='utf-8-sig', newline='') as f:
        content = f.read()
    if not content.strip():
        raise ValueError('CSV is empty.')
    lines = content.splitlines()
    parse_err = None
    for delim in (',', ';', '\t', '|'):         # pick the delimiter that yields a WKT column
        rows = [r for r in csv.reader(lines, delimiter=delim)
                if any((c or '').strip() for c in r)]
        if not rows:
            continue
        col, data_rows = _locate_wkt_column(rows)
        if col is None:
            continue
        name_col = None
        if data_rows is not rows:                   # the first row was a header
            hdr = [(h or '').strip().lower() for h in rows[0]]
            name_col = next((hdr.index(n) for n in _NAME_COLUMNS
                             if n in hdr and hdr.index(n) != col), None)
        named = []
        try:
            for r in data_rows:
                v = (r[col] if col < len(r) else '').strip()
                if v:
                    nm = (r[name_col].strip() if name_col is not None and name_col < len(r)
                          else '') or None
                    named.append((nm, wkt_loads(v)))
        except Exception as e:                  # this separator cut the WKT apart (e.g. a
            parse_err = e                       # ';' CSV read with ',') — try the next one
            continue
        if named:
            return named if with_names else [g for _, g in named]
    if parse_err is not None:
        raise ValueError(f'Could not read the WKT in the CSV: {parse_err}')
    raise ValueError('No WKT column found (expected a "WKT"/"geometry" column, or WKT '
                     'text in a column).')


def _polygons_from_named(named):
    """[(name|None, geometry)] -> ([(name, Polygon)], [(name, problem)]). Each Polygon is
    one survey area; a MultiPolygon is split into its parts ('name (1)', 'name (2)', ...);
    invalid rings are repaired with buffer(0); unnamed ones become 'Polygon <n>'."""
    items, bad = [], []
    for k, (name, g) in enumerate(named, 1):
        base = (name or '').strip() or None
        if g.geom_type == 'Polygon':
            parts = [g]
        elif g.geom_type in ('MultiPolygon', 'GeometryCollection'):
            parts = [x for x in g.geoms if x.geom_type == 'Polygon']
        else:
            bad.append((base or f'Row {k}', f'not a polygon ({g.geom_type})'))
            continue
        for j, poly in enumerate(parts, 1):
            if not poly.is_valid:
                poly = poly.buffer(0)
                if poly.geom_type == 'MultiPolygon':        # repaired into pieces: keep the
                    poly = max(poly.geoms, key=lambda q: q.area)   # main one
            nm = (f'{base} ({j})' if base and len(parts) > 1 else base) \
                or f'Polygon {len(items) + 1}'
            if poly.is_empty or poly.geom_type != 'Polygon':
                bad.append((nm, 'empty or invalid polygon'))
            else:
                items.append((nm, poly))
    return items, bad


# Uploaded-line pieces shorter than this (m) after cropping to a polygon are dropped.
_MIN_PIECE_M = 10.0


def _clip_segment(a, b, poly, min_len):
    """The parts of the straight segment a→b inside `poly`, each as ((x, y, z), (x, y, z))
    in a→b direction (z None for 2D input; for 3D, interpolated along the segment at the
    cut points). A concave polygon can cut one segment into several parts; parts shorter
    than `min_len` (map units) are dropped."""
    seg = LineString([(a[0], a[1]), (b[0], b[1])])
    inter = seg.intersection(poly)
    out = []
    for g in getattr(inter, 'geoms', [inter]):
        if g.is_empty or g.geom_type != 'LineString' or g.length < min_len:
            continue
        p0, p1 = g.coords[0], g.coords[-1]
        if math.hypot(p0[0] - a[0], p0[1] - a[1]) > math.hypot(p1[0] - a[0], p1[1] - a[1]):
            p0, p1 = p1, p0                              # keep the a→b direction

        def z_at(q):
            if len(a) < 3 or len(b) < 3:
                return None
            t = seg.project(Point(q[0], q[1]), normalized=True)
            return float(a[2]) + (float(b[2]) - float(a[2])) * t
        out.append(((float(p0[0]), float(p0[1]), z_at(p0)),
                    (float(p1[0]), float(p1[1]), z_at(p1))))
    return out


def _route_pass_pts(route):
    """[(pass_id, [(x, y) start, (x, y) end])] of a route — the selectable passes."""
    groups, order = {}, []
    for w in route:
        pid = w.get('pass_id')
        if pid not in groups:
            order.append(pid)
        groups.setdefault(pid, []).append((w['x'], w['y']))
    return [(pid, [groups[pid][0], groups[pid][-1]]) for pid in order
            if len(groups[pid]) >= 2]


# Per-polygon working state: what each loaded polygon remembers while you work on another.
_SLOT_FIELDS = ('base_result', 'result', 'survey_route', '_edit_undo', '_edit_redo',
                '_route_active', '_route_auto_alt', 'loaded_route')


def _empty_slot():
    return {'base_result': None, 'result': None, 'survey_route': [], '_edit_undo': [],
            '_edit_redo': [], '_route_active': False, '_route_auto_alt': False,
            'loaded_route': None}


def _hr():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setFrameShadow(QFrame.Sunken)
    return line


class CollapsibleSection(QWidget):
    """A disclosure: a click-to-toggle header (▸/▾) over a content area that hides
    the rarely-touched params so the routine knobs stay uncluttered up top."""

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.toggle = QToolButton()
        self.toggle.setObjectName('disclosure')
        self.toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.toggle.setArrowType(Qt.RightArrow)
        self.toggle.setText(title)
        self.toggle.setCheckable(True)
        self.toggle.setChecked(False)
        self.toggle.setCursor(Qt.PointingHandCursor)
        self._content = QWidget()
        self._content.setVisible(False)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0); lay.setSpacing(2)
        lay.addWidget(self.toggle); lay.addWidget(self._content)
        self.toggle.toggled.connect(self._on_toggle)

    def _on_toggle(self, on):
        self.toggle.setArrowType(Qt.DownArrow if on else Qt.RightArrow)
        self._content.setVisible(on)

    def set_content_layout(self, layout):
        self._content.setLayout(layout)

    def set_expanded(self, on):
        self.toggle.setChecked(on)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('LiDAR Drone Route Planner — desktop')
        self.resize(1280, 820)

        self.dtm = None
        self.dtm_path = None
        self.is_geo = True
        self.base_result = None          # survey result (route + density estimate)
        self.survey_route = []           # survey waypoints
        self._edit_undo = []             # snapshots of survey_route for undo (route edits)
        self._edit_redo = []             # snapshots for redo
        self.result = None               # current result shown in the UI
        self.drawn_polygon = None        # shapely Polygon drawn on the map
        self._map_focused = False        # map zoomed to the AOI (vs full-extent overview)
        self.loaded_route = None         # operator route from .wkt (list of waypoints), for pass selection
        self._route_auto_alt = False     # loaded route had no Z (altitudes auto-assigned -> band on confirm)
        self._route_active = False       # a confirmed uploaded route is active (Compute re-estimates it)
        self._pending_aoi = None         # AOI loaded before a DTM — crop to it on the next Open DTM
        self._aoi_set = []               # several polygons from one file: [{'name','poly','slot'}]
        self._aoi_active = -1            # index of the polygon being worked on
        self._pending_aoi_set = None     # several polygons loaded before a DTM

        self._build_menu()
        self._build_body()
        self._update_scan_freq()                 # derive the initial scan freq
        self._load_settings()                    # restore last-used params + η
        self._update_workflow()                  # seed the ✓ checklist state
        self.lbl_summary.setText(self._empty_summary_html())   # guided empty state
        self._progress = QProgressBar()
        self._progress.setRange(0, 0)            # indeterminate "busy" bar
        self._progress.setMaximumWidth(170)
        self._progress.hide()
        self.statusBar().addPermanentWidget(self._progress)
        self.statusBar().showMessage('Open a DTM to begin.')

    def _set_busy(self, on, msg=None):
        """Show/hide the busy bar + wait cursor and repaint so it's visible
        before a blocking call."""
        self._progress.setVisible(on)
        if msg:
            self.statusBar().showMessage(msg)
        if on:
            QApplication.setOverrideCursor(Qt.WaitCursor)
        else:
            QApplication.restoreOverrideCursor()
        QApplication.processEvents()

    # ---------------------------------------------------------------- UI build
    def _build_menu(self):
        m = self.menuBar().addMenu('&File')
        a_dtm = QAction('Open DTM…', self); a_dtm.triggered.connect(self._open_dtm)
        a_wkt = QAction('Load polygon (.wkt/.csv)…', self)
        a_wkt.triggered.connect(self._load_wkt_aoi)
        a_route = QAction('Load route (.wkt/.csv)…', self)
        a_route.triggered.connect(self._load_wkt_route)
        self.act_save_route = QAction('Save route (.wkt)…', self)
        self.act_save_route.setEnabled(False)
        self.act_save_route.triggered.connect(self._save_wkt_route)
        a_quit = QAction('Quit', self); a_quit.triggered.connect(self.close)
        m.addAction(a_dtm); m.addAction(a_wkt); m.addAction(a_route)
        m.addAction(self.act_save_route)
        m.addSeparator(); m.addAction(a_quit)

        # Undo/redo of route edits — keyboard only (Ctrl+Z / Ctrl+Y), no menu. The
        # actions are added to the window so their shortcuts fire application-wide.
        self.act_undo = QAction('Undo route edit', self)
        self.act_undo.setShortcut(QKeySequence.Undo)          # Ctrl+Z
        self.act_undo.triggered.connect(self._undo_edit)
        self.act_redo = QAction('Redo route edit', self)
        self.act_redo.setShortcuts([QKeySequence.Redo,        # Ctrl+Y / Ctrl+Shift+Z
                                    QKeySequence('Ctrl+Shift+Z')])
        self.act_redo.triggered.connect(self._redo_edit)
        self.addAction(self.act_undo); self.addAction(self.act_redo)
        self._update_undo_actions()

        mv = self.menuBar().addMenu('&View')
        self.act_profile = QAction('Elevation profile', self, checkable=True)
        self.act_profile.setChecked(False)
        self.act_profile.setShortcut('Ctrl+E')
        self.act_profile.toggled.connect(self._toggle_profile)
        mv.addAction(self.act_profile)

    def _build_body(self):
        top = QSplitter(Qt.Horizontal)
        sidebar = self._build_sidebar()                # params (left)
        top.addWidget(sidebar)
        top.addWidget(self._build_map())               # map (center)
        top.addWidget(self._build_summary())           # results (right)
        top.setStretchFactor(0, 0)
        top.setStretchFactor(1, 1)
        top.setStretchFactor(2, 0)
        # Only the map flexes; the side panels can't be collapsed and (thanks to their
        # scroll areas tracking content width, see _build_sidebar/_build_summary) can't
        # be squeezed narrower than their contents — so a change in one panel steals
        # width from the map, never from the other panel.
        top.setChildrenCollapsible(False)

        from .profile import ProfilePanel
        self.profile_panel = ProfilePanel()            # full-width strip below
        self.profile_panel.setVisible(False)           # opened on demand via View menu
        self.profile_panel.passClicked.connect(self._highlight_pass_on_map)
        self.profile_panel.passAltitudeChanged.connect(self._on_pass_altitude_changed)

        outer = QSplitter(Qt.Vertical)
        outer.addWidget(top)
        outer.addWidget(self.profile_panel)
        outer.setStretchFactor(0, 1)
        outer.setStretchFactor(1, 0)
        outer.setCollapsible(1, True)                  # drag-collapse the profile
        self.body_splitter = outer
        self.setCentralWidget(outer)

    def _build_sidebar(self):
        panel = QWidget()
        v = QVBoxLayout(panel)
        v.setSpacing(10)
        v.setContentsMargins(12, 12, 12, 12)

        # ── Data ── (workflow step ①; titles get a ✓ via _update_workflow)
        self.gb_data = gb_data = QGroupBox('① Data')
        dl = QVBoxLayout(gb_data)
        self.lbl_dtm = QLabel('DTM: (none)'); self.lbl_dtm.setWordWrap(True)
        dtm_row = QHBoxLayout()
        b_dtm = QPushButton('Open DTM…'); b_dtm.clicked.connect(self._open_dtm)
        b_dtm_clear = QPushButton('Clear'); b_dtm_clear.clicked.connect(self._clear_dtm)
        dtm_row.addWidget(b_dtm); dtm_row.addWidget(b_dtm_clear)
        dl.addWidget(self.lbl_dtm); dl.addLayout(dtm_row)
        v.addWidget(gb_data)

        # ── AOI ── (workflow step ②)
        self.gb_aoi = gb_aoi = QGroupBox('② Survey area')
        al = QVBoxLayout(gb_aoi)
        self.lbl_aoi = QLabel('Draw a polygon on the map.')
        self.lbl_aoi.setWordWrap(True); self.lbl_aoi.setStyleSheet('color:#888;')
        b_enter_aoi = QPushButton('Enter coordinates…')
        b_enter_aoi.clicked.connect(self._enter_aoi_coords)
        b_clear_aoi = QPushButton('Clear drawn polygon')
        b_clear_aoi.clicked.connect(self._clear_aoi)
        # several polygons from one file: pick which one to work on (hidden otherwise)
        self.aoi_row = QWidget()
        ar = QHBoxLayout(self.aoi_row); ar.setContentsMargins(0, 0, 0, 0)
        ar.addWidget(QLabel('Working on:'))
        self.cmb_aoi = QComboBox()
        self.cmb_aoi.setToolTip('Polygons from the loaded file. Each keeps its own route, '
                                'results and edits while you work on another.')
        self.cmb_aoi.currentIndexChanged.connect(self._switch_aoi)
        ar.addWidget(self.cmb_aoi, 1)
        self.aoi_row.setVisible(False)
        al.addWidget(self.lbl_aoi); al.addWidget(self.aoi_row)
        al.addWidget(b_enter_aoi); al.addWidget(b_clear_aoi)
        v.addWidget(gb_aoi)

        # ── Flight ── (workflow step ③)
        gb_flight = QGroupBox('③ Flight')
        fl = QFormLayout(gb_flight)
        fl.setRowWrapPolicy(QFormLayout.WrapLongRows)          # field drops under label
        fl.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)  # when the pane is narrow
        self.sp_alt = self._dspin(1, 1000, 100, ' m', 5)
        self.sp_overlap = self._dspin(20, 50, 20, ' %', 1)
        fl.addRow('Altitude AGL', self.sp_alt)
        fl.addRow('Overlap', self.sp_overlap)
        v.addWidget(gb_flight)

        # ── Scanner & density ── (workflow step ④) — routine knobs up top, the
        # set-once scanner internals tucked behind an "Advanced" disclosure.
        gb_scan = QGroupBox('④ Scanner & density')
        scl = QFormLayout(gb_scan)
        scl.setRowWrapPolicy(QFormLayout.WrapLongRows)
        scl.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        self.sp_minpts = QSpinBox(); self.sp_minpts.setRange(1, 100000); self.sp_minpts.setValue(100)
        self.sp_speed = self._dspin(0.1, 50, 6.0, ' m/s', 0.5)
        self.cmb_pulse = QComboBox()
        for f in PULSE_FREQS:
            self.cmb_pulse.addItem(f'{f:,}', f)
        self.cmb_pulse.setCurrentText('600,000')
        # Scan freq is DERIVED for a square point pattern from AGL/speed/PRR/FOV,
        # not entered: read-only + locked, and recomputed whenever those change.
        self.sp_scanfreq = self._dspin(1, 5000, 224.4, ' Hz', 10)
        self.sp_scanfreq.setReadOnly(True)
        self.sp_scanfreq.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.sp_scanfreq.setToolTip(
            'Derived for a square (isotropic) point pattern from AGL, speed, pulse '
            'rate and FOV, clamped to the mirror’s 50–400 lines/s (locked).')
        self.cmb_pulse.currentIndexChanged.connect(self._update_scan_freq)
        self.sp_alt.valueChanged.connect(self._update_scan_freq)
        self.sp_speed.valueChanged.connect(self._update_scan_freq)
        scl.addRow('Min points / m²', self.sp_minpts)
        scl.addRow('Drone speed', self.sp_speed)

        self.scan_advanced = CollapsibleSection('Advanced')
        adv = QFormLayout()
        adv.setContentsMargins(0, 2, 0, 0)
        adv.setRowWrapPolicy(QFormLayout.WrapLongRows)
        adv.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        adv.addRow('Pulse freq', self.cmb_pulse)
        adv.addRow('Scan freq (auto)', self.sp_scanfreq)
        adv.addRow(QLabel('FOV fixed at 100° (±50°)'))
        self.scan_advanced.set_content_layout(adv)
        scl.addRow(self.scan_advanced)
        v.addWidget(gb_scan)

        self.btn_compute = QPushButton('Compute Route')
        self.btn_compute.setObjectName('primary')
        self.btn_compute.setEnabled(False)
        self.btn_compute.clicked.connect(self._compute)
        v.addWidget(self.btn_compute)

        gb_exp = QGroupBox('Export lines'); el = QHBoxLayout(gb_exp)
        self.btn_geojson = QPushButton('GeoJSON'); self.btn_geojson.setEnabled(False)
        self.btn_geojson.clicked.connect(self._export_geojson)
        self.btn_csv = QPushButton('CSV'); self.btn_csv.setEnabled(False)
        self.btn_csv.clicked.connect(self._export_csv)
        el.addWidget(self.btn_geojson); el.addWidget(self.btn_csv)
        v.addWidget(gb_exp)
        v.addStretch(1)

        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(panel)
        # Adaptive: form rows wrap the field under the label when narrow, so the panel
        # can compress a long way; the floor is the content's OWN minimum width (which
        # scales with the font/DPI, not a hard-coded number) plus the scrollbar. With
        # the horizontal scrollbar off and a non-collapsible splitter, the pane is
        # exactly as wide as its contents need — never clipping, never scrolling
        # sideways — and the map absorbs any width the results panel gives up.
        self.sidebar_scroll = scroll
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sb_w = scroll.verticalScrollBar().sizeHint().width()
        # Floor = content's own minimum (DPI-aware) + scrollbar + a little comfort
        # padding. The 3-pane splitter opens each side panel at its minimum, so this
        # padding is also the sidebar's opening width — a touch roomier than a tight fit.
        # Measure with the Advanced disclosure EXPANDED so its (wider) rows are counted:
        # a hidden child contributes nothing to the size hint, so measuring collapsed
        # would let the floor clip the advanced rows when the user opens them later.
        self.scan_advanced.set_expanded(True)
        scroll.setMinimumWidth(panel.minimumSizeHint().width() + sb_w + 44)
        self.scan_advanced.set_expanded(False)
        return scroll

    def _dspin(self, lo, hi, val, suffix, step):
        s = QDoubleSpinBox(); s.setRange(lo, hi); s.setValue(val)
        s.setSuffix(suffix); s.setSingleStep(step)
        return s

    def _update_scan_freq(self):
        """Derive the scan (mirror) line rate for an isotropic 'square' point pattern
        from the current flight geometry — scan = sqrt(v·PRR / (2·AGL·tanθ)), clamped
        to the datasheet 50–400 lines/s. Tracks AGL, speed and pulse rate; replaces
        the old fixed nominal anchor."""
        half = 100.0 / 2.0                       # FOV fixed at 100° (±50°)
        self.sp_scanfreq.setValue(scan_lines_for_square_pattern(
            self.cmb_pulse.currentData(), self.sp_alt.value(), half,
            self.sp_speed.value()))

    # -------------------------------------------------------- settings persistence
    def _settings(self):
        # per-user store (Windows registry HKCU); survives restarts on the air-gapped
        # machine. Only planning PARAMETERS + calibration are persisted, not the DTM/
        # AOI mission state (which belongs to a mission, not to preferences).
        return QSettings('AutoRoutePlanning', 'RoutePlanner')

    def _save_settings(self):
        s = self._settings()
        s.setValue('flight/agl', self.sp_alt.value())
        s.setValue('flight/overlap', self.sp_overlap.value())
        s.setValue('scan/min_points', self.sp_minpts.value())
        s.setValue('scan/speed', self.sp_speed.value())
        s.setValue('scan/pulse_freq', self.cmb_pulse.currentData())
        s.setValue('window/geometry', self.saveGeometry())

    def _load_settings(self):
        s = self._settings()
        self.sp_alt.setValue(s.value('flight/agl', self.sp_alt.value(), type=float))
        self.sp_overlap.setValue(
            s.value('flight/overlap', self.sp_overlap.value(), type=float))
        self.sp_minpts.setValue(
            s.value('scan/min_points', self.sp_minpts.value(), type=int))
        self.sp_speed.setValue(s.value('scan/speed', self.sp_speed.value(), type=float))
        pf = s.value('scan/pulse_freq', self.cmb_pulse.currentData(), type=int)
        idx = self.cmb_pulse.findData(pf)
        if idx >= 0:
            self.cmb_pulse.setCurrentIndex(idx)           # re-derives the scan freq
        geo = s.value('window/geometry')
        if geo is not None:
            self.restoreGeometry(geo)

    def closeEvent(self, event):
        self._save_settings()
        super().closeEvent(event)

    def _build_map(self):
        # Native Qt map canvas (offline) — DTM relief, draw the AOI here.
        if CanvasMap is not None:
            self.mapview = CanvasMap()
            self.mapview.polygonDrawn.connect(self._on_polygon_drawn)
            self.mapview.passDrawn.connect(self._on_pass_drawn)
            self.mapview.focusToggled.connect(self._on_map_focus_toggled)
            self.mapview.passesConfirmed.connect(self._confirm_selected_passes)
            self.mapview.passEditDelete.connect(self._on_edit_delete)
            self.mapview.passEditGeom.connect(self._on_edit_geom)
            return self.mapview
        self.mapview = None
        return self._stub('🗺  Map canvas failed to load.\n' + (_MAPVIEW_ERR or ''))

    def _build_summary(self):
        panel = QWidget(); sv = QVBoxLayout(panel)
        title = QLabel('<b>Results</b>')

        self.lbl_summary = QLabel('Compute a route to see results.')
        self.lbl_summary.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.lbl_summary.setTextFormat(Qt.RichText)
        self.lbl_summary.setWordWrap(True)
        sv.addWidget(title)
        sv.addWidget(self.lbl_summary); sv.addStretch(1)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(panel)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)  # wrap, don't scroll sideways
        scroll.setMinimumWidth(260)
        return scroll

    def _update_workflow(self):
        """Flip the ✓ on the numbered workflow groups as each step is satisfied, so
        the sidebar reads as a checklist of where you are."""
        self.gb_data.setTitle('① Data ✓' if self.dtm is not None else '① Data')
        self.gb_aoi.setTitle('② Survey area ✓' if self.drawn_polygon is not None
                             else '② Survey area')

    def _empty_summary_html(self):
        """A guided placeholder for the results panel before a route exists: the
        three gating steps, with done ones checked and the next one highlighted."""
        steps = [('Load a DTM', self.dtm is not None),
                 ('Draw a survey area', self.drawn_polygon is not None),
                 ('Compute the route', False)]
        cur = next((i for i, (_, done) in enumerate(steps) if not done), len(steps))
        nums = ['①', '②', '③']
        out = ['<div style="color:#8b96a3;">Get started</div>',
               '<table cellspacing=5 style="margin-top:6px;">']
        for i, (name, done) in enumerate(steps):
            if done:
                mark, color = '✓', '#4f9d7a'
            elif i == cur:
                mark, color = '→', '#e3e6e9'
            else:
                mark, color = '', '#74808c'
            out.append(f'<tr><td style="color:{color};">{nums[i]}</td>'
                       f'<td style="color:{color};">{name}</td>'
                       f'<td style="color:{color};">{mark}</td></tr>')
        out.append('</table>')
        return ''.join(out)

    def _stub(self, text):
        w = QWidget(); l = QVBoxLayout(w)
        lab = QLabel(text); lab.setAlignment(Qt.AlignCenter)
        lab.setStyleSheet('color:#888; font-size:15px;')
        l.addStretch(1); l.addWidget(lab); l.addStretch(1)
        return w

    # ---------------------------------------------------------------- actions
    def _open_dtm(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Open DTM', '', 'GeoTIFF (*.tif *.tiff);;All files (*)')
        if not path:
            return
        if not self._pending_aoi_set and not self._confirm_discard_set():
            return
        try:
            self.dtm = load_dtm(path)
        except Exception as e:
            QMessageBox.critical(self, 'DTM error', str(e)); return
        self.dtm_path = path
        crs = self.dtm.src.crs
        self.is_geo = crs.is_geographic if crs else True
        w, h = self.dtm.src.width, self.dtm.src.height     # native size (array may be windowed)
        self.lbl_dtm.setText(f'DTM: {path}\n{w}×{h} px · CRS {crs}')
        self.loaded_route = None; self._set_route_mode(False)
        self._clear_results()
        pending_set, self._pending_aoi_set = self._pending_aoi_set, None
        self._clear_aoi_set()
        if pending_set:                           # several polygons were loaded first
            if not self._start_aoi_set(pending_set):
                self._open_dtm_full()
        elif self._pending_aoi is not None:
            poly, self._pending_aoi = self._pending_aoi, None
            self._open_dtm_cropped(poly)
        else:
            self._open_dtm_full()

    def _open_dtm_full(self):
        """Show the whole-extent overview (default DTM view)."""
        self.drawn_polygon = None
        self.btn_compute.setEnabled(False)
        self.lbl_aoi.setText('Draw a polygon on the map.')
        self._refresh_map()
        self.statusBar().showMessage('DTM loaded. Draw an polygon on the map, then Compute.')

    def _open_dtm_cropped(self, poly):
        """Render the just-opened DTM cropped to a pending AOI instead of the whole extent
        — reads only that window, so a giga-pixel DTM opens without the slow full-extent
        scan. Falls back to the full view if the AOI doesn't fit the DTM."""
        b = self.dtm.src.bounds
        area = polygon_area_m2(poly, self.is_geo)
        if not poly.intersects(shapely_box(b.left, b.bottom, b.right, b.top)):
            QMessageBox.warning(
                self, 'Polygon', 'The loaded polygon does not overlap this DTM — showing the full '
                'extent. Its coordinates must be in the DTM CRS.')
            self._open_dtm_full(); return
        if area > MAX_AOI_M2:
            QMessageBox.warning(
                self, 'Polygon', f'The loaded polygon is {area / 1e6:.2f} km² — over the '
                f'{MAX_AOI_M2 / 1e6:.0f} km² limit; showing the full extent.')
            self._open_dtm_full(); return
        self.drawn_polygon = poly
        self._map_focused = True
        self._set_busy(True, 'Cropping the DTM to the polygon…')
        try:
            self.mapview.set_dtm(self.dtm, self.dtm_path, focus_polygon=poly)
            self.mapview.set_aoi_polygon(list(poly.exterior.coords))
        finally:
            self._set_busy(False)
        self._sync_focus_button()
        self.btn_compute.setEnabled(True)
        self.lbl_aoi.setText('✓ polygon loaded — DTM cropped to it (map focused).')
        self.statusBar().showMessage(
            'DTM opened cropped to the polygon — full-extent render skipped.')

    def _clear_dtm(self):
        """Drop the loaded DTM (and the AOI/results that depend on it) and blank the
        map."""
        if not self._confirm_discard_set():
            return
        self._clear_aoi_set()
        self.dtm = None; self.dtm_path = None
        self.is_geo = True
        self.drawn_polygon = None
        self.loaded_route = None; self._set_route_mode(False)
        self.lbl_dtm.setText('DTM: (none)')
        self.lbl_aoi.setText('Draw a polygon on the map.')
        self.btn_compute.setEnabled(False)
        self._clear_results()
        if self.mapview is not None:
            self.mapview.clear()
        self.statusBar().showMessage('Open a DTM to begin.')

    def _enter_aoi_coords(self):
        """Set the AOI from manually-typed vertices instead of drawing on the map."""
        if self.dtm is None:
            QMessageBox.information(self, 'Polygon', 'Open a DTM first.'); return
        from PySide6.QtWidgets import QDialog, QPlainTextEdit, QDialogButtonBox
        dlg = QDialog(self); dlg.setWindowTitle('Enter polygon')
        lay = QVBoxLayout(dlg)
        info = QLabel('One vertex per line as  <b>easting, northing</b>  (UTM 36N, '
                      'matching the map readout). At least 3 vertices; the polygon is '
                      'closed automatically.')
        info.setWordWrap(True); info.setTextFormat(Qt.RichText)
        txt = QPlainTextEdit()
        txt.setPlaceholderText('500000, 3450000\n500500, 3450000\n'
                               '500500, 3450500\n500000, 3450500')
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept); bb.rejected.connect(dlg.reject)
        lay.addWidget(info); lay.addWidget(txt); lay.addWidget(bb)
        dlg.resize(360, 320)
        if dlg.exec() != QDialog.Accepted:
            return
        coords = self._parse_coords(txt.toPlainText())
        if coords is None:
            return
        geom = {'type': 'Polygon', 'coordinates': [coords + [coords[0]]]}
        self.mapview.set_aoi_polygon(coords)
        self._on_polygon_drawn(geom)
        self.lbl_aoi.setText('✓ polygon set from entered coordinates.')

    def _parse_coords(self, text):
        """Parse 'easting, northing' (UTM 36N) lines into a list of [lon, lat] vertices
        in the DTM frame, or None."""
        crs = self.dtm.src.crs if self.dtm is not None else None
        coords = []
        for ln in text.splitlines():
            ln = ln.strip()
            if not ln:
                continue
            parts = ln.replace(',', ' ').split()
            if len(parts) < 2:
                QMessageBox.warning(self, 'Polygon',
                                    f'Bad line: "{ln}"\nUse: easting, northing')
                return None
            try:
                easting, northing = float(parts[0]), float(parts[1])
            except ValueError:
                QMessageBox.warning(self, 'Polygon', f'Not numbers: "{ln}"')
                return None
            lon, lat = from_utm(crs, easting, northing)
            coords.append([lon, lat])
        if len(coords) < 3:
            QMessageBox.warning(self, 'Polygon', 'Enter at least 3 vertices.')
            return None
        return coords

    def _load_wkt_aoi(self):
        """Load AOI polygon(s) from a .wkt file or a .csv with a WKT column (coordinates in
        the DTM's CRS). ONE polygon becomes the survey area (map focused on it). SEVERAL
        (e.g. one per CSV row) are all kept and you choose which to work on — each keeps its
        own route, results and edits when you switch. Without a DTM, remember them and
        crop to the (first) polygon on the next Open DTM."""
        path, _ = QFileDialog.getOpenFileName(
            self, 'Load polygon', '',
            'WKT or CSV (*.wkt *.csv *.txt);;All files (*)')
        if not path:
            return
        try:
            named = _read_wkt_geoms(path, with_names=True)
        except Exception as e:
            QMessageBox.critical(self, 'Polygon', f'Could not read the file:\n{e}'); return
        items, bad = _polygons_from_named(named)
        if not items:
            QMessageBox.warning(self, 'Polygon', 'No polygon found in the file.'); return
        if not self._confirm_discard_set():
            return
        if len(items) > 1:
            if self.dtm is None:
                self._pending_aoi_set, self._pending_aoi = items, None
                self.lbl_aoi.setText(f'{len(items)} polygons loaded — open a DTM to start '
                                     f'working on them.')
                self.statusBar().showMessage(
                    f'{len(items)} polygons stored. File > Open DTM to start on the first one.')
                return
            self._clear_aoi_set()
            self._start_aoi_set(items, bad)
            return
        # ── a single polygon ──
        self._clear_aoi_set(); self._pending_aoi_set = None
        self._warn_unusable(bad)
        poly = items[0][1]
        if self.dtm is None:
            # No DTM yet — remember it; Open DTM will crop to it (CRS is checked then).
            self._pending_aoi = poly
            self.lbl_aoi.setText('Polygon loaded — open a DTM to crop straight to it.')
            self.statusBar().showMessage(
                'Polygon stored. File > Open DTM to render only this area (skips the full map).')
            return
        # WKT carries no CRS, so its coordinates must be in the DTM's frame; require overlap.
        b = self.dtm.src.bounds
        if not poly.intersects(shapely_box(b.left, b.bottom, b.right, b.top)):
            QMessageBox.warning(
                self, 'Polygon', 'The polygon does not overlap the DTM.\nIts coordinates '
                'must be in the same CRS as the DTM.'); return
        # Reuse the drawn-polygon path: applies the area cap, sets state, enables Compute.
        ext = [list(c) for c in poly.exterior.coords]
        self._on_polygon_drawn({'type': 'Polygon', 'coordinates': [ext]})
        if self.drawn_polygon is None:                   # rejected by the area cap
            return
        self._render_map(focus=True)                     # focus the map on the AOI
        self.lbl_aoi.setText('✓ polygon loaded from WKT — map focused at native resolution.')
        self.statusBar().showMessage(
            f'Polygon from {os.path.basename(path)}; map focused on the area.')

    def _load_wkt_route(self):
        """Load an operator-built route from a .wkt or a .csv with a WKT column (each
        LINESTRING = one pass). If vertices carry Z, that's the flight altitude; a 2D
        route gets automatic altitudes the same way the planner does (mean terrain + the
        sidebar AGL, floored to clear the pass's peak). Draw the passes; the operator
        clicks the relevant ones and Confirm runs the density estimate over the CURRENT
        AOI exactly like an auto-computed plan. Requires an AOI first; coords in DTM CRS."""
        if self.dtm is None:
            QMessageBox.information(self, 'Route', 'Open a DTM first.'); return
        if self.drawn_polygon is None:
            QMessageBox.information(
                self, 'Route', 'Set a polygon first (draw one, or Load polygon), then load a '
                'route to estimate inside it.'); return
        path, _ = QFileDialog.getOpenFileName(
            self, 'Load route', '', 'WKT or CSV (*.wkt *.csv *.txt);;All files (*)')
        if not path:
            return
        try:
            geoms = _read_wkt_geoms(path)
        except Exception as e:
            QMessageBox.critical(self, 'Route', f'Could not read the file:\n{e}'); return
        # Collect every pass: LINESTRINGs directly, and the parts of MULTILINESTRINGs
        # (a route CSV may put one MULTILINESTRING in a row, or one LINESTRING per row).
        lines = []
        for g in geoms:
            if g.geom_type == 'LineString':
                lines.append(g)
            elif g.geom_type == 'MultiLineString':
                lines.extend(g.geoms)
            elif g.geom_type == 'GeometryCollection':
                lines.extend(x for x in g.geoms if x.geom_type == 'LineString')
        lines = [ln for ln in lines if not ln.is_empty and len(ln.coords) >= 2]
        if not lines:
            QMessageBox.warning(self, 'Route', 'No line passes found in the file.')
            return
        # Each straight segment (vertex A -> vertex B) is one selectable pass, CROPPED to
        # the polygon: the part inside is kept, the rest dropped. With several polygons
        # loaded, every polygon gets the parts of the lines inside it — the current one
        # shows them now, the others keep theirs for when you switch. 3D -> vertex Z is
        # the flight altitude (interpolated at a cut); 2D -> automatic altitude per pass
        # (mean terrain + sidebar AGL, floored to clear the pass peak), as the planner does.
        have_z = all(ln.has_z for ln in lines)
        params = self._params()
        to_m = _LAT_M if self.is_geo else 1.0
        step_map = params.step_m / to_m
        res_map = min(abs(self.dtm.src.res[0]), abs(self.dtm.src.res[1]))
        elev_step = min(step_map, res_map)
        multi = len(self._aoi_set) > 1
        targets = list(range(len(self._aoi_set))) if multi else [None]
        polys = {k: (self._aoi_set[k]['poly'] if k is not None else self.drawn_polygon)
                 for k in targets}
        active = self._aoi_active if multi else None
        polys[active] = self.drawn_polygon               # the live (maybe redrawn) one
        pieces = {k: [] for k in targets}
        outside = 0
        for ln in lines:
            cs = list(ln.coords)
            for a, b in zip(cs, cs[1:]):
                hit = False
                for k in targets:
                    got = _clip_segment(a, b, polys[k], _MIN_PIECE_M / to_m)
                    if got:
                        pieces[k].extend(got); hit = True
                if not hit:
                    outside += 1

        def build(parts):
            route, skipped, pid = [], 0, 0
            for (x0, y0, z0), (x1, y1, z1) in parts:
                seg = [(x0, y0), (x1, y1)]
                if have_z:
                    zs, td = [z0, z1], None
                else:
                    zc = _pass_altitude(self.dtm, _densify_polyline(seg, step_map),
                                        params.altitude_m, step_map, elev_step,
                                        params.min_peak_clearance_m)
                    if math.isnan(zc):                   # no valid terrain under this pass
                        skipped += 1; continue
                    zs, td = [zc, zc], params.altitude_m
                pid += 1
                for (x, y), z in zip(seg, zs):
                    route.append({'x': x, 'y': y, 'z': z, 'pass_id': pid,
                                  'target_distance': td})
            return route, skipped

        built = {k: build(pieces[k]) for k in targets}
        route, skipped = built[active]
        others = {k: r for k, (r, _) in built.items() if k != active and r}
        if not route and not others:
            QMessageBox.warning(
                self, 'Route', 'No part of the lines lies inside the polygon'
                + ('s' if multi else '') + '.\nCoordinates must be in the same CRS as '
                'the DTM.'); return
        # Lines for OTHER polygons replace whatever route those polygons have — ask first.
        busy = [self._aoi_set[k]['name'] for k in others if self._slot_has_route(k)]
        if busy and QMessageBox.question(
                self, 'Route', f'This file also has lines for {len(busy)} other polygon(s) '
                f'that already have a route ({", ".join(busy[:4])}'
                f'{"…" if len(busy) > 4 else ""}).\n\nReplace their routes with these lines?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        for k, r in others.items():                      # waiting for that polygon
            slot = _empty_slot()
            slot['loaded_route'] = r
            slot['_route_auto_alt'] = not have_z
            self._aoi_set[k]['slot'] = slot
        other_note = (f'; lines kept for {len(others)} other polygon(s): '
                      + ', '.join(self._aoi_set[k]['name'] for k in others)) if others else ''
        out_note = f'; {outside} segment(s) outside every polygon ignored' if outside else ''
        skip_note = f'; {skipped} skipped — off terrain' if skipped else ''
        alt_note = ('altitude from Z' if have_z
                    else f'auto altitude at {params.altitude_m:.0f} m AGL')
        if not route:                                    # nothing for the current polygon
            self._refresh_aoi_combo()
            self.statusBar().showMessage(
                f'No lines inside this polygon{other_note}{out_note}.')
            return
        # Keep the AOI; drop any previous route/estimate. Show the AOI + passes and let
        # the operator pick the relevant passes.
        self.loaded_route = route
        self._route_auto_alt = not have_z    # band the altitudes on confirm (2D route only)
        self._clear_results()
        self._set_route_mode(False)          # active again once passes are confirmed
        self._render_map(focus=True)
        self._show_loaded_passes()
        self._refresh_aoi_combo()
        self.statusBar().showMessage(
            f'Route from {os.path.basename(path)}: {len(_route_pass_pts(route))} passes '
            f'inside the polygon (cropped to it), {alt_note}{skip_note}{other_note}'
            f'{out_note}. Click the relevant ones, then Confirm.')

    def _show_loaded_passes(self):
        """Show the uploaded passes waiting to be selected + confirmed for this polygon."""
        if self.mapview is None or not self.loaded_route:
            return
        self.mapview.show_route_passes(_route_pass_pts(self.loaded_route))
        self.lbl_aoi.setText('Route loaded — click the passes relevant for point-cloud '
                             'production, then ✓ Confirm passes.')

    def _confirm_selected_passes(self):
        """Estimate the selected passes over the CURRENT AOI, then behave exactly like an
        auto-computed plan (same result, summary, overlays, and enabled actions)."""
        if not self.loaded_route or self.drawn_polygon is None:
            return
        ids = set(self.mapview.selected_pass_ids())
        if not ids:
            QMessageBox.information(
                self, 'Passes', 'Click at least one pass on the map to select it, '
                'then Confirm.'); return
        # Copy the waypoints (banding rewrites z in place) so the stored route stays
        # intact for re-selection.
        route = [dict(wp) for wp in self.loaded_route if wp['pass_id'] in ids]
        if self._route_auto_alt:
            # Auto-altitude route: group consecutive passes onto shared heights the same
            # way the planner does (only raises; fewer z-calibrations). 3D routes keep
            # the operator's altitudes untouched.
            route = band_pass_altitudes(route, self.dtm, self._params().altitude_m,
                                        is_geo=self.is_geo)
        self.mapview.clear_route_passes()            # leave selection mode
        self.lbl_aoi.setText(f'✓ polygon + {len(ids)} selected passes (uploaded route).')
        self._estimate_uploaded_route(route)

    def _estimate_uploaded_route(self, route):
        """Estimate a fixed (uploaded/selected) route over the current AOI with the
        CURRENT params, then present it exactly like an auto-computed plan. Runs on
        Confirm and again on Compute after the operator changes params (speed, PRR, …)."""
        self._set_busy(True, 'Estimating density on the uploaded route…')
        self.setEnabled(False)
        try:
            self.base_result = estimate_for_route(
                self.dtm, self.drawn_polygon, route, self._params(),
                is_geo=self.is_geo)
            self.survey_route = route
            self.result = self.base_result
        except Exception as e:
            self.setEnabled(True); self._set_busy(False)
            QMessageBox.critical(self, 'Estimate', str(e))
            self.statusBar().showMessage('Estimate failed.'); return
        self._set_route_mode(True)
        self._finish_result(
            f'Done — {self.result.n_waypoints} waypoints (uploaded route, current params)')

    def _refresh_map(self):
        if self.mapview is not None and self.dtm is not None:
            self._set_busy(True, 'Rendering terrain…')
            try:
                self.mapview.set_dtm(self.dtm, self.dtm_path)
                self._map_focused = False          # whole extent
                self._sync_focus_button()
            finally:
                self._set_busy(False)

    def _sync_focus_button(self):
        """Reflect the focus state on the map's ◎ AOI toggle without re-triggering it."""
        if self.mapview is None:
            return
        b = self.mapview.btn_focus
        b.blockSignals(True)
        b.setChecked(self._map_focused)
        b.setEnabled(self.drawn_polygon is not None)
        b.blockSignals(False)

    def _render_map(self, focus):
        """Render the map base (focused on the AOI, or the full extent) and re-apply the
        AOI outline + route/density overlays on top."""
        if self.mapview is None or self.dtm is None:
            return
        self._map_focused = bool(focus and self.drawn_polygon is not None)
        if self._map_focused:
            area = polygon_area_m2(self.drawn_polygon, self.is_geo)
            self.mapview.focus_on(self.drawn_polygon,
                                  margin_m=max(200.0, 0.15 * math.sqrt(area)))
        else:
            self.mapview.show_full()
        self._sync_focus_button()
        if self.drawn_polygon is not None:
            self.mapview.set_aoi_polygon(list(self.drawn_polygon.exterior.coords))
        self._draw_other_polygons()
        if self.result and self.result.route:
            self._render_map_overlays(self.result)
            # set_dtm -> _reset_scene disabled the Edit button and dropped its overlay
            # data; the route survived the re-render, so restore both.
            self.mapview.btn_edit.setEnabled(True)
            self.mapview.set_editable_route(self.survey_route)

    def _on_map_focus_toggled(self, on):
        self._render_map(focus=on)

    def _clear_results(self):
        """Drop the computed route/estimate and its on-screen traces, so stale
        results never linger next to a changed (or absent) route."""
        self.result = None
        self.base_result = None
        self.survey_route = []
        self._reset_edit_history()
        self.lbl_summary.setText(self._empty_summary_html())
        self._update_workflow()
        if self.mapview is not None:
            self.mapview.clear_overlays()
            self.mapview.btn_pass.setChecked(False)
            self.mapview._toggle_pass(False)
            self.mapview.btn_pass.setEnabled(False)
            self.mapview.btn_edit.setChecked(False)
            self.mapview._toggle_edit(False)
            self.mapview.btn_edit.setEnabled(False)
            self.mapview.set_editable_route([])
        self.btn_geojson.setEnabled(False)
        self.btn_csv.setEnabled(False)
        self.act_save_route.setEnabled(False)
        if getattr(self, 'profile_panel', None) is not None:
            self.profile_panel.clear()

    def _on_polygon_drawn(self, geom):
        try:
            poly = shapely_shape(geom)
            if not poly.is_valid:
                poly = poly.buffer(0)
        except Exception as e:
            self.statusBar().showMessage(f'Bad polygon: {e}'); return
        area = polygon_area_m2(poly, self.is_geo)
        if area > MAX_AOI_M2:
            self.drawn_polygon = None
            self.btn_compute.setEnabled(False)
            self.lbl_aoi.setText(
                f'⚠ Polygon is {area / 1e6:.2f} km² — over the {MAX_AOI_M2 / 1e6:.0f} km² '
                f'limit. Draw a smaller area.')
            self.statusBar().showMessage(
                f'Polygon too large ({area / 1e6:.2f} km²); max {MAX_AOI_M2 / 1e6:.0f} km².')
            return
        self.drawn_polygon = poly
        if 0 <= self._aoi_active < len(self._aoi_set):    # redrawn: replaces this polygon
            self._aoi_set[self._aoi_active]['poly'] = poly
        self._clear_results()                 # previous route no longer matches AOI
        self.loaded_route = None; self._set_route_mode(False)   # new AOI -> auto-plan mode
        self.lbl_aoi.setText('✓ polygon set.')
        self.btn_compute.setEnabled(True)
        # If the map is focused, follow the new AOI; otherwise just enable the toggle.
        if self._map_focused:
            self._render_map(focus=True)
        else:
            self._sync_focus_button()
        self._refresh_aoi_combo()
        self.statusBar().showMessage('Polygon set. Click Compute.')

    # ---- several polygons from one file: switch between them, each keeps its work ----
    def _start_aoi_set(self, items, bad=()):
        """Start working on several polygons [(name, Polygon)]. Drops ones off the DTM or
        over the area cap (listed in a warning), opens the first. False if none usable."""
        b = self.dtm.src.bounds
        dtm_box = shapely_box(b.left, b.bottom, b.right, b.top)
        ok, bad = [], list(bad)
        for name, poly in items:
            area = polygon_area_m2(poly, self.is_geo)
            if not poly.intersects(dtm_box):
                bad.append((name, "outside the DTM (coordinates must be in the DTM's CRS)"))
            elif area > MAX_AOI_M2:
                bad.append((name, f'{area / 1e6:.2f} km² — over the '
                                  f'{MAX_AOI_M2 / 1e6:.0f} km² limit'))
            else:
                ok.append((name, poly))
        self._warn_unusable(bad)
        if not ok:
            return False
        self._clear_results()
        self.loaded_route = None
        self._aoi_set = [{'name': n, 'poly': p, 'slot': _empty_slot()} for n, p in ok]
        self._aoi_active = 0
        for f, v in _empty_slot().items():
            setattr(self, f, v)
        self.drawn_polygon = ok[0][1]
        self._present_current()
        self.lbl_aoi.setText(f'✓ {len(ok)} polygons loaded — pick one below. Each keeps its '
                             f'own route and edits.')
        self.statusBar().showMessage(
            f'{len(ok)} polygons loaded; working on "{ok[0][0]}". Click Compute.')
        return True

    def _warn_unusable(self, bad):
        if bad:
            lines = [f'  • {n}: {why}' for n, why in bad[:15]]
            if len(bad) > 15:
                lines.append(f'  … and {len(bad) - 15} more')
            QMessageBox.warning(
                self, 'Polygons',
                "These polygons can't be used and were left out:\n\n" + '\n'.join(lines))

    def _stash_active(self):
        """Save the active polygon's working state into its slot."""
        a = self._aoi_active
        if 0 <= a < len(self._aoi_set):
            e = self._aoi_set[a]
            e['slot'] = {f: getattr(self, f) for f in _SLOT_FIELDS}
            if self.drawn_polygon is not None:
                e['poly'] = self.drawn_polygon

    def _switch_aoi(self, i):
        """Switch to polygon i: keep the current one's route/results/edits, restore i's."""
        if not 0 <= i < len(self._aoi_set) or i == self._aoi_active:
            return
        self._stash_active()
        self._aoi_active = i
        e = self._aoi_set[i]
        for f, v in e['slot'].items():
            setattr(self, f, v)
        self.drawn_polygon = e['poly']
        self._present_current()
        has = bool(self.result and self.result.route)
        waiting = not has and bool(self.loaded_route)
        if not waiting:
            self.lbl_aoi.setText(f'✓ working on "{e["name"]}" ({i + 1} of {len(self._aoi_set)}).')
        self.statusBar().showMessage(
            f'Switched to "{e["name"]}"' + (' — its route is restored.' if has else
                                             ' — its uploaded lines are waiting: select them, '
                                             'then Confirm.' if waiting else
                                             ' — no route yet; click Compute.'))

    def _present_current(self):
        """Show the active polygon's state: map (focused on it, others outlined), route
        overlays, results, profile, buttons and undo — without resetting its edit history."""
        has = bool(self.result and self.result.route)
        self._set_route_mode(self._route_active)
        self.btn_compute.setEnabled(self.drawn_polygon is not None)
        if self.mapview is not None and self.dtm is not None:
            if self.mapview.dtm is not self.dtm:          # a just-opened DTM: crop straight to it
                self._set_busy(True, 'Cropping the DTM to the polygon…')
                try:
                    self.mapview.set_dtm(self.dtm, self.dtm_path,
                                         focus_polygon=self.drawn_polygon)
                finally:
                    self._set_busy(False)
            self._render_map(focus=True)
            if not has and self.loaded_route:             # uploaded lines waiting here
                self._show_loaded_passes()
        if has:
            self._render_summary(self.result)
        else:
            self.lbl_summary.setText(self._empty_summary_html())
        self.btn_geojson.setEnabled(has)
        self.btn_csv.setEnabled(has)
        self.act_save_route.setEnabled(has)
        self._refresh_profile()
        self._update_undo_actions()
        self._update_workflow()
        self._refresh_aoi_combo()

    def _slot_has_route(self, k):
        if k == self._aoi_active:
            return bool(self.result and self.result.route)
        r = self._aoi_set[k]['slot'].get('result')
        return bool(r and r.route)

    def _refresh_aoi_combo(self):
        """The 'Working on' selector: one entry per polygon (✓ = has a route)."""
        if not hasattr(self, 'cmb_aoi'):
            return
        self.aoi_row.setVisible(len(self._aoi_set) > 1)
        self.cmb_aoi.blockSignals(True)
        self.cmb_aoi.clear()
        for k, e in enumerate(self._aoi_set):
            lr = self.loaded_route if k == self._aoi_active else e['slot'].get('loaded_route')
            tag = ('   ✓ route' if self._slot_has_route(k)
                   else '   • lines to confirm' if lr else '')
            self.cmb_aoi.addItem(e['name'] + tag)
        if 0 <= self._aoi_active < len(self._aoi_set):
            self.cmb_aoi.setCurrentIndex(self._aoi_active)
        self.cmb_aoi.blockSignals(False)

    def _draw_other_polygons(self):
        if self.mapview is not None:
            self.mapview.set_other_polygons(
                [(e['name'], list(e['poly'].exterior.coords))
                 for k, e in enumerate(self._aoi_set) if k != self._aoi_active])

    def _clear_aoi_set(self):
        """Leave multi-polygon mode (the active polygon's state stays as the current one)."""
        self._aoi_set = []
        self._aoi_active = -1
        if self.mapview is not None:
            self.mapview.set_other_polygons([])
        self._refresh_aoi_combo()

    def _confirm_discard_set(self):
        """Before an action that closes the loaded polygons: ask if any have routes."""
        if len(self._aoi_set) < 2:
            return True
        busy = [e['name'] for k, e in enumerate(self._aoi_set) if self._slot_has_route(k)]
        if not busy:
            return True
        names = ', '.join(busy[:4]) + ('…' if len(busy) > 4 else '')
        return QMessageBox.question(
            self, 'Polygons',
            f'This closes the {len(self._aoi_set)} loaded polygons and discards the routes '
            f'planned on {len(busy)} of them ({names}).\n\nContinue?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes

    def _active_aoi_name(self):
        a = self._aoi_active
        return self._aoi_set[a]['name'] if 0 <= a < len(self._aoi_set) else ''

    def _export_name(self, default):
        """Default export file name, prefixed with the active polygon's name if any."""
        name = re.sub(r'[^A-Za-z0-9_\-]+', '_', self._active_aoi_name()).strip('_')
        return f'{name}_{default}' if name else default

    def _clear_aoi(self):
        if not self._confirm_discard_set():
            return
        self._clear_aoi_set()
        self.drawn_polygon = None
        self.loaded_route = None; self._set_route_mode(False)
        self.btn_compute.setEnabled(False)
        self.lbl_aoi.setText('Draw a polygon on the map.')
        self._clear_results()
        self._refresh_map()

    def _survey_end(self):
        """Last valid survey waypoint — where a manually drawn pass chains from."""
        return next((w for w in reversed(self.survey_route or [])
                     if not (isinstance(w['z'], float) and math.isnan(w['z']))), None)

    def _params(self):
        return PlanParams(
            altitude_m=self.sp_alt.value(),
            min_peak_clearance_m=_MIN_PEAK_CLEARANCE_M,
            overlap_pct=self.sp_overlap.value(),
            adaptive_spacing=True,          # always on (no longer a UI option)
            edge_margin=False,              # edge fly-past removed
            min_points=self.sp_minpts.value(),
            speed_ms=self.sp_speed.value(),
            pulse_freq_hz=self.cmb_pulse.currentData(),
            scan_freq_hz=self.sp_scanfreq.value(),
        )

    def _compute(self):
        if self.dtm is None:
            return
        if self.drawn_polygon is None:
            QMessageBox.information(self, 'Polygon', 'Draw a polygon on the map first.')
            return
        if self._route_active and self.survey_route:
            # Uploaded route: re-run the estimate on the same passes with current params.
            self._estimate_uploaded_route(self.survey_route)
            return
        if self._edit_undo:          # net manual edits vs the last computed baseline
            if QMessageBox.question(
                    self, 'Re-plan route',
                    'Auto-planning a new route will discard your manual edits '
                    '(added / moved / deleted passes).\n\nContinue and replace them?',
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
                return
        poly = self.drawn_polygon
        self.lbl_summary.setText(
            '<i style="color:#8b96a3;">Computing route + density estimate…</i>')
        self._set_busy(True, 'Computing route + density estimate…')
        self.setEnabled(False)
        try:
            self.base_result = compute_plan(self.dtm, poly, self._params(),
                                            is_geo=self.is_geo)
            self.survey_route = self.base_result.route
            self._route_auto_alt = True     # program-assigned altitudes -> re-band on edits
            self.result = self.base_result
        except Exception as e:
            self.setEnabled(True)
            self._set_busy(False)
            self.lbl_summary.setText(self._empty_summary_html())   # drop the busy line
            QMessageBox.critical(self, 'Compute error', str(e))
            self.statusBar().showMessage('Compute failed.')
            return
        self._finish_result()

    def _set_route_mode(self, active):
        """Enter/leave 'uploaded-route' mode. In this mode Compute re-estimates the
        confirmed route with the current params instead of auto-planning a new one."""
        self._route_active = active
        self.btn_compute.setText('Re-estimate route' if active else 'Compute Route')

    def _finish_result(self, status_msg=None):
        """Render the summary + map overlays + profile for the current result and
        enable the result-dependent actions. Shared by auto-compute and the loaded-route
        path, so both end up in exactly the same state once a result exists."""
        self.setEnabled(True)
        self._render_summary(self.result)
        self._render_map_overlays(self.result)
        self._refresh_profile()
        has_route = bool(self.result and self.result.route)
        self.btn_geojson.setEnabled(has_route)
        self.btn_csv.setEnabled(has_route)
        self.act_save_route.setEnabled(has_route)
        if self.mapview is not None:
            self.mapview.btn_edit.setEnabled(has_route)   # Add Pass lives inside Edit Route
            self.mapview.set_editable_route(self.survey_route if has_route else [])
        self._reset_edit_history()                        # this route is a fresh baseline
        self._set_busy(False)
        if status_msg is None:
            status_msg = (f'Done — {self.result.n_waypoints} waypoints' if has_route
                          else 'Done — no route produced')
        self.statusBar().showMessage(status_msg)
        self._refresh_aoi_combo()

    def _on_pass_drawn(self, payload):
        """Add Pass (Edit Route sub-tool): two free clicks define the pass — payload is
        ((lon,lat) start, (lon,lat) end). Build it (altitude from terrain), append to the
        survey with a fresh pass id, and re-estimate."""
        if not (self.survey_route and self.dtm and self.drawn_polygon is not None):
            return
        p0, p1 = payload
        pid = max((w.get('pass_id', 0) for w in self.survey_route
                   if isinstance(w.get('pass_id'), int)), default=-1) + 1
        new_pass = build_manual_pass(self.dtm, p0, p1, self._params(), self.is_geo, pid)
        if not new_pass:
            self.statusBar().showMessage('Drawn pass has no valid terrain — not added.')
            return
        pre = self._snapshot_route(self.survey_route)
        if self._reestimate_survey(list(self.survey_route) + new_pass,
                                   'Pass added — re-estimating density…'):
            self._record_edit(pre)
            self.statusBar().showMessage(
                f'Pass added at {new_pass[0]["z"]:.0f} m — {self.result.n_waypoints} '
                f'waypoints. Click two points for another, or untick Add Pass.')

    # ------------------------------------------------------------- edit route
    def _reestimate_survey(self, new_survey, busy_msg, reband=True):
        """Swap in an edited survey route, re-run the density estimate + effective
        route, and repaint everything (summary, map, edit overlay, profile). Restores
        the previous route on failure. Returns True on success. Shared by Edit Route.

        reband=False keeps the incoming per-pass altitudes exactly as given (used by the
        manual altitude editor) — otherwise auto-banding would overwrite the operator's
        hand-set height."""
        if not (self.dtm and self.drawn_polygon is not None):
            return False
        prev = self.survey_route
        self._set_busy(True, busy_msg)
        try:
            # Re-group the (possibly edited) passes onto the fewest shared altitudes —
            # unless the route carries operator-supplied 3D altitudes, which we leave be.
            if reband and self._route_auto_alt:
                band_route_altitudes(self.dtm, new_survey, self._params(), self.is_geo)
            self.survey_route = new_survey
            self.base_result = estimate_for_route(
                self.dtm, self.drawn_polygon, self.survey_route,
                self._params(), is_geo=self.is_geo)
            self.result = self.base_result
        except Exception as e:
            self.survey_route = prev
            QMessageBox.critical(self, 'Re-estimate error', str(e))
            return False
        finally:
            self._set_busy(False)
        self._render_summary(self.result)
        self._render_map_overlays(self.result)
        if self.mapview is not None:
            self.mapview.set_editable_route(self.survey_route)
        self._refresh_profile()
        return True

    # ---- undo / redo of route edits (snapshot history of survey_route) ----
    _EDIT_HISTORY_MAX = 50

    def _snapshot_route(self, route):
        """A shallow, independent copy of a route (waypoints are flat scalar dicts)."""
        return [dict(w) for w in route]

    def _record_edit(self, pre_edit_route):
        """Push the pre-edit survey snapshot onto the undo stack and clear redo. Call
        only after an edit has successfully re-estimated."""
        self._edit_undo.append(pre_edit_route)
        del self._edit_undo[:-self._EDIT_HISTORY_MAX]     # cap history depth
        self._edit_redo.clear()
        self._update_undo_actions()

    def _reset_edit_history(self):
        """Drop all edit history — a new baseline route makes old edits meaningless."""
        self._edit_undo.clear(); self._edit_redo.clear()
        self._update_undo_actions()

    def _update_undo_actions(self):
        if getattr(self, 'act_undo', None) is not None:
            self.act_undo.setEnabled(bool(self._edit_undo))
            self.act_redo.setEnabled(bool(self._edit_redo))

    def _undo_edit(self):
        if not self._edit_undo:
            return
        current = self._snapshot_route(self.survey_route)
        target = self._snapshot_route(self._edit_undo[-1])
        if self._reestimate_survey(target, 'Undoing route edit…'):
            self._edit_undo.pop()
            self._edit_redo.append(current)
            self.statusBar().showMessage('Undo — route restored.')
        self._update_undo_actions()

    def _redo_edit(self):
        if not self._edit_redo:
            return
        current = self._snapshot_route(self.survey_route)
        target = self._snapshot_route(self._edit_redo[-1])
        if self._reestimate_survey(target, 'Redoing route edit…'):
            self._edit_redo.pop()
            self._edit_undo.append(current)
            self.statusBar().showMessage('Redo — route re-applied.')
        self._update_undo_actions()

    def _on_edit_delete(self, pids):
        """Edit Route: delete the selected pass(es) from the survey and re-estimate."""
        if not self.survey_route:
            return
        pidset = set(pids)
        remaining = [w for w in self.survey_route if w.get('pass_id') not in pidset]
        if len(remaining) == len(self.survey_route):
            return                                   # nothing matched
        if not remaining:
            QMessageBox.information(self, 'Delete pass',
                                    "That's the last pass — a route needs at least one.")
            return
        n = len(pidset)
        pre = self._snapshot_route(self.survey_route)
        if self._reestimate_survey(
                remaining, f'Deleting {n} pass{"es" if n != 1 else ""} — re-estimating…'):
            self._record_edit(pre)
            left = len({w.get('pass_id') for w in self.survey_route})
            self.statusBar().showMessage(
                f'Deleted {n} pass{"es" if n != 1 else ""} — {left} passes, '
                f'{self.result.n_waypoints} waypoints.')

    def _on_edit_geom(self, updates):
        """Edit Route: one or more endpoints were dragged — rebuild each affected pass
        (altitude re-derived from terrain) and re-estimate once. A shared point moves
        every pass meeting there as a joint, so `updates` may carry several passes.
        Snaps back if any moved line has no valid terrain."""
        if not self.survey_route or not updates:
            return
        rebuilt = {}                                 # pass_id -> new waypoints
        for pid, p0, p1 in updates:
            wps = build_manual_pass(self.dtm, p0, p1, self._params(), self.is_geo, pid)
            if not wps:
                self.statusBar().showMessage('Moved pass has no valid terrain — edit ignored.')
                if self.mapview is not None:
                    self.mapview.set_editable_route(self.survey_route)   # snap handles back
                return
            rebuilt[pid] = wps
        new_survey, seen = [], set()
        for w in self.survey_route:                  # splice each rebuilt pass in place
            pid = w.get('pass_id')
            if pid in rebuilt:
                if pid not in seen:
                    new_survey.extend(rebuilt[pid]); seen.add(pid)
            else:
                new_survey.append(w)
        for pid, wps in rebuilt.items():             # any not already present (shouldn't happen)
            if pid not in seen:
                new_survey.extend(wps)
        pre = self._snapshot_route(self.survey_route)
        if self._reestimate_survey(new_survey, 'Pass moved — re-estimating density…'):
            self._record_edit(pre)
            n = len(rebuilt)
            self.statusBar().showMessage(
                f'Moved {n} pass{"es" if n != 1 else ""} — {self.result.n_waypoints} waypoints.')

    # ---------------------------------------------------------------- profile
    def _toggle_profile(self, on):
        """Show/hide the bottom elevation-profile bar (View menu / Ctrl+E)."""
        if getattr(self, 'profile_panel', None) is None:
            return
        self.profile_panel.setVisible(on)
        if on:
            sizes = self.body_splitter.sizes()
            if sizes[-1] == 0:                         # give the bar room when opening
                total = sum(sizes) or self.height()
                self.body_splitter.setSizes([max(int(total * 0.72), total - 220), 220])
            self._refresh_profile()

    def _refresh_profile(self):
        """Redraw the elevation-profile bar for the current route (skips work while
        the bar is hidden; reopening refreshes it)."""
        if getattr(self, 'profile_panel', None) is None or not self.profile_panel.isVisible():
            return
        if not (self.result and self.result.route and self.dtm):
            self.profile_panel.clear()
            return
        from .profile import route_profile
        # Draw each pass as its own segment with no turnaround/connector line between them
        # (join_passes=False): the connectors add clutter and, when a pass is dragged to a
        # new altitude, would show as a spurious jump to the neighbouring pass.
        dist, terr, flight, spans, pids = route_profile(
            self.result.route, self.dtm, self.is_geo, join_passes=False)
        # current altitude per pass, so the profile's inline editor shows the live value
        pass_alt = {}
        for w in (self.survey_route or []):
            pid, z = w.get('pass_id'), w.get('z')
            if (pid is not None and pid not in pass_alt
                    and not (isinstance(z, float) and math.isnan(z))):
                pass_alt[pid] = z
        self.profile_panel.update_profile(dist, terr, flight, agl=self.sp_alt.value(),
                                          spans=spans, pass_alt=pass_alt, pids=pids)

    def _on_pass_altitude_changed(self, pass_id, new_z):
        """Set one pass's constant altitude from the elevation-profile editor. Snap rule:
        if the new altitude is within ±10 m of the pass immediately to the RIGHT (the next
        pass along the route), adopt that pass's altitude exactly — so nearby passes share
        a height. Re-estimates without re-banding, so the hand-set altitude is kept."""
        if not self.survey_route:
            return
        # passes in route order (survey pass_ids are ints; skip any non-survey markers)
        order, z_of = [], {}
        for w in self.survey_route:
            pid, z = w.get('pass_id'), w.get('z')
            if isinstance(pid, int) and pid not in z_of \
                    and not (isinstance(z, float) and math.isnan(z)):
                order.append(pid); z_of[pid] = z
        if pass_id not in z_of:
            return
        new_z = float(new_z)
        i = order.index(pass_id)
        did_snap = False
        if i + 1 < len(order):                        # snap to the pass on the right
            z_right = z_of[order[i + 1]]
            if abs(new_z - z_right) <= 10.0 and abs(new_z - z_right) > 1e-9:
                new_z = float(z_right)
                did_snap = True

        new_survey = [dict(w) for w in self.survey_route]
        changed = False
        for w in new_survey:
            if w.get('pass_id') == pass_id \
                    and not (isinstance(w['z'], float) and math.isnan(w['z'])):
                if abs(w['z'] - new_z) > 1e-9:
                    changed = True
                w['z'] = new_z
        if not changed:
            return
        pre = self._snapshot_route(self.survey_route)
        if self._reestimate_survey(new_survey, f'Line {pass_id} altitude → {new_z:.0f} m — '
                                   're-estimating…', reband=False):
            self._record_edit(pre)
            self.statusBar().showMessage(
                f'Line {pass_id} altitude set to {new_z:.0f} m'
                f'{" (matched the line to the right)" if did_snap else ""}.')

    def _highlight_pass_on_map(self, pass_id):
        """A pass was clicked in the elevation profile — outline it on the map, or clear
        the outline when pass_id is None (the pass was toggled off)."""
        if self.mapview is None or not (self.result and self.result.route):
            return
        if pass_id is None:
            self.mapview.highlight_pass([]); return
        coords = [(w['x'], w['y']) for w in self.result.route
                  if w.get('pass_id') == pass_id
                  and not (isinstance(w['z'], float) and math.isnan(w['z']))]
        self.mapview.highlight_pass(coords)

    # ---------------------------------------------------------------- export
    def _export_geojson(self):
        if not (self.result and self.result.route):
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Export route GeoJSON',
                                              self._export_name('route.geojson'), 'GeoJSON (*.geojson)')
        if not path:
            return
        feats = [{
            'type': 'Feature',
            'geometry': {'type': 'Point', 'coordinates': [w['x'], w['y'], w['z']]},
            'properties': {'altitude_m': w['z'],
                           'target_agl_m': w.get('target_distance'),
                           'pass_id': w.get('pass_id')},
        } for w in self.result.route
            if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'type': 'FeatureCollection', 'features': feats}, f, indent=2)
        self.statusBar().showMessage(f'Wrote {len(feats)} waypoints → {path}')

    def _export_csv(self):
        if not (self.result and self.result.route):
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Export route CSV',
                                              self._export_name('route.csv'), 'CSV (*.csv)')
        if not path:
            return
        wps = [w for w in self.result.route
               if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        with open(path, 'w', newline='', encoding='utf-8') as f:
            wr = csv.writer(f)
            wr.writerow(['index', 'x', 'y', 'z', 'target_agl_m', 'pass_id'])
            for i, w in enumerate(wps):
                wr.writerow([i, w['x'], w['y'], w['z'],
                             w.get('target_distance'), w.get('pass_id')])
        self.statusBar().showMessage(f'Wrote {len(wps)} waypoints → {path}')

    def _survey_pass_lines(self):
        """The survey passes as shapely LineString Z (one per pass, endpoint→endpoint),
        in route order."""
        groups = {}
        order = []
        for w in self.survey_route:
            if isinstance(w['z'], float) and math.isnan(w['z']):
                continue
            pid = w.get('pass_id')
            if pid is None:
                continue
            if pid not in groups:
                order.append(pid)
            groups.setdefault(pid, []).append((w['x'], w['y'], w['z']))
        return [LineString([groups[p][0], groups[p][-1]])
                for p in order if len(groups[p]) >= 2]

    def _save_wkt_route(self):
        """Save the (possibly edited) survey passes as a WKT MULTILINESTRING Z — one
        LINESTRING per pass, vertices carrying the flight altitude. This is exactly the
        format Load route reads, so an edited route round-trips: edit → Save → Load."""
        if not self.survey_route:
            return
        lines = self._survey_pass_lines()
        if not lines:
            QMessageBox.warning(self, 'Save route', 'No valid passes to save.')
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Save route WKT',
                                              self._export_name('route.wkt'), 'WKT (*.wkt)')
        if not path:
            return
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(MultiLineString(lines).wkt)
        except OSError as e:
            QMessageBox.critical(self, 'Save route', f'Could not write the file:\n{e}')
            return
        self.statusBar().showMessage(f'Saved {len(lines)} passes → {path}')

    def _render_map_overlays(self, r):
        if self.mapview is None:
            return
        wps = [w for w in r.route
               if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        est = r.estimate or {}
        rad = max(float(est.get('cell_size_m', 2.0)), 3.0)
        # An uploaded route is a set of INDEPENDENT passes — don't draw connectors
        # between them (the auto-planned route stays connected in flight order).
        connect = not self._route_active
        by_reason = est.get('failing_cells_by_reason')
        if by_reason:
            self.mapview.show_plan(wps, None, density_radius_m=rad,
                                   cells_by_reason=by_reason, connect_passes=connect)
        else:                       # older result shape: single-colour fallback
            self.mapview.show_plan(wps, est.get('failing_cells_geo', []),
                                   density_color='#ff9900', density_radius_m=rad,
                                   connect_passes=connect)

    # ---------------------------------------------------------------- render
    def _render_summary(self, r):
        if not r.route:
            self.lbl_summary.setText('No route produced (polygon too small or off the DTM).')
            return
        est = r.estimate or {}
        area = (f'{r.area_m2 / 1e6:.3f} km²' if r.area_m2 >= 1e6
                else f'{r.area_m2:,.0f} m²')
        plen_m = r.path_len_m
        plen = (f'{plen_m / 1000:.2f} km' if plen_m >= 1000
                else f'{plen_m:.0f} m')
        ncell = max(est.get('n_cells', 0), 1)
        cov = 100.0 * (est.get('n_cells', 0) - est.get('n_fail', 0)) / ncell
        n_passes = len({w.get('pass_id', 0) for w in r.route
                        if not (isinstance(w['z'], float) and math.isnan(w['z']))})

        rows = [
            ('<b>Polygon</b>', self._active_aoi_name()),
            ('Area', area),
            ('<b>Route</b>', ''),
            ('Passes', f'{n_passes}'),
            ('Path length', plen),
            ('Alt range', f'{r.alt_min:.0f} – {r.alt_max:.0f} m'),
        ]
        rows += [
            ('<b>Density estimate</b>', ''),
            ('Coverage', f'{cov:.1f}%'),
            ('Median density', f"{est.get('median_density', 0):.0f} pts/m²"),
            ('Average density', f"{est.get('mean_density', 0):.0f} pts/m²"),
        ]
        # Density bands: what fraction of AOI blocks fall in each 50-wide density band,
        # anchored at the target (min_points). Answers "how much of the area is well over
        # target vs just scraping it vs under."
        vals = est.get('in_region_density')
        thr = int(self._params().min_points)
        if vals is not None and getattr(vals, 'size', 0) and thr > 0:
            import numpy as np
            edges = [0]
            x = 50
            while x < thr:
                edges.append(x); x += 50
            edges.append(thr)
            counts, _ = np.histogram(vals, bins=edges + [np.inf])
            total = int(vals.size)
            rows.append((f'<b>Density bands</b> (target {thr})', ''))
            rows.append((f'≥ {thr} pts/m²', f'{100.0 * counts[-1] / total:.1f}%'))
            for i in range(len(counts) - 2, -1, -1):
                lo, hi = edges[i], edges[i + 1]
                lbl = f'below {hi}' if lo == 0 else f'{lo} – {hi}'
                rows.append((lbl, f'{100.0 * counts[i] / total:.1f}%'))
        # Failure breakdown by CAUSE — swatch colours match the map overlay, so the
        # operator reads why each patch is orange/red/etc. and what lever fixes it.
        # 'thin' folds in the uncovered (gap) cells — both are "under target coverage"
        # and share the density gradient; range & shadow stay distinct causes.
        _reason_counts = {'range': est.get('n_beyond_range', 0),
                          'shadow': est.get('n_shadow', 0),
                          'thin': est.get('n_thin', 0) + est.get('n_gap', 0)}
        if any(_reason_counts.values()):
            rows.append(('<b>Why cells fail</b>', ''))
            for key in ('range', 'shadow', 'thin'):
                n = _reason_counts[key]
                if not n:
                    continue
                label, lever = FAILURE_REASON_LABEL[key]
                if key == 'thin':
                    # gradient swatch: red (empty / uncovered) → yellow (at target)
                    swatch = ('<span style="color:#ff0000">■</span>'
                              '<span style="color:#ff9900">■</span>'
                              '<span style="color:#ffee00">■</span>')
                    label = 'Under target (empty → target)'
                else:
                    swatch = f'<span style="color:{FAILURE_REASON_STYLE[key][0]}">■</span>'
                rows.append((f'{swatch} {label}', f'{n:,} cells · <i>{lever}</i>'))

        html = ['<table cellspacing=6>']
        for k, val in rows:
            html.append(f'<tr><td>{k}</td><td><b>{val}</b></td></tr>')
        html.append('</table>')
        self.lbl_summary.setText(''.join(html))
