"""Main window for the desktop planner.

Left: parameter sidebar + Compute. Right: tabbed views — Summary and the 2D
Leaflet Map (draw the AOI, see the route + under-density overlay). Compute runs
the existing model and fills the Summary panel.
"""
import csv
import json
import math
import os

from PySide6.QtCore import Qt, QSettings
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QFormLayout, QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QGroupBox,
    QSplitter, QScrollArea, QFileDialog, QMessageBox, QFrame, QProgressBar,
    QApplication, QAbstractSpinBox, QToolButton,
)

from .planning import (PlanParams, compute_plan, load_dtm, chm_compatible,
                       scan_lines_for_square_pattern, build_manual_pass,
                       estimate_for_route, polygon_area_m2, MAX_AOI_M2,
                       _pass_altitude, band_pass_altitudes, _path_length_m, _LAT_M)

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
        "shadow": ("Occlusion shadow", "needs a cross-pass, or accept"),
        "thin": ("Under target", "lower AGL / tighter spacing"),
    }

from shapely.geometry import shape as shapely_shape, box as shapely_box, MultiPoint
from shapely.wkt import loads as wkt_loads

PULSE_FREQS = (150_000, 300_000, 600_000, 1_200_000, 1_800_000, 2_400_000)

# Ferry/home legs are pure transit (not scanned). They fly at the connecting pass's
# altitude to avoid altitude changes at the survey boundary, and only climb above it
# when the terrain under the leg comes within this clearance margin — enough to keep
# the ferry line off the ground without needless extra altitude (a multirotor
# recovers nothing on the descent, so every wasted metre of climb is wasted energy).
_TRANSIT_BUFFER_M = 20.0

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


def _read_wkt_geoms(path):
    """Geometries from a `.wkt` (plain text, one geometry) or a `.csv` with a WKT column
    (one geometry per row). Raises ValueError with a clear message on failure."""
    if os.path.splitext(path)[1].lower() != '.csv':
        with open(path, 'r', encoding='utf-8') as f:
            txt = f.read().strip()
        if not txt:
            raise ValueError('File is empty.')
        return [wkt_loads(txt)]
    with open(path, 'r', encoding='utf-8-sig', newline='') as f:
        content = f.read()
    if not content.strip():
        raise ValueError('CSV is empty.')
    lines = content.splitlines()
    for delim in (',', ';', '\t', '|'):         # pick the delimiter that yields a WKT column
        rows = [r for r in csv.reader(lines, delimiter=delim)
                if any((c or '').strip() for c in r)]
        if not rows:
            continue
        col, data_rows = _locate_wkt_column(rows)
        if col is None:
            continue
        geoms = []
        for r in data_rows:
            v = (r[col] if col < len(r) else '').strip()
            if v:
                geoms.append(wkt_loads(v))
        if geoms:
            return geoms
    raise ValueError('No WKT column found (expected a "WKT"/"geometry" column, or WKT '
                     'text in a column).')


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
        self.chm = None
        self.chm_path = None
        self.is_geo = True
        self.base_result = None          # survey-only result (no home legs)
        self.survey_route = []           # survey waypoints (base for home legs)
        self.result = None               # effective result = survey + in-AOI home legs
        self.drawn_polygon = None        # shapely Polygon drawn on the map
        self._map_focused = False        # map zoomed to the AOI (vs full-extent overview)
        self.loaded_route = None         # operator route from .wkt (list of waypoints), for pass selection
        self._route_auto_alt = False     # loaded route had no Z (altitudes auto-assigned -> band on confirm)
        self._route_active = False       # a confirmed uploaded route is active (Compute re-estimates it)
        self._pending_aoi = None         # AOI loaded before a DTM — crop to it on the next Open DTM
        self.home = None                 # takeoff/return-home (lon, lat) or None
        self.home_ground = float('nan')  # terrain elevation at home, if in the DTM

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
        a_chm = QAction('Open CHM…', self); a_chm.triggered.connect(self._open_chm)
        a_wkt = QAction('Load polygon (.wkt/.csv)…', self)
        a_wkt.triggered.connect(self._load_wkt_aoi)
        a_route = QAction('Load route (.wkt/.csv)…', self)
        a_route.triggered.connect(self._load_wkt_route)
        a_quit = QAction('Quit', self); a_quit.triggered.connect(self.close)
        m.addAction(a_dtm); m.addAction(a_chm); m.addAction(a_wkt); m.addAction(a_route)
        m.addSeparator(); m.addAction(a_quit)

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
        self.lbl_chm = QLabel('CHM: (none)'); self.lbl_chm.setWordWrap(True)
        dtm_row = QHBoxLayout()
        b_dtm = QPushButton('Open DTM…'); b_dtm.clicked.connect(self._open_dtm)
        b_dtm_clear = QPushButton('Clear'); b_dtm_clear.clicked.connect(self._clear_dtm)
        dtm_row.addWidget(b_dtm); dtm_row.addWidget(b_dtm_clear)
        chm_row = QHBoxLayout()
        b_chm = QPushButton('Open CHM…'); b_chm.clicked.connect(self._open_chm)
        b_chm_clear = QPushButton('Clear'); b_chm_clear.clicked.connect(self._clear_chm)
        chm_row.addWidget(b_chm); chm_row.addWidget(b_chm_clear)
        dl.addWidget(self.lbl_dtm); dl.addLayout(dtm_row)
        dl.addWidget(self.lbl_chm); dl.addLayout(chm_row)
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
        al.addWidget(self.lbl_aoi); al.addWidget(b_enter_aoi); al.addWidget(b_clear_aoi)
        v.addWidget(gb_aoi)

        # ── Takeoff / Home ── (optional — outside the numbered sequence)
        gb_home = QGroupBox('Takeoff / Home · optional')
        hl = QVBoxLayout(gb_home)
        self.lbl_home = QLabel('Home: (none)')
        self.lbl_home.setWordWrap(True); self.lbl_home.setStyleSheet('color:#888;')
        home_row = QHBoxLayout()
        b_home = QPushButton('Set coordinate…'); b_home.clicked.connect(self._set_home_coord)
        b_home_clear = QPushButton('Clear'); b_home_clear.clicked.connect(self._clear_home)
        home_row.addWidget(b_home); home_row.addWidget(b_home_clear)
        hl.addWidget(self.lbl_home); hl.addLayout(home_row)
        v.addWidget(gb_home)

        # ── Flight ── (workflow step ③)
        gb_flight = QGroupBox('③ Flight')
        fl = QFormLayout(gb_flight)
        fl.setRowWrapPolicy(QFormLayout.WrapLongRows)          # field drops under label
        fl.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)  # when the pane is narrow
        self.sp_alt = self._dspin(1, 1000, 100, ' m', 5)
        self.sp_overlap = self._dspin(20, 50, 20, ' %', 1)
        self.cb_adaptive = QCheckBox('Terrain-adaptive spacing'); self.cb_adaptive.setChecked(True)
        self.cb_edge_margin = QCheckBox('Edge fly-past (cover polygon rim)')
        self.cb_edge_margin.setToolTip(
            'Extend passes one pass-pitch beyond the polygon so edge cells get full '
            'overlap (removes the boundary coverage gap). Flies slightly outside the polygon.')
        fl.addRow('Altitude AGL', self.sp_alt)
        fl.addRow('Overlap', self.sp_overlap)
        fl.addRow(self.cb_adaptive)               # span both columns → hug the left edge
        fl.addRow(self.cb_edge_margin)
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
        self.sp_veg = self._dspin(0, 1, 0.4, '', 0.05); self.sp_veg.setDecimals(2)
        scl.addRow('Min points / m²', self.sp_minpts)
        scl.addRow('Drone speed', self.sp_speed)

        self.scan_advanced = CollapsibleSection('Advanced')
        adv = QFormLayout()
        adv.setContentsMargins(0, 2, 0, 0)
        adv.setRowWrapPolicy(QFormLayout.WrapLongRows)
        adv.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        adv.addRow('Pulse freq', self.cmb_pulse)
        adv.addRow('Scan freq (auto)', self.sp_scanfreq)
        adv.addRow('Canopy ground-return frac', self.sp_veg)
        adv.addRow(QLabel('FOV fixed at 100° (±50°)'))
        self.scan_advanced.set_content_layout(adv)
        scl.addRow(self.scan_advanced)
        v.addWidget(gb_scan)

        self.btn_compute = QPushButton('Compute Route')
        self.btn_compute.setObjectName('primary')
        self.btn_compute.setEnabled(False)
        self.btn_compute.clicked.connect(self._compute)
        v.addWidget(self.btn_compute)

        self.btn_helios = QPushButton('Validate (HELIOS++)…')
        self.btn_helios.setEnabled(False)
        self.btn_helios.clicked.connect(self._open_helios)
        v.addWidget(self.btn_helios)

        gb_exp = QGroupBox('Export'); el = QHBoxLayout(gb_exp)
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
        # AOI/home mission state (which belongs to a mission, not to preferences).
        return QSettings('AutoRoutePlanning', 'RoutePlanner')

    def _save_settings(self):
        s = self._settings()
        s.setValue('flight/agl', self.sp_alt.value())
        s.setValue('flight/overlap', self.sp_overlap.value())
        s.setValue('flight/adaptive', self.cb_adaptive.isChecked())
        s.setValue('flight/edge_margin', self.cb_edge_margin.isChecked())
        s.setValue('scan/min_points', self.sp_minpts.value())
        s.setValue('scan/speed', self.sp_speed.value())
        s.setValue('scan/pulse_freq', self.cmb_pulse.currentData())
        s.setValue('scan/veg', self.sp_veg.value())
        s.setValue('window/geometry', self.saveGeometry())

    def _load_settings(self):
        s = self._settings()
        self.sp_alt.setValue(s.value('flight/agl', self.sp_alt.value(), type=float))
        self.sp_overlap.setValue(
            s.value('flight/overlap', self.sp_overlap.value(), type=float))
        self.cb_adaptive.setChecked(
            s.value('flight/adaptive', self.cb_adaptive.isChecked(), type=bool))
        self.cb_edge_margin.setChecked(
            s.value('flight/edge_margin', self.cb_edge_margin.isChecked(), type=bool))
        self.sp_minpts.setValue(
            s.value('scan/min_points', self.sp_minpts.value(), type=int))
        self.sp_speed.setValue(s.value('scan/speed', self.sp_speed.value(), type=float))
        pf = s.value('scan/pulse_freq', self.cmb_pulse.currentData(), type=int)
        idx = self.cmb_pulse.findData(pf)
        if idx >= 0:
            self.cmb_pulse.setCurrentIndex(idx)           # re-derives the scan freq
        self.sp_veg.setValue(s.value('scan/veg', self.sp_veg.value(), type=float))
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
        try:
            self.dtm = load_dtm(path)
        except Exception as e:
            QMessageBox.critical(self, 'DTM error', str(e)); return
        self.dtm_path = path
        crs = self.dtm.src.crs
        self.is_geo = crs.is_geographic if crs else True
        w, h = self.dtm.src.width, self.dtm.src.height     # native size (array may be windowed)
        self.lbl_dtm.setText(f'DTM: {path}\n{w}×{h} px · CRS {crs}')
        self.home = None; self.home_ground = float('nan')   # new area
        self.lbl_home.setText('Home: (none)')
        self.loaded_route = None; self._set_route_mode(False)
        self._clear_results()
        if self._pending_aoi is not None:
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
            self.mapview.set_dtm(self.dtm, self.dtm_path, self.chm, self.chm_path,
                                 focus_polygon=poly)
            self.mapview.set_aoi_polygon(list(poly.exterior.coords))
        finally:
            self._set_busy(False)
        self._sync_focus_button()
        self.btn_compute.setEnabled(True)
        self.lbl_aoi.setText('✓ polygon loaded — DTM cropped to it (map focused).')
        self.statusBar().showMessage(
            'DTM opened cropped to the polygon — full-extent render skipped.')

    def _clear_dtm(self):
        """Drop the loaded DTM (and the CHM/AOI/results that depend on it) and
        blank the map — mirrors the CHM Clear."""
        self.dtm = None; self.dtm_path = None
        self.chm = None; self.chm_path = None
        self.is_geo = True
        self.drawn_polygon = None
        self.loaded_route = None; self._set_route_mode(False)
        self.home = None; self.home_ground = float('nan')
        self.lbl_dtm.setText('DTM: (none)')
        self.lbl_chm.setText('CHM: (none)')
        self.lbl_home.setText('Home: (none)')
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
        info = QLabel('One vertex per line as  <b>lat, lon</b>  (matching the map '
                      'readout). At least 3 vertices; the polygon is closed '
                      'automatically.')
        info.setWordWrap(True); info.setTextFormat(Qt.RichText)
        txt = QPlainTextEdit()
        txt.setPlaceholderText('47.10, 8.30\n47.10, 8.40\n47.20, 8.40\n47.20, 8.30')
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
        """Parse 'lat, lon' lines into a list of [lon, lat] vertices, or None."""
        coords = []
        for ln in text.splitlines():
            ln = ln.strip()
            if not ln:
                continue
            parts = ln.replace(',', ' ').split()
            if len(parts) < 2:
                QMessageBox.warning(self, 'Polygon', f'Bad line: "{ln}"\nUse: lat, lon')
                return None
            try:
                lat, lon = float(parts[0]), float(parts[1])
            except ValueError:
                QMessageBox.warning(self, 'Polygon', f'Not numbers: "{ln}"')
                return None
            coords.append([lon, lat])
        if len(coords) < 3:
            QMessageBox.warning(self, 'Polygon', 'Enter at least 3 vertices.')
            return None
        return coords

    def _load_wkt_aoi(self):
        """Load an AOI polygon from a .wkt file or a .csv with a WKT column (coordinates in
        the DTM's CRS). With a DTM open, focus the map on it at native resolution. WITHOUT a
        DTM, remember it and crop the DTM to it on the next Open DTM."""
        path, _ = QFileDialog.getOpenFileName(
            self, 'Load polygon', '',
            'WKT or CSV (*.wkt *.csv *.txt);;All files (*)')
        if not path:
            return
        try:
            geoms = _read_wkt_geoms(path)
        except Exception as e:
            QMessageBox.critical(self, 'Polygon', f'Could not read the file:\n{e}'); return
        # Reduce to a single polygon (first polygonal geometry in the file).
        poly = None
        for g in geoms:
            if g.geom_type == 'Polygon':
                poly = g; break
            if g.geom_type in ('MultiPolygon', 'GeometryCollection'):
                poly = g.convex_hull; break          # a valid single Polygon
        if poly is None:
            QMessageBox.warning(self, 'Polygon', 'No polygon found in the file.'); return
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty or poly.geom_type != 'Polygon':
            QMessageBox.warning(self, 'Polygon', 'The polygon is empty or invalid.'); return
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

    def _passes_region(self, pts):
        """A polygon covering `pts` [(lon,lat)] — the convex hull, buffered to a real
        area when the points are collinear (a single straight pass)."""
        hull = MultiPoint(list(pts)).convex_hull
        if hull.geom_type != 'Polygon':
            hull = hull.buffer(0.0005 if self.is_geo else 50.0)
        return hull

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
        # Each straight segment (vertex A -> vertex B) is one selectable pass, so a
        # multi-vertex line is split into its individual passes. 3D -> vertex Z is the
        # flight altitude; 2D -> automatic altitude per pass (mean terrain + sidebar AGL,
        # floored to clear the pass peak), same rule as the planner.
        have_z = all(ln.has_z for ln in lines)
        params = self._params()
        to_m = _LAT_M if self.is_geo else 1.0
        step_map = params.step_m / to_m
        res_map = min(abs(self.dtm.src.res[0]), abs(self.dtm.src.res[1]))
        elev_step = min(step_map, res_map)
        route, pass_pts, auto, skipped, pid = [], [], 0, 0, 0
        for ln in lines:
            cs = list(ln.coords)
            for a, b in zip(cs, cs[1:]):
                seg = [(float(a[0]), float(a[1])), (float(b[0]), float(b[1]))]
                if have_z:
                    zs, td = [float(a[2]), float(b[2])], None
                else:
                    zc = _pass_altitude(self.dtm, _densify_polyline(seg, step_map),
                                        params.altitude_m, step_map, elev_step,
                                        params.min_peak_clearance_m)
                    if math.isnan(zc):                   # no valid terrain under this pass
                        skipped += 1; continue
                    auto += 1; zs, td = [zc, zc], params.altitude_m
                pid += 1
                for (x, y), z in zip(seg, zs):
                    route.append({'x': x, 'y': y, 'z': z, 'pass_id': pid,
                                  'target_distance': td})
                pass_pts.append((pid, seg))
        if not route:
            QMessageBox.warning(
                self, 'Route', 'No passes with valid terrain under them '
                '(coordinates off the DTM, or in the wrong CRS).'); return
        allpts = [(wp['x'], wp['y']) for wp in route]
        b = self.dtm.src.bounds
        if not self._passes_region(allpts).intersects(
                shapely_box(b.left, b.bottom, b.right, b.top)):
            QMessageBox.warning(
                self, 'Route', 'The route does not overlap the DTM.\nCoordinates must '
                'be in the same CRS as the DTM.'); return
        # Keep the AOI; drop any previous route/estimate. Show the AOI + passes and let
        # the operator pick the relevant passes.
        self.loaded_route = route
        self._route_auto_alt = not have_z    # band the altitudes on confirm (2D route only)
        self._clear_results()
        self._set_route_mode(False)          # active again once passes are confirmed
        view = self._passes_region(list(self.drawn_polygon.exterior.coords) + allpts)
        self._map_focused = True
        self.mapview.focus_on(view, margin_m=150.0)
        self._sync_focus_button()
        self.mapview.set_aoi_polygon(list(self.drawn_polygon.exterior.coords))
        self.mapview.show_route_passes(pass_pts)
        alt_note = (f'altitude from Z' if have_z
                    else f'auto altitude at {params.altitude_m:.0f} m AGL')
        skip_note = f' ({skipped} skipped — off terrain)' if skipped else ''
        self.lbl_aoi.setText('Route loaded — click the passes relevant for point-cloud '
                             'production, then ✓ Confirm passes.')
        self.statusBar().showMessage(
            f'Route from {os.path.basename(path)}: {len(pass_pts)} passes, {alt_note}'
            f'{skip_note}. Click the relevant ones, then Confirm.')

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
                chm=self.chm, is_geo=self.is_geo)
            self.survey_route = route
            self.result = self._effective_result()
        except Exception as e:
            self.setEnabled(True); self._set_busy(False)
            QMessageBox.critical(self, 'Estimate', str(e))
            self.statusBar().showMessage('Estimate failed.'); return
        self._set_route_mode(True)
        self._finish_result(
            f'Done — {self.result.n_waypoints} waypoints (uploaded route, current params)')

    def _open_chm(self):
        if self.dtm is None:
            QMessageBox.information(self, 'CHM', 'Open a DTM first.'); return
        path, _ = QFileDialog.getOpenFileName(
            self, 'Open CHM', '', 'GeoTIFF (*.tif *.tiff);;All files (*)')
        if not path:
            return
        try:
            chm = load_dtm(path)
        except Exception as e:
            QMessageBox.critical(self, 'CHM error', str(e)); return
        ok, reason = chm_compatible(self.dtm, chm)
        if not ok:
            QMessageBox.warning(self, 'CHM incompatible',
                                f'{reason}\n\nThe CHM was not applied.')
            return
        self.chm = chm
        self.chm_path = path
        self.lbl_chm.setText(f'CHM: {path}')
        self._clear_results()          # density estimate is now stale
        self._refresh_map()
        if reason:                     # soft note (e.g. partial overlap)
            QMessageBox.information(self, 'CHM applied', reason)
            self.statusBar().showMessage(reason)

    def _clear_chm(self):
        self.chm = None; self.chm_path = None
        self.lbl_chm.setText('CHM: (none)')
        self._clear_results()          # density estimate is now stale
        self._refresh_map()

    def _refresh_map(self):
        if self.mapview is not None and self.dtm is not None:
            self._set_busy(True, 'Rendering terrain…')
            try:
                self.mapview.set_dtm(self.dtm, self.dtm_path, self.chm, self.chm_path)
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
        AOI outline + route/density/home overlays on top."""
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
        if self.result and self.result.route:
            self._render_map_overlays(self.result)
        else:
            self._show_home()

    def _on_map_focus_toggled(self, on):
        self._render_map(focus=on)

    def _clear_results(self):
        """Drop the computed route/estimate and its on-screen traces, so stale
        results never linger next to a changed (or absent) route."""
        self.result = None
        self.base_result = None
        self.survey_route = []
        self.lbl_summary.setText(self._empty_summary_html())
        self._update_workflow()
        if self.mapview is not None:
            self.mapview.clear_overlays()
            self._show_home()                 # keep the marker, drop stale ferry legs
            self.mapview.btn_pass.setChecked(False)
            self.mapview._toggle_pass(False)
            self.mapview.btn_pass.setEnabled(False)
        self.btn_helios.setEnabled(False)
        self.btn_geojson.setEnabled(False)
        self.btn_csv.setEnabled(False)
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
        self._clear_results()                 # previous route no longer matches AOI
        self.loaded_route = None; self._set_route_mode(False)   # new AOI -> auto-plan mode
        self.lbl_aoi.setText('✓ polygon set.')
        self.btn_compute.setEnabled(True)
        # If the map is focused, follow the new AOI; otherwise just enable the toggle.
        if self._map_focused:
            self._render_map(focus=True)
        else:
            self._sync_focus_button()
        self.statusBar().showMessage('Polygon set. Click Compute.')

    def _clear_aoi(self):
        self.drawn_polygon = None
        self.loaded_route = None; self._set_route_mode(False)
        self.btn_compute.setEnabled(False)
        self.lbl_aoi.setText('Draw a polygon on the map.')
        self._clear_results()
        self._refresh_map()

    def _set_home_coord(self):
        """Enter the takeoff / return-home point as a typed GPS coordinate."""
        if self.dtm is None:
            QMessageBox.information(self, 'Home', 'Open a DTM first.'); return
        from PySide6.QtWidgets import QInputDialog
        text, ok = QInputDialog.getText(
            self, 'Set takeoff / home', 'Home coordinate as  lat, lon :')
        if not ok or not text.strip():
            return
        parts = text.replace(',', ' ').split()
        try:
            lat, lon = float(parts[0]), float(parts[1])
        except (ValueError, IndexError):
            QMessageBox.warning(self, 'Home', f'Could not read "{text}".\nUse: lat, lon')
            return
        self.home = (lon, lat)
        z = self.dtm.elevation_at(lon, lat)
        self.home_ground = z
        gtxt = f'ground {z:.0f} m' if z == z else 'outside DTM'
        self.lbl_home.setText(f'Home: {lat:.5f}, {lon:.5f}  ({gtxt})')
        self._set_busy(True, 'Adding home — re-estimating…')
        try:
            self._rebuild_effective()
        finally:
            self._set_busy(False)

    def _clear_home(self):
        self.home = None
        self.home_ground = float('nan')
        self.lbl_home.setText('Home: (none)')
        self._rebuild_effective()

    def _show_home(self):
        """Redraw the home marker + ferry legs (legs only when a route exists)."""
        if self.mapview is None:
            return
        wps = self.result.route if (self.result and self.result.route) else []
        self.mapview.show_home(self.home, wps)

    def _transit_wp(self, x, y, z):
        """A non-survey ferry waypoint (home, or a survey-boundary climb/descent
        point) at the given position/altitude."""
        return {'x': x, 'y': y, 'z': z, 'target_distance': None, 'pass_id': 'home'}

    def _terrain_max_along(self, p0, p1):
        """Max DTM terrain elevation sampled (at pixel resolution) along the
        segment p0→p1, ignoring cells outside the DTM. NaN if none have data."""
        ax, ay = p0
        bx, by = p1
        dist = math.hypot(bx - ax, by - ay)
        res_map = min(abs(self.dtm.src.res[0]), abs(self.dtm.src.res[1]))
        n = 0 if dist == 0 else max(1, min(4000, int(dist / max(res_map, 1e-12))))
        best = float('nan')
        for i in range(n + 1):
            f = 0.0 if n == 0 else i / n
            e = self.dtm.elevation_at(ax + (bx - ax) * f, ay + (by - ay) * f)
            if e == e:                       # not NaN
                best = e if math.isnan(best) else max(best, e)
        return best

    def _ferry_altitude(self, endpoint):
        """Flat transit height for the pure-transit ferry leg between home and
        `endpoint` (the first/last survey waypoint). Primary rule: fly at the
        connecting pass's own altitude, so entering/leaving the survey needs no
        altitude change. Only climb above that when the terrain under THIS leg would
        come within the clearance buffer — just enough to keep the ferry line off the
        ground — and never pinned to the global survey max. Falls back to the pass
        altitude where the leg leaves the DTM (no terrain to clear against)."""
        ez = endpoint['z']                              # connecting pass altitude
        tmax = self._terrain_max_along(self.home, (endpoint['x'], endpoint['y']))
        if math.isnan(tmax):
            return ez
        return max(ez, tmax + _TRANSIT_BUFFER_M)        # raise only to clear terrain

    def _route_with_home(self):
        """Effective route bracketed by the home point (takeoff … return).

        Each ferry leg is flown FLAT at its own transit altitude, then the drop to
        (or climb from) the pass altitude happens at the survey edge — not spread
        across the leg. If the descent were interpolated over the whole ferry, the
        flight line would sag through any mid-leg terrain peak; holding altitude to
        the boundary and stepping down over the survey entry point keeps the ferry
        clear of the ground the whole way."""
        route = self.result.route
        if self.home is None or not route:
            return route
        valid = [w for w in route
                 if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        if not valid:
            return route
        first, last = valid[0], valid[-1]
        hx, hy = self.home
        a_out = self._ferry_altitude(first)
        a_ret = self._ferry_altitude(last)

        head = [self._transit_wp(hx, hy, a_out)]
        if a_out > first['z'] + 1e-6:          # hold high, then descend at the edge
            head.append(self._transit_wp(first['x'], first['y'], a_out))
        tail = []
        if a_ret > last['z'] + 1e-6:           # climb at the edge, then hold high
            tail.append(self._transit_wp(last['x'], last['y'], a_ret))
        tail.append(self._transit_wp(hx, hy, a_ret))
        return head + route + tail

    def _survey_end(self):
        """Last valid survey waypoint — where a manually drawn pass chains from."""
        return next((w for w in reversed(self.survey_route or [])
                     if not (isinstance(w['z'], float) and math.isnan(w['z']))), None)

    def _ferry_inpoly_far(self, anchor, toward, poly):
        """Walking from `anchor` (a survey endpoint, on/in the polygon) toward
        `toward` (home), the farthest point still inside the polygon — i.e. the
        in-AOI portion of that ferry. (lon,lat) or None when negligible."""
        from shapely.geometry import Point
        ax, ay = anchor
        tx, ty = toward
        dist = math.hypot(tx - ax, ty - ay)
        if dist == 0:
            return None
        to_m = _LAT_M if self.is_geo else 1.0
        res_m = min(abs(self.dtm.src.res[0]), abs(self.dtm.src.res[1])) * to_m
        n = max(1, min(2000, int(dist / max(res_m / to_m, 1e-12))))
        far = None
        for i in range(1, n + 1):
            f = i / n
            px, py = ax + (tx - ax) * f, ay + (ty - ay) * f
            if poly.covers(Point(px, py)):
                far = (px, py)
            else:
                break                       # left the polygon (contiguous from anchor)
        if far is None:
            return None
        seg_m = math.hypot((far[0] - ax) * to_m, (far[1] - ay) * to_m)
        return far if seg_m >= max(2 * res_m, 5.0) else None

    def _home_legs(self):
        """(start_leg, end_leg) waypoint lists: the in-AOI portions of the
        home↔survey ferry, flown as terrain-following scanning passes. ([],[])
        when neither ferry crosses the polygon."""
        if (self.home is None or not self.survey_route
                or self.drawn_polygon is None):
            return [], []
        valid = [w for w in self.survey_route
                 if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        if not valid:
            return [], []
        params = self._params()
        poly = self.drawn_polygon
        ids = [w.get('pass_id', 0) for w in valid if isinstance(w.get('pass_id'), int)]
        base = max(ids, default=-1)
        sfirst = (valid[0]['x'], valid[0]['y'])
        slast = (valid[-1]['x'], valid[-1]['y'])
        start_leg, end_leg = [], []
        e = self._ferry_inpoly_far(sfirst, self.home, poly)
        if e:                               # entry → survey start (own pass id)
            start_leg = build_manual_pass(self.dtm, e, sfirst, params, self.is_geo, base + 1)
        x = self._ferry_inpoly_far(slast, self.home, poly)
        if x:                               # survey end → exit (own pass id)
            end_leg = build_manual_pass(self.dtm, slast, x, params, self.is_geo, base + 2)
        return start_leg, end_leg

    def _effective_result(self):
        """Base survey result augmented with in-AOI home legs (re-estimated), or
        the base result unchanged when there's no home / no crossing ferry."""
        if self.base_result is None:
            return None
        start_leg, end_leg = self._home_legs()
        if not start_leg and not end_leg:
            return self.base_result
        route = start_leg + list(self.survey_route) + end_leg
        return estimate_for_route(self.dtm, self.drawn_polygon, route,
                                  self._params(), chm=self.chm, is_geo=self.is_geo)

    def _rebuild_effective(self):
        """Recompute the effective result from the survey base + home, and redraw.
        Call after the survey or the home point changes."""
        self.result = self._effective_result()
        if self.result is not None:
            self._render_summary(self.result)
            self._render_map_overlays(self.result)
        else:
            self._show_home()
        self._refresh_profile()

    def _params(self):
        return PlanParams(
            altitude_m=self.sp_alt.value(),
            min_peak_clearance_m=_MIN_PEAK_CLEARANCE_M,
            overlap_pct=self.sp_overlap.value(),
            adaptive_spacing=self.cb_adaptive.isChecked(),
            edge_margin=self.cb_edge_margin.isChecked(),
            min_points=self.sp_minpts.value(),
            speed_ms=self.sp_speed.value(),
            pulse_freq_hz=self.cmb_pulse.currentData(),
            scan_freq_hz=self.sp_scanfreq.value(),
            veg_penetration=self.sp_veg.value(),
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
        poly = self.drawn_polygon
        self.lbl_summary.setText(
            '<i style="color:#8b96a3;">Computing route + density estimate…</i>')
        self._set_busy(True, 'Computing route + density estimate…')
        self.setEnabled(False)
        try:
            self.base_result = compute_plan(self.dtm, poly, self._params(),
                                            chm=self.chm, is_geo=self.is_geo)
            self.survey_route = self.base_result.route
            self.result = self._effective_result()
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
        self.btn_helios.setEnabled(has_route)
        self.btn_geojson.setEnabled(has_route)
        self.btn_csv.setEnabled(has_route)
        if self.mapview is not None:
            self.mapview.btn_pass.setEnabled(has_route)
            if has_route:
                self._update_pass_anchor()
        self._set_busy(False)
        if status_msg is None:
            status_msg = (f'Done — {self.result.n_waypoints} waypoints' if has_route
                          else 'Done — no route produced')
        self.statusBar().showMessage(status_msg)

    def _update_pass_anchor(self):
        """Point the map's pass-preview at the survey's current end (the start of
        the next drawn pass — home legs are auto-generated and excluded)."""
        if self.mapview is None:
            return
        last = self._survey_end()
        if last is not None:
            self.mapview.set_pass_anchor(last['x'], last['y'])

    def _on_pass_drawn(self, pt):
        """A click in pass mode: build a pass from the survey's end (start) to the
        clicked point (end), set altitude from terrain, append to the survey, and
        rebuild the effective route + estimate."""
        if not (self.base_result and self.survey_route and self.dtm
                and self.drawn_polygon is not None):
            return
        params = self._params()
        last = self._survey_end()
        if last is None:
            return
        pid = max((w.get('pass_id', 0) for w in self.survey_route
                   if isinstance(w.get('pass_id'), int)), default=-1) + 1
        new_pass = build_manual_pass(self.dtm, (last['x'], last['y']), pt,
                                     params, self.is_geo, pid)
        if not new_pass:
            self.statusBar().showMessage('Drawn pass has no valid terrain — not added.')
            return
        self._set_busy(True, 'Pass added — re-estimating density…')
        try:
            self.survey_route = self.survey_route + new_pass
            self.base_result = estimate_for_route(
                self.dtm, self.drawn_polygon, self.survey_route,
                params, chm=self.chm, is_geo=self.is_geo)
            self.result = self._effective_result()
        except Exception as e:
            QMessageBox.critical(self, 'Re-estimate error', str(e))
            return
        finally:
            self._set_busy(False)
        self._render_summary(self.result)
        self._render_map_overlays(self.result)
        self._update_pass_anchor()
        self._refresh_profile()
        self.statusBar().showMessage(
            f'Pass added at {new_pass[0]["z"]:.0f} m — {self.result.n_waypoints} '
            f'waypoints. Click to add another or untick Add Pass.')

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
        dist, terr, flight = route_profile(
            self._route_with_home(), self.dtm, self.is_geo)
        self.profile_panel.update_profile(dist, terr, flight, agl=self.sp_alt.value())

    # ---------------------------------------------------------------- HELIOS
    def _open_helios(self):
        if not (self.result and self.result.route and self.drawn_polygon is not None):
            return
        from .helios import HeliosDialog
        dlg = HeliosDialog(self, dtm=self.dtm, dtm_path=self.dtm_path,
                           route=self.result.route, polygon=self.drawn_polygon,
                           params=self._params(), chm=self.chm, is_geo=self.is_geo)
        dlg.resultReady.connect(self._on_helios_result)
        self._helios_dlg = dlg          # keep a ref so it isn't GC'd
        dlg.show()

    def _on_helios_result(self, res):
        if self.mapview is None or res.get('error'):
            return
        cells = res.get('failing_cells_geo', [])
        rad = max(float((self.result.estimate or {}).get('cell_size_m', 2.0)), 3.0)
        self.mapview.show_helios(cells, radius_m=rad)

    # ---------------------------------------------------------------- export
    def _export_geojson(self):
        if not (self.result and self.result.route):
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Export route GeoJSON',
                                              'route.geojson', 'GeoJSON (*.geojson)')
        if not path:
            return
        feats = [{
            'type': 'Feature',
            'geometry': {'type': 'Point', 'coordinates': [w['x'], w['y'], w['z']]},
            'properties': {'altitude_m': w['z'],
                           'target_agl_m': w.get('target_distance'),
                           'pass_id': w.get('pass_id'),
                           'role': 'home' if w.get('pass_id') == 'home' else 'survey'},
        } for w in self._route_with_home()
            if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'type': 'FeatureCollection', 'features': feats}, f, indent=2)
        self.statusBar().showMessage(f'Wrote {len(feats)} waypoints → {path}')

    def _export_csv(self):
        if not (self.result and self.result.route):
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Export route CSV',
                                              'route.csv', 'CSV (*.csv)')
        if not path:
            return
        wps = [w for w in self._route_with_home()
               if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        with open(path, 'w', newline='', encoding='utf-8') as f:
            wr = csv.writer(f)
            wr.writerow(['index', 'x', 'y', 'z', 'target_agl_m', 'pass_id', 'role'])
            for i, w in enumerate(wps):
                role = 'home' if w.get('pass_id') == 'home' else 'survey'
                wr.writerow([i, w['x'], w['y'], w['z'],
                             w.get('target_distance'), w.get('pass_id'), role])
        self.statusBar().showMessage(f'Wrote {len(wps)} waypoints → {path}')

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
        self.mapview.show_home(self.home, wps)

    # ---------------------------------------------------------------- render
    def _render_summary(self, r):
        if not r.route:
            self.lbl_summary.setText('No route produced (polygon too small or off the DTM).')
            return
        est = r.estimate or {}
        area = (f'{r.area_m2 / 1e6:.3f} km²' if r.area_m2 >= 1e6
                else f'{r.area_m2:,.0f} m²')
        # path length includes the ferry legs to/from home when one is set
        plen_m = _path_length_m(self._route_with_home(), self.is_geo) \
            if self.home is not None else r.path_len_m
        plen = (f'{plen_m / 1000:.2f} km' if plen_m >= 1000
                else f'{plen_m:.0f} m')
        ncell = max(est.get('n_cells', 0), 1)
        cov = 100.0 * (est.get('n_cells', 0) - est.get('n_fail', 0)) / ncell
        n_passes = len({w.get('pass_id', 0) for w in r.route
                        if not (isinstance(w['z'], float) and math.isnan(w['z']))})

        rows = [
            ('<b>Polygon</b>', ''),
            ('Area', area),
            ('<b>Route</b>', ''),
            ('Passes', f'{n_passes}'),
            ('Waypoints', f'{r.n_waypoints}'),
            ('Path length', plen),
            ('Alt range', f'{r.alt_min:.0f} – {r.alt_max:.0f} m'),
        ]
        if self.home is not None:
            rows.append(('Takeoff/Home',
                         f'{self.home[1]:.5f}, {self.home[0]:.5f}'))
        rows += [
            ('<b>Density estimate</b>', ''),
            ('Coverage', f'{cov:.1f}%'),
            ('Median density', f"{est.get('median_density', 0):.0f} pts/m²"),
            ('Min density', f"{est.get('min_density', 0):.0f} pts/m²"),
        ]
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
