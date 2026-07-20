"""Main window for the desktop planner.

Left: parameter sidebar + Compute. Right: tabbed views — Summary and the 2D
Leaflet Map (draw the AOI, see the route + under-density overlay). Compute runs
the existing model and fills the Summary panel.
"""
import csv
import json
import math

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
                       estimate_for_route, _path_length_m, _LAT_M)

try:
    from .canvasmap import CanvasMap, FAILURE_REASON_STYLE, FAILURE_REASON_LABEL
    _MAPVIEW_ERR = None
except Exception as _e:
    CanvasMap = None
    _MAPVIEW_ERR = str(_e)
    # Qt-free fallback so the summary legend still renders without the map backend.
    FAILURE_REASON_STYLE = {
        "range": ("#e5484d", 130), "shadow": ("#8250df", 120),
        "thin": ("#ff9900", 95), "gap": ("#8c959f", 120),
    }
    FAILURE_REASON_LABEL = {
        "range": ("Beyond scanner range", "lower AGL or PRR"),
        "shadow": ("Occlusion shadow", "needs a cross-pass, or accept"),
        "thin": ("Thin (under target)", "lower AGL / tighter AOI"),
        "gap": ("Not covered", "spacing / AOI edge"),
    }

from shapely.geometry import shape as shapely_shape

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
        self.home = None                 # takeoff/return-home (lon, lat) or None
        self.home_ground = float('nan')  # terrain elevation at home, if in the DTM

        # mission-feasibility inputs (edited via the Feasibility menu)
        from feasibility import ETA_DEFAULT
        # site elevation is auto-derived from the route/home, not stored here
        self.feas = {'payload_kg': 3.0, 'temp_c': 15.0,
                     'eta': ETA_DEFAULT, 'calibrated': False}

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
        a_quit = QAction('Quit', self); a_quit.triggered.connect(self.close)
        m.addAction(a_dtm); m.addAction(a_chm); m.addSeparator(); m.addAction(a_quit)

        mv = self.menuBar().addMenu('&View')
        self.act_profile = QAction('Elevation profile', self, checkable=True)
        self.act_profile.setChecked(False)
        self.act_profile.setShortcut('Ctrl+E')
        self.act_profile.toggled.connect(self._toggle_profile)
        mv.addAction(self.act_profile)

        mf = self.menuBar().addMenu('&Feasibility')
        a_feas = QAction('Mission conditions & calibration…', self)
        a_feas.triggered.connect(self._open_feasibility)
        mf.addAction(a_feas)

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
        self.lbl_aoi = QLabel('Draw a polygon on the map to set the AOI.')
        self.lbl_aoi.setWordWrap(True); self.lbl_aoi.setStyleSheet('color:#888;')
        b_enter_aoi = QPushButton('Enter coordinates…')
        b_enter_aoi.clicked.connect(self._enter_aoi_coords)
        b_clear_aoi = QPushButton('Clear drawn AOI')
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
        self.cb_edge_margin = QCheckBox('Edge fly-past (cover AOI rim)')
        self.cb_edge_margin.setToolTip(
            'Extend passes one pass-pitch beyond the AOI so edge cells get full '
            'overlap (removes the boundary coverage gap). Flies slightly outside the AOI.')
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
        s.setValue('feas/payload_kg', self.feas['payload_kg'])
        s.setValue('feas/temp_c', self.feas['temp_c'])
        s.setValue('feas/eta', self.feas['eta'])           # measured — worth keeping
        s.setValue('feas/calibrated', self.feas['calibrated'])
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
        self.feas['payload_kg'] = s.value(
            'feas/payload_kg', self.feas['payload_kg'], type=float)
        self.feas['temp_c'] = s.value('feas/temp_c', self.feas['temp_c'], type=float)
        self.feas['eta'] = s.value('feas/eta', self.feas['eta'], type=float)
        self.feas['calibrated'] = s.value(
            'feas/calibrated', self.feas['calibrated'], type=bool)
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
            return self.mapview
        self.mapview = None
        return self._stub('🗺  Map canvas failed to load.\n' + (_MAPVIEW_ERR or ''))

    def _build_summary(self):
        panel = QWidget(); sv = QVBoxLayout(panel)
        title = QLabel('<b>Results</b>')

        # Feasibility verdict banner — the redesign's headline element. Colour-coded
        # (success / warning / danger via a dynamic "state" property) so the go/no-go
        # answer is scannable before any other metric. Hidden until a route exists.
        self.verdict_banner = QFrame(); self.verdict_banner.setObjectName('verdictBanner')
        vb = QVBoxLayout(self.verdict_banner)
        vb.setContentsMargins(14, 12, 14, 12); vb.setSpacing(6)
        self.verdict_headline = QLabel(); self.verdict_headline.setObjectName('verdictHeadline')
        self.verdict_reason = QLabel(); self.verdict_reason.setObjectName('verdictReason')
        self.verdict_reason.setWordWrap(True)
        self.energy_bar = QProgressBar(); self.energy_bar.setObjectName('energyBar')
        self.energy_bar.setTextVisible(False); self.energy_bar.setRange(0, 100)
        self.energy_bar.setFixedHeight(6)
        vb.addWidget(self.verdict_headline); vb.addWidget(self.verdict_reason)
        vb.addWidget(self.energy_bar)
        self.verdict_banner.setVisible(False)

        self.lbl_summary = QLabel('Compute a route to see results.')
        self.lbl_summary.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.lbl_summary.setTextFormat(Qt.RichText)
        self.lbl_summary.setWordWrap(True)
        sv.addWidget(title); sv.addWidget(self.verdict_banner)
        sv.addWidget(self.lbl_summary); sv.addStretch(1)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(panel)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)  # wrap, don't scroll sideways
        scroll.setMinimumWidth(260)
        return scroll

    @staticmethod
    def _set_state(widget, state):
        """Set the dynamic 'state' property (success/warning/danger) and re-polish
        so the QSS attribute selectors re-apply."""
        widget.setProperty('state', state)
        widget.style().unpolish(widget); widget.style().polish(widget)

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
        h, w = self.dtm.array.shape
        self.lbl_dtm.setText(f'DTM: {path}\n{w}×{h} px · CRS {crs}')
        self.home = None; self.home_ground = float('nan')   # new area
        self.lbl_home.setText('Home: (none)')
        self.drawn_polygon = None
        self.btn_compute.setEnabled(False)
        self.lbl_aoi.setText('Draw a polygon on the map to set the AOI.')
        self._clear_results()
        self._refresh_map()
        self.statusBar().showMessage('DTM loaded. Draw an AOI on the map, '
                                     'then Compute.')

    def _clear_dtm(self):
        """Drop the loaded DTM (and the CHM/AOI/results that depend on it) and
        blank the map — mirrors the CHM Clear."""
        self.dtm = None; self.dtm_path = None
        self.chm = None; self.chm_path = None
        self.is_geo = True
        self.drawn_polygon = None
        self.home = None; self.home_ground = float('nan')
        self.lbl_dtm.setText('DTM: (none)')
        self.lbl_chm.setText('CHM: (none)')
        self.lbl_home.setText('Home: (none)')
        self.lbl_aoi.setText('Draw a polygon on the map to set the AOI.')
        self.btn_compute.setEnabled(False)
        self._clear_results()
        if self.mapview is not None:
            self.mapview.clear()
        self.statusBar().showMessage('Open a DTM to begin.')

    def _enter_aoi_coords(self):
        """Set the AOI from manually-typed vertices instead of drawing on the map."""
        if self.dtm is None:
            QMessageBox.information(self, 'AOI', 'Open a DTM first.'); return
        from PySide6.QtWidgets import QDialog, QPlainTextEdit, QDialogButtonBox
        dlg = QDialog(self); dlg.setWindowTitle('Enter AOI polygon')
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
        self.lbl_aoi.setText('✓ AOI set from entered coordinates.')

    def _parse_coords(self, text):
        """Parse 'lat, lon' lines into a list of [lon, lat] vertices, or None."""
        coords = []
        for ln in text.splitlines():
            ln = ln.strip()
            if not ln:
                continue
            parts = ln.replace(',', ' ').split()
            if len(parts) < 2:
                QMessageBox.warning(self, 'AOI', f'Bad line: "{ln}"\nUse: lat, lon')
                return None
            try:
                lat, lon = float(parts[0]), float(parts[1])
            except ValueError:
                QMessageBox.warning(self, 'AOI', f'Not numbers: "{ln}"')
                return None
            coords.append([lon, lat])
        if len(coords) < 3:
            QMessageBox.warning(self, 'AOI', 'Enter at least 3 vertices.')
            return None
        return coords

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
            finally:
                self._set_busy(False)

    def _clear_results(self):
        """Drop the computed route/estimate and its on-screen traces, so stale
        results never linger next to a changed (or absent) route."""
        self.result = None
        self.base_result = None
        self.survey_route = []
        self.lbl_summary.setText(self._empty_summary_html())
        self._update_workflow()
        if getattr(self, 'verdict_banner', None) is not None:
            self.verdict_banner.setVisible(False)
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
        self.drawn_polygon = poly
        self._clear_results()                 # previous route no longer matches AOI
        self.lbl_aoi.setText('✓ AOI set from the drawn polygon.')
        self.btn_compute.setEnabled(True)
        self.statusBar().showMessage('AOI set from drawn polygon. Click Compute.')

    def _clear_aoi(self):
        self.drawn_polygon = None
        self.btn_compute.setEnabled(False)
        self.lbl_aoi.setText('Draw a polygon on the map to set the AOI.')
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
            QMessageBox.information(self, 'AOI', 'Draw a polygon on the map first.')
            return
        poly = self.drawn_polygon
        self.verdict_banner.setVisible(False)
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
        self.setEnabled(True)
        self._render_summary(self.result)
        self._render_map_overlays(self.result)
        self._refresh_profile()
        has_route = bool(self.result.route)
        self.btn_helios.setEnabled(has_route)
        self.btn_geojson.setEnabled(has_route)
        self.btn_csv.setEnabled(has_route)
        if self.mapview is not None:
            self.mapview.btn_pass.setEnabled(has_route)
            if has_route:
                self._update_pass_anchor()
        self._set_busy(False)
        self.statusBar().showMessage(
            f'Done — {self.result.n_waypoints} waypoints' if has_route
            else 'Done — no route produced')

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

    # ------------------------------------------------------------- feasibility
    def _open_feasibility(self):
        from .feasibility_ui import FeasibilityDialog
        air, _ = self._feas_elevations()          # for the calibration air density
        dlg = FeasibilityDialog(self, self.feas['payload_kg'], self.feas['temp_c'],
                                self.feas['eta'], self.feas['calibrated'], air)
        if dlg.exec():                     # Accepted == 1 (truthy), Rejected == 0
            self.feas = dlg.values()
            if self.result is not None:
                self._render_summary(self.result)

    def _feas_elevations(self):
        """(operating_amsl, takeoff_amsl) auto-derived: operating = mean flight
        altitude of the route (drives air density). takeoff = home ground only when
        a home is set (drives the MTOW derate); None otherwise — with no home the
        estimate is based purely on the polygon route and the MTOW gate falls back
        to the operating altitude. (0, None) when there's no route."""
        if not (self.result and self.result.route and self.dtm):
            return 0.0, None
        wps = [w for w in self._route_with_home()
               if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        if not wps:
            return 0.0, None
        air = sum(w['z'] for w in wps) / len(wps)
        takeoff = (self.home_ground
                   if self.home is not None and not math.isnan(self.home_ground)
                   else None)
        return air, takeoff

    def _feasibility(self):
        """FeasibilityResult for the current flown route, or None if no route."""
        if not (self.result and self.result.route and self.dtm):
            return None
        import feasibility as F
        air, takeoff = self._feas_elevations()
        return F.estimate_feasibility(
            self._route_with_home(), is_geo=self.is_geo,
            payload_kg=self.feas['payload_kg'], cruise_ms=self.sp_speed.value(),
            site_elev_m=air, temp_c=self.feas['temp_c'],
            eta=self.feas['eta'],
            home=self.home, terrain_at=self.dtm.elevation_at,
            takeoff_elev_m=takeoff)

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
        by_reason = est.get('failing_cells_by_reason')
        if by_reason:
            self.mapview.show_plan(wps, None, density_radius_m=rad,
                                   cells_by_reason=by_reason)
        else:                       # older result shape: single-colour fallback
            self.mapview.show_plan(wps, est.get('failing_cells_geo', []),
                                   density_color='#ff9900', density_radius_m=rad)
        self.mapview.show_home(self.home, wps)

    # ---------------------------------------------------------------- render
    def _render_summary(self, r):
        if not r.route:
            self.lbl_summary.setText('No route produced (AOI too small or off the DTM).')
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
        _reason_counts = {'range': est.get('n_beyond_range', 0),
                          'shadow': est.get('n_shadow', 0),
                          'thin': est.get('n_thin', 0),
                          'gap': est.get('n_gap', 0)}
        if any(_reason_counts.values()):
            rows.append(('<b>Why cells fail</b>', ''))
            for key in ('range', 'shadow', 'thin', 'gap'):
                n = _reason_counts[key]
                if not n:
                    continue
                hexc = FAILURE_REASON_STYLE[key][0]
                label, lever = FAILURE_REASON_LABEL[key]
                rows.append(
                    (f'<span style="color:{hexc}">■</span> {label}',
                     f'{n:,} cells · <i>{lever}</i>'))

        fr = self._feasibility()
        if fr is not None:
            batt = (f'{fr.batteries_needed} batteries'
                    if fr.batteries_needed > 1 else '1 battery')
            cal = 'calibrated' if self.feas['calibrated'] else 'uncalibrated ±band'
            # ── verdict banner ──
            if fr.robust:
                state, head = 'success', '✓ Feasible'
            elif fr.feasible:
                state, head = 'warning', '~ Feasible (nominal only)'
            else:
                state, head = 'danger', '✗ Over energy budget'
            self._set_state(self.verdict_banner, state)
            self._set_state(self.verdict_headline, state)
            self._set_state(self.energy_bar, state)
            self.verdict_headline.setText(head)
            self.verdict_reason.setText(
                f'Needs {fr.energy_wh:.0f} Wh vs {fr.usable_wh:.0f} Wh usable on '
                f'{batt} — {fr.margin_pct:+.0f}% margin.')
            self.energy_bar.setValue(
                int(min(100, round(100.0 * fr.energy_wh / max(fr.usable_wh, 1.0)))))
            self.verdict_banner.setVisible(True)
            # ── supporting detail (demoted below the banner) ──
            rows += [
                ('<b>Feasibility (Thor)</b>', ''),
                ('Flight time', f'{fr.flight_time_s / 60:.0f} min'),
                ('Payload', f"{self.feas['payload_kg']:.1f} kg"),
            ]
            if self.home is not None:      # takeoff ground only matters with a home
                rows.append(('Takeoff alt', f'{fr.takeoff_elev_m:.0f} m'))
            rows += [
                ('Energy need', f'{fr.energy_wh:.0f} Wh ({cal})'),
                ('Usable / batteries', f'{fr.usable_wh:.0f} Wh · {batt}'),
            ]
        else:
            self.verdict_banner.setVisible(False)

        html = ['<table cellspacing=6>']
        for k, val in rows:
            html.append(f'<tr><td>{k}</td><td><b>{val}</b></td></tr>')
        html.append('</table>')
        if fr is not None and fr.gates:
            html.append('<div style="color:#e6c98a;margin-top:6px;">⚠ '
                        + '<br>⚠ '.join(fr.gates) + '</div>')
        self.lbl_summary.setText(''.join(html))
