"""Main window for the desktop planner.

Left: parameter sidebar + Compute. Right: tabbed views — Summary and the 2D
Leaflet Map (draw the AOI, see the route + under-density overlay). Compute runs
the existing model and fills the Summary panel.
"""
import math

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QFormLayout, QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QGroupBox,
    QSplitter, QScrollArea, QFileDialog, QMessageBox, QFrame,
)

from .planning import PlanParams, compute_plan, load_dtm

try:
    from .mapview import MapView
    _MAPVIEW_ERR = None
except Exception as _e:                       # QtWebEngine missing
    MapView = None
    _MAPVIEW_ERR = str(_e)

from shapely.geometry import shape as shapely_shape

PULSE_FREQS = (150_000, 300_000, 600_000, 1_200_000, 1_800_000, 2_400_000)


def _hr():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setFrameShadow(QFrame.Sunken)
    return line


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
        self.result = None
        self.drawn_polygon = None        # shapely Polygon drawn on the map

        self._build_menu()
        self._build_body()
        self.statusBar().showMessage('Open a DTM to begin.')

    # ---------------------------------------------------------------- UI build
    def _build_menu(self):
        m = self.menuBar().addMenu('&File')
        a_dtm = QAction('Open DTM…', self); a_dtm.triggered.connect(self._open_dtm)
        a_chm = QAction('Open CHM…', self); a_chm.triggered.connect(self._open_chm)
        a_quit = QAction('Quit', self); a_quit.triggered.connect(self.close)
        m.addAction(a_dtm); m.addAction(a_chm); m.addSeparator(); m.addAction(a_quit)

    def _build_body(self):
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_sidebar())      # params (left)
        splitter.addWidget(self._build_map())          # map (center)
        splitter.addWidget(self._build_summary())      # results (right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([340, 760, 300])
        self.setCentralWidget(splitter)

    def _build_sidebar(self):
        panel = QWidget()
        v = QVBoxLayout(panel)

        # ── Data ──
        gb_data = QGroupBox('Data')
        dl = QVBoxLayout(gb_data)
        self.lbl_dtm = QLabel('DTM: (none)'); self.lbl_dtm.setWordWrap(True)
        self.lbl_chm = QLabel('CHM: (none)'); self.lbl_chm.setWordWrap(True)
        b_dtm = QPushButton('Open DTM…'); b_dtm.clicked.connect(self._open_dtm)
        chm_row = QHBoxLayout()
        b_chm = QPushButton('Open CHM…'); b_chm.clicked.connect(self._open_chm)
        b_chm_clear = QPushButton('Clear'); b_chm_clear.clicked.connect(self._clear_chm)
        chm_row.addWidget(b_chm); chm_row.addWidget(b_chm_clear)
        dl.addWidget(self.lbl_dtm); dl.addWidget(b_dtm)
        dl.addWidget(self.lbl_chm); dl.addLayout(chm_row)
        v.addWidget(gb_data)

        # ── AOI ──
        gb_aoi = QGroupBox('AOI')
        al = QVBoxLayout(gb_aoi)
        self.lbl_aoi = QLabel('Draw a polygon on the map to set the AOI.')
        self.lbl_aoi.setWordWrap(True); self.lbl_aoi.setStyleSheet('color:#888;')
        b_clear_aoi = QPushButton('Clear drawn AOI')
        b_clear_aoi.clicked.connect(self._clear_aoi)
        al.addWidget(self.lbl_aoi); al.addWidget(b_clear_aoi)
        v.addWidget(gb_aoi)

        # ── Flight ──
        gb_flight = QGroupBox('Flight')
        fl = QFormLayout(gb_flight)
        self.sp_alt = self._dspin(1, 1000, 100, ' m', 5)
        self.sp_overlap = self._dspin(0, 99, 20, ' %', 5)
        self.cb_adaptive = QCheckBox('Terrain-adaptive spacing'); self.cb_adaptive.setChecked(True)
        self.sp_step = self._dspin(1, 500, 50, ' m', 5)
        fl.addRow('Altitude AGL', self.sp_alt)
        fl.addRow('Overlap', self.sp_overlap)
        fl.addRow('', self.cb_adaptive)
        fl.addRow('Along-track step', self.sp_step)
        v.addWidget(gb_flight)

        # ── Scanner & density ──
        gb_scan = QGroupBox('Scanner & density')
        scl = QFormLayout(gb_scan)
        self.sp_minpts = QSpinBox(); self.sp_minpts.setRange(1, 100000); self.sp_minpts.setValue(100)
        self.sp_speed = self._dspin(0.1, 50, 6.0, ' m/s', 0.5)
        self.cmb_pulse = QComboBox()
        for f in PULSE_FREQS:
            self.cmb_pulse.addItem(f'{f:,}', f)
        self.cmb_pulse.setCurrentText('600,000')
        self.sp_scanfreq = self._dspin(1, 5000, 224.4, ' Hz', 10)
        self.sp_veg = self._dspin(0, 1, 0.4, '', 0.05); self.sp_veg.setDecimals(2)
        scl.addRow('Min points / m²', self.sp_minpts)
        scl.addRow('Drone speed', self.sp_speed)
        scl.addRow('Pulse freq', self.cmb_pulse)
        scl.addRow('Scan freq', self.sp_scanfreq)
        scl.addRow('Canopy ground-return frac', self.sp_veg)
        scl.addRow(QLabel('FOV fixed at 100° (±50°)'))
        v.addWidget(gb_scan)

        self.btn_compute = QPushButton('Compute Route')
        self.btn_compute.setEnabled(False)
        self.btn_compute.clicked.connect(self._compute)
        v.addWidget(self.btn_compute)
        v.addStretch(1)

        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(panel)
        scroll.setMinimumWidth(340)
        return scroll

    def _dspin(self, lo, hi, val, suffix, step):
        s = QDoubleSpinBox(); s.setRange(lo, hi); s.setValue(val)
        s.setSuffix(suffix); s.setSingleStep(step)
        return s

    def _build_map(self):
        # Map view (Leaflet in QWebEngineView) — draw the AOI here.
        if MapView is not None:
            self.mapview = MapView()
            self.mapview.polygonDrawn.connect(self._on_polygon_drawn)
            return self.mapview
        self.mapview = None
        return self._stub('🗺  Map view needs QtWebEngine.\n' + (_MAPVIEW_ERR or ''))

    def _build_summary(self):
        panel = QWidget(); sv = QVBoxLayout(panel)
        title = QLabel('<b>Results</b>')
        self.lbl_summary = QLabel('Compute a route to see results.')
        self.lbl_summary.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.lbl_summary.setTextFormat(Qt.RichText)
        self.lbl_summary.setWordWrap(True)
        sv.addWidget(title); sv.addWidget(self.lbl_summary); sv.addStretch(1)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(panel)
        scroll.setMinimumWidth(260)
        return scroll

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
        self.drawn_polygon = None
        self.btn_compute.setEnabled(False)
        self.lbl_aoi.setText('Draw a polygon on the map to set the AOI.')
        self._refresh_map()
        self.statusBar().showMessage('DTM loaded. Draw an AOI on the map, '
                                     'then Compute.')

    def _open_chm(self):
        if self.dtm is None:
            QMessageBox.information(self, 'CHM', 'Open a DTM first.'); return
        path, _ = QFileDialog.getOpenFileName(
            self, 'Open CHM', '', 'GeoTIFF (*.tif *.tiff);;All files (*)')
        if not path:
            return
        try:
            self.chm = load_dtm(path)
        except Exception as e:
            QMessageBox.critical(self, 'CHM error', str(e)); return
        self.chm_path = path
        self.lbl_chm.setText(f'CHM: {path}')
        self._refresh_map()

    def _clear_chm(self):
        self.chm = None; self.chm_path = None
        self.lbl_chm.setText('CHM: (none)')
        self._refresh_map()

    def _refresh_map(self):
        if self.mapview is not None and self.dtm is not None:
            self.mapview.set_dtm(self.dtm, self.dtm_path, self.chm, self.chm_path)

    def _on_polygon_drawn(self, geom):
        try:
            poly = shapely_shape(geom)
            if not poly.is_valid:
                poly = poly.buffer(0)
        except Exception as e:
            self.statusBar().showMessage(f'Bad polygon: {e}'); return
        self.drawn_polygon = poly
        self.lbl_aoi.setText('✓ AOI set from the drawn polygon.')
        self.btn_compute.setEnabled(True)
        self.statusBar().showMessage('AOI set from drawn polygon. Click Compute.')

    def _clear_aoi(self):
        self.drawn_polygon = None
        self.btn_compute.setEnabled(False)
        self.lbl_aoi.setText('Draw a polygon on the map to set the AOI.')
        self._refresh_map()

    def _params(self):
        return PlanParams(
            altitude_m=self.sp_alt.value(),
            overlap_pct=self.sp_overlap.value(),
            adaptive_spacing=self.cb_adaptive.isChecked(),
            step_m=self.sp_step.value(),
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
        self.statusBar().showMessage('Computing…')
        self.setEnabled(False)
        try:
            self.result = compute_plan(self.dtm, poly, self._params(),
                                       chm=self.chm, is_geo=self.is_geo)
        except Exception as e:
            self.setEnabled(True)
            QMessageBox.critical(self, 'Compute error', str(e))
            self.statusBar().showMessage('Compute failed.')
            return
        self.setEnabled(True)
        self._render_summary(self.result)
        self._render_map_overlays(self.result)
        self.statusBar().showMessage('Done.')

    def _render_map_overlays(self, r):
        if self.mapview is None:
            return
        wps = [w for w in r.route
               if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        est = r.estimate or {}
        cells = est.get('failing_cells_geo', [])
        rad = max(float(est.get('cell_size_m', 2.0)), 3.0)
        self.mapview.show_plan(wps, cells, density_color='#ff9900',
                               density_radius_m=rad)

    # ---------------------------------------------------------------- render
    def _render_summary(self, r):
        if not r.route:
            self.lbl_summary.setText('No route produced (AOI too small or off the DTM).')
            return
        est = r.estimate or {}
        area = (f'{r.area_m2 / 1e6:.3f} km²' if r.area_m2 >= 1e6
                else f'{r.area_m2:,.0f} m²')
        plen = (f'{r.path_len_m / 1000:.2f} km' if r.path_len_m >= 1000
                else f'{r.path_len_m:.0f} m')
        ncell = max(est.get('n_cells', 0), 1)
        cov = 100.0 * (est.get('n_cells', 0) - est.get('n_fail', 0)) / ncell

        rows = [
            ('<b>Polygon</b>', ''),
            ('Area', area),
            ('<b>Route</b>', ''),
            ('Waypoints', f'{r.n_waypoints}'),
            ('Path length', plen),
            ('Alt range', f'{r.alt_min:.0f} – {r.alt_max:.0f} m'),
            ('<b>Density estimate</b>', ''),
            ('Coverage', f'{cov:.1f}%'),
            ('Median density', f"{est.get('median_density', 0):.0f} pts/m²"),
            ('Min density', f"{est.get('min_density', 0):.0f} pts/m²"),
        ]
        html = ['<table cellspacing=6>']
        for k, val in rows:
            html.append(f'<tr><td>{k}</td><td><b>{val}</b></td></tr>')
        html.append('</table>')
        self.lbl_summary.setText(''.join(html))
