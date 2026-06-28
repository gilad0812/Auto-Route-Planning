"""HELIOS++ validation for the desktop app.

A QThread worker builds the terrain mesh and runs the HELIOS++ simulation
(off the UI thread), streaming log lines and returning the result dict. A
non-modal dialog drives it: pick the pre-installed binary (auto-detected),
set mesh resolution, Run/Stop, watch progress, then download the trajectory /
survey XML. Offline appliance: no auto-install — the binary must already exist.
"""
import math
import os
import sys
import shutil
import tempfile
import threading

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src')
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel, QLineEdit,
    QPushButton, QDoubleSpinBox, QPlainTextEdit, QFileDialog, QMessageBox,
)

from helios_integration import run_feedback_loop          # noqa: E402
from terrain_converter import dtm_to_obj                  # noqa: E402
from helios_setup import find_helios_binary               # noqa: E402
from helios_config import DEFAULT_SCANNER_REF, DEFAULT_PLATFORM_REF  # noqa: E402


def _valid_wps(route):
    return [w for w in route
            if not (isinstance(w['z'], float) and math.isnan(w['z']))]


class HeliosWorker(QThread):
    log = Signal(str)
    done = Signal(dict)

    def __init__(self, *, dtm, dtm_path, route, polygon, params, chm, is_geo,
                 helios_bin, mesh_step_m, work_dir):
        super().__init__()
        self.dtm = dtm; self.dtm_path = dtm_path
        self.route = route; self.polygon = polygon; self.params = params
        self.chm = chm; self.is_geo = is_geo
        self.helios_bin = helios_bin; self.mesh_step_m = mesh_step_m
        self.work_dir = work_dir
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()
        self.log.emit('Stop requested — waiting for HELIOS++ to terminate…')

    def run(self):
        try:
            p = self.params
            wps = _valid_wps(self.route)
            ref_lon = sum(w['x'] for w in wps) / len(wps)
            ref_lat = sum(w['y'] for w in wps) / len(wps)
            half = p.fov_deg / 2.0
            swath_m = 2.0 * p.altitude_m * math.tan(math.radians(half))

            obj = os.path.join(self.work_dir, 'terrain.obj')
            self.log.emit(f'Building terrain mesh (step {self.mesh_step_m:.1f} m)…')
            dtm_to_obj(self.dtm_path, obj, step_m=float(self.mesh_step_m),
                       ref_lon=ref_lon, ref_lat=ref_lat,
                       crop_bounds=self.polygon.bounds, margin_m=swath_m)
            if self._stop.is_set():
                self.done.emit({'passed': False, 'error': 'cancelled'}); return

            self.log.emit('Running HELIOS++ simulation…')
            region = list(self.polygon.exterior.coords)
            res = run_feedback_loop(
                route=self.route, helios_bin=self.helios_bin,
                scene_obj_path=obj, work_dir=self.work_dir, is_geo=self.is_geo,
                ref_lon=ref_lon, ref_lat=ref_lat, altitude_m=p.altitude_m,
                min_points=int(p.min_points), speed_ms=float(p.speed_ms),
                pulse_freq_hz=int(p.pulse_freq_hz), scan_freq_hz=float(p.scan_freq_hz),
                scan_angle_deg=half, scanner_ref=DEFAULT_SCANNER_REF,
                platform_ref=DEFAULT_PLATFORM_REF, dtm=self.dtm,
                region_polygon=region, chm=self.chm,
                veg_penetration=float(p.veg_penetration),
                log=lambda m: self.log.emit(m), stop_event=self._stop,
            )
            self.done.emit(res)
        except Exception as e:
            self.done.emit({'passed': False, 'error': str(e)})


class HeliosDialog(QDialog):
    """Drives one HELIOS++ validation run. Emits resultReady(dict) so the main
    window can paint the HELIOS under-density cells (red) on the map."""
    resultReady = Signal(dict)

    def __init__(self, parent, *, dtm, dtm_path, route, polygon, params, chm, is_geo):
        super().__init__(parent)
        self.setWindowTitle('HELIOS++ Validation')
        self.resize(620, 520)
        self.dtm = dtm; self.dtm_path = dtm_path; self.route = route
        self.polygon = polygon; self.params = params; self.chm = chm
        self.is_geo = is_geo
        self.worker = None
        self.result = None

        v = QVBoxLayout(self)
        form = QFormLayout()
        self.ed_bin = QLineEdit()
        found = find_helios_binary()
        if found:
            self.ed_bin.setText(str(found))
        b_browse = QPushButton('Browse…'); b_browse.clicked.connect(self._browse)
        binrow = QHBoxLayout(); binrow.addWidget(self.ed_bin); binrow.addWidget(b_browse)
        form.addRow('HELIOS++ binary', binrow)
        self.sp_mesh = QDoubleSpinBox(); self.sp_mesh.setRange(0.5, 20)
        self.sp_mesh.setValue(3.0); self.sp_mesh.setSuffix(' m')
        form.addRow('Mesh vertex spacing', self.sp_mesh)
        v.addLayout(form)

        btns = QHBoxLayout()
        self.btn_run = QPushButton('Run Validation'); self.btn_run.clicked.connect(self._run)
        self.btn_stop = QPushButton('Stop'); self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self._stop)
        btns.addWidget(self.btn_run); btns.addWidget(self.btn_stop); btns.addStretch(1)
        v.addLayout(btns)

        self.lbl_result = QLabel(''); self.lbl_result.setTextFormat(Qt.RichText)
        self.lbl_result.setWordWrap(True)
        v.addWidget(self.lbl_result)

        self.log = QPlainTextEdit(); self.log.setReadOnly(True)
        self.log.setStyleSheet('font-family:monospace;font-size:11px;')
        v.addWidget(self.log, 1)

        dl = QHBoxLayout()
        self.btn_traj = QPushButton('Save trajectory…'); self.btn_traj.setEnabled(False)
        self.btn_traj.clicked.connect(lambda: self._save('trajectory_path', 'trajectory.txt'))
        self.btn_xml = QPushButton('Save survey XML…'); self.btn_xml.setEnabled(False)
        self.btn_xml.clicked.connect(lambda: self._save('survey_xml_path', 'survey.xml'))
        dl.addWidget(self.btn_traj); dl.addWidget(self.btn_xml); dl.addStretch(1)
        v.addLayout(dl)

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(self, 'HELIOS++ binary', '',
                                              'Executable (*.exe);;All files (*)')
        if path:
            self.ed_bin.setText(path)

    def _run(self):
        helios_bin = self.ed_bin.text().strip()
        if not helios_bin or not os.path.exists(helios_bin):
            QMessageBox.warning(self, 'HELIOS++', 'Set a valid HELIOS++ binary path.')
            return
        if not self.route or self.polygon is None:
            QMessageBox.warning(self, 'HELIOS++', 'Compute a route first.')
            return
        work = os.path.join(tempfile.gettempdir(), 'helios_autoroute_desktop')
        shutil.rmtree(work, ignore_errors=True); os.makedirs(work, exist_ok=True)
        self.log.clear(); self.lbl_result.setText('')
        self.btn_run.setEnabled(False); self.btn_stop.setEnabled(True)
        self.btn_traj.setEnabled(False); self.btn_xml.setEnabled(False)

        self.worker = HeliosWorker(
            dtm=self.dtm, dtm_path=self.dtm_path, route=self.route,
            polygon=self.polygon, params=self.params, chm=self.chm,
            is_geo=self.is_geo, helios_bin=helios_bin,
            mesh_step_m=self.sp_mesh.value(), work_dir=work)
        self.worker.log.connect(self._append)
        self.worker.done.connect(self._finished)
        self.worker.start()

    def _stop(self):
        if self.worker is not None:
            self.worker.stop()

    def _append(self, msg):
        self.log.appendPlainText(msg)

    def _finished(self, res):
        self.result = res
        self.btn_run.setEnabled(True); self.btn_stop.setEnabled(False)
        if res.get('error'):
            self.lbl_result.setText(f'<span style="color:#cf222e"><b>Error:</b> '
                                    f'{res["error"]}</span>')
            self.resultReady.emit(res); return
        stats = res.get('density_stats') or {}
        n_fail = len(res.get('failing_cells_geo', []))
        if stats.get('n_cells'):
            cov = 100.0 * stats['n_met'] / stats['n_cells']
            void_pct = 100.0 * stats['n_void'] / stats['n_cells']
            summary = (f"Coverage <b>{cov:.1f}%</b> · median "
                       f"<b>{stats['median_density']:.0f}</b> pts/m² · voids "
                       f"<b>{stats['n_void']} ({void_pct:.1f}%)</b>")
        else:
            summary = ''
        head = ('<span style="color:#1a7f37"><b>✓ Density validated</b></span>'
                if res.get('passed')
                else f'<span style="color:#cf222e"><b>⚠ {n_fail} under-density '
                     f'cell(s)</b></span> (red on map)')
        self.lbl_result.setText(head + '<br>' + summary)
        self.btn_traj.setEnabled(bool(res.get('trajectory_path')))
        self.btn_xml.setEnabled(bool(res.get('survey_xml_path')))
        self.resultReady.emit(res)

    def _save(self, key, default_name):
        src = (self.result or {}).get(key)
        if not src or not os.path.exists(src):
            return
        dst, _ = QFileDialog.getSaveFileName(self, 'Save', default_name)
        if dst:
            shutil.copyfile(src, dst)
