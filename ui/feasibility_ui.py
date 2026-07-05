"""Dialog for mission conditions + one-flight η calibration (feasibility feature).

Kept out of the main sidebar on purpose — it's reached from the Feasibility menu.
Returns the mission inputs (payload / site / weather) and the propulsion efficiency
η, either the uncalibrated default or one solved from a measured flight.
"""
import os
import sys

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox, QLabel, QWidget,
    QDoubleSpinBox, QPushButton, QDialogButtonBox,
)

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src')
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
import feasibility as F                                      # noqa: E402


def _dspin(lo, hi, val, suffix, step, decimals=1):
    s = QDoubleSpinBox()
    s.setRange(lo, hi); s.setValue(val); s.setSuffix(suffix)
    s.setSingleStep(step); s.setDecimals(decimals)
    return s


class FeasibilityDialog(QDialog):
    """Collects mission conditions and (optionally) calibrates η from one flight.

    Calibration reuses the mission site elevation/temperature for air density, so it
    only needs the flight's payload, duration, and battery used."""

    def __init__(self, parent, payload_kg, temp_c, eta, calibrated,
                 calib_elev_m=0.0):
        super().__init__(parent)
        self.setWindowTitle('Mission feasibility & calibration')
        self._eta = eta
        self._calibrated = calibrated
        self._calib_elev = calib_elev_m         # operating altitude, for calib density
        lay = QVBoxLayout(self); lay.setSpacing(8)

        # ── Mission conditions (site elevation is auto-derived from the route/home) ──
        gc = QGroupBox('Mission conditions'); fc = QFormLayout(gc)
        self.sp_payload = _dspin(0, 20, payload_kg, ' kg', 0.5)
        self.sp_temp = _dspin(-30, 50, temp_c, ' °C', 1, 0)
        fc.addRow('Payload', self.sp_payload)
        fc.addRow('Air temperature', self.sp_temp)
        lay.addWidget(gc)

        # ── Calibration ──
        gk = QGroupBox('Calibrate η (optional — one measured flight)')
        fk = QFormLayout(gk)
        self.k_payload = _dspin(0, 20, payload_kg, ' kg', 0.5)
        self.k_duration = _dspin(1, 120, 10, ' min', 1, 0)
        self.k_batt = _dspin(1, 100, 50, ' %', 1, 0)
        fk.addRow('Flight payload', self.k_payload)
        fk.addRow('Duration', self.k_duration)
        fk.addRow('Battery used', self.k_batt)
        row = QHBoxLayout(); row.setContentsMargins(0, 0, 0, 0)
        b_cal = QPushButton('Compute'); b_cal.clicked.connect(self._calibrate)
        b_reset = QPushButton('Reset'); b_reset.clicked.connect(self._reset)
        self.lbl_eta = QLabel()
        row.addWidget(self.lbl_eta, 1); row.addWidget(b_cal); row.addWidget(b_reset)
        fk.addRow(self._wrap(row))
        lay.addWidget(gk)

        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept); bb.rejected.connect(self.reject)
        lay.addWidget(bb)
        self._show_eta()

    def _wrap(self, layout):
        w = QWidget(); w.setLayout(layout); return w

    def _calibrate(self):
        # air density from the operating elevation + the mission temp (keeps it short)
        self._eta = F.calibrate_eta(self.k_payload.value(), self.k_duration.value(),
                                    self.k_batt.value() / 100.0,
                                    self._calib_elev, self.sp_temp.value())
        self._calibrated = True
        self._show_eta()

    def _reset(self):
        self._eta = F.ETA_DEFAULT; self._calibrated = False
        self._show_eta()

    def _show_eta(self):
        tag = 'calibrated' if self._calibrated else f'default ±{F.ETA_BAND * 100:.0f}%'
        self.lbl_eta.setText(f'η = {self._eta:.2f} ({tag})')

    def values(self):
        return {
            'payload_kg': self.sp_payload.value(),
            'temp_c': self.sp_temp.value(),
            'eta': self._eta,
            'calibrated': self._calibrated,
        }
