"""Elevation / clearance profile along the flown route.

Samples the terrain under the route and plots it against the flight line so the
operator can see clearance per pass and spot passes that dip toward (or through)
the ground. Terrain-vs-flight-line is exactly the check that reveals a constant
per-pass altitude clipping a ridge. Lives as a full-width strip below the map and
redraws whenever the route changes.
"""
import math

import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

_LAT_M = 111139.0


def route_profile(route, dtm, is_geo=True, sample_step_m=None, join_passes=True):
    """Sample terrain + flight altitude along the flown route (ordered waypoints
    with x, y, z, pass_id). Returns (dist_m, terrain_m, flight_m, spans), lists in
    metres plus `spans` = [(start_dist, end_dist, pass_id)] — the x-range each pass
    occupies, so a click on the profile maps back to a pass. Turnaround/connector
    samples get pass_id None (not clickable).

    NaN-z waypoints are dropped; flight altitude is linear between kept waypoints —
    flat within a pass (equal endpoint z) and a climb/descent across a turn. Terrain
    is NaN where the path leaves the DTM (e.g. a ferry outside the tile).

    join_passes=False (independent uploaded passes): the leg between two DIFFERENT
    passes is dropped — no pseudo-pass connector (the flight line does not link them)
    and no gap: the line just breaks and the next pass is laid right after this one.
    join_passes=True keeps the route continuous (auto-planned routes, where the
    turnaround/ferry clearance matters)."""
    wps = [w for w in route
           if not (isinstance(w['z'], float) and math.isnan(w['z']))]
    if len(wps) < 2:
        return [], [], [], []
    lat0 = sum(w['y'] for w in wps) / len(wps)
    lon_m = _LAT_M * math.cos(math.radians(lat0)) if is_geo else 1.0
    lat_m = _LAT_M if is_geo else 1.0
    res_m = min(abs(dtm.src.res[0]), abs(dtm.src.res[1])) * (lat_m if is_geo else 1.0)
    step = sample_step_m or max(res_m, 2.0)

    dist, terr, flight, pids, acc = [], [], [], [], 0.0
    for a, b in zip(wps, wps[1:]):
        seg_m = math.hypot((b['x'] - a['x']) * lon_m, (b['y'] - a['y']) * lat_m)
        same_pass = a.get('pass_id') == b.get('pass_id')
        if not join_passes and not same_pass:
            # Independent passes: drop the connector leg. Break the line (NaN) but
            # DON'T advance the x-axis — the next pass sits directly after this one.
            dist.append(acc); terr.append(float('nan')); flight.append(float('nan'))
            pids.append(None)
            continue
        seg_pid = a.get('pass_id') if same_pass else None   # None = turnaround/connector
        n = max(1, min(2000, int(seg_m / step)))
        for i in range(n + 1):
            # skip the point shared with the previous same-pass segment; but after a
            # break (terr[-1] is NaN) keep i==0 — it starts the next pass.
            if i == 0 and dist and not math.isnan(terr[-1]):
                continue
            f = i / n
            x = a['x'] + (b['x'] - a['x']) * f
            y = a['y'] + (b['y'] - a['y']) * f
            dist.append(acc + seg_m * f)
            terr.append(dtm.elevation_at(x, y))
            flight.append(a['z'] + (b['z'] - a['z']) * f)
            pids.append(seg_pid)
        acc += seg_m

    # coalesce consecutive same-pass samples into (start, end, pass_id) spans.
    spans = []
    for dd, pid in zip(dist, pids):
        if pid is None:
            continue
        if spans and spans[-1][2] == pid:
            spans[-1] = (spans[-1][0], dd, pid)
        else:
            spans.append((dd, dd, pid))
    return dist, terr, flight, spans


class ProfilePanel(QWidget):
    """Full-width strip: terrain silhouette, flight line, target-AGL line, and any
    below-ground clearance. Call update_profile() when the route changes. Clicking a
    pass emits passClicked(pass_id) so the map can highlight it."""

    passClicked = Signal(object)          # pass_id of the clicked pass

    def __init__(self, parent=None):
        super().__init__(parent)
        self._spans = []                  # [(start_dist, end_dist, pass_id)]
        self._selected_pid = None
        self._sel_artist = None           # axvspan shading the selected pass
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(2)
        self.lbl = QLabel('Elevation profile — compute a route to populate.')
        self.lbl.setWordWrap(True)
        self.lbl.setStyleSheet('color:#9aa0a6;')
        lay.addWidget(self.lbl)

        self.fig = Figure(figsize=(8, 2.4))
        self.fig.patch.set_facecolor('#232629')
        self.fig.subplots_adjust(left=0.06, right=0.995, top=0.97, bottom=0.22)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.mpl_connect('button_press_event', self._on_click)
        lay.addWidget(self.canvas)
        self.ax = self.fig.add_subplot(111)
        self._style_axes()
        self.canvas.draw()

    def _on_click(self, event):
        """Map a click's x (distance) to a pass and announce it."""
        if event.inaxes is not self.ax or event.xdata is None:
            return
        x = event.xdata
        for s, e, pid in self._spans:
            if s <= x <= e:
                # clicking the already-selected pass toggles the highlight off
                self._selected_pid = None if pid == self._selected_pid else pid
                self._draw_selection()
                self.passClicked.emit(self._selected_pid)
                return

    def _draw_selection(self):
        if self._sel_artist is not None:
            try:
                self._sel_artist.remove()
            except (ValueError, AttributeError):
                pass
            self._sel_artist = None
        for s, e, pid in self._spans:
            if pid == self._selected_pid:
                self._sel_artist = self.ax.axvspan(s, e, color='#ffd400', alpha=0.18,
                                                   linewidth=0, zorder=0)
                break
        self.canvas.draw_idle()

    def _style_axes(self):
        ax = self.ax
        ax.set_facecolor('#1b1d21')
        ax.tick_params(colors='#9aa0a6', labelsize=8)
        for s in ax.spines.values():
            s.set_color('#383c42')
        ax.grid(True, color='#2b2f33', lw=0.6)
        ax.set_xlabel('Distance along route (m)', color='#c9ccd1', fontsize=8)
        ax.set_ylabel('Elevation (m)', color='#c9ccd1', fontsize=8)

    def clear(self):
        self.ax.clear()
        self._style_axes()
        self.lbl.setText('Elevation profile — compute a route to populate.')
        self.canvas.draw_idle()

    def update_profile(self, dist, terr, flight, agl=None, tol=50.0, spans=None):
        # tol defaults to ±50 so the 50–150 m AGL band is drawn as a reference
        # corridor even though the route itself isn't constrained to it.
        self.ax.clear()                 # drops the old selection artist too
        self._style_axes()
        self._spans = spans or []
        self._selected_pid = None
        self._sel_artist = None
        if not dist:
            self.lbl.setText('Elevation profile — no route.')
            self.canvas.draw_idle()
            return

        d = np.asarray(dist, float)
        t = np.asarray(terr, float)
        fl = np.asarray(flight, float)
        clear = fl - t                              # NaN where terrain has no data
        valid = ~np.isnan(clear)
        under = valid & (clear < 0)

        base = float(np.nanmin(t)) if np.isfinite(t).any() else 0.0
        ax = self.ax
        ax.fill_between(d, base, t, where=np.isfinite(t), color='#6f5a3d',
                        alpha=0.85, linewidth=0, zorder=1)
        ax.plot(d, t, color='#c8a97a', lw=1.2, label='Terrain', zorder=2)
        if agl and tol:
            # the allowed corridor: AGL band terrain+[agl-tol, agl+tol]
            ax.fill_between(d, t + (agl - tol), t + (agl + tol), where=np.isfinite(t),
                            color='#3fb0ff', alpha=0.12, linewidth=0, zorder=2,
                            label=f'AGL band ({agl - tol:.0f}–{agl + tol:.0f} m)')
        if agl:
            ax.plot(d, t + agl, color='#6b7280', lw=0.9, ls='--',
                    label=f'Target ({agl:.0f} m AGL)', zorder=3)
        ax.plot(d, fl, color='#3fb0ff', lw=1.8, label='Flight line', zorder=4)
        ax.fill_between(d, fl, t, where=under, color='#e5484d', alpha=0.7,
                        linewidth=0, zorder=5, label='Below ground')

        leg = ax.legend(loc='upper right', fontsize=7, framealpha=0.85, ncol=4)
        leg.get_frame().set_facecolor('#232629')
        for txt in leg.get_texts():
            txt.set_color('#c9ccd1')

        min_clear = float(np.nanmin(clear)) if valid.any() else float('nan')
        self.lbl.setText(self._headline(min_clear, under, valid))
        self.canvas.draw_idle()

    def _headline(self, min_clear, under, valid):
        if math.isnan(min_clear):
            return 'Elevation profile — no terrain data under the route.'
        if under.any():
            frac = 100.0 * float(np.count_nonzero(under)) / max(int(valid.sum()), 1)
            return (f'⚠ Minimum clearance {min_clear:.0f} m — flight line goes below '
                    f'ground on {frac:.0f}% of the route (red).')
        return f'Minimum clearance {min_clear:.0f} m above terrain along the route.'
