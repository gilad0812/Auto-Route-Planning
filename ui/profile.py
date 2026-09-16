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
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

_LAT_M = 111139.0


def route_profile(route, dtm, is_geo=True, sample_step_m=None, join_passes=True):
    """Sample terrain + flight altitude along the flown route (ordered waypoints
    with x, y, z, pass_id). Returns (dist_m, terrain_m, flight_m, spans, pids): the first
    three are per-sample lists in metres; `spans` = [(start_dist, end_dist, pass_id)] — the
    x-range each pass occupies, so a click on the profile maps back to a pass; `pids` is the
    per-sample pass_id (None for turnaround/connector samples), so a drag can move exactly
    one pass's samples. Turnaround/connector samples get pass_id None (not clickable).

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
        return [], [], [], [], []
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
    return dist, terr, flight, spans, pids


class ProfilePanel(QWidget):
    """Full-width strip: terrain silhouette, flight line, target-AGL line, and any
    below-ground clearance. Call update_profile() when the route changes. Clicking a
    pass emits passClicked(pass_id) so the map can highlight it, and reveals an inline
    altitude editor for that pass (passAltitudeChanged(pass_id, new_z) on commit)."""

    passClicked = Signal(object)          # pass_id of the clicked pass
    passAltitudeChanged = Signal(object, float)   # (pass_id, new absolute altitude m)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._spans = []                  # [(start_dist, end_dist, pass_id)]
        self._pass_alt = {}               # {pass_id: current flight altitude (m)}
        self._selected_pid = None
        self._sel_artist = None           # axvspan shading the selected pass
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(2)
        self.lbl = QLabel('Elevation profile — compute a route to populate.')
        self.lbl.setWordWrap(True)
        self.lbl.setStyleSheet('color:#9aa0a6;')
        lay.addWidget(self.lbl)

        # Live altitude readout for the selected / dragged pass (hidden until one is
        # picked). Drag the pass line in the plot to change its altitude — no manual input.
        self._editor = QWidget()
        er = QHBoxLayout(self._editor)
        er.setContentsMargins(0, 0, 0, 0); er.setSpacing(6)
        self.ed_lbl = QLabel('')
        self.ed_lbl.setStyleSheet('color:#c9ccd1;')
        er.addWidget(self.ed_lbl); er.addStretch(1)
        self._editor.setVisible(False)
        lay.addWidget(self._editor)

        self.fig = Figure(figsize=(8, 2.4))
        self.fig.patch.set_facecolor('#232629')
        self.fig.subplots_adjust(left=0.06, right=0.995, top=0.97, bottom=0.22)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.mpl_connect('button_press_event', self._on_press)
        self.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self.canvas.mpl_connect('button_release_event', self._on_release)
        self._drag = None                 # active drag: {pid, y0, z0, span, moved, z, …}
        self._d = None                    # sample distances (np) for live drag redraw
        self._fl_base = None              # flight altitudes (np) before the current drag
        self._sample_pids = None          # per-sample pass_id (object np) — exact drag mask
        self._flight_artist = None        # the flight-line Line2D, moved live while dragging
        lay.addWidget(self.canvas)
        self.ax = self.fig.add_subplot(111)
        self._style_axes()
        self.canvas.draw()

    _DRAG_MIN_PX = 3.0                     # move less than this = a click, not a drag

    def _span_at(self, x):
        """pass_id whose distance-span contains x, or None."""
        for s, e, pid in self._spans:
            if s <= x <= e:
                return pid
        return None

    def _span_of(self, pid):
        """(start, end) distance-span for pass `pid`, or None."""
        for s, e, p in self._spans:
            if p == pid:
                return (s, e)
        return None

    def _on_press(self, event):
        """Begin a potential drag of the pass under the cursor. A drag moves the pass's
        flight line up/down (its altitude); a press-with-no-move is treated as a click
        (toggles the selection), so both gestures still work on the same canvas."""
        if event.inaxes is not self.ax or event.xdata is None:
            return
        pid = self._span_at(event.xdata)
        if pid is None:
            return
        self._drag = {'pid': pid, 'y0': event.ydata, 'z0': self._pass_alt.get(pid),
                      'z': self._pass_alt.get(pid), 'span': self._span_of(pid),
                      'x_px': event.x, 'y_px': event.y, 'moved': False,
                      'was_sel': (pid == self._selected_pid)}

    def _on_motion(self, event):
        """While dragging, move the pass's flight line to follow the cursor and write the
        new altitude live (in the readout + spinbox). Nothing is committed until release."""
        d = self._drag
        if not d or event.inaxes is not self.ax:
            return
        if not d['moved']:
            if event.x is None or math.hypot(event.x - d['x_px'], event.y - d['y_px']) < self._DRAG_MIN_PX:
                return                      # still within the click threshold
            d['moved'] = True
            self._selected_pid = d['pid']   # dragging selects the pass (shows the readout)
            self._draw_selection()
            self._editor.setVisible(d['z0'] is not None)
        if d['z0'] is None or event.ydata is None:
            return
        new_z = min(max(d['z0'] + (event.ydata - d['y0']), 0.0), 10000.0)
        d['z'] = new_z
        if self._flight_artist is not None and self._fl_base is not None:
            # move EXACTLY this pass's samples (by pass_id) — never the neighbouring pass
            # or the break between them, so no connector line appears while dragging.
            if self._sample_pids is not None:
                m = self._sample_pids == d['pid']
            elif self._d is not None and d['span']:
                s, e = d['span']; m = (self._d >= s) & (self._d <= e)
            else:
                m = None
            if m is not None:
                work = self._fl_base.copy()
                work[m] = self._fl_base[m] + (new_z - d['z0'])
                self._flight_artist.set_ydata(work)
        self.ed_lbl.setText(f'Line {d["pid"]} altitude: {new_z:.0f} m   —   release to apply')
        self.canvas.draw_idle()

    def _on_release(self, event):
        """Commit a drag (emit the new altitude) or, if the pass wasn't dragged, treat it
        as a click that toggles the selection."""
        d = self._drag
        self._drag = None
        if not d:
            return
        if d['moved'] and d['z0'] is not None:
            self.passAltitudeChanged.emit(d['pid'], float(d['z']))
            return
        # a plain click: toggle the highlight for this pass
        self._selected_pid = None if d['was_sel'] else d['pid']
        self._draw_selection()
        self._sync_editor()
        self.passClicked.emit(self._selected_pid)

    def _sync_editor(self):
        """Show the altitude readout for the selected pass (its current altitude), or hide
        it when nothing is selected. The altitude is changed by dragging the pass line."""
        pid = self._selected_pid
        if pid is None or pid not in self._pass_alt:
            self._editor.setVisible(False)
            return
        self.ed_lbl.setText(f'Line {pid} altitude: {float(self._pass_alt[pid]):.0f} m'
                            '   —   drag the line to change')
        self._editor.setVisible(True)

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
        self._selected_pid = None
        self._sync_editor()
        self.lbl.setText('Elevation profile — compute a route to populate.')
        self.canvas.draw_idle()

    def update_profile(self, dist, terr, flight, agl=None, tol=50.0, spans=None,
                       pass_alt=None, pids=None):
        # tol defaults to ±50 so the 50–150 m AGL band is drawn as a reference
        # corridor even though the route itself isn't constrained to it.
        self.ax.clear()                 # drops the old selection artist too
        self._style_axes()
        self._spans = spans or []
        self._pass_alt = pass_alt or {}
        # Keep the current selection if that pass still exists, so an altitude edit
        # doesn't clear it out from under the operator; otherwise deselect.
        valid_pids = {pid for _, _, pid in self._spans}
        self._selected_pid = self._selected_pid if self._selected_pid in valid_pids else None
        self._sel_artist = None
        if not dist:
            self._sync_editor()
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
        self._flight_artist = ax.plot(d, fl, color='#3fb0ff', lw=1.8,
                                      label='Flight line', zorder=4)[0]
        self._d = d                      # kept so a drag can move a pass's samples live
        self._fl_base = fl.copy()
        self._sample_pids = np.array(pids, dtype=object) if pids else None
        ax.fill_between(d, fl, t, where=under, color='#e5484d', alpha=0.7,
                        linewidth=0, zorder=5, label='Below ground')

        leg = ax.legend(loc='upper right', fontsize=7, framealpha=0.85, ncol=4)
        leg.get_frame().set_facecolor('#232629')
        for txt in leg.get_texts():
            txt.set_color('#c9ccd1')

        min_clear = float(np.nanmin(clear)) if valid.any() else float('nan')
        self.lbl.setText(self._headline(min_clear, under, valid))
        self._draw_selection()          # re-shade a surviving selection after the redraw
        self._sync_editor()             # and refresh the altitude editor to match
        self.canvas.draw_idle()

    def _headline(self, min_clear, under, valid):
        if math.isnan(min_clear):
            return 'Elevation profile — no terrain data under the route.'
        if under.any():
            frac = 100.0 * float(np.count_nonzero(under)) / max(int(valid.sum()), 1)
            return (f'⚠ Minimum clearance {min_clear:.0f} m — flight line goes below '
                    f'ground on {frac:.0f}% of the route (red).')
        return f'Minimum clearance {min_clear:.0f} m above terrain along the route.'
