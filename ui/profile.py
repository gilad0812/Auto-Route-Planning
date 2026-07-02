"""Elevation / clearance profile along the flown route.

Samples the terrain under the route and plots it against the flight line so the
operator can see clearance per pass and spot passes that dip toward (or through)
the ground. Terrain-vs-flight-line is exactly the check that reveals a constant
per-pass altitude clipping a ridge. Lives as a full-width strip below the map and
redraws whenever the route changes.
"""
import math

import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

_LAT_M = 111139.0


def route_profile(route, dtm, is_geo=True, sample_step_m=None):
    """Sample terrain + flight altitude along the flown route (ordered waypoints
    with x, y, z). Returns (dist_m, terrain_m, flight_m) as lists in metres.

    NaN-z waypoints are dropped; flight altitude is linear between kept waypoints —
    flat within a pass (equal endpoint z) and a climb/descent across a turn. Terrain
    is NaN where the path leaves the DTM (e.g. a ferry outside the tile)."""
    wps = [w for w in route
           if not (isinstance(w['z'], float) and math.isnan(w['z']))]
    if len(wps) < 2:
        return [], [], []
    lat0 = sum(w['y'] for w in wps) / len(wps)
    lon_m = _LAT_M * math.cos(math.radians(lat0)) if is_geo else 1.0
    lat_m = _LAT_M if is_geo else 1.0
    res_m = min(abs(dtm.src.res[0]), abs(dtm.src.res[1])) * (lat_m if is_geo else 1.0)
    step = sample_step_m or max(res_m, 2.0)

    dist, terr, flight, acc = [], [], [], 0.0
    for a, b in zip(wps, wps[1:]):
        seg_m = math.hypot((b['x'] - a['x']) * lon_m, (b['y'] - a['y']) * lat_m)
        n = max(1, min(2000, int(seg_m / step)))
        for i in range(n + 1):
            if i == 0 and dist:            # skip the point shared with the last segment
                continue
            f = i / n
            x = a['x'] + (b['x'] - a['x']) * f
            y = a['y'] + (b['y'] - a['y']) * f
            dist.append(acc + seg_m * f)
            terr.append(dtm.elevation_at(x, y))
            flight.append(a['z'] + (b['z'] - a['z']) * f)
        acc += seg_m
    return dist, terr, flight


class ProfilePanel(QWidget):
    """Full-width strip: terrain silhouette, flight line, target-AGL line, and any
    below-ground clearance. Call update_profile() when the route changes."""

    def __init__(self, parent=None):
        super().__init__(parent)
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
        lay.addWidget(self.canvas)
        self.ax = self.fig.add_subplot(111)
        self._style_axes()
        self.canvas.draw()

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

    def update_profile(self, dist, terr, flight, agl=None, tol=50.0):
        # tol defaults to ±50 so the 50–150 m AGL band is drawn as a reference
        # corridor even though the route itself isn't constrained to it.
        self.ax.clear()
        self._style_axes()
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
