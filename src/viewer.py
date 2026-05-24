import sys
import os
import csv
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.widgets import TextBox, Button
from shapely.geometry import Polygon as ShapelyPolygon

sys.path.insert(0, os.path.dirname(__file__))
from dtm import DTM
from route_planner import plan_route


class InteractiveDTMViewer:
    """Top-down DTM viewer. Click to draw a polygon, then auto-generates a drone route."""

    def __init__(self, dtm_path, distance_above=30, spacing=20, step=10, error_tol=2):
        self.dtm = DTM(dtm_path)
        self.dtm_path = dtm_path

        # defaults (overridable via UI widgets)
        self._default_altitude = distance_above
        self._default_spacing  = spacing
        self._default_step     = step
        self._default_tol      = error_tol

        self._pts = []
        self._closed = False
        self._route = None
        self._route_artists = []

        self.fig = plt.figure(figsize=(14, 10))
        # Leave bottom 11 % for the controls strip
        self.fig.subplots_adjust(left=0.07, right=0.93, top=0.95, bottom=0.13)
        self.ax = self.fig.add_subplot(111)

        self._draw_dtm()
        self._init_artists()
        self._build_controls()
        self._connect()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _draw_dtm(self):
        src = self.dtm.src
        arr = self.dtm.array.astype(float)
        if self.dtm.nodata is not None:
            arr[arr == self.dtm.nodata] = np.nan

        bounds = src.bounds
        self._extent = [bounds.left, bounds.right, bounds.bottom, bounds.top]

        im = self.ax.imshow(
            arr,
            extent=self._extent,
            origin='upper',
            cmap='terrain',
            aspect='equal',
            interpolation='bilinear',
        )
        cbar = self.fig.colorbar(im, ax=self.ax, shrink=0.8, pad=0.02)
        cbar.set_label('Elevation (m)')

        crs_str = str(src.crs) if src.crs else 'unknown'
        self.ax.set_xlabel(f'X  [{crs_str}]')
        self.ax.set_ylabel(f'Y  [{crs_str}]')

    def _init_artists(self):
        self._poly_line, = self.ax.plot([], [], 'r-o', lw=2, ms=6, zorder=5, label='Polygon')
        self._rubber,    = self.ax.plot([], [], 'r--', lw=1, alpha=0.6, zorder=4)
        self._status = self.ax.text(
            0.01, 0.99,
            'Left-click to add vertices  |  Double-click or Enter to close  |  R to reset  |  S to save',
            transform=self.ax.transAxes,
            va='top', ha='left', fontsize=8.5, color='white',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#111', alpha=0.72),
            zorder=20,
        )

    def _build_controls(self):
        """Add parameter text boxes and action buttons along the bottom strip."""
        # Row 1 of controls: y=0.055, height=0.045 (figure coords)
        BOX_H = 0.045
        BOX_Y = 0.045

        # --- text boxes ---
        # (left, bottom, width, height) in figure fraction
        specs = [
            ('Altitude (m)',  str(self._default_altitude), 0.05,  0.13),
            ('Spacing',       str(self._default_spacing),  0.235, 0.13),
            ('Step',          str(self._default_step),     0.415, 0.10),
            ('Error tol (m)', str(self._default_tol),      0.565, 0.13),
        ]

        self._textboxes = {}
        for label, initial, left, width in specs:
            ax_tb = self.fig.add_axes([left, BOX_Y, width, BOX_H])
            tb = TextBox(ax_tb, label + '  ', initial=initial,
                         color='0.18', hovercolor='0.25',
                         label_pad=0.04)
            tb.label.set_color('white')
            tb.label.set_fontsize(8.5)
            tb.text_disp.set_color('white')
            tb.text_disp.set_fontsize(9)
            self._textboxes[label] = tb

        # --- buttons ---
        ax_recompute = self.fig.add_axes([0.745, BOX_Y, 0.09, BOX_H])
        ax_reset     = self.fig.add_axes([0.845, BOX_Y, 0.06, BOX_H])
        ax_save      = self.fig.add_axes([0.915, BOX_Y, 0.06, BOX_H])

        self._btn_recompute = Button(ax_recompute, 'Recompute', color='0.22', hovercolor='0.35')
        self._btn_reset     = Button(ax_reset,     'Reset',     color='0.22', hovercolor='0.35')
        self._btn_save      = Button(ax_save,      'Save',      color='0.22', hovercolor='0.35')

        for btn in (self._btn_recompute, self._btn_reset, self._btn_save):
            btn.label.set_color('white')
            btn.label.set_fontsize(8.5)

        self._btn_recompute.on_clicked(self._on_recompute_clicked)
        self._btn_reset    .on_clicked(lambda _: self._reset())
        self._btn_save     .on_clicked(lambda _: self._save_route())

        # Separator line above controls
        self.fig.add_artist(
            plt.Line2D([0.04, 0.98], [0.108, 0.108],
                       transform=self.fig.transFigure,
                       color='0.35', linewidth=0.8)
        )

    def _connect(self):
        c = self.fig.canvas
        c.mpl_connect('button_press_event', self._on_click)
        c.mpl_connect('motion_notify_event', self._on_motion)
        c.mpl_connect('key_press_event', self._on_key)

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _read_params(self):
        """Parse text boxes. Returns (altitude, spacing, step, tol) or raises ValueError."""
        vals = {}
        for label, tb in self._textboxes.items():
            try:
                vals[label] = float(tb.text.strip())
            except ValueError:
                raise ValueError(f'Invalid value for "{label}": {tb.text!r}')
        return (
            vals['Altitude (m)'],
            vals['Spacing'],
            vals['Step'],
            vals['Error tol (m)'],
        )

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_click(self, ev):
        if ev.inaxes is not self.ax or self._closed:
            return
        if ev.xdata is None:
            return

        x, y = ev.xdata, ev.ydata

        if ev.dblclick:
            if self._pts:
                self._pts.pop()
            if len(self._pts) >= 3:
                self._close_polygon()
            return

        if ev.button == 1:
            self._pts.append((x, y))
            self._redraw_polygon()
            n = len(self._pts)
            self._set_status(
                f'{n} vert{"ex" if n == 1 else "ices"} '
                f'— double-click or Enter to close  |  R to reset'
            )

    def _on_motion(self, ev):
        if ev.inaxes is not self.ax or self._closed or not self._pts:
            return
        if ev.xdata is None:
            return
        lx, ly = self._pts[-1]
        fx, fy = self._pts[0]
        cx, cy = ev.xdata, ev.ydata
        if len(self._pts) >= 3:
            self._rubber.set_data([lx, cx, fx], [ly, cy, fy])
        else:
            self._rubber.set_data([lx, cx], [ly, cy])
        self.fig.canvas.draw_idle()

    def _on_key(self, ev):
        if ev.key == 'enter' and not self._closed and len(self._pts) >= 3:
            self._close_polygon()
        elif ev.key in ('r', 'escape'):
            self._reset()
        elif ev.key == 's':
            self._save_route()

    def _on_recompute_clicked(self, _):
        if not self._closed or len(self._pts) < 3:
            self._set_status('Draw and close a polygon first, then click Recompute')
            return
        # Clear old route
        for a in self._route_artists:
            a.remove()
        self._route_artists.clear()
        if self.ax.get_legend():
            self.ax.get_legend().remove()
        self._route = None
        self._set_status('Recomputing route with new parameters…')
        plt.pause(0.05)
        self._compute_route()

    # ------------------------------------------------------------------
    # Polygon / route logic
    # ------------------------------------------------------------------

    def _close_polygon(self):
        self._closed = True
        self._rubber.set_data([], [])
        closed_pts = self._pts + [self._pts[0]]
        self._poly_line.set_data(
            [p[0] for p in closed_pts],
            [p[1] for p in closed_pts],
        )
        self.fig.canvas.draw_idle()
        self._set_status('Polygon closed — computing route…')
        plt.pause(0.05)
        self._compute_route()

    def _compute_route(self):
        try:
            altitude, spacing, step, tol = self._read_params()
        except ValueError as e:
            self._set_status(str(e))
            return

        poly = ShapelyPolygon(self._pts)
        if not poly.is_valid:
            poly = poly.buffer(0)

        self._route = plan_route(self.dtm, poly, altitude, tol, spacing, step)

        valid = [
            (wp['x'], wp['y'], wp['z']) for wp in self._route
            if not (isinstance(wp['z'], float) and np.isnan(wp['z']))
        ]

        if len(valid) < 2:
            self._set_status('No valid waypoints inside DTM coverage — adjust params or redraw')
            return

        xs = np.array([p[0] for p in valid])
        ys = np.array([p[1] for p in valid])
        zs = np.array([p[2] for p in valid])

        pts_arr = np.c_[xs, ys].reshape(-1, 1, 2)
        segs = np.concatenate([pts_arr[:-1], pts_arr[1:]], axis=1)
        lc = LineCollection(segs, cmap='cool', lw=2, zorder=7, alpha=0.9)
        lc.set_array(zs[:-1])
        self.ax.add_collection(lc)

        sc       = self.ax.scatter(xs, ys, c=zs, cmap='cool', s=12, zorder=8, alpha=0.85)
        start_m, = self.ax.plot(xs[0],  ys[0],  'g^', ms=14, zorder=9, label='Start')
        end_m,   = self.ax.plot(xs[-1], ys[-1], 'rs', ms=12, zorder=9, label='End')
        self.ax.legend(loc='lower right', fontsize=8)

        self._route_artists = [lc, sc, start_m, end_m]

        total = float(np.sum(np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)))
        self._set_status(
            f'Route ready: {len(valid)} waypoints  |  {total:.4f} map-units total  |  '
            f'Alt range: {zs.min():.1f}–{zs.max():.1f} m  |  S to save'
        )
        self.fig.canvas.draw_idle()

    def _reset(self):
        self._pts.clear()
        self._closed = False
        self._route = None
        self._poly_line.set_data([], [])
        self._rubber.set_data([], [])
        for a in self._route_artists:
            a.remove()
        self._route_artists.clear()
        if self.ax.get_legend():
            self.ax.get_legend().remove()
        self._set_status(
            'Left-click to add vertices  |  Double-click or Enter to close  |  R to reset  |  S to save'
        )
        self.fig.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Save
    # ------------------------------------------------------------------

    def _save_route(self):
        if not self._route:
            self._set_status('No route to save yet — draw a polygon first')
            return

        base = os.path.splitext(os.path.basename(self.dtm_path))[0]
        out_dir = os.path.dirname(self.dtm_path)
        geojson_path = os.path.join(out_dir, f'{base}_route.geojson')
        csv_path     = os.path.join(out_dir, f'{base}_route.csv')

        features = []
        for wp in self._route:
            if isinstance(wp['z'], float) and np.isnan(wp['z']):
                continue
            features.append({
                'type': 'Feature',
                'geometry': {'type': 'Point', 'coordinates': [wp['x'], wp['y'], wp['z']]},
                'properties': {
                    'altitude_m':    wp['z'],
                    'target_agl_m':  wp['target_distance'],
                    'error_tol_m':   wp['error_tol'],
                },
            })

        with open(geojson_path, 'w') as f:
            json.dump({'type': 'FeatureCollection', 'features': features}, f, indent=2)

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['index', 'x', 'y', 'z', 'target_agl_m', 'error_tol_m'])
            writer.writeheader()
            for i, feat in enumerate(features):
                x, y, z = feat['geometry']['coordinates']
                writer.writerow({
                    'index':        i,
                    'x':            x,
                    'y':            y,
                    'z':            z,
                    'target_agl_m': feat['properties']['target_agl_m'],
                    'error_tol_m':  feat['properties']['error_tol_m'],
                })

        self._set_status(
            f'Saved {len(features)} waypoints → '
            f'{os.path.basename(geojson_path)}  +  {os.path.basename(csv_path)}'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _redraw_polygon(self):
        if self._pts:
            self._poly_line.set_data(
                [p[0] for p in self._pts],
                [p[1] for p in self._pts],
            )
        else:
            self._poly_line.set_data([], [])
        self.fig.canvas.draw_idle()

    def _set_status(self, msg):
        self._status.set_text(msg)
        self.fig.canvas.draw_idle()

    def show(self):
        plt.show()
