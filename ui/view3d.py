"""Interactive 3D terrain + route view embedded in Qt (PyVista / VTK).

The desktop counterpart of viz3d.route_3d_figure: a DTM surface clipped around
the route, with the planned path draped on top and coloured by terrain
clearance. Drag to orbit, scroll to zoom — a real VTK scene, pickable later for
draggable waypoints.
"""
import math
import numpy as np

from PySide6.QtWidgets import QWidget, QVBoxLayout
from pyvistaqt import QtInteractor
import pyvista as pv

_LAT_M = 111139.0


def _geometry(dtm, route, is_geo, max_surf=220, pad_frac=0.08):
    """Return (GXm, GYm, Z) terrain surface in metres + (rx, ry, rz) route, all
    sharing the same local metric origin so they overlay correctly."""
    xs = [w['x'] for w in route]
    ys = [w['y'] for w in route]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    px = (maxx - minx) * pad_frac or 1e-4
    py = (maxy - miny) * pad_frac or 1e-4
    minx, maxx, miny, maxy = minx - px, maxx + px, miny - py, maxy + py

    inv = ~dtm.transform
    c0, r0 = inv * (minx, maxy)
    c1, r1 = inv * (maxx, miny)
    arr = dtm.array
    H, W = arr.shape
    j0, j1 = sorted((int(max(0, min(W - 1, c0))), int(max(0, min(W - 1, c1)))))
    i0, i1 = sorted((int(max(0, min(H - 1, r0))), int(max(0, min(H - 1, r1)))))
    j1 = max(j1, j0 + 1); i1 = max(i1, i0 + 1)

    sub = arr[i0:i1 + 1, j0:j1 + 1].astype(float)
    if dtm.nodata is not None:
        sub = np.where(sub == dtm.nodata, np.nan, sub)
    sh, sw = sub.shape
    si = max(1, -(-sh // max_surf))
    sj = max(1, -(-sw // max_surf))
    sub = sub[::si, ::sj]

    rows = np.arange(i0, i1 + 1, si)[:sub.shape[0]]
    cols = np.arange(j0, j1 + 1, sj)[:sub.shape[1]]
    wx = np.array([(dtm.transform * (c + 0.5, 0))[0] for c in cols])
    wy = np.array([(dtm.transform * (0, r + 0.5))[1] for r in rows])
    GX, GY = np.meshgrid(wx, wy)

    lat0 = float(np.nanmean(GY))
    if is_geo:
        xfac = _LAT_M * math.cos(math.radians(lat0)); yfac = _LAT_M
    else:
        xfac = yfac = 1.0
    ox, oy = GX.min(), GY.min()
    GXm = (GX - ox) * xfac
    GYm = (GY - oy) * yfac
    Z = sub.copy()
    if np.isnan(Z).any():               # StructuredGrid can't hold NaN points
        Z = np.where(np.isnan(Z), np.nanmin(Z), Z)

    rx = (np.array(xs) - ox) * xfac
    ry = (np.array(ys) - oy) * yfac
    rz = np.array([w['z'] for w in route])
    return GXm, GYm, Z, rx, ry, rz


class View3D(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor)
        self.plotter.set_background('white')
        self.plotter.add_text('Compute a route to populate the 3D view.',
                              font_size=10, color='gray', name='hint')

    def update_scene(self, dtm, route, clearances, is_geo=True,
                     floor_m=30.0, ceiling_m=120.0, z_exag=1.0):
        self.plotter.clear()
        if len(route) < 2:
            return
        GXm, GYm, Z, rx, ry, rz = _geometry(dtm, route, is_geo)

        # Terrain surface, coloured by elevation.
        terrain = pv.StructuredGrid(GXm, GYm, Z * z_exag)
        terrain['Elevation (m)'] = terrain.points[:, 2] / z_exag
        self.plotter.add_mesh(terrain, scalars='Elevation (m)', cmap='gist_earth',
                              show_scalar_bar=False, smooth_shading=True)

        # Route line, coloured by clearance.
        pts = np.column_stack([rx, ry, rz * z_exag])
        line = pv.lines_from_points(pts)
        line['Clearance (m)'] = np.asarray(clearances, dtype=float)
        self.plotter.add_mesh(
            line, scalars='Clearance (m)', cmap='RdYlGn',
            clim=[floor_m, max(ceiling_m, floor_m + 1)], line_width=4,
            scalar_bar_args={'title': 'Clearance (m)'},
        )
        self.plotter.add_points(pts, color='black', point_size=4,
                                render_points_as_spheres=True)

        self.plotter.show_grid(color='gray')
        self.plotter.reset_camera()
        self.plotter.camera.elevation = -55       # look down at the terrain
