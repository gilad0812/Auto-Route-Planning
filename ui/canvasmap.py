"""Native Qt map canvas (offline) — replaces the Leaflet/QWebEngine map.

The DTM is rendered as a shaded-relief image in a QGraphicsView; the scene
coordinate system IS the DTM's pixel grid, so screen ↔ lon/lat is just the
raster transform. Pan (drag), zoom (wheel), and draw the AOI polygon by
clicking. The route and under-density cells are drawn as overlays. No internet,
no web engine — everything renders from the local DTM, so it works air-gapped.
"""
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors as mcolors

from PySide6.QtCore import Qt, Signal, QPointF, QRectF
from PySide6.QtGui import (
    QImage, QPixmap, QPainter, QPen, QColor, QBrush, QPolygonF,
)
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGraphicsView, QGraphicsScene,
    QGraphicsPixmapItem, QGraphicsEllipseItem, QGraphicsPathItem,
    QGraphicsPolygonItem, QToolButton, QLabel, QGraphicsItemGroup,
)
from PySide6.QtGui import QPainterPath

_LAT_M = 111139.0


# ----------------------------------------------------------------- imagery
def _shaded_relief(arr, nodata):
    a = np.asarray(arr, dtype=float)
    if nodata is not None:
        a = np.where(a == nodata, np.nan, a)
    finite = np.isfinite(a)
    fill = np.nanmin(a[finite]) if finite.any() else 0.0
    af = np.where(finite, a, fill)
    zmin, zmax = float(af.min()), float(af.max())
    norm = (af - zmin) / max(zmax - zmin, 1e-9)
    rgb = plt.get_cmap('gist_earth')(norm)[..., :3]
    dy, dx = np.gradient(af)
    slope = np.pi / 2 - np.arctan(np.hypot(dx, dy))
    aspect = np.arctan2(dy, -dx)
    az, alt = np.radians(315), np.radians(45)
    hs = (np.sin(alt) * np.sin(slope)
          + np.cos(alt) * np.cos(slope) * np.cos(az - aspect))
    hs = np.clip(hs, 0, 1)[..., None]
    out = np.clip(rgb * (0.45 + 0.55 * hs), 0, 1)
    return np.ascontiguousarray((out * 255).astype(np.uint8))


def _chm_rgba(arr, nodata):
    a = np.asarray(arr, dtype=float)
    if nodata is not None:
        a = np.where(a == nodata, np.nan, a)
    veg = np.isfinite(a) & (a > 0)
    rgba = np.zeros((*a.shape, 4), dtype=np.uint8)
    rgba[veg] = (60, 160, 60, 120)
    return np.ascontiguousarray(rgba)


class _View(QGraphicsView):
    """QGraphicsView with wheel-zoom and click-to-draw, delegating to the owner."""
    def __init__(self, scene, owner):
        super().__init__(scene)
        self._owner = owner
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setMouseTracking(True)

    def wheelEvent(self, e):
        factor = 1.25 if e.angleDelta().y() > 0 else 0.8
        self.scale(factor, factor)

    def mousePressEvent(self, e):
        if self._owner.drawing and e.button() == Qt.LeftButton:
            self._owner.add_vertex(self.mapToScene(e.position().toPoint()))
            e.accept(); return
        super().mousePressEvent(e)

    def mouseDoubleClickEvent(self, e):
        if self._owner.drawing:
            self._owner.finish_draw(); e.accept(); return
        super().mouseDoubleClickEvent(e)

    def mouseMoveEvent(self, e):
        self._owner.on_hover(self.mapToScene(e.position().toPoint()))
        super().mouseMoveEvent(e)


class CanvasMap(QWidget):
    polygonDrawn = Signal(object)        # emits a GeoJSON Polygon geometry dict

    def __init__(self, parent=None):
        super().__init__(parent)
        self.dtm = None
        self.chm = None
        self._inv = None                 # world -> pixel
        self.drawing = False
        self._verts = []                 # scene QPointF vertices in progress
        self._draw_items = []
        self._aoi_item = None
        self._route_group = None
        self._density_item = None
        self._helios_item = None
        self._chm_item = None

        v = QVBoxLayout(self); v.setContentsMargins(0, 0, 0, 0); v.setSpacing(0)
        barw = QWidget(); barw.setObjectName('mapbar')
        barw.setStyleSheet('#mapbar { background:#232629; '
                           'border-bottom:1px solid #383c42; }')
        bar = QHBoxLayout(barw); bar.setContentsMargins(8, 6, 8, 6); bar.setSpacing(6)
        self.btn_draw = QToolButton(); self.btn_draw.setText('✎ Draw AOI')
        self.btn_draw.setCheckable(True); self.btn_draw.clicked.connect(self._toggle_draw)
        self.btn_finish = QToolButton(); self.btn_finish.setText('Finish')
        self.btn_finish.clicked.connect(self.finish_draw)
        self.btn_fit = QToolButton(); self.btn_fit.setText('⤢ Fit')
        self.btn_fit.clicked.connect(self._fit)
        self.btn_chm = QToolButton(); self.btn_chm.setText('CHM')
        self.btn_chm.setCheckable(True); self.btn_chm.setEnabled(False)
        self.btn_chm.clicked.connect(self._toggle_chm)
        for b in (self.btn_draw, self.btn_finish, self.btn_fit, self.btn_chm):
            bar.addWidget(b)
        bar.addStretch(1)
        self.lbl_coord = QLabel(''); self.lbl_coord.setStyleSheet('color:#9aa0a6;')
        bar.addWidget(self.lbl_coord)
        v.addWidget(barw)

        self.scene = QGraphicsScene(self)
        self.view = _View(self.scene, self)
        self.view.setBackgroundBrush(QColor('#16181b'))
        self.view.setFrameShape(self.view.Shape.NoFrame)
        v.addWidget(self.view, 1)

    # ----------------------------------------------------------- data
    def set_dtm(self, dtm, dtm_path=None, chm=None, chm_path=None):
        self.dtm = dtm; self.chm = chm
        self._inv = ~dtm.transform
        self.scene.clear()
        self._aoi_item = self._route_group = self._density_item = None
        self._helios_item = self._chm_item = None
        self._verts = []; self._draw_items = []

        self._relief = _shaded_relief(dtm.array, dtm.nodata)
        h, w, _ = self._relief.shape
        img = QImage(self._relief.data, w, h, 3 * w, QImage.Format_RGB888)
        self.scene.addItem(QGraphicsPixmapItem(QPixmap.fromImage(img)))
        self.scene.setSceneRect(QRectF(0, 0, w, h))

        if chm is not None:
            self._chm_rgba = _chm_rgba(chm.array, chm.nodata)
            ch, cw, _ = self._chm_rgba.shape
            cimg = QImage(self._chm_rgba.data, cw, ch, 4 * cw, QImage.Format_RGBA8888)
            self._chm_item = QGraphicsPixmapItem(QPixmap.fromImage(cimg))
            self._chm_item.setVisible(self.btn_chm.isChecked())
            self.scene.addItem(self._chm_item)
            self.btn_chm.setEnabled(True)
        else:
            self.btn_chm.setEnabled(False)
        self._fit()

    def _fit(self):
        if self.dtm is not None:
            self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    # ----------------------------------------------------------- coords
    def _world(self, sp):
        x, y = self.dtm.transform * (sp.x(), sp.y())
        return x, y

    def _scene(self, lon, lat):
        c, r = self._inv * (lon, lat)
        return QPointF(c, r)

    def on_hover(self, sp):
        if self.dtm is None:
            return
        lon, lat = self._world(sp)
        z = self.dtm.elevation_at(lon, lat)
        ztxt = f'{z:.0f} m' if z == z else '—'      # NaN check
        self.lbl_coord.setText(f'{lat:.5f}, {lon:.5f}   ·   {ztxt}')

    # ----------------------------------------------------------- drawing
    def _toggle_draw(self, on):
        self.drawing = on
        self.view.setDragMode(QGraphicsView.NoDrag if on else QGraphicsView.ScrollHandDrag)
        self.view.setCursor(Qt.CrossCursor if on else Qt.ArrowCursor)
        if on:
            self.clear_aoi()

    def add_vertex(self, sp):
        self._verts.append(sp)
        dot = QGraphicsEllipseItem(-3, -3, 6, 6)
        dot.setPos(sp); dot.setBrush(QBrush(QColor('#ff3333')))
        dot.setPen(QPen(Qt.NoPen))
        dot.setFlag(QGraphicsEllipseItem.ItemIgnoresTransformations)
        self.scene.addItem(dot); self._draw_items.append(dot)
        if len(self._verts) >= 2:
            path = QPainterPath(self._verts[0])
            for p in self._verts[1:]:
                path.lineTo(p)
            if self._aoi_item is None:
                self._aoi_item = QGraphicsPathItem()
                pen = QPen(QColor('#ff3333'), 2); pen.setCosmetic(True)
                self._aoi_item.setPen(pen)
                self.scene.addItem(self._aoi_item)
            self._aoi_item.setPath(path)

    def finish_draw(self):
        if len(self._verts) < 3:
            return
        coords = []
        for p in self._verts:
            lon, lat = self._world(p)
            coords.append([lon, lat])
        coords.append(coords[0])
        # draw the closed polygon
        for it in self._draw_items:
            self.scene.removeItem(it)
        self._draw_items = []
        if self._aoi_item is not None:
            self.scene.removeItem(self._aoi_item); self._aoi_item = None
        poly = QPolygonF([self._verts[i] for i in range(len(self._verts))])
        self._aoi_item = QGraphicsPolygonItem(poly)
        pen = QPen(QColor('#ff3333'), 2); pen.setCosmetic(True)
        self._aoi_item.setPen(pen)
        self._aoi_item.setBrush(QBrush(QColor(255, 51, 51, 30)))
        self.scene.addItem(self._aoi_item)
        self._verts = []
        self.btn_draw.setChecked(False); self._toggle_draw(False)
        self.polygonDrawn.emit({'type': 'Polygon', 'coordinates': [coords]})

    def clear_aoi(self):
        for it in self._draw_items:
            self.scene.removeItem(it)
        self._draw_items = []
        if self._aoi_item is not None:
            self.scene.removeItem(self._aoi_item); self._aoi_item = None
        self._verts = []

    def set_aoi_polygon(self, coords):
        """Draw an AOI from externally-supplied [lon, lat] vertices (manual entry),
        replacing any current AOI. Requires a loaded DTM for the coordinate frame."""
        if self.dtm is None or not coords:
            return
        self.clear_aoi()
        pts = [self._scene(lon, lat) for lon, lat in coords]
        self._aoi_item = QGraphicsPolygonItem(QPolygonF(pts))
        pen = QPen(QColor('#ff3333'), 2); pen.setCosmetic(True)
        self._aoi_item.setPen(pen)
        self._aoi_item.setBrush(QBrush(QColor(255, 51, 51, 30)))
        self.scene.addItem(self._aoi_item)

    def clear(self):
        """Full reset to the empty state (used when the DTM is cleared)."""
        self.scene.clear()
        self.dtm = None; self.chm = None; self._inv = None
        self._aoi_item = self._route_group = self._density_item = None
        self._helios_item = self._chm_item = None
        self._verts = []; self._draw_items = []
        self.drawing = False
        self.btn_draw.setChecked(False)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.view.setCursor(Qt.ArrowCursor)
        self.btn_chm.setChecked(False); self.btn_chm.setEnabled(False)
        self.lbl_coord.setText('')

    def _toggle_chm(self, on):
        if self._chm_item is not None:
            self._chm_item.setVisible(on)

    # ----------------------------------------------------------- overlays
    def show_plan(self, route_wps, density_cells, density_color='#ff9900',
                  density_radius_m=3.0, max_density_pts=20000):
        if self.dtm is None:
            return
        # clear previous overlays
        if self._route_group is not None:
            self.scene.removeItem(self._route_group); self._route_group = None
        if self._density_item is not None:
            self.scene.removeItem(self._density_item); self._density_item = None

        # ── under-density: overlay image of ground-sized dots (scales w/ zoom) ──
        if density_cells:
            self._density_item = self._paint_cells(
                density_cells, density_color, 90, density_radius_m, max_density_pts)

        # ── route: altitude-coloured cosmetic polylines + start/end markers ──
        wps = [w for w in (route_wps or [])
               if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        if len(wps) >= 2:
            grp = QGraphicsItemGroup(); self.scene.addItem(grp)
            zs = [w['z'] for w in wps]; zmin, zmax = min(zs), max(zs)
            cmap = plt.get_cmap('cool')
            for a, b in zip(wps, wps[1:]):
                t = (a['z'] - zmin) / max(zmax - zmin, 1e-9)
                pa = self._scene(a['x'], a['y']); pb = self._scene(b['x'], b['y'])
                seg = QPainterPath(pa); seg.lineTo(pb)
                item = QGraphicsPathItem(seg)
                pen = QPen(QColor(mcolors.to_hex(cmap(t))), 2); pen.setCosmetic(True)
                item.setPen(pen); grp.addToGroup(item)
            self._marker(grp, self._scene(wps[0]['x'], wps[0]['y']), '#1a7f37')
            self._marker(grp, self._scene(wps[-1]['x'], wps[-1]['y']), '#cf222e')
            self._route_group = grp

    def _paint_cells(self, cells, color_hex, alpha, radius_m, max_pts=20000):
        """Paint cells (lon,lat) as ground-sized dots onto a transparent overlay
        image at DTM resolution, returned as a scene item (scales with zoom)."""
        if len(cells) > max_pts:
            step = len(cells) / max_pts
            cells = [cells[int(i * step)] for i in range(max_pts)]
        h, w, _ = self._relief.shape
        ov = QImage(w, h, QImage.Format_RGBA8888); ov.fill(0)
        p = QPainter(ov)
        col = QColor(color_hex); col.setAlpha(alpha)
        p.setBrush(QBrush(col)); p.setPen(QPen(Qt.NoPen))
        rad_px = max(1.0, radius_m / self._pixel_m())
        for lon, lat in cells:
            c, r = self._inv * (lon, lat)
            p.drawEllipse(QPointF(c, r), rad_px, rad_px)
        p.end()
        item = QGraphicsPixmapItem(QPixmap.fromImage(ov))
        self.scene.addItem(item)
        return item

    def show_helios(self, cells, radius_m=3.0):
        """Paint HELIOS++ under-density cells (red), separate from the estimate."""
        if self._helios_item is not None:
            self.scene.removeItem(self._helios_item); self._helios_item = None
        if cells:
            self._helios_item = self._paint_cells(cells, '#e5484d', 130, radius_m)

    def clear_overlays(self):
        """Remove route + density + HELIOS overlays (keeps the DTM and drawn AOI)."""
        for attr in ('_route_group', '_density_item', '_helios_item'):
            it = getattr(self, attr, None)
            if it is not None:
                self.scene.removeItem(it)
                setattr(self, attr, None)

    def _marker(self, grp, sp, hexcolor):
        m = QGraphicsEllipseItem(-5, -5, 10, 10)
        m.setPos(sp); m.setBrush(QBrush(QColor(hexcolor)))
        m.setPen(QPen(Qt.white, 1))
        m.setFlag(QGraphicsEllipseItem.ItemIgnoresTransformations)
        grp.addToGroup(m)

    def _pixel_m(self):
        rx, ry = abs(self.dtm.src.res[0]), abs(self.dtm.src.res[1])
        crs = self.dtm.src.crs
        if crs is not None and crs.is_geographic:
            lat0 = (self.dtm.src.bounds.bottom + self.dtm.src.bounds.top) / 2
            return (rx * _LAT_M * math.cos(math.radians(lat0)) + ry * _LAT_M) / 2
        return (rx + ry) / 2
