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
    QImage, QPixmap, QPainter, QPen, QColor, QBrush, QPolygonF, QCursor, QTransform,
)
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGraphicsView, QGraphicsScene,
    QGraphicsPixmapItem, QGraphicsEllipseItem, QGraphicsPathItem,
    QGraphicsPolygonItem, QToolButton, QLabel, QGraphicsItemGroup, QToolTip,
)
from PySide6.QtGui import QPainterPath

_LAT_M = 111139.0

# Display budget (cells) for a focused AOI window (Load AOI .wkt). Larger than the
# whole-extent budget: a focused region is bounded, so we can afford to show it at (near)
# native resolution. A 3 km² AOI at 0.5 m (~12M cells, +margin) lands under this, so it
# renders native. Beyond it the region is decimated to fit (still far sharper than the
# whole-extent view).
_FOCUS_DISP_CELLS = 16_000_000

# Under-density overlay palette, keyed by the estimator's failure CAUSE. Shared
# with the summary legend so map colours and text agree. (hex, alpha).
FAILURE_REASON_STYLE = {
    "range": ("#8c959f", 130),    # beyond scanner max range — lower AGL/PRR (grey)
    "shadow": ("#8250df", 120),   # occlusion shadow — cross-pass or accept (purple)
    "thin": ("#ff9900", 95),      # under target (incl. uncovered) — drawn as a gradient
}
# Human labels + the operator's lever, for the legend.
FAILURE_REASON_LABEL = {
    "range": ("Beyond scanner range", "lower AGL or PRR"),
    "shadow": ("Occlusion shadow", "needs a cross-pass, or accept"),
    "thin": ("Under target", "lower AGL / tighter spacing"),
}
# Under-target cells (thin + uncovered) are shaded by density/target: 0 → red, target → yellow.
_THIN_CMAP = plt.get_cmap('autumn')


def _point_seg_dist(px, py, ax, ay, bx, by):
    """Shortest distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    d2 = dx * dx + dy * dy
    if d2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / d2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


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
    # Hillshade = surface-normal · sun direction. gradient gives dy per row (south,
    # since row increases downward) and dx per col (east). North is up-screen = -row,
    # so the outward surface normal in (East, North, Up) is (-dx, +dy, 1). Lighting
    # it with the sun at azimuth 315° (NW), 45° elevation. Using the raw dot product
    # (not an aspect angle) avoids the sign/quadrant slips that invert the relief.
    dy, dx = np.gradient(af)
    az, alt = np.radians(315), np.radians(45)
    lx, ly, lz = np.cos(alt) * np.sin(az), np.cos(alt) * np.cos(az), np.sin(alt)
    hs = (-dx * lx + dy * ly + lz) / np.sqrt(dx * dx + dy * dy + 1.0)
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
        if self._owner.drawing_pass and e.button() == Qt.LeftButton:
            self._owner.add_pass_vertex(self.mapToScene(e.position().toPoint()))
            e.accept(); return
        if self._owner.selecting_passes and e.button() == Qt.LeftButton:
            self._owner.toggle_pass_at(self.mapToScene(e.position().toPoint()))
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
    passDrawn = Signal(object)           # emits (lon,lat) — the END of a new pass (start = route end)
    focusToggled = Signal(bool)          # Focus/Full toggle: True = focus on the AOI
    passSelectionChanged = Signal(int)   # # of route passes currently selected
    passesConfirmed = Signal()           # operator confirmed the selected passes

    def __init__(self, parent=None):
        super().__init__(parent)
        self.dtm = None
        self.chm = None
        self._inv = None                 # world -> pixel
        self._focus_polygon = None       # AOI the map is focused on, or None (full extent)
        self.selecting_passes = False     # route-pass selection mode (Load route .wkt)
        self._route_passes = {}          # pass_id -> {'items','segs','selected'}
        self._disp_transform = None      # scene(overview) pixel -> world; None until loaded
        self.drawing = False
        self.drawing_pass = False
        self._verts = []                 # scene QPointF vertices in progress
        self._draw_items = []
        self._pass_anchor = None         # scene point a drawn pass starts from (route end)
        self._pass_preview = None        # rubber-band line to the cursor
        self._aoi_item = None
        self._route_group = None
        self._density_item = None
        self._helios_item = None
        self._chm_item = None
        self._home_item = None           # takeoff/return marker + ferry legs
        self._pass_segs = []             # [(ax, ay, bx, by, z)] scene coords, for hover

        v = QVBoxLayout(self); v.setContentsMargins(0, 0, 0, 0); v.setSpacing(0)
        barw = QWidget(); barw.setObjectName('mapbar')
        barw.setStyleSheet('#mapbar { background:#232629; '
                           'border-bottom:1px solid #383c42; }')
        bar = QHBoxLayout(barw); bar.setContentsMargins(8, 6, 8, 6); bar.setSpacing(6)
        self.btn_draw = QToolButton(); self.btn_draw.setText('✎ Draw polygon')
        self.btn_draw.setCheckable(True); self.btn_draw.clicked.connect(self._toggle_draw)
        self.btn_draw.setToolTip('Click to add vertices; double-click to close the polygon.')
        self.btn_pass = QToolButton(); self.btn_pass.setText('✚ Add Pass')
        self.btn_pass.setCheckable(True); self.btn_pass.setEnabled(False)
        self.btn_pass.setToolTip('Click two points to add a flight pass; its height '
                                 'is set from the terrain. Stays on for more passes.')
        self.btn_pass.clicked.connect(self._toggle_pass)
        self.btn_fit = QToolButton(); self.btn_fit.setText('⤢ Fit')
        self.btn_fit.clicked.connect(self._fit)
        self.btn_focus = QToolButton(); self.btn_focus.setText('◎ polygon')
        self.btn_focus.setCheckable(True); self.btn_focus.setEnabled(False)
        self.btn_focus.setToolTip('Zoom the map to the polygon at native resolution; '
                                  'toggle off to show the full DTM.')
        self.btn_focus.toggled.connect(self.focusToggled)
        self.btn_chm = QToolButton(); self.btn_chm.setText('CHM')
        self.btn_chm.setCheckable(True); self.btn_chm.setEnabled(False)
        self.btn_chm.clicked.connect(self._toggle_chm)
        # Shown only while selecting passes from a loaded route (.wkt).
        self.btn_all = QToolButton(); self.btn_all.setText('Select all')
        self.btn_all.setEnabled(False); self.btn_all.setVisible(False)
        self.btn_all.setToolTip('Select every pass in the loaded route (toggles to Clear all).')
        self.btn_all.clicked.connect(self._toggle_select_all)
        self.btn_confirm = QToolButton(); self.btn_confirm.setText('✓ Confirm passes')
        self.btn_confirm.setEnabled(False); self.btn_confirm.setVisible(False)
        self.btn_confirm.setToolTip('Run the density estimate on the selected passes.')
        self.btn_confirm.clicked.connect(self.passesConfirmed)
        for b in (self.btn_draw, self.btn_pass,
                  self.btn_fit, self.btn_focus, self.btn_chm,
                  self.btn_all, self.btn_confirm):
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
    def _reset_scene(self):
        self.scene.clear()                           # deletes all items, incl. route passes
        self._aoi_item = self._route_group = self._density_item = None
        self._helios_item = self._chm_item = None; self._home_item = None
        self._verts = []; self._draw_items = []; self._pass_segs = []
        self._pass_anchor = None; self._pass_preview = None
        self._route_passes = {}; self.selecting_passes = False
        self.btn_confirm.setVisible(False); self.btn_confirm.setEnabled(False)
        self.btn_all.setVisible(False); self.btn_all.setEnabled(False)
        self._disp_transform = None; self._inv = None
        self.drawing_pass = False; self.btn_pass.setChecked(False)
        self.btn_pass.setEnabled(False)

    # ------------------------------------------- loaded route: pass selection
    def show_route_passes(self, passes):
        """Draw a loaded route as individually SELECTABLE passes and enter selection
        mode. `passes` = list of (pass_id, [(lon, lat), ...]). Passes start unselected
        (dim); clicking one on the map toggles it. Requires a rendered map (focus first)."""
        self.clear_route_passes()
        if self._inv is None:
            return
        for pid, pts in passes:
            sp = [self._scene(lon, lat) for lon, lat in pts]
            if len(sp) < 2:
                continue
            path = QPainterPath(sp[0])
            for p in sp[1:]:
                path.lineTo(p)
            item = QGraphicsPathItem(path); item.setZValue(10)
            self.scene.addItem(item)
            segs = [(sp[i].x(), sp[i].y(), sp[i + 1].x(), sp[i + 1].y())
                    for i in range(len(sp) - 1)]
            self._route_passes[pid] = {'item': item, 'segs': segs, 'selected': False}
            self._style_pass(pid)
        self.selecting_passes = True
        self.btn_confirm.setVisible(True)
        self.btn_confirm.setEnabled(False)
        self.btn_all.setVisible(True)
        self.btn_all.setEnabled(bool(self._route_passes))
        self._sync_all_button()
        self.passSelectionChanged.emit(0)

    def _style_pass(self, pid):
        d = self._route_passes[pid]
        pen = QPen(QColor('#3fb950' if d['selected'] else '#8c959f'),
                   3 if d['selected'] else 2)
        pen.setCosmetic(True)
        d['item'].setPen(pen)

    def toggle_pass_at(self, sp):
        """Toggle the pass nearest the click point (within a few screen pixels)."""
        if not self._route_passes:
            return
        scale = abs(self.view.transform().m11()) or 1.0
        tol = 8.0 / scale
        best_pid, best_d = None, tol
        for pid, d in self._route_passes.items():
            for ax, ay, bx, by in d['segs']:
                dist = _point_seg_dist(sp.x(), sp.y(), ax, ay, bx, by)
                if dist <= best_d:
                    best_d, best_pid = dist, pid
        if best_pid is None:
            return
        d = self._route_passes[best_pid]
        d['selected'] = not d['selected']
        self._style_pass(best_pid)
        n = len(self.selected_pass_ids())
        self.btn_confirm.setEnabled(n > 0)
        self._sync_all_button()
        self.passSelectionChanged.emit(n)

    def _toggle_select_all(self):
        """Select every pass at once — or clear all when they're already all selected —
        so the operator isn't forced to click each pass of a loaded route by hand."""
        if not self._route_passes:
            return
        want = not all(d['selected'] for d in self._route_passes.values())
        for pid, d in self._route_passes.items():
            d['selected'] = want
            self._style_pass(pid)
        n = len(self.selected_pass_ids())
        self.btn_confirm.setEnabled(n > 0)
        self._sync_all_button()
        self.passSelectionChanged.emit(n)

    def _sync_all_button(self):
        """Label the toggle for the action it will perform next."""
        all_sel = bool(self._route_passes) and all(
            d['selected'] for d in self._route_passes.values())
        self.btn_all.setText('Clear all' if all_sel else 'Select all')

    def selected_pass_ids(self):
        return [pid for pid, d in self._route_passes.items() if d['selected']]

    def clear_route_passes(self):
        for d in self._route_passes.values():
            self.scene.removeItem(d['item'])
        self._route_passes = {}
        self.selecting_passes = False
        self.btn_confirm.setVisible(False); self.btn_confirm.setEnabled(False)
        self.btn_all.setVisible(False); self.btn_all.setEnabled(False)

    def set_dtm(self, dtm, dtm_path=None, chm=None, chm_path=None, focus_polygon=None):
        self.dtm = dtm; self.chm = chm
        self._focus_polygon = None
        self._reset_scene()
        if focus_polygon is not None:
            # Crop straight to the AOI — reads only that window, skipping the (slow)
            # whole-extent overview read entirely.
            self.focus_on(focus_polygon)
            return
        # DISPLAY only: a bounded, decimated whole-extent overview (read once). The scene
        # IS this overview's pixel grid; the full-resolution raster is never rendered, so
        # a giga-pixel DTM shows and pans like a small one with bounded memory. Planning
        # still reads native resolution (planning._aoi_native). For a large DTM this read
        # scans the whole file once — a few seconds at load, then the map is static.
        self._render_overview(*dtm.overview_array(), chm)

    def focus_on(self, polygon, margin_m=250.0):
        """Render ONLY the DTM window around `polygon` (+ margin) at ~native resolution.
        For a giga-pixel DTM this shows the relevant area SHARPLY (a bounded region fits
        the display budget natively) instead of the coarse whole-extent overview. Renders
        the base layer only; the caller re-applies the AOI + route overlays. `polygon` is
        a shapely Polygon in the DTM's CRS."""
        if self.dtm is None:
            return
        self._focus_polygon = polygon
        self._reset_scene()
        view = self.dtm.read_window(polygon.bounds, margin_m=margin_m)
        self._render_overview(*view.overview_array(_FOCUS_DISP_CELLS), self.chm)
        self._fit()

    def show_full(self):
        """Render the whole-extent overview (undo a focus). Base layer only; the caller
        re-applies the AOI + route overlays. The overview is cached on the DTM, so this
        is instant after the initial load."""
        if self.dtm is None:
            return
        self._focus_polygon = None
        self._reset_scene()
        self._render_overview(*self.dtm.overview_array(), self.chm)
        self._fit()

    def _render_overview(self, disp, disp_t, stride, chm):
        """Build the static relief pixmap (+ CHM overlay) from the display overview and
        set up the scene coordinate system. Runs on the GUI thread."""
        self._disp_transform = disp_t
        self._disp_stride = stride
        self._inv = ~disp_t
        self._relief = _shaded_relief(disp, self.dtm.nodata)
        h, w, _ = self._relief.shape
        img = QImage(self._relief.data, w, h, 3 * w, QImage.Format_RGB888)
        self.scene.addItem(QGraphicsPixmapItem(QPixmap.fromImage(img)))
        self.scene.setSceneRect(QRectF(0, 0, w, h))

        if chm is not None:
            # CHM's own bounded overview; a transform places it in the DTM scene grid so
            # it lands correctly even at a different resolution/extent.
            cdisp, chm_t, _cs = chm.overview_array()
            self._chm_rgba = _chm_rgba(cdisp, chm.nodata)
            ch, cw, _ = self._chm_rgba.shape
            cimg = QImage(self._chm_rgba.data, cw, ch, 4 * cw, QImage.Format_RGBA8888)
            self._chm_item = QGraphicsPixmapItem(QPixmap.fromImage(cimg))
            m = self._inv * chm_t
            self._chm_item.setTransform(QTransform(m.a, m.d, m.b, m.e, m.c, m.f))
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
        x, y = self._disp_transform * (sp.x(), sp.y())
        return x, y

    def _scene(self, lon, lat):
        c, r = self._inv * (lon, lat)
        return QPointF(c, r)

    def on_hover(self, sp):
        if self.dtm is None:
            return
        lon, lat = self._world(sp)
        z = self.dtm.elevation_at(lon, lat)
        ztxt = f'{z:.0f} m' if z == z else '000 m'      # NaN check
        self.lbl_coord.setText(f'{lat:.5f}, {lon:.5f}   ·   {ztxt}')
        if self.drawing_pass:
            self._update_pass_preview(sp)
        else:
            self._pass_tooltip(sp)

    def _pass_tooltip(self, sp):
        """Show the pass altitude in a tooltip when the cursor is near a pass line."""
        if not self._pass_segs:
            QToolTip.hideText()
            return
        # hover tolerance: a few screen pixels expressed in scene units
        scale = abs(self.view.transform().m11()) or 1.0
        tol = 6.0 / scale
        best_z, best_d = None, tol
        for ax, ay, bx, by, z in self._pass_segs:
            d = _point_seg_dist(sp.x(), sp.y(), ax, ay, bx, by)
            if d <= best_d:
                best_d, best_z = d, z
        if best_z is not None:
            QToolTip.showText(QCursor.pos(), f'Pass altitude: {best_z:.0f} m')
        else:
            QToolTip.hideText()

    # ----------------------------------------------------------- drawing
    def _toggle_draw(self, on):
        self.drawing = on
        self.view.setDragMode(QGraphicsView.NoDrag if on else QGraphicsView.ScrollHandDrag)
        self.view.setCursor(Qt.CrossCursor if on else Qt.ArrowCursor)
        if on:
            if self.drawing_pass:                       # AOI and pass draw are exclusive
                self.btn_pass.setChecked(False); self._toggle_pass(False)
            self.clear_aoi()

    # ----------------------------------------------------------- manual passes
    def _toggle_pass(self, on):
        self.drawing_pass = on
        self.view.setDragMode(QGraphicsView.NoDrag if on else QGraphicsView.ScrollHandDrag)
        self.view.setCursor(Qt.CrossCursor if on else Qt.ArrowCursor)
        if on and self.drawing:
            self.btn_draw.setChecked(False); self._toggle_draw(False)
        if not on:
            self._clear_pass_temp()

    def set_pass_anchor(self, lon, lat):
        """Where a drawn pass starts from — the route's current end. The preview
        rubber-bands from here to the cursor."""
        self._pass_anchor = (self._scene(lon, lat)
                             if self.dtm is not None and lon is not None else None)

    def add_pass_vertex(self, sp):
        """Single click: the click is the END of a new pass; its start is the route
        end (anchor). Emits the clicked (lon, lat). Pass-draw stays on for chaining."""
        self.passDrawn.emit(self._world(sp))

    def _update_pass_preview(self, sp):
        if self._pass_anchor is None:
            return
        path = QPainterPath(self._pass_anchor); path.lineTo(sp)
        if self._pass_preview is None:
            self._pass_preview = QGraphicsPathItem()
            pen = QPen(QColor('#3fb950'), 2); pen.setCosmetic(True); pen.setStyle(Qt.DashLine)
            self._pass_preview.setPen(pen)
            self.scene.addItem(self._pass_preview)
        self._pass_preview.setPath(path)

    def _clear_pass_temp(self):
        if self._pass_preview is not None:
            self.scene.removeItem(self._pass_preview); self._pass_preview = None

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
        self.dtm = None; self.chm = None; self._inv = None; self._disp_transform = None
        self._focus_polygon = None
        self._route_passes = {}; self.selecting_passes = False
        self.btn_confirm.setVisible(False); self.btn_confirm.setEnabled(False)
        self.btn_all.setVisible(False); self.btn_all.setEnabled(False)
        self._aoi_item = self._route_group = self._density_item = None
        self._helios_item = self._chm_item = None; self._home_item = None
        self._verts = []; self._draw_items = []; self._pass_segs = []
        self._pass_anchor = None; self._pass_preview = None
        self.drawing = False; self.drawing_pass = False
        self.btn_draw.setChecked(False)
        self.btn_pass.setChecked(False); self.btn_pass.setEnabled(False)
        self.btn_focus.blockSignals(True)
        self.btn_focus.setChecked(False); self.btn_focus.setEnabled(False)
        self.btn_focus.blockSignals(False)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.view.setCursor(Qt.ArrowCursor)
        self.btn_chm.setChecked(False); self.btn_chm.setEnabled(False)
        self.lbl_coord.setText('')

    def _toggle_chm(self, on):
        if self._chm_item is not None:
            self._chm_item.setVisible(on)

    # ----------------------------------------------------------- overlays
    def show_plan(self, route_wps, density_cells, density_color='#ff9900',
                  density_radius_m=3.0, max_density_pts=20000,
                  cells_by_reason=None, connect_passes=True):
        if self.dtm is None:
            return
        # clear previous overlays
        if self._route_group is not None:
            self.scene.removeItem(self._route_group); self._route_group = None
        if self._density_item is not None:
            self.scene.removeItem(self._density_item); self._density_item = None

        # ── under-density: overlay image of ground-sized dots (scales w/ zoom) ──
        # Prefer the cause-coloured breakdown when the estimate provides it; fall
        # back to a single colour for older results / callers.
        if cells_by_reason:
            # thin cells are drawn as a density gradient (see _paint_cell_layers), the
            # rest as their flat cause colour.
            layers = [(cells_by_reason.get(k, []), hexc, alpha)
                      for k, (hexc, alpha) in FAILURE_REASON_STYLE.items() if k != 'thin']
            self._density_item = self._paint_cell_layers(
                layers, cells_by_reason.get('thin', []), density_radius_m, max_density_pts)
        elif density_cells:
            self._density_item = self._paint_cells(
                density_cells, density_color, 90, density_radius_m, max_density_pts)

        # ── route: altitude-coloured cosmetic polylines + start/end markers ──
        wps = [w for w in (route_wps or [])
               if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        self._pass_segs = []
        if len(wps) >= 2:
            grp = QGraphicsItemGroup(); self.scene.addItem(grp)
            zs = [w['z'] for w in wps]; zmin, zmax = min(zs), max(zs)
            cmap = plt.get_cmap('cool')
            for a, b in zip(wps, wps[1:]):
                same_pass = (a.get('pass_id') is not None
                             and a.get('pass_id') == b.get('pass_id'))
                if not same_pass and not connect_passes:
                    continue                 # independent passes — don't draw a connector
                t = (a['z'] - zmin) / max(zmax - zmin, 1e-9)
                pa = self._scene(a['x'], a['y']); pb = self._scene(b['x'], b['y'])
                seg = QPainterPath(pa); seg.lineTo(pb)
                item = QGraphicsPathItem(seg)
                pen = QPen(QColor(mcolors.to_hex(cmap(t))), 2); pen.setCosmetic(True)
                item.setPen(pen); grp.addToGroup(item)
                # remember the pass lines (both ends same pass_id) for hover readout;
                # skip the inter-pass connector legs.
                if same_pass:
                    self._pass_segs.append((pa.x(), pa.y(), pb.x(), pb.y(), a['z']))
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

    def _paint_cell_layers(self, layers, thin_cells, radius_m, max_pts=20000):
        """Paint the flat-colour cause layers plus the THIN cells as a density gradient
        onto ONE overlay image. `layers` = [(cells[(lon,lat)], color_hex, alpha), …];
        `thin_cells` = [(lon, lat, frac), …] with frac = density/target in [0,1] — drawn
        via a red→yellow heat map (0 = empty, target = yellow), so the operator sees HOW
        thin, not just that a cell is thin. Each set is down-sampled to keep it bounded."""
        h, w, _ = self._relief.shape
        ov = QImage(w, h, QImage.Format_RGBA8888); ov.fill(0)
        p = QPainter(ov)
        p.setPen(QPen(Qt.NoPen))
        rad_px = max(1.0, radius_m / self._pixel_m())

        def _sub(cells):
            if len(cells) <= max_pts:
                return cells
            step = len(cells) / max_pts
            return [cells[int(i * step)] for i in range(max_pts)]

        for cells, color_hex, alpha in layers:
            if not cells:
                continue
            col = QColor(color_hex); col.setAlpha(alpha)
            p.setBrush(QBrush(col))
            for lon, lat in _sub(cells):
                c, r = self._inv * (lon, lat)
                p.drawEllipse(QPointF(c, r), rad_px, rad_px)

        if thin_cells:
            for lon, lat, frac in _sub(thin_cells):
                rr, gg, bb, _a = _THIN_CMAP(float(frac))   # 0→red, 1(target)→yellow
                p.setBrush(QBrush(QColor(int(rr * 255), int(gg * 255), int(bb * 255), 150)))
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

    def show_home(self, home, wps):
        """Draw the takeoff/return-home point (entered as a coordinate) and dashed
        ferry legs to the first and last survey waypoints. `home` is (lon, lat);
        pass home=None to remove it. The scene is grown to keep the point reachable
        even when it lies outside the DTM extent."""
        if self._home_item is not None:
            self.scene.removeItem(self._home_item); self._home_item = None
        if self.dtm is None or home is None:
            return
        grp = QGraphicsItemGroup(); self.scene.addItem(grp)
        hp = self._scene(home[0], home[1])
        valid = [w for w in (wps or [])
                 if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        if valid:
            pen = QPen(QColor('#f0b000'), 2); pen.setCosmetic(True)
            pen.setStyle(Qt.DashLine)
            for end in (valid[0], valid[-1]):
                path = QPainterPath(hp); path.lineTo(self._scene(end['x'], end['y']))
                seg = QGraphicsPathItem(path); seg.setPen(pen); grp.addToGroup(seg)
        m = QGraphicsEllipseItem(-6, -6, 12, 12)
        m.setPos(hp); m.setBrush(QBrush(QColor('#f0b000')))
        m.setPen(QPen(Qt.black, 1.5))
        m.setFlag(QGraphicsEllipseItem.ItemIgnoresTransformations)
        grp.addToGroup(m)
        self._home_item = grp
        self.scene.setSceneRect(self.scene.sceneRect().united(
            QRectF(hp.x() - 20, hp.y() - 20, 40, 40)))

    def clear_overlays(self):
        """Remove route + density + HELIOS overlays (keeps the DTM and drawn AOI).
        The home marker is managed separately via show_home so it persists."""
        for attr in ('_route_group', '_density_item', '_helios_item'):
            it = getattr(self, attr, None)
            if it is not None:
                self.scene.removeItem(it)
                setattr(self, attr, None)
        self._pass_segs = []

    def _marker(self, grp, sp, hexcolor):
        m = QGraphicsEllipseItem(-5, -5, 10, 10)
        m.setPos(sp); m.setBrush(QBrush(QColor(hexcolor)))
        m.setPen(QPen(Qt.white, 1))
        m.setFlag(QGraphicsEllipseItem.ItemIgnoresTransformations)
        grp.addToGroup(m)

    def _pixel_m(self):
        # metres per SCENE pixel = stride native pixels (the scene is the display overview).
        s = getattr(self, '_disp_stride', 1)
        rx, ry = abs(self.dtm.src.res[0]), abs(self.dtm.src.res[1])
        crs = self.dtm.src.crs
        if crs is not None and crs.is_geographic:
            lat0 = (self.dtm.src.bounds.bottom + self.dtm.src.bounds.top) / 2
            return s * (rx * _LAT_M * math.cos(math.radians(lat0)) + ry * _LAT_M) / 2
        return s * (rx + ry) / 2
