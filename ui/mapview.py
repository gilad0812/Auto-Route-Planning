"""Leaflet map embedded in Qt (QWebEngineView) with polygon drawing.

Replaces the "centered box %" AOI stand-in: the DTM (and optional CHM) is shown
as an image overlay over CartoDB tiles, and leaflet-draw lets the user draw the
survey polygon. The drawn GeoJSON is pushed back to Python over a QWebChannel
bridge, which re-emits it as a Qt signal the main window consumes.

DTM bounds are treated as lon/lat (matching the Streamlit app) — geographic
DTMs only, for now.
"""
import base64
import io
import os
import tempfile

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from PySide6.QtCore import QObject, Signal, Slot, QUrl
from PySide6.QtWidgets import QWidget, QVBoxLayout
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebEngineCore import QWebEngineSettings
from PySide6.QtWebChannel import QWebChannel


def _dtm_png(dtm, path):
    arr = dtm.array.astype(float)
    if dtm.nodata is not None:
        arr[arr == dtm.nodata] = np.nan
    vmin, vmax = np.nanmin(arr), np.nanmax(arr)
    normed = np.clip((arr - vmin) / max(vmax - vmin, 1e-9), 0, 1)
    rgba = (plt.get_cmap('terrain')(np.nan_to_num(normed)) * 255).astype(np.uint8)
    if dtm.nodata is not None:
        rgba[dtm.array == dtm.nodata, 3] = 0
    Image.fromarray(rgba, 'RGBA').save(path)


def _chm_png(chm, path):
    arr = chm.array.astype(float)
    if chm.nodata is not None:
        arr[arr == chm.nodata] = np.nan
    veg = np.isfinite(arr) & (arr > 0)
    vmax = np.nanmax(np.where(veg, arr, np.nan)) if veg.any() else 1.0
    normed = np.clip(arr / max(vmax, 1e-9), 0, 1)
    rgba = (plt.get_cmap('Greens')(0.35 + 0.6 * np.nan_to_num(normed)) * 255).astype(np.uint8)
    rgba[~veg, 3] = 0
    Image.fromarray(rgba, 'RGBA').save(path)


class _Bridge(QObject):
    """JS → Python channel object."""
    polygonDrawn = Signal(str)

    @Slot(str)
    def onPolygon(self, geojson):
        self.polygonDrawn.emit(geojson)


_HTML = """<!DOCTYPE html><html><head><meta charset="utf-8">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.css"/>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.css"/>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.js"></script>
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<style>html,body,#map{height:100%;margin:0}</style></head>
<body><div id="map"></div><script>
var bridge=null;
new QWebChannel(qt.webChannelTransport,function(ch){bridge=ch.objects.bridge;});
var map=L.map('map');
L.tileLayer('https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png',
  {attribution:'&copy; OpenStreetMap, &copy; CARTO',maxZoom:20}).addTo(map);
var south=__S__,west=__W__,north=__N__,east=__E__;
var bnds=[[south,west],[north,east]];
__DTM_OVERLAY__
__CHM_OVERLAY__
map.fitBounds(bnds);
var drawn=new L.FeatureGroup();map.addLayer(drawn);
var draw=new L.Control.Draw({
  edit:{featureGroup:drawn},
  draw:{polygon:{allowIntersection:false,showArea:true},
        rectangle:true,polyline:false,circle:false,marker:false,circlemarker:false}
});
map.addControl(draw);
function emit(layer){
  if(bridge){bridge.onPolygon(JSON.stringify(layer.toGeoJSON().geometry));}
}
map.on(L.Draw.Event.CREATED,function(e){
  drawn.clearLayers();              // keep a single AOI
  drawn.addLayer(e.layer);
  emit(e.layer);
});
map.on(L.Draw.Event.EDITED,function(e){
  e.layers.eachLayer(function(l){emit(l);});
});
</script></body></html>"""


class MapView(QWidget):
    polygonDrawn = Signal(object)        # re-emits a GeoJSON geometry dict

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.web = QWebEngineView()
        s = self.web.settings()
        s.setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls, True)
        s.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
        layout.addWidget(self.web)

        self.bridge = _Bridge()
        self.channel = QWebChannel()
        self.channel.registerObject('bridge', self.bridge)
        self.web.page().setWebChannel(self.channel)
        self.bridge.polygonDrawn.connect(self._on_polygon)

        self._tmpdir = tempfile.mkdtemp(prefix='routeplanner_map_')
        self.web.setHtml('<body style="font-family:sans-serif;color:#888;'
                         'padding:1em">Open a DTM to load the map.</body>')

    def _on_polygon(self, geojson_str):
        import json
        try:
            geom = json.loads(geojson_str)
        except Exception:
            return
        self.polygonDrawn.emit(geom)

    def set_dtm(self, dtm, dtm_path, chm=None, chm_path=None):
        b = dtm.src.bounds
        dtm_file = os.path.join(self._tmpdir, 'dtm.png')
        _dtm_png(dtm, dtm_file)
        dtm_overlay = (f"L.imageOverlay('dtm.png',bnds,{{opacity:0.6}}).addTo(map);")

        chm_overlay = ''
        if chm is not None:
            chm_file = os.path.join(self._tmpdir, 'chm.png')
            _chm_png(chm, chm_file)
            cb = chm.src.bounds
            chm_overlay = (
                f"var chmLayer=L.imageOverlay('chm.png',"
                f"[[{cb.bottom},{cb.left}],[{cb.top},{cb.right}]],{{opacity:0.7}});"
                "L.control.layers(null,{'CHM (vegetation)':chmLayer}).addTo(map);")

        html = (_HTML
                .replace('__S__', repr(b.bottom)).replace('__W__', repr(b.left))
                .replace('__N__', repr(b.top)).replace('__E__', repr(b.right))
                .replace('__DTM_OVERLAY__', dtm_overlay)
                .replace('__CHM_OVERLAY__', chm_overlay))
        base = QUrl.fromLocalFile(self._tmpdir + os.sep)
        self.web.setHtml(html, base)
