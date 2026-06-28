"""Leaflet map embedded in Qt (QWebEngineView) with polygon drawing.

Replaces the "centered box %" AOI stand-in: the DTM (and optional CHM) is shown
as an image overlay over CartoDB tiles, and leaflet-draw lets the user draw the
survey polygon. The drawn GeoJSON is pushed back to Python over a QWebChannel
bridge, which re-emits it as a Qt signal the main window consumes.

DTM bounds are treated as lon/lat (matching the Streamlit app) — geographic
DTMs only, for now.
"""
import json
import os
import tempfile

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors as mcolors
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
    vmin, vmax = float(np.nanmin(arr)), float(np.nanmax(arr))
    normed = np.clip((arr - vmin) / max(vmax - vmin, 1e-9), 0, 1)
    rgba = (plt.get_cmap('terrain')(np.nan_to_num(normed)) * 255).astype(np.uint8)
    if dtm.nodata is not None:
        rgba[dtm.array == dtm.nodata, 3] = 0
    Image.fromarray(rgba, 'RGBA').save(path)
    return vmin, vmax


def _colorbar_png(path):
    """A 1×256 'terrain' gradient strip, stretched by CSS into the legend."""
    grad = np.linspace(0, 1, 256).reshape(1, 256)
    rgba = (plt.get_cmap('terrain')(grad) * 255).astype(np.uint8)
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


def route_segments(wps):
    """Build altitude-coloured polyline segments (lat/lon) + start/end markers
    for the route, mirroring the Streamlit map's cool-colormap-by-altitude look.
    Returns (segments, start_latlon, end_latlon)."""
    zs = [w['z'] for w in wps]
    zmin, zmax = min(zs), max(zs)
    cmap = plt.get_cmap('cool')
    segs = []
    for a, b in zip(wps, wps[1:]):
        t = (a['z'] - zmin) / max(zmax - zmin, 1e-9)
        segs.append({'p': [[a['y'], a['x']], [b['y'], b['x']]],
                     'c': mcolors.to_hex(cmap(t)),
                     't': f"{a['z']:.0f} m"})
    start = [wps[0]['y'], wps[0]['x']]
    end = [wps[-1]['y'], wps[-1]['x']]
    return segs, start, end


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
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.heat/0.2.0/leaflet-heat.js"></script>
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<style>html,body,#map{height:100%;margin:0}</style></head>
<body><div id="map"></div><script>
var bridge=null;
new QWebChannel(qt.webChannelTransport,function(ch){bridge=ch.objects.bridge;});
var map=L.map('map',{preferCanvas:true});
L.tileLayer('https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png',
  {attribution:'&copy; OpenStreetMap, &copy; CARTO',maxZoom:20}).addTo(map);
var south=__S__,west=__W__,north=__N__,east=__E__;
var bnds=L.latLngBounds([south,west],[north,east]);
__DTM_OVERLAY__
__CHM_OVERLAY__
// Outline the DTM extent so the surveyable area is unmistakable.
L.rectangle(bnds,{color:'#333',weight:1.5,fill:false,dashArray:'5,4'}).addTo(map);
// Lock the view onto the DTM: fit it, then constrain pan/zoom to its extent.
map.fitBounds(bnds);
map.setMaxBounds(bnds.pad(0.15));
map.options.maxBoundsViscosity=1.0;
map.setMinZoom(map.getBoundsZoom(bnds));
// The web view often has no size yet at load — refit once it does.
function refit(){map.invalidateSize();map.fitBounds(bnds);map.setMinZoom(map.getBoundsZoom(bnds));}
window.addEventListener('resize',refit);
setTimeout(refit,150);setTimeout(refit,600);
// Elevation legend (marks the area by its values).
var legend=L.control({position:'bottomright'});
legend.onAdd=function(){
  var d=L.DomUtil.create('div');
  d.style.cssText='background:#fff;padding:4px 7px;border:1px solid #aaa;'+
    'border-radius:3px;font:11px sans-serif;color:#222;box-shadow:0 1px 4px rgba(0,0,0,.3)';
  d.innerHTML='Elevation (m)<br><span>__VMIN__</span> '+
    '<img src="cbar.png" style="height:11px;width:90px;vertical-align:middle;'+
    'image-rendering:auto"> <span>__VMAX__</span>';
  return d;};
legend.addTo(map);
// Overlay layers for the plan results, toggleable from one control.
var routeLayer=L.layerGroup().addTo(map);
var densityLayer=L.layerGroup().addTo(map);
var overlays={'Route':routeLayer,'Under-density':densityLayer};
if(typeof chmLayer!=='undefined'&&chmLayer){overlays['CHM (vegetation)']=chmLayer;}
L.control.layers(null,overlays,{collapsed:false}).addTo(map);

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

// ── API called from Python via runJavaScript ──────────────────────────────
window.showRoute=function(segs,start,end){
  routeLayer.clearLayers();
  segs.forEach(function(s){
    L.polyline(s.p,{color:s.c,weight:3,opacity:0.9}).bindTooltip(s.t).addTo(routeLayer);
  });
  L.circleMarker(start,{radius:6,color:'#1a7f37',fillColor:'#1a7f37',
    fillOpacity:1,weight:1}).bindTooltip('Start').addTo(routeLayer);
  L.circleMarker(end,{radius:6,color:'#cf222e',fillColor:'#cf222e',
    fillOpacity:1,weight:1}).bindTooltip('End').addTo(routeLayer);
};
window.clearRoute=function(){routeLayer.clearLayers();};
window.showDensity=function(pts,color,radiusM){
  densityLayer.clearLayers();
  if(!pts.length)return;
  // Ground-sized circles (metres): tiny when zoomed out so they don't flood
  // the map, real-size when zoomed in. Low opacity keeps it a hint, not a wall.
  pts.forEach(function(p){
    L.circle([p[0],p[1]],{radius:radiusM,color:color,fillColor:color,
      fillOpacity:0.30,weight:0,stroke:false}).addTo(densityLayer);
  });
};
window.clearDensity=function(){densityLayer.clearLayers();};
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

        # runJavaScript only works once the page has loaded; queue until then.
        self._loaded = False
        self._pending = []
        self.web.loadFinished.connect(self._on_load)

        self._tmpdir = tempfile.mkdtemp(prefix='routeplanner_map_')
        self.web.setHtml('<body style="font-family:sans-serif;color:#888;'
                         'padding:1em">Open a DTM to load the map.</body>')

    def _on_load(self, ok):
        self._loaded = True
        for js in self._pending:
            self.web.page().runJavaScript(js)
        self._pending = []

    def _run_js(self, js):
        if self._loaded:
            self.web.page().runJavaScript(js)
        else:
            self._pending.append(js)

    def _on_polygon(self, geojson_str):
        try:
            geom = json.loads(geojson_str)
        except Exception:
            return
        self.polygonDrawn.emit(geom)

    def show_plan(self, route_wps, density_cells, density_color='#ff9900',
                  density_radius_m=2.0, max_density_pts=6000):
        """Draw the route (altitude-coloured) and under-density cells on the map.
        density_cells is a list of (lon, lat). Cells are drawn as ground-sized
        circles (density_radius_m), capped/subsampled so they stay a hint rather
        than flooding the map when zoomed out."""
        if route_wps and len(route_wps) >= 2:
            segs, start, end = route_segments(route_wps)
            self._run_js(f'showRoute({json.dumps(segs)},'
                         f'{json.dumps(start)},{json.dumps(end)});')
        else:
            self._run_js('clearRoute();')
        pts = [[lat, lon] for lon, lat in (density_cells or [])]
        if pts:
            if len(pts) > max_density_pts:
                step = len(pts) / max_density_pts
                pts = [pts[int(i * step)] for i in range(max_density_pts)]
            self._run_js(f'showDensity({json.dumps(pts)},'
                         f'{json.dumps(density_color)},{float(density_radius_m)});')
        else:
            self._run_js('clearDensity();')

    def clear_overlays(self):
        self._run_js('clearRoute();clearDensity();')

    def set_dtm(self, dtm, dtm_path, chm=None, chm_path=None):
        # Reloading the page resets the JS context; drop any queued/old calls.
        self._loaded = False
        self._pending = []
        b = dtm.src.bounds
        dtm_file = os.path.join(self._tmpdir, 'dtm.png')
        vmin, vmax = _dtm_png(dtm, dtm_file)
        _colorbar_png(os.path.join(self._tmpdir, 'cbar.png'))
        dtm_overlay = (f"L.imageOverlay('dtm.png',bnds,{{opacity:0.6}}).addTo(map);")

        chm_overlay = ''
        if chm is not None:
            chm_file = os.path.join(self._tmpdir, 'chm.png')
            _chm_png(chm, chm_file)
            cb = chm.src.bounds
            chm_overlay = (
                f"var chmLayer=L.imageOverlay('chm.png',"
                f"[[{cb.bottom},{cb.left}],[{cb.top},{cb.right}]],{{opacity:0.7}});")

        html = (_HTML
                .replace('__S__', repr(b.bottom)).replace('__W__', repr(b.left))
                .replace('__N__', repr(b.top)).replace('__E__', repr(b.right))
                .replace('__VMIN__', f'{vmin:.0f}').replace('__VMAX__', f'{vmax:.0f}')
                .replace('__DTM_OVERLAY__', dtm_overlay)
                .replace('__CHM_OVERLAY__', chm_overlay))
        base = QUrl.fromLocalFile(self._tmpdir + os.sep)
        self.web.setHtml(html, base)
