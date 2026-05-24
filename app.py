import sys
import os
import math
import json
import csv
import io
import base64

import numpy as np
import streamlit as st
import folium
from folium.plugins import Draw
from streamlit_folium import st_folium
from matplotlib import cm as mpl_cm, colors as mpl_colors
import matplotlib.pyplot as plt
from PIL import Image
from shapely.geometry import shape as shapely_shape

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from dtm import DTM
from route_planner import plan_route

st.set_page_config(page_title='LiDAR Drone Route Planner', layout='wide')

# ------------------------------------------------------------------ helpers

@st.cache_data
def dtm_to_png(dtm_path: str) -> bytes:
    dtm = DTM(dtm_path)
    arr = dtm.array.astype(float)
    if dtm.nodata is not None:
        arr[arr == dtm.nodata] = np.nan
    vmin, vmax = np.nanmin(arr), np.nanmax(arr)
    normed = np.clip((arr - vmin) / max(vmax - vmin, 1e-9), 0, 1)
    rgba = (plt.get_cmap('terrain')(np.nan_to_num(normed)) * 255).astype(np.uint8)
    if dtm.nodata is not None:
        rgba[dtm.array == dtm.nodata, 3] = 0
    buf = io.BytesIO()
    Image.fromarray(rgba, 'RGBA').save(buf, format='PNG')
    return buf.getvalue()


@st.cache_resource
def load_dtm(path: str) -> DTM:
    return DTM(path)


def swath_and_spacing(altitude_m: float, fov_deg: float, overlap_pct: float, is_geo: bool):
    swath_m = 2 * altitude_m * math.tan(math.radians(fov_deg / 2))
    spacing_m = swath_m * (1.0 - overlap_pct / 100.0)
    factor = 1 / 111139.0 if is_geo else 1.0
    return swath_m, spacing_m, spacing_m * factor


def to_map_units(meters: float, is_geo: bool) -> float:
    return meters / 111139.0 if is_geo else meters


def z_to_hex(z: float, zmin: float, zmax: float) -> str:
    t = np.clip((z - zmin) / max(zmax - zmin, 1e-9), 0, 1)
    return mpl_colors.to_hex(mpl_cm.cool(t))


# ------------------------------------------------------------------ session state

for _k, _v in [('polygon', None), ('route', None)]:
    if _k not in st.session_state:
        st.session_state[_k] = _v

# ------------------------------------------------------------------ sidebar

with st.sidebar:
    st.title('Flight Parameters')
    dtm_path = st.text_input('DTM file path', value='data/dtm.tif')
    st.divider()

    altitude  = st.number_input('Altitude AGL (m)',      value=80.0,  min_value=1.0,    step=5.0)
    fov       = st.number_input('LiDAR FOV (°)',         value=60.0,  min_value=1.0,    max_value=179.0, step=5.0)
    overlap   = st.number_input('Overlap (%)',           value=20.0,  min_value=0.0,    max_value=99.0,  step=5.0)
    step_m    = st.number_input('Along-track step (m)',  value=50.0,  min_value=1.0,    step=5.0)
    error_tol = st.number_input('Error tolerance (m)',   value=2.0,   min_value=0.1,    step=0.5)
    st.divider()

    compute_btn = st.button(
        'Compute Route',
        disabled=st.session_state.polygon is None,
        use_container_width=True,
        type='primary',
    )
    if st.button('Clear / Reset', use_container_width=True):
        st.session_state.polygon = None
        st.session_state.route = None
        st.rerun()

# ------------------------------------------------------------------ load DTM

if not os.path.exists(dtm_path):
    st.error(f'DTM file not found: `{dtm_path}`')
    st.stop()

dtm = load_dtm(dtm_path)
bounds = dtm.src.bounds
is_geo = dtm.src.crs.is_geographic if dtm.src.crs else True
elev_min, elev_max = float(np.nanmin(dtm.array)), float(np.nanmax(dtm.array))

swath_m, spacing_m, spacing_map = swath_and_spacing(altitude, fov, overlap, is_geo)
step_map = to_map_units(step_m, is_geo)

with st.sidebar:
    st.caption(
        f'Swath width: **{swath_m:.1f} m**  \n'
        f'Pass spacing: **{spacing_m:.1f} m** ({overlap:.0f}% overlap)'
    )

# ------------------------------------------------------------------ compute route

if compute_btn and st.session_state.polygon is not None:
    poly = shapely_shape(st.session_state.polygon['geometry'])
    if not poly.is_valid:
        poly = poly.buffer(0)
    with st.spinner('Computing route…'):
        st.session_state.route = plan_route(dtm, poly, altitude, error_tol, spacing_map, step_map)

# ------------------------------------------------------------------ build map

center = [(bounds.bottom + bounds.top) / 2, (bounds.left + bounds.right) / 2]
m = folium.Map(location=center, zoom_start=12, tiles='CartoDB positron')

# DTM overlay
png_b64 = base64.b64encode(dtm_to_png(dtm_path)).decode()
folium.raster_layers.ImageOverlay(
    image=f'data:image/png;base64,{png_b64}',
    bounds=[[bounds.bottom, bounds.left], [bounds.top, bounds.right]],
    opacity=0.75,
    name='DTM elevation',
).add_to(m)

# Draw tool
Draw(
    export=False,
    draw_options={
        'polygon':      {'allowIntersection': False},
        'rectangle':    {'showArea': True},
        'polyline':     False,
        'circle':       False,
        'marker':       False,
        'circlemarker': False,
    },
    edit_options={'edit': True, 'remove': True},
).add_to(m)

# Stored polygon
if st.session_state.polygon:
    folium.GeoJson(
        st.session_state.polygon,
        style_function=lambda _: {'color': '#ff4444', 'weight': 2.5, 'fillOpacity': 0.08},
        name='Scan polygon',
    ).add_to(m)

# Route
if st.session_state.route:
    wps = [wp for wp in st.session_state.route
           if not (isinstance(wp['z'], float) and math.isnan(wp['z']))]
    if len(wps) >= 2:
        zs = [wp['z'] for wp in wps]
        zmin, zmax = min(zs), max(zs)
        latlons = [[wp['y'], wp['x']] for wp in wps]
        for i in range(len(wps) - 1):
            folium.PolyLine(
                [latlons[i], latlons[i + 1]],
                color=z_to_hex(zs[i], zmin, zmax),
                weight=3, opacity=0.9,
                tooltip=f'{zs[i]:.1f} m',
            ).add_to(m)
        folium.Marker(latlons[0],  tooltip=f'Start  {zs[0]:.1f} m',
                      icon=folium.Icon(color='green', icon='play',  prefix='fa')).add_to(m)
        folium.Marker(latlons[-1], tooltip=f'End  {zs[-1]:.1f} m',
                      icon=folium.Icon(color='red',   icon='stop',  prefix='fa')).add_to(m)

folium.LayerControl().add_to(m)

# ------------------------------------------------------------------ render

st.title('LiDAR Drone Route Planner')

map_col, info_col = st.columns([3, 1])

with map_col:
    output = st_folium(m, width='100%', height=640)

with info_col:
    st.markdown('**How to use**')
    st.markdown(
        '1. Draw a polygon on the map  \n'
        '2. Set params in the sidebar  \n'
        '3. Click **Compute Route**  \n'
        '4. Download results below'
    )
    st.divider()
    st.markdown('**DTM**')
    st.markdown(
        f'Elevation: {elev_min:.0f} – {elev_max:.0f} m  \n'
        f'CRS: `{dtm.src.crs}`  \n'
        f'Size: {dtm.array.shape[1]} × {dtm.array.shape[0]} px'
    )
    if st.session_state.route:
        wps = [wp for wp in st.session_state.route
               if not (isinstance(wp['z'], float) and math.isnan(wp['z']))]
        if wps:
            xs = [wp['x'] for wp in wps]
            ys = [wp['y'] for wp in wps]
            zs = [wp['z'] for wp in wps]
            total = sum(math.sqrt((xs[i+1]-xs[i])**2 + (ys[i+1]-ys[i])**2)
                        for i in range(len(xs)-1))
            st.divider()
            st.markdown('**Route**')
            st.metric('Waypoints', len(wps))
            st.metric('Path length', f'{total:.4f} °')
            st.metric('Alt range', f'{min(zs):.0f} – {max(zs):.0f} m')

# ------------------------------------------------------------------ capture new drawing

drawings = (output or {}).get('all_drawings') or []
if drawings:
    latest = next(
        (d for d in reversed(drawings)
         if d.get('geometry', {}).get('type') in ('Polygon',)),
        None,
    )
    if latest and latest != st.session_state.polygon:
        st.session_state.polygon = latest
        st.session_state.route = None
        st.rerun()

# ------------------------------------------------------------------ downloads

if st.session_state.route:
    wps = [wp for wp in st.session_state.route
           if not (isinstance(wp['z'], float) and math.isnan(wp['z']))]
    if wps:
        st.divider()
        dl1, dl2 = st.columns(2)

        features = [
            {
                'type': 'Feature',
                'geometry': {'type': 'Point', 'coordinates': [wp['x'], wp['y'], wp['z']]},
                'properties': {
                    'altitude_m':   wp['z'],
                    'target_agl_m': wp['target_distance'],
                    'error_tol_m':  wp['error_tol'],
                },
            }
            for wp in wps
        ]
        dl1.download_button(
            'Download GeoJSON',
            json.dumps({'type': 'FeatureCollection', 'features': features}, indent=2),
            file_name='route.geojson', mime='application/json',
            use_container_width=True,
        )

        buf = io.StringIO()
        w = csv.DictWriter(buf, fieldnames=['index', 'x', 'y', 'z', 'target_agl_m', 'error_tol_m'])
        w.writeheader()
        for i, wp in enumerate(wps):
            w.writerow({'index': i, 'x': wp['x'], 'y': wp['y'], 'z': wp['z'],
                        'target_agl_m': wp['target_distance'], 'error_tol_m': wp['error_tol']})
        dl2.download_button(
            'Download CSV', buf.getvalue(),
            file_name='route.csv', mime='text/csv',
            use_container_width=True,
        )