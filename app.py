import sys
import os
import math
import json
import csv
import io
import base64
import tempfile
import threading
import queue

import numpy as np
import streamlit as st
import folium
from folium.plugins import Draw, HeatMap
from streamlit_folium import st_folium
from matplotlib import cm as mpl_cm, colors as mpl_colors
import matplotlib.pyplot as plt
from PIL import Image
from shapely.geometry import shape as shapely_shape

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from dtm import DTM
from route_planner import plan_route, plan_route_adaptive

try:
    from helios_integration import run_feedback_loop, export_trajectory
    from terrain_converter import dtm_to_obj
    from helios_setup import find_helios_binary, download_and_install
    from helios_config import (
        DEFAULT_MIN_POINTS_PER_SQM, DEFAULT_DRONE_SPEED_MS,
        DEFAULT_PULSE_FREQ_HZ, DEFAULT_SCAN_FREQ_HZ, DEFAULT_SCAN_ANGLE_DEG,
        DEFAULT_MAX_ITERATIONS, DEFAULT_DTM_MESH_STEP_M,
        DEFAULT_SCANNER_REF, DEFAULT_PLATFORM_REF,
    )
    _HELIOS_AVAILABLE = True
except ImportError:
    _HELIOS_AVAILABLE = False
    DEFAULT_MIN_POINTS_PER_SQM = 50
    DEFAULT_DRONE_SPEED_MS = 5.0
    DEFAULT_PULSE_FREQ_HZ = 300_000
    DEFAULT_SCAN_FREQ_HZ = 100.0
    DEFAULT_SCAN_ANGLE_DEG = 30.0
    DEFAULT_MAX_ITERATIONS = 3
    DEFAULT_DTM_MESH_STEP_M = 2.0
    DEFAULT_SCANNER_REF = "data/scanners/als.xml#als_default"
    DEFAULT_PLATFORM_REF = "data/platforms.xml#linearpath"

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


def polygon_area_m2(poly, is_geo: bool) -> float:
    if is_geo:
        lat_c = poly.centroid.y
        return poly.area * (111139.0 ** 2) * math.cos(math.radians(lat_c))
    return poly.area


def z_to_hex(z: float, zmin: float, zmax: float) -> str:
    t = np.clip((z - zmin) / max(zmax - zmin, 1e-9), 0, 1)
    return mpl_colors.to_hex(mpl_cm.cool(t))


# ------------------------------------------------------------------ session state

for _k, _v in [
    ('polygon', None), ('route', None), ('dtm_tmp_path', None),
    ('dtm_upload_name', None), ('helios_result', None),
    ('helios_bin', ''), ('helios_scene_obj', ''),
    ('helios_installing', False), ('helios_expander_open', False),
    ('helios_running', False), ('helios_stop_event', None),
    ('helios_queue', None), ('helios_log', []),
    ('helios_scene_ref_lon', None), ('helios_scene_ref_lat', None),
]:
    if _k not in st.session_state:
        st.session_state[_k] = _v

# Auto-detect HELIOS++ binary once per session (skipped if already found)
if _HELIOS_AVAILABLE and not st.session_state.helios_bin:
    _detected = find_helios_binary()
    if _detected:
        st.session_state.helios_bin = str(_detected)

# ------------------------------------------------------------------ sidebar

with st.sidebar:
    st.title('Flight Parameters')

    uploaded = st.file_uploader(
        'Upload DTM/DEM',
        type=['tif', 'tiff', 'img', 'dem', 'asc', 'hgt'],
        help='GeoTIFF or other GDAL-supported raster',
    )
    if uploaded is not None and uploaded.name != st.session_state.dtm_upload_name:
        suffix = os.path.splitext(uploaded.name)[1]
        tmp = tempfile.NamedTemporaryFile(suffix=suffix, delete=False)
        tmp.write(uploaded.read())
        tmp.flush()
        tmp.close()
        st.session_state.dtm_tmp_path = tmp.name
        st.session_state.dtm_upload_name = uploaded.name
        st.session_state.polygon = None
        st.session_state.route = None

    if st.session_state.dtm_tmp_path:
        dtm_path = st.session_state.dtm_tmp_path
        st.caption(f'Using uploaded file: **{st.session_state.dtm_upload_name}**')
    else:
        dtm_path = st.text_input('or enter local file path', value='') or None

    st.divider()

    with st.expander('Enter polygon manually'):
        st.caption('One vertex per line: `lon, lat`')
        coord_text = st.text_area(
            'Coordinates',
            placeholder='34.12345, 31.98765\n34.12400, 31.98765\n34.12400, 31.98700\n34.12345, 31.98700',
            height=160,
            label_visibility='collapsed',
        )
        if st.button('Apply polygon', use_container_width=True):
            try:
                pts = []
                for line in coord_text.strip().splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.replace(';', ',').split(',')
                    if len(parts) != 2:
                        raise ValueError(f'Expected "lon, lat" — got: {line!r}')
                    pts.append([float(parts[0]), float(parts[1])])
                if len(pts) < 3:
                    st.error('Need at least 3 vertices.')
                else:
                    if pts[0] != pts[-1]:
                        pts.append(pts[0])
                    st.session_state.polygon = {
                        'type': 'Feature',
                        'geometry': {'type': 'Polygon', 'coordinates': [pts]},
                    }
                    st.session_state.route = None
                    st.rerun()
            except ValueError as exc:
                st.error(str(exc))
    st.divider()

    altitude  = st.number_input('Altitude AGL (m)',      value=80.0,  min_value=1.0,    step=5.0)
    fov       = st.number_input('LiDAR FOV (°)',         value=60.0,  min_value=1.0,    max_value=179.0, step=5.0)
    overlap   = st.number_input('Overlap (%)',           value=20.0,  min_value=0.0,    max_value=99.0,  step=5.0)
    adaptive_spacing = st.checkbox(
        'Terrain-adaptive spacing', value=True,
        help='Tighten pass spacing wherever terrain between two passes rises and '
             'narrows the effective swath, instead of one constant spacing everywhere. '
             'Reduces HELIOS++ refinement iterations and keeps point density uniform.',
    )
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

if not dtm_path:
    st.title('LiDAR Drone Route Planner')
    st.info('Upload a DTM/DEM file (or enter a local path) in the sidebar to get started.')
    st.stop()

if not os.path.exists(dtm_path):
    st.title('LiDAR Drone Route Planner')
    st.error(f'File not found: `{dtm_path}`')
    st.stop()

dtm = load_dtm(dtm_path)
bounds = dtm.src.bounds
is_geo = dtm.src.crs.is_geographic if dtm.src.crs else True
elev_min, elev_max = float(np.nanmin(dtm.array)), float(np.nanmax(dtm.array))

swath_m, spacing_m, spacing_map = swath_and_spacing(altitude, fov, overlap, is_geo)
step_map = to_map_units(step_m, is_geo)
# Terrain-sampling resolution for the per-pass max-elevation (clearance) check,
# decoupled from the waypoint step: the finer of the waypoint step and the DTM's
# native pixel size, so a peak between waypoints can't be missed — without
# adding waypoints/legs to the route or to the HELIOS++ simulation.
dtm_res_map = min(abs(dtm.src.res[0]), abs(dtm.src.res[1]))
elev_step_map = min(step_map, dtm_res_map)

with st.sidebar:
    st.caption(
        f'Swath width: **{swath_m:.1f} m**  \n'
        f'Pass spacing: **{spacing_m:.1f} m** ({overlap:.0f}% overlap)'
        + (' — baseline, tightened over ridges' if adaptive_spacing else '')
    )

# ------------------------------------------------------------------ compute route

if compute_btn and st.session_state.polygon is not None:
    poly = shapely_shape(st.session_state.polygon['geometry'])
    if not poly.is_valid:
        poly = poly.buffer(0)
    with st.spinner('Computing route…'):
        if adaptive_spacing:
            st.session_state.route = plan_route_adaptive(
                dtm, poly, altitude, error_tol,
                scan_half_angle_deg=fov / 2.0, step=step_map,
                overlap_frac=overlap / 100.0, is_geo=is_geo,
                elev_sample_step=elev_step_map,
            )
        else:
            st.session_state.route = plan_route(
                dtm, poly, altitude, error_tol, spacing_map, step_map,
                elev_sample_step=elev_step_map,
            )

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

# HELIOS++ under-density zones overlay
if st.session_state.helios_result and not st.session_state.helios_result.get('passed'):
    _fail_cells = st.session_state.helios_result.get('failing_cells_geo', [])
    if _fail_cells:
        if len(_fail_cells) <= 500:
            for _lon, _lat in _fail_cells:
                folium.Circle(
                    location=[_lat, _lon],
                    radius=0.8,
                    color='#ff2222',
                    fill=True,
                    fill_color='#ff2222',
                    fill_opacity=0.55,
                    tooltip='Under-density zone',
                ).add_to(m)
        else:
            # Too many individual markers would freeze the browser — render
            # a heatmap layer instead, which scales to any point count.
            HeatMap(
                [[_lat, _lon, 1] for _lon, _lat in _fail_cells],
                name='Under-density zones',
                radius=8,
                blur=6,
                min_opacity=0.4,
            ).add_to(m)

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
    if st.session_state.polygon:
        poly_shape = shapely_shape(st.session_state.polygon['geometry'])
        area_m2 = polygon_area_m2(poly_shape, is_geo)
        st.divider()
        st.markdown('**Polygon**')
        if area_m2 >= 1_000_000:
            st.metric('Area', f'{area_m2 / 1_000_000:.3f} km²')
        else:
            st.metric('Area', f'{area_m2:,.0f} m²')

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
            if is_geo:
                lat_m = 111139.0
                lon_m = 111139.0 * math.cos(math.radians(sum(ys) / len(ys)))
                total_m = sum(
                    math.sqrt(((xs[i+1]-xs[i]) * lon_m)**2 + ((ys[i+1]-ys[i]) * lat_m)**2)
                    for i in range(len(xs)-1)
                )
            else:
                total_m = sum(
                    math.sqrt((xs[i+1]-xs[i])**2 + (ys[i+1]-ys[i])**2)
                    for i in range(len(xs)-1)
                )
            st.divider()
            st.markdown('**Route**')
            st.metric('Waypoints', len(wps))
            st.metric('Path length', f'{total_m/1000:.2f} km' if total_m >= 1000 else f'{total_m:.0f} m')
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

# ------------------------------------------------------------------ HELIOS++ validation

st.divider()
with st.expander('HELIOS++ LiDAR Validation', expanded=bool(st.session_state.helios_result) or st.session_state.helios_expander_open):

    if not _HELIOS_AVAILABLE:
        st.warning(
            'helios_integration module not found. '
            'Make sure `src/helios_integration.py` is present and `laspy` is installed.'
        )
    else:
        # ── HELIOS++ binary (auto-managed) ────────────────────────────────────
        _bin = st.session_state.helios_bin
        _bin_ok = bool(_bin and os.path.exists(_bin))

        if _bin_ok:
            _status_c, _redetect_c = st.columns([5, 1])
            _status_c.success(f"HELIOS++ ready: `{_bin}`")
            if _redetect_c.button('Re-detect', use_container_width=True, key='_hredetect'):
                st.session_state.helios_bin = ''
                st.rerun()
        else:
            st.warning("HELIOS++ is not installed or could not be found on this machine.")
            _inst_c, _manual_c = st.columns(2)

            if _inst_c.button('Install HELIOS++ automatically', type='primary',
                              use_container_width=True, key='_hinstall'):
                st.session_state.helios_installing = True
                st.rerun()

            with _manual_c.expander('Enter path manually'):
                _manual_path = st.text_input('Binary path', key='_hbin_manual',
                                             placeholder='C:\\helios\\helios++.exe')
                if st.button('Use this path', key='_hbin_use') and _manual_path:
                    st.session_state.helios_bin = _manual_path
                    st.rerun()

        # Installation progress block (active on the rerun after clicking Install)
        if st.session_state.helios_installing:
            with st.status('Installing HELIOS++…', expanded=True) as _install_status:
                try:
                    _new_bin = download_and_install(log=st.write)
                    st.session_state.helios_bin = str(_new_bin)
                    st.session_state.helios_installing = False
                    _install_status.update(label='HELIOS++ installed!', state='complete')
                    st.rerun()
                except Exception as _install_err:
                    st.session_state.helios_installing = False
                    _install_status.update(label=f'Installation failed: {_install_err}', state='error')

        # Terrain OBJ + XML references (always shown)
        helios_bin_input = st.session_state.helios_bin
        path_c1, path_c2 = st.columns(2)
        scene_obj_input = path_c1.text_input(
            'Terrain OBJ',
            value=st.session_state.helios_scene_obj,
            placeholder='/path/to/terrain.obj',
        )
        st.session_state.helios_scene_obj = scene_obj_input

        scanner_ref_input  = path_c1.text_input('Scanner XML ref',  value=DEFAULT_SCANNER_REF,  key='_hscan')
        platform_ref_input = path_c2.text_input('Platform XML ref', value=DEFAULT_PLATFORM_REF, key='_hplat')

        # ── Flight & density ──────────────────────────────────────────────────
        st.markdown('**Flight & density**')
        fd_c1, fd_c2, fd_c3 = st.columns(3)
        h_min_pts  = fd_c1.number_input('Min points / m²',     value=DEFAULT_MIN_POINTS_PER_SQM, min_value=1,   step=5)
        h_speed    = fd_c2.number_input('Drone speed (m/s)',    value=DEFAULT_DRONE_SPEED_MS,     min_value=0.1, step=0.5)
        h_max_iter = fd_c3.number_input('Max refinement cycles',value=DEFAULT_MAX_ITERATIONS,     min_value=1,   max_value=10)

        # ── Scanner parameters ────────────────────────────────────────────────
        st.markdown('**Scanner**')
        sc_c1, sc_c2, sc_c3 = st.columns(3)
        h_pulse_freq  = sc_c1.number_input('Pulse frequency (Hz)',   value=DEFAULT_PULSE_FREQ_HZ,   min_value=1000,  step=10000)
        h_scan_freq   = sc_c2.number_input('Scan frequency (Hz)',    value=DEFAULT_SCAN_FREQ_HZ,    min_value=1.0,   step=10.0)
        h_scan_angle  = sc_c3.number_input('Scan half-angle (°)',    value=DEFAULT_SCAN_ANGLE_DEG,  min_value=1.0,   max_value=89.0, step=5.0)

        # ── Terrain mesh ──────────────────────────────────────────────────────
        st.markdown('**Terrain mesh**')
        mesh_c1, mesh_c2 = st.columns([2, 1])
        h_mesh_step = mesh_c1.number_input('Mesh vertex spacing (m)', value=DEFAULT_DTM_MESH_STEP_M, min_value=0.5, step=0.5)

        if dtm_path and mesh_c2.button('Convert DTM → OBJ', use_container_width=True):
            with st.spinner('Generating terrain mesh…'):
                try:
                    import tempfile as _tf
                    _obj_tmp = _tf.NamedTemporaryFile(suffix='.obj', delete=False)
                    _obj_tmp.close()
                    if st.session_state.route:
                        _valid = [wp for wp in st.session_state.route
                                  if not math.isnan(wp.get('z', float('nan')))]
                        _ref_lon = sum(wp['x'] for wp in _valid) / max(len(_valid), 1)
                        _ref_lat = sum(wp['y'] for wp in _valid) / max(len(_valid), 1)
                    else:
                        _ref_lon = (bounds.left + bounds.right) / 2
                        _ref_lat = (bounds.bottom + bounds.top) / 2
                    dtm_to_obj(dtm_path, _obj_tmp.name, step_m=float(h_mesh_step),
                               ref_lon=_ref_lon, ref_lat=_ref_lat)
                    st.session_state.helios_scene_obj = _obj_tmp.name
                    # Remember the exact origin baked into this mesh so the
                    # simulation reuses it verbatim — recomputing a fresh
                    # centroid from (possibly changed) route data would shift
                    # the mesh away from the flight legs and yield 0 points.
                    st.session_state.helios_scene_ref_lon = _ref_lon
                    st.session_state.helios_scene_ref_lat = _ref_lat
                    st.session_state.helios_expander_open = True
                    st.success(f'OBJ written: `{_obj_tmp.name}`')
                    st.rerun()
                except Exception as _e:
                    st.error(f'Conversion failed: {_e}')

        # ── Run ───────────────────────────────────────────────────────────────
        st.markdown('**Run simulation**')
        _can_run = bool(st.session_state.route and helios_bin_input and scene_obj_input)
        if not _can_run:
            st.caption('Compute a route, set the HELIOS++ binary, and provide a terrain OBJ to enable.')
        elif (scene_obj_input != st.session_state.helios_scene_obj
              or st.session_state.helios_scene_ref_lon is None):
            st.caption(
                "⚠️ This OBJ wasn't generated by the “Convert DTM → OBJ” button above, so its "
                "coordinate origin is unknown to the simulator — it may not align with the route's "
                "flight legs and could yield an empty point cloud. Use the converter for a guaranteed match."
            )

        if not st.session_state.helios_running:
            if st.button('Run HELIOS++ Validation', type='primary', use_container_width=True, disabled=not _can_run):
                st.session_state.helios_result = None
                st.session_state.helios_log = []
                _q: 'queue.Queue' = queue.Queue()
                _stop_event = threading.Event()
                st.session_state.helios_queue = _q
                st.session_state.helios_stop_event = _stop_event

                _work = os.path.join(tempfile.gettempdir(), 'helios_autoroute')

                _region = (
                    list(shapely_shape(st.session_state.polygon['geometry']).exterior.coords)
                    if st.session_state.polygon else None
                )

                def _worker(
                    _route=st.session_state.route, _bin=helios_bin_input, _obj=scene_obj_input,
                    _work=_work, _is_geo=is_geo, _alt=altitude,
                    _ref_lon=st.session_state.helios_scene_ref_lon,
                    _ref_lat=st.session_state.helios_scene_ref_lat,
                    _min_pts=int(h_min_pts), _speed=float(h_speed), _max_iter=int(h_max_iter),
                    _pulse=int(h_pulse_freq), _scan_freq=float(h_scan_freq), _scan_angle=float(h_scan_angle),
                    _scanner_ref=scanner_ref_input, _platform_ref=platform_ref_input,
                    _dtm=dtm, _region=_region,
                    _q=_q, _stop_event=_stop_event,
                ):
                    try:
                        _r = run_feedback_loop(
                            route=_route,
                            helios_bin=_bin,
                            scene_obj_path=_obj,
                            work_dir=_work,
                            is_geo=_is_geo,
                            ref_lon=_ref_lon,
                            ref_lat=_ref_lat,
                            altitude_m=_alt,
                            min_points=_min_pts,
                            speed_ms=_speed,
                            max_iterations=_max_iter,
                            pulse_freq_hz=_pulse,
                            scan_freq_hz=_scan_freq,
                            scan_angle_deg=_scan_angle,
                            scanner_ref=_scanner_ref,
                            platform_ref=_platform_ref,
                            dtm=_dtm,
                            region_polygon=_region,
                            log=lambda msg: _q.put(('log', msg)),
                            stop_event=_stop_event,
                        )
                    except Exception as _exc:
                        _q.put(('done', {'passed': False, 'error': str(_exc)}))
                    else:
                        _q.put(('done', _r))

                st.session_state.helios_running = True
                threading.Thread(target=_worker, daemon=True).start()
                st.rerun()
        else:
            # Auto-refreshing fragment: only THIS widget reruns every second,
            # not the whole page (a full-page st.rerun() loop would re-execute
            # route planning / map rendering each tick and feel "stuck").
            @st.fragment(run_every=1.0)
            def _helios_progress_fragment():
                if not st.session_state.helios_running:
                    return

                if st.button('Stop simulation', use_container_width=True):
                    if st.session_state.helios_stop_event is not None:
                        st.session_state.helios_stop_event.set()
                        st.session_state.helios_log.append('Stop requested — waiting for HELIOS++ to terminate…')

                _q = st.session_state.helios_queue
                _done_result = None
                while True:
                    try:
                        _kind, _payload = _q.get_nowait()
                    except queue.Empty:
                        break
                    if _kind == 'log':
                        st.session_state.helios_log.append(_payload)
                    elif _kind == 'done':
                        _done_result = _payload

                # Plain bordered container instead of st.status: st.status is an
                # expander that re-mounts (with its open/close animation) every
                # fragment tick, which looked like it was "opening and closing
                # itself". A static container has no such animation.
                if _done_result is not None and _done_result.get('error'):
                    _label = f"❌ Simulation failed: {_done_result['error']}"
                elif _done_result is not None:
                    _label = '✅ Validation passed!' if _done_result.get('passed') else '⚠️ Validation finished (density not reached)'
                else:
                    _label = '🔄 Running LiDAR simulation… (updates ~every 1s — click "Stop simulation" to cancel)'

                with st.container(border=True):
                    st.markdown(f'**{_label}**')
                    with st.container(height=240):
                        st.code('\n'.join(st.session_state.helios_log[-300:]) or '(waiting for output…)', language=None)

                if _done_result is not None:
                    st.session_state.helios_result = _done_result
                    st.session_state.helios_running = False
                    st.session_state.helios_stop_event = None
                    st.session_state.helios_queue = None
                    st.rerun()

            _helios_progress_fragment()

        # ── Results ───────────────────────────────────────────────────────────
        _res = st.session_state.helios_result
        if _res:
            st.divider()
            if _res.get('error'):
                st.error(f"Simulation error: {_res['error']}")
            else:
                _passed = _res['passed']
                _iters  = _res['iterations']
                _n_fail = len(_res.get('failing_cells_geo', []))

                if _passed:
                    st.success(
                        f'Density validated after {_iters} iteration(s). '
                        f'All cells meet the ≥{int(h_min_pts)} pts/m² threshold.'
                    )
                else:
                    st.warning(
                        f'Density validation failed after {_iters} iteration(s). '
                        f'{_n_fail} under-density cell(s) remain '
                        f'(shown as red circles on the map).'
                    )

                res_c1, res_c2, res_c3 = st.columns(3)
                res_c1.metric('Iterations', _iters)
                res_c2.metric('Failing cells', _n_fail)
                _extra_wps = len(_res.get('final_route', [])) - len(st.session_state.route or [])
                res_c3.metric('Supplemental waypoints added', max(0, _extra_wps))

                if _extra_wps > 0:
                    if st.button('Apply densified route', use_container_width=True):
                        st.session_state.route = _res['final_route']
                        st.session_state.helios_result = None
                        st.rerun()

                _dl1, _dl2 = st.columns(2)
                if _res.get('trajectory_path') and os.path.exists(_res['trajectory_path']):
                    with open(_res['trajectory_path'], 'rb') as _f:
                        _dl1.download_button(
                            'Download trajectory .txt', _f.read(),
                            file_name='trajectory.txt', mime='text/plain',
                            use_container_width=True,
                        )
                if _res.get('survey_xml_path') and os.path.exists(_res['survey_xml_path']):
                    with open(_res['survey_xml_path'], 'rb') as _f:
                        _dl2.download_button(
                            'Download survey .xml', _f.read(),
                            file_name='survey.xml', mime='application/xml',
                            use_container_width=True,
                        )