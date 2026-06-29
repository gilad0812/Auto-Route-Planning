# PyInstaller spec for the LiDAR Route Planner desktop app.
#   pyinstaller desktop.spec      → dist/LidarRoutePlanner/
#
# The geo stack (rasterio/GDAL, pyproj/PROJ, shapely/GEOS) ships native data
# files that must be bundled; collect_all grabs them. src/ is on pathex so the
# `from dtm import ...` style imports in ui/ resolve at analysis time.
import os
from PyInstaller.utils.hooks import collect_all, collect_submodules

# App/exe icon. Drop a .ico at assets/app.ico to brand the executable; if it's
# absent the build still works (PyInstaller treats icon=None as the default icon).
ICON = os.path.join('assets', 'app.ico')
ICON = ICON if os.path.exists(ICON) else None

datas, binaries, hiddenimports = [], [], []
for pkg in ('rasterio', 'pyproj', 'shapely', 'laspy', 'lazrs', 'matplotlib'):
    try:
        d, b, h = collect_all(pkg)
        datas += d; binaries += b; hiddenimports += h
    except Exception:
        pass

hiddenimports += collect_submodules('rasterio')
# src/ modules imported dynamically (sys.path.insert) — name them explicitly.
hiddenimports += [
    'dtm', 'route_planner', 'density_estimate', 'helios_integration',
    'terrain_converter', 'helios_setup', 'helios_config', 'patch_scanner',
]

a = Analysis(
    ['desktop.py'],
    pathex=['.', 'src'],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    excludes=['streamlit', 'folium', 'plotly', 'pyvista', 'pyvistaqt',
              'PySide6.QtWebEngineWidgets', 'PySide6.QtWebEngineCore', 'tkinter'],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz, a.scripts, [],
    exclude_binaries=True,
    name='LidarRoutePlanner',
    console=False,
    icon=ICON,
)
coll = COLLECT(exe, a.binaries, a.datas, name='LidarRoutePlanner')
