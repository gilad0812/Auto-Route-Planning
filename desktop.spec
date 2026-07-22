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

# Onedir (default) vs onefile. Set RP_ONEFILE=1 (build.ps1 -OneFile) to get a single
# movable LidarRoutePlanner.exe. Onefile self-extracts to a temp dir on every launch,
# so it starts slower and canNOT hold HELIOS (find_helios_binary looks for a helios/
# folder next to the exe, not inside _MEIPASS) — ship helios/ beside the exe if needed.
ONEFILE = bool(os.environ.get('RP_ONEFILE'))
if ONEFILE:
    exe = EXE(
        pyz, a.scripts, a.binaries, a.datas, [],
        name='LidarRoutePlanner',
        console=False,
        icon=ICON,
    )
else:
    exe = EXE(
        pyz, a.scripts, [],
        exclude_binaries=True,
        name='LidarRoutePlanner',
        console=False,
        icon=ICON,
    )
    coll = COLLECT(exe, a.binaries, a.datas, name='LidarRoutePlanner')
