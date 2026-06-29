# Auto-Route-Planning

A drone LiDAR survey route planner that generates lawnmower-pattern flight paths
from Digital Terrain Model (DTM) data, holding a constant altitude above ground
level (AGL) across complex terrain, and predicts/validates per-m² point density.

It is a **native Qt desktop application** designed to run offline on a standalone
machine — no web server, no browser, no internet.

## Features

- Native offline map canvas: DTM shaded-relief with pan/zoom; set the survey AOI by drawing on the terrain or entering coordinates
- Terrain-adaptive lawnmower path generation (constant AGL per pass, contour-aligned)
- Analytical point-density estimate with optional CHM vegetation thinning (the CHM is validated against the DTM before use)
- Scan frequency derived automatically from the pulse rate
- HELIOS++ LiDAR simulation to validate density (runs off the UI thread)
- Under-density overlay (estimate in orange, HELIOS in red)
- Export waypoints as GeoJSON or CSV; save HELIOS trajectory / survey XML

## Installation

```bash
python -m venv .venv
.venv\Scripts\activate          # Windows
# source .venv/bin/activate     # Linux/macOS
pip install -r requirements-desktop.txt
```

## Usage

```bash
python desktop.py
```

> For a full step-by-step walkthrough (parameters, reading results, HELIOS
> validation, exports, troubleshooting) see **[USER_GUIDE.md](USER_GUIDE.md)**.

1. **Open DTM** (toolbar / File menu) — optionally **Open CHM** for vegetation (checked for CRS/extent compatibility with the DTM first). **Clear** removes either.
2. Set the AOI: **Draw AOI** on the map (click vertices → **Finish** / double-click), or **Enter coordinates…** to type the vertices.
3. Set flight/scanner parameters — scan frequency is derived from the pulse rate and shown locked.
4. **Compute Route** — the route and under-density estimate render on the map; stats appear in the Results panel.
5. **Validate (HELIOS++)…** — runs the simulation against the bundled (or pre-installed, auto-detected) HELIOS++ binary.
6. **Export** the route as GeoJSON / CSV.

HELIOS++ is bundled into the packaged app (see Packaging). For development,
`src/helios_setup.py` can download and install it on an internet-connected machine.

## Packaging (standalone .exe)

`build.ps1` builds the app and bakes HELIOS++ into a single self-contained folder:

```powershell
.\.venv\Scripts\Activate.ps1
.\build.ps1
```

It runs PyInstaller (onedir) into `C:\route_planner_build` (off OneDrive, to dodge
file locks), then copies the HELIOS++ install (default `C:\helios_bin`) into the
bundle as `helios\`. The result —

```
C:\route_planner_build\dist\LidarRoutePlanner\
```

— is fully self-contained (its own Python, all libraries, and HELIOS): copy that
one folder to the target machine (same OS + architecture) and run
`LidarRoutePlanner.exe`. **No Python or dependencies are needed on the target.**

Flags: `-SkipHelios` (exe only), `-HeliosSource <dir>`, `-BuildRoot <dir>`. Drop a
`assets/app.ico` to brand the executable.

### Developing on an offline machine

To edit + rebuild without internet, bring Python, the source, and an offline wheel
cache built on a connected machine of matching OS/arch/Python:

```powershell
pip download -r requirements-desktop.txt -d wheelhouse          # connected machine
pip install --no-index --find-links wheelhouse -r requirements-desktop.txt  # target
```

Then `python desktop.py` runs your edits instantly (no rebuild needed); `.\build.ps1`
produces a fresh self-contained bundle.

## Project Structure

```
Auto-Route-Planning/
├── desktop.py            # Entry point (PySide6)
├── build.ps1             # Build .exe + bake in HELIOS (self-contained bundle)
├── requirements-desktop.txt
├── desktop.spec          # PyInstaller build spec
├── assets/               # App icon (app.ico) + generator
├── data/                 # DTM/CHM + HELIOS scanner/platform XML
├── ui/
│   ├── main_window.py    # Window: params sidebar · map · results
│   ├── canvasmap.py      # Native offline DTM map (QGraphicsView)
│   ├── planning.py       # Qt-free glue to the src/ model
│   └── helios.py         # HELIOS++ validation worker + dialog
└── src/
    ├── dtm.py            # DTM raster reader (bilinear interpolation)
    ├── route_planner.py  # Lawnmower path generation + AGL computation
    ├── density_estimate.py
    ├── helios_integration.py / helios_setup.py / helios_config.py
    ├── terrain_converter.py  # DTM → OBJ mesh for HELIOS
    ├── viewer.py         # Standalone matplotlib viewer (optional)
    └── main.py           # CLI route generator (optional)
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `PySide6` | Qt desktop UI |
| `rasterio` | GeoTIFF raster I/O |
| `shapely` | Polygon geometry |
| `numpy` / `pandas` | Numerics |
| `matplotlib` | Relief colormaps / optional viewer |
| `Pillow` | Image handling |
| `laspy` | HELIOS++ LAS point-cloud readback |
