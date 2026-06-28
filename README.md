# Auto-Route-Planning

A drone LiDAR survey route planner that generates lawnmower-pattern flight paths
from Digital Terrain Model (DTM) data, holding a constant altitude above ground
level (AGL) across complex terrain, and predicts/validates per-m² point density.

It is a **native Qt desktop application** designed to run offline on a standalone
machine — no web server, no browser, no internet.

## Features

- Native offline map canvas: DTM shaded-relief with pan/zoom; draw the survey AOI directly on the terrain
- Terrain-adaptive lawnmower path generation (constant AGL per pass, contour-aligned)
- Analytical point-density estimate with optional CHM vegetation thinning
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

1. **Open DTM** (toolbar / File menu) — optionally **Open CHM** for vegetation.
2. **Draw AOI** on the map → click vertices → **Finish** (or double-click).
3. **Compute Route** — the route and under-density estimate render on the map; stats appear in the Results panel.
4. **Validate (HELIOS++)…** — point at a pre-installed HELIOS++ binary (auto-detected if present) to run the simulation.
5. **Export** the route as GeoJSON / CSV.

HELIOS++ must already be installed on the machine (offline). `setup_helios.py`
can install it on a connected machine if needed.

## Packaging (standalone .exe)

```bash
pip install -r requirements-desktop.txt
pyinstaller desktop.spec
```

The build lands in `dist/`. See `desktop.spec` for the bundled data files
(scanner/platform XML under `data/`).

## Project Structure

```
Auto-Route-Planning/
├── desktop.py            # Entry point (PySide6)
├── requirements-desktop.txt
├── desktop.spec          # PyInstaller build spec
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
