# Auto-Route-Planning

A drone LiDAR survey route planner that generates optimal lawnmower-pattern flight paths from Digital Terrain Model (DTM) data, maintaining a constant altitude above ground level (AGL) across complex terrain.

## Features

- Draw a survey polygon on an interactive map
- Automatic lawnmower path generation with configurable pass spacing and waypoint density
- Per-waypoint elevation lookup via bilinear interpolation of DTM raster data
- Swath width calculation from LiDAR FOV and altitude
- Color-coded route visualization by elevation
- Export waypoints as GeoJSON or CSV for drone autopilots
- Two interfaces: Streamlit web app (primary) and matplotlib desktop viewer

## Installation

```bash
python -m venv .venv
.venv\Scripts\activate        # Windows
# source .venv/bin/activate   # Linux/macOS
pip install -r requirements.txt
```

Requires a GeoTIFF DTM file at `data/dtm.tif`.

### Docker (recommended for portability)

Builds a self-contained image with the app, its Python dependencies, and a Linux HELIOS++ install baked in — no manual setup needed on the target machine.

```bash
docker compose up --build
```

Opens at `http://localhost:8501`. Place your DTM at `data/dtm.tif` on the host — it's mounted into the container at `/app/data`.

To build/run without compose:

```bash
docker build -t auto-route-planning .
docker run -p 8501:8501 -v "$(pwd)/data:/app/data" auto-route-planning
```

On Windows PowerShell, replace `$(pwd)` with `${PWD}`:

```powershell
docker run -p 8501:8501 -v "${PWD}/data:/app/data" auto-route-planning
```

The HELIOS++ download during `docker build` requires network access. The resulting image is portable — copy it (`docker save`/`docker load`) or rebuild from this repo on any Docker-capable machine.

## Usage

### Web app (recommended)

```bash
python main.py
# or: streamlit run app.py
```

Opens at `http://localhost:8501`. Draw a polygon on the map, adjust flight parameters in the sidebar, and download the generated route.

### Desktop viewer (matplotlib)

```python
from src.viewer import InteractiveDTMViewer
InteractiveDTMViewer('data/dtm.tif').show()
```

Click to place polygon vertices, press **Enter** to close, **S** to save, **R** to reset.

### Command line

```bash
python -m src.main \
  --dtm data/dtm.tif \
  --polygon examples/polygon.geojson \
  --distance 30 \
  --error 2.0 \
  --spacing 20 \
  --step 5 \
  --out route.csv
```

| Argument | Description |
|----------|-------------|
| `--dtm` | Path to GeoTIFF elevation raster |
| `--polygon` | GeoJSON file defining the survey area |
| `--distance` | Target AGL altitude in meters |
| `--error` | Allowed AGL deviation in meters |
| `--spacing` | Distance between lawnmower passes (meters) |
| `--step` | Waypoint interval along each pass (meters) |
| `--out` | Output CSV path |

## Output Format

**CSV** — one waypoint per row:

| x | y | z | target_distance | error_tolerance |
|---|---|---|-----------------|-----------------|
| easting | northing | absolute altitude (m) | AGL target | allowed deviation |

**GeoJSON** — 3D `MultiLineString` with one feature per pass.

## How It Works

1. The survey polygon is sliced into parallel horizontal passes separated by the configured spacing.
2. Passes alternate direction (snake/boustrophedon) to minimize repositioning.
3. Each pass is sampled at the configured step interval to produce candidate waypoints.
4. The DTM is queried at each waypoint position using bilinear interpolation.
5. Absolute altitude is computed as `z = terrain_elevation + AGL_distance`.

## Project Structure

```
Auto-Route-Planning/
├── app.py                # Streamlit web interface
├── main.py               # Launcher (runs app.py via streamlit)
├── requirements.txt
├── data/
│   └── dtm.tif           # Input Digital Terrain Model (GeoTIFF)
├── examples/
│   └── polygon.geojson   # Example survey polygon
└── src/
    ├── dtm.py            # DTM raster reader with bilinear interpolation
    ├── route_planner.py  # Lawnmower path generation and AGL computation
    ├── viewer.py         # Interactive matplotlib desktop viewer
    └── main.py           # CLI argument parser
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `rasterio` | GeoTIFF raster I/O |
| `shapely` | Polygon geometry and clipping |
| `numpy` | Numerical operations |
| `pandas` | Waypoint data handling |
| `streamlit` | Web UI framework |
| `folium` / `streamlit-folium` | Interactive map |
| `matplotlib` | Desktop viewer |
| `Pillow` | Elevation visualization |
