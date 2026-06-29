# User Guide — LiDAR Drone Route Planner

A step-by-step walkthrough for planning a survey, reading the results, validating
with HELIOS++, and exporting waypoints. The app runs fully offline.

---

## 1. Launch

- **Packaged build:** double-click `LidarRoutePlanner.exe` in the bundle folder.
- **From source:** `python desktop.py`

The window has three parts: the **parameters sidebar** (left), the **map** (centre),
and the **Results** panel (right).

---

## 2. Load terrain

1. **Open DTM…** (sidebar *Data* box or the File menu) and pick a GeoTIFF DTM.
   The terrain renders as shaded relief on the map. Pan by dragging, zoom with the
   wheel, and use **⤢ Fit** on the map toolbar to recentre.
2. *(Optional)* **Open CHM…** to load a canopy-height/vegetation raster. It is
   checked for CRS and extent compatibility with the DTM first; an incompatible
   CHM is rejected with a reason and not applied. Toggle its overlay with the
   **CHM** button on the map toolbar.
3. **Clear** (next to each) removes the DTM or CHM. Clearing the DTM also clears
   the CHM, the AOI, and any results.

Hovering the map shows the cursor's `lat, lon` and terrain elevation in the
toolbar.

---

## 3. Set the Area of Interest (AOI)

Two ways — both produce the same survey polygon:

- **Draw AOI** (map toolbar): click each vertex on the terrain, then **Finish**
  (or double-click the last point). **Clear AOI** discards it.
- **Enter coordinates…** (sidebar *AOI* box): type one vertex per line as
  `lat, lon` (matching the map readout). At least 3 vertices; the polygon closes
  automatically.

> **Tip:** draw the AOI slightly larger than the region you actually need. Passes
> stop at the AOI boundary, so over-drawing guarantees full density right up to
> your real edge.

---

## 4. Set parameters

**Flight**

| Parameter | Meaning |
|-----------|---------|
| **Altitude AGL** | Height above the *highest* terrain along each pass. Higher → wider swath, fewer passes, lower density. Held constant per pass (required for clean strip registration). |
| **Overlap** | Sidelap between neighbouring passes' swaths. More overlap → safer coverage and higher density, but more passes. |
| **Terrain-adaptive spacing** | On: passes follow the terrain contours and tighten over ridges to hold coverage. Off: plain fixed-spacing east–west lawnmower. |
| **Along-track step** | Distance between waypoints along a pass (output resolution only — terrain is always sampled finely for clearance safety). |

**Scanner & density**

| Parameter | Meaning |
|-----------|---------|
| **Min points / m²** | Target ground density. Cells below this are flagged under-dense. |
| **Drone speed** | Faster flight → fewer points per area (thinner coverage). |
| **Pulse freq** | Laser pulse rate; higher → denser. Limited to the scanner's supported rates. |
| **Scan freq** | Derived automatically from the pulse rate and shown locked (not editable). |
| **Canopy ground-return frac** | With a CHM loaded, the fraction of pulses that reach the ground through vegetation (default 0.4). |
| **FOV** | Fixed at 100° (±50°) for the RIEGL VUX-120-23. |

---

## 5. Compute the route

Click **Compute Route**. A busy indicator shows in the status bar; when it
finishes the map and Results panel update.

**On the map:**
- The flight path is drawn as a line **coloured by altitude** (per-pass), with a
  **green** start marker and a **red** end marker.
- **Orange** dots mark cells the analytical estimate predicts will be under the
  target density.

**In the Results panel:**
- **Polygon** area.
- **Route** — waypoint count, total path length, altitude range.
- **Density estimate** — coverage %, median and minimum predicted density.

Changing the AOI, DTM, or CHM clears stale results so what you see always matches
the current inputs.

> Under-dense / void cells are often **occlusion shadows** — gully floors or lee
> slopes hidden from the flight direction — not a planning mistake. The analytical
> estimate is fast and meant for iterating; confirm with HELIOS++ before trusting
> a marginal result.

---

## 6. Validate with HELIOS++ (optional)

For a physically simulated check rather than the analytical estimate:

1. Click **Validate (HELIOS++)…**.
2. The binary path is auto-detected (bundled with the app, or a pre-installed
   copy). Use **Browse…** if needed.
3. Set **Mesh vertex spacing** (coarser = faster build, less memory).
4. **Run Validation** — progress streams in the log. **Stop** cancels.
5. When done, cells below target are painted **red** on the map, and the dialog
   shows coverage / median / void stats.
6. Optionally **Save trajectory…** or **Save survey XML…**.

The simulation runs off the UI thread, so the app stays responsive.

---

## 7. Export

From the sidebar *Export* box (enabled once a route exists):

- **GeoJSON** — one point feature per waypoint, with altitude, target AGL, and
  pass id.
- **CSV** — `index, x, y, z, target_agl_m, pass_id`.

Both contain the flight waypoints in the DTM's coordinate reference system.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| **Compute Route** is disabled | No AOI set yet — draw one or enter coordinates. |
| "Open a DTM first" | Load a DTM before a CHM or an AOI. |
| CHM rejected | CRS mismatch or it covers a different area than the DTM. |
| Lots of orange/red cells in valleys | Occlusion shadows — irreducible from one flight direction; not a parameter error. |
| HELIOS won't run | Confirm the binary path; in a packaged build it should auto-fill to the bundled `helios\` copy. |
