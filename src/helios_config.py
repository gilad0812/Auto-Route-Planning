"""
Default values for the HELIOS++ UI widgets in app.py.

These constants are ONLY used as the `value=` argument for Streamlit inputs.
Every pipeline function receives the actual user-supplied values at call time —
nothing here is read during a simulation run.
"""

# ── Point density ──────────────────────────────────────────────────────────────
DEFAULT_MIN_POINTS_PER_SQM: int = 50   # UI default for "Min points / m²"
CELL_SIZE_M: float = 1.0               # grid cell edge — not exposed in UI

# ── Drone platform ─────────────────────────────────────────────────────────────
DEFAULT_DRONE_SPEED_MS: float = 5.0    # UI default for "Drone speed (m/s)"

# ── UAV LiDAR scanner ─────────────────────────────────────────────────────────
DEFAULT_PULSE_FREQ_HZ: int = 300_000   # UI default for "Pulse frequency (Hz)"
DEFAULT_SCAN_FREQ_HZ: float = 100.0    # UI default for "Scan frequency (Hz)"
DEFAULT_SCAN_ANGLE_DEG: float = 30.0   # UI default for "Scan half-angle (°)"

# ── Terrain mesh ───────────────────────────────────────────────────────────────
DEFAULT_DTM_MESH_STEP_M: float = 2.0   # UI default for "Mesh vertex spacing (m)"

# ── Feedback loop ──────────────────────────────────────────────────────────────
DEFAULT_MAX_ITERATIONS: int = 3        # UI default for "Max refinement cycles"

# ── HELIOS++ built-in references (relative to the helios++ working directory) ──
# Matched to the Conda-installed v2.2.2 layout.
# riegl_vux-1uav  → purpose-built UAV LiDAR scanner.
# copter_linearpath → quadcopter on a linear-path trajectory.
DEFAULT_SCANNER_REF: str = "data/scanners_als.xml#riegl_vux-1uav"
DEFAULT_PLATFORM_REF: str = "data/platforms.xml#copter_linearpath"
