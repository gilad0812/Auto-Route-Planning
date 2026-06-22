"""
Default values for the HELIOS++ UI widgets in app.py.

These constants are ONLY used as the `value=` argument for Streamlit inputs.
Every pipeline function receives the actual user-supplied values at call time —
nothing here is read during a simulation run.
"""

# ── Point density ──────────────────────────────────────────────────────────────
DEFAULT_MIN_POINTS_PER_SQM: int = 100  # UI default for "Min points / m²"
CELL_SIZE_M: float = 1.0               # grid cell edge — not exposed in UI

# ── Drone platform ─────────────────────────────────────────────────────────────
DEFAULT_DRONE_SPEED_MS: float = 6.0    # UI default for "Drone speed (m/s)"

# ── UAV LiDAR scanner ─────────────────────────────────────────────────────────
DEFAULT_PULSE_FREQ_HZ: int = 600_000   # UI default for "Pulse frequency (Hz)"
# Discrete pulse rates the default scanner (riegl_vux_120_23) supports — HELIOS
# only accepts these, so the UI is restricted to them to keep the density
# estimate in lockstep with the simulation (an arbitrary value would diverge).
DEFAULT_PULSE_FREQS_HZ: tuple = (150_000, 300_000, 600_000, 1_200_000, 1_800_000, 2_400_000)
DEFAULT_SCAN_FREQ_HZ: float = 224.4    # UI default for "Scan frequency (Hz)"
DEFAULT_SCAN_ANGLE_DEG: float = 50.0   # UI default for "Scan half-angle (°)"

# ── Terrain mesh ───────────────────────────────────────────────────────────────
DEFAULT_DTM_MESH_STEP_M: float = 3.0   # UI default for "Mesh vertex spacing (m)"

# ── Feedback loop ──────────────────────────────────────────────────────────────
DEFAULT_MAX_ITERATIONS: int = 1        # UI default for "Max refinement cycles"

# ── HELIOS++ built-in references (relative to the helios++ working directory) ──
# Matched to the Conda-installed v2.2.2 layout.
# riegl_vux_120_23  → purpose-built UAV LiDAR scanner (scanners_als.xml).
# copter_linearpath → quadcopter on a linear-path trajectory (platforms.xml).
DEFAULT_SCANNER_REF: str = "data/scanners_als.xml#riegl_vux_120_23"
DEFAULT_PLATFORM_REF: str = "data/platforms.xml#copter_linearpath"
