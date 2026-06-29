"""Default parameter values and HELIOS++ asset references for the desktop UI."""

DEFAULT_MIN_POINTS_PER_SQM: int = 100
CELL_SIZE_M: float = 1.0
DEFAULT_DRONE_SPEED_MS: float = 6.0

DEFAULT_PULSE_FREQ_HZ: int = 600_000
# discrete rates the riegl_vux_120_23 supports — HELIOS only accepts these, so the
# UI is restricted to them to keep the estimate in lockstep with the simulation.
DEFAULT_PULSE_FREQS_HZ: tuple = (150_000, 300_000, 600_000, 1_200_000, 1_800_000, 2_400_000)
DEFAULT_SCAN_FREQ_HZ: float = 224.4
DEFAULT_SCAN_ANGLE_DEG: float = 50.0

DEFAULT_DTM_MESH_STEP_M: float = 3.0
DEFAULT_MAX_ITERATIONS: int = 1

# HELIOS++ built-in refs (Conda v2.2.2 layout): UAV scanner + linear-path quadcopter.
DEFAULT_SCANNER_REF: str = "data/scanners_als.xml#riegl_vux_120_23"
DEFAULT_PLATFORM_REF: str = "data/platforms.xml#copter_linearpath"
