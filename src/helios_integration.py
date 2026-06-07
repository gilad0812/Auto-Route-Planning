"""
HELIOS++ LiDAR simulation feedback loop.

Three-stage pipeline
────────────────────────────────────────────────────────────────────────────
Stage 1 │ export_trajectory   – route → trajectory .txt (timestamp x y z hdg r p)
Stage 2 │ run_helios           – survey .xml → HELIOS++ subprocess → .las output
Stage 3 │ verify_point_density – .las → 1×1 m density grid → pass/fail + cells
────────────────────────────────────────────────────────────────────────────
Orchestrator: run_feedback_loop chains all three stages and re-runs with
supplemental lawnmower passes injected over under-density zones until either
all cells pass or MAX_ITERATIONS is reached.
"""

from __future__ import annotations

import math
import os
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from xml.dom import minidom
from xml.etree import ElementTree as ET

import numpy as np

try:
    import laspy
    _LASPY_OK = True
except ImportError:
    _LASPY_OK = False

from helios_config import (
    DEFAULT_MIN_POINTS_PER_SQM,
    DEFAULT_DRONE_SPEED_MS,
    CELL_SIZE_M,
    DEFAULT_PULSE_FREQ_HZ,
    DEFAULT_SCAN_FREQ_HZ,
    DEFAULT_SCAN_ANGLE_DEG,
    DEFAULT_MAX_ITERATIONS,
    DEFAULT_SCANNER_REF,
    DEFAULT_PLATFORM_REF,
)

# Convenience aliases so internal call-sites stay readable
MIN_POINTS_PER_SQM = DEFAULT_MIN_POINTS_PER_SQM
DRONE_SPEED_MS     = DEFAULT_DRONE_SPEED_MS
PULSE_FREQ_HZ      = DEFAULT_PULSE_FREQ_HZ
SCAN_FREQ_HZ       = DEFAULT_SCAN_FREQ_HZ
SCAN_ANGLE_DEG     = DEFAULT_SCAN_ANGLE_DEG
MAX_ITERATIONS     = DEFAULT_MAX_ITERATIONS

_LAT_M = 111_139.0  # metres per degree latitude (WGS-84 approximation)


# ─────────────────────────────────────────────────────────────────────────────
# Internal coordinate helpers
# ─────────────────────────────────────────────────────────────────────────────

def _route_to_metric(
    route: List[Dict],
    is_geo: bool,
) -> Tuple[List[Dict], float, float]:
    """
    Project geographic (lon/lat) route waypoints to flat-Earth metric coordinates.

    Returns (metric_route, ref_lon, ref_lat).  When is_geo=False the route is
    returned unchanged and the reference values are 0.
    """
    if not is_geo:
        return route, 0.0, 0.0

    lons = [wp["x"] for wp in route]
    lats = [wp["y"] for wp in route]
    ref_lon = sum(lons) / len(lons)
    ref_lat = sum(lats) / len(lats)
    lon_m = _LAT_M * math.cos(math.radians(ref_lat))

    metric: List[Dict] = []
    for wp in route:
        metric.append({
            **wp,
            "x": (wp["x"] - ref_lon) * lon_m,
            "y": (wp["y"] - ref_lat) * _LAT_M,
            # z is already in metres (terrain elevation + AGL)
        })
    return metric, ref_lon, ref_lat


def _cells_to_geo(
    cells: List[Tuple[float, float]],
    ref_lon: float,
    ref_lat: float,
    is_geo: bool,
) -> List[Tuple[float, float]]:
    """Convert metric cell centres back to geographic (lon, lat) degrees."""
    if not is_geo:
        return cells
    lon_m = _LAT_M * math.cos(math.radians(ref_lat))
    return [
        (ref_lon + x / lon_m, ref_lat + y / _LAT_M)
        for x, y in cells
    ]


def _posix(path: str) -> str:
    """Normalise a path to forward slashes (HELIOS++ XML compatibility)."""
    return Path(path).as_posix()


# ─────────────────────────────────────────────────────────────────────────────
# Task 1 – Trajectory Exporter
# ─────────────────────────────────────────────────────────────────────────────

def export_trajectory(
    route: List[Dict],
    output_path: str,
    speed_ms: float = DRONE_SPEED_MS,
    is_geo: bool = True,
) -> str:
    """
    Export route waypoints as a space-separated HELIOS++ trajectory file.

    Columns: timestamp  x  y  z  heading  roll  pitch

    When is_geo=True the geographic coordinates are automatically converted to
    flat-Earth metric so that HELIOS++ receives numerically stable values.
    Heading is computed from the direction vector to the next waypoint.
    Roll and pitch are fixed to 0 (stable horizontal UAV flight).

    Args:
        route:       Waypoint dicts from plan_route() — keys: x, y, z.
        output_path: Destination .txt file path.
        speed_ms:    Drone cruising speed (m/s) used to compute timestamps.
        is_geo:      True when x=longitude and y=latitude in degrees.

    Returns:
        Absolute path of the written trajectory file.
    """
    if not route:
        raise ValueError("Route is empty — nothing to export.")

    metric_route, _, _ = _route_to_metric(route, is_geo)

    lines: List[str] = ["timestamp x y z heading roll pitch"]
    timestamp = 0.0
    heading = 0.0

    for i, wp in enumerate(metric_route):
        x = float(wp["x"])
        y = float(wp["y"])
        z = float(wp["z"]) if not math.isnan(float(wp["z"])) else 0.0

        # Heading: clockwise from North (+Y axis), in degrees
        if i < len(metric_route) - 1:
            dx = float(metric_route[i + 1]["x"]) - x
            dy = float(metric_route[i + 1]["y"]) - y
            heading = math.degrees(math.atan2(dx, dy)) % 360.0

        lines.append(
            f"{timestamp:.6f} {x:.4f} {y:.4f} {z:.4f} {heading:.4f} 0.0000 0.0000"
        )

        # Advance timestamp by Euclidean distance / speed
        if i < len(metric_route) - 1:
            nx = float(metric_route[i + 1]["x"])
            ny = float(metric_route[i + 1]["y"])
            nz_raw = metric_route[i + 1]["z"]
            nz = float(nz_raw) if not math.isnan(float(nz_raw)) else z
            dist = math.sqrt((nx - x) ** 2 + (ny - y) ** 2 + (nz - z) ** 2)
            timestamp += dist / max(speed_ms, 1e-6)

    output_path = os.path.abspath(output_path)
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")

    return output_path


# ─────────────────────────────────────────────────────────────────────────────
# Survey XML builders (internal helpers)
# ─────────────────────────────────────────────────────────────────────────────

def _pretty_xml(root: ET.Element) -> str:
    raw = ET.tostring(root, encoding="unicode")
    dom = minidom.parseString(raw)
    lines = dom.toprettyxml(indent="    ").splitlines()
    if lines[0].startswith("<?xml"):
        lines = lines[1:]
    return "\n".join(lines)


def _write_scene_xml(obj_path: str, xml_path: str, scene_id: str = "terrain") -> str:
    """Write a HELIOS++ scene XML that references a Wavefront OBJ mesh."""
    doc = ET.Element("document")
    scene = ET.SubElement(doc, "scene", id=scene_id, name="Survey Terrain")
    part = ET.SubElement(scene, "part")
    filt = ET.SubElement(part, "filter", type="objloader")
    ET.SubElement(filt, "param", type="string", key="filepath",
                  value=_posix(os.path.abspath(obj_path)))

    xml_path = os.path.abspath(xml_path)
    Path(xml_path).parent.mkdir(parents=True, exist_ok=True)
    with open(xml_path, "w", encoding="utf-8") as fh:
        fh.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        fh.write(_pretty_xml(doc))
    return xml_path


def build_survey_xml(
    metric_route: List[Dict],
    scene_xml_path: str,
    output_xml_path: str,
    scene_id: str = "terrain",
    speed_ms: float = DRONE_SPEED_MS,
    pulse_freq_hz: int = PULSE_FREQ_HZ,
    scan_freq_hz: float = SCAN_FREQ_HZ,
    scan_angle_deg: float = SCAN_ANGLE_DEG,
    scanner_ref: str = DEFAULT_SCANNER_REF,
    platform_ref: str = DEFAULT_PLATFORM_REF,
    survey_name: str = "auto_route_survey",
) -> str:
    """
    Generate a HELIOS++ survey XML from metric route waypoints.

    One leg per waypoint is emitted. HELIOS++ linearly interpolates the platform
    trajectory between consecutive leg positions, firing the scanner along each
    segment.

    Args:
        metric_route:     Waypoints in Cartesian metres (output of _route_to_metric).
        scene_xml_path:   Path to the terrain scene XML.
        output_xml_path:  Destination survey .xml path.
        scene_id:         Fragment id for the scene element in scene_xml_path.
        speed_ms:         Platform speed (m/s).
        pulse_freq_hz:    LiDAR pulse repetition frequency.
        scan_freq_hz:     Scanner rotation frequency.
        scan_angle_deg:   Half-FOV from nadir.
        scanner_ref:      HELIOS++ scanner reference string (path#id).
        platform_ref:     HELIOS++ platform reference string (path#id).
        survey_name:      Identifier string for this survey.

    Returns:
        Absolute path of the written survey XML.
    """
    scene_ref = f"{_posix(os.path.abspath(scene_xml_path))}#{scene_id}"

    doc = ET.Element("document")

    survey = ET.SubElement(
        doc, "survey",
        name=survey_name,
        scene=scene_ref,
        platform=platform_ref,
        scanner=scanner_ref,
    )

    # Write scanner settings inline on every leg — avoids template lookup issues
    _scan_attrs = dict(
        active="true",
        pulseFreq_hz=str(pulse_freq_hz),   # HELIOS++ expects lowercase 'hz'
        scanFreq_hz=str(scan_freq_hz),
        scanAngle_deg=str(scan_angle_deg),
        headRotatePerSec_deg="0",
    )

    for wp in metric_route:
        x = float(wp["x"])
        y = float(wp["y"])
        z_raw = wp["z"]
        z = float(z_raw) if not math.isnan(float(z_raw)) else 0.0

        leg = ET.SubElement(survey, "leg")
        ET.SubElement(
            leg, "platformSettings",
            x=f"{x:.4f}", y=f"{y:.4f}", z=f"{z:.4f}",
            onGround="false",
            movePerSec_m=str(speed_ms),
        )
        ET.SubElement(leg, "scannerSettings", **_scan_attrs)

    output_xml_path = os.path.abspath(output_xml_path)
    Path(output_xml_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_xml_path, "w", encoding="utf-8") as fh:
        fh.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        fh.write(_pretty_xml(doc))

    return output_xml_path


# ─────────────────────────────────────────────────────────────────────────────
# Task 2 – HELIOS++ Subprocess Runner
# ─────────────────────────────────────────────────────────────────────────────

def run_helios(
    helios_bin: str,
    survey_xml: str,
    output_dir: Optional[str] = None,
    extra_args: Optional[List[str]] = None,
) -> Path:
    """
    Execute the HELIOS++ binary and wait for completion.

    Args:
        helios_bin:  Absolute path to the helios++ (or helios++.exe) binary.
        survey_xml:  Path to the master survey XML file.
        output_dir:  Directory where HELIOS++ should write LAS output.
                     When None, HELIOS++ uses its own default output/ folder.
        extra_args:  Additional CLI flags forwarded verbatim.

    Returns:
        Path to the directory that contains the output .las / .laz files.

    Raises:
        FileNotFoundError: helios_bin or survey_xml does not exist.
        RuntimeError:      HELIOS++ exits with a non-zero status code.
    """
    helios_bin = Path(helios_bin)
    survey_xml = Path(survey_xml)

    if not helios_bin.exists():
        raise FileNotFoundError(f"HELIOS++ binary not found: {helios_bin}")
    if not survey_xml.exists():
        raise FileNotFoundError(f"Survey XML not found: {survey_xml}")

    cmd: List[str] = [str(helios_bin), str(survey_xml), "--lasOutput", "--silent", "-j", "0"]
    if output_dir:
        cmd += ["--output", str(output_dir)]
    if extra_args:
        cmd.extend(extra_args)

    # Run from the dir that contains data/ so relative XML refs resolve correctly
    try:
        from helios_setup import find_helios_root as _find_root
        helios_cwd = _find_root(helios_bin)
    except ImportError:
        helios_cwd = helios_bin.parent

    # Build an env that includes the Conda DLL directories so the binary can
    # load its runtime libraries (STATUS_DLL_NOT_FOUND = 0xC0000135 otherwise).
    import platform as _platform
    run_env = os.environ.copy()
    if _platform.system() == "Windows":
        # Walk up from the binary to find the Conda env root (has Library\bin\)
        _candidate = helios_bin.parent
        _conda_root = None
        for _ in range(8):
            if (_candidate / "Library" / "bin").is_dir():
                _conda_root = _candidate
                break
            _candidate = _candidate.parent
        if _conda_root:
            _dll_dirs = [
                str(_conda_root),
                str(_conda_root / "Library" / "bin"),
                str(_conda_root / "Library" / "mingw-w64" / "bin"),
                str(_conda_root / "Scripts"),
                str(_conda_root / "DLLs"),
            ]
            run_env["PATH"] = os.pathsep.join(_dll_dirs) + os.pathsep + run_env.get("PATH", "")

    try:
        subprocess.run(
            cmd,
            check=True,
            capture_output=True,
            text=True,
            cwd=str(helios_cwd),
            env=run_env,
        )
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            f"HELIOS++ exited with code {exc.returncode}.\n"
            f"Command: {' '.join(cmd)}\n"
            f"Stderr:\n{exc.stderr or '(none)'}"
        ) from exc

    # Locate the output directory
    search_root = Path(output_dir) if output_dir else helios_bin.parent / "output"
    if not search_root.exists():
        raise RuntimeError(
            f"HELIOS++ completed but output directory not found: {search_root}"
        )

    # Return the folder of the most recently written LAS/LAZ file
    las_files = sorted(search_root.rglob("*.las"), key=lambda p: p.stat().st_mtime)
    las_files += sorted(search_root.rglob("*.laz"), key=lambda p: p.stat().st_mtime)
    if not las_files:
        raise RuntimeError(f"HELIOS++ produced no LAS/LAZ files under: {search_root}")

    return max(las_files, key=lambda p: p.stat().st_mtime).parent


# ─────────────────────────────────────────────────────────────────────────────
# Task 3 – Point Density Verifier
# ─────────────────────────────────────────────────────────────────────────────

def verify_point_density(
    las_path: str,
    min_points: int = MIN_POINTS_PER_SQM,
    cell_size: float = CELL_SIZE_M,
) -> Tuple[bool, List[Tuple[float, float]]]:
    """
    Verify that every 1 × 1 m grid cell in a LAS point cloud meets the density
    threshold.

    Algorithm:
        1. Load x, y from the .las file via laspy.
        2. Bin all points into a 2-D grid of cell_size × cell_size metre cells.
        3. Flag every populated cell (≥1 point) that has fewer than min_points.

    Args:
        las_path:   Path to the .las or .laz file produced by HELIOS++.
        min_points: Required points per cell (default 50 pts/m²).
        cell_size:  Grid cell edge length in metres (default 1.0 m).

    Returns:
        (passed, failing_cells) where:
          - passed:        True if every populated cell meets the threshold.
          - failing_cells: List of (x_centre, y_centre) in the same coordinate
                           system as the LAS file.  Empty when passed=True.

    Raises:
        ImportError:       laspy is not installed.
        FileNotFoundError: LAS file does not exist.
    """
    if not _LASPY_OK:
        raise ImportError(
            "laspy is required for point density verification.\n"
            "Install with:  pip install laspy[lazrs]"
        )

    las_path = Path(las_path)
    if not las_path.exists():
        raise FileNotFoundError(f"LAS file not found: {las_path}")

    with laspy.open(str(las_path)) as reader:
        las = reader.read()

    xs = np.asarray(las.x, dtype=np.float64)
    ys = np.asarray(las.y, dtype=np.float64)

    if xs.size == 0:
        # Empty cloud — complete coverage failure; no specific cells to report
        return False, []

    # Build 2-D point count grid
    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()

    nx = max(1, int(math.ceil((x_max - x_min) / cell_size)))
    ny = max(1, int(math.ceil((y_max - y_min) / cell_size)))

    xi = np.clip(((xs - x_min) / cell_size).astype(np.int32), 0, nx - 1)
    yi = np.clip(((ys - y_min) / cell_size).astype(np.int32), 0, ny - 1)

    grid = np.zeros((ny, nx), dtype=np.int32)
    np.add.at(grid, (yi, xi), 1)

    # Cells with at least one point but below threshold
    fail_mask = (grid > 0) & (grid < min_points)
    rows, cols = np.where(fail_mask)

    failing_cells: List[Tuple[float, float]] = [
        (x_min + (c + 0.5) * cell_size, y_min + (r + 0.5) * cell_size)
        for r, c in zip(rows.tolist(), cols.tolist())
    ]

    return len(failing_cells) == 0, failing_cells


# ─────────────────────────────────────────────────────────────────────────────
# Route densification helper
# ─────────────────────────────────────────────────────────────────────────────

def _supplemental_passes(
    failing_cells_metric: List[Tuple[float, float]],
    terrain_z_m: float,
    altitude_m: float,
    spacing_m: float = 3.0,
    step_m: float = 3.0,
) -> List[Dict]:
    """
    Generate a tight boustrophedon grid over the bounding box of under-density cells.

    Returns waypoints in the same metric coordinate system as the input cells.
    The caller is responsible for converting back to the original CRS if needed.
    """
    if not failing_cells_metric:
        return []

    xs, ys = zip(*failing_cells_metric)
    x_min = min(xs) - 2.0
    x_max = max(xs) + 2.0
    y_min = min(ys) - 2.0
    y_max = max(ys) + 2.0
    target_z = terrain_z_m + altitude_m

    waypoints: List[Dict] = []
    y = y_min
    flip = False

    while y <= y_max:
        x_start = x_min if not flip else x_max
        x_end   = x_max if not flip else x_min
        dist = abs(x_end - x_start)
        n_steps = max(1, int(math.ceil(dist / step_m)))
        sign = 1 if x_end > x_start else -1

        for i in range(n_steps + 1):
            x = x_start + sign * i * step_m
            waypoints.append({
                "x": x, "y": y, "z": target_z,
                "target_distance": altitude_m, "error_tol": 2.0,
            })

        y += spacing_m
        flip = not flip

    return waypoints


# ─────────────────────────────────────────────────────────────────────────────
# Feedback Loop Orchestrator
# ─────────────────────────────────────────────────────────────────────────────

def run_feedback_loop(
    route: List[Dict],
    helios_bin: str,
    scene_obj_path: str,
    work_dir: str,
    is_geo: bool = True,
    altitude_m: float = 80.0,
    min_points: int = MIN_POINTS_PER_SQM,
    speed_ms: float = DRONE_SPEED_MS,
    max_iterations: int = MAX_ITERATIONS,
    pulse_freq_hz: int = PULSE_FREQ_HZ,
    scan_freq_hz: float = SCAN_FREQ_HZ,
    scan_angle_deg: float = SCAN_ANGLE_DEG,
    scanner_ref: str = DEFAULT_SCANNER_REF,
    platform_ref: str = DEFAULT_PLATFORM_REF,
) -> Dict:
    """
    Full HELIOS++ validation pipeline with automatic density refinement.

    Runs: export_trajectory → build_survey_xml → run_helios → verify_point_density.
    On failure, supplemental lawnmower passes are injected over low-density zones
    and the simulation repeats (up to max_iterations).  The pathfinding algorithm
    is never touched — all adjustments are additive passes appended to the route.

    Args:
        route:          Waypoints from plan_route() in the DTM's CRS.
        helios_bin:     Path to the HELIOS++ executable.
        scene_obj_path: Path to the terrain Wavefront OBJ mesh.
        work_dir:       Working directory for all intermediate files.
        is_geo:         True when route x=longitude, y=latitude (degrees).
        altitude_m:     AGL altitude used for supplemental passes.
        min_points:     Minimum points/m² threshold.
        speed_ms:       Drone speed (m/s).
        max_iterations: Maximum simulation-refine cycles.
        pulse_freq_hz:  LiDAR pulse repetition frequency (Hz).
        scan_freq_hz:   Scanner rotation frequency (Hz).
        scan_angle_deg: Half-FOV from nadir (degrees).
        scanner_ref:    HELIOS++ scanner XML reference (path#id).
        platform_ref:   HELIOS++ platform XML reference (path#id).

    Returns:
        {
            "passed":            bool,
            "iterations":        int,
            "trajectory_path":   str,
            "survey_xml_path":   str,
            "las_path":          str | None,
            "failing_cells_geo": List[Tuple[float, float]],  # original CRS
            "final_route":       List[Dict],    # original + any supplemental wps
            "error":             str | None,
        }
    """
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)

    result: Dict = {
        "passed": False,
        "iterations": 0,
        "trajectory_path": None,
        "survey_xml_path": None,
        "las_path": None,
        "failing_cells_geo": [],
        "final_route": list(route),
        "error": None,
    }

    current_route = list(route)

    try:
        scene_xml = work_dir / "scene.xml"
        _write_scene_xml(scene_obj_path, str(scene_xml))

        for iteration in range(max_iterations):
            iter_dir = work_dir / f"iter_{iteration}"
            iter_dir.mkdir(exist_ok=True)

            # Stage 1 — export trajectory (geographic → metric internally)
            traj_path = iter_dir / "trajectory.txt"
            export_trajectory(current_route, str(traj_path), speed_ms, is_geo)
            result["trajectory_path"] = str(traj_path)

            # Convert route to metric coordinates for the survey XML
            metric_route, ref_lon, ref_lat = _route_to_metric(current_route, is_geo)

            # Stage 2 — build survey XML and run HELIOS++
            survey_xml = iter_dir / "survey.xml"
            build_survey_xml(
                metric_route=metric_route,
                scene_xml_path=str(scene_xml),
                output_xml_path=str(survey_xml),
                speed_ms=speed_ms,
                pulse_freq_hz=pulse_freq_hz,
                scan_freq_hz=scan_freq_hz,
                scan_angle_deg=scan_angle_deg,
                scanner_ref=scanner_ref,
                platform_ref=platform_ref,
            )
            result["survey_xml_path"] = str(survey_xml)

            las_output_dir = iter_dir / "output"
            las_output_dir.mkdir(exist_ok=True)
            las_dir = run_helios(helios_bin, str(survey_xml), str(las_output_dir))

            las_files = list(las_dir.glob("*.las")) + list(las_dir.glob("*.laz"))
            if not las_files:
                raise RuntimeError(f"No LAS/LAZ files found under {las_dir}")
            las_path = max(las_files, key=lambda p: p.stat().st_mtime)
            result["las_path"] = str(las_path)

            # Stage 3 — verify density
            passed, failing_metric = verify_point_density(str(las_path), min_points)
            result["iterations"] = iteration + 1
            result["passed"] = passed
            result["failing_cells_geo"] = _cells_to_geo(
                failing_metric, ref_lon, ref_lat, is_geo
            )

            if passed:
                break

            # Refinement — inject supplemental passes over failing zone
            if iteration < max_iterations - 1 and failing_metric:
                valid_zs = [float(wp["z"]) for wp in metric_route
                            if not math.isnan(float(wp["z"]))]
                avg_z = float(np.mean(valid_zs)) if valid_zs else altitude_m
                terrain_z = avg_z - altitude_m

                extra_metric = _supplemental_passes(
                    failing_cells_metric=failing_metric,
                    terrain_z_m=terrain_z,
                    altitude_m=altitude_m,
                    spacing_m=3.0,
                    step_m=3.0,
                )

                if is_geo and extra_metric:
                    lon_m = _LAT_M * math.cos(math.radians(ref_lat))
                    extra_geo = [
                        {
                            **wp,
                            "x": ref_lon + wp["x"] / lon_m,
                            "y": ref_lat + wp["y"] / _LAT_M,
                        }
                        for wp in extra_metric
                    ]
                    current_route = current_route + extra_geo
                else:
                    current_route = current_route + extra_metric

        result["final_route"] = current_route

    except Exception as exc:
        result["error"] = str(exc)

    return result
