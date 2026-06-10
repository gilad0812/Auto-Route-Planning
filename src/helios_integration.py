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
import queue
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple, Union
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
    ref_lon: Optional[float] = None,
    ref_lat: Optional[float] = None,
) -> Tuple[List[Dict], float, float]:
    """
    Project geographic (lon/lat) route waypoints to flat-Earth metric coordinates.

    Returns (metric_route, ref_lon, ref_lat).  When is_geo=False the route is
    returned unchanged and the reference values are 0.

    ref_lon / ref_lat: Optional fixed projection origin. MUST be supplied (and
    match the origin used to build the terrain OBJ via dtm_to_obj) so the
    survey legs and the terrain mesh share the same local coordinate frame —
    otherwise the platform flies over a patch of empty space and HELIOS++
    records zero points. When omitted, the route's own centroid is used.
    """
    if not is_geo:
        return route, 0.0, 0.0

    if ref_lon is None or ref_lat is None:
        lons = [wp["x"] for wp in route]
        lats = [wp["y"] for wp in route]
        ref_lon = sum(lons) / len(lons) if ref_lon is None else ref_lon
        ref_lat = sum(lats) / len(lats) if ref_lat is None else ref_lat
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

class SimulationCancelled(Exception):
    """Raised when a caller requests early termination via a stop event."""


def run_helios(
    helios_bin: str,
    survey_xml: str,
    output_dir: Optional[str] = None,
    extra_args: Optional[List[str]] = None,
    log: Optional[Callable[[str], None]] = None,
    stop_event: Optional["threading.Event"] = None,
) -> Path:
    """
    Execute the HELIOS++ binary and wait for completion, streaming progress.

    Args:
        helios_bin:  Absolute path to the helios++ (or helios++.exe) binary.
        survey_xml:  Path to the master survey XML file.
        output_dir:  Directory where HELIOS++ should write LAS output.
                     When None, HELIOS++ uses its own default output/ folder.
        extra_args:  Additional CLI flags forwarded verbatim.
        log:         Optional callback(message) invoked for each progress line
                     HELIOS++ prints (e.g. "Leg3/40 12.34% ..."), so the caller
                     can show that the simulation is alive and not hanging.
        stop_event:  Optional threading.Event; when set while the simulation is
                     running, the HELIOS++ process is terminated and
                     SimulationCancelled is raised.

    Returns:
        Path to the directory that contains the output .las / .laz files.

    Raises:
        FileNotFoundError:   helios_bin or survey_xml does not exist.
        RuntimeError:        HELIOS++ exits with a non-zero status code.
        SimulationCancelled: stop_event was set before completion.
    """
    helios_bin = Path(helios_bin)
    survey_xml = Path(survey_xml)

    if not helios_bin.exists():
        raise FileNotFoundError(f"HELIOS++ binary not found: {helios_bin}")
    if not survey_xml.exists():
        raise FileNotFoundError(f"Survey XML not found: {survey_xml}")

    # No --silent: HELIOS++ then prints per-leg progress ("LegX/Y NN.NN% ...")
    # which we stream back through `log` so the caller can show live status.
    cmd: List[str] = [str(helios_bin), str(survey_xml), "--lasOutput", "-j", "0"]
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

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        cwd=str(helios_cwd),
        env=run_env,
    )

    # HELIOS++ fully-buffers stdout when it's piped (not a terminal), so its
    # per-leg progress lines can sit in the child's libc buffer for a long time
    # before they reach us — the process can be busy for minutes while we see
    # nothing. Read its output on a background thread and emit periodic
    # heartbeats whenever nothing has arrived for a while, so the caller can
    # tell the simulation is alive rather than hung.
    line_queue: "queue.Queue[Optional[str]]" = queue.Queue()

    def _pump() -> None:
        assert proc.stdout is not None
        for raw_line in proc.stdout:
            line_queue.put(raw_line.rstrip("\r\n"))
        line_queue.put(None)

    pump_thread = threading.Thread(target=_pump, daemon=True)
    pump_thread.start()

    tail: List[str] = []
    start_time = time.monotonic()
    last_activity = start_time
    HEARTBEAT_SEC = 20.0
    eof = False
    cancelled = False
    while not eof:
        if stop_event is not None and stop_event.is_set():
            cancelled = True
            if log is not None:
                log("Stop requested — terminating HELIOS++ process…")
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
            break

        try:
            item = line_queue.get(timeout=2.0)
        except queue.Empty:
            now = time.monotonic()
            if log is not None and now - last_activity >= HEARTBEAT_SEC:
                elapsed = now - start_time
                log(
                    f"… still running ({elapsed:.0f}s elapsed, no console output yet — "
                    "HELIOS++ buffers its progress messages, so silence is normal for "
                    "large surveys; it has not hung)"
                )
                last_activity = now
            continue

        if item is None:
            eof = True
            continue

        line = item
        last_activity = time.monotonic()
        if not line:
            continue
        tail.append(line)
        if len(tail) > 40:
            tail.pop(0)
        if log is not None:
            log(line)

    if cancelled:
        proc.wait()
        raise SimulationCancelled("Simulation cancelled by user")

    pump_thread.join()
    returncode = proc.wait()
    if returncode != 0:
        raise RuntimeError(
            f"HELIOS++ exited with code {returncode}.\n"
            f"Command: {' '.join(cmd)}\n"
            f"Output (last lines):\n" + "\n".join(tail[-20:] or ["(none)"])
        )

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
    las_path: Union[str, List[str]],
    min_points: int = MIN_POINTS_PER_SQM,
    cell_size: float = CELL_SIZE_M,
) -> Tuple[bool, List[Tuple[float, float]]]:
    """
    Verify that every cell in the survey area meets the point density threshold.

    Accepts a single LAS/LAZ file, a directory of per-leg files (the default
    HELIOS++ output layout), or a list of any mix of the above (used by the
    incremental refinement loop to merge density across all iterations'
    output directories).  All *.las and *.laz files found are merged into a
    single density grid using a two-pass approach that keeps peak RAM
    proportional to the grid size, not the total point count.

    Algorithm:
        Pass 1 – Read LAS headers to determine the global bounding box.
        Pass 2 – Bin points from each file into the shared density grid, freeing
                 each file from memory immediately after processing.
        Flag every populated cell (≥1 point) with fewer than min_points.

    Args:
        las_path:   Path (or list of paths) to .las/.laz file(s) and/or
                    directories of leg files.
        min_points: Required points per cell (default 50 pts/m²).
        cell_size:  Grid cell edge length in metres (default 1.0 m).

    Returns:
        (passed, failing_cells) where:
          - passed:        True if every populated cell meets the threshold.
          - failing_cells: List of (x_centre, y_centre) in the LAS coordinate
                           system.  Empty when passed=True or the cloud is empty.

    Raises:
        ImportError:       laspy is not installed.
        FileNotFoundError: las_path does not exist.
    """
    if not _LASPY_OK:
        raise ImportError(
            "laspy is required for point density verification.\n"
            "Install with:  pip install laspy[lazrs]"
        )

    paths = [las_path] if isinstance(las_path, (str, Path)) else list(las_path)

    all_files: List[Path] = []
    for raw in paths:
        p = Path(raw)
        if not p.exists():
            raise FileNotFoundError(f"LAS path not found: {p}")
        if p.is_dir():
            all_files.extend(sorted(p.glob("*.las")) + sorted(p.glob("*.laz")))
        else:
            all_files.append(p)

    if not all_files:
        return False, []

    # Pass 1 — global extents from LAS headers (fast, no full data read)
    x_min, x_max = float("inf"), float("-inf")
    y_min, y_max = float("inf"), float("-inf")
    any_points = False
    for lf in all_files:
        with laspy.open(str(lf)) as reader:
            h = reader.header
            if h.point_count > 0:
                any_points = True
                x_min = min(x_min, float(h.x_min))
                x_max = max(x_max, float(h.x_max))
                y_min = min(y_min, float(h.y_min))
                y_max = max(y_max, float(h.y_max))

    if not any_points:
        return False, []

    nx = max(1, int(math.ceil((x_max - x_min) / cell_size)))
    ny = max(1, int(math.ceil((y_max - y_min) / cell_size)))
    grid = np.zeros((ny, nx), dtype=np.int32)

    # Pass 2 — accumulate point counts, one file at a time to limit RAM usage
    for lf in all_files:
        with laspy.open(str(lf)) as reader:
            las = reader.read()
        xs = np.asarray(las.x, dtype=np.float64)
        ys = np.asarray(las.y, dtype=np.float64)
        del las  # free full point record immediately
        if xs.size > 0:
            xi = np.clip(((xs - x_min) / cell_size).astype(np.int32), 0, nx - 1)
            yi = np.clip(((ys - y_min) / cell_size).astype(np.int32), 0, ny - 1)
            np.add.at(grid, (yi, xi), 1)
        del xs, ys

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

_MIN_SUPPLEMENTAL_WPS = 20      # floor for tiny surveys, so a handful of legs still gets some refinement
_SUPPLEMENTAL_WPS_RATIO = 1.0   # per-iteration cap = max(_MIN_SUPPLEMENTAL_WPS, ratio * len(original route))
_CLUSTER_MERGE_RADIUS_M = 10.0  # failing cells within this distance are grouped into one local pass


def _cluster_cells(
    cells: List[Tuple[float, float]],
    cell_size: float,
    merge_radius_m: float,
) -> List[List[Tuple[float, float]]]:
    """
    Group failing cells into spatial clusters via grid-based connected components.

    Cells within merge_radius_m (Chebyshev distance) of each other end up in the
    same cluster, so isolated patches of under-density cells each get their own
    small local supplemental pass instead of one pass spanning the whole survey.
    """
    if not cells:
        return []

    r = max(1, int(math.ceil(merge_radius_m / cell_size)))
    grid_to_cell = {
        (round(x / cell_size), round(y / cell_size)): (x, y)
        for x, y in cells
    }
    remaining = set(grid_to_cell.keys())
    clusters: List[List[Tuple[float, float]]] = []

    while remaining:
        seed = next(iter(remaining))
        remaining.discard(seed)
        stack = [seed]
        component = [seed]
        while stack:
            gx, gy = stack.pop()
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    neighbor = (gx + dx, gy + dy)
                    if neighbor in remaining:
                        remaining.discard(neighbor)
                        stack.append(neighbor)
                        component.append(neighbor)
        clusters.append([grid_to_cell[g] for g in component])

    return clusters


def _boustrophedon(
    x_min: float, x_max: float, y_min: float, y_max: float,
    target_z: float, altitude_m: float, spacing_m: float, step_m: float,
) -> List[Dict]:
    """Generate a lawnmower grid of waypoints over [x_min,x_max] x [y_min,y_max]."""
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


def _supplemental_passes(
    failing_cells_metric: List[Tuple[float, float]],
    terrain_z_m: float,
    altitude_m: float,
    max_wps: int,
    spacing_m: float = 3.0,
    step_m: float = 3.0,
    cell_size: float = CELL_SIZE_M,
) -> List[Dict]:
    """
    Generate a boustrophedon grid over each spatial cluster of under-density cells.

    Returns waypoints in the same metric coordinate system as the input cells.
    The caller is responsible for converting back to the original CRS if needed.

    Failing cells are first grouped into clusters (see _cluster_cells) so that
    isolated gaps each get a small, tightly-spaced local pass at full quality
    instead of one giant pass over the bounding box of the whole survey.
    Spacing is only scaled up — uniformly across all clusters — if the combined
    waypoint count would exceed max_wps.

    max_wps: Per-iteration waypoint budget. The caller scales this with the size
             of the original survey so a tiny survey doesn't get swamped by a
             huge supplemental pass, and a large survey isn't over-restricted.
    """
    if not failing_cells_metric:
        return []

    target_z = terrain_z_m + altitude_m

    # Each cluster contributes a minimum of ~2 waypoints (one short pass line)
    # no matter how much spacing is scaled up, so too many tiny clusters can
    # blow the budget on their own. Grow the merge radius until the cluster
    # count itself fits, before considering spacing scale-down.
    merge_radius = _CLUSTER_MERGE_RADIUS_M
    clusters = _cluster_cells(failing_cells_metric, cell_size, merge_radius)
    while len(clusters) * 2 > max_wps and merge_radius < 500.0:
        merge_radius *= 2.0
        clusters = _cluster_cells(failing_cells_metric, cell_size, merge_radius)

    bboxes: List[Tuple[float, float, float, float]] = []
    total_estimated = 0
    for cluster in clusters:
        xs, ys = zip(*cluster)
        x_min, x_max = min(xs) - 2.0, max(xs) + 2.0
        y_min, y_max = min(ys) - 2.0, max(ys) + 2.0
        bboxes.append((x_min, x_max, y_min, y_max))
        span_x = max(x_max - x_min, step_m)
        span_y = max(y_max - y_min, spacing_m)
        total_estimated += (math.ceil(span_x / step_m) + 1) * (math.ceil(span_y / spacing_m) + 1)

    # Scale up spacing — uniformly across all clusters — only if the combined
    # waypoint count would exceed the budget.
    scale = 1.0
    if total_estimated > max_wps:
        scale = math.sqrt(total_estimated / max_wps)

    waypoints: List[Dict] = []
    for x_min, x_max, y_min, y_max in bboxes:
        waypoints.extend(_boustrophedon(
            x_min, x_max, y_min, y_max, target_z, altitude_m,
            spacing_m=spacing_m * scale, step_m=step_m * scale,
        ))

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
    ref_lon: Optional[float] = None,
    ref_lat: Optional[float] = None,
    altitude_m: float = 80.0,
    min_points: int = MIN_POINTS_PER_SQM,
    speed_ms: float = DRONE_SPEED_MS,
    max_iterations: int = MAX_ITERATIONS,
    pulse_freq_hz: int = PULSE_FREQ_HZ,
    scan_freq_hz: float = SCAN_FREQ_HZ,
    scan_angle_deg: float = SCAN_ANGLE_DEG,
    scanner_ref: str = DEFAULT_SCANNER_REF,
    platform_ref: str = DEFAULT_PLATFORM_REF,
    log: Optional[Callable[[str], None]] = None,
    stop_event: Optional["threading.Event"] = None,
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
        ref_lon:        Projection-origin longitude. MUST equal the ref_lon
                        passed to dtm_to_obj() when the terrain OBJ was built —
                        a mismatch silently shifts the mesh away from the
                        flight path, producing an empty point cloud. When
                        omitted, the route's centroid is used (and reused for
                        every refinement iteration).
        ref_lat:        Projection-origin latitude — see ref_lon.
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
            "las_path":          List[str],   # one per-iteration LAS output dir
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
        "las_path": [],
        "failing_cells_geo": [],
        "final_route": list(route),
        "error": None,
    }

    current_route = list(route)

    # Per-iteration supplemental waypoint budget, scaled to the size of the
    # ORIGINAL survey (computed once, so it doesn't grow as supplemental
    # passes accumulate into current_route across iterations).
    supplemental_cap = max(_MIN_SUPPLEMENTAL_WPS, int(len(route) * _SUPPLEMENTAL_WPS_RATIO))

    # Incremental simulation: iteration 0 simulates the full route, but every
    # later iteration simulates ONLY the newly-added supplemental waypoints
    # (already-validated legs are never re-flown/re-traced). The per-iteration
    # LAS outputs are merged for the density check so the grid still reflects
    # the full accumulated point cloud.
    sim_metric_route: Optional[List[Dict]] = None
    las_dirs: List[str] = []

    try:
        scene_xml = work_dir / "scene.xml"
        _write_scene_xml(scene_obj_path, str(scene_xml))

        for iteration in range(max_iterations):
            if stop_event is not None and stop_event.is_set():
                result["error"] = "Cancelled by user"
                break

            iter_dir = work_dir / f"iter_{iteration}"
            iter_dir.mkdir(exist_ok=True)

            # Stage 1 — export trajectory of the full accumulated route so far
            # (geographic → metric internally)
            traj_path = iter_dir / "trajectory.txt"
            export_trajectory(current_route, str(traj_path), speed_ms, is_geo)
            result["trajectory_path"] = str(traj_path)

            # Convert route to metric coordinates for the survey XML
            # Reuse the same projection origin across all refinement iterations
            # (and require it to match the terrain OBJ's origin — see docstring
            # of _route_to_metric) so the mesh and the flight legs never drift
            # apart from one run to the next.
            metric_route, ref_lon, ref_lat = _route_to_metric(current_route, is_geo, ref_lon, ref_lat)

            # Iteration 0 simulates the full route; later iterations simulate
            # only the supplemental waypoints added by the previous iteration.
            survey_metric_route = metric_route if sim_metric_route is None else sim_metric_route

            # Stage 2 — build survey XML and run HELIOS++
            survey_xml = iter_dir / "survey.xml"
            build_survey_xml(
                metric_route=survey_metric_route,
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
            las_dir = run_helios(
                helios_bin, str(survey_xml), str(las_output_dir), log=log, stop_event=stop_event
            )
            las_dirs.append(str(las_dir))

            # Stage 3 — verify density across all per-leg LAS files from every
            # iteration so far (full accumulated point cloud)
            result["las_path"] = list(las_dirs)
            passed, failing_metric = verify_point_density(las_dirs, min_points)
            result["iterations"] = iteration + 1
            result["passed"] = passed
            result["failing_cells_geo"] = _cells_to_geo(
                failing_metric, ref_lon, ref_lat, is_geo
            )

            if passed:
                break

            # Refinement — inject supplemental passes over failing zone
            if iteration < max_iterations - 1 and failing_metric:
                valid_zs = [float(wp["z"]) for wp in survey_metric_route
                            if not math.isnan(float(wp["z"]))]
                avg_z = float(np.mean(valid_zs)) if valid_zs else altitude_m
                terrain_z = avg_z - altitude_m

                extra_metric = _supplemental_passes(
                    failing_cells_metric=failing_metric,
                    terrain_z_m=terrain_z,
                    altitude_m=altitude_m,
                    max_wps=supplemental_cap,
                    spacing_m=3.0,
                    step_m=3.0,
                )

                if not extra_metric:
                    # No further targeted passes can be generated — stop here
                    # rather than re-simulating nothing next iteration.
                    break

                sim_metric_route = extra_metric

                if is_geo:
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

    except SimulationCancelled:
        result["error"] = "Cancelled by user"
    except Exception as exc:
        result["error"] = str(exc)

    return result
