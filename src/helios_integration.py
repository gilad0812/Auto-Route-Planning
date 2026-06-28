"""
HELIOS++ LiDAR simulation feedback loop.

Three-stage pipeline
────────────────────────────────────────────────────────────────────────────
Stage 1 │ export_trajectory   – route → trajectory .txt (timestamp x y z hdg r p)
Stage 2 │ run_helios           – survey .xml → HELIOS++ subprocess → .las output
Stage 3 │ verify_point_density – .las → 1×1 m density grid → pass/fail + cells
────────────────────────────────────────────────────────────────────────────
Orchestrator: run_feedback_loop chains all three stages in a single simulation
pass and reports whether point density meets the threshold across the AOI.
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


def _collapse_group(group: List[Dict], tol_m: float) -> List[Dict]:
    """Reduce one same-pass run to its two endpoints when it is a straight,
    constant-altitude line; otherwise keep it intact."""
    if len(group) <= 2:
        return list(group)
    a, b = group[0], group[-1]

    zs = [float(w["z"]) for w in group if not math.isnan(float(w["z"]))]
    z_const = (not zs) or (max(zs) - min(zs) <= 0.01)

    ax, ay = float(a["x"]), float(a["y"])
    dx, dy = float(b["x"]) - ax, float(b["y"]) - ay
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return list(group)
    # Perpendicular distance of every interior point to the chord a→b.
    straight = all(
        abs(dx * (float(w["y"]) - ay) - dy * (float(w["x"]) - ax)) / length <= tol_m
        for w in group[1:-1]
    )
    return [a, b] if (straight and z_const) else list(group)


def collapse_straight_passes(metric_route: List[Dict], tol_m: float = 0.5) -> List[Dict]:
    """Collapse colinear, constant-altitude runs within each pass to their two
    endpoints.

    HELIOS++ interpolates the platform linearly between legs and scans
    continuously, so a straight pass needs only its start and end legs — the
    intermediate colinear waypoints fire identical pulses while multiplying the
    leg count and the number of per-leg LAS output files. Collapsing them
    leaves the trajectory, pulses, and resulting point cloud unchanged while
    cutting leg/LAS-file count roughly to the number of passes.

    Runs are grouped by `pass_id`. Any run that lacks a pass_id, or that isn't a
    straight constant-z line within `tol_m`, is kept verbatim so route shape is
    never distorted (turns and per-pass altitude changes are always preserved).
    """
    if len(metric_route) <= 2:
        return list(metric_route)
    out: List[Dict] = []
    i, n = 0, len(metric_route)
    while i < n:
        pid = metric_route[i].get("pass_id")
        if pid is None:
            out.append(metric_route[i])
            i += 1
            continue
        j = i
        while j + 1 < n and metric_route[j + 1].get("pass_id") == pid:
            j += 1
        out.extend(_collapse_group(metric_route[i:j + 1], tol_m))
        i = j + 1
    return out


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
    collapse_legs: bool = True,
) -> str:
    """
    Generate a HELIOS++ survey XML from metric route waypoints.

    By default each straight, constant-altitude pass is collapsed to two legs
    (start + end) — see collapse_straight_passes. HELIOS++ linearly interpolates
    the platform between consecutive legs and fires the scanner along each
    segment, so this yields an identical point cloud with far fewer legs and
    per-leg LAS output files. Set collapse_legs=False to emit one leg per
    waypoint.

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
        collapse_legs:    When True, collapse straight passes to endpoint legs.

    Returns:
        Absolute path of the written survey XML.
    """
    if collapse_legs:
        metric_route = collapse_straight_passes(metric_route)

    scene_ref = f"{_posix(os.path.abspath(scene_xml_path))}#{scene_id}"

    doc = ET.Element("document")

    survey = ET.SubElement(
        doc, "survey",
        name=survey_name,
        scene=scene_ref,
        platform=platform_ref,
        scanner=scanner_ref,
    )

    # Scanner settings written inline on every leg (avoids template lookup issues).
    # `active` is decided per leg below: the scanner fires only WHILE flying along
    # a pass, and is OFF during the connector hop between passes (the side lines),
    # which real post-processing discards. HELIOS' leg `active` governs the segment
    # from this leg to the NEXT one, so a leg scans iff its next leg is in the same
    # pass. (If a sim ever comes back near-empty on the passes, this is the flag to
    # flip — it would mean HELIOS uses the previous→this convention instead.)
    _scan_base = dict(
        pulseFreq_hz=str(pulse_freq_hz),   # HELIOS++ expects lowercase 'hz'
        scanFreq_hz=str(scan_freq_hz),
        scanAngle_deg=str(scan_angle_deg),
        headRotatePerSec_deg="0",
    )

    n_legs = len(metric_route)
    for i, wp in enumerate(metric_route):
        x = float(wp["x"])
        y = float(wp["y"])
        z_raw = wp["z"]
        z = float(z_raw) if not math.isnan(float(z_raw)) else 0.0

        nxt = metric_route[i + 1] if i + 1 < n_legs else None
        on_pass = (nxt is not None
                   and wp.get("pass_id") is not None
                   and wp.get("pass_id") == nxt.get("pass_id"))

        leg = ET.SubElement(survey, "leg")
        ET.SubElement(
            leg, "platformSettings",
            x=f"{x:.4f}", y=f"{y:.4f}", z=f"{z:.4f}",
            onGround="false",
            movePerSec_m=str(speed_ms),
        )
        ET.SubElement(leg, "scannerSettings",
                      active=("true" if on_pass else "false"), **_scan_base)

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

    # Build an env that includes the Conda runtime library directories so the
    # binary can load its dependencies (STATUS_DLL_NOT_FOUND on Windows /
    # "error while loading shared libraries" on Linux otherwise). HELIOS++ is
    # distributed as a full Conda env, identifiable by a conda-meta/ marker
    # directory at its root.
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
            # Build a CLEAN PATH (conda dirs + Windows system dirs only) instead
            # of inheriting ours. The parent process's PATH — the project venv's
            # rasterio/GDAL DLLs in dev, or the PyInstaller bundle's DLLs in the
            # frozen .exe — otherwise shadows HELIOS's own libraries and it dies
            # at startup with a wrong-version DLL (STATUS_ENTRYPOINT_NOT_FOUND,
            # 0xC0000139). A clean PATH makes HELIOS load only its own DLLs.
            _winroot = os.environ.get("SystemRoot", r"C:\Windows")
            _sys_dirs = [
                os.path.join(_winroot, "System32"),
                _winroot,
                os.path.join(_winroot, "System32", "Wbem"),
            ]
            run_env["PATH"] = os.pathsep.join(_dll_dirs + _sys_dirs)
            # Drop our GDAL/PROJ data pointers so HELIOS uses its own (a stray
            # PROJ_LIB/GDAL_DATA from rasterio can mismatch HELIOS's GDAL).
            for _v in ("GDAL_DATA", "PROJ_LIB", "PROJ_DATA", "GDAL_DRIVER_PATH"):
                run_env.pop(_v, None)
    else:
        # Walk up from the binary to find the Conda env root (has conda-meta/)
        _candidate = helios_bin.parent
        _conda_root = None
        for _ in range(8):
            if (_candidate / "conda-meta").is_dir():
                _conda_root = _candidate
                break
            _candidate = _candidate.parent
        if _conda_root:
            _lib_dir = _conda_root / "lib"
            if _lib_dir.is_dir():
                run_env["LD_LIBRARY_PATH"] = (
                    str(_lib_dir) + os.pathsep + run_env.get("LD_LIBRARY_PATH", "")
                )

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

def _points_in_polygon(
    xs: "np.ndarray", ys: "np.ndarray", poly: List[Tuple[float, float]]
) -> "np.ndarray":
    """Vectorised even-odd ray-casting point-in-polygon test.

    poly: list of (x, y) vertices (exterior ring) in the same coords as xs/ys.
    Returns a boolean array, True where (xs, ys) falls inside the polygon.
    """
    inside = np.zeros(xs.shape, dtype=bool)
    n = len(poly)
    if n < 3:
        return inside
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        crosses = (yi > ys) != (yj > ys)
        x_at_y = (xj - xi) * (ys - yi) / ((yj - yi) or 1e-15) + xi
        inside ^= crosses & (xs < x_at_y)
        j = i
    return inside


def _surface_stretch(dtm, gx, gy, ref_lon, ref_lat, is_geo):
    """Per-cell tilted-surface / horizontal area ratio (= sec slope), from the DTM
    at its NATIVE pixel resolution. Lets verify_point_density report density per
    SURFACE m² (count / (cell² · stretch)) to match the estimator's convention.
    Sampling the DTM's own pixels avoids the 1 m staircase a coarse DTM would make."""
    arr = np.asarray(dtm.array, dtype=float)
    t = dtm.src.transform
    if is_geo:
        lon_m = _LAT_M * math.cos(math.radians(ref_lat)); lat_m = _LAT_M
        lon = ref_lon + gx / lon_m; lat = ref_lat + gy / lat_m
    else:
        lon, lat, lon_m, lat_m = gx, gy, 1.0, 1.0
    af = np.where(arr == dtm.nodata, np.nan, arr) if dtm.nodata is not None else arr
    af = np.where(np.isnan(af), np.nanmean(af), af)
    gE = np.gradient(af, axis=1) / (t.a * lon_m)
    gN = np.gradient(af, axis=0) / (t.e * lat_m)
    ci = np.clip(((lon - t.c) / t.a).astype(int), 0, arr.shape[1] - 1)
    ri = np.clip(((lat - t.f) / t.e).astype(int), 0, arr.shape[0] - 1)
    ge, gn = gE[ri, ci], gN[ri, ci]
    return np.sqrt(1.0 + ge * ge + gn * gn)


def verify_point_density(
    las_path: Union[str, List[str]],
    min_points: int = MIN_POINTS_PER_SQM,
    cell_size: float = CELL_SIZE_M,
    region: Optional[List[Tuple[float, float]]] = None,
    dtm=None,
    ref_lon: Optional[float] = None,
    ref_lat: Optional[float] = None,
    is_geo: bool = True,
    chm=None,
    veg_penetration: float = 0.4,
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
        region:     Optional polygon (list of (x, y) vertices in the LAS
                    coordinate system) bounding the area of interest. When
                    given, cells whose centre falls outside it are ignored — so
                    the swath overhang scanned beyond the survey polygon never
                    counts as a density failure. When None, every populated
                    cell is evaluated.

    Returns:
        (passed, failing_cells, stats) where:
          - passed:        True if every populated cell meets the threshold.
          - failing_cells: List of (x_centre, y_centre) in the LAS coordinate
                           system.  Empty when passed=True or the cloud is empty.
          - stats:         Dict of AOI density statistics over every cell incl.
                           empty ones — n_cells, n_met, n_under, n_void,
                           median_density, min_density, cell_size_m (pts/m²).
                           Empty {} when the cloud is empty.

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
        return False, [], {}

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
        return False, [], {}

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

    # Per-cell density in pts/m². By default the divisor is the flat cell area
    # (cell_size²). When a DTM is supplied, divide instead by the TILTED SURFACE
    # area (cell_size² · sec slope) so density is reported per SURFACE m² — the
    # survey-quality metric, matching the estimator's convention. Computed as a
    # full grid so the pass/fail test and the stats use the same definition.
    all_cx = x_min + (np.arange(nx) + 0.5) * cell_size
    all_cy = y_min + (np.arange(ny) + 0.5) * cell_size
    gx, gy = np.meshgrid(all_cx, all_cy)
    cell_area = float(cell_size * cell_size)
    if dtm is not None:
        cell_area = cell_area * _surface_stretch(dtm, gx, gy, ref_lon, ref_lat, is_geo)
    dens_grid = grid.astype(float) / cell_area     # pts per (surface) m²

    # Canopy thinning: where a binary vegetation mask (chm > 0) marks vegetation,
    # multiply the ground density by `veg_penetration` (thumb rule 0.4) — the SAME
    # planning factor the estimator applies, so the two agree over vegetated
    # cells. HELIOS itself sims bare earth; this is a post-process factor, not
    # ray-traced canopy occlusion. Sample the mask by location (metric→lon/lat).
    if chm is not None:
        ca = np.asarray(chm.array, dtype=float)
        ct = chm.src.transform
        if is_geo:
            lon_m = _LAT_M * math.cos(math.radians(ref_lat)); lat_m = _LAT_M
            lon = ref_lon + gx / lon_m; lat = ref_lat + gy / lat_m
        else:
            lon, lat = gx, gy
        cc = np.clip(((lon - ct.c) / ct.a).astype(int), 0, ca.shape[1] - 1)
        rr = np.clip(((lat - ct.f) / ct.e).astype(int), 0, ca.shape[0] - 1)
        mask = ca[rr, cc]
        if chm.nodata is not None:
            mask = np.where(mask == chm.nodata, 0.0, mask)
        veg = np.isfinite(mask) & (mask > 0)
        dens_grid = np.where(veg, dens_grid * float(veg_penetration), dens_grid)

    # With an AOI polygon, count EVERY in-region cell below the threshold —
    # including empty ones (occlusion voids are real coverage gaps); the polygon
    # mask below excludes the rectangular grid's empty margin. Without a polygon
    # we can't tell "outside the survey" from "void inside it", so we fall back to
    # only judging populated cells (grid > 0) to avoid flagging the empty margin.
    if region is not None:
        fail_mask = (dens_grid < min_points)
    else:
        fail_mask = (grid > 0) & (dens_grid < min_points)
    rows, cols = np.where(fail_mask)

    cx = x_min + (cols + 0.5) * cell_size
    cy = y_min + (rows + 0.5) * cell_size

    # Ignore failures outside the area of interest (swath overhang beyond the
    # survey polygon) so they don't trigger pointless refinement.
    if region is not None:
        keep = _points_in_polygon(cx, cy, region)
        cx, cy = cx[keep], cy[keep]

    failing_cells: List[Tuple[float, float]] = list(zip(cx.tolist(), cy.tolist()))

    # ── AOI density statistics (coverage %, voids, median/min) ───────────────
    # Over every AOI cell INCLUDING empty ones, so coverage reflects the whole
    # surveyed area (voids = empty cells are counted too).
    if region is not None:
        in_region = _points_in_polygon(gx.ravel(), gy.ravel(), region).reshape(grid.shape)
    else:
        in_region = np.ones_like(grid, dtype=bool)
    dens = dens_grid[in_region]
    n_cells = int(dens.size)
    stats = {
        "n_cells": n_cells,
        "n_met":   int((dens >= min_points).sum()),
        "n_under": int(((dens > 0) & (dens < min_points)).sum()),
        "n_void":  int((dens <= 0).sum()),
        "median_density": float(np.median(dens)) if n_cells else 0.0,
        "min_density":    float(dens.min()) if n_cells else 0.0,
        "cell_size_m": cell_size,
    }

    return len(failing_cells) == 0, failing_cells, stats


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
    pulse_freq_hz: int = PULSE_FREQ_HZ,
    scan_freq_hz: float = SCAN_FREQ_HZ,
    scan_angle_deg: float = SCAN_ANGLE_DEG,
    scanner_ref: str = DEFAULT_SCANNER_REF,
    platform_ref: str = DEFAULT_PLATFORM_REF,
    dtm: Optional[object] = None,
    region_polygon: Optional[List[Tuple[float, float]]] = None,
    chm: Optional[object] = None,
    veg_penetration: float = 0.4,
    log: Optional[Callable[[str], None]] = None,
    stop_event: Optional["threading.Event"] = None,
) -> Dict:
    """
    HELIOS++ validation pipeline — a single simulation pass (no refinement).

    Runs: export_trajectory → build_survey_xml → run_helios → verify_point_density,
    and reports whether the route's point density meets the threshold across the
    AOI. The route is never modified.

    Args:
        route:          Waypoints from plan_route() in the DTM's CRS.
        helios_bin:     Path to the HELIOS++ executable.
        scene_obj_path: Path to the terrain Wavefront OBJ mesh.
        work_dir:       Working directory for all intermediate files.
        is_geo:         True when route x=longitude, y=latitude (degrees).
        ref_lon:        Projection-origin longitude. MUST equal the ref_lon
                        passed to dtm_to_obj() when the terrain OBJ was built —
                        a mismatch silently shifts the mesh away from the flight
                        path, producing an empty point cloud. When omitted, the
                        route's centroid is used.
        ref_lat:        Projection-origin latitude — see ref_lon.
        altitude_m:     AGL altitude (recorded for reference).
        min_points:     Minimum points/m² threshold.
        speed_ms:       Drone speed (m/s).
        pulse_freq_hz:  LiDAR pulse repetition frequency (Hz).
        scan_freq_hz:   Scanner rotation frequency (Hz).
        scan_angle_deg: Half-FOV from nadir (degrees).
        scanner_ref:    HELIOS++ scanner XML reference (path#id).
        platform_ref:   HELIOS++ platform XML reference (path#id).
        region_polygon: Optional survey-polygon vertices [(x, y), ...] in the
                        same CRS as `route`. The density check is restricted to
                        cells inside it, so swath overhang scanned beyond the
                        polygon doesn't count as a failure.
        dtm:            Unused (kept for call-site compatibility).

    Returns:
        {
            "passed":            bool,
            "iterations":        int,   # always 1
            "trajectory_path":   str,
            "survey_xml_path":   str,
            "las_path":          List[str],
            "failing_cells_geo": List[Tuple[float, float]],  # original CRS
            "density_stats":     dict,
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
        "density_stats": {},
        "error": None,
    }

    # Survey polygon in metric coords, derived once the projection origin is
    # known. Restricts the density check to the area of interest so swath
    # overhang beyond the polygon doesn't count as a failure.
    region_metric: Optional[List[Tuple[float, float]]] = None

    try:
        scene_xml = work_dir / "scene.xml"
        _write_scene_xml(scene_obj_path, str(scene_xml))

        if stop_event is not None and stop_event.is_set():
            result["error"] = "Cancelled by user"
            return result

        # Stage 1 — export trajectory (geographic → metric internally)
        traj_path = work_dir / "trajectory.txt"
        export_trajectory(route, str(traj_path), speed_ms, is_geo)
        result["trajectory_path"] = str(traj_path)

        # Route → metric for the survey XML. The projection origin MUST match the
        # terrain OBJ's origin (see _route_to_metric) or the mesh and flight legs
        # drift apart and HELIOS records zero points.
        metric_route, ref_lon, ref_lat = _route_to_metric(route, is_geo, ref_lon, ref_lat)

        # Project the survey polygon into the same metric frame.
        if region_polygon is not None:
            if is_geo:
                lon_m = _LAT_M * math.cos(math.radians(ref_lat))
                region_metric = [
                    ((lx - ref_lon) * lon_m, (ly - ref_lat) * _LAT_M)
                    for lx, ly in region_polygon
                ]
            else:
                region_metric = list(region_polygon)

        # Stage 2 — build survey XML and run HELIOS++
        survey_xml = work_dir / "survey.xml"
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

        las_output_dir = work_dir / "output"
        las_output_dir.mkdir(exist_ok=True)
        las_dir = run_helios(
            helios_bin, str(survey_xml), str(las_output_dir), log=log, stop_event=stop_event
        )
        result["las_path"] = [str(las_dir)]

        # Stage 3 — verify density inside the AOI
        passed, failing_metric, density_stats = verify_point_density(
            [str(las_dir)], min_points, region=region_metric,
            dtm=dtm, ref_lon=ref_lon, ref_lat=ref_lat, is_geo=is_geo,
            chm=chm, veg_penetration=veg_penetration)
        result["iterations"] = 1
        result["passed"] = passed
        result["density_stats"] = density_stats
        result["failing_cells_geo"] = _cells_to_geo(
            failing_metric, ref_lon, ref_lat, is_geo
        )

    except SimulationCancelled:
        result["error"] = "Cancelled by user"
    except Exception as exc:
        result["error"] = str(exc)

    return result
