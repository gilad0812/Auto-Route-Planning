"""UI-agnostic glue between the Qt views and the model in ``src/``.

Keeps the widget code thin: load a DTM/CHM, build an AOI polygon, run the route
plan + density estimate + safety check, and return a plain result object. No Qt
imports here on purpose — this is testable without a display.
"""
import math
import os
import sys
from dataclasses import dataclass, field

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src')
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from shapely.geometry import Polygon                     # noqa: E402
from dtm import DTM                                      # noqa: E402
from route_planner import plan_route_adaptive, plan_route  # noqa: E402
from density_estimate import estimate_density_grid       # noqa: E402
from safety import mission_safety                        # noqa: E402

_LAT_M = 111139.0


@dataclass
class PlanParams:
    """Everything the compute loop needs, mirroring the Streamlit sidebar."""
    altitude_m: float = 100.0
    fov_deg: float = 100.0           # fixed (RIEGL VUX-120-23, ±50°)
    overlap_pct: float = 20.0
    adaptive_spacing: bool = True
    step_m: float = 50.0
    min_clearance_m: float = 30.0
    agl_ceiling_m: float = 120.0
    min_points: int = 100
    speed_ms: float = 8.0
    pulse_freq_hz: int = 1_200_000
    scan_freq_hz: float = 200.0
    veg_penetration: float = 0.4


@dataclass
class PlanResult:
    route: list = field(default_factory=list)
    estimate: dict = field(default_factory=dict)
    safety: dict = field(default_factory=dict)
    polygon: object = None
    area_m2: float = 0.0
    path_len_m: float = 0.0
    n_waypoints: int = 0
    alt_min: float = float('nan')
    alt_max: float = float('nan')


def centered_box(dtm, frac=0.5):
    """A polygon covering the central `frac` of the DTM extent — the stand-in for
    map-drawn AOIs until the map view lands."""
    b = dtm.src.bounds
    cx, cy = (b.left + b.right) / 2, (b.bottom + b.top) / 2
    dx = (b.right - b.left) * frac / 2
    dy = (b.top - b.bottom) * frac / 2
    return Polygon([(cx - dx, cy - dy), (cx + dx, cy - dy),
                    (cx + dx, cy + dy), (cx - dx, cy + dy)])


def polygon_area_m2(poly, is_geo):
    if not is_geo:
        return poly.area
    lat0 = poly.centroid.y
    return poly.area * _LAT_M * (_LAT_M * math.cos(math.radians(lat0)))


def _path_length_m(route, is_geo):
    wps = [w for w in route
           if not (isinstance(w['z'], float) and math.isnan(w['z']))]
    if len(wps) < 2:
        return 0.0
    lat0 = sum(w['y'] for w in wps) / len(wps)
    lon_m = _LAT_M * math.cos(math.radians(lat0)) if is_geo else 1.0
    lat_m = _LAT_M if is_geo else 1.0
    tot = 0.0
    for a, b in zip(wps, wps[1:]):
        tot += math.hypot((b['x'] - a['x']) * lon_m, (b['y'] - a['y']) * lat_m)
    return tot


def compute_plan(dtm, polygon, params: PlanParams, chm=None, is_geo=True):
    """Run plan + estimate + safety for one AOI. Returns a PlanResult."""
    to_m = _LAT_M if is_geo else 1.0
    step_map = params.step_m / to_m
    dtm_res_map = min(abs(dtm.src.res[0]), abs(dtm.src.res[1]))
    elev_step_map = min(step_map, dtm_res_map)
    half = params.fov_deg / 2.0

    if params.adaptive_spacing:
        route = plan_route_adaptive(
            dtm, polygon, params.altitude_m,
            scan_half_angle_deg=half, step=step_map,
            overlap_frac=params.overlap_pct / 100.0, is_geo=is_geo,
            elev_sample_step=elev_step_map,
        )
    else:
        spacing_map = (2.0 * params.altitude_m
                       * math.tan(math.radians(half))
                       * (1.0 - params.overlap_pct / 100.0)) / to_m
        route = plan_route(dtm, polygon, params.altitude_m, spacing_map,
                           step_map, elev_sample_step=elev_step_map)

    res = PlanResult(route=route, polygon=polygon)
    res.area_m2 = polygon_area_m2(polygon, is_geo)
    if not route:
        return res

    res.estimate = estimate_density_grid(
        route, dtm, list(polygon.exterior.coords),
        pulse_freq_hz=int(params.pulse_freq_hz), scan_freq_hz=float(params.scan_freq_hz),
        scan_half_angle_deg=half, speed_ms=float(params.speed_ms),
        min_points=int(params.min_points), is_geo=is_geo,
        chm=chm, veg_penetration=float(params.veg_penetration),
    )

    wps = [w for w in route
           if not (isinstance(w['z'], float) and math.isnan(w['z']))]
    if len(wps) >= 2:
        res.safety = mission_safety(
            wps, dtm, is_geo=is_geo,
            clearance_floor_m=float(params.min_clearance_m),
            agl_ceiling_m=float(params.agl_ceiling_m),
        )
    res.n_waypoints = len(wps)
    res.path_len_m = _path_length_m(route, is_geo)
    if wps:
        zs = [w['z'] for w in wps]
        res.alt_min, res.alt_max = min(zs), max(zs)
    return res


def load_dtm(path):
    return DTM(path)
