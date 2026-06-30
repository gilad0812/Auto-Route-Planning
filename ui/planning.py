"""UI-agnostic glue between the Qt views and the model in ``src/``.

Keeps the widget code thin: load a DTM/CHM, build an AOI polygon, run the route
plan + density estimate, and return a plain result object. No Qt imports here on
purpose — this is testable without a display.
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
from route_planner import (                                # noqa: E402
    plan_route_adaptive, plan_route, _pass_altitude)
from density_estimate import estimate_density_grid       # noqa: E402

_LAT_M = 111139.0

# Scan (mirror-oscillation) frequency is derived from the pulse rate: across-track
# resolution f_scan = Δθ·PRR/(2·FOV) is linear in PRR with Δθ and the FOV fixed, so
# anchor to the scanner's nominal setting (600 kHz → 224.4 Hz) and scale.
_REF_PRR_HZ = 600_000.0
_REF_SCAN_HZ = 224.4


def scan_freq_for_prr(pulse_freq_hz):
    """Scan frequency (Hz) for a given pulse rate, scaled from the 600 kHz → 224.4 Hz
    nominal point so the across-track resolution stays fixed."""
    return _REF_SCAN_HZ * (float(pulse_freq_hz) / _REF_PRR_HZ)


@dataclass
class PlanParams:
    """Everything the compute loop needs, mirroring the Streamlit sidebar."""
    altitude_m: float = 100.0
    fov_deg: float = 100.0           # fixed (RIEGL VUX-120-23, ±50°)
    overlap_pct: float = 20.0
    adaptive_spacing: bool = True
    step_m: float = 50.0
    min_points: int = 100
    speed_ms: float = 6.0
    pulse_freq_hz: int = 600_000
    scan_freq_hz: float = 224.4
    veg_penetration: float = 0.4


@dataclass
class PlanResult:
    route: list = field(default_factory=list)
    estimate: dict = field(default_factory=dict)
    polygon: object = None
    area_m2: float = 0.0
    path_len_m: float = 0.0
    n_waypoints: int = 0
    alt_min: float = float('nan')
    alt_max: float = float('nan')


def chm_compatible(dtm, chm):
    """Whether `chm` can be used as a vegetation mask over `dtm`.

    The density estimator samples the CHM through the CHM's OWN raster transform at
    the DTM-frame lon/lat of each cell, so the CHM must share the DTM's CRS and
    actually overlap its extent — otherwise the mask lands on the wrong ground and
    the result is silently wrong. Resolution may differ (nearest-pixel lookup).

    Returns (ok, reason): ok False blocks applying it (reason = why); ok True with a
    non-empty reason is a soft note (e.g. partial overlap) the caller may surface.
    """
    dcrs, ccrs = dtm.src.crs, chm.src.crs
    de = dcrs.to_epsg() if dcrs else None
    ce = ccrs.to_epsg() if ccrs else None
    if de is not None and ce is not None:
        if de != ce:
            return False, f'CRS mismatch: DTM is EPSG:{de}, CHM is EPSG:{ce}.'
    elif dcrs != ccrs:
        return False, f'CRS mismatch: DTM {dcrs}, CHM {ccrs}.'

    db, cb = dtm.src.bounds, chm.src.bounds
    ox = min(db.right, cb.right) - max(db.left, cb.left)
    oy = min(db.top, cb.top) - max(db.bottom, cb.bottom)
    if ox <= 0 or oy <= 0:
        return False, 'CHM does not overlap the DTM extent (different area).'

    d_area = (db.right - db.left) * (db.top - db.bottom)
    o_area = ox * oy
    if d_area > 0 and o_area < 0.5 * d_area:
        return True, f'CHM covers only {100 * o_area / d_area:.0f}% of the DTM extent.'
    return True, ''


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
    """Run plan + density estimate for one AOI. Returns a PlanResult."""
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

    return estimate_for_route(dtm, polygon, route, params, chm=chm, is_geo=is_geo)


def estimate_for_route(dtm, polygon, route, params: PlanParams, chm=None, is_geo=True):
    """Run the density estimate + route stats for an already-built route over the
    AOI. Used both for a freshly planned route and after manually adding passes."""
    half = params.fov_deg / 2.0
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
    res.n_waypoints = len(wps)
    res.path_len_m = _path_length_m(route, is_geo)
    if wps:
        zs = [w['z'] for w in wps]
        res.alt_min, res.alt_max = min(zs), max(zs)
    return res


def build_manual_pass(dtm, p0, p1, params: PlanParams, is_geo, pass_id):
    """Build waypoints for a hand-drawn straight pass between (lon,lat) endpoints
    p0→p1. Altitude is set automatically like any pass: max terrain along it + AGL,
    held constant. Returns [] if the segment finds no valid terrain."""
    to_m = _LAT_M if is_geo else 1.0
    step_map = params.step_m / to_m
    dtm_res_map = min(abs(dtm.src.res[0]), abs(dtm.src.res[1]))
    elev_step_map = min(step_map, dtm_res_map)

    (x0, y0), (x1, y1) = p0, p1
    dist = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(math.ceil(dist / step_map))) if step_map > 0 else 1
    pts = [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]
    z = _pass_altitude(dtm, pts, params.altitude_m, step_map, elev_step_map)
    if math.isnan(z):
        return []
    return [{'x': x, 'y': y, 'z': z, 'target_distance': params.altitude_m,
             'pass_id': pass_id} for x, y in pts]


def load_dtm(path):
    return DTM(path)
