import math
import numpy as np
from shapely.geometry import shape, LineString, Point, mapping

_LAT_M = 111139.0  # metres per degree latitude (WGS-84 approximation)


def lawnmower_waypoints(polygon, spacing, step):
    """Generate a simple lawnmower coverage path inside `polygon`.
    spacing: distance between adjacent passes (meters, or same CRS as polygon)
    step: distance between consecutive waypoints along a pass
    Returns a list of passes, each a list of (x,y) points.
    """
    minx, miny, maxx, maxy = polygon.bounds
    # choose orientation = 0 (horizontal passes along x)
    lines = []
    y = miny
    toggle = False
    while y <= maxy:
        line = LineString([(minx, y), (maxx, y)])
        inter = line.intersection(polygon)
        if not inter.is_empty:
            if inter.geom_type == 'MultiLineString':
                for seg in inter:
                    coords = list(seg.coords)
                    if toggle:
                        coords.reverse()
                    lines.append(coords)
            elif inter.geom_type == 'LineString':
                coords = list(inter.coords)
                if toggle:
                    coords.reverse()
                lines.append(coords)
        y += spacing
        toggle = not toggle
    # sample points along each pass, keeping passes separate
    passes = []
    for seg in lines:
        line = LineString(seg)
        length = line.length
        if length == 0:
            continue
        n = max(1, int(math.ceil(length / step)))
        pts = []
        for i in range(n+1):
            frac = i / n
            x, y = line.interpolate(frac, normalized=True).coords[0]
            pts.append((x, y))
        passes.append(pts)
    return passes


def _pass_altitude(dtm, pts, agl, step, elev_sample_step=None):
    """Constant altitude for a straight pass: (max terrain elevation along the
    pass) + agl.

    Terrain sampling resolution is decoupled from the waypoint step. When
    `elev_sample_step` is finer than `step`, terrain is resampled densely
    between the pass endpoints so a peak BETWEEN waypoints can't be missed —
    a flight-safety concern — without adding any waypoints/legs to the route
    (which would only inflate output size and LAS file count, not improve the
    altitude estimate). Returns NaN when no valid terrain is found.
    """
    if elev_sample_step and elev_sample_step < step and len(pts) >= 2:
        (x0, y0), (x1, y1) = pts[0], pts[-1]
        dist = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(dist / elev_sample_step)))
        sample_pts = [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n)
                      for i in range(n + 1)]
    else:
        sample_pts = pts
    elevs = [dtm.elevation_at(x, y) for x, y in sample_pts]
    valid = [e for e in elevs if not math.isnan(e)]
    return (max(valid) + agl) if valid else float('nan')


def plan_route(dtm, polygon, distance_above_surface, error_tolerance,
               spacing=10, step=5, elev_sample_step=None):
    """Plan a route that keeps `distance_above_surface` above the highest
    terrain point along each pass.

    For valid LiDAR strip registration, the drone must hold a constant
    altitude (height above sea level / DTM datum) along each straight pass —
    terrain-following per-waypoint altitude breaks registration. So Z is
    computed once per pass, as (max terrain elevation along that pass) +
    distance_above_surface, and held constant for every waypoint in the pass.
    Different passes may sit at different (but each internally constant)
    altitudes depending on the terrain under them.

    elev_sample_step: terrain-sampling resolution for the per-pass max-elevation
        check, independent of the waypoint `step`. When finer than `step`, the
        clearance calc won't skip a peak between waypoints — without adding
        waypoints/legs to the route. Defaults to `step` (no extra sampling).

    Returns list of dicts: {x, y, z, target_distance}
    """
    passes = lawnmower_waypoints(polygon, spacing, step)
    route = []
    for pass_id, pts in enumerate(passes):
        z = _pass_altitude(dtm, pts, distance_above_surface, step, elev_sample_step)
        for x, y in pts:
            route.append({'x': x, 'y': y, 'z': z, 'target_distance': distance_above_surface,
                          'error_tol': error_tolerance, 'pass_id': pass_id})
    return route


def _sample_line_in_polygon(polygon, y, step):
    """Sample (x, y) points at `step` intervals along the polygon's
    intersection with the horizontal line at `y`. Returns [] when the line
    misses the polygon."""
    minx, _, maxx, _ = polygon.bounds
    inter = LineString([(minx, y), (maxx, y)]).intersection(polygon)
    if inter.is_empty:
        return []
    segs = list(inter.geoms) if hasattr(inter, 'geoms') else [inter]
    pts = []
    for seg in segs:
        if seg.geom_type != 'LineString' or seg.length == 0:
            continue
        n = max(1, int(math.ceil(seg.length / step)))
        for i in range(n + 1):
            pts.append(seg.interpolate(i / n, normalized=True).coords[0])
    return pts


def plan_route_adaptive(dtm, polygon, distance_above_surface, error_tolerance,
                        scan_half_angle_deg, step,
                        overlap_frac=0.2, is_geo=True, min_spacing_m=2.0,
                        elev_sample_step=None):
    """Plan a lawnmower route with terrain-adaptive pass spacing.

    Altitude follows the same rule as plan_route (constant per pass:
    max terrain elevation along the pass + AGL). Spacing, however, is chosen
    per gap instead of globally: the baseline is the flat-ground optimum
    (2 * AGL * tan(half_angle) * (1 - overlap)), but it is tightened wherever
    terrain BETWEEN two passes rises above the terrain under them. At such a
    ridge the effective swath of both flanking passes shrinks (the drone is
    pegged to the max elevation under its own line, not the ridge), which is
    exactly where constant-spacing plans develop coverage gaps that the
    HELIOS++ feedback loop would otherwise have to repair.

    Coverage rule per gap: spacing <= (half_swath_cur + half_swath_next)
    evaluated at the highest terrain sample in the strip between the passes,
    scaled by (1 - overlap_frac). The candidate spacing depends on the next
    pass's altitude, which depends on its position, so the choice is iterated
    to a fixed point (3 rounds is plenty in practice).

    Returns list of dicts: {x, y, z, target_distance, error_tol, pass_id}.
    """
    agl = distance_above_surface
    tan_t = math.tan(math.radians(scan_half_angle_deg))
    base_spacing_m = 2.0 * agl * tan_t * (1.0 - overlap_frac)

    minx, miny, maxx, maxy = polygon.bounds
    units_per_m = (1.0 / _LAT_M) if is_geo else 1.0
    strip_xs = [minx + (maxx - minx) * i / 12.0 for i in range(13)]

    route = []
    y = miny
    toggle = False
    pass_id = 0
    while y <= maxy:
        pts = _sample_line_in_polygon(polygon, y, step)
        z = float('nan')
        if pts:
            # Sample terrain finely for the clearance calc (independent of the
            # waypoint step) so a peak between waypoints isn't missed.
            if elev_sample_step and elev_sample_step < step:
                elev_pts = _sample_line_in_polygon(polygon, y, elev_sample_step)
            else:
                elev_pts = pts
            elevs = [dtm.elevation_at(px, py) for px, py in elev_pts]
            valid = [e for e in elevs if not math.isnan(e)]
            if valid:
                z = max(valid) + agl
            ordered = list(reversed(pts)) if toggle else pts
            for px, py in ordered:
                route.append({'x': px, 'y': py, 'z': z,
                              'target_distance': agl, 'error_tol': error_tolerance,
                              'pass_id': pass_id})
            pass_id += 1
            toggle = not toggle

        z_cur = z if not math.isnan(z) else None
        s_m = base_spacing_m
        for _ in range(3):
            y_next = y + s_m * units_per_m
            strip_elevs, next_elevs = [], []
            for frac in (0.25, 0.5, 0.75):
                yy = y + (y_next - y) * frac
                for x in strip_xs:
                    if polygon.intersects(Point(x, yy)):
                        e = dtm.elevation_at(x, yy)
                        if not math.isnan(e):
                            strip_elevs.append(e)
            for x in strip_xs:
                if polygon.intersects(Point(x, y_next)):
                    e = dtm.elevation_at(x, y_next)
                    if not math.isnan(e):
                        next_elevs.append(e)
            if not strip_elevs and not next_elevs:
                break
            ridge = max(strip_elevs + next_elevs)
            z_next = (max(next_elevs) + agl) if next_elevs else None
            agl_cur = (z_cur - ridge) if z_cur is not None else agl
            agl_next = (z_next - ridge) if z_next is not None else agl
            hs_cur = max(agl_cur, 1.0) * tan_t
            hs_next = max(agl_next, 1.0) * tan_t
            allowed = (hs_cur + hs_next) * (1.0 - overlap_frac)
            s_new = max(min(base_spacing_m, allowed), min_spacing_m)
            if abs(s_new - s_m) < 0.25:
                s_m = s_new
                break
            s_m = s_new
        y += s_m * units_per_m

    return route
