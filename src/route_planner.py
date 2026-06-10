import math
import numpy as np
from shapely.geometry import shape, LineString, Point, mapping


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


def plan_route(dtm, polygon, distance_above_surface, error_tolerance, spacing=10, step=5):
    """Plan a route that keeps `distance_above_surface` above the highest
    terrain point along each pass.

    For valid LiDAR strip registration, the drone must hold a constant
    altitude (height above sea level / DTM datum) along each straight pass —
    terrain-following per-waypoint altitude breaks registration. So Z is
    computed once per pass, as (max terrain elevation along that pass) +
    distance_above_surface, and held constant for every waypoint in the pass.
    Different passes may sit at different (but each internally constant)
    altitudes depending on the terrain under them.

    Returns list of dicts: {x, y, z, target_distance}
    """
    passes = lawnmower_waypoints(polygon, spacing, step)
    route = []
    for pts in passes:
        elevs = [dtm.elevation_at(x, y) for x, y in pts]
        valid_elevs = [e for e in elevs if not math.isnan(e)]
        if valid_elevs:
            z = max(valid_elevs) + distance_above_surface
        else:
            z = float('nan')
        for x, y in pts:
            route.append({'x': x, 'y': y, 'z': z, 'target_distance': distance_above_surface, 'error_tol': error_tolerance})
    return route
