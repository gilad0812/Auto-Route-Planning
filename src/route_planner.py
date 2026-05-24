import math
import numpy as np
from shapely.geometry import shape, LineString, Point, mapping


def lawnmower_waypoints(polygon, spacing, step):
    """Generate a simple lawnmower coverage path inside `polygon`.
    spacing: distance between adjacent passes (meters, or same CRS as polygon)
    step: distance between consecutive waypoints along a pass
    Returns list of (x,y) points.
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
    # sample points along segments
    pts = []
    for seg in lines:
        line = LineString(seg)
        length = line.length
        if length == 0:
            continue
        n = max(1, int(math.ceil(length / step)))
        for i in range(n+1):
            frac = i / n
            x, y = line.interpolate(frac, normalized=True).coords[0]
            pts.append((x, y))
    return pts


def plan_route(dtm, polygon, distance_above_surface, error_tolerance, spacing=10, step=5):
    """Plan a route that keeps `distance_above_surface` from the DTM surface.
    Returns list of dicts: {x, y, z, target_distance}
    """
    pts = lawnmower_waypoints(polygon, spacing, step)
    route = []
    for x, y in pts:
        elev = dtm.elevation_at(x, y)
        if math.isnan(elev):
            z = float('nan')
        else:
            z = elev + distance_above_surface
        route.append({'x': x, 'y': y, 'z': z, 'target_distance': distance_above_surface, 'error_tol': error_tolerance})
    return route
