import math
import numpy as np
from shapely.geometry import shape, LineString, Point, Polygon, mapping

_LAT_M = 111139.0  # metres per degree latitude (WGS-84 approximation)

# Adaptive spacing tightens over ridges but never below this fraction of base
# spacing — without a floor a steep feature between passes collapses spacing to
# min_spacing_m, stamping dozens of coincident lines that don't even fix the
# (occlusion-driven) under-density. 0.4 holds coverage within ~0.2% of no floor.
_MIN_TIGHTEN_FRAC = 0.4


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
    """Constant altitude for a straight pass: mean terrain elevation along it + agl.

    elev_sample_step decouples terrain sampling from the waypoint step: when finer,
    terrain is resampled densely so the mean reflects the whole pass, not just the
    waypoints. Returns NaN when no valid terrain is found.

    NOTE: altitude tracks the AVERAGE terrain of the pass, so AGL is only nominal on
    average — over a local peak the true clearance drops below `agl` (and can go
    negative if a spike rises more than `agl` above the pass mean). The autopilot /
    operator owns terrain-collision avoidance.
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
    return (sum(valid) / len(valid) + agl) if valid else float('nan')


def plan_route(dtm, polygon, distance_above_surface,
               spacing=10, step=5, elev_sample_step=None):
    """Plan a route holding `distance_above_surface` above the highest terrain along
    each pass.

    LiDAR strip registration needs constant altitude per straight pass (terrain-
    following per-waypoint breaks it), so Z = mean terrain along the pass + AGL, held
    constant for the whole pass; different passes may sit at different altitudes.

    elev_sample_step: terrain-sampling resolution for the max-elevation check,
    independent of `step`. Returns list of {x, y, z, target_distance}.
    """
    passes = lawnmower_waypoints(polygon, spacing, step)
    route = []
    for pass_id, pts in enumerate(passes):
        # altitude from the DENSE samples; emit only the pass endpoints (a straight
        # constant-altitude line needs no intermediate waypoints — the turns are it).
        z = _pass_altitude(dtm, pts, distance_above_surface, step, elev_sample_step)
        ends = [pts[0], pts[-1]] if len(pts) >= 2 else pts
        for x, y in ends:
            route.append({'x': x, 'y': y, 'z': z, 'target_distance': distance_above_surface,
                          'pass_id': pass_id})
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


def _auto_pass_angle(dtm, polygon, lon_m, lat_m, n=21, n_angles=180):
    """Pick the pass heading (rad, local metric frame) that best follows the contours
    — the orientation minimising how much each pass climbs.

    Bins sampled terrain by across-pass coordinate v and minimises the mean within-
    bin elevation spread, so passes stay near constant terrain elevation (the
    constant-altitude rule then doesn't fly far above a pass's low end). Replaces a
    mean-gradient heuristic that cancelled out on ridges/valleys. 0.0 if flat.
    """
    minx, miny, maxx, maxy = polygon.bounds
    xs = np.linspace(minx, maxx, n)
    ys = np.linspace(miny, maxy, n)
    cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0
    pxs, pys, pes = [], [], []                       # sample points in local metres
    for yy in ys:
        for xx in xs:
            if polygon.intersects(Point(xx, yy)):
                e = dtm.elevation_at(xx, yy)
                if not math.isnan(e):
                    pxs.append((xx - cx) * lon_m)
                    pys.append((yy - cy) * lat_m)
                    pes.append(e)
    if len(pes) < 4:
        return 0.0
    pxs, pys, pes = np.asarray(pxs), np.asarray(pys), np.asarray(pes)
    if float(pes.max() - pes.min()) < 1.0:           # ~flat: orientation irrelevant
        return 0.0

    # Fixed metric band width (independent of heading) so the spread metric is
    # comparable across all candidate angles.
    span = max(float(pxs.max() - pxs.min()), float(pys.max() - pys.min()), 1.0)
    band = span / 14.0

    best_theta, best_cost = 0.0, float('inf')
    for k in range(n_angles):
        theta = math.pi * k / n_angles               # 0 … <π (a pass axis is bidirectional)
        ct, st = math.cos(theta), math.sin(theta)
        v = -pxs * st + pys * ct                     # across-pass coordinate
        idx = np.floor((v - v.min()) / band).astype(int)
        cost, weight = 0.0, 0
        for b in np.unique(idx):
            ez = pes[idx == b]
            if ez.size >= 2:
                cost += float(ez.max() - ez.min()) * ez.size
                weight += ez.size
        cost = cost / weight if weight else float('inf')
        if cost < best_cost - 1e-9:
            best_cost, best_theta = cost, theta
    return best_theta


def plan_route_adaptive(dtm, polygon, distance_above_surface,
                        scan_half_angle_deg, step,
                        overlap_frac=0.2, is_geo=True, min_spacing_m=2.0,
                        elev_sample_step=None, orientation='auto'):
    """Lawnmower route with terrain-adaptive pass spacing and orientation.

    Orientation: passes follow the contours (via _auto_pass_angle) — since altitude
    is constant per pass, a pass crossing a big elevation range flies far above its
    low end, widening the swath and thinning density. 'auto' / 'ew' / 'ns'.

    Spacing: tightened where terrain between two passes rises and shrinks their
    swath, evaluated at the highest terrain in the strip, iterated to a fixed point.

    Passes stop at the AOI boundary (rim margin is the operator's job, by over-
    drawing the polygon). Runs in a rotated metric frame so passes are horizontal.
    Returns list of {x, y, z, target_distance, pass_id}.
    """
    agl = distance_above_surface
    tan_t = math.tan(math.radians(scan_half_angle_deg))
    half_swath_m = agl * tan_t
    base_spacing_m = 2.0 * half_swath_m * (1.0 - overlap_frac)

    # Local metric frame centred on the polygon.
    c = polygon.centroid
    lon0, lat0 = c.x, c.y
    if is_geo:
        lat_m = _LAT_M
        lon_m = _LAT_M * math.cos(math.radians(lat0))
    else:
        lat_m = lon_m = 1.0

    if orientation == 'ew':
        theta = 0.0
    elif orientation == 'ns':
        theta = math.pi / 2.0
    else:
        theta = _auto_pass_angle(dtm, polygon, lon_m, lat_m)
    ct, st = math.cos(theta), math.sin(theta)

    def g2uv(x, y):                       # DTM CRS -> rotated metric (u along pass, v across)
        e = (x - lon0) * lon_m
        nn = (y - lat0) * lat_m
        return (e * ct + nn * st, -e * st + nn * ct)

    def uv2g(u, v):                       # rotated metric -> DTM CRS
        e = u * ct - v * st
        nn = u * st + v * ct
        return (lon0 + e / lon_m, lat0 + nn / lat_m)

    def elev_uv(u, v):
        gx, gy = uv2g(u, v)
        return dtm.elevation_at(gx, gy)

    poly_uv = Polygon([g2uv(px, py) for px, py in polygon.exterior.coords])
    if not poly_uv.is_valid:
        poly_uv = poly_uv.buffer(0)
    poly_cov = poly_uv          # passes stop at the AOI boundary (rim = operator's job)

    # step / elev_sample_step arrive in map units; convert to metres for the frame.
    to_m = lat_m if is_geo else 1.0
    step_m = max(step * to_m, 0.5)
    esample_m = (elev_sample_step * to_m) if elev_sample_step else None

    _, minv, _, maxv = poly_cov.bounds
    # Inter-pass strip is scanned for hidden peaks at the DTM pixel size, so a
    # narrow spire between passes can't slip between samples and leave a local
    # coverage gap. Sampling finer than a pixel only re-reads the same cells, so
    # the pixel size is both the safe and the cheapest resolution.
    dtm_res_m = min(abs(dtm.src.res[0]), abs(dtm.src.res[1])) * to_m
    strip_res = max(dtm_res_m, 0.5)

    route = []
    v = minv
    toggle = False
    pass_id = 0
    while v <= maxv:
        pts = _sample_line_in_polygon(poly_cov, v, step_m)   # (u, v) in metres
        z = float('nan')
        if pts:
            ordered = list(reversed(pts)) if toggle else pts
            # whole-line altitude (dense for safety): spacing fixed-point + pass alt
            if esample_m and esample_m < step_m:
                elev_pts = _sample_line_in_polygon(poly_cov, v, esample_m)
            else:
                elev_pts = ordered
            valid = [e for e in (elev_uv(pu, pv) for pu, pv in elev_pts)
                     if not math.isnan(e)]
            if valid:
                # altitude tracks the pass MEAN terrain (not the peak); AGL is only
                # nominal on average — true clearance dips below AGL over local highs.
                z = sum(valid) / len(valid) + agl

            # emit only the pass endpoints (the turns) — altitude already came from
            # the dense elev_pts above, so no intermediate waypoints are needed.
            ends = [ordered[0], ordered[-1]] if len(ordered) >= 2 else ordered
            for pu, pv in ends:
                gx, gy = uv2g(pu, pv)
                route.append({'x': gx, 'y': gy, 'z': z,
                              'target_distance': agl, 'pass_id': pass_id})
            pass_id += 1
            toggle = not toggle

        z_cur = z if not math.isnan(z) else None
        s_m = base_spacing_m
        for _ in range(3):
            v_next = v + s_m
            strip_elevs, next_elevs = [], []
            # sample interior lines of the gap densely along-track (one shapely
            # clip per line, count scaled to the gap width so a spire can't hide)
            n_cross = max(3, min(int(s_m / strip_res), 12))
            for k in range(1, n_cross + 1):
                vv = v + s_m * k / (n_cross + 1)
                for pu, pv in _sample_line_in_polygon(poly_cov, vv, strip_res):
                    e = elev_uv(pu, pv)
                    if not math.isnan(e):
                        strip_elevs.append(e)
            for pu, pv in _sample_line_in_polygon(poly_cov, v_next, strip_res):
                e = elev_uv(pu, pv)
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
            spacing_floor = max(min_spacing_m, base_spacing_m * _MIN_TIGHTEN_FRAC)
            s_new = max(min(base_spacing_m, allowed), spacing_floor)
            if abs(s_new - s_m) < 0.25:
                s_m = s_new
                break
            s_m = s_new
        v += s_m

    return route
