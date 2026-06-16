import math
import numpy as np
from shapely.geometry import shape, LineString, Point, Polygon, mapping

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


def _auto_pass_angle(dtm, polygon, lon_m, lat_m, n=21, n_angles=180):
    """Pick the pass heading (radians, in the local metric frame) that best follows
    the terrain contours, by SEARCHING for the orientation that minimises how much
    a single pass climbs or descends.

    Passes run along the rotated frame's u-axis, so a pass is a strip of (near-)
    constant across-axis v. For each candidate heading the sampled terrain is
    binned by v and we measure the mean elevation spread within a bin — i.e. how
    much a pass at that heading would change elevation. The minimising heading
    keeps every pass at near-constant terrain elevation, so the constant-altitude-
    per-pass rule doesn't fly the drone far above a pass's low end (which widens
    the swath and thins point density).

    This replaces a mean-gradient heuristic that CANCELLED OUT on a ridge, valley
    or peak inside the AOI (opposing slopes average to ~zero) and produced passes
    that crossed the slope instead of hugging it. Returns 0.0 (east–west) for flat
    or unreadable terrain.
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


def _seg_max_elev(p0, p1, elev_uv, sample_step):
    """Max terrain elevation along a straight (u,v) segment, densely sampled so a
    peak between the endpoints can't be missed. NaN when no valid terrain."""
    (u0, v0), (u1, v1) = p0, p1
    dist = math.hypot(u1 - u0, v1 - v0)
    n = max(1, int(math.ceil(dist / max(sample_step, 0.5))))
    vals = []
    for i in range(n + 1):
        e = elev_uv(u0 + (u1 - u0) * i / n, v0 + (v1 - v0) * i / n)
        if not math.isnan(e):
            vals.append(e)
    return max(vals) if vals else float("nan")


def _split_by_relief(points, elevs, max_relief):
    """Split an ordered list of pass waypoints into contiguous chunks so each
    chunk's terrain spans no more than `max_relief` metres. Consecutive chunks
    share their boundary waypoint so coverage stays continuous. Each chunk later
    becomes its own straight line at its own constant altitude — the registration-
    safe way to hold low AGL across terrain a single constant-altitude line can't.
    """
    chunks = []
    start = 0
    cmin = cmax = None
    for i in range(len(points)):
        e = elevs[i]
        if e is None or math.isnan(e):
            continue
        nmin = e if cmin is None else min(cmin, e)
        nmax = e if cmax is None else max(cmax, e)
        if cmin is not None and (nmax - nmin) > max_relief and i - start >= 1:
            chunks.append(points[start:i])        # waypoints start … i-1
            start = i - 1                          # next chunk shares the boundary
            be = elevs[i - 1]
            base = be if (be is not None and not math.isnan(be)) else e
            cmin, cmax = min(base, e), max(base, e)
        else:
            cmin, cmax = nmin, nmax
    chunks.append(points[start:])
    return [c for c in chunks if len(c) >= 1]


def plan_route_adaptive(dtm, polygon, distance_above_surface, error_tolerance,
                        scan_half_angle_deg, step,
                        overlap_frac=0.2, is_geo=True, min_spacing_m=2.0,
                        elev_sample_step=None, orientation='auto',
                        edge_margin_m=None, max_relief_per_line_m=None):
    """Plan a lawnmower route with terrain-adaptive pass spacing AND orientation.

    Three terrain/density adaptations:

    1. Orientation — passes run ALONG the terrain contours (perpendicular to the
       mean slope) instead of always east-west. Because altitude is held constant
       per pass at (max terrain under the pass + AGL), a pass that crosses a large
       elevation range forces the drone far above the low end, widening the swath
       and thinning point density there. Contour-aligned passes stay at roughly
       constant terrain elevation, keeping density uniform. `orientation` may be
       'auto' (default, minimise within-pass climb), 'ew', or 'ns'.

    4. Altitude-split lines — when `max_relief_per_line_m` is set, any pass whose
       terrain spans more than that is broken into several contiguous STRAIGHT
       lines, each held at its own constant altitude (registration-safe). This
       keeps the drone at low AGL across terrain a single constant-altitude line
       would otherwise force it high above — without curving the line. Each split
       segment is emitted as its own pass_id ("a new line"). None = never split.

    2. Spacing — tightened wherever terrain between two passes rises and shrinks
       their effective swath, so coverage holds over ridges:
       spacing <= (half_swath_cur + half_swath_next) * (1 - overlap_frac),
       evaluated at the highest terrain in the strip, iterated to a fixed point.

    3. Edge margin — to make the FIRST HELIOS run pass density validation at the
       perimeter (avoiding an expensive second simulation). `edge_margin_m` runs
       the lawnmower this far OUTSIDE the AOI, so the outermost/end-of-pass swaths
       put their dense nadir over the AOI rim instead of only a thin grazing-angle
       tail. Implemented by buffering the survey polygon in the rotated frame.
       Default = ~1.15 pass spacings: just enough to guarantee a full extra pass
       lands beyond every parallel edge (two-sided overlap for the rim) without
       wasted flight. (To raise density everywhere, increase `overlap_frac` —
       spacing scales with 1 − overlap, so overlap is the single density knob.)

    Implementation: everything runs in a local metric frame rotated so passes are
    horizontal; terrain is sampled by mapping back to the DTM CRS, and waypoints
    are emitted back in that CRS (x, y).

    Returns list of dicts: {x, y, z, target_distance, error_tol, pass_id}.
    """
    agl = distance_above_surface
    tan_t = math.tan(math.radians(scan_half_angle_deg))
    half_swath_m = agl * tan_t
    base_spacing_m = 2.0 * half_swath_m * (1.0 - overlap_frac)
    if edge_margin_m is None:
        # Slightly more than one pass spacing: guarantees a full extra pass lands
        # BEYOND each parallel (top/bottom) AOI edge — giving the rim two-sided
        # overlap — rather than the margin falling inside a single pass gap and
        # adding nothing across. (The along-pass extension this also buys is cheap.)
        edge_margin_m = 1.15 * base_spacing_m

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
    # Coverage polygon: the AOI grown by the edge margin so passes run past the
    # rim. Sampling against this puts the dense swath centre over the AOI edge.
    poly_cov = poly_uv.buffer(edge_margin_m) if edge_margin_m > 0 else poly_uv

    # step / elev_sample_step arrive in map units; convert to metres for the frame.
    to_m = lat_m if is_geo else 1.0
    step_m = max(step * to_m, 0.5)
    esample_m = (elev_sample_step * to_m) if elev_sample_step else None

    minu, minv, maxu, maxv = poly_cov.bounds
    strip_us = [minu + (maxu - minu) * i / 12.0 for i in range(13)]

    route = []
    v = minv
    toggle = False
    pass_id = 0
    while v <= maxv:
        pts = _sample_line_in_polygon(poly_cov, v, step_m)   # (u, v) in metres
        z = float('nan')
        if pts:
            ordered = list(reversed(pts)) if toggle else pts
            # Whole-line altitude (dense for safety) — used for the spacing
            # fixed-point below and as the altitude when the line isn't split.
            if esample_m and esample_m < step_m:
                elev_pts = _sample_line_in_polygon(poly_cov, v, esample_m)
            else:
                elev_pts = ordered
            valid = [e for e in (elev_uv(pu, pv) for pu, pv in elev_pts)
                     if not math.isnan(e)]
            if valid:
                z = max(valid) + agl

            # Split the line where its terrain spans more than the relief budget.
            if max_relief_per_line_m:
                we = [elev_uv(pu, pv) for pu, pv in ordered]
                wv = [e for e in we if not math.isnan(e)]
                relief = (max(wv) - min(wv)) if wv else 0.0
                chunks = (_split_by_relief(ordered, we, max_relief_per_line_m)
                          if relief > max_relief_per_line_m else [ordered])
            else:
                chunks = [ordered]

            sstep = esample_m or step_m
            for chunk in chunks:
                if len(chunks) == 1:
                    cz = z
                else:
                    ce = _seg_max_elev(chunk[0], chunk[-1], elev_uv, sstep)
                    cz = (ce + agl) if not math.isnan(ce) else float('nan')
                for pu, pv in chunk:
                    gx, gy = uv2g(pu, pv)
                    route.append({'x': gx, 'y': gy, 'z': cz,
                                  'target_distance': agl, 'error_tol': error_tolerance,
                                  'pass_id': pass_id})
                pass_id += 1
            toggle = not toggle

        z_cur = z if not math.isnan(z) else None
        s_m = base_spacing_m
        for _ in range(3):
            v_next = v + s_m
            strip_elevs, next_elevs = [], []
            for frac in (0.25, 0.5, 0.75):
                vv = v + (v_next - v) * frac
                for u in strip_us:
                    if poly_cov.intersects(Point(u, vv)):
                        e = elev_uv(u, vv)
                        if not math.isnan(e):
                            strip_elevs.append(e)
            for u in strip_us:
                if poly_cov.intersects(Point(u, v_next)):
                    e = elev_uv(u, v_next)
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
        v += s_m

    return route
