"""Mission safety / feasibility checks for a planned route.

The planner holds each pass at a constant altitude (max terrain under the pass +
AGL), so clearance is smallest at the highest terrain *under* a pass. The real
blind spot is the transit between passes: the drone climbs/descends from one
pass altitude to the next while flying over terrain that may rise in between.
These helpers sample terrain clearance along the WHOLE 3D path — every pass
segment AND every connector — so a turn that skims a ridge can't hide.
"""
import math

_LAT_M = 111139.0


def _lon_m(lat):
    return _LAT_M * math.cos(math.radians(lat))


def _valid_wps(route):
    return [wp for wp in route
            if not (isinstance(wp.get('z'), float) and math.isnan(wp['z']))]


def clearance_profile(route, dtm, is_geo=True, sample_m=10.0):
    """Sample terrain clearance (flight altitude - terrain elevation) along the
    full ordered path, densely between waypoints so peaks between them count.

    Returns dict with:
      per_wp   : list of (x, y, z, clearance) at each waypoint
      min_clear: minimum clearance found anywhere along the path (incl. connectors)
      min_at   : (x, y) where that minimum occurs
      n_seg_samples: how many sub-samples were taken
    """
    wps = _valid_wps(route)
    if len(wps) < 2:
        if len(wps) == 1:
            w = wps[0]
            c = w['z'] - dtm.elevation_at(w['x'], w['y'])
            return {'per_wp': [(w['x'], w['y'], w['z'], c)],
                    'min_clear': c, 'min_at': (w['x'], w['y']), 'n_seg_samples': 1}
        return {'per_wp': [], 'min_clear': float('nan'),
                'min_at': None, 'n_seg_samples': 0}

    lat0 = sum(w['y'] for w in wps) / len(wps)
    lon_m = _lon_m(lat0) if is_geo else 1.0
    lat_m = _LAT_M if is_geo else 1.0

    per_wp = []
    for w in wps:
        t = dtm.elevation_at(w['x'], w['y'])
        per_wp.append((w['x'], w['y'], w['z'], w['z'] - t))

    min_clear = float('inf')
    min_at = None
    n = 0
    for a, b in zip(wps, wps[1:]):
        dx = (b['x'] - a['x']) * lon_m
        dy = (b['y'] - a['y']) * lat_m
        dist = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(dist / max(sample_m, 0.5))))
        for k in range(steps + 1):
            f = k / steps
            x = a['x'] + (b['x'] - a['x']) * f
            y = a['y'] + (b['y'] - a['y']) * f
            z = a['z'] + (b['z'] - a['z']) * f          # linear climb/descent
            terr = dtm.elevation_at(x, y)
            if math.isnan(terr):
                continue
            c = z - terr
            n += 1
            if c < min_clear:
                min_clear = c
                min_at = (x, y)
    if min_clear == float('inf'):
        min_clear = float('nan')
    return {'per_wp': per_wp, 'min_clear': min_clear,
            'min_at': min_at, 'n_seg_samples': n}


def mission_safety(route, dtm, is_geo=True, clearance_floor_m=30.0,
                   agl_ceiling_m=120.0, sample_m=10.0):
    """Summarise mission feasibility against safety limits.

    clearance_floor_m : minimum terrain clearance the path must keep everywhere.
    agl_ceiling_m     : regulatory / density AGL ceiling (e.g. 120 m rule).

    Returns dict with the numbers and pass/fail flags, ready for a UI panel.
    """
    prof = clearance_profile(route, dtm, is_geo=is_geo, sample_m=sample_m)
    per = prof['per_wp']
    if not per:
        return {'ok': False, 'reason': 'no valid waypoints',
                'min_clear': float('nan'), 'min_at': None,
                'max_agl': float('nan'), 'n_over_ceiling': 0, 'n_wp': 0,
                'floor': clearance_floor_m, 'ceiling': agl_ceiling_m,
                'floor_ok': False, 'ceiling_ok': False, 'profile': prof}

    agls = [c for *_xyz, c in per]
    max_agl = max(agls)
    n_over = sum(1 for a in agls if a > agl_ceiling_m)
    floor_ok = prof['min_clear'] >= clearance_floor_m
    ceiling_ok = n_over == 0
    return {
        'ok': floor_ok and ceiling_ok,
        'min_clear': prof['min_clear'],
        'min_at': prof['min_at'],
        'max_agl': max_agl,
        'n_over_ceiling': n_over,
        'n_wp': len(per),
        'floor': clearance_floor_m,
        'ceiling': agl_ceiling_m,
        'floor_ok': floor_ok,
        'ceiling_ok': ceiling_ok,
        'profile': prof,
    }
