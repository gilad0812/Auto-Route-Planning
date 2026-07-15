"""Locks the invariant behind _auto_pass_angle: the chosen pass heading follows
the terrain contours, i.e. it minimizes how much each pass climbs.

Runs standalone (`python tests/test_pass_angle.py`, non-zero exit on failure) and
under pytest if that ever gets added. Uses an analytic fake DTM so it needs no
raster files — fast and deterministic. Terrains have a KNOWN contour direction, so
the expected heading is not derived from the code under test.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from shapely.geometry import Polygon                      # noqa: E402
from route_planner import _auto_pass_angle                # noqa: E402


class FakeDTM:
    """Minimal stand-in: _auto_pass_angle only calls elevation_at(x, y)."""
    def __init__(self, z_fn):
        self._z = z_fn

    def elevation_at(self, x, y):
        return self._z(x, y)


# Projected metric frame (lon_m = lat_m = 1); AOI is a centred square.
BOX = Polygon([(-100, -100), (100, -100), (100, 100), (-100, 100)])


def _axis_diff_deg(a_rad, b_rad):
    """Smallest angle between two pass AXES (bidirectional, mod pi), in degrees."""
    d = abs(a_rad - b_rad) % math.pi
    return math.degrees(min(d, math.pi - d))


def _within_pass_spread(z_fn, theta, spacing=80.0, n=61):
    """Independent metric: mean within-pass elevation spread for straight parallel
    passes of width `spacing` at heading `theta`. Does NOT reuse the code's binning."""
    import numpy as np
    xs = np.linspace(-100, 100, n)
    X, Y = np.meshgrid(xs, xs)
    Z = np.vectorize(z_fn)(X, Y)
    v = -X * math.sin(theta) + Y * math.cos(theta)     # across-pass coordinate
    idx = np.floor((v - v.min()) / spacing).astype(int)
    cost = weight = 0.0
    for b in np.unique(idx):
        zz = Z.ravel()[idx.ravel() == b]
        if zz.size >= 2:
            cost += float(zz.max() - zz.min()) * zz.size
            weight += zz.size
    return cost / weight if weight else float('inf')


# (terrain, expected pass-axis heading in degrees, human name)
CASES = [
    (lambda x, y: 0.2 * x, 90.0, 'rises east -> contours run N-S'),
    (lambda x, y: 0.2 * y, 0.0, 'rises north -> contours run E-W'),
    (lambda x, y: 0.15 * (x + y), 135.0, 'rises NE -> contours run NW-SE'),
    (lambda x, y: -0.05 * abs(x), 90.0, 'N-S ridge crest -> contours run N-S'),
]


def _run_case(z_fn, expected_deg, name):
    theta = _auto_pass_angle(FakeDTM(z_fn), BOX, 1.0, 1.0)
    # (1) heading matches the analytic contour direction (~1 sampling bin tolerance)
    err = _axis_diff_deg(theta, math.radians(expected_deg))
    assert err <= 6.0, f'{name}: heading {math.degrees(theta):.1f} deg vs ' \
                       f'expected {expected_deg:.1f} deg (off by {err:.1f})'
    # (2) the chosen heading really climbs less than flying across the contours
    s_auto = _within_pass_spread(z_fn, theta)
    s_perp = _within_pass_spread(z_fn, theta + math.pi / 2)
    assert s_auto <= s_perp + 1e-6, \
        f'{name}: spread @auto {s_auto:.2f} > @perp {s_perp:.2f}'
    return theta, s_auto, s_perp


def test_auto_pass_angle_follows_contours():
    for z_fn, expected_deg, name in CASES:
        _run_case(z_fn, expected_deg, name)


if __name__ == '__main__':
    ok = True
    for z_fn, expected_deg, name in CASES:
        try:
            theta, s_auto, s_perp = _run_case(z_fn, expected_deg, name)
            print(f'PASS  {name:38s} heading={math.degrees(theta):5.1f} deg  '
                  f'spread auto={s_auto:5.2f} perp={s_perp:5.2f}')
        except AssertionError as e:
            ok = False
            print(f'FAIL  {e}')
    sys.exit(0 if ok else 1)
