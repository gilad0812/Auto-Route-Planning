"""RIEGL VUX-120-23 operating envelope, transcribed from the manufacturer
datasheet (2025-10-03). Only numbers the planner/estimator consume live here,
so range limits are cited spec, not tuning knobs.

Max measuring range is per PRR and target reflectivity. We default to the
"natural targets >= 20 %" column — the conservative bound covering vegetation,
dark soil, and wet surfaces (the >= 60 % column is kept for reference). The
datasheet quotes these WITH multiple-time-around (MTA) ambiguity processing
resolved, so this envelope already embeds the MTA constraint — no separate
MTA-zone gate is needed.

Not modelled on purpose: the 5 m minimum range (never reachable at survey AGL)
and the NFB nadir/forward/backward channel geometry (single-fan approximation).
"""

import math

# Max measuring range [m] by pulse repetition rate [Hz] — datasheet p.7.
MAX_RANGE_M_20PCT = {
    150_000: 760.0, 300_000: 550.0, 600_000: 400.0,
    1_200_000: 280.0, 1_800_000: 230.0, 2_400_000: 200.0,
}
MAX_RANGE_M_60PCT = {                      # bright targets — reference only
    150_000: 1260.0, 300_000: 920.0, 600_000: 670.0,
    1_200_000: 480.0, 1_800_000: 400.0, 2_400_000: 350.0,
}

# Scanner mirror line rate limits [lines/sec] — datasheet "Scan Speed 50-400".
SCAN_LINES_MIN_HZ = 50.0
SCAN_LINES_MAX_HZ = 400.0


def max_range_m(pulse_freq_hz, table=None):
    """Max measuring range [m] at the given PRR (>= 20 % reflectivity unless a
    different datasheet column is passed). Linear interpolation between the
    tabulated PRRs; clamped at the table ends."""
    t = MAX_RANGE_M_20PCT if table is None else table
    ks = sorted(t)
    p = float(pulse_freq_hz)
    if p <= ks[0]:
        return t[ks[0]]
    if p >= ks[-1]:
        return t[ks[-1]]
    for a, b in zip(ks, ks[1:]):
        if a <= p <= b:
            f = (p - a) / (b - a)
            return t[a] + f * (t[b] - t[a])
    return t[ks[-1]]                        # unreachable; defensive


def scan_lines_for_square_pattern(pulse_freq_hz, agl_m, scan_half_angle_deg,
                                  speed_ms):
    """Mirror line rate [lines/s] that makes the ground point pattern isotropic
    ("square"): the along-track line spacing equals the across-track point spacing.

    A rotating mirror lays down one scan line per sweep, so the two spacings are
        along-track  = speed / scan_rate                    (successive lines)
        across-track = swath / points-per-line
                     = 2·AGL·tan(half_fov) / (PRR / scan_rate)
    Setting them equal and solving for scan_rate gives
        scan_rate = sqrt( speed · PRR / (2 · AGL · tan(half_fov)) ).

    This replaces the old fixed 600 kHz→224.4 Hz anchor: scan rate is no longer a
    chosen nominal but follows the flight geometry (AGL, speed, FOV) and the pulse
    rate, grounded in the isotropic-sampling criterion. Clamped to the scanner's
    physical mirror range (datasheet 50–400 lines/s); a combination that would need
    a rate outside that window is pinned to the bound, so the pattern is as square
    as the hardware allows rather than silently invalid.
    """
    tan_h = math.tan(math.radians(scan_half_angle_deg))
    denom = 2.0 * max(float(agl_m), 1.0) * max(tan_h, 1e-6)
    rate = math.sqrt(max(float(speed_ms), 1e-6) * float(pulse_freq_hz) / denom)
    return min(max(rate, SCAN_LINES_MIN_HZ), SCAN_LINES_MAX_HZ)
