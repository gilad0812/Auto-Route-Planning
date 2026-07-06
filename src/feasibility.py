"""Mission-feasibility / endurance estimate for the Fuse Thor multirotor.

Answers "can this specific drone complete this mission?" as an ENERGY question,
not a flight-dynamics sim. Two computations:

  * flight time  — from the route geometry + drone speed/climb specs (no unknowns);
  * energy       — hover-power physics × time vs the usable battery, with payload as
                   the main input and the propulsion efficiency η the only unknown
                   (so uncalibrated the answer carries a ±band; one real flight pins η).

Plus operational gates (comms range, max work altitude, derated MTOW).

All Fuse Thor numbers are baked in as defaults below (from the manual). The module
is Qt-free and deterministic so it can be unit-tested without a display.
"""
import math
from dataclasses import dataclass, field

_LAT_M = 111139.0
G = 9.80665

# ── Fuse Thor specs (from the manual) ───────────────────────────────────────
NET_KG = 5.5                 # airframe, no battery/payload
BATTERY_KG = 5.9
BATTERY_WH = 1234.0          # 43.2 V × 30 Ah
RH_RESERVE_FRAC = 0.20       # auto return-home trigger at 20% SoC → usable = 80%
MTOW_KG = 20.4               # max AUW at sea level / 20 °C
ROTOR_DIAM_M = 16 * 0.0254   # 16" props
N_ROTORS = 4
DISK_AREA_M2 = N_ROTORS * math.pi * (ROTOR_DIAM_M / 2.0) ** 2
CLIMB_MS = 3.0               # ±3 m/s
CRUISE_MAX_MS = 15.0
MAX_WORK_AGL_M = 400.0
COMMS_RANGE_M = 10_000.0     # 6 dBi omni (mid option)

# Propulsion efficiency (figure of merit) — the one unknown. Uncalibrated we assume
# a typical value with a wide band; a one-flight calibration replaces this.
ETA_DEFAULT = 0.65
ETA_BAND = 0.25              # ±25% until calibrated

TURN_PENALTY_S = 5.0         # decelerate/yaw/accelerate at each turn
TAKEOFF_LAND_S = 40.0        # spin-up + final descent/land maneuvering


def air_density(elev_m, temp_c):
    """ISA barometric air density (kg/m³) at pressure altitude `elev_m` and the
    actual air temperature `temp_c`."""
    p = 101325.0 * (1.0 - 2.25577e-5 * elev_m) ** 5.25588   # pressure at altitude
    return p / (287.05 * (temp_c + 273.15))


def hover_power_w(mass_kg, rho, eta):
    """Momentum-theory induced hover power (W): (m g)^1.5 / (√(2 ρ A) · η)."""
    return (mass_kg * G) ** 1.5 / (math.sqrt(2.0 * rho * DISK_AREA_M2) * eta)


def calibrate_eta(payload_kg, duration_min, battery_used_frac,
                  elev_m=0.0, temp_c=15.0):
    """Solve the propulsion efficiency η from ONE measured steady flight:
    η = ideal_hover_power / measured_avg_power, where measured avg power = energy
    drawn / duration. Returns η (clamped to a sane 0.3–0.95)."""
    m = NET_KG + BATTERY_KG + payload_kg
    rho = air_density(elev_m, temp_c)
    hours = duration_min / 60.0
    if hours <= 0 or battery_used_frac <= 0:
        return ETA_DEFAULT
    avg_power = (BATTERY_WH * battery_used_frac) / hours
    ideal = (m * G) ** 1.5 / math.sqrt(2.0 * rho * DISK_AREA_M2)   # η = 1 power
    return max(0.3, min(0.95, ideal / avg_power))


@dataclass
class FeasibilityResult:
    feasible: bool = False           # nominal energy fits the usable budget
    robust: bool = False             # fits even at the worst-case (low-η) end
    flight_time_s: float = 0.0
    distance_m: float = 0.0
    n_turns: int = 0
    auw_kg: float = 0.0
    air_density: float = 0.0
    site_elev_m: float = 0.0         # operating altitude used for air density
    takeoff_elev_m: float = 0.0      # takeoff ground used for the MTOW gate
    hover_power_w: float = 0.0
    energy_wh: float = 0.0           # nominal
    energy_wh_lo: float = 0.0        # η band
    energy_wh_hi: float = 0.0
    usable_wh: float = 0.0
    margin_pct: float = 0.0          # (usable − nominal)/usable
    batteries_needed: int = 1
    gates: list = field(default_factory=list)   # gate warning strings
    notes: list = field(default_factory=list)


def _route_wps(route):
    return [w for w in route
            if not (isinstance(w.get('z'), float) and math.isnan(w.get('z')))]


def _flight_profile(route, is_geo=True, cruise_ms=6.0, climb_ms=CLIMB_MS,
                    turn_penalty_s=TURN_PENALTY_S, takeoff_land_s=TAKEOFF_LAND_S):
    """Walk the route once → (segments, distance_m, n_turns). `segments` is a list of
    (duration_s, elev_m) energy chunks: one per flown segment (at its mean altitude),
    one per turn (at the turn waypoint's altitude), and one takeoff/land allowance
    (at the first waypoint). Summing durations gives total flight time; evaluating
    hover power at each chunk's altitude gives altitude-resolved energy."""
    wps = _route_wps(route)
    if len(wps) < 2:
        return [(takeoff_land_s, wps[0]['z'] if wps else 0.0)], 0.0, 0
    lat0 = sum(w['y'] for w in wps) / len(wps)
    lon_m = _LAT_M * math.cos(math.radians(lat0)) if is_geo else 1.0
    lat_m = _LAT_M if is_geo else 1.0

    segments, dist, headings = [], 0.0, []
    for a, b in zip(wps, wps[1:]):
        dx = (b['x'] - a['x']) * lon_m
        dy = (b['y'] - a['y']) * lat_m
        horiz = math.hypot(dx, dy)
        dz = abs(b['z'] - a['z'])
        dist += horiz
        seg_s = max(horiz / cruise_ms, dz / climb_ms) if cruise_ms > 0 else 0.0
        segments.append((seg_s, 0.5 * (a['z'] + b['z'])))   # energy at pass altitude
        headings.append(math.atan2(dy, dx) if horiz > 0 else None)

    n_turns = 0
    for (h0, h1), turn_wp in zip(zip(headings, headings[1:]), wps[1:]):
        if h0 is None or h1 is None:
            continue
        d = abs(h1 - h0) % (2 * math.pi)
        d = min(d, 2 * math.pi - d)
        if d > math.radians(30):
            n_turns += 1
            segments.append((turn_penalty_s, turn_wp['z']))
    segments.append((takeoff_land_s, wps[0]['z']))          # spin-up + land near start
    return segments, dist, n_turns


def flight_time(route, is_geo=True, cruise_ms=6.0, climb_ms=CLIMB_MS,
                turn_penalty_s=TURN_PENALTY_S, takeoff_land_s=TAKEOFF_LAND_S):
    """(total_s, distance_m, n_turns). Per segment the drone flies horizontal and
    vertical concurrently, so segment time = max(horiz/cruise, |Δz|/climb); plus a
    penalty per heading change and a fixed takeoff/land allowance."""
    segments, dist, n_turns = _flight_profile(
        route, is_geo=is_geo, cruise_ms=cruise_ms, climb_ms=climb_ms,
        turn_penalty_s=turn_penalty_s, takeoff_land_s=takeoff_land_s)
    return sum(s for s, _ in segments), dist, n_turns


def estimate_feasibility(route, is_geo=True, payload_kg=3.0, cruise_ms=6.0,
                         site_elev_m=0.0, temp_c=15.0,
                         eta=ETA_DEFAULT, eta_band=ETA_BAND,
                         home=None, terrain_at=None, takeoff_elev_m=None):
    """Full feasibility estimate for a flown route. `terrain_at(x,y)->elev` (optional)
    enables the AGL gate; `home=(lon,lat)` (optional) enables the comms-range gate.

    site_elev_m is the OPERATING altitude (drives air density → energy, so pass the
    mean flight altitude AMSL). takeoff_elev_m is the launch ground (drives the MTOW
    derate — takeoff can be at a different elevation than the survey); defaults to
    site_elev_m when not given."""
    r = FeasibilityResult()
    r.auw_kg = NET_KG + BATTERY_KG + payload_kg
    r.site_elev_m = site_elev_m
    r.takeoff_elev_m = site_elev_m if takeoff_elev_m is None else takeoff_elev_m
    r.air_density = air_density(site_elev_m, temp_c)   # representative (mean-alt) ρ

    segments, r.distance_m, r.n_turns = _flight_profile(
        route, is_geo=is_geo, cruise_ms=cruise_ms)
    r.flight_time_s = sum(s for s, _ in segments)

    # Altitude-resolved energy: each chunk burns power at ITS OWN air density (a high
    # pass in thinner air costs more than a low one), so sum energy over the segments
    # instead of flight-mean power × total time. Air is thinner with height; temp is
    # held constant (the ISA lapse is too site-variable to assume — see notes).
    def _energy_wh(e):
        return sum(hover_power_w(r.auw_kg, air_density(z, temp_c), e) * (dur / 3600.0)
                   for dur, z in segments)

    r.hover_power_w = hover_power_w(r.auw_kg, r.air_density, eta)   # representative
    r.energy_wh = _energy_wh(eta)
    # energy ∝ 1/η, so the η band maps to an energy band (low η → worst case)
    r.energy_wh_lo = _energy_wh(eta * (1 + eta_band))
    r.energy_wh_hi = _energy_wh(eta * (1 - eta_band))

    r.usable_wh = BATTERY_WH * (1.0 - RH_RESERVE_FRAC)
    r.feasible = r.energy_wh <= r.usable_wh
    r.robust = r.energy_wh_hi <= r.usable_wh
    r.margin_pct = 100.0 * (r.usable_wh - r.energy_wh) / r.usable_wh
    r.batteries_needed = max(1, math.ceil(r.energy_wh / r.usable_wh))

    # ── operational gates ──
    wps = _route_wps(route)
    # both derates only ever REDUCE MTOW: clamp elevation ≥ 0 so a below-sea-level
    # takeoff (e.g. the Dead Sea, −430 m) can't inflate MTOW past the structural limit
    derated_mtow = MTOW_KG * (1 - 0.05 * max(0.0, r.takeoff_elev_m) / 1000.0) \
        * (1 - 0.05 * max(0.0, temp_c - 20.0) / 10.0)
    if r.auw_kg > derated_mtow:
        r.gates.append(f'Takeoff weight {r.auw_kg:.1f} kg exceeds derated MTOW '
                       f'{derated_mtow:.1f} kg (at {r.takeoff_elev_m:.0f} m / '
                       f'{temp_c:.0f} °C).')
    if terrain_at is not None and wps:
        max_agl = max((w['z'] - terrain_at(w['x'], w['y'])) for w in wps
                      if not math.isnan(terrain_at(w['x'], w['y'])))
        if max_agl > MAX_WORK_AGL_M:
            r.gates.append(f'Max altitude {max_agl:.0f} m AGL exceeds the '
                           f'{MAX_WORK_AGL_M:.0f} m work ceiling.')
    if home is not None and wps:
        lat0 = sum(w['y'] for w in wps) / len(wps)
        lon_m = _LAT_M * math.cos(math.radians(lat0)) if is_geo else 1.0
        lat_m = _LAT_M if is_geo else 1.0
        far = max(math.hypot((w['x'] - home[0]) * lon_m, (w['y'] - home[1]) * lat_m)
                  for w in wps)
        if far > COMMS_RANGE_M:
            r.gates.append(f'Farthest waypoint {far / 1000:.1f} km from home exceeds '
                           f'the {COMMS_RANGE_M / 1000:.0f} km comms range.')
        # RTH-reserve energy: the fraction held back for auto-return-home must cover
        # the flight back from the farthest point (still air, horizontal at cruise).
        # If not, the drone can hit the RH trigger too far out to make it home.
        reserve_wh = BATTERY_WH * RH_RESERVE_FRAC
        return_wh = (r.hover_power_w * (far / cruise_ms) / 3600.0
                     if cruise_ms > 0 else 0.0)
        if return_wh > reserve_wh:
            r.gates.append(f'Return from the farthest point ({far / 1000:.1f} km) needs '
                           f'~{return_wh:.0f} Wh, but the {RH_RESERVE_FRAC * 100:.0f}% '
                           f'auto-return reserve holds only {reserve_wh:.0f} Wh.')

    if eta == ETA_DEFAULT:
        r.notes.append('Uncalibrated η — energy carries a ±%.0f%% band; one measured '
                       'flight pins it.' % (eta_band * 100))
    return r
