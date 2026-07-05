"""Mission-feasibility / endurance estimate for the Fuse Thor multirotor.

Answers "can this specific drone complete this mission?" as an ENERGY question,
not a flight-dynamics sim. Two computations:

  * flight time  — from the route geometry + drone speed/climb specs (no unknowns);
  * energy       — hover-power physics × time vs the usable battery, with payload as
                   the main input and the propulsion efficiency η the only unknown
                   (so uncalibrated the answer carries a ±band; one real flight pins η).

Plus operational gates (comms range, max work altitude, derated MTOW, wind).

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
WIND_MAX_KT = 20.0

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


@dataclass
class FeasibilityResult:
    feasible: bool = False           # nominal energy fits the usable budget
    robust: bool = False             # fits even at the worst-case (low-η) end
    flight_time_s: float = 0.0
    distance_m: float = 0.0
    n_turns: int = 0
    auw_kg: float = 0.0
    air_density: float = 0.0
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


def flight_time(route, is_geo=True, cruise_ms=6.0, climb_ms=CLIMB_MS,
                turn_penalty_s=TURN_PENALTY_S, takeoff_land_s=TAKEOFF_LAND_S):
    """(total_s, distance_m, n_turns). Per segment the drone flies horizontal and
    vertical concurrently, so segment time = max(horiz/cruise, |Δz|/climb); plus a
    penalty per heading change and a fixed takeoff/land allowance."""
    wps = _route_wps(route)
    if len(wps) < 2:
        return takeoff_land_s, 0.0, 0
    lat0 = sum(w['y'] for w in wps) / len(wps)
    lon_m = _LAT_M * math.cos(math.radians(lat0)) if is_geo else 1.0
    lat_m = _LAT_M if is_geo else 1.0

    total, dist, headings = 0.0, 0.0, []
    for a, b in zip(wps, wps[1:]):
        dx = (b['x'] - a['x']) * lon_m
        dy = (b['y'] - a['y']) * lat_m
        horiz = math.hypot(dx, dy)
        dz = abs(b['z'] - a['z'])
        dist += horiz
        total += max(horiz / cruise_ms, dz / climb_ms) if cruise_ms > 0 else 0.0
        headings.append(math.atan2(dy, dx) if horiz > 0 else None)

    n_turns = 0
    for h0, h1 in zip(headings, headings[1:]):
        if h0 is None or h1 is None:
            continue
        d = abs(h1 - h0) % (2 * math.pi)
        d = min(d, 2 * math.pi - d)
        if d > math.radians(30):
            n_turns += 1
    total += n_turns * turn_penalty_s + takeoff_land_s
    return total, dist, n_turns


def estimate_feasibility(route, is_geo=True, payload_kg=5.0, cruise_ms=6.0,
                         site_elev_m=0.0, temp_c=15.0, wind_kt=None,
                         eta=ETA_DEFAULT, eta_band=ETA_BAND,
                         home=None, terrain_at=None):
    """Full feasibility estimate for a flown route. `terrain_at(x,y)->elev` (optional)
    enables the AGL gate; `home=(lon,lat)` (optional) enables the comms-range gate."""
    r = FeasibilityResult()
    r.auw_kg = NET_KG + BATTERY_KG + payload_kg
    r.air_density = air_density(site_elev_m, temp_c)

    r.flight_time_s, r.distance_m, r.n_turns = flight_time(
        route, is_geo=is_geo, cruise_ms=cruise_ms)

    r.hover_power_w = hover_power_w(r.auw_kg, r.air_density, eta)
    hours = r.flight_time_s / 3600.0
    r.energy_wh = r.hover_power_w * hours
    # energy ∝ 1/η, so the η band maps to an energy band (low η → worst case)
    r.energy_wh_lo = hover_power_w(r.auw_kg, r.air_density, eta * (1 + eta_band)) * hours
    r.energy_wh_hi = hover_power_w(r.auw_kg, r.air_density, eta * (1 - eta_band)) * hours

    r.usable_wh = BATTERY_WH * (1.0 - RH_RESERVE_FRAC)
    r.feasible = r.energy_wh <= r.usable_wh
    r.robust = r.energy_wh_hi <= r.usable_wh
    r.margin_pct = 100.0 * (r.usable_wh - r.energy_wh) / r.usable_wh
    r.batteries_needed = max(1, math.ceil(r.energy_wh / r.usable_wh))

    # ── operational gates ──
    wps = _route_wps(route)
    derated_mtow = MTOW_KG * (1 - 0.05 * site_elev_m / 1000.0) \
        * (1 - 0.05 * max(0.0, temp_c - 20.0) / 10.0)
    if r.auw_kg > derated_mtow:
        r.gates.append(f'Takeoff weight {r.auw_kg:.1f} kg exceeds derated MTOW '
                       f'{derated_mtow:.1f} kg (at {site_elev_m:.0f} m / {temp_c:.0f} °C).')
    if wind_kt is not None and wind_kt > WIND_MAX_KT:
        r.gates.append(f'Wind {wind_kt:.0f} kt exceeds the {WIND_MAX_KT:.0f} kt limit.')
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

    if eta == ETA_DEFAULT:
        r.notes.append('Uncalibrated η — energy carries a ±%.0f%% band; one measured '
                       'flight pins it.' % (eta_band * 100))
    return r
