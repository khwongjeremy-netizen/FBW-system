#
#     try:
#         generate_report(aircraft, user_input)
#     except ValueError as error:
#         print(f"\n[CALCULATION ERROR] {error}")
#
#
# if __name__ == "__main__":
#     main()

# ============================================================
# TMPE — Tiltrotor Modeling Physics Engine
# Interactive input and XML loading prototype.
#
# This is an educational prototype, NOT a validated flight simulator.
# It does not yet calculate lift, rotor thrust, aircraft motion, or FBW laws.
# ============================================================

from pathlib import Path
import argparse
import xml.etree.ElementTree as ET
import math
from dataclasses import dataclass

PROJECT_DIR = Path(__file__).resolve().parent
DATA_DIR = PROJECT_DIR / "data(v.2)"
G = 9.80665  # m/s²


# ============================================================
# 1. XML DATA LOADING
# ============================================================

def fetchvalue(filename, name):
    """Load a numeric XML element identified by its name attribute."""
    file_path = DATA_DIR / filename
    root = ET.parse(file_path).getroot()

    for element in root.iter():
        if element.get("name") == name:
            if element.text is None or not element.text.strip():
                raise ValueError(f"XML value '{name}' is empty.")
            return float(element.text.strip())

    raise ValueError(f"XML parameter '{name}' was not found in {filename}.")


def fetchpara(filename, tag_path, attribute=None):
    """Load a value from an XML tag path, optionally reading an attribute."""
    file_path = DATA_DIR / filename
    root = ET.parse(file_path).getroot()
    element = root.find(tag_path)

    if element is None:
        raise ValueError(f"XML tag '{tag_path}' was not found in {filename}.")

    value = element.get(attribute) if attribute is not None else element.text
    if value is None or not value.strip():
        raise ValueError(f"No value found at '{tag_path}' in {filename}.")

    value = value.strip()
    try:
        return float(value)
    except ValueError:
        return value


def try_load(loader, *args):
    """Load a parameter if possible; otherwise keep it unknown and warn."""
    try:
        return loader(*args)
    except (OSError, ET.ParseError, ValueError) as error:
        print(f"[DATA WARNING] Could not load {args}: {error}")
        return None


# ============================================================
# 2. AIRCRAFT PARAMETERS
# ============================================================

class Aircraft:
    """Selected XV-15 parameters loaded from XML."""

    def __init__(self):
        self.design_gross = try_load(fetchvalue, "mass.xml", "design_gross")

        self.wing_span_ft = try_load(
            fetchpara, "geometry.xml", "geometry/wing/span"
        )
        self.wing_area_ft2 = try_load(
            fetchpara, "geometry.xml", "geometry/wing/area"
        )
        self.wing_chord_ft = try_load(
            fetchpara, "geometry.xml", "geometry/wing/chord"
        )
        self.wing_aspect_ratio = try_load(
            fetchpara, "geometry.xml", "geometry/wing/aspect_ratio"
        )
        self.wing_incidence_deg = try_load(
            fetchpara, "geometry.xml", "geometry/wing/incidence"
        )
        self.fuselage_length_ft = try_load(
            fetchpara, "geometry.xml", "geometry/fuselage", "length"
        )
        self.rotor_diameter_ft = try_load(
            fetchpara, "rotor.xml",
            "proprotor_and_drive_system/rotor_diameter"
        )
        self.rotor_blades_per_hub = try_load(
            fetchpara, "rotor.xml",
            "proprotor_and_drive_system/rotor_blades_per_hub"
        )

        # Not supplied/confirmed in the current data framework:
        self.rotor_hub_positions = None
        self.fuselage_width_ft = None
        self.fuselage_height_ft = None

    @property
    def wing_area_m2(self):
        """Convert square feet to square metres for SI calculations."""
        if self.wing_area_ft2 is None:
            return None
        return self.wing_area_ft2 * 0.09290304


# ============================================================
# 3. ENVIRONMENT AND USER INPUT
# ============================================================

class Environment:
    def __init__(
        self,
        windspeed=5.0,
        winddirection=180.0,
        air_density=1.225,
        temperature=15.0,
        altitude=0.0,
    ):
        self.windspeed = windspeed          # m/s
        self.winddirection = winddirection  # degrees FROM, clockwise from north
        self.air_density = air_density      # kg/m³
        self.temperature = temperature      # °C
        self.altitude = altitude            # m


# World: North/East/Down (NED), m/s. Body: forward/right/down.
# Right-handed intrinsic yaw-pitch-roll (3-2-1); positive yaw turns north
# toward east, pitch raises the nose, roll lowers the right wing. Radians.
# This is prescribed initial state, not the response to a pilot command.
FLOW_EPS_MPS = 1e-6  # Below this chord-plane speed AoA is unavailable.


@dataclass(frozen=True)
class AircraftState:
    ground_velocity_ned: tuple = (0.0, 0.0, 0.0)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


def world_to_body_matrix(state):
    """Transpose of the active body-to-NED yaw-pitch-roll rotation."""
    cr, sr = math.cos(state.roll), math.sin(state.roll)
    cp, sp = math.cos(state.pitch), math.sin(state.pitch)
    cy, sy = math.cos(state.yaw), math.sin(state.yaw)
    return (
        (cp * cy, cp * sy, -sp),
        (sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp),
        (cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp),
    )


def transform(matrix, vector):
    return tuple(sum(a * b for a, b in zip(row, vector)) for row in matrix)


def wind_velocity_ned(speed, from_degrees):
    """Horizontal meteorological wind FROM clockwise from true north.

    E.g. FROM north yields air moving south, a negative north component.
    Vertical wind is assumed zero; direction is immaterial at zero speed.
    """
    direction = math.radians(from_degrees)
    return (-speed * math.cos(direction), -speed * math.sin(direction), 0.0)


def calculate_aoa(state, environment, incidence_deg):
    """Freestream AoA from prescribed motion, orientation and wing geometry.

    AoA is the chord/relative-airflow angle, not an environmental or fixed
    aircraft parameter. Wing AoA is distinct from rotor-blade AoA, nacelle
    tilt, blade pitch and aircraft attitude. Incoming flow is -V_air.
    Wing x is forward along the chord, y right, z down. Positive incidence
    raises the leading edge: body-to-wing = Ry(incidence). Sweep, dihedral,
    panel variation, rotor wash, induced flow and local angular-rate flow
    are excluded from this single reference-wing model.
    atan2(w, u) preserves quadrants and positive nose-up AoA; no clamping.
    """
    values = (*state.ground_velocity_ned, state.roll, state.pitch, state.yaw,
              environment.windspeed, environment.winddirection)
    if not all(math.isfinite(v) for v in values) or environment.windspeed < 0:
        raise ValueError("State and wind must be finite, with non-negative wind speed.")
    wind = wind_velocity_ned(environment.windspeed, environment.winddirection)
    air_world = tuple(g - w for g, w in zip(state.ground_velocity_ned, wind))
    rotation = world_to_body_matrix(state)
    body = transform(rotation, air_world)
    body_alpha = (math.atan2(body[2], body[0])
                  if math.hypot(body[0], body[2]) >= FLOW_EPS_MPS else None)
    result = dict(wind=wind, air_world=air_world, rotation=rotation,
                  body=body, body_alpha=body_alpha, speed=math.hypot(*air_world),
                  wing=None, alpha=None, wing_rotation=None)
    if incidence_deg is None:
        result['status'] = 'unavailable: wing incidence missing'
        return result
    if not math.isfinite(incidence_deg):
        raise ValueError("Wing incidence must be finite.")
    i = math.radians(incidence_deg)
    wing_rotation = ((math.cos(i), 0.0, -math.sin(i)),
                     (0.0, 1.0, 0.0),
                     (math.sin(i), 0.0, math.cos(i)))
    wing = transform(wing_rotation, body)
    result.update(wing=wing, wing_rotation=wing_rotation)
    if math.hypot(wing[0], wing[2]) < FLOW_EPS_MPS:
        result['status'] = 'undefined: insufficient chord-plane airflow'
    else:
        result.update(alpha=math.atan2(wing[2], wing[0]), status='available')
    return result


def wing_aerodynamics(flow):
    """Separate kinematic AoA validity from coefficient availability.

    No supported CL/CD/Cm tables exist in the supplied XML. Do not turn
    approximate curve descriptors into a force model. Exact zero freestream
    speed gives zero freestream force without evaluating any coefficient;
    the AoA numerical threshold is not a force cutoff.
    """
    if flow['speed'] == 0.0:
        return dict(status='zero freestream force; rotor wash excluded',
                    force_body_n=(0.0, 0.0, 0.0))
    regime = ''
    if flow['wing'] is not None and flow['wing'][0] <= FLOW_EPS_MPS:
        regime = '; unsupported reverse/near-normal/spanwise flow'
    return dict(status=('unavailable: CL/CD/Cm coefficient model missing; '
                        + ('wing AoA ' + flow['status']) + regime), force_body_n=None)


def format_number(value, places=2):
    """Round only the display string; remove a minus sign on displayed zero."""
    displayed = f"{value:.{places}f}"
    return displayed[1:] if displayed.startswith('-') and float(displayed) == 0 else displayed


def format_quantity(value, unit, places=2, reason="data missing"):
    if value is None:
        return f"Unavailable — {reason}"
    return f"{format_number(value, places)}{unit if unit == '°' else ' ' + unit}"


def format_vector(vector, unit="m/s", places=2, reason="wing incidence missing"):
    if vector is None:
        return f"Unavailable — {reason}"
    return f"({', '.join(format_number(v, places) for v in vector)}) {unit}"


def angle_text(angle, reason="insufficient chord-plane airflow"):
    return format_quantity(None if angle is None else math.degrees(angle),
                           "°", 1, reason)


def flow_condition(flow):
    """Describe flow for display using full-precision values, not rounded text."""
    if flow['speed'] == 0.0:
        return "No freestream airflow"
    if flow['wing'] is None:
        return "Unavailable — wing incidence missing"
    u, v, _ = flow['wing']
    if flow['alpha'] is None:
        return "Spanwise flow" if abs(v) >= FLOW_EPS_MPS else "Insufficient chord-plane airflow"
    if u < -FLOW_EPS_MPS:
        return "Reverse flow"
    if u <= FLOW_EPS_MPS:
        return "Near-normal flow"
    return "Forward flow"


def report_results(flow, aero):
    print("\n--- Results ---")
    print(f"{'Airspeed:':<19}{format_quantity(flow['speed'], 'm/s')}")
    reason = "wing incidence missing" if flow['wing'] is None else "insufficient chord-plane airflow"
    print(f"{'Wing AoA:':<19}{angle_text(flow['alpha'], reason)}")
    # Zero freestream force is already established by the aerodynamic model.
    # Never infer zero lift from a speed that merely rounds to 0.00.
    lift = "0.00 N" if aero['force_body_n'] == (0.0, 0.0, 0.0) else "Unavailable — aerodynamic model missing"
    print(f"{'Wing lift:':<19}{lift}")
    print(f"{'Flow condition:':<19}{flow_condition(flow)}")


def report_matrix(label, matrix):
    print(f"{label} (dimensionless):")
    if matrix is None:
        print("  Unavailable — wing incidence missing")
    else:
        for row in matrix:
            print("  [" + "  ".join(f"{format_number(v, 6):>9}" for v in row) + "]")


def report_aoa(state, aircraft, flow):
    """Verbose trace only; display rounding never feeds back into calculations."""
    print("\n--- Freestream AoA calculation trace ---")
    print("Prescribed ground velocity NED:", format_vector(state.ground_velocity_ned))
    print("Wind: meteorological FROM, clockwise from north; vertical wind = 0.00 m/s")
    print("Wind velocity NED:", format_vector(flow['wind']))
    print("V_air_NED = V_ground_NED - V_wind_NED:")
    print(f"  {format_vector(state.ground_velocity_ned)} - {format_vector(flow['wind'])}")
    print("  =", format_vector(flow['air_world']))
    print("Incoming airflow = -V_air; airspeed:", format_quantity(flow['speed'], 'm/s'))
    angles = tuple(math.degrees(v) for v in (state.roll, state.pitch, state.yaw))
    print("Prescribed roll/pitch/yaw:", format_vector(angles, 'deg', 1))
    report_matrix("R_world_to_body = transpose(R_body_to_NED)", flow['rotation'])
    print("V_air_body = R_world_to_body @ V_air_NED:", format_vector(flow['body']))
    print("Body AoA:", angle_text(flow['body_alpha']))
    print("Wing incidence from XML:", format_quantity(aircraft.wing_incidence_deg, '°', 1,
                                                     'wing incidence missing'))
    report_matrix("R_body_to_wing (reference wing; incidence only)", flow['wing_rotation'])
    print("V_air_wing = R_body_to_wing @ V_air_body:", format_vector(flow['wing']))
    if flow['alpha'] is None:
        reason = "wing incidence missing" if flow['wing'] is None else "insufficient chord-plane airflow"
        print("Wing AoA:", angle_text(None, reason))
    else:
        u, _, w = flow['wing']
        print(f"Wing chordwise u: {format_quantity(u, 'm/s')}; normal w: {format_quantity(w, 'm/s')}")
        if format_number(u) == format_number(w) == '0.00':
            # Display-only positive scaling keeps tiny, valid atan2 inputs
            # readable without changing velocity units or computed AoA.
            chord_speed = math.hypot(u, w)
            print("For readable atan2 inputs, normalize by chord-plane speed (dimensionless):")
            print(f"  u/|V_xz| = {format_number(u / chord_speed, 6)}, w/|V_xz| = {format_number(w / chord_speed, 6)}")
            print(f"Wing AoA = atan2(w/|V_xz|, u/|V_xz|) = atan2({format_number(w / chord_speed, 6)}, {format_number(u / chord_speed, 6)}) = {angle_text(flow['alpha'])}")
        else:
            print(f"Wing AoA = atan2(w, u) = atan2({format_number(w)}, {format_number(u)}) = {angle_text(flow['alpha'])}")
    print(f"AoA unavailable when chord-plane speed < {FLOW_EPS_MPS:.1e} m/s.")
    print("Trace values are rounded for display; calculations use full precision.")


def report_details(environment, aircraft, aero):
    print("\n--- Environment and aircraft data ---")
    for label, value in (
        ("Air density", format_quantity(environment.air_density, 'kg/m³')),
        ("Wind speed", format_quantity(environment.windspeed, 'm/s')),
        ("Wind FROM direction", format_quantity(environment.winddirection, '°', 1)),
        ("Temperature", format_quantity(environment.temperature, '°C', 1)),
        ("Altitude", format_quantity(environment.altitude, 'm')),
        ("Design gross", format_quantity(aircraft.design_gross, 'lb')),
        ("Wing span", format_quantity(aircraft.wing_span_ft, 'ft')),
        ("Wing area", format_quantity(aircraft.wing_area_ft2, 'ft²')),
        ("Wing area (SI)", format_quantity(aircraft.wing_area_m2, 'm²')),
        ("Rotor diameter", format_quantity(aircraft.rotor_diameter_ft, 'ft')),
        ("Rotor blades/hub", format_quantity(aircraft.rotor_blades_per_hub, 'blades', 0)),
        ("Wing freestream force (body)", format_vector(aero['force_body_n'], 'N',
                                                     reason='aerodynamic model missing')),
    ):
        print(f"{label}: {value}")


def ask_text(prompt, default):
    answer = input(f"{prompt} [{default}]: ").strip()
    return answer if answer else default


def ask_float(prompt, default=None, minimum=None, places=2):
    while True:
        suffix = f" [{format_number(default, places)}]" if default is not None else " [Enter to leave unknown]"
        answer = input(f"{prompt}{suffix}: ").strip()

        if not answer:
            return default

        try:
            value = float(answer)
        except ValueError:
            print("Please enter a valid number.")
            continue

        if not math.isfinite(value):
            print("Please enter a finite number.")
            continue

        if minimum is not None and value < minimum:
            print(f"Please enter a value of at least {minimum}.")
            continue

        return value


def ask_initial_state():
    while True:
        answer = input("Use default initial state: stationary, level, facing north? [Y/n] ").strip().lower()
        if answer in ('', 'y', 'yes'):
            return AircraftState(), True
        if answer in ('n', 'no'):
            break
        print("Please enter y or n.")

    velocity = tuple(ask_float(f"Initial ground velocity {axis} (m/s)", default=0.0)
                     for axis in ("north", "east", "down (negative = climb)"))
    angles = tuple(math.radians(ask_float(f"Initial {axis} (degrees)", default=0.0, places=1))
                   for axis in ("roll (right wing down positive)", "pitch (nose up positive)",
                                "yaw (clockwise from north)"))
    return AircraftState(velocity, *angles), False


def calculate_lift(air_density, airspeed, wing_area_m2, lift_coefficient):
    """Calculate wing lift when supplied with model-derived inputs.

    L = 0.5 * rho * V^2 * S * CL
    The interactive program does not call this until a verified CL model
    and an airspeed source are available.
    """
    if None in (air_density, airspeed, wing_area_m2, lift_coefficient):
        return None

    if air_density < 0 or airspeed < 0 or wing_area_m2 <= 0:
        raise ValueError("Density and airspeed must be non-negative; wing area must be positive.")

    return 0.5 * air_density * airspeed**2 * wing_area_m2 * lift_coefficient


# ============================================================
# 4. MAIN PROGRAM — CONNECT THE COMPONENTS
# ============================================================

def main(argv=None):
    parser = argparse.ArgumentParser(description="TMPE — Tiltrotor Modeling Physics Engine")
    parser.add_argument('-v', '--verbose', action='store_true',
                        help="show aircraft data and the complete AoA calculation trace")
    args = parser.parse_args(argv)
    print("\nTMPE — Tiltrotor Modeling Physics Engine")

    # 1. Load aircraft parameters from XML.
    aircraft = Aircraft()

    # 2. Collect mode and command labels. These do not drive a flight model.
    flight_mode = ask_text(
        "Flight mode (helicopter / transition / airplane)",
        "helicopter"
    ).lower()

    command = ask_text(
        "Pilot command (label only; e.g. hover / takeoff / climb / move_forward / land)",
        "hover"
    ).lower()

    # 3. Collect environmental conditions.
    print("\nEnter environmental conditions.")

    density = ask_float(
        "Air density (kg/m^3)",
        default=1.225,
        minimum=0
    )

    windspeed = ask_float(
        "Wind speed (m/s)",
        default=5.0,
        minimum=0
    )

    wind_direction = ask_float(
        "Wind FROM direction (degrees clockwise from north)",
        default=180.0,
        places=1
    )

    temperature = ask_float(
        "Temperature (degrees C)",
        default=15.0,
        places=1
    )

    altitude = ask_float(
        "Altitude (m)",
        default=0.0
    )

    environment = Environment(
        windspeed=windspeed,
        winddirection=wind_direction,
        air_density=density,
        temperature=temperature,
        altitude=altitude
    )

    state, used_default = ask_initial_state()
    flow = calculate_aoa(state, environment, aircraft.wing_incidence_deg)
    aero = wing_aerodynamics(flow)

    report_results(flow, aero)
    print(f"\nScenario labels: {flight_mode} / {command} (no simulated command response).")
    initial = "stationary, level, facing north" if used_default else "custom velocity and attitude"
    print(f"State: {initial} (prescribed). Wind is horizontal; freestream wing model only.")
    print("Local rotor wash, induced and rotational flow excluded; rotor thrust, dynamics and FBW unimplemented.")
    print("Educational prototype; not a validated flight simulator.")
    if args.verbose:
        report_details(environment, aircraft, aero)
        report_aoa(state, aircraft, flow)


if __name__ == "__main__":
    main()
