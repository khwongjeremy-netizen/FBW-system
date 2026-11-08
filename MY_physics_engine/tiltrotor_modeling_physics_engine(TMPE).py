# ============================================================
# TMPE — Tiltrotor Modeling Physics Engine
# Interactive input and XML loading prototype.
#
# This is an educational prototype, NOT a validated flight simulator.
# It does not yet calculate lift, rotor thrust, aircraft motion, or FBW laws.
# ============================================================

# ============================================================
# QUICK WORD BANK
# ============================================================
# air_world  : Aircraft velocity relative to air, in north/east/down axes.
# body_alpha : Angle of attack relative to the aircraft body (radians).
# alpha      : Angle of attack relative to the wing, including incidence.
# displayed  : Rounded text for printing; calculations keep full precision.
# flow       : Dictionary holding airflow vectors, angles and their status.
# aero       : Dictionary holding force availability and the force result.
# None       : Unknown or unavailable; never a substitute for zero.
#
# BOTTOM REFERENCE GUIDE — search for these headings:
#   A. HOW THE CODE WORKS       Step-by-step calculation and examples.
#   B. DETAILED VARIABLE WORD BANK
#   C. MISSING DATA AND CURRENT ISSUES
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

# ======================================================================
# +--------------------------------------------------------------------+
# | TMPE REFERENCE GUIDE                                                |
# | Read alongside the code; these comments do not run or print.         |
# +--------------------------------------------------------------------+
#
# +--------------------------------------------------------------------+
# | A. HOW THE CODE WORKS                                               |
# +--------------------------------------------------------------------+
#
# 1. LOAD THE AIRCRAFT — Aircraft(), fetchvalue(), fetchpara(), try_load()
#    - Read selected mass, geometry and rotor values from XML.
#    - Convert wing area from ft² to m² when wing_area_m2 is requested.
#    - Keep failed loads as None and print a data warning.
#
# 2. COLLECT THE SCENARIO — main(), Environment, ask_initial_state()
#    - Ask for environmental conditions and prescribed velocity/attitude.
#    - Flight mode and pilot command are labels only.
#    - The state is a snapshot supplied by the user; motion is not simulated.
#
# 3. SUBTRACT WIND — wind_velocity_ned(), calculate_aoa()
#    - Convert the wind's FROM direction into a velocity toward its destination.
#    - air_world = ground_velocity_ned - wind, component by component.
#    - Example: northward ground speed 10 m/s and northward wind 3 m/s
#      give northward air-relative speed 7 m/s.
#    - Airspeed is the length of air_world, including all three components.
#    - Incoming airflow points opposite to this air-relative velocity.
#
# 4. CHANGE THE VIEWING AXES — world_to_body_matrix(), transform()
#    - rotation turns the NED components into aircraft body components.
#    - wing_rotation then accounts for the fixed wing mounting incidence.
#    - air_world -> body -> wing describes the SAME velocity in three frames.
#    - These rotations change its components, not its physical speed.
#
# 5. FIND THE ANGLE — calculate_aoa()
#    - body_alpha = atan2(body down component, body forward component).
#    - alpha = atan2(wing down component, wing forward component).
#    - atan2 preserves forward/reverse-flow quadrants; angles are not clamped.
#    - Example: level northbound flight, calm air, 3° wing incidence:
#      body_alpha is 0° and wing alpha is 3°.
#    - Wing AoA needs known incidence and enough forward/down-plane airflow.
#      Purely sideways airflow can have airspeed but no defined wing AoA.
#
# 6. CHECK FORCE AVAILABILITY — wing_aerodynamics(), calculate_lift()
#    - Exactly zero freestream speed produces a zero freestream force tuple.
#    - Every nonzero speed currently returns an unavailable force result.
#    - calculate_lift() contains L = 0.5 * density * airspeed² * area * CL,
#      but main() does not call it: a supported CL model is still missing.
#    - A valid AoA alone does not establish a valid lift coefficient.
#
# 7. PRINT THE RESULTS — report_results(), format_*(), angle_text()
#    - Convert radians to degrees for readable angle output.
#    - displayed is a string: 3.14159 becomes '3.14' at two decimal places.
#    - Remove a minus sign if rounding produced text such as '-0.00'.
#    - Printing never replaces the original numbers used by calculations.
#    - --verbose adds aircraft/environment details and the calculation trace.
#
# ============================================================
# B. DETAILED VARIABLE WORD BANK
# ============================================================
# Read entries as: variable name = meaning (units, where applicable).
# Some short names are reused locally; their meanings depend on the function.
# Dictionary keys such as flow['air_world'] use the same definitions below.
#
# +--------------------------------------------------------------------+
# | B1. COORDINATES, UNITS AND PHYSICS TERMS
# +--------------------------------------------------------------------+
# NED / world       = Earth-fixed axes: north, east, down, in that order.
# body              = Aircraft-fixed axes: forward, right, down.
# wing              = Wing-fixed axes: forward along chord, right, down.
# chord             = Line from the wing's leading edge to its trailing edge.
# chord plane       = The forward/down (x/z) plane; excludes sideways flow.
# AoA / alpha       = Angle of attack, calculated with atan2(down, forward).
# incidence         = Fixed wing mounting angle relative to the aircraft body;
#                     positive incidence raises the wing's leading edge.
# freestream        = Airflow before local rotor wash or induced-flow effects.
# rad / deg / °     = Radians / degrees; calculations use radians unless named
#                     *_deg, *_degrees, winddirection, or wind_direction.
# ft / ft2 / m2     = Feet / square feet / square metres.
# CL / CD / Cm      = Lift / drag / pitching-moment coefficients. The XML does
#                     not provide a supported model for these coefficients.
# None              = Unknown or unavailable value; it does not mean zero.
#
# +--------------------------------------------------------------------+
# | B2. FILE LOCATIONS AND CONSTANTS
# +--------------------------------------------------------------------+
# PROJECT_DIR       = Folder containing this Python file.
# DATA_DIR          = Folder containing the aircraft XML data.
# G                 = Standard gravitational acceleration (m/s²); unused so far.
# FLOW_EPS_MPS      = Minimum chord-plane speed for a defined AoA (m/s).
#                     This threshold does not turn small forces into zero.
#
# +--------------------------------------------------------------------+
# | B3. XML LOADING
# +--------------------------------------------------------------------+
# filename          = Name of the XML file to read inside DATA_DIR.
# file_path         = Full path to that XML file.
# root              = Top-level XML element in the loaded document.
# element           = XML element currently being inspected.
# name              = Requested XML element's 'name' attribute.
# tag_path          = Path through XML tags to the requested element.
# attribute         = Optional XML attribute to read instead of element text.
# value             = Value being loaded, formatted, or entered by the user.
# loader            = Function used to load one aircraft parameter.
# args              = Arguments passed to loader in try_load(); in main(),
#                     the parsed command-line options instead.
# error             = Caught XML, file-reading, or numeric conversion error.
#
# +--------------------------------------------------------------------+
# | B4. AIRCRAFT DATA (attributes of aircraft / self)
# +--------------------------------------------------------------------+
# aircraft          = Aircraft object holding geometry and mass data from XML.
# self              = The current Aircraft or Environment object in a method.
# design_gross      = Aircraft design gross weight as supplied by XML (lb).
# wing_span_ft      = Wingtip-to-wingtip distance (ft).
# wing_area_ft2     = Wing reference area (ft²).
# wing_area_m2      = The same wing reference area converted to m².
# wing_chord_ft     = Wing chord length from XML (ft).
# wing_aspect_ratio = Wing span squared divided by wing area (dimensionless).
# wing_incidence_deg = Wing mounting angle loaded from XML (degrees).
# fuselage_length_ft / fuselage_width_ft / fuselage_height_ft
#                   = Aircraft fuselage dimensions (ft); width/height unknown.
# rotor_diameter_ft = Diameter of one rotor disk (ft).
# rotor_blades_per_hub = Number of blades attached to each rotor hub.
# rotor_hub_positions = Rotor hub locations; currently unknown, with no frame
#                       or units established in this prototype.
#
# +--------------------------------------------------------------------+
# | B5. ENVIRONMENT AND PRESCRIBED AIRCRAFT STATE
# +--------------------------------------------------------------------+
# environment       = Environment object holding the conditions below.
# windspeed         = Horizontal wind speed (m/s), never negative.
# winddirection     = Direction wind comes FROM, clockwise from north (degrees).
# wind_direction    = User-entered value used to set winddirection.
# air_density / density = Air density (kg/m³); density is the user input.
# temperature       = Air temperature (°C).
# altitude          = Altitude entered by the user (m).
# state             = AircraftState object: prescribed velocity and attitude.
# ground_velocity_ned = Aircraft velocity over the ground: north/east/down (m/s).
# roll / pitch / yaw = Aircraft orientation angles (radians): positive means
#                      right wing down / nose up / turning north toward east.
# velocity          = User-entered ground velocity tuple (m/s).
# angles            = Roll/pitch/yaw tuple; radians in ask_initial_state(),
#                     degrees for display in report_aoa().
# axis              = Name of the velocity or attitude component being entered.
# used_default      = Whether the user chose the stationary, level, north state.
#
# +--------------------------------------------------------------------+
# | B6. ROTATIONS AND AIRFLOW CALCULATIONS
# +--------------------------------------------------------------------+
# cr / sr           = Cosine / sine of roll (dimensionless).
# cp / sp           = Cosine / sine of pitch (dimensionless).
# cy / sy           = Cosine / sine of yaw (dimensionless).
# matrix / rotation = Rotation matrix; rotation changes NED to body coordinates.
# row               = One row of a matrix.
# vector            = Three-component quantity being rotated or displayed.
# a / b             = Matching matrix-row/vector components in transform().
# speed             = Wind speed in wind_velocity_ned(); aircraft airspeed
#                     magnitude in a flow dictionary (both m/s).
# from_degrees      = Wind FROM direction passed to wind_velocity_ned() (degrees).
# direction         = The same wind direction converted to radians.
# values            = State/wind numbers collected for finite-value validation.
# v                 = A local number during validation/formatting; also the
#                     wing's sideways velocity when unpacking (u, v, w).
# wind              = Wind velocity toward its destination, in NED axes (m/s).
# g / w             = Ground/wind components in the air_world subtraction.
#                     Elsewhere w means the wing's down-axis airspeed component.
# air_world         = Aircraft velocity relative to air, in NED axes (m/s):
#                     ground_velocity_ned minus wind. Example: northward ground
#                     speed 10 minus northward wind 3 gives airspeed north 7.
# body              = That same air-relative velocity in body axes (m/s).
# body_alpha        = AoA relative to the BODY forward axis (radians); this
#                     does not yet account for wing mounting incidence.
# incidence_deg     = Wing mounting angle passed into calculate_aoa() (degrees).
# i                 = That wing incidence converted to radians.
# wing_rotation     = Matrix changing body coordinates to wing coordinates.
# wing              = Air-relative velocity in wing axes (m/s), or None when
#                     wing incidence is missing.
# u / v / w         = Wing forward / right / down velocity components (m/s).
# alpha             = WING AoA including incidence (radians), or None when
#                     incidence is missing or chord-plane airflow is too small.
# chord_speed       = Magnitude of wing forward/down velocity (m/s).
# _                 = An unpacked component that this calculation does not use.
# result / flow     = Dictionary of airflow vectors, rotations, speed, AoA,
#                     and a status explaining whether wing AoA is available.
#                     These velocities describe aircraft motion relative to air;
#                     incoming airflow points in the opposite direction.
#
# +--------------------------------------------------------------------+
# | B7. AERODYNAMIC RESULTS
# +--------------------------------------------------------------------+
# aero              = Dictionary containing aerodynamic status and force.
# status            = Text explaining availability of AoA or aerodynamic force.
# force_body_n      = Wing freestream force tuple in body axes (N); None when
#                     unavailable. Exactly zero freestream gives a zero tuple.
# regime            = Extra status text for unsupported flow directions.
# airspeed          = Aircraft speed relative to air in calculate_lift() (m/s).
# lift_coefficient  = Supplied CL value (dimensionless); no verified model yet.
# lift              = Display text for wing lift; not a calculated force value.
#
# +--------------------------------------------------------------------+
# | B8. DISPLAY, INPUT AND PROGRAM OPTIONS
# +--------------------------------------------------------------------+
# displayed         = Rounded TEXT made from a number, e.g. '3.14'; used only
#                     for output. The original number keeps its full precision.
# places            = Number of decimal places to show.
# unit              = Unit text appended to a displayed number/vector.
# reason            = Explanation shown when a value is unavailable.
# label             = Heading/name printed beside a value or matrix.
# prompt            = Question shown when requesting user input.
# default           = Value used when the user presses Enter without typing.
# minimum           = Lowest accepted numeric input, when a limit is supplied.
# answer            = User's typed response, with surrounding spaces removed.
# suffix            = Prompt text showing a default or an unknown-value option.
# parser            = Command-line argument parser.
# argv              = Command-line arguments supplied to main(); None uses the
#                     arguments with which the program was launched.
# verbose           = Option enabled by -v or --verbose (args.verbose).
# flight_mode       = User's scenario label: helicopter/transition/airplane.
# command           = User's pilot-command label; it does not drive a simulation.
# initial           = Display description of the chosen prescribed state.
# ============================================================

# +--------------------------------------------------------------------+
# | C. MISSING DATA AND CURRENT ISSUES                                  |
# +--------------------------------------------------------------------+
#
# C1. MISSING AIRCRAFT FIELDS — explicitly left as None in Aircraft
#    - rotor_hub_positions: hub locations are not supplied/confirmed.
#    - fuselage_width_ft: fuselage width is not supplied/confirmed.
#    - fuselage_height_ft: fuselage height is not supplied/confirmed.
#    - Any XML loading failure can also leave a loaded field as None.
#    - Missing wing incidence prevents wing-frame velocity and wing AoA.
#
# C2. DATA NEEDED FOR SUPPORTED NONZERO WING LIFT
#    - Numerical CL data or a verified coefficient model is missing from
#      the current calculation path. Approximate XML descriptors do not
#      establish a complete supported curve.
#    - Source AoA convention, reference area, velocity definition and
#      component scope must be checked before applying coefficient data.
#    - Actual nacelle/flap/flaperon settings and supported operating ranges
#      need definitions; mode labels do not supply these numeric settings.
#    - CD and Cm models are also missing for drag and pitching moment.
#      They are separate requirements from a supported scalar lift model.
#
# C3. IMPLEMENTATION STILL NEEDED
#    - Load aerodynamic data and evaluate coefficients within valid bounds.
#    - Supply density, area, configuration and coefficient data to the force
#      calculation; wing_aerodynamics() currently receives only flow.
#    - Connect calculate_lift() to the live calculation/reporting path.
#    - Define force directions before reporting nonzero body-axis forces.
#    - Validate coefficient results against supported source reference cases;
#      the current tests check kinematics and existing prototype behavior.
#
# C4. CURRENT MODEL LIMITS
#    - Wind is horizontal; vertical wind is assumed zero.
#    - Wing flow uses one reference wing with incidence only: no sweep,
#      dihedral, panel variation, rotor wash, induced or angular-rate flow.
#    - Rotor-blade AoA, rotor thrust, time integration, command response and
#      flight-by-wire control laws are not implemented.
#    - Temperature and altitude are recorded but do not derive air density.
#    - Aircraft geometry is loaded selectively; loaded data does not imply
#      that a corresponding force or dynamics model exists.
#
# C5. RESULTS THAT CAN LOOK SURPRISING
#    - Default stationary state + wind FROM south produces reverse flow:
#      with the current 3° incidence, wing AoA is approximately -177°.
#      That angle is a kinematic result, not a supported lift prediction.
#    - Undefined AoA, missing coefficients and exactly zero freestream force
#      are different outcomes; None must not be replaced with zero.
#    - A tiny nonzero speed can print as 0.00 m/s and still have unavailable
#      force. Display rounding is not a physical zero-speed test.
#    - Zero freestream force excludes rotor wash; it is not a statement
#      that a hovering tiltrotor has no aerodynamic loads.
#
# Further detail: TMPE_AERODYNAMIC_DATA_GAPS_REPORT.md at the project root.
# +--------------------------------------------------------------------+
# | END OF REFERENCE GUIDE                                             |
# +--------------------------------------------------------------------+
