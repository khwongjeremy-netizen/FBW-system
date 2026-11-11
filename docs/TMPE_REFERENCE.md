# TMPE physics and variable reference

## Calculation sequence

1. `Aircraft` loads XML mass, engine/rotor inventory, geometry, configuration and citations.
2. `simulate_takeoff()` selects the XML design-gross loading and checks separate rotor models.
3. `vertical_force_balance()` projects rotor thrust upward, adds explicit other loads and subtracts weight.
4. Missing inputs produce unavailable results and prevent time advancement.
5. Known forces feed `integrate_vertical()`, which applies ground support and advances height/velocity.
6. Repeat force evaluation each timestep until the requested duration.
7. Report primary rotor/vertical results; optional secondary wing calculations are excluded from force balance.

## Vertical equations and assumptions

- Aviation gross weight in lb converts to kg with 0.45359237; weight in N is mass times standard gravity 9.80665 m/s².
- Nacelle angle beta is 0° forward and 90° vertical; checked against NASA TM-81244, printed p. 4 / PDF p. 6 (source S14).
- Rotor body force: `(T*cos(beta), 0, -T*sin(beta))`. Vertical-only simulation fixes level attitude and 90° nacelles.
- Up positive: `F = TL*sin(betaL) + TR*sin(betaR) + Fother - m*g`; ground support is separate.
- On the ground at rest, support cancels downward net force. Positive net force permits liftoff.
- Each force is held constant over a step: `h_next = h + v*dt + 0.5*a*dt²`, `v_next = v + a*dt`. Ground crossings are clamped; no impact-response model is claimed.
- No conversion from engine rating to actual rotor thrust is established. The reported transmission relation and OGE download are not applied to invent runway conditions.
- An ideal static hover actuator disk would require explicit induced power and applicable assumptions; that does not supply the missing actual rotor or climb model.

## Frames and equations

- NED: north/east/down. Body: forward/right/down.
- Attitude uses yaw/pitch/roll (3-2-1); world-to-body rotation is the transpose of body-to-world rotation.
- Positive roll lowers the right wing; positive pitch raises the nose; positive yaw turns north toward east.
- Wind is meteorological FROM, clockwise from north: `(-speed*cos(d), -speed*sin(d), 0)`.
- Air-relative velocity describes aircraft motion relative to air; incoming airflow points opposite it.
- Positive wing incidence raises the leading edge. The body-to-wing rotation gives
  `u_w = cos(i)*u_b - sin(i)*w_b`, `w_w = sin(i)*u_b + cos(i)*w_b`.
- Incidence is applied once. In planar forward flight, AoA agrees with pitch + incidence − air-relative flight-path angle.
- Chord-plane speed below `1e-6 m/s` leaves AoA undefined. This is a numerical angle threshold, not a force cutoff.
- `calculate_lift()` contains `L = 0.5 * rho * V**2 * S * CL`, but the live program cannot use it without a supported coefficient model.
- Exact zero freestream gives zero freestream force without evaluating coefficients; rotor wash is excluded.

## Model boundaries

- One reference wing; no panel sweep/dihedral correction, local rotor wash, induced flow or angular-rate flow.
- Airspeed and AoA are kinematics, not evidence of supported force coefficients.
- Reverse flow retains its physical angle, but no reverse-flow aerodynamic model exists.
- Temperature and ground elevation are recorded; density remains an independent input.
- Loaded geometry is not evidence that a corresponding dynamics model exists.
- Supplied hub positions are conversion tilt-axis locations in aircraft FS/BL/WL datum feet (aft/right/up), not CG body coordinates.
- Width is supplied as an exterior range, depth as a cited claim; verification remains incomplete.
- Actual rotor performance, applicable ground/wake loads and aircraft takeoff validation remain future work. Vertical force balance and integration are implemented for explicit known forces.

## Variable dictionary

```text
B. DETAILED VARIABLE WORD BANK
============================================================
Read entries as: variable name = meaning (units, where applicable).
Some short names are reused locally; their meanings depend on the function.
Dictionary keys such as flow['air_world'] use the same definitions below.

+--------------------------------------------------------------------+
| B1. COORDINATES, UNITS AND PHYSICS TERMS
+--------------------------------------------------------------------+
NED / world       = Earth-fixed axes: north, east, down, in that order.
body              = Aircraft-fixed axes: forward, right, down.
wing              = Wing-fixed axes: forward along chord, right, down.
chord             = Line from the wing's leading edge to its trailing edge.
chord plane       = The forward/down (x/z) plane; excludes sideways flow.
AoA / alpha       = Angle of attack, calculated with atan2(down, forward).
incidence         = Fixed wing mounting angle relative to the aircraft body;
                    positive incidence raises the wing's leading edge.
freestream        = Airflow before local rotor wash or induced-flow effects.
rad / deg / °     = Radians / degrees; calculations use radians unless named
                    *_deg, *_degrees, winddirection, or wind_direction.
ft / ft2 / m2     = Feet / square feet / square metres.
CL / CD / Cm      = Lift / drag / pitching-moment coefficients. The XML does
                    not provide a supported model for these coefficients.
None              = Unknown or unavailable value; it does not mean zero.

+--------------------------------------------------------------------+
| B2. FILE LOCATIONS AND CONSTANTS
+--------------------------------------------------------------------+
PROJECT_DIR       = Folder containing this Python file.
DATA_DIR          = Folder containing the aircraft XML data.
G                 = Standard gravitational acceleration (m/s²); unused so far.
FLOW_EPS_MPS      = Minimum chord-plane speed for a defined AoA (m/s).
                    This threshold does not turn small forces into zero.

+--------------------------------------------------------------------+
| B3. XML LOADING
+--------------------------------------------------------------------+
filename          = Name of the XML file to read inside DATA_DIR.
file_path         = Full path to that XML file.
root              = Top-level XML element in the loaded document.
element           = XML element currently being inspected.
name              = Requested XML element's 'name' attribute.
tag_path          = Path through XML tags to the requested element.
attribute         = Optional XML attribute to read instead of element text.
value             = Value being loaded, formatted, or entered by the user.
loader            = Function used to load one aircraft parameter.
args              = Arguments passed to loader in try_load(); in main(),
                    the parsed command-line options instead.
error             = Caught XML, file-reading, or numeric conversion error.

+--------------------------------------------------------------------+
| B4. AIRCRAFT DATA (attributes of aircraft / self)
+--------------------------------------------------------------------+
aircraft          = Aircraft object holding geometry and mass data from XML.
self              = The current Aircraft or Environment object in a method.
design_gross      = Aircraft design gross weight as supplied by XML (lb).
wing_span_ft      = Wingtip-to-wingtip distance (ft).
wing_area_ft2     = Wing reference area (ft²).
wing_area_m2      = The same wing reference area converted to m².
wing_chord_ft     = Wing chord length from XML (ft).
wing_aspect_ratio = Wing span squared divided by wing area (dimensionless).
wing_incidence_deg = Wing mounting angle loaded from XML (degrees).
fuselage_length_ft / fuselage_width_ft / fuselage_height_ft
                  = Aircraft fuselage dimensions (ft); scalar width remains unknown.
rotor_diameter_ft = Diameter of one rotor disk (ft).
rotor_blades_per_hub = Number of blades attached to each rotor hub.
rotor_hub_positions = Supplied tilt-axis locations in FS/BL/WL datum feet;
                      aft/right/up, not CG-centered body coordinates.

+--------------------------------------------------------------------+
| B5. ENVIRONMENT AND PRESCRIBED AIRCRAFT STATE
+--------------------------------------------------------------------+
environment       = Environment object holding the conditions below.
windspeed         = Horizontal wind speed (m/s), never negative.
winddirection     = Direction wind comes FROM, clockwise from north (degrees).
wind_direction    = User-entered value used to set winddirection.
air_density / density = Air density (kg/m³); density is the user input.
temperature       = Air temperature (°C).
altitude          = Ground elevation above sea level (m), not height AGL.
state             = AircraftState object: prescribed velocity and attitude.
ground_velocity_ned = Aircraft velocity over the ground: north/east/down (m/s).
roll / pitch / yaw = Aircraft orientation angles (radians): positive means
                     right wing down / nose up / turning north toward east.
angles            = Roll/pitch/yaw tuple in degrees for report_aoa().

+--------------------------------------------------------------------+
| B6. ROTATIONS AND AIRFLOW CALCULATIONS
+--------------------------------------------------------------------+
cr / sr           = Cosine / sine of roll (dimensionless).
cp / sp           = Cosine / sine of pitch (dimensionless).
cy / sy           = Cosine / sine of yaw (dimensionless).
matrix / rotation = Rotation matrix; rotation changes NED to body coordinates.
row               = One row of a matrix.
vector            = Three-component quantity being rotated or displayed.
a / b             = Matching matrix-row/vector components in transform().
speed             = Wind speed in wind_velocity_ned(); aircraft airspeed
                    magnitude in a flow dictionary (both m/s).
from_degrees      = Wind FROM direction passed to wind_velocity_ned() (degrees).
direction         = The same wind direction converted to radians.
values            = State/wind numbers collected for finite-value validation.
v                 = A local number during validation/formatting; also the
                    wing's sideways velocity when unpacking (u, v, w).
wind              = Wind velocity toward its destination, in NED axes (m/s).
g / w             = Ground/wind components in the air_world subtraction.
                    Elsewhere w means the wing's down-axis airspeed component.
air_world         = Aircraft velocity relative to air, in NED axes (m/s):
                    ground_velocity_ned minus wind. Example: northward ground
                    speed 10 minus northward wind 3 gives airspeed north 7.
body              = That same air-relative velocity in body axes (m/s).
body_alpha        = AoA relative to the BODY forward axis (radians); this
                    does not yet account for wing mounting incidence.
incidence_deg     = Wing mounting angle passed into calculate_aoa() (degrees).
i                 = That wing incidence converted to radians.
wing_rotation     = Matrix changing body coordinates to wing coordinates.
wing              = Air-relative velocity in wing axes (m/s), or None when
                    wing incidence is missing.
u / v / w         = Wing forward / right / down velocity components (m/s).
alpha             = WING AoA including incidence (radians), or None when
                    incidence is missing or chord-plane airflow is too small.
chord_speed       = Magnitude of wing forward/down velocity (m/s).
_                 = An unpacked component that this calculation does not use.
result / flow     = Dictionary of airflow vectors, rotations, speed, AoA,
                    and a status explaining whether wing AoA is available.
                    These velocities describe aircraft motion relative to air;
                    incoming airflow points in the opposite direction.

+--------------------------------------------------------------------+
| B7. AERODYNAMIC RESULTS
+--------------------------------------------------------------------+
aero              = Dictionary containing aerodynamic status and force.
status            = Text explaining availability of AoA or aerodynamic force.
force_body_n      = Wing freestream force tuple in body axes (N); None when
                    unavailable. Exactly zero freestream gives a zero tuple.
regime            = Extra status text for unsupported flow directions.
airspeed          = Aircraft speed relative to air in calculate_lift() (m/s).
lift_coefficient  = Supplied CL value (dimensionless); no verified model yet.
lift              = Display text for wing lift; not a calculated force value.

+--------------------------------------------------------------------+
| B8. DISPLAY, INPUT AND PROGRAM OPTIONS
+--------------------------------------------------------------------+
displayed         = Rounded TEXT made from a number, e.g. '3.14'; used only
                    for output. The original number keeps its full precision.
places            = Number of decimal places to show.
unit              = Unit text appended to a displayed number/vector.
reason            = Explanation shown when a value is unavailable.
label             = Heading/name printed beside a value or matrix.
prompt            = Question shown when requesting user input.
default           = Value used when the user presses Enter without typing.
minimum           = Lowest accepted numeric input, when a limit is supplied.
answer            = User's typed response, with surrounding spaces removed.
suffix            = Prompt text showing a default or an unknown-value option.
parser            = Command-line argument parser.
argv              = Command-line arguments supplied to main(); None uses the
                    arguments with which the program was launched.
verbose           = Option enabled by -v or --verbose (args.verbose).
============================================================
```
