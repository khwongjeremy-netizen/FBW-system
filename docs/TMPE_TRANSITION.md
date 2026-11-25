# TMPE Phase 2 timed transition

Phase 2 starts at the last Phase 1 state. Altitude is above ground, up velocity
carries over, and forward distance and speed start at zero because Phase 1 is
vertical. An airborne Phase 1 result is required. Each stage has its own positive
duration prompt, with equivalent CLI flags for scripted runs. The shared
`--dt` defaults to 0.1 seconds. A final step can be shorter.

```sh
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' \
  --rotor-model prototype-fixed-ct --duration 5 --transition-duration 2 \
  --transition-end 45:45 --transition-pitch-deg 0
```

The example selects the fixed-CT software-test model. The illustrative
collective fixture also requires explicit opt-in. Left and right nacelles each
ramp linearly from their Phase 1 angles to the requested final angles (default
`0:0`). The range 0–90° is a software input range. Enter a wind FROM bearing
of 0° for a headwind in this north-facing example. The Phase 1 exit velocity
and wind determine the early Phase 2 flow. `--transition-pitch-deg` holds an
explicit nose-up-positive body pitch for the AoA calculation. With the
bundled rotor model the NASA path reports the first evaluated flow and the
missing rotor inputs before it can advance.
For a software-only run of the older constant-coefficient demonstration, add
`--illustrative-transition`.

The separate
`--nacelle-schedule 90:90 45:45 0:0` option still prints untimed geometric
samples. Those samples leave the timed schedule and state untouched.

## NASA CR-166536 wing-pylon coefficients

Phase 2 defaults to the freestream wing-pylon model. The source file holds
dimensionless `C_LWP` and `C_DWP` values in separate table columns, each
with its NASA table, printed page, mast angle, flap/flaperon pair and Mach
interval or exact Mach. Each column maps `alpha_W` in degrees to the
transcribed coefficient or `None` for unavailable cells. The 20 supplied
screenshots cover pp. B-32–B-51. The lift and drag coefficients come from
Tables 4-I–4-IV, pp. B-34–B-44. The other screenshots cover stability
derivatives, spinner and wake constants, stabilizer downwash, moments and
zero-lift slopes. The report describes wind-tunnel-based coefficients up to
stall and approximations above stall (pp. 26–27).
[Source data](../MY_physics_engine/data%28v.2%29/xv15_wing_pylon_data.py) ·
[NASA CR-166536](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf)

- **Mast angle.** TMPE uses 90° for vertical nacelles and 0° for forward.
  The report's `beta_m` runs in the opposite direction, so
  `beta_m_deg = 90 - nacelle_deg` (p. A-50). A single symmetric mast angle
  is required. The report calls for interpolation between the endpoint mast
  angles (p. 25). TMPE linearly interpolates there only when both endpoint
  configurations support the query, and labels the result interpolated.
  Add `--transition-mast-endpoint-fallback` to allow a mast-angle estimate
  when an interior lookup cannot interpolate. The usable endpoint must
  supply both CL and CD at the requested AoA, flap setting and Mach, and
  lie within 5° by default. Pass a value such as
  `--transition-mast-endpoint-fallback 3` to change the maximum gap in degrees.
- **Angle of attack.** TMPE calculates fuselage AoA as `alpha_F = pitch - gamma`.
  With no pitch input, fuselage AoA is unavailable on the NASA path. The
  freestream wing AoA subtracts the rotor-flow correction of NASA CR-166536
  p. 25, Eq. 57.3, directly from fuselage AoA. The correction in radians is
  `K_XRW * x_R/W * (C_RFR + C_RFL) / max(mu, 0.15)^2`.
  TMPE uses the left/right mean advance ratio and
  `x_R/W = X_RW0 + beta_m * (X_RW1 + beta_m * X_RW2)` from p. A-70,
  with mast angle in degrees and the constants from p. B-33. The correction
  is converted to degrees for the lookup. This angle definition has no
  separate wing-incidence term.
- **Rotor-flow inputs.** Each `C_RF` is the resultant mast-axis rotor force
  divided by `rho * pi * effective_speed^2 * R^4` (pp. A-34–A-38). Each
  advance ratio uses air-relative hub speed in the mast plane divided by
  effective tip speed (pp. A-18–A-20). The prototype's axial thrust and
  design RPM do not supply these operating-state quantities. Python callers
  can pass `rotor_flow_provider(state, environment)` to `simulate_transition()`
  to return a sourced `XV15RotorFlow` for each step. If a required AoA input
  is missing, TMPE reports any available fuselage AoA, leaves the NASA wing
  angle, coefficients and loads blank, and stops Phase 2 at that step.
- **Flaps and Mach.** `--transition-flaps FLAP:FLAPERON` holds one setting
  for the run. The default is `40:25`. The other available settings are
  `0:0`, `20:12.5`, and `75:47` (p. A-133). The lookup matches the flap pair
  and uses planar airspeed divided by
  `sqrt(1.4 * 287.05 * (temperature_C + 273.15))` for Mach. It matches the
  actual Mach to a printed interval or exact Mach column. It neither
  interpolates across Mach or flap settings nor extrapolates across AoA.
- **Reference area.** The report scales freestream loads by
  `S_WFS = S_W - S_iWL - S_iWR` (p. A-70). TMPE corrects the freestream
  angle and uses the full GTRS reference area of 181 ft² (16.81545024 m²), as
  documented by NASA TM-4448 Appendix A. [NASA TM-4448](https://ntrs.nasa.gov/api/citations/19930013297/downloads/19930013297.pdf)

### Alternate coefficient inputs

Select `--illustrative-transition` for the software-test constants CL = 0.50
and CD = 0.08, with lift applied upward and drag backward. Select
`--transition-polar path/to/wing.csv` for a fixed wing polar with the header
`alpha_deg,cl,cd`. A CSV needs at least two finite rows with strictly
increasing AoA in degrees and nonnegative CD. Its dimensionless coefficients
use the XML wing area and a geometric AoA approximation with wing incidence.
With prescribed pitch the approximation uses `pitch + incidence - gamma`.
Without pitch, this user-polar path labels its level-attitude assumption.
TMPE labels supplied tables
USER-SUPPLIED / UNVALIDATED and records any `--polar-source` attribution.
These two alternate inputs are mutually exclusive. The flap setting flag
applies to the NASA model.

## Data availability

| Input or result | Basis or blocker |
| --- | --- |
| Design gross mass, weight, wing geometry, nacelle setting | Existing XML. The program converts gross weight in pounds to mass in kg and calculates weight using standard gravity. [NASA TM-81244](https://ntrs.nasa.gov/api/citations/19810001546/downloads/19810001546.pdf) documents the design gross weight and nacelle convention. |
| Wing-pylon CL/CD | NASA CR-166536 Appendix B Tables 4-I–4-IV supply the default coefficients. The lookup requires compatible AoA, mast angle, flap setting and Mach coverage for both coefficients. |
| Actual rotor thrust, power and collective calibration | The shipped collective calibration lacks matched pitch/thrust data and reference conditions. Design RPM, diameter and engine rating do not establish mast power or a takeoff/conversion thrust law. [NASA TM-86833](https://ntrs.nasa.gov/citations/19860005773) is a full-scale hover-test data lead with a different scope from conversion. |
| Additional loads and attitude | Pitch is prescribed for AoA. Attitude dynamics, wake-immersed wing loads, and other airframe loads are outside the modeled force balance. |
| Air density, temperature and elevation | The entered density is held constant and used for dynamic pressure and prototype rotor thrust. The NASA lookup also holds the entered temperature constant to compute Mach. Ground elevation MSL is displayed as context. |


## Axes and force equations

The inertial plane faces north. Forward, up, and nose-up pitch are positive. The
wind-FROM bearing is clockwise from north. Wind blowing toward the north is
`-wind_speed * cos(bearing)`. The east component is logged and excluded from
the planar force calculation. The up component of wind is zero because the
entered wind is horizontal.

Subtract wind from ground velocity to get planar air-relative velocity
`(u, w)`. Airspeed is `hypot(u,w)`. Flight-path angle `gamma = atan2(w,u)`
is positive when the air-relative velocity points upward. The NASA lookup
uses rotor-corrected wing AoA. The CSV path uses geometric AoA with positive
incidence for leading-edge-up mounting. Both area inputs convert
from ft² to m² using 0.09290304 m²/ft².

```text
q = 0.5 rho V²
L = q S CL
D = q S CD
wing_forward = -L sin(gamma) - D cos(gamma)
wing_up      =  L cos(gamma) - D sin(gamma)
rotor_forward = T cos(nacelle)
rotor_up      = T sin(nacelle)
net_forward = left_forward + right_forward + wing_forward
net_up = left_up + right_up + wing_up - mass g
a = net_force / mass
position_next = position + velocity dt + 0.5 a dt²
velocity_next = velocity + a dt
```

At each step, the model uses the original planar projection of the retained Phase 1 thrust
and recalculates the wing loads from the current air-relative state. It applies
constant acceleration during that step and stops at first ground contact.
Rotor inflow, RPM response, power limits, ground effect, wake-immersed wing
loads, other airframe loads, lateral motion and attitude dynamics are
excluded. A calibrated Phase 1 input does not supply a rotor conversion
performance map or establish a validated transition trajectory.

## Audit

The console shows both stage times and transition checkpoints. Use
`--verbose` to print the equations. When
Phase 2 advances no time, the final state is explicitly attributed to Phase 1.
All evaluated rows are saved to a unique `output/stage2_audit_*.csv`
in the working directory. Use `--transition-audit` to choose a new file path.
The CSV records phase and cumulative times, flow, loads, motion, and per-rotor
and aerodynamic provenance. It shows air-relative velocity, flight-path
angle, pitch, and fuselage AoA. NASA rows also show the rotor-flow inputs
and correction, corrected wing AoA, any missing inputs, requested report mast,
Mach, flap/flaperon setting, reference area, source and lookup status.
When a single mast table is used, the CSV also records its angle and
the angular gap. Blank numerical cells represent unavailable
quantities. A preflight block before any step is reported on the console.

## Primary-source review

All coefficient paths hold Phase 1 thrust and label their trajectories
as illustrative in the console and CSV. Flight validation requires independent
measured XV-15 transition data.

| Report and locator | Applicable data | Why no runtime values were imported |
| --- | --- | --- |
| [NASA TM-86833](https://ntrs.nasa.gov/citations/19860005773), Fig. 17, Tables 2–3 and Appendix A | Full-scale original metal-blade rotor on a hover test stand, tip Mach 0.60–0.73. Table 2 defines `COLL` at 0.75 radius in degrees, `RPM` in rev/min, `RHO` in kg/m³ and thrust in N. Fig. 17 uses thrust coefficient divided by solidity. | The report supplies paired stand data. TMPE accepts a pilot pitch input with no established mapping to the actuator-derived blade datum, and its design RPM and entered density are scenario inputs. A matched run and a separate installed-rotor applicability model are needed before applying the data to takeoff. |
| [NASA CR-177436](https://ntrs.nasa.gov/citations/19880016983), §7, Fig. 7.3 (printed p. 53), Figs. 7.13 and 7.32–7.33 | Distinct full-scale baseline metal and advanced technology blade (ATB) hover configurations. The report compares nondimensional thrust and power and identifies run-level RPM/tip Mach. | The authors flag the baseline metal-blade collective readings as suspect and use them to illustrate curve shape. The ATB data apply to a different blade system. |
| [NASA TM-81177](https://ntrs.nasa.gov/citations/19800015802), notation, Fig. 13, Table II and tunnel scale-data printout | Full-aircraft tunnel runs in helicopter, conversion and airplane configurations. Test configurations include changes to pylon and wing hardware. | TMPE has no ingested run map matching rotor-on/off configuration, flap/flaperon settings, tilt, airspeed, thrust and shaft power. Total tunnel loads require a consistent decomposition before use alongside separate rotor forces. |
| [NASA CR-176970](https://ntrs.nasa.gov/citations/19860020297), Chapter 2, Tables 1 and 5, and Chapter 5 | Configuration-specific hover download estimates combining out-of-ground-effect hover data and representative wing tunnel data. Table 5 gives download relative to total aircraft thrust at a selected power setting across flap settings. | These hover results do not define wake download across nacelle angle and forward speed. No fixed hover download fraction was applied to takeoff or conversion. |

The existing fixed-CT demonstration uses the illustrative `CT=0.0102`
from `rotor.xml`, a 25 ft diameter and the XML design hover setting of
565 rev/min. At an entered density of 1.225 kg/m³, the diameter converts to
7.62 m, radius to 3.81 m, and angular speed to `565 × 2π/60` rad/s.
Evaluating `CT × ρ × πR² × (ΩR)²` gives 28,956.11 N per rotor and
57,912.21 N total. Output rounds each displayed value independently.
