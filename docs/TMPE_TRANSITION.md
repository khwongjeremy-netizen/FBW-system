# TMPE Phase 2 timed transition

Phase 2 starts at the last Phase 1 state. Altitude is above ground, up velocity
carries over, and forward distance and speed start at zero because Phase 1 is
vertical. An airborne Phase 1 result is required. Each stage has its own positive
duration prompt, with equivalent CLI flags for scripted runs. The shared
`--dt` defaults to 0.1 seconds. A final step can be shorter.

```sh
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' \
  --rotor-model prototype-fixed-ct --duration 5 --transition-duration 2 \
  --transition-end 45:30 --transition-polar path/to/wing.csv
```

The example selects the fixed-CT software-test model. The illustrative
collective fixture also requires explicit opt-in. Left and right nacelles each
ramp linearly from their Phase 1 angles to the requested final angles (default
`0:0`). The range 0–90° is a software input range. The separate
`--nacelle-schedule 90:90 45:45 0:0` option still prints untimed geometric
samples. Those samples leave the timed schedule and state untouched.

## Wing coefficient input

The XML contains curve descriptions and approximate descriptors, but no usable
paired wing CL/CD arrays. Supply a CSV with the header `alpha_deg,cl,cd`.
Use wing angle of attack in degrees and dimensionless lift and drag
coefficients based on the XML wing reference area. Provide at least two rows
with finite values and strictly increasing angles. CD must be nonnegative.
Both endpoints are usable. Values between rows are linearly interpolated.
If AoA is undefined or outside the supplied range, the simulation stops
advancing and reports the condition. The lookup assumes one fixed wing polar
throughout the conversion.

Every supplied table is marked USER-SUPPLIED / UNVALIDATED. The optional
`--polar-source` text is stored as user-provided metadata, separately from
validation status. The program does not infer CL or CD from curve descriptions.

## Data availability

| Input or result | Basis or blocker |
| --- | --- |
| Design gross mass, weight, wing geometry, nacelle setting | Existing XML. The program converts gross weight in pounds to mass in kg and calculates weight using standard gravity. [NASA TM-81244](https://ntrs.nasa.gov/api/citations/19810001546/downloads/19810001546.pdf) documents the design gross weight and nacelle convention. |
| Wing CL/CD | Requires a CSV for this run. The configuration-dependent wing/pylon tables cited in [NASA CR-166536](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf) need their flow-angle, control-setting, reference-area and nacelle-angle definitions mapped before use. The [repository data-gap report](../TMPE_AERODYNAMIC_DATA_GAPS_REPORT.md) records the table locations and remaining mapping work. A generic airfoil polar cannot establish aircraft wing/pylon loads. |
| Actual rotor thrust, power and collective calibration | The shipped collective calibration lacks matched pitch/thrust data and reference conditions. Design RPM, diameter and engine rating do not establish mast power or a takeoff/conversion thrust law. [NASA TM-86833](https://ntrs.nasa.gov/citations/19860005773) is a full-scale hover-test data lead with a different scope from conversion. |
| Additional loads and attitude | Rotor/wing interference, other airframe loads, and attitude response are outside the implemented force model. |
| Air density, temperature and elevation | The entered density is held constant and used for dynamic pressure and prototype rotor thrust. Temperature and ground elevation MSL are displayed as context. There is no atmosphere calculation. |


## Axes and force equations

The aircraft stays level and faces north. Forward and up are positive. The
wind-FROM bearing is clockwise from north. Wind blowing toward the north is
`-wind_speed * cos(bearing)`. The east component is logged and excluded from
the planar force calculation. The up component of wind is zero because the
entered wind is horizontal.

Subtract wind from ground velocity to get planar air-relative velocity
`(u, w)`. Airspeed is `hypot(u,w)`. Flight-path angle `gamma = atan2(w,u)`
is positive when the air-relative velocity points upward. Wing AoA is
`incidence - gamma`, with incidence positive for leading-edge-up mounting.
Wing area converts from ft² to m² using 0.09290304 m²/ft².

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

At each step, the model reprojects the two retained Phase 1 thrust magnitudes
and recalculates the wing loads from the current air-relative state. It applies
constant acceleration during that step and stops at first ground contact.
Rotor inflow, RPM response, power limits, ground effect, rotor/wing
interference, other airframe loads, lateral motion and attitude dynamics are
excluded. A calibrated Phase 1 input does not supply a rotor conversion
performance map or establish a validated transition trajectory.

## Audit

The console shows both stage times and up to five checkpoints, including the
first and last evaluated states. Use `--verbose` to print the equations. When
Phase 2 advances no time, the final state is explicitly attributed to Phase 1.
All evaluated rows are saved to a unique `output/stage2_audit_*.csv`
in the working directory. Use `--transition-audit` to choose a new file path.
The CSV records phase and cumulative times, flow, loads, motion, and per-rotor
and aerodynamic provenance. Blank numerical cells represent unavailable
quantities. A preflight block before any step is reported on the console.

## Primary-source review

The aircraft transition remains unavailable. The timed demonstration uses
held Phase 1 thrust and one supplied wing polar. Both the console and CSV
identify that trajectory as illustrative. Independent measured XV-15
transition data would be needed for flight validation.

| Report and locator | Applicable data | Why no runtime values were imported |
| --- | --- | --- |
| [NASA TM-86833](https://ntrs.nasa.gov/citations/19860005773), Fig. 17, Tables 2–3 and Appendix A | Full-scale original metal-blade rotor on a hover test stand, tip Mach 0.60–0.73. Table 2 defines `COLL` at 0.75 radius in degrees, `RPM` in rev/min, `RHO` in kg/m³ and thrust in N. Fig. 17 uses thrust coefficient divided by solidity. | The report supplies paired stand data. TMPE accepts a pilot pitch input with no established mapping to the actuator-derived blade datum, and its design RPM and entered density are scenario inputs. A matched run and a separate installed-rotor applicability model are needed before applying the data to takeoff. |
| [NASA CR-177436](https://ntrs.nasa.gov/citations/19880016983), §7, Fig. 7.3 (printed p. 53), Figs. 7.13 and 7.32–7.33 | Distinct full-scale baseline metal and advanced technology blade (ATB) hover configurations. The report compares nondimensional thrust and power and identifies run-level RPM/tip Mach. | The authors flag the baseline metal-blade collective readings as suspect and use them to illustrate curve shape. The ATB data apply to a different blade system. |
| [NASA TM-81177](https://ntrs.nasa.gov/citations/19800015802), notation, Fig. 13, Table II and tunnel scale-data printout | Full-aircraft tunnel runs in helicopter, conversion and airplane configurations. Test configurations include changes to pylon and wing hardware. | TMPE has no ingested run map matching rotor-on/off configuration, flap/flaperon settings, tilt, airspeed, thrust and shaft power. Total tunnel loads require a consistent decomposition before use alongside separate rotor forces. |
| [NASA CR-176970](https://ntrs.nasa.gov/citations/19860020297), Chapter 2, Tables 1 and 5, and Chapter 5 | Configuration-specific hover download estimates combining out-of-ground-effect hover data and representative wing tunnel data. Table 5 gives download relative to total aircraft thrust at a selected power setting across flap settings. | These hover results do not define wake download across nacelle angle and forward speed. No fixed hover download fraction was applied to takeoff or conversion. |

No NASA test coefficients were added or converted into live model inputs.
The existing fixed-CT demonstration uses the illustrative `CT=0.0102`
from `rotor.xml`, a 25 ft diameter and the XML design hover setting of
565 rev/min. At an entered density of 1.225 kg/m³, the diameter converts to
7.62 m, radius to 3.81 m, and angular speed to `565 × 2π/60` rad/s.
Evaluating `CT × ρ × πR² × (ΩR)²` gives 28,956.11 N per rotor and
57,912.21 N total. Output rounds each displayed value independently.
