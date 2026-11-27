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
explicit nose-up-positive pitch for body-axis velocity and AoA. With the
bundled rotor model the NASA path reports the available flow inputs and stops
when a required in-plane force `H` or `Y` is unavailable. Add
`--illustrative-rotor-flow` to assume `H = Y = 0` and exercise the NASA
wing AoA correction. The lookup still requires the report's table coverage.
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
  `beta_m_deg = 90 - nacelle_deg` (p. A-50). Each rotor uses its own angle
  for the hub transform. The NASA wing correction and coefficient lookup
  require a common mast angle. The report calls for interpolation between
  the endpoint mast angles (p. 25). Both endpoint configurations must
  support the query for an interior-angle interpolation. At an endpoint the
  lookup requires CL and CD coverage at that angle.
- **Angle of attack.** TMPE calculates fuselage AoA as `alpha_F = pitch - gamma`.
  With no pitch input, fuselage AoA is unavailable on the NASA path. The
  freestream wing AoA subtracts the rotor-flow correction on NASA CR-166536
  p. 25 directly from fuselage AoA. The correction in radians is
  `K_XRW * x_R/W * (C_RFR + C_RFL) / max(mu, 0.15)^2`.
  TMPE uses the left/right mean advance ratio and
  `x_R/W = X_RW0 + beta_m * (X_RW1 + beta_m * X_RW2)` from p. A-70,
  with mast angle in degrees and the constants from p. B-33. The AoA
  subtraction stays in radians. The resulting wing AoA is converted to
  degrees for the table lookup. This angle definition has no
  separate wing-incidence term.
- **Rotor-flow inputs.** Figure A1-1 (p. A-13) distinguishes mast axes from
  wind-mast axes and shows the mirrored rotor conventions. At each NASA
  lookup step, TMPE adds `(p, q, r) × CG_to_hub` to aircraft body
  velocity to obtain each hub velocity in body axes. It then applies the
  corresponding mast-axis transform from pp. A-18–A-19. The audit reports both
  vectors and the input mast angles.
  It computes shaft speed as `Omega = RPM * 2*pi / 60` and corrects it
  for the body rates using p. A-16. Page A-20 uses that corrected speed
  `Omega_prime` for `mu = hypot(U_HM, V_HM) / (Omega_prime * R)`.
  Page A-38 uses the **unprimed shaft speed** for the force coefficient
  `C_RF = hypot(T, H, Y) / (rho * pi * Omega^2 * R^4)`.
  The wake subsystem input list on p. A-35 includes corrected speed.
  TMPE follows the explicit p. A-38 normalization for `C_RF`.
  The rotor aerodynamic model's `C_T = T / DN` (Fig. A1-3, p. A-25)
  instead uses `DN = rho * pi * Omega_prime^2 * R^4` (pp. A-15, A-20).
  The planar model's zero body rates make the two speeds numerically equal.
  The arithmetic mean of left and right advance ratios is specified on
  p. A-70. Figure A4-2 covers the wake-immersed wing calculation.
  [NASA CR-166536, Appendix A](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf#page=166)
- **Input availability.** The built-in provider uses the 25 ft rotor
  diameter and accepts the design hover setting of 565 RPM from
  `rotor.xml` as the operating speed for both modeled rotors. It uses the
  1° lateral mast tilt from the XV-15 input deck and the held Phase 1 axial
  thrust for `T` on each rotor, with the selected Phase 1 calibration status.
  The bundled rotor models lack the operating inputs for
  [NASA rotor in-plane forces](#nasa-rotor-in-plane-forces). This blocks both
  `C_RF` values and the wing AoA correction. With prescribed pitch, TMPE
  still calculates `U`, `W` and both mast-plane advance ratios using the
  planar assumptions `V = 0` and `p = q = r = 0`.
  [NASA CR-166535, Appendix I, p. I-2](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166535_881.pdf)
- **CG-to-hub geometry.** The `hub_left_*` and `hub_right_*` fields in
  `geometry.xml` describe the conversion tilt-axis centerline in the
  aircraft datum. The same file's `candidate_cg_*` fields are unverified,
  and `mass.xml` gives the nominal CG fuselage station. Nonzero-rate hub
  velocities require left and right hub-center offsets from the same
  aircraft CG in meters, using forward/right/down body axes at the current
  mast angles. The NASA shaft-pivot construction also works when matched
  CG and shaft-pivot station, butt line and waterline coordinates and the
  shaft-pivot-to-hub mast length are available. The built-in model leaves
  the offsets blank. The rate terms vanish at the assumed zero rates.
- **Illustrative rotor flow.** `--illustrative-rotor-flow` opts into a
  zero in-plane force assumption of `H = Y = 0` on each rotor. The
  correction, lookup, and resulting loads are labeled
  **ILLUSTRATIVE / UNVALIDATED**. This mode needs prescribed pitch, an
  airborne Phase 1 handoff and table coverage at each evaluated step.
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

Python callers can pass `rotor_flow_provider(state, environment)` to
`simulate_transition()`. Return a sourced `XV15RotorInputs` with signed
body velocity in m/s, body rates in rad/s, lateral mast tilt in degrees,
and a left and right `XV15RotorOperatingPoint` with RPM and forces in
newtons. For each rotor, `left_hub_offset_body_m` and
`right_hub_offset_body_m` accept signed forward/right/down offsets from the
aircraft CG at the current mast angle. The interface also accepts the
report's mirrored `LXH, LYH, LZH` lever terms in meters for symmetric
geometry, or matched CG and right shaft-pivot datums and mast length
through `XV15RotorHubGeometry`. The hook also accepts previously resolved
`XV15RotorFlow` values and identifies them as externally supplied in the audit.

### NASA rotor in-plane forces

The simplified run holds the `rotor.xml` design hover speed of 565 rpm
through Phase 2 and labels it `XML-SOURCED / HELD FIXED`. `Omega` is the
shaft speed in rad/s. Appendix A p. A-16 uses body rates and each rotor's
mast angle to obtain effective speed `Omega_prime`. The planar model
assumes zero body rates and lateral body velocity. The hub transform on
pp. A-18–A-19 applies the left and right signs in their local frames. With
nonzero rates, the hub velocity also requires a sourced CG-to-hub offset.
[NASA CR-166536, Appendix A](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf#page=166)

The effective tip speed `Omega_prime * R` normalizes wind-mast rates,
advance ratio `mu = hypot(U_HM, V_HM) / (Omega_prime * R)`, and axial inflow
`lambda_0 = -W_HM / (Omega_prime * R)`. The wind azimuth is
`xi = atan2(V_HM, U_HM)` in radians for nonzero mast-plane speed. If that
speed is zero, `mu` is zero when the effective tip speed is available, and
`xi` stays blank because the flow has no in-plane direction. Page A-21
rotates the supplied local `A1` and `B1` cyclic angles using `xi`.
[NASA CR-166536, pp. A-16–A-21](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf#page=166)

Tip Mach uses CG airspeed `V_T` and shaft tip speed `Omega * R`, per
p. A-21. The temperature-based sound speed uses `sqrt(1.4 * 287.05 * T_K)`
in m/s for calorically perfect dry air, with `T_K = T_C + 273.15`.
[NASA CR-166536, p. A-21](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf) ·
[NASA Glenn, Speed of Sound](https://www.grc.nasa.gov/www/BGH/sound.html)

```text
M_TIP = sqrt(V_T² + (Omega * R)² + 2 * V_T * Omega * R * cos(beta_m)) / sound_speed
DN    = rho * pi * R² * (Omega_prime * R)²
a_R   = (a0 + mu * (a1 - a2 * mu)) / sqrt(1 - (0.75 * M_TIP)² * sin(beta_m))
Q6    = 0.5 * sigma * a_R * DN
```

`DN` and `Q6` are in newtons. Both use effective speed. Section 9 on
pp. A-30–A-31 evaluates the separate wind-mast forces `H_bar` and `Y_bar`.
Section 10 on p. A-31 resolves them into mast axes at that rotor's azimuth.
[NASA CR-166536, pp. A-20–A-31](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf#page=166)

```text
H = H_bar * cos(xi) + Y_bar * sin(xi)
Y = -H_bar * sin(xi) + Y_bar * cos(xi)
```

Each rotated force requires only the intermediate terms with nonzero
rotation coefficients. For example, an exact `xi = 0` permits `Y` from
`Y_bar` even with `H_bar` unavailable. TMPE records a supplied
`h_lambda_coefficient` as an unverified candidate and keeps `H_bar` blank.
A legible p. A-30 scan is needed to identify the coefficient multiplying
`lambda * (3/2 * a_bar1 - p_hat)`. A provider can supply `H` and `Y`
independently with provenance. `--illustrative-rotor-flow` explicitly
assumes both are zero. The resultant `C_RF` requires `T`, `H`, `Y`, density,
radius, and shaft RPM (p. A-38). Thrust-dependent calculations from
illustrative or assumed thrust carry an illustrative, unvalidated status.
[NASA CR-166536, pp. A-30–A-31, A-38](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166536_882.pdf#page=166)

Attach a sourced `XV15InPlaneState` to `XV15RotorOperatingPoint.in_plane_state`
for each rotor. Provide the input citations in `source` and describe any
assumptions in `assumptions`. Existing sourced operating values take precedence
over values derived from raw inputs.

| Operating value | Provider fields and dependencies |
| --- | --- |
| Effective lift slope `a_R` and force scale `Q6` | Supply `lift_slope_per_rad` or the report's average lift coefficients `blade_lift_a0`, `blade_lift_a1`, and `blade_lift_a2`. The derivation also needs `mu`, `M_TIP`, and the mast angle. `Q6` needs the selected slope, `DN`, and geometric solidity. |
| Blade-pitch constants `CSn` and `CCn` | Supply the required operating constants or `root_collective_rad` and each sourced `twist_terms_n0` through `twist_terms_n3` tuple as `(TW1n, TW2n, TW3n, TW4n)`. Root collective uses the report's datum. The CLI pilot collective has no verified root-collective mapping. |
| Profile drag factor `Cdf` | Supply `profile_drag_factor` or all three `blade_drag_delta*` values and `cdmax`, `cdalph`, `cdlim`, `cdmach`, and `cdfact`. The p. A-22 calculation also uses `M_TIP`, blade count, and `alpha_r = 7 * (T / DN) / (sigma * a_R)`. |
| Operating coning angle | Supply `coning_rad` or `blade_flapping_inertia_kg_m2` and `coning_hub_spring_nm_per_rad` for the coning spring `K_HUB`. The p. A-22 equation uses the configuration precone, per-rotor thrust, blade count, radius, and effective `Omega_prime`. A provider can supply `precone_rad` in place of the 2.5° XML precone. The report's `TW34` is 34.525° and is recorded in radians. |
| Induced inflow `lambda_i` | Supply `induced_inflow` or all five Fig. A1-3 factors `tip_loss_factor` (`B`), `ground_effect_factor` (`G`), `washout` (`WASHOUT`), `side_by_side_factor` (`X_SS`), and `tandem_factor` (`X_SF`). With `T`, `DN`, `mu`, and `lambda_0` available, the p. A-24 inner iteration uses held thrust and records its iteration count and convergence status. Total inflow is `lambda_0 + lambda_i`. Supply the operating `inflow_distribution` (`K`) separately for H/Y. |
| Wind-mast flapping `a_bar1`, `b_bar1` | Supply both current angles in `longitudinal_flap_rad` and `lateral_flap_rad`, or provide `flapping_c_matrix` (N m s/rad) and `flapping_a_matrix` (N m/rad) as four row-major entries each, `flapping_forcing_nm` as the matched-control `(B1, B2)` pair, `flapping_prior_rad`, and a positive `flapping_step_s`. TMPE applies a backward Euler step with the supplied terms held fixed for that step. Supply the local cyclic angles in `lateral_cyclic_mast_rad` and `longitudinal_cyclic_mast_rad` for the force formulas. |

The XV-15 companion input decks list candidate twist stations, lift and
drag coefficients, stiffnesses, and inflow parameters. Appendix I and
Appendix J report different `a2` signs, and the operating configuration
must match before selecting the deck. Candidate blade-performance and
inflow parameters require an explicit source-backed match.
Default CLI runs report unavailable `H_bar`, `Y_bar`, `H`, `Y`, and `C_RF`
when the operating inputs are missing. The wide audit gives the individual
missing dependencies, with force results and status in the rotor CSV.
[NASA CR-166535, Appendices I and J](https://rotorcraft.arc.nasa.gov/Publications/files/CR-166535_881.pdf)

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
| Additional loads and attitude | Pitch is prescribed for body-axis velocity and AoA. Attitude dynamics, wake-immersed wing loads, and other airframe loads are outside the modeled force balance. |
| Air density, temperature and elevation | The entered density is held constant and used for dynamic pressure and prototype rotor thrust. The NASA lookup also holds the entered temperature constant to compute Mach. Ground elevation MSL is displayed as context. |


## Axes and force equations

The inertial plane faces north. Forward, up, and nose-up pitch are positive. The
wind-FROM bearing is clockwise from north. Wind blowing toward the north is
`-wind_speed * cos(bearing)`. The east component is logged and excluded from
the planar force calculation. The up component of wind is zero because the
entered wind is horizontal.

Subtract wind from ground velocity to get planar air-relative forward
and up components. The flight-path angle is `atan2(air_up, air_forward)`.
For nose-up-positive pitch `theta`, the north-facing plane gives NASA body
`U = air_forward*cos(theta) + air_up*sin(theta)` and
`W = air_forward*sin(theta) - air_up*cos(theta)`. Body `U`, `V` and `W`
point forward, right and down. The NASA lookup uses rotor-corrected wing AoA.
The CSV path uses geometric AoA with positive incidence for leading-edge-up
mounting. Both area inputs convert from ft² to m² using 0.09290304 m²/ft².

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

The default terminal report shows stage times, transition checkpoints, rotor
and wing status, model limits, and CSV paths. It highlights the main blockers
when a stage cannot advance. Use `--verbose` for the longer report and force
equations, or `--diagnostics` for XML sources and input diagnostics. When
Phase 2 advances no time, the report attributes the final state to Phase 1.

The full audit uses a unique `output/stage2_audit_*.csv` in the working
directory. Use `--transition-audit` to provide a new file path. The audit
keeps every evaluation and its motion, loads, model assumptions, individual
sources, and missing-input reasons. It includes the body-axis and mast-axis
details behind the NASA rotor-flow lookup and records the selected TMPE rotor
coefficient and the CR-177436 hover estimate as separate fields.

Three companion files use fixed names inside a sibling `<audit_stem>_reports/`
directory. They use the same checkpoints as the console, at the start, every
2.5 s, and the end, sampled from the nearest evaluated timesteps.
If the audit path cannot be opened, TMPE attempts to save the companions in a
separate `stage2_reports_*` directory.

- `stage2_checkpoints.csv` records one row per checkpoint with state, nacelle
  angles, rotor thrust, wing loads, net forces, and checkpoint status.
- `stage2_rotors.csv` records one row per rotor per checkpoint with RPM,
  selected mast-axis `T`, `H`, and `Y`, advance ratio, `C_RF`, source, and
  status. The held Phase 1 axial thrust has its own column.
- `stage2_aero.csv` records one row per checkpoint with air-relative state,
  angles of attack, requested and table mast angle, flap settings, Mach,
  coefficients, wing loads, and lookup status.

Units appear in column names where applicable. Empty numeric cells indicate
unavailable quantities, with status and reasons alongside. A blocked Phase 1
handoff produces a preflight-only audit record with the blocker and available
inputs, and the companion files contain headers only.

## Primary-source review

All coefficient paths hold Phase 1 thrust and label their trajectories
as illustrative in the console and CSV. Flight validation requires independent
measured XV-15 transition data.

| Report and locator | Applicable data | Why no runtime values were imported |
| --- | --- | --- |
| [NASA TM-86833](https://ntrs.nasa.gov/citations/19860005773), Fig. 17, Tables 2–3 and Appendix A | Full-scale original metal-blade rotor on a hover test stand, tip Mach 0.60–0.73. Table 2 defines `COLL` at 0.75 radius in degrees, `RPM` in rev/min, `RHO` in kg/m³ and thrust in N. Fig. 17 uses thrust coefficient divided by solidity. | The report supplies paired stand data. TMPE accepts a pilot pitch input with no established mapping to the actuator-derived blade datum, and its design RPM and entered density are scenario inputs. A matched run and a separate installed-rotor applicability model are needed before applying the data to takeoff. |
| [NASA CR-177436](https://ntrs.nasa.gov/citations/19880016983), §7.1 and Figs. 7.1–7.3 (printed pp. 50–54) | Original XV-15 metal blades on an outdoor hover stand at nominal tip Mach 0.69. Figs. 7.1 and 7.2 compare `C_T` with wind-corrected power coefficient and figure of merit, respectively. Fig. 7.3 compares `C_T` with collective at 0.75 radius, with OARF runs at 582.8–586.2 RPM and WPAFB comparison points at several tip speeds. | The authors flag the OARF pitch readings and use them to show curve shape. The two facilities differ by about 4° in reported pitch. The comparison points are plotted, and TMPE lacks a mapping from pilot input to the test pitch datum. The stand curves cover hover. ATB curves elsewhere in the report describe a different blade system. |
| [NASA TM-81177](https://ntrs.nasa.gov/citations/19800015802), notation, Fig. 13, Table II and tunnel scale-data printout | Full-aircraft tunnel runs in helicopter, conversion and airplane configurations. Test configurations include changes to pylon and wing hardware. | TMPE has no ingested run map matching rotor-on/off configuration, flap/flaperon settings, tilt, airspeed, thrust and shaft power. Total tunnel loads require a consistent decomposition before use alongside separate rotor forces. |
| [NASA CR-176970](https://ntrs.nasa.gov/citations/19860020297), Chapter 2, Tables 1 and 5, and Chapter 5 | Configuration-specific hover download estimates combining out-of-ground-effect hover data and representative wing tunnel data. Table 5 gives download relative to total aircraft thrust at a selected power setting across flap settings. | These hover results do not define wake download across nacelle angle and forward speed. No fixed hover download fraction was applied to takeoff or conversion. |

TMPE leaves the CR-177436 hover estimate unavailable. The plotted axis
limits are display limits. A numerical pitch-to-thrust relationship would
need verified test points and applicable test conditions.

The existing fixed-CT demonstration uses the illustrative `CT=0.0102`
from `rotor.xml`, a 25 ft diameter and the XML design hover setting of
565 rev/min. At an entered density of 1.225 kg/m³, the diameter converts to
7.62 m, radius to 3.81 m, and angular speed to `565 × 2π/60` rad/s.
Evaluating `CT × ρ × πR² × (ΩR)²` gives 28,956.11 N per rotor and
57,912.21 N total. Output rounds each displayed value independently.
