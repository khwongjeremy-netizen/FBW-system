# TMPE variable and model priorities

## What TMPE already takes as input

| Input | Where it comes from and how TMPE uses it |
| --- | --- |
| Air density | Entered in kg/m³ and held constant for the run. It affects prototype rotor thrust and the wing's dynamic pressure, which scales with density and the square of airspeed. |
| Wing reference area | Loaded from `geometry.xml` in ft² and converted to m² for the transition lift and drag calculations. |
| Nacelle angle | The starting helicopter setting comes from `aircraft.xml`. Timed transition accepts separate left/right end angles and moves each nacelle from its starting angle toward its end angle. At 0° thrust points forward, and at 90° it points up. |
| Left and right blade collective pitch | Entered separately in degrees in prototype collective mode. Collective is the pitch setting shared by the blades of one rotor. Nacelle tilt sets the direction of rotor thrust. |

## Rotor-thrust method and status

- **Research/design — SOLVED.** Use `CT = T / [rho A (Omega R)^2]` to express measured per-rotor thrust as a dimensionless coefficient, and `T = CT rho A (Omega R)^2` to recover thrust. Here `rho` is air density, `A = pi R^2` is the rotor disk area, `Omega` is angular speed in rad/s, and `R` is rotor radius. NASA CR-2017-219486 presents XV-15 hover thrust coefficient versus collective pitch at 75% blade radius in Fig. 20. NASA CR-177436 also provides hover data, but §7 and Fig. 7.3 flag some recorded baseline metal-blade collective settings as suspect and use those readings only to illustrate the curve shape. [NASA CR-2017-219486, Fig. 20](https://ntrs.nasa.gov/api/citations/20180000109/downloads/20180000109.pdf) · [NASA CR-177436, §7 and Fig. 7.3](https://ntrs.nasa.gov/api/citations/19880016983/downloads/19880016983.pdf)
- **Implementation — prototype.** TMPE implements the thrust equation and can evaluate a supplied linear collective-to-CT fit. The shipped configuration is uncalibrated and has no fit parameters. The checked-in transition run uses synthetic rotor data and carries the takeoff thrust magnitudes forward as the nacelles tilt. [Engine code](../MY_physics_engine/tiltrotor_modeling_physics_engine%28TMPE%29.py) · [Shipped collective configuration](../MY_physics_engine/data%28v.2%29/collective_model.json)
- **Validation — open.** TMPE's **15° collective** input is a blade-pitch command. The mapping to NASA's measured pitch at 75% blade radius is unresolved, including the pitch datum and how the command corresponds to measured blade angle. To establish XV-15 thrust at that input, match the pitch definition and rotor version (baseline metal or advanced technology blade) to source data at applicable RPM or tip Mach, density, and inflow. Validate against measured thrust under comparable conditions. The hover data establish a starting point for hover, and conversion requires its own operating-condition coverage. [Collective model notes](TMPE_COLLECTIVE_MODEL.md) · [Transition source review](TMPE_TRANSITION.md#primary-source-review)

## Model priority, highest to lowest

| Rank | Missing piece | Why it matters |
| --- | --- | --- |
| 1 | Wing lift and drag as angle of attack changes | Wing lift supports the aircraft as it gains speed, and drag affects forward acceleration. TMPE can already look up coefficients by angle of attack in a supplied table. The latest checked-in transition CSV uses fixed coefficients. A source-backed model should cover the relevant airflow and aircraft configurations, including nacelle tilt where supported by the data. |
| 2 | Source-backed rotor-thrust implementation and validation | Match collective and operating conditions to measurements, fit or tabulate CT, and check predicted thrust against measured results. The transition currently changes the direction of the Phase 1 thrust estimates while holding their magnitudes fixed. |
| 3 | Rotor power, torque, and engine limits | Torque is the twisting load on the rotor shaft. Power and torque limits determine whether the engines and drivetrain can deliver the thrust the model asks for. The code has an engine rating and ideal-hover estimates, while the timed transition has no power or torque limits. |
| 4 | Rotor/wing interaction, ground effect, and wind | Rotor wake can change wing loads, and proximity to the ground can change rotor performance. TMPE already accounts for the forward component of entered horizontal wind in the wing airflow calculation. The east component is logged, and wind-dependent rotor loads and sideways response remain gaps. |
| 5 | Full attitude and sideways motion | The timed model moves forward and up with the aircraft held level and facing north. Sideways motion and changes in roll, pitch, and heading are needed to study crosswind response and how unequal rotor forces turn the aircraft. |

## Practical difficulty, easiest to hardest

| Rank | Work | What it takes |
| --- | --- | --- |
| 1 | Check units, force directions, and basic calculations | Compare a few rows against hand calculations for unit conversions, thrust direction at 0° and 90°, net forces, and acceleration. |
| 2 | Check airflow and angle of attack through the wing-force calculation | Use the existing table-driven path and known inputs to trace wind subtraction, airflow angle, coefficient lookup, and the resulting forward and upward wing forces. |
| 3 | Find or build a source-backed wing lift/drag model | Match the source's angle, wing reference area, and aircraft configuration to TMPE. Identify where the source supports interpolation and where coverage ends. |
| 4 | Calibrate and validate the rotor-thrust model | Match the blade-pitch definition and rotor configuration to thrust measurements across pitch, rotor speed, density, and airflow. Establish where the model can be used before carrying it into conversion. |
| 5 | Add power limits and rotor/wing interactions | Use shaft power and torque data to constrain thrust, then account for how the rotor wake changes wing loads across relevant conditions. |
| 6 | Add full attitude and sideways motion | Extend the model to track rotation and sideways movement. This requires aircraft inertia, force locations, and the turning effects of those forces. |

## Recommended next step

**Calculate and connect wing angle of attack (AoA) in a transition run.** Use the existing table-driven path with a small test table. Trace how it subtracts forward wind from ground velocity, computes the air-relative flight-path angle, and subtracts that angle from wing mounting incidence for the level-aircraft model. Check a case by hand, then verify that `alpha_deg` appears in the audit where airflow and incidence support it, selects the expected CL/CD, and produces lift perpendicular to the airflow path and drag opposing motion through the air.

This subtask uses the calculation and audit columns already in the code. A test table checks the connection from airflow to wing force. An applicable XV-15 coefficient table and comparison with measured wing loads are separate validation work. [Transition implementation notes](TMPE_TRANSITION.md)

## Current CSV limitations

The latest checked-in transition audit uses illustrative, unvalidated software-test assumptions. All 101 rows are labeled `ILLUSTRATIVE / SOFTWARE-TEST ONLY`, and both rotors use the synthetic collective fixture. The stored thrust magnitudes stay fixed as the nacelles tilt. [Transition audit CSV](../output/stage2_audit_zwrjdnhe.csv)

- **Blank `gamma_deg` and `alpha_deg`.** The simplified path computes air-relative speed and skips the flight-path angle and AoA calculations. A blank means that the angle was left uncalculated in this run. The table-driven path calculates these angles when the airflow and wing mounting angle are available.
- **Fixed `cl = 0.50` and `cd = 0.08`.** These are dimensionless test values for lift and drag. The wing forces can change with airspeed through dynamic pressure, but the coefficients stay fixed. The simplified path applies lift upward and drag backward. The table-driven path looks up coefficients by AoA and directs the forces relative to the airflow.
- **Validation remains open.** The code labels supplied coefficient tables as unvalidated as well. A table can exercise the AoA and force calculations. Predicting XV-15 performance requires applicable aircraft data and comparison with measured results.
