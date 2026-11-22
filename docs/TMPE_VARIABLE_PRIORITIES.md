# TMPE variable and model priorities

## What TMPE already takes as input

| Input | Where it comes from and how TMPE uses it |
| --- | --- |
| Air density | Entered in kg/m³ and held constant for the run. It affects prototype rotor thrust and the wing's dynamic pressure, which scales with density and the square of airspeed. |
| Wing reference area | Loaded from `geometry.xml` in ft² and converted to m² for the transition lift and drag calculations. |
| Nacelle angle | The starting helicopter setting comes from `aircraft.xml`. Timed transition accepts separate left/right end angles and moves each nacelle from its starting angle toward its end angle. At 0° thrust points forward, and at 90° it points up. |
| Left and right blade collective pitch | Entered separately in degrees in prototype collective mode. Collective is the pitch setting shared by the blades of one rotor. Nacelle tilt sets the direction of rotor thrust. |

Entering **15° collective** supplies a blade-pitch command. Estimating how much thrust that command produces requires a supported pitch-to-thrust relationship for the rotor and its operating conditions. The shipped collective configuration is marked uncalibrated and has no pitch-to-thrust fit. The synthetic test fixture can exercise the calculation, but validated XV-15 thrust at 15° cannot be confirmed from these files. [Engine code](../MY_physics_engine/tiltrotor_modeling_physics_engine%28TMPE%29.py) · [Shipped collective configuration](../MY_physics_engine/data%28v.2%29/collective_model.json)

## Model priority, highest to lowest

| Rank | Missing piece | Why it matters |
| --- | --- | --- |
| 1 | Rotor thrust across collective pitch and operating conditions | Thrust determines whether the aircraft can lift off and how it accelerates as the nacelles tilt. Phase 2 carries over the Phase 1 thrust magnitudes and changes their direction. A supported model should account for pitch, rotor speed, air density, and the airflow through the rotor. |
| 2 | Wing lift and drag as flight conditions change | Wing lift supports the aircraft as it gains speed, and drag affects forward acceleration. TMPE can read one table of lift and drag coefficients versus angle of attack. A source-backed model should cover the relevant airflow and aircraft configurations, including nacelle tilt where supported by the data. |
| 3 | Rotor power, torque, and engine limits | Torque is the twisting load on the rotor shaft. Power and torque limits determine whether the engines and drivetrain can deliver the thrust the model asks for. The code has an engine rating and ideal-hover estimates, while the timed transition has no power or torque limits. |
| 4 | Rotor/wing interaction, ground effect, and wind | Rotor wake can change wing loads, and proximity to the ground can change rotor performance. TMPE already accounts for the forward component of entered horizontal wind in the wing airflow calculation. The east component is logged, and wind-dependent rotor loads and sideways response remain gaps. |
| 5 | Full attitude and sideways motion | The timed model moves forward and up with the aircraft held level and facing north. Sideways motion and changes in roll, pitch, and heading are needed to study crosswind response and how unequal rotor forces turn the aircraft. |

## Practical difficulty, easiest to hardest

| Rank | Work | What it takes |
| --- | --- | --- |
| 1 | Check units, force directions, and basic calculations | Compare a few rows against hand calculations for unit conversions, thrust direction at 0° and 90°, net forces, and acceleration. |
| 2 | Check airflow and angle of attack through the wing-force calculation | Use the existing table-driven path and known inputs to trace wind subtraction, airflow angle, coefficient lookup, and the resulting forward and upward wing forces. |
| 3 | Find or build a source-backed wing lift/drag model | Match the source's angle, wing reference area, and aircraft configuration to TMPE. Identify where the source supports interpolation and where coverage ends. |
| 4 | Find or build a supported rotor-thrust model | Match the blade-pitch definition and rotor configuration to thrust measurements across pitch, rotor speed, density, and airflow. Establish where the model can be used before carrying it into conversion. |
| 5 | Add power limits and rotor/wing interactions | Use shaft power and torque data to constrain thrust, then account for how the rotor wake changes wing loads across relevant conditions. |
| 6 | Add full attitude and sideways motion | Extend the model to track rotation and sideways movement. This requires aircraft inertia, force locations, and the turning effects of those forces. |

## Recommended next step

**Check the airflow and angle-of-attack path with a small test table and a hand-calculated case.** Angle of attack (AoA) is the angle between the wing's front-to-back reference line and its direction of travel through the air. The table-driven path subtracts wind from ground velocity, finds the air-relative flight-path angle, and subtracts that angle from the wing's mounting angle to get AoA. Verify that this AoA selects the expected lift and drag coefficients and that the resulting forces point in the expected directions.

This is a manageable first task because the calculation and audit columns already exist. A test table with known values can verify the calculation without calibrated aircraft data. Rotor thrust remains the highest-priority modeling gap because it drives takeoff and the current transition carries its takeoff estimate forward. [Transition implementation notes](TMPE_TRANSITION.md)

## Current CSV limitations

The checked-in transition audit uses illustrative, unvalidated software-test assumptions. **It is not a prediction of real XV-15 performance.** All 101 rows are labeled `ILLUSTRATIVE / SOFTWARE-TEST ONLY`, and both rotors use the synthetic collective fixture. The stored thrust magnitudes stay fixed as the nacelles tilt. [Transition audit CSV](../output/stage2_audit_zwrjdnhe.csv)

- **Blank `gamma_deg` and `alpha_deg`.** The simplified path computes air-relative speed but skips the flight-path angle and AoA calculations. A blank means that the angle was left uncalculated in this run. The table-driven path calculates these angles when the airflow and wing mounting angle are available.
- **Fixed `cl = 0.50` and `cd = 0.08`.** These are dimensionless test values for lift and drag. The wing forces can change with airspeed through dynamic pressure, but the coefficients stay fixed. The simplified path applies lift upward and drag backward. The table-driven path looks up coefficients by AoA and directs the forces relative to the airflow.
- **Validation remains open.** The code labels supplied coefficient tables as unvalidated as well. A table can exercise the AoA and force calculations. Predicting XV-15 performance requires applicable aircraft data and comparison with measured results.
