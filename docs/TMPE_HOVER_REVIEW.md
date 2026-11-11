# TMPE rotor thrust review and ideal hover estimate

## Finding

A conditional isolated actuator-disk hover estimate is possible from existing XML geometry and rated engine power. Actual XV-15 thrust is not determined by those inputs. `calculate_proprotor()` therefore still reports unavailable; the opt-in estimate does not feed ground-start motion. Existing vertical force balance and integration are retained and tested independently.

## Existing XML inventory

| Quantity | Supplied value | XML location |
| --- | --- | --- |
| Diameter, each rotor | 25 ft (7.62 m) | rotor.xml, proprotor_and_drive_system/rotor_diameter |
| Area, each rotor | 491 ft², rounded | rotor.xml, rotor_disc_area_per_rotor |
| Radius | 12.5 ft, supplied unverified | rotor.xml supplemental_rotor_geometry; geometry.xml notebooklm_geometry_recovery |
| Design hover / cruise RPM | 565 / 458 | rotor.xml, design_rotor_speed |
| Reported flight-test hover / cruise RPM | 589 / 517 | rotor.xml, test_rotor_speed; distinct operating conditions |
| Per-engine contingency rating | 1802 SHP, 2 min | engine.xml, engine_data/power_rating |
| Per-engine takeoff rating | 1550 SHP, 10 min | same; sea-level standard day |
| Per-engine military rating | 1401 SHP, 30 min | same |
| Per-engine continuous rating | 1250 SHP | same; naming discrepancy retained in XML |
| Engine count | 2 | aircraft.xml, aircraft_overview/engine_count |
| Engine-to-mast gear ratio | 35.11:1 | rotor.xml, gear_ratio_engine_to_rotor_mast |
| Interconnect ratio | 3.10:1 | rotor.xml, gear_ratio_interconnect_shaft |
| Reported transmission efficiency | 0.93 | relationships.xml, transmission_efficiency equation |

These are existing dataset claims, not independently revalidated aircraft data. The structured `efficiency` child added beside the original equation normalizes its existing 0.93 value; it adds no new empirical parameter. Source S2 and section/page metadata remain inherited in the diagnostic ledger. Gear ratios transform speed and torque; they are not efficiencies and cannot create power. RPM does not enter the ideal actuator-disk thrust equation and cannot determine thrust without a rotor performance law. No choice between design and flight-test RPM is silently made.

## Minimum inputs and power meanings

The minimal ideal-hover inversion needs disk area A, positive density rho, and explicitly specified ideal induced power Pi for each rotor. Diameter determines area, and Environment provides density independently of temperature/elevation. The XML does not provide actual commanded Pi.

If starting from engine power, additional explicit choices are required: actual operating power per engine, accessory/drive losses, power allocation between masts (including crossfeed), and the relationship between mast power and ideal induced power. The API requires engine power, transmission efficiency and an ideal-power fraction per side. None defaults to an aircraft efficiency. Missing values return unavailable.

* Engine power is mechanical output at the engine shaft. A rating describes an operating limit under stated conditions, not a command or a measured output.
* Rotor shaft power is mechanical input at a rotor mast after the specified transmission loss/allocation.
* Ideal induced power is the minimum actuator-disk airflow power associated with a thrust. Real induced power can be larger because of nonuniform inflow and other effects; mast power also includes blade profile and other aerodynamic losses.
* Calculated thrust is an output in newtons, not power and not aircraft net lift.

The API fraction Pi_ideal/Pshaft is a figure-of-merit style closure in hover. It is not a measured fraction of real induced power. No supported value is available in the dataset. A validated model could supply profile power and an induced loss factor instead. The optional scenario explicitly uses fraction = 1, the lossless rotor limit, and conditionally uses the reported 0.93 transmission ratio. It assumes each engine supplies its full takeoff rating to its corresponding mast. It assumes no accessory deduction, crossfeed or additional transmission limits. This is a theoretical ceiling at that assigned power, not a demonstrated achievable XV-15 thrust or a full aircraft upper bound.

## Equations and idealizations

Mechanical SHP converts to watts with 745.6998715822702 W/SHP (550 ft·lbf/s). The scenario uses:

1. Pshaft = eta_transmission Pengine, separately for left and right; this assumes the stated per-side allocation.
2. Pi = fraction Pshaft, an explicit ideal power closure.
3. A = pi (D/2)², converting feet to metres using 0.3048. The exact circular area is used instead of mixing it with rounded XML area 491 ft².
4. In static hover, momentum conservation gives T = 2 rho A vi²; far-wake speed is twice disk induced speed vi.
5. Ideal airflow power Pi = T vi, giving Pi = T^(3/2)/sqrt(2 rho A).
6. Inverting gives T = (Pi sqrt(2 rho A))^(2/3); vi = sqrt(T/(2 rho A)). Zero power produces zero thrust without division by zero.

Reference for hover actuator-disk theory: [NASA rotorcraft theory](https://rotorcraft.arc.nasa.gov/Publications/files/NASA%20TP-2009-215402-app3.pdf). The implementation assumes steady, still-air, incompressible flow; uniform disk loading; isolated rotors; out-of-ground-effect hover; no swirl, finite-blade, tip, compressibility, profile, interaction or wing-download model. It checks finite inputs, positive diameter/density, nonnegative power and fractions in (0,1]. This static estimate is not applicable as a climb or descent force law. Entered wind is explicitly excluded from this separate ideal scenario. Ground-start motion would also require a supported ground-effect/interference model or explicit exclusions.

## Independent vertical mechanics

For a level body and nacelle angle beta (0 forward, 90 vertical), rotor force in forward/right/down body axes is (T cos beta, 0, -T sin beta). Taking up positive:

Fnet = TL sin betaL + TR sin betaR + Fother - mg; a = Fnet/m.

Other forces must be supplied explicitly; unknown is never zero. Weight uses G = 9.80665 m/s² and selected XML design-gross mass conversion lb × 0.45359237 kg/lb. This is a selected loading, not measured aircraft mass. Ground support cancels downward net force at rest. Constant-force integration uses h_next = h + v dt + a dt²/2 and v_next = v + a dt. Ground crossings clamp height and downward velocity to zero; no impact model is claimed.

Tests use an arbitrary 10 kg body with hypothetical equal rotor thrusts totaling 0.8, 1.0 and 1.2 times weight. The resulting free vertical accelerations are -0.2g, 0 and +0.2g. These are mechanics examples, not XV-15 predictions. Existing tests also check asymmetric thrust projection, explicit other loads, ground support and an analytic constant-acceleration trajectory using synthetic forces.

## Usage and replacement boundary

Run the regular script with `--ideal-rated-hover --diagnostics` for a separately labelled conditional estimate and the existing source ledger. The regular takeoff result remains unavailable. No synthetic thrust inputs are added to aircraft XML.

`ideal_hover_from_induced_power()` is independent of aircraft data. `estimate_proprotor_hover()` performs explicit per-side power bookkeeping. `calculate_proprotor()` remains the future validated aircraft-model boundary; `simulate_takeoff(force_provider=...)` accepts independently established forces at each step. A validated XV-15 rotor model can replace that provider without changing the vertical mechanics. It must establish operating pitch/RPM, thrust/torque behavior, power constraints and applicable wake/ground loads.

Validation: `python3 -m unittest discover -s tests -v` checks a hypothetical disk with analytic T=36 N and Pi=108 W, power identity, power scaling, unequal side powers, unavailable/invalid inputs, opt-in reporting, and vertical mechanics. No numerical result is validated against XV-15 flight or rotor-test data.
