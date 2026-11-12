# Default collective-mode execution evidence

Normal execution now selects prototype-collective. The software-test fixture requires explicit opt-in. Experimental XV-15 validation remains NOT VALIDATED.

## No arguments, explicit illustrative opt-in

```sh
printf '\n\n\n\n\ny\n6\n6\n' | PYTHONDONTWRITEBYTECODE=1 python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py'
```

Exit code: 0

```text
TMPE :: XV-15 HELICOPTER TAKEOFF
Scenario: XV-15 helicopter-mode vertical takeoff.
Initial state: on the ground, stationary, level, facing north.
MODE: HELICOPTER; nacelle angle: 90.0°

Enter ground-level environmental conditions.
Air density (kg/m^3) [1.23]: Wind speed (m/s) [5.00]: Wind FROM direction (degrees clockwise from north) [180.0]: Temperature (degrees C) [15.0]: Ground elevation above sea level (m) [0.00]: 
Aircraft collective model is UNCALIBRATED / UNAVAILABLE.
UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition
Illustrative fixture: /Users/JeremyWong/Desktop/FBW-system/tests/fixtures/collective_illustrative.json
Illustrative coefficients are for SOFTWARE TESTING ONLY; NOT validated XV-15 predictions.
Enable illustrative collective testing? [y/N]: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED

Collective pitch bounds: {'minimum_deg': -10.0, 'maximum_deg': 20.0, 'basis': 'provisional-software', 'source': 'Synthetic fixture input guard; NOT aircraft limits'}
Calibration source: Arbitrary synthetic software-test fixture; NOT measured XV-15 data
Pilot left collective pitch (degrees) [Enter to leave unknown]: Pilot right collective pitch (degrees) [Enter to leave unknown]: 
Model: prototype-collective
CT(theta) = CT_reference + k_theta * (theta - theta_reference)
Wing download, ground effect, rotor interference and other aerodynamic loads are EXCLUDED, not measured zero. Wind, inflow response, stall and power limits are excluded. Fixed RPM; constant selected pitch per rotor. Software mechanics exercise; NOT XV-15 aircraft predictions.
left: pilot collective = 6.0 deg; CT = 0.011; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED
right: pilot collective = 6.0 deg; CT = 0.011; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED

--- Rotor performance ---
LEFT PROPROTOR         : 31227.17 N
RIGHT PROPROTOR        : 31227.17 N
TOTAL VERTICAL         : 62454.35 N

--- Aircraft response (up positive) ---
TAKEOFF MASS           : 5896.70 kg
AIRCRAFT WEIGHT        : 57826.88 N
NET VERTICAL FORCE     : 4627.47 N
VERTICAL ACCEL.        : 0.78 m/s²
VERTICAL VELOCITY      : 3.92 m/s
HEIGHT ABOVE GROUND    : 9.81 m
TAKEOFF STATUS         : AIRBORNE
Simulated time         : 5.00 s
Educational prototype; no validated XV-15 rotor performance model.
```

## No arguments, decline illustrative testing

```sh
printf '\n\n\n\n\nn\n6\n6\n' | PYTHONDONTWRITEBYTECODE=1 python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py'
```

Exit code: 0

```text
TMPE :: XV-15 HELICOPTER TAKEOFF
Scenario: XV-15 helicopter-mode vertical takeoff.
Initial state: on the ground, stationary, level, facing north.
MODE: HELICOPTER; nacelle angle: 90.0°

Enter ground-level environmental conditions.
Air density (kg/m^3) [1.23]: Wind speed (m/s) [5.00]: Wind FROM direction (degrees clockwise from north) [180.0]: Temperature (degrees C) [15.0]: Ground elevation above sea level (m) [0.00]: 
Aircraft collective model is UNCALIBRATED / UNAVAILABLE.
UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition
Illustrative fixture: /Users/JeremyWong/Desktop/FBW-system/tests/fixtures/collective_illustrative.json
Illustrative coefficients are for SOFTWARE TESTING ONLY; NOT validated XV-15 predictions.
Enable illustrative collective testing? [y/N]: 
Collective pitch bounds: {'minimum_deg': -30.0, 'maximum_deg': 30.0, 'basis': 'provisional-software', 'source': 'TMPE software input guard only; not XV-15 mechanical, aerodynamic or flight limits'}
Calibration source: Repository data audit: no paired collective/thrust measurements supplied
Pilot left collective pitch (degrees) [Enter to leave unknown]: Pilot right collective pitch (degrees) [Enter to leave unknown]: 
Model: prototype-collective
CT(theta) = CT_reference + k_theta * (theta - theta_reference)
Wing download, ground effect, rotor interference and other aerodynamic loads are EXCLUDED, not measured zero. Wind, inflow response, stall and power limits are excluded. Fixed RPM; constant selected pitch per rotor. Software mechanics exercise; NOT XV-15 aircraft predictions.
left: pilot collective = 6.0 deg; CT = None; UNAVAILABLE
  Collective calculation: UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition
  Calibration: UNCALIBRATED; experimental validation: NOT VALIDATED
right: pilot collective = 6.0 deg; CT = None; UNAVAILABLE
  Collective calculation: UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition
  Calibration: UNCALIBRATED; experimental validation: NOT VALIDATED

--- Rotor performance ---
LEFT PROPROTOR         : Unavailable — rotor model inputs missing
RIGHT PROPROTOR        : Unavailable — rotor model inputs missing
TOTAL VERTICAL         : Unavailable — rotor model inputs missing

--- Aircraft response (up positive) ---
TAKEOFF MASS           : 5896.70 kg
AIRCRAFT WEIGHT        : 57826.88 N
NET VERTICAL FORCE     : Unavailable — force balance unavailable
VERTICAL ACCEL.        : Unavailable — force balance unavailable
VERTICAL VELOCITY      : 0.00 m/s
HEIGHT ABOVE GROUND    : 0.00 m
TAKEOFF STATUS         : UNAVAILABLE
Simulated time         : 0.00 s
Initial state retained; takeoff not simulated. Initial velocity/height are not trajectory predictions.
Blockers: UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition; UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition; Both rotor thrusts, nacelle angles and explicit other vertical loads are required
Educational prototype; no validated XV-15 rotor performance model.
```

## No arguments, reject invalid pitch and recover

```sh
printf '\n\n\n\n\ny\nnan\n21\n6\n6\n' | PYTHONDONTWRITEBYTECODE=1 python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py'
```

Exit code: 0

```text
TMPE :: XV-15 HELICOPTER TAKEOFF
Scenario: XV-15 helicopter-mode vertical takeoff.
Initial state: on the ground, stationary, level, facing north.
MODE: HELICOPTER; nacelle angle: 90.0°

Enter ground-level environmental conditions.
Air density (kg/m^3) [1.23]: Wind speed (m/s) [5.00]: Wind FROM direction (degrees clockwise from north) [180.0]: Temperature (degrees C) [15.0]: Ground elevation above sea level (m) [0.00]: 
Aircraft collective model is UNCALIBRATED / UNAVAILABLE.
UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition
Illustrative fixture: /Users/JeremyWong/Desktop/FBW-system/tests/fixtures/collective_illustrative.json
Illustrative coefficients are for SOFTWARE TESTING ONLY; NOT validated XV-15 predictions.
Enable illustrative collective testing? [y/N]: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED

Collective pitch bounds: {'minimum_deg': -10.0, 'maximum_deg': 20.0, 'basis': 'provisional-software', 'source': 'Synthetic fixture input guard; NOT aircraft limits'}
Calibration source: Arbitrary synthetic software-test fixture; NOT measured XV-15 data
Pilot left collective pitch (degrees) [Enter to leave unknown]: Please enter a finite number.
Pilot left collective pitch (degrees) [Enter to leave unknown]: Collective outside provisional-software bounds [-10.0, 20.0] deg
Pilot left collective pitch (degrees) [Enter to leave unknown]: Pilot right collective pitch (degrees) [Enter to leave unknown]: 
Model: prototype-collective
CT(theta) = CT_reference + k_theta * (theta - theta_reference)
Wing download, ground effect, rotor interference and other aerodynamic loads are EXCLUDED, not measured zero. Wind, inflow response, stall and power limits are excluded. Fixed RPM; constant selected pitch per rotor. Software mechanics exercise; NOT XV-15 aircraft predictions.
left: pilot collective = 6.0 deg; CT = 0.011; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED
right: pilot collective = 6.0 deg; CT = 0.011; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED

--- Rotor performance ---
LEFT PROPROTOR         : 31227.17 N
RIGHT PROPROTOR        : 31227.17 N
TOTAL VERTICAL         : 62454.35 N

--- Aircraft response (up positive) ---
TAKEOFF MASS           : 5896.70 kg
AIRCRAFT WEIGHT        : 57826.88 N
NET VERTICAL FORCE     : 4627.47 N
VERTICAL ACCEL.        : 0.78 m/s²
VERTICAL VELOCITY      : 3.92 m/s
HEIGHT ABOVE GROUND    : 9.81 m
TAKEOFF STATUS         : AIRBORNE
Simulated time         : 5.00 s
Educational prototype; no validated XV-15 rotor performance model.
```

## Explicit fixed-CT regression mode

```sh
printf '\n\n\n\n\n' | PYTHONDONTWRITEBYTECODE=1 python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --rotor-model prototype-fixed-ct
```

Exit code: 0

```text
TMPE :: XV-15 HELICOPTER TAKEOFF
Scenario: XV-15 helicopter-mode vertical takeoff.
Initial state: on the ground, stationary, level, facing north.
MODE: HELICOPTER; nacelle angle: 90.0°

Enter ground-level environmental conditions.
Air density (kg/m^3) [1.23]: Wind speed (m/s) [5.00]: Wind FROM direction (degrees clockwise from north) [180.0]: Temperature (degrees C) [15.0]: Ground elevation above sea level (m) [0.00]: 
PROTOTYPE ESTIMATE — FIXED CT, FIXED RPM
Model: prototype-fixed-ct; CT source status: illustrative assumption, not verified XV-15 data.
Wing download, ground effect, rotor interference and other aerodynamic loads are excluded from this force balance, not established as physically zero. Entered wind is excluded from rotor thrust and motion; no windy-hover claim. Fixed CT/RPM do not respond to collective, inflow, climb speed or active control. No realistic climb performance or engine power demand is predicted.
left: rotor.xml: proprotor_and_drive_system/prototype_thrust_model/thrust_coefficient | T = CT rho A (Omega R)^2; PROTOTYPE ESTIMATE — FIXED CT, FIXED RPM
  Inputs: {'rotor_diameter_ft': 25.0, 'hover_rotor_rpm': 565.0, 'air_density': 1.225, 'thrust_coefficient': 0.0102}
right: rotor.xml: proprotor_and_drive_system/prototype_thrust_model/thrust_coefficient | T = CT rho A (Omega R)^2; PROTOTYPE ESTIMATE — FIXED CT, FIXED RPM
  Inputs: {'rotor_diameter_ft': 25.0, 'hover_rotor_rpm': 565.0, 'air_density': 1.225, 'thrust_coefficient': 0.0102}

--- Rotor performance ---
LEFT PROPROTOR         : 28956.11 N
RIGHT PROPROTOR        : 28956.11 N
TOTAL VERTICAL         : 57912.21 N

--- Aircraft response (up positive) ---
TAKEOFF MASS           : 5896.70 kg
AIRCRAFT WEIGHT        : 57826.88 N
NET VERTICAL FORCE     : 85.33 N
VERTICAL ACCEL.        : 0.01 m/s²
VERTICAL VELOCITY      : 0.07 m/s
HEIGHT ABOVE GROUND    : 0.18 m
Prototype force excess / weight: 0.148% (excluded-load assumption).
Almost weight-balanced; no substantial takeoff margin. Any motion is a simplified mechanics result.
TAKEOFF STATUS         : AIRBORNE
Simulated time         : 5.00 s
Educational prototype; no validated XV-15 rotor performance model.
```

## Automated suite

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -q
```

Exit code: 0

```text
----------------------------------------------------------------------
Ran 57 tests in 0.559s

OK
```
