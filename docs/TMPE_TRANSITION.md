# TMPE Phase 2A — nacelle force decomposition

Phase 1 remains the existing helicopter takeoff model. Its collective-pitch path,
fixed-CT regression mode, and safeguard against tilted-nacelle vertical-only
integration are unchanged. Terminal results appear immediately after Phase 1,
followed by Phase 2 diagnostics.

Run the fixed-CT regression with independent illustrative nacelle samples:

```sh
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' \
  --rotor-model prototype-fixed-ct --nacelle-schedule 90:90 60:45 45:45 0:0
```

The default samples are `90:90 45:45 0:0`. Each pair is left:right degrees.
Samples are untimed: no conversion rate, operational speed or altitude threshold
is implied. The provisional software range is inclusive 0–90 degrees; this is
not a documented XV-15 operating envelope or validated conversion procedure.
Blade collective and nacelle tilt remain separate quantities.

`initialize_transition(result)` creates an immutable `TransitionState` from the
last retained takeoff altitude AGL, upward velocity, elapsed time, per-rotor
thrust magnitudes and nacelle angles. Forward speed and distance start at zero
because Phase 1 is vertical only. An unsuccessful Phase 1 retains its actual
last supported state, including unknown thrust; it does not imply a successful
takeoff. Ground elevation MSL is separate from altitude AGL.

`sample_nacelle_transition(result, angle_pairs)` retains this state without
advancing time or motion. It holds the final Phase 1 thrust magnitudes solely
for a geometric demonstration. It does not extend the static rotor calibration
to conversion airflow. Each rotor uses `rotor_force_body_n()`: body x forward,
body z down. For the assumed level aircraft, forward force is body x and upward
force is negative body z. At 90 degrees thrust is vertical, at 0 horizontal,
and at 45 the components are equal. Each individual vector retains its thrust
magnitude. With unequal angles, the combined vector magnitude can be less than
the sum of the two scalar thrust magnitudes.

Every sample reports angles, held rotor thrusts, upward/forward force, weight,
and unavailable wing lift and drag. Unknown forces remain unknown, never zero.
These samples are not transition trajectories or realistic flight performance.
The original rotor model labels, calibration limitations and Phase 1 exclusions
remain visible before the Phase 2 report.

`planar_force_balance(mass_kg, forward_force_n, upward_force_n)` is the small
Phase 2B preparation interface. Arguments are **net** forces in newtons, with
gravity already included in upward net force. It returns forward and upward
acceleration in m/s² independently; unavailable inputs yield `None`. It does
not integrate state or treat rotor components alone as complete net forces.

Phase 2B must supply supported wing lift/drag data, rotor performance across
inflow and advance ratio, rotor/wing interference and other airframe loads,
and a supported treatment of attitude and moments. Independent nacelle angles
can produce moments; Phase 2A does not predict attitude. Only after those models
are supplied should force reevaluation and horizontal/vertical integration
advance forward velocity/distance, upward velocity/altitude, and elapsed time.
The current transition readiness remains BLOCKED for trajectory prediction.

Verification: `python3 -m unittest discover -s tests -q` checks angle geometry,
individual vector norms, asymmetric rotors, handoff preservation, validation,
unknown-force propagation, reporting, and the existing five-second regression.
