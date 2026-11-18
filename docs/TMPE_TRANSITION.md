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

The table is marked user-supplied and unvalidated unless `--polar-source`
provides source metadata. In that case the audit records the supplied reference
and labels it sourced. XV-15 validation is a separate question in either case.
The program does not infer CL or CD from the XML curve descriptions.

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

The console prints the equations, assumptions, per-step forces and both stage
times. Evaluated rows are also saved to a unique `output/stage2_audit_*.csv`
in the working directory. Use `--transition-audit` to choose a new file path.
The CSV records phase and cumulative times, flow, loads, motion, and per-rotor
and aerodynamic provenance. Blank numerical cells represent unavailable
quantities. A preflight block before any step is reported on the console.
