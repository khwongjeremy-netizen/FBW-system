# TMPE — Tiltrotor Modeling Physics Engine

An educational Python project focused on the NASA/Bell XV-15 generating rotor lift and taking off vertically in helicopter mode.

## Overview

The current task is limited to helicopter-mode lift generation and vertical takeoff. Every run begins on the ground, stationary, level and facing north. Transition, airplane flight, landing and flight-by-wire systems are outside this task.

**Current state:** early prototype. The program calculates freestream wing angle of attack from the fixed ground-start state and wind. Rotor thrust, nonzero aerodynamic forces and vertical takeoff motion are not implemented yet. XML values and generated code still require systematic verification.

## Ground-start takeoff scenario

- **Mode:** helicopter only; no mode selection.
- **Starting position:** on the ground, at zero height above ground.
- **Starting motion:** stationary, level and facing north; no airborne or custom initial-state option.
- **Inputs:** ground-level environmental conditions only. Ground elevation above sea level does not change the initial height above ground.
- **Project task:** model rotor lift generation and vertical liftoff. Transition, airplane flight, landing and flight-by-wire are outside the current scope.

The current run reports initial wing airflow and leaves the aircraft in its ground state. It explicitly reports takeoff as not simulated until rotor thrust and vertical motion are implemented.

## Current features

**Implemented**

- Loads selected XV-15 parameters from
    - `mass.xml`,
    - `geometry.xml`,
    - `rotor.xml`
    with
    - `xml.etree.ElementTree`
    - `pathlib`

- `fetchvalue()` looks up numeric XML values by `name` attribute

- `fetchpara()` reads a tag path or attribute.

- `try_load()` warns on missing or invalid values and leaves them unknown (`None`).

- Uses a fixed helicopter-mode ground start. Prompts only for air density (kg/m³), wind speed (m/s), wind FROM direction (degrees), temperature (°C), and ground elevation above sea level (m). Ground elevation does not put the aircraft in the air; initial height above ground is zero.

- Prints a concise results report; `--verbose` adds entered conditions, aircraft properties and the full AoA calculation trace.

**Partial**

- `Aircraft` and `Environment` hold selected parameters and inputs. The program reports these values but does not use most of them in a physical model.

- Wing area is read from `geometry.xml` in ft²; the `Aircraft.wing_area_m2` property converts it to m² for future SI calculations. Other loaded fields retain XML units, including design gross weight in lb and geometry in ft.

- The broader XML dataset contains additional research fields, but most are not loaded by the program.

## Technical approach

The intended wing-lift calculation is:

```
L = 0.5 * ρ * V² * S * CL
```

- `L`: wing lift (N)
- `ρ`: air density (kg/m³)
- `V`: airspeed (m/s)
- `S`: wing reference area (m²)
- `CL`: dimensionless lift coefficient

- `calculate_lift()` implements the equation, and `S` loads from XML, but the interactive program does not call the helper yet. `aerodynamics.xml` includes approximate lift-curve slope and zero-lift-angle metadata, but no complete verified `CL(α)` lookup data.

- TMPE calculates airspeed and wing AoA from the fixed stationary, level, north-facing ground state plus wind. It does not request AoA, pilot commands or a custom initial state. CL/CD/Cm tables remain unavailable; nonzero-speed forces are explicitly unavailable. At exactly zero freestream speed, freestream force is zero without evaluating coefficients.

### AoA and coordinate conventions

Wing AoA is the angle between the wing chord and incoming relative airflow. It changes with flight state; it is not a fixed XV-15 parameter or an environmental input. Pitch attitude, wing mounting incidence, air-relative flight-path angle, nacelle tilt and rotor-blade pitch are distinct. Wing AoA must not be used as rotor-blade AoA.

World axes are North/East/Down (NED); body axes are forward/right/down. Velocities use m/s, internal angles radians and displayed/input angles degrees. Positive right-handed roll lowers the right wing, pitch raises the nose, and yaw turns north toward east. Attitude uses a yaw-pitch-roll (3-2-1) body-to-world rotation; its transpose transforms world vectors into body coordinates.

Wind direction is meteorological **FROM**, clockwise from north. With direction `d`, the horizontal wind NED vector is `(-speed*cos(d), -speed*sin(d), 0)`. Vertical wind is assumed zero. Aircraft velocity relative to air is `V_air_NED = V_ground_NED - V_wind_NED`; incoming airflow points opposite this vector. Orientation determines how the airflow meets the wing.

The reference wing frame has x forward along the chord toward the leading edge, y right and z downward for an unrotated wing. Positive XML incidence raises the leading edge. After world-to-body rotation, body-to-wing rotation is `Ry(incidence)`, so `u_w = cos(i)*u_b - sin(i)*w_b` and `w_w = sin(i)*u_b + cos(i)*w_b`. AoA is `atan2(w_w, u_w)`, preserving signs and quadrants, including reverse flow. The reported physical angle is never clamped. In planar forward flight only, this agrees with `pitch + incidence - air-relative flight-path angle`.

The single reference-wing approximation uses the XML incidence (currently 3°). Missing incidence leaves wing AoA unavailable. Panel sweep/dihedral effects, rotor wash, induced flow and angular-rate velocity at the wing are excluded. This is freestream AoA, not a local blade or panel solution.

Chord-plane speed below `1e-6 m/s` leaves AoA explicitly undefined, including zero-speed and purely spanwise flow. No invalid coefficient is evaluated. The threshold is numerical, not a physical force cutoff: nonzero-speed forces remain unavailable without coefficient data. Hover rotor wash can load the wing even when freestream force is zero; that flow is not modeled. Rotor thrust and vertical dynamics remain separate, unimplemented systems.

The verbose report traces ground/wind vectors, subtraction, prescribed attitude, both rotation matrices, body/wing velocities, incidence and the numeric `atan2` result. Every interactive run uses a fixed stationary, level, north-facing ground state; these are scenario assumptions, not dynamically simulated motion.

- XML values include units and source metadata where available. Unknown values are not filled by guesswork; source notes flag data that still needs checking against original reports.

## Project structure

```text
MY_physics_engine/
├── tiltrotor_modeling_physics_engine(TMPE).py
└── data(v.2)/                 # XML aircraft data and source/review notes
    ├── mass.xml, geometry.xml, rotor.xml
    ├── aerodynamics.xml, controls.xml, control_response.xml
    ├── engine.xml, inertia.xml, performance.xml
    ├── relationships.xml, systems.xml, wake_interference.xml
    ├── aircraft.xml, sources.xml
    └── review/re-interation/  # data_gaps.xml, review_flags.xml
```

The repository also includes `data(v.2).zip`. The program reads the data directory beside its Python file.

## Installation & usage

- Requires Python 3; uses only the standard library. No package installation is needed.
- Keep `MY_physics_engine/data(v.2)` in place, then run from the repository root:

  ```bash
  python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py'
  ```

- Enter ground-level environmental values, or press Enter to accept each default. There are no mode, command or initial-state choices.
- The program reports initial wing airflow and explicitly reports takeoff as not simulated. Rotor thrust and vertical motion are still missing.

With all defaults, the normal results are:

```text
--- Results ---
Airspeed:           5.00 m/s
Wing AoA:          -177.0°
Wing lift:          Unavailable — aerodynamic model missing
Flow condition:     Reverse flow

Takeoff status: not simulated — rotor thrust and vertical motion model missing.
Aircraft remains at its initial ground state; height above ground = 0.00 m.
Wing results describe freestream airflow only; they do not determine rotor lift or liftoff.
Educational prototype; not a validated flight simulator.
```

Assumptions and model limitations remain visible in a few short lines. For the complete numerical trace, initial state used, aircraft data and environmental conditions, run:

```bash
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --verbose
```

Display formatting uses two decimal places for velocities, dimensions, area, density and forces, and one for angles and temperature. Tiny displayed values and negative zero are shown as zero. Verbose matrices retain extra display precision and numerical thresholds use scientific notation. Internal values, physics and coefficient-validity checks retain full precision; a tiny nonzero speed rounded to `0.00 m/s` does not become a zero-speed force calculation. Missing results display `Unavailable` with a short reason.

## Limitations

TMPE is **not a validated flight simulator**. The current version also does not implement:

- Rotor thrust, complete aerodynamic forces, or rotor–wing aerodynamic interactions.
- A complete, verified lift-coefficient model or wing-lift calculation. AoA is calculated, but the available curve metadata does not supply supported coefficient lookup data.
- Aircraft equations of motion, position/velocity/attitude integration, or transition physics.
- Flight-by-wire laws or control of a real aircraft.
- Validation against flight-test data. XML provenance and extracted parameter citations also require further verification.

The scenario is fixed to helicopter-mode ground takeoff; no takeoff trajectory is calculated yet.

## Roadmap

All items are planned, not implemented, and serve the ground-to-liftoff task:

1. Establish supported helicopter-mode rotor thrust data, rotor settings and operating limits.
2. Define takeoff mass/loading and convert aircraft weight to SI units.
3. Calculate upward rotor thrust and other vertical loads, then compare them with weight. Wing freestream lift alone does not establish takeoff capability.
4. Model ground support before liftoff and vertical acceleration, velocity and height after liftoff.
5. Validate thrust and vertical takeoff against supported XV-15 reference cases.

## Sources

- [`sources.xml`](MY_physics_engine/data(v.2)/sources.xml) records the research inventory and citation-status notes.
- NASA, [XV-15 Tiltrotor overview](https://www.nasa.gov/aeronautics/nasa-aircraft/xv-15-tiltrotor/).
- NASA, [NASA/Army XV-15 Tilt Rotor Research Aircraft Familiarization Document (NASA TM-X-62407)](https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf), cited for detailed values in the XML. Verify extracted values against the report before using them quantitatively.
