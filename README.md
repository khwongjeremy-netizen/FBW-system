# TMPE — Tiltrotor Modeling Physics Engine

An independent Python project for learning how aerodynamic physics and pilot-assistance software systems work on a tiltrotor, using the NASA/Bell XV-15 as a reference aircraft.

## Overview

TMPE is a hands-on project for building my understanding of the physical laws that shape tiltrotor flight and the software systems that can assist pilots. I use the XV-15 and its technical data as a basis for exploring those topics through code. The longer-term direction is to model aircraft forces and motion, then investigate simplified pilot-assistance and flight-by-wire (FBW) concepts. These systems are development goals; the current program does not implement them.

> **Current state:** Early prototype; it's supposed to calculate the freestream angle of attack from prescribed initial state, but does not simulate aircraft responses or calculate nonzero aerodynamic forces.
Much of the code and XML data is AI-generated and not systematically human-reviewed or fact-checked; many errors may remain.

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

- Prompts for a flight-mode label and pilot-command label, plus air density (kg/m³), wind speed (m/s), wind direction (degrees), temperature (°C), and altitude (m).

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

- TMPE calculates airspeed and wing AoA from prescribed aircraft ground velocity and attitude plus wind. It does not request AoA or infer motion from commands. CL/CD/Cm tables remain unavailable; nonzero-speed forces are explicitly unavailable. At exactly zero freestream speed, freestream force is zero without evaluating coefficients.

### AoA and coordinate conventions

Wing AoA is the angle between the wing chord and incoming relative airflow. It changes with flight state; it is not a fixed XV-15 parameter or an environmental input. Pitch attitude, wing mounting incidence, air-relative flight-path angle, nacelle tilt and rotor-blade pitch are distinct. Wing AoA must not be used as rotor-blade AoA. A requested climb rate is a target, not actual velocity or attitude.

World axes are North/East/Down (NED); body axes are forward/right/down. Velocities use m/s, internal angles radians and displayed/input angles degrees. Positive right-handed roll lowers the right wing, pitch raises the nose, and yaw turns north toward east. Attitude uses a yaw-pitch-roll (3-2-1) body-to-world rotation; its transpose transforms world vectors into body coordinates.

Wind direction is meteorological **FROM**, clockwise from north. With direction `d`, the horizontal wind NED vector is `(-speed*cos(d), -speed*sin(d), 0)`. Vertical wind is assumed zero. Aircraft velocity relative to air is `V_air_NED = V_ground_NED - V_wind_NED`; incoming airflow points opposite this vector. Orientation determines how the airflow meets the wing.

The reference wing frame has x forward along the chord toward the leading edge, y right and z downward for an unrotated wing. Positive XML incidence raises the leading edge. After world-to-body rotation, body-to-wing rotation is `Ry(incidence)`, so `u_w = cos(i)*u_b - sin(i)*w_b` and `w_w = sin(i)*u_b + cos(i)*w_b`. AoA is `atan2(w_w, u_w)`, preserving signs and quadrants, including reverse flow. The reported physical angle is never clamped. In planar forward flight only, this agrees with `pitch + incidence - air-relative flight-path angle`.

The single reference-wing approximation uses the XML incidence (currently 3°). Missing incidence leaves wing AoA unavailable. Panel sweep/dihedral effects, rotor wash, induced flow and angular-rate velocity at the wing are excluded. This is freestream AoA, not a local blade or panel solution.

Chord-plane speed below `1e-6 m/s` leaves AoA explicitly undefined, including zero-speed and purely spanwise flow. No invalid coefficient is evaluated. The threshold is numerical, not a physical force cutoff: nonzero-speed forces remain unavailable without coefficient data. Hover rotor wash can load the wing even when freestream force is zero; that flow is not modeled. Rotor thrust and vertical dynamics remain separate, unimplemented systems.

The verbose report traces ground/wind vectors, subtraction, prescribed attitude, both rotation matrices, body/wing velocities, incidence and the numeric `atan2` result. Initial-state defaults assume stationary, level, north-facing aircraft; these are scenario assumptions, not verified aircraft data or dynamically simulated motion.

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

- Enter an aircraft mode, a pilot-command label and environmental values. For initial conditions, answer `Use default initial state: stationary, level, facing north? [Y/n]`. Enter or `y` accepts that state; `n` asks for individual NED ground-velocity and roll/pitch/yaw values. No AoA input is needed.
- Commands and modes are recorded only; they do not control simulated aircraft behavior.

With all defaults, the normal results are:

```text
--- Results ---
Airspeed:           5.00 m/s
Wing AoA:          -177.0°
Wing lift:          Unavailable — aerodynamic model missing
Flow condition:     Reverse flow
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

Pilot commands and flight modes are labels only; no trajectory is calculated.

## Roadmap

All items are planned, not implemented:

1. **Foundation:** Refine XML loading and reporting, aircraft/environment structures, unit consistency, and basic aerodynamic calculations.
2. **Physical modeling:** Research rotor thrust and fuller aerodynamic forces; extend the documented coordinate frames and document missing parameters.
3. **Flight dynamics:** Add force/moment equations and translational/rotational motion; compare with appropriate published XV-15 data.
4. **Control exploration:** Separate pilot commands from aircraft dynamics and investigate control logic responding to aircraft state and pilot intent.

## Sources

- [`sources.xml`](MY_physics_engine/data(v.2)/sources.xml) records the research inventory and citation-status notes.
- NASA, [XV-15 Tiltrotor overview](https://www.nasa.gov/aeronautics/nasa-aircraft/xv-15-tiltrotor/).
- NASA, [NASA/Army XV-15 Tilt Rotor Research Aircraft Familiarization Document (NASA TM-X-62407)](https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf), cited for detailed values in the XML. Verify extracted values against the report before using them quantitatively.
