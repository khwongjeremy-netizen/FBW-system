# TMPE — Tiltrotor Modeling Physics Engine

An independent Python project for learning how tiltrotor physics and pilot-assistance software systems work, using the NASA/Bell XV-15 as a reference aircraft.

## Overview

TMPE is a hands-on project for building my understanding of the physical laws that shape tiltrotor flight and the software systems that can assist pilots. I use the XV-15 and its technical data as a basis for exploring those topics through code. The longer-term direction is to model aircraft forces and motion, then investigate simplified pilot-assistance and flight-by-wire (FBW) concepts. These systems are development goals; the current program does not implement them.

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

- Prints entered conditions and automatically loaded aircraft properties in an input report.

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

- TMPE has no angle-of-attack or airspeed model, so it does not invent `CL` or request it from the pilot. Wind direction is stored in degrees, but its convention is not yet defined. Environment values do not determine airspeed or lift.

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

- Enter an aircraft mode, a pilot-command label, then environmental values. Press Enter to accept displayed defaults.
- Commands and modes are recorded only; they do not control simulated aircraft behavior.

## Limitations

TMPE is **not a validated flight simulator**. The current version does not implement:

- Rotor thrust, complete aerodynamic forces, or rotor–wing aerodynamic interactions.
- A complete, verified lift-coefficient model or wing-lift calculation. The available lift-curve metadata is insufficient without angle of attack and coefficient lookup data.
- Aircraft equations of motion, position/velocity/attitude integration, or transition physics.
- Flight-by-wire laws or control of a real aircraft.
- Validation against flight-test data. XML provenance and extracted parameter citations also require further verification.

Pilot commands and flight modes are labels only; no trajectory is calculated.

## Roadmap

All items are planned, not implemented:

1. **Foundation:** Refine XML loading and reporting, aircraft/environment structures, unit consistency, and basic aerodynamic calculations.
2. **Physical modeling:** Research rotor thrust and fuller aerodynamic forces; establish coordinate frames and document missing parameters.
3. **Flight dynamics:** Add force/moment equations and translational/rotational motion; compare with appropriate published XV-15 data.
4. **Control exploration:** Separate pilot commands from aircraft dynamics and investigate control logic responding to aircraft state and pilot intent.

## Sources

- [`sources.xml`](MY_physics_engine/data(v.2)/sources.xml) records the research inventory and citation-status notes.
- NASA, [XV-15 Tiltrotor overview](https://www.nasa.gov/aeronautics/nasa-aircraft/xv-15-tiltrotor/).
- NASA, [NASA/Army XV-15 Tilt Rotor Research Aircraft Familiarization Document (NASA TM-X-62407)](https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf), cited for detailed values in the XML. Verify extracted values against the report before using them quantitatively.
