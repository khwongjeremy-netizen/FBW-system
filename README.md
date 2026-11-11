# TMPE — Tiltrotor Modeling Physics Engine

- Educational XV-15 helicopter-takeoff prototype; not a validated flight simulator.
- Every run starts on the ground, stationary, level and facing north.
- Inputs remain air density, wind speed, wind FROM direction, temperature and ground elevation. Press Enter for defaults.
- Calculates freestream airspeed and wing AoA. Rotor thrust and vertical dynamics are **not implemented**, so takeoff is not simulated.
- Standard-library Python 3; no dependency installation needed.

## Run

From the repository root:

```bash
# Normal: concise results and limitations
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py'

# Diagnostics: parameter values, units, sources, failures and physics blockers
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --diagnostics

# Verbose: environment, aircraft values and full AoA calculation trace
python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --verbose
```

- Combine `--diagnostics --verbose` when both reports are useful.
- All modes use the same inputs and calculations; display rounding never changes internal precision.

## Data and diagnostics

- Aircraft data lives in `MY_physics_engine/data(v.2)`; the program loads selected mass, geometry and rotor fields.
- `Aircraft.diagnostics` maps parameter names to records containing meaning, XML file/path, value, units, status, reason, citation metadata and live-physics usage.
- `LOADED` means parsing succeeded, not that independent verification was performed. `UNVERIFIED` preserves explicit XML verification flags.
- `MISSING` identifies absent files/elements/attributes and intentionally unavailable fields. `INVALID` identifies malformed XML, incompatible units or invalid numbers.
- A failed item remains `None`; other parameters continue loading. Normal mode points to diagnostics if loading fails.
- Geometry recovery retains per-item source references in XML and the [original NotebookLM response](MY_physics_engine/data(v.2)/notebooklm_geometry_recovery_2026-11-10.md).
- Rotor hub positions describe conversion tilt axes in FS/BL/WL datum feet (aft/right/up), not CG body coordinates. The conflicting candidate CG is not selected.
- Exterior width remains a 5.0–5.2 ft range; no scalar width is assumed. Exterior depth is the supplied 74 inches converted to feet.

## Physics conventions and limits

- World axes: North/East/Down; body axes: forward/right/down. Internal angles use radians; inputs and printed angles use degrees.
- Wind direction is meteorological FROM, clockwise from north; vertical wind is zero.
- Air-relative velocity is ground velocity minus wind. Wing incidence is applied once through the body-to-wing rotation; AoA uses `atan2(w, u)` without clamping.
- Undefined AoA, unavailable coefficients and exactly zero freestream force remain distinct. Rotor wash is excluded from the zero-force result.
- No supported CL/CD/Cm lookup model is available. Unsupported flow directions are identified separately from missing coefficients.
- Transition, airplane flight, landing and flight-by-wire are outside the current task.
- XML citations are preserved; many supplied values still require primary-source verification. No new physics or aircraft values are inferred.

## Maintenance and tests

- [Debugging guide](docs/TMPE_DEBUGGING.md): loading flow, record structure and failure examples.
- [Physics and variable reference](docs/TMPE_REFERENCE.md): equations, frames, variable dictionary and model boundaries.
- [Source registry](MY_physics_engine/data(v.2)/sources.xml): report identities and citation notes.

```bash
python3 -m unittest discover -s tests -v
```
