# XV-15 Engine Starter

This is an educational starter scaffold, not a validated XV-15 flight simulator.

## Layout

- `engine/state.py`: simulated aircraft state
- `engine/environment.py`: air density, wind and gravity inputs
- `engine/forces.py`: general force helpers; coefficients and thrust must be supplied
- `engine/dynamics.py`: translational equations and simple time integration
- `engine/simulation.py`: time-step orchestration
- `engine/fbw.py`: pilot-command/controller interface placeholder

## Important limitations

- SI units are used inside these starter modules.
- XML values are not automatically converted from their source units.
- The initial simulation applies gravity only unless a force model is supplied.
- No XV-15-specific thrust model, aerodynamic model, control law, or actuator model
  is invented here.
- Rotational dynamics are not integrated yet because the inertia tensor and frame
  conventions need to be resolved and verified first.
- The environment defaults are convenience values for software testing, not a
  claim about a particular XV-15 flight condition.

## Suggested project layout

Place this `engine/` directory next to your existing `data/` folder and `main.py`.
The existing `data_loader.py` can be placed inside `engine/` separately.
