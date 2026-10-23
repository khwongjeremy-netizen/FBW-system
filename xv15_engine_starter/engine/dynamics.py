"""Rigid-body motion integration utilities.

Translational motion is implemented. Rotational motion is intentionally not
integrated yet: the current dataset does not provide a complete, verified
inertia tensor for every nacelle angle, and body/world attitude conventions
must be settled before rotational equations are connected.
"""

from dataclasses import dataclass
import math

from .forces import ForceMoment, gravity_force, add
from .state import AircraftState


@dataclass(frozen=True)
class DynamicsResult:
    acceleration_world_mps2: tuple[float, float, float]
    net_force_world_n: tuple[float, float, float]


def translational_acceleration(
    mass_kg: float,
    forces: ForceMoment,
    gravity_mps2: float = 9.80665,
    include_gravity: bool = True,
) -> DynamicsResult:
    """Compute world-frame acceleration from supplied forces and gravity."""
    if not math.isfinite(mass_kg) or mass_kg <= 0:
        raise ValueError("mass_kg must be finite and greater than zero.")

    total_force = forces.force_world_n
    if include_gravity:
        total_force = add(total_force, gravity_force(mass_kg, gravity_mps2))
    acceleration = tuple(component / mass_kg for component in total_force)
    return DynamicsResult(acceleration, total_force)


def integrate_translation(
    state: AircraftState,
    acceleration_world_mps2: tuple[float, float, float],
    dt_s: float,
) -> None:
    """Advance position and velocity using semi-implicit Euler integration.

    This numerical integrator is a simple starting point, not a claim that
    the resulting model is accurate enough for flight simulation.
    """
    if not math.isfinite(dt_s) or dt_s <= 0:
        raise ValueError("dt_s must be finite and greater than zero.")
    if len(acceleration_world_mps2) != 3 or not all(
        math.isfinite(v) for v in acceleration_world_mps2
    ):
        raise ValueError("Acceleration must contain three finite numbers.")

    new_velocity = tuple(
        state.velocity_world_mps[i] + acceleration_world_mps2[i] * dt_s
        for i in range(3)
    )
    new_position = tuple(
        state.position_m[i] + new_velocity[i] * dt_s
        for i in range(3)
    )
    state.velocity_world_mps = new_velocity
    state.position_m = new_position
    state.time_s += dt_s
    state.validate()
