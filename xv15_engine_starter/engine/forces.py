"""Force and moment helpers.

Only general mechanics are implemented here. Aircraft-specific rotor thrust
and aerodynamic coefficients must be supplied from documented data/models;
this module does not invent XV-15 values.
"""

from dataclasses import dataclass
import math

Vector3 = tuple[float, float, float]


def add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(vector: Vector3, factor: float) -> Vector3:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)


def dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def magnitude(vector: Vector3) -> float:
    return math.sqrt(dot(vector, vector))


def unit(vector: Vector3) -> Vector3:
    length = magnitude(vector)
    if length == 0:
        raise ValueError("Cannot normalize a zero-length vector.")
    return scale(vector, 1.0 / length)


@dataclass(frozen=True)
class ForceMoment:
    """Net external force in world axes and moment in body axes."""

    force_world_n: Vector3 = (0.0, 0.0, 0.0)
    moment_body_nm: Vector3 = (0.0, 0.0, 0.0)


def gravity_force(mass_kg: float, gravity_mps2: float = 9.80665) -> Vector3:
    """Gravity force in a world frame where +Z points upward."""
    if not math.isfinite(mass_kg) or mass_kg <= 0:
        raise ValueError("mass_kg must be finite and greater than zero.")
    if not math.isfinite(gravity_mps2) or gravity_mps2 < 0:
        raise ValueError("gravity_mps2 must be finite and non-negative.")
    return (0.0, 0.0, -mass_kg * gravity_mps2)


def directed_thrust(thrust_n: float, direction_world: Vector3) -> Vector3:
    """Convert a supplied thrust magnitude and direction into a force vector."""
    if not math.isfinite(thrust_n) or thrust_n < 0:
        raise ValueError("thrust_n must be finite and non-negative.")
    return scale(unit(direction_world), thrust_n)


def aerodynamic_force_from_coefficients(
    air_density_kg_m3: float,
    airspeed_mps: float,
    reference_area_m2: float,
    lift_coefficient: float,
    drag_coefficient: float,
    drag_direction_world: Vector3,
    lift_direction_world: Vector3,
) -> Vector3:
    """Calculate lift/drag magnitudes from supplied coefficients.

    q = 0.5 * rho * V^2; lift = q*S*CL; drag = q*S*CD.
    Direction vectors must be supplied by a separately validated model.
    The function does not calculate XV-15 lift/drag coefficients or angle of
    attack, and should not be used as a complete aircraft aerodynamic model.
    """
    values = (air_density_kg_m3, airspeed_mps, reference_area_m2,
              lift_coefficient, drag_coefficient)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Aerodynamic inputs must be finite.")
    if air_density_kg_m3 < 0 or airspeed_mps < 0 or reference_area_m2 < 0:
        raise ValueError("Density, airspeed and reference area must be non-negative.")
    if drag_coefficient < 0:
        raise ValueError("drag_coefficient must be non-negative.")

    dynamic_pressure = 0.5 * air_density_kg_m3 * airspeed_mps**2
    lift_n = dynamic_pressure * reference_area_m2 * lift_coefficient
    drag_n = dynamic_pressure * reference_area_m2 * drag_coefficient
    lift_force = scale(unit(lift_direction_world), lift_n)
    drag_force = scale(unit(drag_direction_world), -drag_n)
    return add(lift_force, drag_force)
