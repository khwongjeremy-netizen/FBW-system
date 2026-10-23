"""Environmental conditions supplied to the simulation.

No weather model is assumed. Defaults represent still air and standard
near-sea-level density only as a convenient software starting condition;
they are not a claim about the conditions of any particular flight.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AirEnvironment:
    """Environment snapshot; SI units throughout."""

    air_density_kg_m3: float = 1.225
    wind_velocity_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    temperature_k: float | None = None
    pressure_pa: float | None = None
    gravity_mps2: float = 9.80665

    def validate(self) -> None:
        if not math.isfinite(self.air_density_kg_m3) or self.air_density_kg_m3 < 0:
            raise ValueError("Air density must be finite and non-negative.")
        if len(self.wind_velocity_world_mps) != 3 or not all(
            math.isfinite(v) for v in self.wind_velocity_world_mps
        ):
            raise ValueError("Wind velocity must contain three finite numbers.")
        if not math.isfinite(self.gravity_mps2) or self.gravity_mps2 < 0:
            raise ValueError("Gravity must be finite and non-negative.")
        if self.temperature_k is not None and (
            not math.isfinite(self.temperature_k) or self.temperature_k <= 0
        ):
            raise ValueError("Temperature must be positive Kelvin when provided.")
        if self.pressure_pa is not None and (
            not math.isfinite(self.pressure_pa) or self.pressure_pa < 0
        ):
            raise ValueError("Pressure must be non-negative when provided.")

    def relative_air_velocity(
        self, aircraft_velocity_world_mps: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """Aircraft velocity relative to moving air, in the world frame."""
        self.validate()
        return tuple(
            aircraft_velocity_world_mps[i] - self.wind_velocity_world_mps[i]
            for i in range(3)
        )
