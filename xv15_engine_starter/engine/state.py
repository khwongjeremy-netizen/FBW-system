"""Aircraft state definitions.

Coordinate convention for this starter:
- Position and linear velocity use a right-handed world Cartesian frame.
- World +Z points upward.
- Attitude is stored as roll, pitch, yaw in radians.
- Angular velocity and applied moments are expressed in the body frame.
- The world/body transform convention must be made explicit before adding
  flight-dynamics models that depend on aircraft-specific axes.
All distances are metres, time is seconds, mass is kilograms, forces are
newtons, moments are newton-metres, and angles are radians.
"""

from dataclasses import dataclass
import math


@dataclass
class AircraftState:
    """Instantaneous simulated aircraft state."""

    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    attitude_euler_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity_body_radps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    nacelle_angle_rad: float = math.pi / 2
    time_s: float = 0.0

    def validate(self) -> None:
        """Raise ValueError if a state contains invalid or non-finite values."""
        vectors = {
            "position_m": self.position_m,
            "velocity_world_mps": self.velocity_world_mps,
            "attitude_euler_rad": self.attitude_euler_rad,
            "angular_velocity_body_radps": self.angular_velocity_body_radps,
        }
        for name, vector in vectors.items():
            if len(vector) != 3 or not all(math.isfinite(v) for v in vector):
                raise ValueError(f"{name} must contain three finite numbers.")
        if not math.isfinite(self.nacelle_angle_rad):
            raise ValueError("nacelle_angle_rad must be finite.")
        if not math.isfinite(self.time_s) or self.time_s < 0:
            raise ValueError("time_s must be finite and non-negative.")
