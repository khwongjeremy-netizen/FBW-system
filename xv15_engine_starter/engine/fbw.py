"""Initial fly-by-wire interface.

This module defines command and output data structures only. It does not
implement XV-15 control laws, control mixing, actuator limits, or protections.
Those require documented control-system behavior and validated parameters.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PilotCommand:
    """Normalized pilot inputs for the future control model.

    Normalized values are interface placeholders, not real control travel.
    """
    throttle: float = 0.0
    longitudinal: float = 0.0
    lateral: float = 0.0
    yaw: float = 0.0
    nacelle_command_rad: float | None = None

    def validate(self) -> None:
        for name in ("throttle", "longitudinal", "lateral", "yaw"):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite.")
        if not 0.0 <= self.throttle <= 1.0:
            raise ValueError("throttle must be in the range [0, 1].")
        for name in ("longitudinal", "lateral", "yaw"):
            if not -1.0 <= getattr(self, name) <= 1.0:
                raise ValueError(f"{name} must be in the range [-1, 1].")
        if self.nacelle_command_rad is not None and not math.isfinite(
            self.nacelle_command_rad
        ):
            raise ValueError("nacelle_command_rad must be finite when provided.")


@dataclass(frozen=True)
class ControlOutput:
    """Commands passed from a controller to an aircraft/actuator model."""

    throttle: float
    longitudinal: float
    lateral: float
    yaw: float
    nacelle_command_rad: float | None = None


class FlyByWireController:
    """Safe pass-through placeholder until real control laws are sourced."""

    def update(self, command: PilotCommand, dt_s: float) -> ControlOutput:
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be finite and greater than zero.")
        command.validate()
        # Deliberately no invented control laws or actuator behavior.
        return ControlOutput(
            throttle=command.throttle,
            longitudinal=command.longitudinal,
            lateral=command.lateral,
            yaw=command.yaw,
            nacelle_command_rad=command.nacelle_command_rad,
        )
