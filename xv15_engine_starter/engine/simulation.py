"""Simulation orchestration.

The simulation owns the time loop and connects environment, force model and
translational dynamics. Until a validated XV-15 force model is supplied,
default operation includes gravity only; it is a software test, not flight.
"""

from dataclasses import dataclass
from typing import Callable

from .dynamics import integrate_translation, translational_acceleration
from .environment import AirEnvironment
from .forces import ForceMoment
from .state import AircraftState

ForceModel = Callable[
    [AircraftState, AirEnvironment, object],
    ForceMoment,
]


@dataclass
class Simulation:
    mass_kg: float
    state: AircraftState
    environment: AirEnvironment
    force_model: ForceModel | None = None

    def __post_init__(self) -> None:
        if self.mass_kg <= 0:
            raise ValueError("mass_kg must be greater than zero.")
        self.state.validate()
        self.environment.validate()

    def step(self, dt_s: float) -> None:
        """Advance one time step.

        A future force model receives state, environment and aircraft data.
        For now, no thrust or aerodynamic forces are assumed if no model is
        supplied.
        """
        if self.force_model is None:
            applied_forces = ForceMoment()
        else:
            # The third argument is reserved for aircraft data in the next
            # integration step; it is None in this initial scaffold.
            applied_forces = self.force_model(self.state, self.environment, None)

        result = translational_acceleration(
            mass_kg=self.mass_kg,
            forces=applied_forces,
            gravity_mps2=self.environment.gravity_mps2,
        )
        integrate_translation(self.state, result.acceleration_world_mps2, dt_s)

    def run(self, duration_s: float, dt_s: float) -> AircraftState:
        """Run for duration_s, using fixed time steps."""
        if duration_s < 0:
            raise ValueError("duration_s must be non-negative.")
        if dt_s <= 0:
            raise ValueError("dt_s must be greater than zero.")
        if duration_s == 0:
            return self.state

        steps = int(duration_s / dt_s)
        remainder = duration_s - steps * dt_s
        for _ in range(steps):
            self.step(dt_s)
        if remainder > 1e-12:
            self.step(remainder)
        return self.state
