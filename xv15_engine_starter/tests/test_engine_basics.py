"""Small smoke tests for the starter modules; run with python -m unittest."""
import unittest

from engine.environment import AirEnvironment
from engine.forces import ForceMoment, gravity_force, directed_thrust
from engine.dynamics import translational_acceleration, integrate_translation
from engine.state import AircraftState
from engine.fbw import FlyByWireController, PilotCommand


class EngineBasicsTests(unittest.TestCase):
    def test_gravity_points_down_in_z_up_world(self):
        self.assertEqual(gravity_force(2.0, 10.0), (0.0, 0.0, -20.0))

    def test_translational_acceleration_includes_gravity(self):
        result = translational_acceleration(2.0, ForceMoment(), gravity_mps2=10.0)
        self.assertEqual(result.acceleration_world_mps2, (0.0, 0.0, -10.0))

    def test_integrator_updates_state(self):
        state = AircraftState()
        integrate_translation(state, (0.0, 0.0, -10.0), 0.1)
        self.assertAlmostEqual(state.velocity_world_mps[2], -1.0)
        self.assertAlmostEqual(state.position_m[2], -0.1)
        self.assertAlmostEqual(state.time_s, 0.1)

    def test_thrust_direction_is_normalized(self):
        self.assertEqual(directed_thrust(100.0, (0.0, 0.0, 2.0)), (0.0, 0.0, 100.0))

    def test_fbw_passes_valid_command_through(self):
        output = FlyByWireController().update(PilotCommand(throttle=0.5), 0.01)
        self.assertEqual(output.throttle, 0.5)

    def test_environment_relative_velocity(self):
        env = AirEnvironment(wind_velocity_world_mps=(1.0, 0.0, 0.0))
        self.assertEqual(env.relative_air_velocity((5.0, 0.0, 0.0)), (4.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
