"""Kinematic sign/frame checks; no aircraft coefficient assumptions."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

path = Path(__file__).resolve().parents[1] / 'MY_physics_engine' / 'tiltrotor_modeling_physics_engine(TMPE).py'
spec = importlib.util.spec_from_file_location('tmpe', path)
tmpe = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = tmpe
spec.loader.exec_module(tmpe)


class AoATests(unittest.TestCase):
    def flow(self, velocity=(50, 0, 0), angles=(0, 0, 0), incidence=3, wind=0, direction=0):
        return tmpe.calculate_aoa(tmpe.AircraftState(velocity, *map(math.radians, angles)),
                                  tmpe.Environment(windspeed=wind, winddirection=direction), incidence)

    def test_planar_signs(self):
        self.assertAlmostEqual(math.degrees(self.flow()['alpha']), 3)
        self.assertAlmostEqual(math.degrees(self.flow(angles=(0, 10, 0))['alpha']), 13)
        gamma = math.radians(7)
        velocity = (50 * math.cos(gamma), 0, -50 * math.sin(gamma))
        self.assertAlmostEqual(math.degrees(self.flow(velocity, (0, 10, 0))['alpha']), 6)

    def test_yaw_and_roll(self):
        f = self.flow((0, 50, 0), (0, 0, 90))
        self.assertAlmostEqual(f['body'][0], 50)
        self.assertAlmostEqual(math.degrees(f['alpha']), 3)
        f = self.flow((0, 0, 10), (90, 0, 0), incidence=0)
        self.assertAlmostEqual(f['body'][1], 10)
        self.assertIsNone(f['alpha'])

    def test_rotation_is_orthonormal(self):
        matrix = tmpe.world_to_body_matrix(tmpe.AircraftState(roll=.3, pitch=-.6, yaw=1.2))
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(sum(matrix[i][k] * matrix[j][k] for k in range(3)), float(i == j))
        # Body forward expressed in NED must transform back to body forward.
        world_forward = tuple(matrix[0])
        for actual, expected in zip(tmpe.transform(matrix, world_forward), (1, 0, 0)):
            self.assertAlmostEqual(actual, expected)

    def test_wind_from_and_subtraction(self):
        f = self.flow(wind=10, direction=0)
        self.assertEqual(f['air_world'], (60, 0, 0))
        f = self.flow(wind=10, direction=180)
        self.assertAlmostEqual(f['speed'], 40)

    def test_undefined_and_zero_force(self):
        f = self.flow((0, 0, 0))
        self.assertIsNone(f['alpha'])
        self.assertEqual(tmpe.wing_aerodynamics(f)['force_body_n'], (0, 0, 0))
        for velocity in ((0, 10, 0), (1e-8, 0, 0)):
            f = self.flow(velocity)
            self.assertIsNone(f['alpha'])
            self.assertIsNone(tmpe.wing_aerodynamics(f)['force_body_n'])

    def test_co_moving_air(self):
        self.assertIsNone(self.flow((-5, 0, 0), wind=5)['alpha'])

    def test_missing_geometry_and_unsupported_regimes(self):
        self.assertIsNone(self.flow(incidence=None)['alpha'])
        for velocity in ((-50, 0, 0), (0, 0, 50), (50, 0, 0)):
            f = self.flow(velocity)
            self.assertIsNotNone(f['alpha'])
            self.assertIsNone(tmpe.wing_aerodynamics(f)['force_body_n'])
        self.assertAlmostEqual(abs(math.degrees(self.flow((-50, 0, 0), incidence=0)['alpha'])), 180)

    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):
            self.flow((float('nan'), 0, 0))


if __name__ == '__main__':
    unittest.main()
