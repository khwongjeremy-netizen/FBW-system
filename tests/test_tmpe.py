"""Kinematic sign/frame checks; no aircraft coefficient assumptions."""
import contextlib
import io
from unittest.mock import patch
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import shutil
import xml.etree.ElementTree as ET
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


class GeometryRecoveryTests(unittest.TestCase):
    def test_citations_and_datum_coordinates(self):
        aircraft = tmpe.Aircraft()
        for item in aircraft.geometry_recovery.values():
            self.assertTrue(item['url'].startswith('https://ntrs.nasa.gov/'))
            self.assertTrue(item['location'])
            self.assertEqual(item['status'], 'user_supplied_unverified')
        self.assertAlmostEqual(aircraft.rotor_hub_positions['left'][0], 290 / 12)
        self.assertAlmostEqual(aircraft.rotor_hub_positions['left'][1], -193 / 12)
        self.assertAlmostEqual(aircraft.rotor_hub_positions['right'][2], 86.75 / 12)
        self.assertEqual(aircraft.fuselage_width_range_ft, (5.0, 5.2))
        self.assertIsNone(aircraft.fuselage_width_ft)
        self.assertAlmostEqual(aircraft.fuselage_height_ft, 74 / 12)
        self.assertIsNone(aircraft.isolated_nacelle_polars)

    def test_length_units_and_invalid_data(self):
        for unit, value in [('in', 12), ('ft', 1), ('m', .3048)]:
            self.assertAlmostEqual(tmpe.recovery_length_ft({'x': {'value': value, 'unit': unit}}, 'x'), 1)
        for unit, value in [('deg', 1), ('ft', 'nan')]:
            with self.assertRaises(ValueError):
                tmpe.recovery_length_ft({'x': {'value': value, 'unit': unit}}, 'x')


class LoadingDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.patch = patch.object(tmpe, 'DATA_DIR', self.directory)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def load(self, xml=None, path='value', unit='ft'):
        if xml is not None:
            (self.directory / 'sample.xml').write_text(xml)
        records = {}
        value = tmpe.load_parameter(records, 'sample', 'sample.xml', path, unit, 'Sample length')
        return value, records['sample']

    def test_missing_file_element_attribute_and_empty_value(self):
        for xml, path, status in [
            (None, 'value', 'MISSING'),
            ('<root/>', 'value', 'MISSING'),
            ('<root><value/></root>', 'value/@length', 'MISSING'),
            ('<root><value> </value></root>', 'value', 'INVALID'),
        ]:
            with self.subTest(xml=xml, path=path):
                value, row = self.load(xml, path)
                self.assertIsNone(value)
                self.assertEqual(row['status'], status)
                self.assertEqual(row['path'], path)
                self.assertTrue(row['reason'])

    def test_invalid_syntax_numbers_and_units(self):
        for xml in ['<root>', '<root><value unit="ft">abc</value></root>',
                    '<root><value unit="ft">nan</value></root>',
                    '<root><value unit="ft">inf</value></root>',
                    '<root><value unit="deg">3</value></root>',
                    '<root><value>3</value></root>']:
            with self.subTest(xml=xml):
                value, row = self.load(xml)
                self.assertIsNone(value)
                self.assertEqual(row['status'], 'INVALID')

    def test_conversion_metadata_zero_and_attribute(self):
        value, row = self.load('<root source="S2" status="user_supplied_unverified"><value unit="in" location="p. 12">12</value></root>')
        self.assertEqual(value, 1)
        self.assertEqual(row['status'], 'UNVERIFIED')
        self.assertEqual(row['citation']['source'], 'S2')
        self.assertEqual(row['citation']['location'], 'p. 12')
        self.assertEqual(row['source_unit'], 'in')
        value, row = self.load('<root><value length="0" length_unit="ft"/></root>', 'value/@length')
        self.assertEqual(value, 0)
        self.assertEqual(row['status'], 'LOADED')

    def test_recovery_failure_isolated_and_attributes_always_exist(self):
        original = Path(__file__).resolve().parents[1] / 'MY_physics_engine' / 'data(v.2)'
        for filename in ('geometry.xml', 'rotor.xml', 'mass.xml'):
            shutil.copyfile(original / filename, self.directory / filename)
        path = self.directory / 'geometry.xml'
        tree = ET.parse(path)
        tree.find("notebooklm_geometry_recovery/item[@name='hub_left_fs_in']").text = 'bad'
        tree.write(path)
        aircraft = tmpe.Aircraft()
        self.assertIsNone(aircraft.rotor_hub_positions)
        self.assertEqual(aircraft.diagnostics['rotor_hub_positions']['status'], 'INVALID')
        self.assertEqual(aircraft.rotor_radius_ft, 12.5)
        self.assertEqual(aircraft.wing_incidence_deg, 3)
        tree.find("notebooklm_geometry_recovery/item[@name='hub_left_fs_in']").text = '290'
        tree.find("notebooklm_geometry_recovery/item[@name='hub_left_fs_in']").set('unit', 'deg')
        tree.write(path)
        aircraft = tmpe.Aircraft()
        self.assertIsNone(aircraft.rotor_hub_positions)
        self.assertIn('Unsupported length unit', aircraft.diagnostics['geometry_recovery.hub_left_fs_in']['reason'])
        section = tree.find('notebooklm_geometry_recovery')
        section.remove(section.find("item[@name='hub_left_fs_in']"))
        tree.write(path)
        aircraft = tmpe.Aircraft()
        self.assertIsNone(aircraft.rotor_hub_positions)
        self.assertEqual(aircraft.diagnostics['geometry_recovery.hub_left_fs_in']['status'], 'MISSING')
        self.assertIn("[@name='hub_left_fs_in']", aircraft.diagnostics['geometry_recovery.hub_left_fs_in']['path'])
        path.write_text('<broken>')
        aircraft = tmpe.Aircraft()
        self.assertEqual(aircraft.diagnostics['geometry_recovery']['status'], 'INVALID')
        self.assertEqual(aircraft.diagnostics['wing_incidence_deg']['status'], 'INVALID')
        path.unlink()
        aircraft = tmpe.Aircraft()
        self.assertIsNone(aircraft.wing_incidence_deg)
        self.assertIsNone(aircraft.fuselage_width_range_ft)
        self.assertIsNone(aircraft.rotor_radius_ft)
        self.assertEqual(aircraft.diagnostics['wing_incidence_deg']['status'], 'MISSING')


class DiagnosticReportTests(unittest.TestCase):
    def run_main(self, flags):
        output = io.StringIO()
        with patch('builtins.input', side_effect=[''] * 5), contextlib.redirect_stdout(output):
            tmpe.main(['--rotor-model', 'prototype-fixed-ct'] + flags)
        return output.getvalue()

    def test_output_modes(self):
        normal = self.run_main([])
        diagnostics = self.run_main(['--diagnostics'])
        verbose = self.run_main(['--verbose'])
        both = self.run_main(['--diagnostics', '--verbose'])
        for output in (normal, diagnostics, verbose, both):
            self.assertIn('LEFT PROPROTOR', output)
            self.assertIn('AIRCRAFT WEIGHT', output)
            self.assertIn('TAKEOFF STATUS         : AIRBORNE', output)
        self.assertNotIn('DIAGNOSTIC REPORT', normal)
        self.assertNotIn('calculation trace', normal)
        self.assertIn('DIAGNOSTIC REPORT', diagnostics)
        self.assertNotIn('calculation trace', diagnostics)
        self.assertIn('calculation trace', verbose)
        self.assertNotIn('DIAGNOSTIC REPORT', verbose)
        self.assertEqual(both.count('DIAGNOSTIC REPORT'), 1)
        self.assertEqual(both.count('calculation trace'), 1)
        self.assertIn('geometry/wing/incidence', diagnostics)
        self.assertIn('not used in live physics', diagnostics)
        self.assertIn('UNVERIFIED', diagnostics)

    def test_actual_physics_statuses(self):
        for velocity, incidence, expected in [
            ((0, 0, 0), 3, 'CALCULATED'),
            ((50, 0, 0), 3, 'UNAVAILABLE'),
            ((-50, 0, 0), 3, 'UNSUPPORTED'),
            ((0, 10, 0), 3, 'UNSUPPORTED'),
            ((1e-8, 0, 0), 3, 'UNAVAILABLE'),
            ((50, 0, 0), None, 'UNAVAILABLE'),
        ]:
            with self.subTest(velocity=velocity, incidence=incidence):
                flow = tmpe.calculate_aoa(tmpe.AircraftState(velocity), tmpe.Environment(windspeed=0), incidence)
                aero = tmpe.wing_aerodynamics(flow)
                before = dict(flow)
                results = tmpe.physics_diagnostics(flow, aero)
                self.assertEqual(results['Wing lift'][0], expected)
                self.assertEqual(results['Wing AoA'][0], 'CALCULATED' if flow['alpha'] is not None else 'UNAVAILABLE')
                self.assertEqual(flow, before)
                self.assertEqual(results['Rotor thrust'][0], 'NOT IMPLEMENTED')

    def test_missing_incidence_report_includes_exact_source(self):
        aircraft = tmpe.Aircraft()
        aircraft.wing_incidence_deg = None
        aircraft.diagnostics['wing_incidence_deg'].update(status='INVALID', value=None, reason='ValueError: invalid float')
        flow = tmpe.calculate_aoa(tmpe.AircraftState(), tmpe.Environment(), None)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            tmpe.report_diagnostics(aircraft, flow, tmpe.wing_aerodynamics(flow))
        text = output.getvalue()
        self.assertIn('Wing AoA: UNAVAILABLE', text)
        self.assertIn('geometry.xml :: geometry/wing/incidence — ValueError: invalid float', text)


class VerticalTakeoffTests(unittest.TestCase):
    def test_design_mass_and_weight_independent_units(self):
        self.assertEqual(tmpe.takeoff_mass_kg(1), .45359237)
        mass = tmpe.takeoff_mass_kg(13000)
        self.assertAlmostEqual(mass, 5896.70081)
        self.assertAlmostEqual(mass * tmpe.G, 13000 * 4.4482216152605)
        self.assertIsNone(tmpe.takeoff_mass_kg(None))
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                tmpe.takeoff_mass_kg(value)

    def test_left_right_models_do_not_invent_thrust_from_ratings(self):
        aircraft = tmpe.Aircraft()
        for side in ('left', 'right'):
            row = tmpe.calculate_proprotor(side, aircraft, tmpe.Environment())
            self.assertEqual(row['status'], 'UNAVAILABLE')
            self.assertIsNone(row['value'])
            self.assertIsNone(row['rotor_shaft_power_w'])
            self.assertIn(side, row['reason'])
            self.assertIn('mast power', row['reason'])
        aircraft.engine_takeoff_shp = None
        row = tmpe.calculate_proprotor('left', aircraft, tmpe.Environment())
        self.assertIn('engine takeoff rating', row['reason'])
        self.assertEqual(aircraft.nacelle_angle_deg, 90)
        self.assertEqual(aircraft.diagnostics['nacelle_angle_deg']['citation']['source'], 'S14')

    def test_rotor_vector_directions(self):
        for angle, expected in [(0, (100, 0, 0)), (90, (0, 0, -100)),
                                (180, (-100, 0, 0)), (45, (math.sqrt(5000), 0, -math.sqrt(5000)))]:
            for actual, reference in zip(tmpe.rotor_force_body_n(100, angle), expected):
                self.assertAlmostEqual(actual, reference)
        self.assertIsNone(tmpe.rotor_force_body_n(None, 90))
        with self.assertRaises(ValueError):
            tmpe.rotor_force_body_n(-10, 90)

    def test_force_balance_projects_each_rotor_and_other_loads(self):
        row = tmpe.vertical_force_balance(10, 100, 200, 90, 30, -20)
        self.assertAlmostEqual(row['total_rotor_vertical_force_n'], 200)
        self.assertAlmostEqual(row['net_vertical_force_n'], 180 - 10 * tmpe.G)
        self.assertAlmostEqual(row['vertical_acceleration_mps2'], 18 - tmpe.G)
        self.assertIsNone(tmpe.vertical_force_balance(10, None, 200, other_vertical_force_n=0)['value'])
        self.assertIsNone(tmpe.vertical_force_balance(10, 100, 200)['value'])

    def test_ground_support_and_liftoff_threshold(self):
        for net in (-100, 0):
            row = tmpe.integrate_vertical(0, 0, 10, net, .1)
            self.assertEqual(row['takeoff_status'], 'GROUNDED')
            self.assertEqual(row['height_agl_m'], 0)
            self.assertEqual(row['vertical_acceleration_mps2'], 0)
            self.assertEqual(row['ground_support_n'], -net)
        row = tmpe.integrate_vertical(0, 0, 10, 20, .5)
        self.assertEqual(row['takeoff_status'], 'AIRBORNE')
        self.assertAlmostEqual(row['height_agl_m'], .25)
        self.assertAlmostEqual(row['vertical_velocity_mps'], 1)
        self.assertEqual(row['ground_support_n'], 0)

    def test_constant_force_trajectory_and_ground_constraint(self):
        row = tmpe.integrate_vertical(4, 3, 10, -20, .5)
        self.assertAlmostEqual(row['height_agl_m'], 5.25)
        self.assertAlmostEqual(row['vertical_velocity_mps'], 2)
        row = tmpe.integrate_vertical(.1, -2, 10, -20, 1)
        self.assertEqual(row['height_agl_m'], 0)
        self.assertEqual(row['vertical_velocity_mps'], 0)
        with self.assertRaises(ValueError):
            tmpe.integrate_vertical(0, 0, 10, 0, 0)

    def test_unknown_force_does_not_advance_or_become_zero(self):
        row = tmpe.integrate_vertical(3, 2, 10, None, .1)
        self.assertEqual(row['height_agl_m'], 3)
        self.assertEqual(row['vertical_velocity_mps'], 2)
        self.assertIsNone(row['vertical_acceleration_mps2'])
        self.assertFalse(row['advanced'])
        result = tmpe.simulate_takeoff(tmpe.Aircraft(), tmpe.Environment())
        self.assertEqual(result['elapsed_s'], 0)
        self.assertEqual(result['history'], [])
        self.assertEqual(result['motion']['takeoff_status'], 'UNAVAILABLE')

    def test_time_loop_against_analytic_synthetic_force_case(self):
        # Synthetic mechanics case; these thrusts are NOT XV-15 reference data.
        aircraft = tmpe.Aircraft()
        mass = tmpe.takeoff_mass_kg(aircraft.design_gross)
        def forces(time, height, velocity):
            thrust = mass * (tmpe.G + 2) / 2
            return dict(left_proprotor_thrust_n=thrust, right_proprotor_thrust_n=thrust,
                        other_vertical_force_n=0)
        for dt in (.1, .3, 2):
            result = tmpe.simulate_takeoff(aircraft, tmpe.Environment(), 1, dt, forces)
            self.assertAlmostEqual(result['elapsed_s'], 1)
            self.assertAlmostEqual(result['motion']['height_agl_m'], 1)
            self.assertAlmostEqual(result['motion']['vertical_velocity_mps'], 2)
        aircraft.wing_incidence_deg = None
        result = tmpe.simulate_takeoff(aircraft, tmpe.Environment(), 1, .1, forces)
        self.assertAlmostEqual(result['motion']['height_agl_m'], 1)

    def test_missing_mass_and_unsupported_configuration(self):
        aircraft = tmpe.Aircraft()
        aircraft.design_gross = None
        result = tmpe.simulate_takeoff(aircraft, tmpe.Environment())
        self.assertIsNone(result['takeoff_mass_kg'])
        self.assertEqual(result['elapsed_s'], 0)
        aircraft = tmpe.Aircraft()
        aircraft.nacelle_angle_deg = 45
        result = tmpe.simulate_takeoff(aircraft, tmpe.Environment())
        self.assertEqual(result['motion']['status'], 'UNSUPPORTED')
        self.assertEqual(result['elapsed_s'], 0)

    def test_loop_time_controls_and_unavailable_later_step(self):
        aircraft = tmpe.Aircraft()
        mass = tmpe.takeoff_mass_kg(aircraft.design_gross)
        def forces(time, height, velocity):
            thrust = mass * (tmpe.G + 2) / 2 if time == 0 else None
            return dict(left_proprotor_thrust_n=thrust, right_proprotor_thrust_n=thrust,
                        other_vertical_force_n=0)
        result = tmpe.simulate_takeoff(aircraft, tmpe.Environment(), 1, .1, forces)
        self.assertAlmostEqual(result['elapsed_s'], .1)
        self.assertAlmostEqual(result['motion']['height_agl_m'], .01)
        self.assertEqual(result['motion']['status'], 'UNAVAILABLE')
        result = tmpe.simulate_takeoff(aircraft, tmpe.Environment(), 0, .1, forces)
        self.assertEqual(result['elapsed_s'], 0)
        self.assertEqual(result['motion']['height_agl_m'], 0)
        self.assertEqual(result['motion']['takeoff_status'], 'GROUNDED')
        for duration, dt in [(-1, .1), (1, 0), (1, float('nan'))]:
            with self.assertRaises(ValueError):
                tmpe.simulate_takeoff(aircraft, tmpe.Environment(), duration, dt)


class IdealHoverTests(unittest.TestCase):
    def test_analytic_disk_and_power_identity(self):
        # Hypothetical disk: area 2 m², rho 1, vi 3 -> T 36 N, Pi 108 W.
        row = tmpe.ideal_hover_from_induced_power(math.sqrt(8 / math.pi), 1, 108)
        self.assertAlmostEqual(row['value'], 36)
        self.assertAlmostEqual(row['induced_velocity_mps'], 3)
        self.assertAlmostEqual(row['value'] * row['induced_velocity_mps'], 108)
        self.assertEqual(tmpe.ideal_hover_from_induced_power(2, 1, 0)['value'], 0)

    def test_explicit_separate_side_power_chain(self):
        aircraft = tmpe.Aircraft()
        env = tmpe.Environment(windspeed=0)
        left = tmpe.estimate_proprotor_hover('left', aircraft, env,
            engine_power_w=1000, transmission_efficiency=.8, ideal_power_fraction=.5)
        right = tmpe.estimate_proprotor_hover('right', aircraft, env,
            engine_power_w=8000, transmission_efficiency=.8, ideal_power_fraction=.5)
        self.assertEqual(left['rotor_shaft_power_w'], 800)
        self.assertEqual(left['induced_power_w'], 400)
        self.assertAlmostEqual(right['value'] / left['value'], 4)
        self.assertEqual(tmpe.calculate_proprotor('left', aircraft, env)['status'], 'UNAVAILABLE')
        self.assertAlmostEqual(aircraft.transmission_efficiency, .93)
        self.assertEqual(aircraft.diagnostics['transmission_efficiency']['citation']['source'], 'S2')
        missing = tmpe.estimate_proprotor_hover('left', aircraft, env,
            engine_power_w=None, transmission_efficiency=.93, ideal_power_fraction=1)
        self.assertIsNone(missing['thrust_n'])

    def test_invalid_inputs(self):
        for diameter, density, power in [(0, 1, 1), (1, 0, 1), (1, 1, -1),
                                         (float('nan'), 1, 1), (1, 1, float('inf'))]:
            with self.assertRaises(ValueError):
                tmpe.ideal_hover_from_induced_power(diameter, density, power)
        for eta, fraction in [(0, 1), (1.1, 1), (1, 0), (1, 1.1)]:
            with self.assertRaises(ValueError):
                tmpe.estimate_proprotor_hover('left', tmpe.Aircraft(), tmpe.Environment(),
                    engine_power_w=100, transmission_efficiency=eta, ideal_power_fraction=fraction)

    def test_hypothetical_vertical_balance_thresholds(self):
        # Arbitrary 10 kg body and prescribed forces, NOT XV-15 predictions.
        for ratio in (.8, 1, 1.2):
            thrust = ratio * 10 * tmpe.G / 2
            row = tmpe.vertical_force_balance(10, thrust, thrust, other_vertical_force_n=0)
            self.assertAlmostEqual(row['net_vertical_force_n'], (ratio - 1) * 10 * tmpe.G)
            self.assertAlmostEqual(row['vertical_acceleration_mps2'], (ratio - 1) * tmpe.G)

    def test_opt_in_report_keeps_takeoff_unavailable(self):
        output = io.StringIO()
        with patch('builtins.input', side_effect=[''] * 5), contextlib.redirect_stdout(output):
            tmpe.main(['--ideal-rated-hover', '--diagnostics', '--rotor-model', 'unsupported'])
        text = output.getvalue()
        self.assertIn('NOT XV-15 PREDICTIONS', text)
        self.assertIn('takeoff not simulated', text)
        for label in ('engine_power_w', 'rotor_shaft_power_w', 'induced_power_w', 'thrust_n'):
            self.assertIn(label, text)


class FixedCTPrototypeTests(unittest.TestCase):
    def setUp(self):
        self.aircraft = tmpe.Aircraft()
        self.environment = tmpe.Environment(windspeed=0)

    def rotor(self, side='left', **kwargs):
        return tmpe.calculate_proprotor(side, self.aircraft, self.environment,
                                       rotor_model='prototype-fixed-ct', **kwargs)

    def simulate(self, **kwargs):
        return tmpe.simulate_takeoff(self.aircraft, self.environment,
                                     rotor_model='prototype-fixed-ct', **kwargs)

    def test_conversions_and_independent_numerical_reference(self):
        row = self.rotor()
        self.assertAlmostEqual(row['diameter_m'], 7.62)
        self.assertAlmostEqual(row['radius_m'], 3.81)
        self.assertAlmostEqual(row['angular_speed_rad_s'], 59.16666164260777)
        self.assertAlmostEqual(row['disk_area_m2'], 45.60367311877479)
        # Independently evaluated SI reference for CT=.0102, rho=1.225,
        # D=7.62 m and n=565/60 revolutions/s. Not flight-test data.
        self.assertAlmostEqual(row['value'], 28956.106644071755, places=7)
        self.assertAlmostEqual(row['value'], 28956, delta=1)
        self.assertNotEqual(row['disk_area_m2'], self.aircraft.rotor_disk_area_ft2 * .09290304)
        self.assertEqual(self.aircraft.diagnostics['prototype_thrust_coefficient']['status'], 'UNVERIFIED')
        self.assertIsNone(row['rotor_shaft_power_w'])
        # A unit-radius disk, Omega=1 rad/s, rho=1, CT=1 -> T=pi N.
        self.aircraft.rotor_diameter_ft = 2 / .3048
        self.aircraft.hover_rotor_rpm = 60 / (2 * math.pi)
        self.environment.air_density = 1
        self.assertAlmostEqual(self.rotor(ct_override=1)['value'], math.pi)

    def test_independent_sides_and_input_scaling(self):
        left = self.rotor('left')
        right = self.rotor('right', ct_override=.0204)
        self.assertEqual(left['side'], 'left')
        self.assertEqual(right['side'], 'right')
        self.assertAlmostEqual(right['value'], 2 * left['value'])
        self.assertEqual(self.aircraft.prototype_thrust_coefficient, .0102)
        self.assertEqual(self.rotor('left')['value'], left['value'])
        self.environment.air_density *= .5
        self.assertAlmostEqual(self.rotor()['value'], left['value'] / 2)
        self.environment.air_density *= 2
        self.aircraft.hover_rotor_rpm *= 2
        self.assertAlmostEqual(self.rotor()['value'], left['value'] * 4)
        self.aircraft.rotor_diameter_ft *= 2
        self.assertAlmostEqual(self.rotor()['value'], left['value'] * 64)

    def test_baseline_near_balance_and_vertical_trajectory(self):
        result = self.simulate(duration_s=5, timestep_s=.3)
        balance, motion = result['balance'], result['motion']
        self.assertAlmostEqual(balance['aircraft_weight_n'], 57826.8809983865)
        self.assertAlmostEqual(balance['net_vertical_force_n'], 85.33228975701059)
        self.assertLess(balance['net_vertical_force_n'] / balance['aircraft_weight_n'], .002)
        self.assertAlmostEqual(motion['vertical_acceleration_mps2'], .014471192028650777)
        self.assertAlmostEqual(motion['height_agl_m'], .1808899003581349)
        self.assertAlmostEqual(motion['vertical_velocity_mps'], .0723559601432539)
        self.assertEqual(result['elapsed_s'], 5)
        self.assertIn('excluded', result['excluded_loads'])
        self.assertTrue(self.aircraft.diagnostics['prototype_thrust_coefficient']['used'])

    def test_ground_contact_equilibrium_and_motion_with_override(self):
        grounded = self.simulate(ct_override=.009)
        self.assertEqual(grounded['motion']['height_agl_m'], 0)
        self.assertEqual(grounded['motion']['vertical_acceleration_mps2'], 0)
        self.assertGreater(grounded['motion']['ground_support_n'], 0)
        baseline = self.rotor()['value']
        weight = tmpe.takeoff_mass_kg(self.aircraft.design_gross) * tmpe.G
        equilibrium_ct = .0102 * weight / (2 * baseline)
        hover = self.simulate(ct_override=equilibrium_ct)
        self.assertAlmostEqual(hover['motion']['height_agl_m'], 0, places=10)
        climb = self.simulate(ct_override=.012, duration_s=2)
        acceleration = climb['balance']['net_vertical_force_n'] / climb['takeoff_mass_kg']
        self.assertAlmostEqual(climb['motion']['height_agl_m'], 2 * acceleration)
        self.assertGreater(climb['motion']['height_agl_m'], 0)
        self.assertFalse(self.aircraft.diagnostics['prototype_thrust_coefficient']['used'])

    def test_missing_and_invalid_parameters(self):
        for obj, name, label in ((self.aircraft, 'rotor_diameter_ft', 'rotor_diameter_ft'),
                                  (self.aircraft, 'hover_rotor_rpm', 'hover_rotor_rpm'),
                                  (self.aircraft, 'prototype_thrust_coefficient', 'thrust_coefficient'),
                                  (self.environment, 'air_density', 'air_density')):
            original = getattr(obj, name)
            for value in (None, -1, float('nan'), float('inf'), 'bad'):
                with self.subTest(name=name, value=value):
                    setattr(obj, name, value)
                    row = self.rotor()
                    self.assertIsNone(row['value'])
                    self.assertIn(label, row['reason'])
            setattr(obj, name, original)
        for obj, name in ((self.aircraft, 'rotor_diameter_ft'), (self.environment, 'air_density')):
            original = getattr(obj, name)
            setattr(obj, name, 0)
            self.assertEqual(self.rotor()['status'], 'UNAVAILABLE')
            setattr(obj, name, original)
        self.assertEqual(self.rotor(ct_override=0)['value'], 0)
        self.aircraft.hover_rotor_rpm = 0
        self.assertEqual(self.rotor()['value'], 0)

    def test_missing_xml_ct_stops_simulation_without_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)
            for path in tmpe.DATA_DIR.glob('*.xml'):
                shutil.copyfile(path, target / path.name)
            path = target / 'rotor.xml'
            tree = ET.parse(path)
            section = tree.find('proprotor_and_drive_system/prototype_thrust_model')
            section.remove(section.find('thrust_coefficient'))
            tree.write(path)
            with patch.object(tmpe, 'DATA_DIR', target):
                self.aircraft = tmpe.Aircraft()
                result = self.simulate()
                self.assertEqual(result['elapsed_s'], 0)
                self.assertEqual(result['motion']['takeoff_status'], 'UNAVAILABLE')
                self.assertIn('missing thrust_coefficient', result['left']['reason'])
                self.assertEqual(self.aircraft.diagnostics['prototype_thrust_coefficient']['status'], 'MISSING')
                self.assertEqual(self.simulate(ct_override=.0102)['elapsed_s'], 5)

    def test_default_mode_and_excluded_wind_and_power(self):
        self.assertEqual(tmpe.calculate_proprotor('left', self.aircraft, self.environment)['status'], 'UNAVAILABLE')
        self.assertEqual(tmpe.simulate_takeoff(self.aircraft, self.environment)['elapsed_s'], 0)
        self.aircraft.engine_takeoff_shp = None
        self.assertEqual(self.rotor()['status'], 'CALCULATED')
        reference = self.simulate()
        self.environment.windspeed = 50
        self.assertEqual(self.simulate()['motion'], reference['motion'])
        self.aircraft.nacelle_angle_deg = 45
        tilted = self.simulate()
        self.assertEqual(tilted['motion']['status'], 'UNSUPPORTED')
        self.assertAlmostEqual(tilted['balance']['total_rotor_vertical_force_n'],
                               2 * self.rotor()['value'] * math.sin(math.pi / 4))
        with self.assertRaises(ValueError):
            self.rotor('middle')
        with self.assertRaises(ValueError):
            tmpe.simulate_takeoff(self.aircraft, self.environment, ct_override=.01)
        with self.assertRaises(ValueError):
            self.simulate(force_provider=lambda *args: {})

    def test_requested_ct_scenarios_and_explicit_fixed_cli(self):
        thrusts = []
        for ct, status in ((.0100, 'GROUNDED'), (.0102, 'AIRBORNE'), (.0110, 'AIRBORNE')):
            result = self.simulate(ct_override=ct)
            balance, motion = result['balance'], result['motion']
            total = result['left']['value'] + result['right']['value']
            thrusts.append(total)
            self.assertAlmostEqual(balance['total_rotor_vertical_force_n'], total)
            self.assertAlmostEqual(balance['net_vertical_force_n'], total - balance['aircraft_weight_n'])
            self.assertEqual(motion['takeoff_status'], status)
            self.assertAlmostEqual(result['elapsed_s'], 5)
            self.assertTrue(all(row['height_agl_m'] >= 0 for row in result['history']))
        self.assertEqual(thrusts, sorted(thrusts))
        output = io.StringIO()
        with patch('builtins.input', side_effect=[''] * 5), contextlib.redirect_stdout(output):
            tmpe.main(['--rotor-model', 'prototype-fixed-ct'])
        self.assertIn(tmpe.PROTOTYPE_LABEL, output.getvalue())
        self.assertIn('TAKEOFF STATUS         : AIRBORNE', output.getvalue())

    def test_cli_labels_and_override_validation(self):
        output = io.StringIO()
        with patch('builtins.input', side_effect=[''] * 5), contextlib.redirect_stdout(output):
            tmpe.main(['--rotor-model', 'prototype-fixed-ct', '--prototype-ct', '.009', '--diagnostics'])
        text = output.getvalue()
        self.assertIn(tmpe.PROTOTYPE_LABEL, text)
        self.assertIn('explicit CT override assumption', text)
        self.assertIn('EXCLUDED', text)
        self.assertIn('GROUNDED', text)
        self.assertIn('not verified XV-15', text)
        for flags in (['--rotor-model', 'unsupported', '--prototype-ct', '.01'],
                      ['--rotor-model', 'prototype-fixed-ct', '--prototype-ct', 'nan']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                tmpe.main(flags)


class GroundStartTests(unittest.TestCase):
    def test_initial_state_is_stationary_on_ground(self):
        state = tmpe.AircraftState()
        self.assertEqual(state.height_agl_m, 0.0)
        self.assertEqual(state.ground_velocity_ned, (0.0, 0.0, 0.0))
        self.assertEqual((state.roll, state.pitch, state.yaw), (0.0, 0.0, 0.0))

    def test_ground_start_at_nonzero_field_elevation(self):
        output = io.StringIO()
        # Only environmental inputs remain; field elevation is not height AGL.
        with patch('builtins.input', side_effect=['', '', '', '', '500']) as prompts:
            with contextlib.redirect_stdout(output):
                tmpe.main(['--verbose', '--rotor-model', 'prototype-fixed-ct'])
        self.assertEqual(prompts.call_count, 5)
        text = output.getvalue()
        self.assertIn('helicopter-mode vertical takeoff', text)
        self.assertIn('on the ground, stationary, level, facing north', text)
        self.assertRegex(text, r'HEIGHT ABOVE GROUND\s+: 0.18 m')
        self.assertIn('Ground elevation (MSL): 500.00 m', text)
        self.assertIn('Prescribed ground velocity NED: (0.00, 0.00, 0.00) m/s', text)
        self.assertIn('TAKEOFF STATUS         : AIRBORNE', text)


class CollectiveModelTests(unittest.TestCase):
    # Arbitrary algebra fixtures, never aircraft calibration data.
    def model(self, **changes):
        parameters = dict(CT_reference=.02, k_theta_per_deg=.001,
                          theta_reference_deg=4, parameter_status='illustrative',
                          source='Synthetic unit-test fixture',
                          reference_condition='Arbitrary mathematical test only',
                          pitch_bounds=dict(minimum_deg=-10, maximum_deg=20,
                                            basis='provisional-software', source='Test bounds'))
        parameters.update(changes)
        return tmpe.LinearCollectiveModel(parameters)

    def rotor(self, theta, side='left', model=None, **kwargs):
        return tmpe.calculate_proprotor(side, tmpe.Aircraft(), tmpe.Environment(),
            rotor_model='linear-collective', collective_deg=theta,
            ct_model=model or self.model(), allow_illustrative=True, **kwargs)

    def test_reference_slope_and_independent_rotors(self):
        model = self.model()
        self.assertIsNone(model.evaluate(4)['value'])
        self.assertEqual(model.evaluate(4, allow_illustrative=True)['value'], .02)
        left, right = self.rotor(4), self.rotor(6, 'right')
        self.assertAlmostEqual(right['thrust_coefficient'], .022)
        self.assertAlmostEqual(right['value'] / left['value'], 1.1)
        self.assertEqual(self.rotor(4)['value'], left['value'])
        self.assertEqual(left['experimental_validation_status'], 'NOT VALIDATED')
        self.assertIn('MATHEMATICAL TEST ONLY', left['calibration_status'])
        self.assertEqual((left['side'], right['side']), ('left', 'right'))

    def test_uncalibrated_configuration_and_fixed_model_preserved(self):
        model = tmpe.LinearCollectiveModel.load()
        row = model.evaluate(10)
        self.assertIsNone(row['value'])
        self.assertIn('CT_reference', row['reason'])
        self.assertEqual(row['calibration_status'], 'UNCALIBRATED')
        self.assertIsNone(self.rotor(10, model=model)['value'])
        aircraft = tmpe.Aircraft()
        fixed = tmpe.calculate_proprotor('left', aircraft, tmpe.Environment(), rotor_model='prototype-fixed-ct')
        self.assertAlmostEqual(fixed['value'], 28956.106644071755)

    def test_finite_inputs_bounds_and_negative_ct(self):
        for value in (None, True, '4', float('nan'), float('inf'), -11, 21):
            self.assertIsNone(self.rotor(value)['value'])
        for value in (-10, 20):
            self.assertIsNotNone(self.rotor(value)['value'])
        self.assertIsNone(self.rotor(-10, model=self.model(k_theta_per_deg=.01))['value'])
        for key in ('CT_reference', 'k_theta_per_deg', 'theta_reference_deg'):
            for value in (None, float('inf'), 'bad'):
                self.assertIsNone(self.rotor(4, model=self.model(**{key: value}))['value'])
        self.assertIsNone(self.rotor(4, model=self.model(pitch_bounds={}))['value'])
        documented = self.model(pitch_bounds=dict(minimum_deg=3, maximum_deg=5,
                                                 basis='documented', source='Synthetic limit'))
        self.assertIsNone(self.rotor(6, model=documented)['value'])

    def test_zero_and_physical_scaling(self):
        aircraft, env = tmpe.Aircraft(), tmpe.Environment()
        def evaluate():
            return tmpe.calculate_collective_proprotor('left', aircraft, env, 4, self.model(), allow_illustrative=True)
        baseline = evaluate()['value']
        env.air_density *= 2
        self.assertAlmostEqual(evaluate()['value'], 2 * baseline)
        aircraft.hover_rotor_rpm *= 2
        self.assertAlmostEqual(evaluate()['value'], 8 * baseline)
        aircraft.hover_rotor_rpm = 0
        self.assertEqual(evaluate()['value'], 0)
        self.assertEqual(self.rotor(4, model=self.model(CT_reference=0))['value'], 0)
        env.air_density = -1
        self.assertIsNone(evaluate()['value'])

    def test_configuration_file_errors_and_loading(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.json'
            self.assertIsNone(tmpe.LinearCollectiveModel.load(path).evaluate(4)['value'])
            path.write_text('{')
            self.assertIsNone(tmpe.LinearCollectiveModel.load(path).evaluate(4)['value'])
            import json
            path.write_text(json.dumps(self.model().parameters))
            self.assertEqual(tmpe.LinearCollectiveModel.load(path).evaluate(4, allow_illustrative=True)['value'], .02)

    def test_replacement_interface(self):
        class MeasuredCurveAdapter:
            def evaluate(self, theta, *, allow_illustrative=False):
                return dict(value=.03, status='CALCULATED', reason='Synthetic adapter',
                            calibration_status='TEST', experimental_validation_status='NOT VALIDATED')
        row = self.rotor(4, model=MeasuredCurveAdapter())
        self.assertEqual(row['thrust_coefficient'], .03)
        self.assertAlmostEqual(row['value'], self.rotor(4)['value'] * 1.5)


class CollectiveCLITests(unittest.TestCase):
    fixture = Path(__file__).parent / 'fixtures' / 'collective_illustrative.json'

    def run_cli(self, pitches, extra=None):
        output = io.StringIO()
        flags = ['--rotor-model', 'prototype-collective'] + (extra or [])
        with patch('builtins.input', side_effect=[''] * 5 + (['n'] if not extra else []) + pitches) as prompts:
            with patch.object(tmpe, 'calculate_collective_proprotor', wraps=tmpe.calculate_collective_proprotor) as calculation:
                with patch.object(tmpe, 'simulate_takeoff', wraps=tmpe.simulate_takeoff) as simulation:
                    with contextlib.redirect_stdout(output):
                        tmpe.main(['--rotor-model', 'prototype-fixed-ct'] + flags)
        return output.getvalue(), prompts, calculation, simulation

    def test_cli_prompts_dispatch_and_motion(self):
        flags = ['--collective-config', str(self.fixture), '--allow-illustrative-collective']
        low, prompts, calls, _ = self.run_cli(['4', '4'], flags)
        high, _, high_calls, _ = self.run_cli(['6', '6'], flags)
        self.assertIn('Pilot left collective pitch', prompts.call_args_list[5].args[0])
        self.assertIn('Pilot right collective pitch', prompts.call_args_list[6].args[0])
        self.assertEqual({call.args[0] for call in calls.call_args_list}, {'left', 'right'})
        self.assertTrue(all(call.args[3] == 4 for call in calls.call_args_list))
        self.assertTrue(all(call.args[3] == 6 for call in high_calls.call_args_list))
        self.assertIn('CT = 0.009', low)
        self.assertIn('CT = 0.011', high)
        self.assertIn('TAKEOFF STATUS         : GROUNDED', low)
        self.assertIn('TAKEOFF STATUS         : AIRBORNE', high)
        self.assertIn('experimental validation: NOT VALIDATED', high)
        self.assertIn('MATHEMATICAL TEST ONLY', high)

    def test_per_side_ct_and_analytic_trajectory(self):
        model = tmpe.LinearCollectiveModel.load(self.fixture)
        aircraft, env = tmpe.Aircraft(), tmpe.Environment()
        result = tmpe.simulate_takeoff(aircraft, env, duration_s=2,
            rotor_model='prototype-collective', collective_deg={'left': 5, 'right': 7},
            ct_model=model, allow_illustrative=True)
        self.assertAlmostEqual(result['left']['thrust_coefficient'], .010)
        self.assertAlmostEqual(result['right']['thrust_coefficient'], .012)
        self.assertAlmostEqual(result['right']['value'] / result['left']['value'], 1.2)
        acceleration = result['balance']['net_vertical_force_n'] / result['takeoff_mass_kg']
        self.assertGreater(acceleration, 0)
        self.assertAlmostEqual(result['motion']['height_agl_m'], .5 * acceleration * 2**2)
        self.assertFalse(aircraft.diagnostics['prototype_thrust_coefficient']['used'])

    def test_missing_data_and_illustrative_opt_in(self):
        text, _, _, _ = self.run_cli(['4', '6'])
        for name in ('CT_reference', 'k_theta_per_deg', 'theta_reference_deg', 'reference_condition'):
            self.assertIn(name, text)
        self.assertIn('takeoff not simulated', text)
        self.assertNotIn('CT = 0.0102', text)
        denied, _, _, _ = self.run_cli(['4', '6'], ['--collective-config', str(self.fixture)])
        self.assertIn('Illustrative parameters require explicit', denied)
        self.assertIn('takeoff not simulated', denied)

    def test_reject_inputs_and_reprompt(self):
        flags = ['--collective-config', str(self.fixture), '--allow-illustrative-collective']
        text, prompts, calls, _ = self.run_cli(['nan', 'inf', '21', '4', '6'], flags)
        self.assertIn('finite number', text)
        self.assertIn('outside provisional-software bounds', text)
        self.assertEqual(prompts.call_count, 10)
        self.assertEqual({call.args[3] for call in calls.call_args_list}, {4, 6})

    def test_invalid_option_combinations(self):
        for flags in (['--rotor-model', 'prototype-fixed-ct', '--collective-config', str(self.fixture)],
                      ['--rotor-model', 'prototype-fixed-ct', '--allow-illustrative-collective'],
                      ['--rotor-model', 'prototype-collective', '--prototype-ct', '.01']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                tmpe.main(flags)


class DefaultCollectiveTests(unittest.TestCase):
    def run_default(self, answers):
        output = io.StringIO()
        with patch('builtins.input', side_effect=[''] * 5 + answers) as prompts:
            with contextlib.redirect_stdout(output):
                tmpe.main([])
        return output.getvalue(), prompts

    def test_normal_execution_opt_in_and_pitch_validation(self):
        text, prompts = self.run_default(['maybe', 'yes', 'nan', 'inf', '21', '6', '6'])
        self.assertIn('Enable illustrative collective testing?', prompts.call_args_list[5].args[0])
        self.assertIn('Please answer yes or no.', text)
        self.assertIn('Model: prototype-collective', text)
        self.assertIn('finite number', text)
        self.assertIn('outside provisional-software bounds', text)
        self.assertIn('source', text)
        self.assertIn('left: pilot collective = 6.0 deg; CT = 0.011', text)
        self.assertIn('right: pilot collective = 6.0 deg; CT = 0.011', text)
        self.assertRegex(text, r'HEIGHT ABOVE GROUND\s+: 9.81 m')
        self.assertIn('MATHEMATICAL TEST ONLY', text)
        self.assertIn('experimental validation: NOT VALIDATED', text)

    def test_decline_preserves_unavailable_aircraft_model(self):
        for answer in ('no', ''):
            text, prompts = self.run_default([answer, '4', '6'])
            self.assertEqual(prompts.call_count, 8)
            self.assertIn('Model: prototype-collective', text)
            for name in ('CT_reference', 'k_theta_per_deg', 'theta_reference_deg', 'reference_condition'):
                self.assertIn(name, text)
            self.assertIn('TAKEOFF STATUS         : UNAVAILABLE', text)
            self.assertIn('takeoff not simulated', text)
            self.assertNotIn('CT = 0.0102', text)

    def test_fixture_resolves_independently_of_working_directory(self):
        import os
        with tempfile.TemporaryDirectory() as directory:
            previous = Path.cwd()
            try:
                os.chdir(directory)
                text, _ = self.run_default(['y', '6', '6'])
            finally:
                os.chdir(previous)
        self.assertRegex(text, r'HEIGHT ABOVE GROUND\s+: 9.81 m')


class TransitionTests(unittest.TestCase):
    def takeoff(self, **options):
        return tmpe.simulate_takeoff(tmpe.Aircraft(), tmpe.Environment(),
                                     rotor_model='prototype-fixed-ct', **options)

    def test_cardinal_angles_and_individual_vector_magnitudes(self):
        result = self.takeoff()
        result['right_proprotor_thrust_n'] *= 2  # Unequal synthetic thrust fixture.
        transition = tmpe.sample_nacelle_transition(result, [(90, 90), (0, 0), (45, 45), (90, 0)])
        total = result['left_proprotor_thrust_n'] + result['right_proprotor_thrust_n']
        vertical, horizontal, diagonal, mixed = transition['samples']
        self.assertAlmostEqual(vertical['forward_rotor_force_n'], 0)
        self.assertAlmostEqual(vertical['upward_rotor_force_n'], total)
        self.assertAlmostEqual(horizontal['forward_rotor_force_n'], total)
        self.assertAlmostEqual(horizontal['upward_rotor_force_n'], 0)
        self.assertAlmostEqual(diagonal['forward_rotor_force_n'], total / math.sqrt(2))
        self.assertAlmostEqual(diagonal['upward_rotor_force_n'], total / math.sqrt(2))
        self.assertAlmostEqual(mixed['upward_rotor_force_n'], result['left_proprotor_thrust_n'])
        self.assertAlmostEqual(mixed['forward_rotor_force_n'], result['right_proprotor_thrust_n'])
        for row in transition['samples']:
            for side in ('left', 'right'):
                self.assertAlmostEqual(math.hypot(*row[side + '_force_body_n']),
                                       result[side + '_proprotor_thrust_n'])
        # Nonparallel rotors need not sum to the sum of thrust magnitudes.
        self.assertLess(math.hypot(mixed['forward_rotor_force_n'], mixed['upward_rotor_force_n']), total)

    def test_handoff_and_five_second_regression(self):
        import copy
        result = self.takeoff()
        before = copy.deepcopy(result)
        transition = tmpe.sample_nacelle_transition(result, [(45, 30)])
        state = transition['starting_state']
        self.assertEqual(result, before)
        self.assertEqual(state, transition['final_state'])
        self.assertEqual(state.height_agl_m, result['motion']['height_agl_m'])
        self.assertEqual(state.vertical_velocity_mps, result['motion']['vertical_velocity_mps'])
        self.assertEqual(state.elapsed_s, 5)
        for side in ('left', 'right'):
            self.assertEqual(getattr(state, side + '_proprotor_thrust_n'), result[side]['value'])
            self.assertEqual(getattr(state, side + '_nacelle_angle_deg'), 90)
        self.assertAlmostEqual(state.height_agl_m, .1808899003581349)
        self.assertAlmostEqual(state.vertical_velocity_mps, 2 * .1808899003581349 / 5)
        self.assertEqual(state.forward_velocity_mps, 0)
        self.assertEqual(state.forward_distance_m, 0)
        self.assertEqual(transition['trajectory_status'], 'BLOCKED')

    def test_validation_missing_thrust_and_net_force_interface(self):
        for angle in (-1, 91, float('nan'), float('inf'), True, '45'):
            with self.assertRaises(ValueError):
                tmpe.sample_nacelle_transition(self.takeoff(), [(angle, 90)])
        result = self.takeoff()
        result['left_proprotor_thrust_n'] = None
        row = tmpe.sample_nacelle_transition(result, [(45, 0)])['samples'][0]
        self.assertIsNone(row['upward_rotor_force_n'])
        self.assertIsNone(row['forward_rotor_force_n'])
        self.assertEqual(row['wing_lift_status'], 'UNAVAILABLE')
        self.assertEqual(row['drag_status'], 'UNAVAILABLE')
        self.assertEqual(tmpe.planar_force_balance(2, 10, -4),
                         dict(forward_acceleration_mps2=5, vertical_acceleration_mps2=-2))
        self.assertIsNone(tmpe.planar_force_balance(2)['vertical_acceleration_mps2'])
        self.assertIsNone(tmpe.planar_force_balance(None, 10, -4)['forward_acceleration_mps2'])
        for mass in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                tmpe.planar_force_balance(mass, 10, -4)

    def test_formatted_report_preserves_values_and_state(self):
        import copy
        takeoff = self.takeoff()
        result = tmpe.sample_nacelle_transition(takeoff, [(90, 90), (45, 45), (0, 0)])
        before = copy.deepcopy(result)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            tmpe.report_transition(result)
        text = output.getvalue()
        self.assertEqual(result, before)
        self.assertNotIn('TransitionState(', text)
        self.assertEqual(text.count('--- Model limitations ---'), 1)
        self.assertEqual(text.count('--- Transition configuration ---'), 3)
        for angle in (90, 45, 0):
            self.assertIn(f'{"LEFT NACELLE ANGLE":<23}: {angle:.2f}°', text)
            self.assertIn(f'{"RIGHT NACELLE ANGLE":<23}: {angle:.2f}°', text)
        for label, value, unit in (
            ('STARTING ALTITUDE', takeoff['motion']['height_agl_m'], 'm'),
            ('STARTING VERTICAL SPEED', takeoff['motion']['vertical_velocity_mps'], 'm/s'),
            ('STARTING FORWARD SPEED', 0, 'm/s'),
            ('ELAPSED SIMULATION TIME', takeoff['elapsed_s'], 's'),
        ):
            self.assertIn(f'{label:<23}: {value:.2f} {unit}', text)
        for sample in result['samples']:
            self.assertIn(f'{"TOTAL UPWARD THRUST":<23}: {sample["upward_rotor_force_n"]:.2f} N', text)
            self.assertIn(f'{"TOTAL FORWARD THRUST":<23}: {sample["forward_rotor_force_n"]:.2f} N', text)
        for label in ('WING LIFT', 'AERODYNAMIC DRAG', 'NET FORWARD FORCE',
                      'NET VERTICAL FORCE', 'FORWARD ACCELERATION', 'VERTICAL ACCELERATION'):
            self.assertEqual(text.count(f'{label:<23}: UNAVAILABLE'), 3)
        self.assertEqual(text.count(f'{"SIMULATED TIME":<23}: 5.00 s'), 3)
        self.assertEqual(text.count(f'{"TRAJECTORY":<23}: NOT SIMULATED'), 3)

    def test_cli_reporting_and_invalid_schedule(self):
        output = io.StringIO()
        with patch('builtins.input', side_effect=[''] * 5), contextlib.redirect_stdout(output):
            tmpe.main(['--rotor-model', 'prototype-fixed-ct', '--nacelle-schedule', '90:90', '45:30', '0:0'])
        text = output.getvalue()
        self.assertLess(text.index('PHASE 1 — HELICOPTER TAKEOFF'), text.index('PHASE 2 — NACELLE TRANSITION'))
        self.assertLess(text.index('Simulated time'), text.index('PHASE 2 — NACELLE TRANSITION'))
        self.assertIn('LEFT NACELLE ANGLE     : 45.00°', text)
        self.assertIn('RIGHT NACELLE ANGLE    : 30.00°', text)
        self.assertIn('Trajectory prediction BLOCKED', text)
        self.assertIn('unavailable, not zero', text)
        for value in ('91:0', 'nan:45', '45', '45:30:0', 'bad:0'):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                tmpe.main(['--nacelle-schedule', value])


if __name__ == '__main__':
    unittest.main()
