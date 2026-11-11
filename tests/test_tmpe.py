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
            tmpe.main(flags)
        return output.getvalue()

    def test_output_modes(self):
        normal = self.run_main([])
        diagnostics = self.run_main(['--diagnostics'])
        verbose = self.run_main(['--verbose'])
        both = self.run_main(['--diagnostics', '--verbose'])
        for output in (normal, diagnostics, verbose, both):
            self.assertRegex(output, r'Airspeed:\s+5\.00 m/s')
            self.assertIn('Takeoff status: not simulated', output)
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
                tmpe.main(['--verbose'])
        self.assertEqual(prompts.call_count, 5)
        text = output.getvalue()
        self.assertIn('helicopter-mode vertical takeoff', text)
        self.assertIn('on the ground, stationary, level, facing north', text)
        self.assertIn('height above ground = 0.00 m', text)
        self.assertIn('Ground elevation (MSL): 500.00 m', text)
        self.assertIn('Prescribed ground velocity NED: (0.00, 0.00, 0.00) m/s', text)
        self.assertIn('Takeoff status: not simulated', text)


if __name__ == '__main__':
    unittest.main()
