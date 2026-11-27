# ============================================================
# TMPE — Tiltrotor Modeling Physics Engine
# Ground-start helicopter takeoff prototype for the XV-15.
#
# This is an educational prototype, NOT a validated flight simulator.
# Scope: vertical takeoff and a supplied-coefficient planar transition model.
# Rotor performance data is incomplete; vertical dynamics require known forces.
# ============================================================

# ============================================================
# QUICK WORD BANK
# ============================================================
# air_world  : Aircraft velocity relative to air, in north/east/down axes.
# body_alpha : Angle of attack relative to the aircraft body (radians).
# alpha      : Wing angle of attack, including mounting incidence.
# flow       : Dictionary of airflow vectors, angles and their status.
# aero       : Dictionary of force availability and force results.
# diagnostics: Parameter records containing values, sources and load status.
# displayed  : Rounded text; calculations keep full precision.
# None       : Unknown or unavailable; never a substitute for zero.
#
# Detailed variable word bank is at the bottom of this file.
# Physics reference: ../docs/TMPE_REFERENCE.md
# Debugging guide: ../docs/TMPE_DEBUGGING.md
# ============================================================

from pathlib import Path
import argparse
from bisect import bisect_left
import csv
import runpy
import tempfile
import json
import xml.etree.ElementTree as ET
import math
from dataclasses import dataclass, replace

PROJECT_DIR = Path(__file__).resolve().parent
DATA_DIR = PROJECT_DIR / "data(v.2)"
G = 9.80665
ROTOR_DIAMETER_XML_PATH = 'proprotor_and_drive_system/rotor_diameter'
HOVER_RPM_XML_PATH = "proprotor_and_drive_system/design_rotor_speed[@mode='hover']"
ROTOR_DELTA3_XML_PATH = 'proprotor_and_drive_system/pitch_flap_coupling_angle'
ROTOR_PRECONE_XML_PATH = 'proprotor_and_drive_system/hub_precone_angle'
HELD_HOVER_RPM_STATUS = 'XML-SOURCED / HELD FIXED'
SOUND_SPEED_SOURCE = 'NASA Glenn Speed of Sound: https://www.grc.nasa.gov/www/BGH/snddrv.html'



def fetchvalue(filename, name):
    file_path = DATA_DIR / filename
    root = ET.parse(file_path).getroot()

    for element in root.iter():
        if element.get("name") == name:
            if element.text is None or not element.text.strip():
                raise ValueError(f"XML value '{name}' is empty.")
            return float(element.text.strip())

    raise ValueError(f"XML parameter '{name}' was not found in {filename}.")


def fetchpara(filename, tag_path, attribute=None):
    file_path = DATA_DIR / filename
    root = ET.parse(file_path).getroot()
    element = root.find(tag_path)

    if element is None:
        raise ValueError(f"XML tag '{tag_path}' was not found in {filename}.")

    value = element.get(attribute) if attribute is not None else element.text
    if value is None or not value.strip():
        raise ValueError(f"No value found at '{tag_path}' in {filename}.")

    value = value.strip()
    try:
        return float(value)
    except ValueError:
        return value


def try_load(loader, *args):
    try:
        return loader(*args)
    except (OSError, ET.ParseError, ValueError) as error:
        print(f"[DATA WARNING] Could not load {args}: {error}")
        return None



def load_geometry_recovery():
    """Load supplied geometry with per-item citations, without promoting claims to verified data."""
    root = ET.parse(DATA_DIR / 'geometry.xml').getroot()
    section = root.find('notebooklm_geometry_recovery')
    if section is None:
        raise ValueError('NotebookLM geometry recovery section is missing.')
    return {node.attrib['name']: dict(node.attrib, value=node.text)
            for node in section.findall('item')}


def recovery_length_ft(items, name):
    """Convert a cited length to feet; reject unknown units and invalid numbers."""
    item = items[name]
    factors = {'in': 1 / 12, 'ft': 1.0, 'm': 1 / 0.3048}
    if item['unit'] not in factors:
        raise ValueError(f"Unsupported length unit: {item['unit']}")
    value = float(item['value'])
    if not math.isfinite(value):
        raise ValueError(f'Nonfinite geometry value: {name}')
    return value * factors[item['unit']]


def load_parameter(records, name, filename, path, unit, meaning, *, used=False, text=False):
    """Read one value and keep its source and failure in the shared load ledger."""
    row = dict(name=name, meaning=meaning, file=filename, path=path, value=None,
               unit=unit, status='MISSING', reason='', used=used, citation={})
    records[name] = row
    try:
        root = ET.parse(DATA_DIR / filename).getroot()
        element_path, separator, attribute = path.partition('/@')
        node = root.find(element_path)
        if node is None:
            raise KeyError(f'XML element not found: {element_path}')
        # Metadata may be inherited from a containing section.
        def inherit(current, parents):
            combined = dict(parents, **current.attrib)
            if current is node:
                return combined
            for child in current:
                found = inherit(child, combined)
                if found is not None:
                    return found
            return None
        metadata = inherit(root, {})
        row['citation'] = {key: metadata[key] for key in
                           ('source', 'location', 'url', 'provenance', 'citation_status') if key in metadata}
        source_unit = node.get(f'{attribute}_unit') if separator else node.get('unit')
        source_unit = source_unit or ('1' if unit == '1' else None)
        row['source_unit'] = source_unit
        raw = node.get(attribute) if separator else node.text
        if raw is None:
            raise KeyError(f'XML value/attribute missing at {path}')
        if not raw.strip():
            raise ValueError(f'Empty XML value at {path}')
        if text:
            value = raw.strip()
        else:
            value = float(raw)
            if not math.isfinite(value):
                raise ValueError('Numeric value must be finite')
            if source_unit != unit:
                lengths = {'in': 1 / 12, 'ft': 1.0, 'm': 1 / .3048}
                if source_unit in lengths and unit in lengths:
                    value *= lengths[source_unit] / lengths[unit]
                else:
                    raise ValueError(f'Expected unit {unit}; found {source_unit!r}')
        row['value'] = value
        supplied = 'unverified' in metadata.get('status', '').lower()
        row['status'] = 'UNVERIFIED' if supplied else 'LOADED'
        row['reason'] = ('Existing data marked unverified in XML.' if supplied else
                         'Loaded successfully; LOADED does not assert independent verification.')
        return value
    except (FileNotFoundError, KeyError) as error:
        row.update(status='MISSING', reason=str(error))
    except (OSError, ET.ParseError, ValueError, TypeError) as error:
        row.update(status='INVALID', reason=f'{type(error).__name__}: {error}')
    return None


def load_recovery_items(records):
    """Keep citations and isolate malformed recovery items from other parameters."""
    path = 'notebooklm_geometry_recovery'
    try:
        items = load_geometry_recovery()
    except (OSError, ET.ParseError, ValueError, KeyError) as error:
        records['geometry_recovery'] = dict(name='geometry_recovery', meaning='Supplied geometry collection',
            file='geometry.xml', path=path, value=None, unit='mixed', used=False,
            status='MISSING' if isinstance(error, (FileNotFoundError, KeyError)) or 'section is missing' in str(error) else 'INVALID',
            reason=f'{type(error).__name__}: {error}')
        return None
    records['geometry_recovery'] = dict(name='geometry_recovery', meaning='Supplied geometry collection',
        file='geometry.xml', path=path, value=f'{len(items)} items', unit='mixed', used=False,
        status='UNVERIFIED', reason='Collection loaded; see individual item validation and citations.')
    for name, item in items.items():
        unit = item.get('unit', '')
        item_path = f"{path}/item[@name='{name}']"
        value = load_parameter(records, f'geometry_recovery.{name}', 'geometry.xml',
                               item_path, unit, item.get('definition', name), text=unit == 'text')
        if value is not None:
            item['value'] = value
    return items


def physics_diagnostics(flow, aero):
    """Classify existing runtime results without changing or recomputing physics."""
    force = aero['force_body_n']
    condition = flow_condition(flow)
    unsupported = condition in ('Reverse flow', 'Near-normal flow', 'Spanwise flow')
    return {
        'Airspeed': ('CALCULATED', format_quantity(flow['speed'], 'm/s')),
        'Wing AoA': ('CALCULATED' if flow['alpha'] is not None else 'UNAVAILABLE',
                     angle_text(flow['alpha']) if flow['alpha'] is not None else flow['status']),
        'CL coefficient': ('UNAVAILABLE', 'No supported CL model available; approximate XML descriptors are not lookup arrays.'),
        'Wing freestream force': ('CALCULATED' if force is not None else 'UNSUPPORTED' if unsupported else 'UNAVAILABLE', aero['status']),
        'Wing lift': ('CALCULATED' if force == (0.0, 0.0, 0.0) else 'UNSUPPORTED' if unsupported else 'UNAVAILABLE',
                      '0.00 N; zero freestream only, rotor wash excluded.' if force == (0.0, 0.0, 0.0) else aero['status']),
        'Rotor thrust': ('NOT IMPLEMENTED', 'Supported XV-15 performance map missing; see primary takeoff diagnostics.'),
    }


def report_diagnostics(aircraft, flow, aero, takeoff_result=None):
    print('\nTMPE :: DIAGNOSTIC REPORT\nAIRCRAFT DATA')
    for name, row in aircraft.diagnostics.items():
        print(f"  {name}: {row['status']} | {row['meaning']}")
        print(f"    Source: {row['file']} :: {row['path']}")
        print(f"    Value: {row['value']!r} {row['unit']} | "
              f"{'used in live physics' if row['used'] else 'not used in live physics'}")
        if row.get('source_unit') != row['unit'] and 'source_unit' in row:
            print(f"    XML unit: {row['source_unit']!r}")
        print(f"    {row['reason']}")
        if row.get('dependencies'):
            print('    Inputs: ' + ', '.join(row['dependencies']))
        if row.get('citation'):
            print('    Citation: ' + '; '.join(f'{key}={value}' for key, value in row['citation'].items()))
    results = physics_diagnostics(flow, aero)
    if takeoff_result is not None and takeoff_result.get('rotor_model') in ('prototype-fixed-ct', 'prototype-collective'):
        ready = all(takeoff_result[side]['value'] is not None for side in ('left', 'right'))
        results['Rotor thrust'] = ('CALCULATED' if ready else 'UNAVAILABLE', takeoff_result['rotor_model'] + '; see primary rotor results and assumptions')
    print('\nPHYSICS')
    for name, (status, reason) in results.items():
        print(f'  {name}: {status} — {reason}')
    print('\nBLOCKERS')
    for name, (status, reason) in results.items():
        if status != 'CALCULATED':
            print(f'  {name}: {reason}')
    if flow['wing'] is None:
        row = aircraft.diagnostics['wing_incidence_deg']
        print(f"  Wing AoA input: {row['file']} :: {row['path']} — {row['reason']}")


class Aircraft:
    """Selected XV-15 parameters loaded from XML."""

    def __init__(self):
        self.diagnostics = {}
        # name, XML file/path, expected unit, meaning, used in live physics
        specs = (
            ('design_gross', 'mass.xml', ".//*[@name='design_gross']", 'lb', 'Selected design-gross takeoff weight', True),
            ('wing_span_ft', 'geometry.xml', 'geometry/wing/span', 'ft', 'Wing span', False),
            ('wing_area_ft2', 'geometry.xml', 'geometry/wing/area', 'ft^2', 'Wing reference area', False),
            ('wing_chord_ft', 'geometry.xml', 'geometry/wing/chord', 'ft', 'Wing chord', False),
            ('wing_aspect_ratio', 'geometry.xml', 'geometry/wing/aspect_ratio', '1', 'Wing aspect ratio', False),
            ('wing_incidence_deg', 'geometry.xml', 'geometry/wing/incidence', 'deg', 'Wing mounting incidence', True),
            ('fuselage_length_ft', 'geometry.xml', 'geometry/fuselage/@length', 'ft', 'Fuselage length', False),
            ('rotor_diameter_ft', 'rotor.xml', ROTOR_DIAMETER_XML_PATH, 'ft', 'Rotor diameter', False),
            ('rotor_blades_per_hub', 'rotor.xml', 'proprotor_and_drive_system/rotor_blades_per_hub', '1', 'Blades per hub', False),
        )
        for name, filename, path, unit, meaning, used in specs:
            value = load_parameter(self.diagnostics, name, filename, path, unit, meaning, used=used)
            setattr(self, name, value)
        for name, filename, path, unit, meaning in (
            ('prototype_thrust_coefficient', 'rotor.xml', 'proprotor_and_drive_system/prototype_thrust_model/thrust_coefficient', '1', 'Derived illustrative hover-equilibrium CT assumption; not verified XV-15 data'),
            ('engine_takeoff_shp', 'engine.xml', "engine_data/power_rating[@name='takeoff']", 'SHP', 'Per-engine rated takeoff power; not mast power'),
            ('hover_rotor_rpm', 'rotor.xml', HOVER_RPM_XML_PATH, 'rpm', 'Design hover rotor speed accepted as a fixed model input'),
            ('transmission_efficiency', 'relationships.xml', "reported_relationships/relationship[@name='transmission_efficiency']/efficiency", '1', 'Reported mast/engine power ratio; conditional applicability'),
            ('engine_count', 'aircraft.xml', 'aircraft_overview/engine_count', '1', 'Installed engine count'),
            ('rotor_disk_area_ft2', 'rotor.xml', 'proprotor_and_drive_system/rotor_disc_area_per_rotor', 'ft^2', 'Per-rotor disk area'),
            ('blade_chord_ft', 'rotor.xml', 'proprotor_and_drive_system/blade_chord', 'ft', 'Blade chord converted from inches'),
            ('rotor_delta3_deg', 'rotor.xml', ROTOR_DELTA3_XML_PATH, 'deg', 'Signed pitch-flap coupling angle delta3'),
            ('rotor_precone_deg', 'rotor.xml', ROTOR_PRECONE_XML_PATH, 'deg', 'Built-in hub precone angle'),
            ('hover_download_fraction', 'rotor.xml', 'proprotor_and_drive_system/wing_download/fraction', '1', 'Provisional OGE hover download fraction'),
            ('hover_drive_engine_limit_shp', 'rotor.xml', 'proprotor_and_drive_system/hover_takeoff_drive_limits/engine_power_limit', 'SHP', 'Per-side installed takeoff engine power reference'),
        ):
            setattr(self, name, load_parameter(self.diagnostics, name, filename, path, unit, meaning))
        self.nacelle_angle_deg = load_parameter(self.diagnostics, 'nacelle_angle_deg',
            'aircraft.xml', 'helicopter_configuration/nacelle_angle', 'deg',
            'Helicopter nacelle angle; 0 forward / 90 vertical', used=True)
        self.transmission_relationship = load_parameter(self.diagnostics, 'transmission_relationship',
            'relationships.xml', "reported_relationships/relationship[@name='transmission_efficiency']/equation",
            'text', 'Reported efficiency relationship; applicability not established', text=True)
        if self.design_gross is not None and self.design_gross <= 0:
            self.design_gross = None
            self.diagnostics['design_gross'].update(value=None, status='INVALID', reason='Selected gross weight must be positive.')
        self.geometry_recovery = load_recovery_items(self.diagnostics)
        self.rotor_hub_positions_frame = 'aircraft datum FS/BL/WL, ft; aft/right/up'
        self.rotor_hub_positions = self._recovered_group('rotor_hub_positions', {
            side: [f'hub_{side}_{axis}_in' for axis in ('fs', 'bl', 'wl')]
            for side in ('left', 'right')})
        self.fuselage_width_range_ft = self._recovered_group('fuselage_width_range_ft',
            ['fuselage_exterior_width_min_ft', 'fuselage_exterior_width_max_ft'])
        for name, item in (
            ('fuselage_height_ft', 'fuselage_exterior_depth_ft'),
            ('fuselage_interior_width_ft', 'fuselage_interior_max_width_ft'),
            ('rotor_radius_ft', 'rotor_radius_ft'),
        ):
            setattr(self, name, self._recovered_group(name, [item], scalar=True))
        self.fuselage_width_ft = None
        self.isolated_nacelle_polars = None
        for name, filename, path, meaning, reason in (
            ('fuselage_width_ft', 'geometry.xml', 'notebooklm_geometry_recovery',
             'Single exterior width', 'Intentionally None: only a 5.0–5.2 ft range was supplied.'),
            ('isolated_nacelle_polars', 'aerodynamics.xml', 'aerodynamics/wing_pylon_coefficients/numerical_lookup_arrays',
             'Isolated nacelle aerodynamic polars', 'Intentionally None: supported numerical coefficient arrays are not supplied.'),
        ):
            self.diagnostics[name] = dict(name=name, meaning=meaning, file=filename, path=path,
                value=None, unit='ft' if name == 'fuselage_width_ft' else '1',
                status='MISSING', reason=reason, used=False)

    def _recovered_group(self, name, keys, scalar=False):
        """Only build a derived field when every cited input loaded successfully."""
        flat = [key for group in keys.values() for key in group] if isinstance(keys, dict) else keys
        dependencies = [f'geometry_recovery.{key}' for key in flat]
        for key, dependency in zip(flat, dependencies):
            if dependency not in self.diagnostics:
                load_parameter(self.diagnostics, dependency, 'geometry.xml',
                    f"notebooklm_geometry_recovery/item[@name='{key}']", 'ft',
                    key.replace('_', ' '))
            elif self.diagnostics[dependency]['value'] is not None:
                try:
                    recovery_length_ft(self.geometry_recovery, key)
                except (KeyError, ValueError, TypeError) as error:
                    self.diagnostics[dependency].update(value=None, status='INVALID',
                        reason=f'{type(error).__name__}: {error}')
        failures = [key for key in dependencies
                    if key not in self.diagnostics or self.diagnostics[key]['value'] is None]
        value = None
        if not failures:
            def lengths(group):
                return tuple(recovery_length_ft(self.geometry_recovery, key) for key in group)
            value = {side: lengths(group) for side, group in keys.items()} if isinstance(keys, dict) else lengths(keys)
            if scalar:
                value = value[0]
        status = 'UNVERIFIED' if not failures else (
            'INVALID' if any(self.diagnostics.get(key, {}).get('status') == 'INVALID' for key in failures) else 'MISSING')
        self.diagnostics[name] = dict(name=name, meaning=name.replace('_', ' '),
            file='geometry.xml', path='notebooklm_geometry_recovery/item', value=value,
            unit='ft', status=status, used=False, dependencies=dependencies,
            reason='Blocked by: ' + ', '.join(failures) if failures else 'Derived from cited recovery items; not used by live physics.')
        return value

    @property
    def wing_area_m2(self):
        """Convert square feet to square metres for SI calculations."""
        if self.wing_area_ft2 is None:
            return None
        return self.wing_area_ft2 * 0.09290304



class Environment:
    def __init__(
        self,
        windspeed=5.0,
        winddirection=180.0,
        air_density=1.225,
        temperature=15.0,
        altitude=0.0,
        *,
        input_statuses=None,
    ):
        self.windspeed = windspeed
        self.winddirection = winddirection
        self.air_density = air_density
        self.temperature = temperature
        self.altitude = altitude
        self.input_statuses = {} if input_statuses is None else dict(input_statuses)


FLOW_EPS_MPS = 1e-6


@dataclass(frozen=True)
class AircraftState:
    ground_velocity_ned: tuple = (0.0, 0.0, 0.0)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    height_agl_m: float = 0.0


def world_to_body_matrix(state):
    """Transpose of body-to-NED yaw/pitch/roll rotation (forward/right/down)."""
    cr, sr = math.cos(state.roll), math.sin(state.roll)
    cp, sp = math.cos(state.pitch), math.sin(state.pitch)
    cy, sy = math.cos(state.yaw), math.sin(state.yaw)
    return (
        (cp * cy, cp * sy, -sp),
        (sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp),
        (cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp),
    )


def transform(matrix, vector):
    return tuple(sum(a * b for a, b in zip(row, vector)) for row in matrix)


def wind_velocity_ned(speed, from_degrees):
    direction = math.radians(from_degrees)
    return (-speed * math.cos(direction), -speed * math.sin(direction), 0.0)


def calculate_aoa(state, environment, incidence_deg):
    values = (*state.ground_velocity_ned, state.roll, state.pitch, state.yaw,
              environment.windspeed, environment.winddirection)
    if not all(math.isfinite(v) for v in values) or environment.windspeed < 0:
        raise ValueError("State and wind must be finite, with non-negative wind speed.")
    wind = wind_velocity_ned(environment.windspeed, environment.winddirection)
    # Air-relative velocity = ground velocity minus wind, in NED axes.
    air_world = tuple(g - w for g, w in zip(state.ground_velocity_ned, wind))
    rotation = world_to_body_matrix(state)
    body = transform(rotation, air_world)
    body_alpha = (math.atan2(body[2], body[0])
                  if math.hypot(body[0], body[2]) >= FLOW_EPS_MPS else None)
    result = dict(wind=wind, air_world=air_world, rotation=rotation,
                  body=body, body_alpha=body_alpha, speed=math.hypot(*air_world),
                  wing=None, alpha=None, wing_rotation=None)
    if incidence_deg is None:
        result['status'] = 'unavailable: wing incidence missing'
        return result
    if not math.isfinite(incidence_deg):
        raise ValueError("Wing incidence must be finite.")
    # Apply mounting incidence once, then alpha = atan2(wing down, forward).
    i = math.radians(incidence_deg)
    wing_rotation = ((math.cos(i), 0.0, -math.sin(i)),
                     (0.0, 1.0, 0.0),
                     (math.sin(i), 0.0, math.cos(i)))
    wing = transform(wing_rotation, body)
    result.update(wing=wing, wing_rotation=wing_rotation)
    if math.hypot(wing[0], wing[2]) < FLOW_EPS_MPS:
        result['status'] = 'undefined: insufficient chord-plane airflow'
    else:
        result.update(alpha=math.atan2(wing[2], wing[0]), status='available')
    return result


def wing_aerodynamics(flow):
    if flow['speed'] == 0.0:
        return dict(status='zero freestream force; rotor wash excluded',
                    force_body_n=(0.0, 0.0, 0.0))
    regime = ''
    if flow['wing'] is not None and flow['wing'][0] <= FLOW_EPS_MPS:
        regime = '; unsupported reverse/near-normal/spanwise flow'
    return dict(status=('unavailable: CL/CD/Cm coefficient model missing; '
                        + ('wing AoA ' + flow['status']) + regime), force_body_n=None)


def format_number(value, places=2):
    """Round only the display string; remove a minus sign on displayed zero."""
    displayed = f"{value:.{places}f}"
    return displayed[1:] if displayed.startswith('-') and float(displayed) == 0 else displayed


def format_quantity(value, unit, places=2, reason="data missing"):
    if value is None:
        return f"Unavailable — {reason}"
    return f"{format_number(value, places)}{unit if unit == '°' else ' ' + unit}"


def format_vector(vector, unit="m/s", places=2, reason="wing incidence missing"):
    if vector is None:
        return f"Unavailable — {reason}"
    return f"({', '.join(format_number(v, places) for v in vector)}) {unit}"


def angle_text(angle, reason="insufficient chord-plane airflow"):
    return format_quantity(None if angle is None else math.degrees(angle),
                           "°", 1, reason)


def flow_condition(flow):
    if flow['speed'] == 0.0:
        return "No freestream airflow"
    if flow['wing'] is None:
        return "Unavailable — wing incidence missing"
    u, v, _ = flow['wing']
    if flow['alpha'] is None:
        return "Spanwise flow" if abs(v) >= FLOW_EPS_MPS else "Insufficient chord-plane airflow"
    if u < -FLOW_EPS_MPS:
        return "Reverse flow"
    if u <= FLOW_EPS_MPS:
        return "Near-normal flow"
    return "Forward flow"


def report_results(flow, aero):
    print("\n--- Results ---")
    print(f"{'Airspeed:':<19}{format_quantity(flow['speed'], 'm/s')}")
    reason = "wing incidence missing" if flow['wing'] is None else "insufficient chord-plane airflow"
    print(f"{'Wing AoA:':<19}{angle_text(flow['alpha'], reason)}")
    lift = "0.00 N" if aero['force_body_n'] == (0.0, 0.0, 0.0) else "Unavailable — aerodynamic model missing"
    print(f"{'Wing lift:':<19}{lift}")
    print(f"{'Flow condition:':<19}{flow_condition(flow)}")


def report_matrix(label, matrix):
    print(f"{label} (dimensionless):")
    if matrix is None:
        print("  Unavailable — wing incidence missing")
    else:
        for row in matrix:
            print("  [" + "  ".join(f"{format_number(v, 6):>9}" for v in row) + "]")


def report_aoa(state, aircraft, flow):
    """Verbose trace only; display rounding never feeds back into calculations."""
    print("\n--- Freestream AoA calculation trace ---")
    print("Prescribed ground velocity NED:", format_vector(state.ground_velocity_ned))
    print("Wind: meteorological FROM, clockwise from north; vertical wind = 0.00 m/s")
    print("Wind velocity NED:", format_vector(flow['wind']))
    print("V_air_NED = V_ground_NED - V_wind_NED:")
    print(f"  {format_vector(state.ground_velocity_ned)} - {format_vector(flow['wind'])}")
    print("  =", format_vector(flow['air_world']))
    print("Incoming airflow = -V_air; airspeed:", format_quantity(flow['speed'], 'm/s'))
    angles = tuple(math.degrees(v) for v in (state.roll, state.pitch, state.yaw))
    print("Prescribed roll/pitch/yaw:", format_vector(angles, 'deg', 1))
    report_matrix("R_world_to_body = transpose(R_body_to_NED)", flow['rotation'])
    print("V_air_body = R_world_to_body @ V_air_NED:", format_vector(flow['body']))
    print("Body AoA:", angle_text(flow['body_alpha']))
    print("Wing incidence from XML:", format_quantity(aircraft.wing_incidence_deg, '°', 1,
                                                     'wing incidence missing'))
    report_matrix("R_body_to_wing (reference wing; incidence only)", flow['wing_rotation'])
    print("V_air_wing = R_body_to_wing @ V_air_body:", format_vector(flow['wing']))
    if flow['alpha'] is None:
        reason = "wing incidence missing" if flow['wing'] is None else "insufficient chord-plane airflow"
        print("Wing AoA:", angle_text(None, reason))
    else:
        u, _, w = flow['wing']
        print(f"Wing chordwise u: {format_quantity(u, 'm/s')}; normal w: {format_quantity(w, 'm/s')}")
        if format_number(u) == format_number(w) == '0.00':
            chord_speed = math.hypot(u, w)
            print("For readable atan2 inputs, normalize by chord-plane speed (dimensionless):")
            print(f"  u/|V_xz| = {format_number(u / chord_speed, 6)}, w/|V_xz| = {format_number(w / chord_speed, 6)}")
            print(f"Wing AoA = atan2(w/|V_xz|, u/|V_xz|) = atan2({format_number(w / chord_speed, 6)}, {format_number(u / chord_speed, 6)}) = {angle_text(flow['alpha'])}")
        else:
            print(f"Wing AoA = atan2(w, u) = atan2({format_number(w)}, {format_number(u)}) = {angle_text(flow['alpha'])}")
    print(f"AoA unavailable when chord-plane speed < {FLOW_EPS_MPS:.1e} m/s.")
    print("Trace values are rounded for display; calculations use full precision.")


def report_details(environment, aircraft, aero):
    print("\n--- Environment and aircraft data ---")
    for label, value in (
        ("Air density", format_quantity(environment.air_density, 'kg/m³')),
        ("Wind speed", format_quantity(environment.windspeed, 'm/s')),
        ("Wind FROM direction", format_quantity(environment.winddirection, '°', 1)),
        ("Temperature", format_quantity(environment.temperature, '°C', 1)),
        ("Ground elevation (MSL)", format_quantity(environment.altitude, 'm')),
        ("Design gross", format_quantity(aircraft.design_gross, 'lb')),
        ("Wing span", format_quantity(aircraft.wing_span_ft, 'ft')),
        ("Wing area", format_quantity(aircraft.wing_area_ft2, 'ft²')),
        ("Wing area (SI)", format_quantity(aircraft.wing_area_m2, 'm²')),
        ("Rotor diameter", format_quantity(aircraft.rotor_diameter_ft, 'ft')),
        ("Rotor blades/hub", format_quantity(aircraft.rotor_blades_per_hub, 'blades', 0)),
        ("Wing freestream force (body)", format_vector(aero['force_body_n'], 'N',
                                                     reason='aerodynamic model missing')),
    ):
        print(f"{label}: {value}")


def ask_float(prompt, default=None, minimum=None, places=2, *, with_status=False):
    while True:
        suffix = f" [{format_number(default, places)}]" if default is not None else " [Enter to leave unknown]"
        answer = input(f"{prompt}{suffix}: ").strip()

        if not answer:
            if with_status:
                return default, 'DEFAULTED' if default is not None else 'UNAVAILABLE'
            return default

        try:
            value = float(answer)
        except ValueError:
            print("Please enter a valid number.")
            continue

        if not math.isfinite(value):
            print("Please enter a finite number.")
            continue

        if minimum is not None and value < minimum:
            print(f"Please enter a value of at least {minimum}.")
            continue

        return (value, 'USER-PRESCRIBED') if with_status else value


def calculate_lift(air_density, airspeed, wing_area_m2, lift_coefficient):
    if None in (air_density, airspeed, wing_area_m2, lift_coefficient):
        return None

    if air_density < 0 or airspeed < 0 or wing_area_m2 <= 0:
        raise ValueError("Density and airspeed must be non-negative; wing area must be positive.")

    # Wing lift L = 1/2 rho V² S CL; no supported CL model is supplied here.
    return 0.5 * air_density * airspeed**2 * wing_area_m2 * lift_coefficient



# Nacelle convention independently checked in NASA TM-81244, printed p. 4 / PDF p. 6.
NACELLE_SOURCE = 'https://ntrs.nasa.gov/api/citations/19810001546/downloads/19810001546.pdf'
LB_TO_KG = 0.45359237


def calculated(value, reason):
    if value is None:
        return unavailable('Explicit force provider returned unavailable data; ' + reason)
    if not math.isfinite(value):
        raise ValueError('Calculated values must be finite')
    return dict(value=value, status='CALCULATED', reason=reason)


def unavailable(reason, status='UNAVAILABLE'):
    return dict(value=None, status=status, reason=reason)


def require_finite(name, value, minimum=None):
    if not math.isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError(f'{name} must be finite' + (f' and at least {minimum}' if minimum is not None else ''))


def takeoff_mass_kg(gross_weight_lb):
    """Aviation gross weight in lb corresponds to lbm at standard gravity.

    Equivalently W_N = W_lbf * 4.4482216152605 and m = W_N / G.
    This selects the XML design-gross configuration, not measured loading.
    """
    if gross_weight_lb is None:
        return None
    require_finite('Gross weight', gross_weight_lb, 0)
    if gross_weight_lb == 0:
        raise ValueError('Takeoff mass must be positive')
    return gross_weight_lb * LB_TO_KG


def rotor_force_body_n(thrust_n, nacelle_angle_deg):
    """Rotor-axis thrust: nacelle 0° forward, 90° up; body z is down."""
    if thrust_n is None or nacelle_angle_deg is None:
        return None
    require_finite('Thrust', thrust_n, 0)
    require_finite('Nacelle angle', nacelle_angle_deg)
    angle = math.radians(nacelle_angle_deg)
    return (thrust_n * math.cos(angle), 0.0, -thrust_n * math.sin(angle))



@dataclass(frozen=True)
class LinearCollectiveModel:
    """Configuration-backed local CT law. evaluate(theta) is the replacement interface.

    A future measured curve/BEMT adapter can implement evaluate(theta) and return
    the same value/status/reason plus validation metadata without changing forces.
    """
    parameters: dict

    @classmethod
    def load(cls, path=None):
        path = Path(path) if path is not None else DATA_DIR / 'collective_model.json'
        try:
            parameters = json.loads(path.read_text())
            if not isinstance(parameters, dict):
                raise ValueError('Configuration must be an object')
            return cls(dict(parameters, configuration_source=str(path)))
        except (OSError, ValueError) as error:
            return cls({'load_error': str(error), 'configuration_source': str(path)})

    def evaluate(self, theta, *, allow_illustrative=False):
        p = self.parameters
        metadata = dict(model='linear-collective',
                        calibration_status='UNCALIBRATED',
                        experimental_validation_status='NOT VALIDATED',
                        configuration_source=p.get('configuration_source'),
                        source=p.get('source'), condition=p.get('reference_condition'),
                        theta_deg=theta)

        def fail(reason):
            return dict(unavailable(reason), **metadata)

        def finite(value):
            return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

        if not finite(theta):
            return fail('Pilot collective must be a finite number in degrees')
        if p.get('load_error'):
            return fail('UNCALIBRATED: configuration unavailable: ' + p['load_error'])
        bounds = p.get('pitch_bounds', {})
        if not isinstance(bounds, dict):
            return fail('Invalid pitch bounds configuration')
        lo, hi = bounds.get('minimum_deg'), bounds.get('maximum_deg')
        basis = bounds.get('basis')
        if (not finite(lo) or not finite(hi) or lo >= hi or
                basis not in ('documented', 'provisional-software') or not bounds.get('source')):
            return fail('Finite, sourced pitch bounds and documented/provisional-software basis required')
        metadata.update(pitch_bounds=dict(bounds))
        if not lo <= theta <= hi:
            return fail(f'Collective outside {basis} bounds [{lo}, {hi}] deg')
        names = ('CT_reference', 'k_theta_per_deg', 'theta_reference_deg')
        missing = [name for name in (*names, 'reference_condition') if p.get(name) is None]
        if missing:
            return fail('UNCALIBRATED: missing ' + ', '.join(missing))
        if any(not finite(p[name]) for name in names):
            return fail('UNCALIBRATED: model parameters must be finite numbers')
        if p['CT_reference'] < 0 or not lo <= p['theta_reference_deg'] <= hi:
            return fail('Invalid reference CT or reference pitch outside model bounds')
        if not p.get('source') or not p.get('reference_condition'):
            return fail('UNCALIBRATED: parameter source and reference condition required')
        kind = p.get('parameter_status')
        if kind == 'illustrative':
            if not allow_illustrative:
                return fail('Illustrative parameters require explicit mathematical-test opt-in')
            metadata.update(calibration_status='ILLUSTRATIVE — MATHEMATICAL TEST ONLY')
        elif kind == 'calibrated':
            metadata.update(calibration_status='CALIBRATED — EXPERIMENTAL VALIDATION SEPARATE')
        else:
            return fail('UNCALIBRATED: parameter_status must establish calibrated or illustrative data')
        ct = p['CT_reference'] + p['k_theta_per_deg'] * (theta - p['theta_reference_deg'])
        if not math.isfinite(ct) or ct < 0:
            return fail('Linear approximation produced negative/nonfinite CT; no extrapolated thrust available')
        return dict(calculated(ct, 'CT = CT_reference + k_theta * (theta - theta_reference)'), **metadata)


def calculate_collective_proprotor(side, aircraft, environment, theta, model=None, *, allow_illustrative=False):
    """Evaluate each side separately, then connect supported CT to the existing SI thrust law."""
    if side not in ('left', 'right'):
        raise ValueError('Proprotor side must be left or right')
    coefficient = (model if model is not None else LinearCollectiveModel.load()).evaluate(
        theta, allow_illustrative=allow_illustrative)
    calibration = coefficient.get('calibration_status') or ''
    kind = 'ILLUSTRATIVE LINEAR MODEL' if 'ILLUSTRATIVE' in calibration else 'CONFIGURED LINEAR MODEL'
    metadata = dict(coefficient, side=side, thrust_coefficient=coefficient['value'],
                    ct_status=coefficient['status'], ct_data_kind=kind, ct_reason=coefficient['reason'])
    if coefficient['value'] is None:
        return metadata
    thrust = calculate_fixed_ct_proprotor(side, aircraft, environment, ct_override=coefficient['value'])
    # Preserve force-law availability and SI inputs, replace fixed-CT provenance.
    for key in ('value', 'status', 'reason'):
        metadata.pop(key)
    if thrust['status'] == 'CALCULATED':
        thrust['reason'] = coefficient['reason'] + '; T = CT rho A (Omega R)^2'
    thrust.update(metadata, label='LOCAL COLLECTIVE APPROXIMATION — ' + coefficient['calibration_status'],
                  source_status=coefficient['calibration_status'],
                  assumptions=('Local pitch response only; fixed RPM. No inflow, stall, power limits, '
                               'ground effect, interference, wind response or realistic climb prediction.'))
    return thrust


def calculate_proprotor(side, aircraft, environment, *, rotor_model='unsupported', ct_override=None,
                       collective_deg=None, ct_model=None, allow_illustrative=False):
    """Evaluate one real-rotor model's readiness; never infer thrust from rating.

    RPM, diameter and engine rating do not establish collective, mast power or
    CT/CQ. The opt-in fixed-CT path is an illustrative force assumption.
    A hover momentum estimate also needs explicit induced-power input
    and cannot be reused as a climb/ground-effect thrust law.
    """
    if side not in ('left', 'right'):
        raise ValueError('Proprotor side must be left or right')
    if rotor_model in ('linear-collective', 'prototype-collective'):
        if ct_override is not None:
            raise ValueError('CT override requires prototype-fixed-ct mode')
        return calculate_collective_proprotor(side, aircraft, environment, collective_deg, ct_model,
                                             allow_illustrative=allow_illustrative)
    if collective_deg is not None or ct_model is not None or allow_illustrative:
        raise ValueError('Collective inputs require linear-collective mode')
    if rotor_model == 'prototype-fixed-ct':
        return calculate_fixed_ct_proprotor(side, aircraft, environment, ct_override=ct_override)
    if rotor_model != 'unsupported':
        raise ValueError(f'Unknown rotor model: {rotor_model}')
    if ct_override is not None:
        raise ValueError('CT override requires prototype-fixed-ct mode')
    missing = []
    for label, value in (('rotor diameter', aircraft.rotor_diameter_ft),
                         ('hover RPM', aircraft.hover_rotor_rpm),
                         ('engine takeoff rating', aircraft.engine_takeoff_shp)):
        if value is None:
            missing.append(label)
        elif not math.isfinite(value) or value <= 0:
            return unavailable(f'{side}: invalid {label}')
    if not math.isfinite(environment.air_density) or environment.air_density <= 0:
        return unavailable(f'{side}: positive finite air density required')
    missing.extend((f'{side} commanded/available rotor mast power',
                    f'{side} collective and supported thrust/torque performance map'))
    result = unavailable('; '.join(missing))
    result.update(side=side, rotor_shaft_power_w=None,
                  model='XV-15 thrust model unavailable; ideal hover estimate not substituted',
                  source='rotor.xml: proprotor_and_drive_system; engine.xml: engine_data; relationships.xml: transmission_efficiency')
    return result


PROTOTYPE_LABEL = 'PROTOTYPE ESTIMATE — FIXED CT, FIXED RPM'
PROTOTYPE_EXCLUSIONS = ('Wing download, ground effect, rotor interference and other aerodynamic loads '
                        'are excluded from this force balance, not established as physically zero. '
                        'Entered wind is excluded from rotor thrust and motion; no windy-hover claim. '
                        'Fixed CT/RPM do not respond to collective, inflow, climb speed or active control. '
                        'No realistic climb performance or engine power demand is predicted.')


COLLECTIVE_EXCLUSIONS = ('Wing download, ground effect, rotor interference and other aerodynamic loads '
                         'are EXCLUDED, not measured zero. Wind, inflow response, stall and power limits '
                         'are excluded. Fixed RPM; constant selected pitch per rotor. '
                         'Software mechanics exercise; NOT XV-15 aircraft predictions.')


def calculate_fixed_ct_proprotor(side, aircraft, environment, *, ct_override=None):
    """One independent rotor evaluation in the rotorcraft CT convention.

    D_m = D_ft * 0.3048; R = D_m/2; A = pi R²; Omega = RPM*2*pi/60.
    T = CT rho A (Omega R)², with the unprimed shaft speed per CR-177436
    List of Symbols pp. xiv–xv. No engine/shaft/induced power is inferred.
    """
    if side not in ('left', 'right'):
        raise ValueError('Proprotor side must be left or right')
    ct = aircraft.prototype_thrust_coefficient if ct_override is None else ct_override
    inputs = dict(rotor_diameter_ft=aircraft.rotor_diameter_ft,
                  hover_rotor_rpm=aircraft.hover_rotor_rpm,
                  air_density=environment.air_density, thrust_coefficient=ct)
    metadata = dict(side=side, model='prototype-fixed-ct', label=PROTOTYPE_LABEL,
                    source_status='ILLUSTRATIVE / SOFTWARE-TEST ONLY — not measured or verified XV-15 data',
                    source=('explicit CT override assumption' if ct_override is not None else
                            'rotor.xml: proprotor_and_drive_system/prototype_thrust_model/thrust_coefficient'),
                    thrust_coefficient=ct, ct_status='ASSUMED' if ct is not None else 'UNAVAILABLE',
                    ct_data_kind='RUNTIME ASSUMPTION' if ct_override is not None else 'XML PROTOTYPE ASSUMPTION',
                    assumptions=PROTOTYPE_EXCLUSIONS, inputs=inputs,
                    engine_power_w=None, rotor_shaft_power_w=None, induced_power_w=None)
    errors = []
    for name, value in inputs.items():
        if value is None:
            errors.append(f'missing {name}')
        elif not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0 or (
                name in ('rotor_diameter_ft', 'air_density') and value == 0):
            errors.append(f'invalid {name}: {value!r}')
    if errors:
        return dict(unavailable(f"{side}: " + '; '.join(errors)), **metadata)
    try:
        diameter = inputs['rotor_diameter_ft'] * .3048
        radius = diameter / 2
        area = math.pi * radius**2
        omega = inputs['hover_rotor_rpm'] * (2 * math.pi / 60)
        thrust = ct * inputs['air_density'] * area * (omega * radius)**2
        result = calculated(thrust, 'T = CT rho A (Omega R)^2; ' + PROTOTYPE_LABEL)
    except (OverflowError, ValueError):
        return dict(unavailable(f'{side}: nonfinite derived thrust from supplied inputs'), **metadata)
    result.update(metadata, diameter_m=diameter, radius_m=radius, disk_area_m2=area,
                  angular_speed_rad_s=omega)
    return result


# Mechanical horsepower: 550 ft lbf/s, expressed in SI.
SHP_TO_W = 745.6998715822702


def ideal_hover_from_induced_power(diameter_m, density_kg_m3, induced_power_w):
    """Isolated uniform actuator disk in static OGE hover; never a climb law.

    T = 2 rho A vi² and Pi = T vi imply T = (Pi sqrt(2 rho A))**(2/3).
    Pi is ideal induced power, NOT engine or total rotor shaft power.
    """
    for name, value in (('Diameter', diameter_m), ('Density', density_kg_m3)):
        require_finite(name, value, 0)
        if value == 0:
            raise ValueError(name + ' must be positive')
    require_finite('Induced power', induced_power_w, 0)
    area = math.pi * (diameter_m / 2)**2
    thrust = (induced_power_w * math.sqrt(2 * density_kg_m3 * area))**(2 / 3)
    result = calculated(thrust, 'Ideal static isolated OGE disk; T = (Pi sqrt(2 rho A))^(2/3)')
    result.update(model='IDEAL HOVER ONLY — not an XV-15 prediction', thrust_n=thrust,
                  disk_area_m2=area, induced_power_w=induced_power_w,
                  induced_velocity_mps=math.sqrt(thrust / (2 * density_kg_m3 * area)))
    return result


def estimate_proprotor_hover(side, aircraft, environment, *, engine_power_w,
                             transmission_efficiency, ideal_power_fraction):
    """Explicit per-side power allocation; no implicit cross-shaft split.

    ideal_power_fraction = ideal induced power / rotor shaft power (a hover
    figure-of-merit style closure). No measured value is supplied by the XML.
    Setting it to 1 explicitly selects a lossless rotor idealization.
    """
    if side not in ('left', 'right'):
        raise ValueError('Proprotor side must be left or right')
    inputs = (engine_power_w, transmission_efficiency, ideal_power_fraction,
              aircraft.rotor_diameter_ft, environment.air_density)
    if any(value is None for value in inputs):
        result = unavailable('Explicit engine power, transmission efficiency, ideal power fraction, diameter and density required')
        result.update(side=side, engine_power_w=engine_power_w, rotor_shaft_power_w=None,
                      induced_power_w=None, thrust_n=None)
        return result
    require_finite('Engine power', engine_power_w, 0)
    for name, value in (('Transmission efficiency', transmission_efficiency),
                        ('Ideal power fraction', ideal_power_fraction)):
        require_finite(name, value, 0)
        if not 0 < value <= 1:
            raise ValueError(name + ' must be in (0, 1]')
    shaft = engine_power_w * transmission_efficiency
    result = ideal_hover_from_induced_power(aircraft.rotor_diameter_ft * .3048,
                                           environment.air_density, shaft * ideal_power_fraction)
    result.update(side=side, engine_power_w=engine_power_w, rotor_shaft_power_w=shaft,
                  transmission_efficiency=transmission_efficiency,
                  ideal_power_fraction=ideal_power_fraction,
                  assumptions='Explicit per-side engine allocation; steady still-air OGE hover; uniform incompressible disk; no swirl, profile loss model, ground effect, interference or wing download')
    return result


def report_rated_ideal_hover(aircraft, environment):
    """Conditional rating-based ceiling, separated from live motion and wind."""
    print('\nIDEAL RATED-POWER HOVER SCENARIO — NOT XV-15 PREDICTIONS')
    print('Assume each engine supplies its XML takeoff rating to its corresponding rotor; no crossfeed or accessory deduction.')
    print('Apply reported XML transmission efficiency conditionally; assume all mast power becomes ideal induced power (fraction = 1).')
    print('Still-air static OGE disks; entered wind and ground contact are excluded. No takeoff trajectory is inferred.')
    for side in ('left', 'right'):
        rating = aircraft.engine_takeoff_shp
        row = estimate_proprotor_hover(side, aircraft, environment,
            engine_power_w=None if rating is None else rating * SHP_TO_W,
            transmission_efficiency=aircraft.transmission_efficiency, ideal_power_fraction=1.0)
        print(f"  {side}: {row['status']} | {row['reason']}")
        for key, unit in (('engine_power_w', 'W'), ('rotor_shaft_power_w', 'W'),
                          ('induced_power_w', 'W'), ('thrust_n', 'N')):
            print(f"    {key}: {format_quantity(row[key], unit)}")
    print('Equations: Pshaft = eta Pengine; Pi = fraction Pshaft; A = pi (D/2)^2; T = (Pi sqrt(2 rho A))^(2/3).')


def vertical_force_balance(mass_kg, left_thrust_n, right_thrust_n,
                           left_angle_deg=90.0, right_angle_deg=90.0,
                           other_vertical_force_n=None):
    """Up positive. Other loads must be supplied explicitly, including exclusions."""
    if mass_kg is None:
        return unavailable('Selected takeoff mass missing: mass.xml design_gross')
    require_finite('Mass', mass_kg, 0)
    if mass_kg == 0:
        raise ValueError('Mass must be positive')
    weight = mass_kg * G
    left = rotor_force_body_n(left_thrust_n, left_angle_deg)
    right = rotor_force_body_n(right_thrust_n, right_angle_deg)
    rotor_up = None if left is None or right is None else -left[2] - right[2]
    if left is None or right is None or other_vertical_force_n is None:
        result = unavailable('Both rotor thrusts, nacelle angles and explicit other vertical loads are required')
        result.update(aircraft_weight_n=weight, total_rotor_vertical_force_n=rotor_up,
                      net_vertical_force_n=None, vertical_acceleration_mps2=None)
        return result
    require_finite('Other vertical force', other_vertical_force_n)
    net = rotor_up + other_vertical_force_n - weight
    result = calculated(net, 'F_up = T_L sin(beta_L) + T_R sin(beta_R) + F_other - m g; a = F_up / m')
    result.update(aircraft_weight_n=weight, total_rotor_vertical_force_n=rotor_up,
                  net_vertical_force_n=net, vertical_acceleration_mps2=net / mass_kg)
    return result


def integrate_vertical(height_agl_m, vertical_velocity_mps, mass_kg, net_vertical_force_n, timestep_s):
    """Exact constant-force step; forces must be reevaluated each timestep.

    Unilateral ground constraint: support cancels negative force while at rest.
    Any ground crossing is arrested at ground level with zero downward velocity.
    This contact constraint is not a landing or impact-response model.
    """
    for name, value in (('Height', height_agl_m), ('Velocity', vertical_velocity_mps),
                        ('Timestep', timestep_s)):
        require_finite(name, value)
    if height_agl_m < 0 or timestep_s <= 0:
        raise ValueError('Height must be nonnegative and timestep positive')
    if mass_kg is None or net_vertical_force_n is None:
        result = unavailable('Motion not advanced: mass or net vertical force unavailable')
        result.update(height_agl_m=height_agl_m, vertical_velocity_mps=vertical_velocity_mps,
                      vertical_acceleration_mps2=None, ground_support_n=None, takeoff_status='UNAVAILABLE', advanced=False)
        return result
    require_finite('Mass', mass_kg, 0)
    require_finite('Net force', net_vertical_force_n)
    if mass_kg == 0:
        raise ValueError('Mass must be positive')
    grounded = height_agl_m == 0 and vertical_velocity_mps <= 0
    support = max(0.0, -net_vertical_force_n) if grounded else 0.0
    acceleration = (net_vertical_force_n + support) / mass_kg
    velocity = max(0.0, vertical_velocity_mps) if grounded else vertical_velocity_mps
    height = height_agl_m + velocity * timestep_s + .5 * acceleration * timestep_s**2
    velocity += acceleration * timestep_s
    if height <= 0:
        height, velocity = 0.0, 0.0
    result = calculated(height, 'h_new = h + v dt + 0.5 a dt²; v_new = v + a dt; unilateral ground support')
    result.update(height_agl_m=height, vertical_velocity_mps=velocity,
                  vertical_acceleration_mps2=acceleration, ground_support_n=support,
                  takeoff_status='AIRBORNE' if height > 0 else 'GROUNDED', advanced=True)
    return result


def simulate_takeoff(aircraft, environment, duration_s=5.0, timestep_s=.1, force_provider=None,
                     *, rotor_model='unsupported', ct_override=None, collective_deg=None,
                     ct_model=None, allow_illustrative=False):
    """Run vertical-only motion; a supplied provider must establish forces each step.

    Provider(time, height, upward_velocity) returns left/right thrust and explicit
    other_vertical_force_n in SI. It is an extension point, not aircraft data.
    Default XV-15 execution stops at the first unsupported force evaluation.
    Prototype mode holds XML RPM and selected CT fixed, excluding other loads
    and wind explicitly; density is the scenario input throughout the run.
    """
    require_finite('Duration', duration_s, 0)
    require_finite('Timestep', timestep_s, 0)
    if timestep_s == 0:
        raise ValueError('Timestep must be positive')
    if rotor_model not in ('unsupported', 'prototype-fixed-ct', 'prototype-collective'):
        raise ValueError(f'Unknown rotor model: {rotor_model}')
    if ct_override is not None and rotor_model != 'prototype-fixed-ct':
        raise ValueError('CT override requires prototype-fixed-ct mode')
    prototype = rotor_model in ('prototype-fixed-ct', 'prototype-collective')
    if force_provider is not None and prototype:
        raise ValueError('Choose a prototype rotor model or an external force provider')
    if rotor_model != 'prototype-collective' and (collective_deg is not None or ct_model is not None or allow_illustrative):
        raise ValueError('Collective inputs require prototype-collective mode')
    if rotor_model == 'prototype-collective':
        ct_model = ct_model if ct_model is not None else LinearCollectiveModel.load()
    pitches = collective_deg if isinstance(collective_deg, dict) else dict(left=collective_deg, right=collective_deg)

    def rotor(side):
        options = dict(rotor_model=rotor_model, ct_override=ct_override)
        if rotor_model == 'prototype-collective':
            options.update(collective_deg=pitches.get(side), ct_model=ct_model, allow_illustrative=allow_illustrative)
        return calculate_proprotor(side, aircraft, environment, **options)
    for name in ('rotor_diameter_ft', 'hover_rotor_rpm', 'prototype_thrust_coefficient'):
        aircraft.diagnostics[name]['used'] = (prototype and
            (name != 'prototype_thrust_coefficient' or (rotor_model == 'prototype-fixed-ct' and ct_override is None)))
    mass = takeoff_mass_kg(aircraft.design_gross)
    time, height, velocity = 0.0, 0.0, 0.0
    history = []
    left = rotor('left')
    right = rotor('right')
    while True:
        if force_provider is None:
            left = rotor('left')
            right = rotor('right')
        # Algebraic zero implements explicit exclusions; it is not measured zero load.
        other = 0.0 if prototype else None
        if force_provider is not None:
            forces = force_provider(time, height, velocity)
            left = calculated(forces['left_proprotor_thrust_n'], 'Explicit external force provider; not validated XV-15 data')
            right = calculated(forces['right_proprotor_thrust_n'], 'Explicit external force provider; not validated XV-15 data')
            other = forces['other_vertical_force_n']
        balance = vertical_force_balance(mass, left['value'], right['value'],
                                         aircraft.nacelle_angle_deg, aircraft.nacelle_angle_deg, other)
        if aircraft.nacelle_angle_deg is not None and aircraft.nacelle_angle_deg != 90:
            balance.update(value=None, status='UNSUPPORTED', net_vertical_force_n=None,
                           vertical_acceleration_mps2=None,
                           reason='Vertical-only simulation requires the 90 degree helicopter configuration; horizontal force/attitude dynamics excluded.')
        dt = min(timestep_s, max(0.0, duration_s - time))
        motion = integrate_vertical(height, velocity, mass, balance.get('net_vertical_force_n'), dt or timestep_s)
        if dt == 0 and motion['advanced']:
            motion.update(height_agl_m=height, vertical_velocity_mps=velocity, advanced=False,
                          takeoff_status='AIRBORNE' if height > 0 else 'GROUNDED')
        if balance['status'] == 'UNSUPPORTED':
            motion.update(status='UNSUPPORTED', reason=balance['reason'])
        if motion['status'] != 'CALCULATED' or dt == 0:
            break
        time += dt
        height, velocity = motion['height_agl_m'], motion['vertical_velocity_mps']
        history.append(dict(time_s=time, **motion))
        if time >= duration_s:
            break
    return dict(left=left, right=right, balance=balance, motion=motion,
                left_proprotor_thrust_n=left['value'], right_proprotor_thrust_n=right['value'],
                takeoff_mass_kg=mass, elapsed_s=time, history=history,
                other_vertical_force_n=other, nacelle_angle_deg=aircraft.nacelle_angle_deg,
                rotor_model=rotor_model, excluded_loads=(COLLECTIVE_EXCLUSIONS if rotor_model == 'prototype-collective' else
                                                       PROTOTYPE_EXCLUSIONS if prototype else None))


@dataclass(frozen=True)
class TransitionState:
    """Inertial heading-aligned plane in SI, with positive altitude and up velocity.

    Phase 1 has no horizontal motion. Phase 2 pitch is prescribed for the
    airflow calculation, with nose-up positive. Nacelle tilt is a separate input.
    """
    height_agl_m: float
    vertical_velocity_mps: float
    elapsed_s: float
    left_proprotor_thrust_n: float | None
    right_proprotor_thrust_n: float | None
    left_nacelle_angle_deg: float | None
    right_nacelle_angle_deg: float | None
    forward_velocity_mps: float = 0.0
    forward_distance_m: float = 0.0
    pitch_attitude_deg: float | None = None
    heading_deg: float = 0.0


def initialize_transition(takeoff_result):
    """Copy the last retained Phase 1 state, including unsuccessful runs."""
    r = takeoff_result
    return TransitionState(r['motion']['height_agl_m'], r['motion']['vertical_velocity_mps'],
                           r['elapsed_s'], r['left_proprotor_thrust_n'], r['right_proprotor_thrust_n'],
                           r['nacelle_angle_deg'], r['nacelle_angle_deg'])


def validate_transition_angle(angle):
    """Provisional software range only; not XV-15 operational conversion limits."""
    if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or not 0 <= angle <= 90:
        raise ValueError('Nacelle angle must be finite within provisional software range [0, 90] degrees')
    return angle


def planar_force_balance(mass_kg, forward_force_n=None, upward_force_n=None):
    """Phase 2B interface: explicit NET forces, including gravity, in level axes.

    Missing forces remain unknown. Each available component is divided by mass.
    """
    if mass_kg is not None:
        require_finite('Mass', mass_kg, 0)
        if mass_kg == 0:
            raise ValueError('Mass must be positive')
    for force in (forward_force_n, upward_force_n):
        if force is not None:
            require_finite('Net force', force)
    return dict(forward_acceleration_mps2=None if mass_kg is None or forward_force_n is None else forward_force_n / mass_kg,
                vertical_acceleration_mps2=None if mass_kg is None or upward_force_n is None else upward_force_n / mass_kg)


@dataclass(frozen=True)
class TransitionPolar:
    """Strictly increasing wing AoA in degrees, with dimensionless CL and CD."""

    rows: tuple
    source: str
    status: str
    user_source: str | None = None

    @classmethod
    def load(cls, path, source=None):
        with Path(path).open(newline='', encoding='utf-8-sig') as stream:
            reader = csv.reader(stream)
            if next(reader, None) != ['alpha_deg', 'cl', 'cd']:
                raise ValueError('Coefficient CSV header must be alpha_deg,cl,cd (dimensionless CL and CD)')
            rows = []
            for line, row in enumerate(reader, 2):
                try:
                    alpha, cl, cd = map(float, row)
                    require_finite('alpha_deg', alpha)
                    require_finite('CL', cl)
                    require_finite('CD', cd, 0)
                    if rows and alpha <= rows[-1][0]:
                        raise ValueError('alpha_deg must be strictly increasing')
                except ValueError as exc:
                    raise ValueError(f'Coefficient CSV line {line}: {exc}') from exc
                rows.append((alpha, cl, cd))
        if len(rows) < 2:
            raise ValueError('Coefficient CSV requires at least two rows')
        return cls(tuple(rows), str(path), 'USER-SUPPLIED / UNVALIDATED', source)

    def coefficients(self, alpha_deg):
        if not self.rows[0][0] <= alpha_deg <= self.rows[-1][0]:
            return None
        for lower, upper in zip(self.rows, self.rows[1:]):
            if alpha_deg <= upper[0]:
                fraction = (alpha_deg - lower[0]) / (upper[0] - lower[0])
                return tuple(a + fraction * (b - a) for a, b in zip(lower[1:], upper[1:]))


SIMPLIFIED_TRANSITION_LABEL = 'SIMPLIFIED ILLUSTRATIVE TRANSITION'
# Editable software-test assumptions. CL and CD are dimensionless.
SIMPLIFIED_TRANSITION_CL = 0.50
SIMPLIFIED_TRANSITION_CD = 0.08


@dataclass(frozen=True)
class XV15RotorFlow:
    """Resolved C_RF and advance ratios, or externally supplied equivalents."""

    source: str
    left_force_coefficient: float | None = None
    right_force_coefficient: float | None = None
    left_advance_ratio: float | None = None
    right_advance_ratio: float | None = None
    assumptions: str = ''


@dataclass(frozen=True)
class XV15InPlaneState:
    """Sourced per-rotor inputs at the evaluated operating point.

    Use local rotor conventions for cyclic controls and wind-mast flapping.
    Angles are radians. The legacy H-bar coefficient field is audited as a
    candidate pending verification of the A-30 symbol.
    """

    source: str = ''
    assumptions: str = ''
    lift_slope_per_rad: float | None = None
    cs0: float | None = None
    cs1: float | None = None
    cs2: float | None = None
    cc1: float | None = None
    cc2: float | None = None
    h_lambda_coefficient: float | None = None
    profile_drag_factor: float | None = None
    induced_inflow: float | None = None
    inflow_distribution: float | None = None
    coning_rad: float | None = None
    longitudinal_flap_rad: float | None = None
    lateral_flap_rad: float | None = None
    lateral_cyclic_mast_rad: float | None = None
    longitudinal_cyclic_mast_rad: float | None = None
    # Optional Appendix A raw inputs. Each twist tuple is (TW1n, TW2n, TW3n, TW4n).
    blade_lift_a0: float | None = None
    blade_lift_a1: float | None = None
    blade_lift_a2: float | None = None
    root_collective_rad: float | None = None
    twist_terms_n0: tuple | None = None
    twist_terms_n1: tuple | None = None
    twist_terms_n2: tuple | None = None
    twist_terms_n3: tuple | None = None
    blade_drag_delta0: float | None = None
    blade_drag_delta1_per_rad: float | None = None
    blade_drag_delta2_per_rad2: float | None = None
    cdmax: float | None = None
    cdalph: float | None = None
    cdlim: float | None = None
    cdmach: float | None = None
    cdfact: float | None = None
    blade_flapping_inertia_kg_m2: float | None = None
    coning_hub_spring_nm_per_rad: float | None = None
    precone_rad: float | None = None
    tip_loss_factor: float | None = None
    ground_effect_factor: float | None = None
    washout: float | None = None
    side_by_side_factor: float | None = None
    tandem_factor: float | None = None
    # A-22 wind-mast flapping matrices (row-major), forcing, and prior a-bar1/b-bar1 in SI.
    flapping_c_matrix: tuple | None = None
    flapping_a_matrix: tuple | None = None
    flapping_forcing_nm: tuple | None = None
    flapping_prior_rad: tuple | None = None
    flapping_step_s: float | None = None


@dataclass(frozen=True)
class XV15RotorOperatingPoint:
    """Shaft RPM and mast-axis T, H, Y in newtons for the evaluated model state."""

    rpm: float | None = None
    t_n: float | None = None
    h_n: float | None = None
    y_n: float | None = None
    rpm_source: str = ''
    rpm_status: str = 'PROVIDED'
    t_source: str = ''
    t_status: str = 'PROVIDED'
    in_plane_force_source: str = ''
    in_plane_force_status: str = 'PROVIDED'
    in_plane_state: XV15InPlaneState | None = None


@dataclass(frozen=True)
class XV15RotorHubGeometry:
    """Matched CG and right-hand shaft pivot datum coordinates, all in meters.

    Datum tuples are station line (aft), butt line (right) and waterline (up).
    The A-19 left rotor equations mirror the right-hand lever terms.
    """

    cg_datum_m: tuple
    shaft_pivot_datum_m: tuple
    mast_length_m: float

    def lever_terms_m(self, beta_rad, phi_rad):
        cg = tuple(map(rotor_input_number, self.cg_datum_m))
        pivot = tuple(map(rotor_input_number, self.shaft_pivot_datum_m))
        mast_length = rotor_input_number(self.mast_length_m, positive=True)
        if mast_length is None or None in (*cg, *pivot):
            return (None, None, None)
        sl_cg, bl_cg, wl_cg = cg
        sl_sp, bl_sp, wl_sp = pivot
        mast_in_pitch_plane = mast_length * math.cos(phi_rad)
        return (
            sl_cg - sl_sp + mast_in_pitch_plane * math.sin(beta_rad),
            bl_sp - bl_cg + mast_length * math.sin(phi_rad),
            wl_sp - wl_cg + mast_in_pitch_plane * math.cos(beta_rad),
        )


@dataclass(frozen=True)
class XV15RotorInputs:
    """Per-step inputs in the CR-166536 conventions.

    Body U, V, W point forward, right and down. Rates are p, q, r in rad/s.
    Hub offsets are the report's common LXH, LYH, LZH lever terms in meters
    for symmetric hub geometry, with its left/right signs applied below.
    """

    source: str
    body_velocity_mps: tuple = (None, None, None)
    body_rates_rad_s: tuple = (None, None, None)
    hub_offsets_m: tuple = (None, None, None)
    lateral_mast_tilt_deg: float | None = None
    left: XV15RotorOperatingPoint = XV15RotorOperatingPoint()
    right: XV15RotorOperatingPoint = XV15RotorOperatingPoint()
    assumptions: str = ''
    hub_geometry: XV15RotorHubGeometry | None = None
    lateral_mast_tilt_source: str = ''
    planar_kinematics: bool = False
    left_hub_offset_body_m: tuple | None = None
    right_hub_offset_body_m: tuple | None = None


ILLUSTRATIVE_ROTOR_FLOW = 'ILLUSTRATIVE / UNVALIDATED'
ILLUSTRATIVE_CALCULATION = 'CALCULATED / ' + ILLUSTRATIVE_ROTOR_FLOW
XV15_LATERAL_MAST_TILT_DEG = 1.0
XV15_MAST_TILT_SOURCE = 'NASA CR-166535 Appendix I p. I-2 (XV-15 PHIM)'
PLANAR_V_SOURCE = 'TMPE Phase 2 planar model omits cross-plane flow and lateral motion'
ZERO_RATES_SOURCE = 'TMPE Phase 2 has no rotational dynamics'
HUB_XML_INSPECTED = (
    'geometry.xml: notebooklm_geometry_recovery/item[@name=hub_{left,right}_{fs,bl,wl}_in] '
    '(conversion tilt-axis datum), candidate_cg_{fs,bl,wl}_in (unverified), '
    'geometry/wing/span and pylon_spindle/butt_line_{left,right} (tilt-axis spacing); '
    'mass.xml: cg[@name=nominal_fuselage_station]; '
    'inertia.xml: moments_of_inertia/centre_of_gravity/@station_in (CG station only)'
)
HUB_GEOMETRY_NEEDED = (
    'Signed left and right hub x/y/z in forward/right/down body axes from the aircraft CG '
    'at the evaluated mast angles, or matched CG and shaft-pivot FS/BL/WL plus verified '
    'pivot-to-hub mast geometry'
)
WIND_MAST_AZIMUTH_SOURCE = 'NASA CR-166536 pp. A-17–A-20, atan2(V_HM, U_HM) in the local rotor frame'
WIND_MAST_FORCE_SOURCE = 'NASA CR-166536 pp. A-30–A-31, Section 9'
MAST_FORCE_ROTATION_SOURCE = 'NASA CR-166536 p. A-31, Section 10'
H_BAR_BLOCKER = ('Legible NASA CR-166536 p. A-30 scan required to verify the coefficient of '
                 'the H-bar term λ × (3/2 a-bar1 − p-hat)')
IN_PLANE_STATE_FIELDS = (
    ('lift_slope_per_rad', 'effective blade lift slope a', 'p. A-20'),
    ('cs0', 'blade-pitch constant CS0', 'p. A-21'),
    ('cs1', 'blade-pitch constant CS1', 'p. A-21'),
    ('cs2', 'blade-pitch constant CS2', 'p. A-21'),
    ('cc1', 'blade-pitch constant CC1', 'p. A-21'),
    ('cc2', 'blade-pitch constant CC2', 'p. A-21'),
    ('profile_drag_factor', 'effective profile drag factor Cdf', 'p. A-22'),
    ('induced_inflow', 'induced inflow ratio lambda_i', 'Fig. A1-3, pp. A-24–A-25'),
    ('inflow_distribution', 'inflow distribution factor K', 'p. A-29'),
    ('coning_rad', 'operating coning a0', 'p. A-22'),
    ('longitudinal_flap_rad', 'wind-mast longitudinal flapping a-bar1', 'pp. A-22–A-28'),
    ('lateral_flap_rad', 'wind-mast lateral flapping b-bar1', 'pp. A-22–A-28'),
    ('lateral_cyclic_mast_rad', 'mast-axis lateral cyclic A1', 'p. A-21'),
    ('longitudinal_cyclic_mast_rad', 'mast-axis longitudinal cyclic B1', 'p. A-21'),
)
HOVER_CT_SOURCE = 'NASA CR-177436 List of Symbols pp. xiv–xvi and §7.1, Figs. 7.1–7.3, pp. 50–54'
HOVER_CT_COMPARISON_SOURCE = 'NASA CR-177436 §7.1, p. 53, original metal-blade isolated hover tests'
HOVER_CT_ALTERNATING_LOAD_ONSET = .0161
HOVER_CT_SUGGESTED_MAXIMUM = .0165
HOVER_CT_COVERAGE = (
    'Isolated full-scale baseline metal rotor hover tests. Figure 7.3 uses blade pitch at 0.75R. '
    'Figure 7.3 Ames run averages 582.8–586.2 rpm and tip Mach 0.688–0.691. '
    'The separate WPAFB whirl-tower comparison includes a 740 ft/s tip-speed series.'
)
HOVER_CT_LIMITATION = (
    'Ames recorded collective is suitable for curve shape only, per p. 53. '
    'The WPAFB pitch comparison is uncorrected for tower blockage and differs by about 4°. '
    'TMPE lacks a verified pilot-command-to-0.75R pitch mapping and matched test setup, so has no supported pitch interval. '
    'Installed climb and transition are outside the isolated hover coverage.'
)
THRUST_CT_DEFINITION = 'NASA CR-177436 pp. xiv–xv: CT = T / [rho pi R² (unprimed shaft Omega R)²]'


def cr177436_hover_ct_comparison(ct):
    """Compare coefficients in the same convention without transferring the test limit."""
    if not isinstance(ct, (int, float)) or isinstance(ct, bool) or not math.isfinite(ct) or ct < 0:
        return 'UNAVAILABLE'
    if ct > HOVER_CT_SUGGESTED_MAXIMUM:
        return 'ABOVE SUGGESTED HOVER MAXIMUM'
    if ct > HOVER_CT_ALTERNATING_LOAD_ONSET:
        return 'ABOVE REPORTED ALTERNATING-LOAD ONSET'
    return 'AT OR BELOW REPORTED ALTERNATING-LOAD ONSET'


def transition_thrust_coefficient_audit(takeoff):
    """Identify the selected Phase 1 CT and the separate hover-data availability."""
    audit = dict(cr177436_hover_ct_status='UNAVAILABLE',
                 cr177436_hover_ct_coverage_status='OUTSIDE DOCUMENTED TEST CONDITIONS',
                 cr177436_hover_ct_source=HOVER_CT_SOURCE,
                 cr177436_hover_ct_data_kind='PUBLISHED PLOTS; no graph-digitized values used',
                 cr177436_hover_ct_coverage=HOVER_CT_COVERAGE,
                 cr177436_hover_ct_limitation=HOVER_CT_LIMITATION,
                 cr177436_ct_alternating_load_onset=HOVER_CT_ALTERNATING_LOAD_ONSET,
                 cr177436_ct_suggested_hover_maximum=HOVER_CT_SUGGESTED_MAXIMUM,
                 cr177436_ct_comparison_source=HOVER_CT_COMPARISON_SOURCE,
                 cr177436_ct_comparison_scope='Coefficient convention comparison only. Test limits do not establish installed takeoff or transition capability.',
                 thrust_coefficient_definition=THRUST_CT_DEFINITION,
                 selected_ct_application='Phase 1 model. Phase 2 holds its modeled axial thrust for the demonstration.')
    for side in ('left', 'right'):
        rotor = takeoff[side]
        bounds = rotor.get('pitch_bounds') or {}
        pitch = rotor.get('theta_deg')
        ct = rotor.get('thrust_coefficient')
        audit.update({f'{side}_{key}': value for key, value in (
            ('cr177436_hover_ct', None),
            ('cr177436_hover_thrust_n', None),
            ('collective_input_deg', pitch),
            ('collective_input_status', 'PROVIDED' if pitch is not None else 'UNAVAILABLE'),
            ('collective_input_source', 'TMPE pilot collective command; blade angle at 0.75R unverified'),
            ('thrust_coefficient', ct),
            ('cr177436_ct_comparison_status', cr177436_hover_ct_comparison(ct)),
            ('ct_status', rotor.get('ct_status', 'UNAVAILABLE')),
            ('ct_data_kind', rotor.get('ct_data_kind', 'UNAVAILABLE')),
            ('ct_source', rotor.get('source') if 'ct_status' in rotor else None),
            ('ct_reason', rotor.get('ct_reason', rotor['reason'])),
            ('ct_condition', rotor.get('condition')),
            ('rotor_configuration_source', rotor.get('configuration_source')),
            ('ct_pitch_min_deg', bounds.get('minimum_deg')),
            ('ct_pitch_max_deg', bounds.get('maximum_deg')),
            ('ct_pitch_bounds_basis', bounds.get('basis')),
            ('ct_pitch_bounds_source', bounds.get('source')),
        )})
    return audit


def transition_environment_audit(environment):
    fields = dict(air_density_kg_m3=environment.air_density, wind_speed_mps=environment.windspeed,
                  wind_from_deg=environment.winddirection, temperature_c=environment.temperature)
    for name, key, value in (
        ('air_density', 'air_density', environment.air_density),
        ('wind_speed', 'windspeed', environment.windspeed),
        ('wind_from', 'winddirection', environment.winddirection),
        ('temperature', 'temperature', environment.temperature),
    ):
        status = getattr(environment, 'input_statuses', {}).get(key, 'SCENARIO INPUT')
        fields[f'{name}_status'] = status if value is not None else 'UNAVAILABLE'
        fields[f'{name}_source'] = 'TMPE CLI prompt default' if status == 'DEFAULTED' else 'TMPE environment scenario input'
    return fields


def transition_rotor_inputs(state, aircraft, *, illustrative=False, takeoff=None):
    """Use the rotation and held axial forces already defined for Phase 2."""
    source = 'TMPE Phase 2 planar state'
    assumptions = ('V = 0 and cross-plane wind omitted. p = q = r = 0. '
                   'Operating RPM is held at the accepted XML value. '
                   'Per-rotor axial T is held at the Phase 1 value.')
    in_plane_force = None
    in_plane_source = 'TMPE Phase 1 rotor outputs supply axial thrust only'
    if illustrative:
        in_plane_force = 0.0
        in_plane_source = '--illustrative-rotor-flow'
        assumptions += ' H = Y = 0 for each rotor.'

    def operating_point(side, thrust_n):
        phase1 = takeoff[side] if takeoff is not None else {}
        thrust_source = phase1.get('source') or phase1.get('reason')
        provenance = phase1.get('source_status') or phase1.get('calibration_status') or ''
        t_status = (ILLUSTRATIVE_CALCULATION if 'ILLUSTRATIVE' in provenance or phase1.get('ct_status') == 'ASSUMED'
                    else 'CALCULATED')
        return XV15RotorOperatingPoint(
            aircraft.hover_rotor_rpm, thrust_n, in_plane_force, in_plane_force,
            rpm_source=f'rotor.xml: {HOVER_RPM_XML_PATH} (design hover rpm held through Phase 2)',
            rpm_status=HELD_HOVER_RPM_STATUS,
            t_source='Held Phase 1 axial thrust' + (f' ({thrust_source})' if thrust_source else ''),
            t_status=t_status,
            in_plane_force_source=in_plane_source, in_plane_force_status='ASSUMED',
        )

    return XV15RotorInputs(
        source,
        body_velocity_mps=(None, 0.0, None), body_rates_rad_s=(0.0, 0.0, 0.0),
        lateral_mast_tilt_deg=XV15_LATERAL_MAST_TILT_DEG,
        lateral_mast_tilt_source=XV15_MAST_TILT_SOURCE,
        left=operating_point('left', state.left_proprotor_thrust_n),
        right=operating_point('right', state.right_proprotor_thrust_n),
        assumptions=assumptions, planar_kinematics=True,
    )


def rotor_input_number(value, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        finite = math.isfinite(value)
    except OverflowError:
        return None
    if not finite:
        return None
    return None if positive and value <= 0 else value


def rotor_solidity(blades, chord_m, radius_m):
    """Geometric solidity, CR-166536 p. A-16."""
    blades, chord_m, radius_m = (rotor_input_number(v, positive=True) for v in (blades, chord_m, radius_m))
    if None not in (blades, chord_m, radius_m):
        return rotor_input_number(blades * chord_m / (math.pi * radius_m), positive=True)
    return None


def speed_of_sound_mps(temperature_c):
    """Calorically perfect dry air. Input degrees Celsius, output m/s."""
    temperature_c = rotor_input_number(temperature_c)
    if temperature_c is not None and temperature_c > -273.15:
        return rotor_input_number(math.sqrt(1.4 * 287.05 * (temperature_c + 273.15)), positive=True)
    return None


def calculate_xv15_blade_performance(state, *, mu, tip_mach, mast_angle_deg, density, radius, tip_speed,
                                    solidity, blade_count, thrust_n, omega_effective, precone_rad,
                                    precone_source):
    """Evaluate the A-20–A-22 operating values in SI from sourced inputs."""
    state = state if state is not None else XV15InPlaneState()
    source = state.source.strip() if isinstance(state.source, str) else ''
    audit = {}
    derived = {}
    density, radius, tip_speed, solidity, blade_count, omega_effective = (
        rotor_input_number(value, positive=True)
        for value in (density, radius, tip_speed, solidity, blade_count, omega_effective)
    )
    mu, tip_mach, mast_angle_deg, thrust_n = map(rotor_input_number, (mu, tip_mach, mast_angle_deg, thrust_n))
    mu = mu if mu is not None and mu >= 0 else None
    tip_mach = tip_mach if tip_mach is not None and tip_mach >= 0 else None
    mast_angle_deg = mast_angle_deg if mast_angle_deg is not None and 0 <= mast_angle_deg <= 90 else None

    def record(key, value, units, basis, reason='', status='CALCULATED'):
        key = f'in_plane_{key}'
        audit[key] = value
        audit[f'{key}_units'] = units
        audit[f'{key}_status'] = status if value is not None else 'UNAVAILABLE'
        audit[f'{key}_source'] = basis
        audit[f'{key}_reason'] = reason if value is None else ''
        return value

    def supplied(key, value, units, page, *, positive=False, nonnegative=False):
        number = rotor_input_number(value, positive=positive)
        if nonnegative and number is not None and number < 0:
            number = None
        basis = f'NASA CR-166536 {page}' + (f' | {source}' if source else '')
        reason = f'Requires a finite sourced {key}' if number is None else f'Requires provenance for {key}'
        return record(key, number if source else None, units, basis, reason,
                      'PROVIDED / ASSUMPTIONS' if state.assumptions else 'PROVIDED')

    def calculate(key, units, equation, inputs, formula, *, positive=False, nonnegative=False,
                  invalid='Formula result is nonfinite or outside its domain'):
        missing = ', '.join(label for label, value in inputs if value is None)
        value = None
        if not missing:
            try:
                value = rotor_input_number(formula(*(value for _, value in inputs)), positive=positive)
            except ArithmeticError:
                pass
        if nonnegative and value is not None and value < 0:
            value = None
        return record(key, value, units, f'NASA CR-166536 {equation}',
                      f'Missing or invalid: {missing}' if missing else invalid)

    def select(field, calculated, *, positive=False, nonnegative=False):
        provided = rotor_input_number(getattr(state, field), positive=positive) if source else None
        if nonnegative and provided is not None and provided < 0:
            provided = None
        if provided is not None:
            return provided
        if calculated is not None:
            derived[field] = calculated
        return calculated

    # A-15 and A-20: DN' = pi R^4, DN = rho Omega-prime^2 DN'.
    dn = calculate('dn_n', 'N', 'pp. A-15, A-20, DN = rho pi R^2 (Omega prime R)^2',
                   (('air density rho', density), ('rotor radius R', radius), ('effective tip speed', tip_speed)),
                   lambda rho, r, speed: rho * math.pi * r * r * speed * speed, positive=True)
    lift_coefficients = tuple(
        (f'sourced blade lift a{n}', supplied(f'blade_lift_a{n}', getattr(state, f'blade_lift_a{n}'),
                                             '1/rad', 'pp. A-9, A-20'))
        for n in range(3)
    )

    def effective_lift_slope(a0, a1, a2, advance_ratio, mach, beta_deg):
        # A-20 places sin(beta_m) outside the squared Mach term.
        mach_at_75_percent = .75 * mach
        radicand = 1 - mach_at_75_percent * mach_at_75_percent * math.sin(math.radians(beta_deg))
        if radicand > 0:
            return (a0 + advance_ratio * (a1 - a2 * advance_ratio)) / math.sqrt(radicand)
        return None

    calculated_lift = calculate('derived_lift_slope_per_rad', '1/rad', 'p. A-20, effective lift slope a_R',
                                (*lift_coefficients, ('advance ratio mu', mu), ('tip Mach', tip_mach),
                                 ('mast angle beta_m', mast_angle_deg)), effective_lift_slope, positive=True,
                                invalid='Compressibility radicand and effective lift slope must be finite and positive')
    lift_slope = select('lift_slope_per_rad', calculated_lift, positive=True)
    # A-21: Q6 = sigma a_R DN / 2. Fig. A1-3 uses T / DN for its C_T.
    calculate('q6_n', 'N', 'p. A-21, Q6 = sigma a_R DN / 2',
              (('solidity sigma', solidity), ('effective lift slope a_R', lift_slope), ('DN', dn)),
              lambda sigma, slope, force: .5 * sigma * slope * force, positive=True)
    ct = calculate('cr166536_ct', '1', 'Fig. A1-3, p. A-24, C_T = T / DN',
                   (('axial thrust T', thrust_n), ('DN', dn)), lambda thrust, force: thrust / force)
    alpha_r = calculate('blade_alpha_r_rad', 'rad', 'p. A-21, alpha_r = 7 C_T / (sigma a_R)',
                        (('CR-166536 C_T', ct), ('solidity sigma', solidity), ('effective lift slope a_R', lift_slope)),
                        lambda coefficient, sigma, slope: 7 * coefficient / (sigma * slope))

    theta = supplied('root_collective_rad', state.root_collective_rad, 'rad', 'pp. A-7, A-21')
    for n in range(4):
        terms = getattr(state, f'twist_terms_n{n}')
        if not isinstance(terms, (tuple, list)) or len(terms) != 4:
            terms = (None,) * 4
        twist_inputs = tuple((f'TW{index}_{n}', supplied(f'tw{index}_{n}', value, '1', 'pp. A-14–A-15'))
                             for index, value in enumerate(terms, 1))
        pitch_inputs = (*twist_inputs, ('root collective theta_0', theta))
        # A-21: compute CSn and CCn from the four twist-integral terms and root pitch.
        cs = calculate(f'derived_cs{n}', '1', 'p. A-21, blade-pitch constant CSn', pitch_inputs,
                       lambda tw1, tw2, tw3, tw4, collective: ((tw1 - tw3) * math.sin(collective)
                                                              + (tw2 + tw4) * math.cos(collective)))
        if n < 3:
            select(f'cs{n}', cs)
        cc = calculate(f'derived_cc{n}', '1', 'p. A-21, blade-pitch constant CCn', pitch_inputs,
                       lambda tw1, tw2, tw3, tw4, collective: (-(tw2 + tw4) * math.sin(collective)
                                                              + (tw1 - tw3) * math.cos(collective)))
        if n in (1, 2):
            select(f'cc{n}', cc)

    drag_coefficients = tuple(
        (label, supplied(field, getattr(state, field), units, 'pp. A-9, A-22', nonnegative=field == 'cdmax'))
        for field, label, units in (
            ('blade_drag_delta0', 'blade drag delta_0', '1'),
            ('blade_drag_delta1_per_rad', 'blade drag delta_1', '1/rad'),
            ('blade_drag_delta2_per_rad2', 'blade drag delta_2', '1/rad^2'),
            ('cdmax', 'CDMAX', '1'), ('cdalph', 'CDALPH', '1'), ('cdlim', 'CDLIM', '1'),
            ('cdmach', 'CDMACH', '1'), ('cdfact', 'CDFACT', '1'),
        )
    )

    def blade_drag(delta0, delta1, delta2, cdmax, cdalph, cdlim, cdmach, cdfact, alpha, mach):
        # A-22: the Mach contribution uses the printed plus signs and CDMAX caps Cd.
        correction = cdalph * alpha + cdfact * (cdlim + max(mach, cdmach))
        correction = rotor_input_number(correction)
        if correction is not None:
            raw_drag = rotor_input_number(delta0 + alpha * (delta1 + alpha * delta2) + max(0, correction))
            if raw_drag is not None:
                return min(cdmax, raw_drag)
        return None

    cd = calculate('blade_cd', '1', 'p. A-22, blade drag Cd',
                   (*drag_coefficients, ('blade alpha_r', alpha_r), ('tip Mach', tip_mach)),
                   blade_drag, nonnegative=True)
    cdf = calculate('derived_cdf', '1', 'p. A-22, Cdf = Cd / (n_b a_R)',
                    (('blade drag Cd', cd), ('blade count n_b', blade_count), ('effective lift slope a_R', lift_slope)),
                    lambda drag, blades, slope: drag / (blades * slope), nonnegative=True)
    select('profile_drag_factor', cdf, nonnegative=True)

    inertia = supplied('blade_flapping_inertia_kg_m2', state.blade_flapping_inertia_kg_m2, 'kg m^2',
                       'pp. A-8, A-22', positive=True)
    spring = supplied('coning_hub_spring_nm_per_rad', state.coning_hub_spring_nm_per_rad, 'N m/rad',
                      'pp. A-8, A-22', nonnegative=True)
    explicit_precone = rotor_input_number(state.precone_rad) if source else None
    if explicit_precone is not None:
        precone = supplied('precone_rad', explicit_precone, 'rad', 'pp. A-9, A-22')
    else:
        precone = rotor_input_number(precone_rad) if precone_source else None
        record('precone_rad', precone, 'rad', precone_source or 'NASA CR-166536 pp. A-9, A-22',
               'Requires a sourced precone angle', 'XML-SOURCED')
    record('tw34_rad', math.radians(34.525), 'rad',
           'NASA CR-166536 p. A-22, XV-15 TW34 34.525 deg converted to rad', status='NASA-SOURCED')
    # A-22: coning uses effective (primed) rotor speed and the coning spring K_HUB.
    coning = calculate('derived_coning_rad', 'rad', 'p. A-22, operating coning a_0',
                       (('rotor radius R', radius), ('axial thrust T', thrust_n), ('blade count n_b', blade_count),
                        ('blade flapping inertia I_b', inertia), ('coning hub spring K_HUB', spring),
                        ('precone angle', precone), ('effective angular speed Omega prime', omega_effective)),
                       lambda r, thrust, blades, ib, khub, pre, omega: (
                           (.75 * r * (thrust / blades) + khub * pre) / (ib * omega * omega + khub)))
    select('coning_rad', coning)
    return derived, audit


def calculate_xv15_in_plane_forces(state, *, xi, mu, lambda_zero, p_hat, q_hat,
                                  tip_speed, radius, density, solidity, delta3_tan,
                                  derived=None, derivation_audit=None, force_status='CALCULATED'):
    """Evaluate CR-166536 Section 9 from sourced operating inputs, then rotate by Section 10."""
    state = state if state is not None else XV15InPlaneState()
    source = state.source.strip() if isinstance(state.source, str) else ''
    audit = dict(derivation_audit or {})
    derived = derived or {}
    xi = rotor_input_number(xi)

    def record(key, value, equation, *, status='CALCULATED', input_source='', units='1', reason=''):
        audit[key] = value
        audit[f'{key}_status'] = status if value is not None else 'UNAVAILABLE'
        audit[f'{key}_source'] = equation + (f' | {input_source}' if input_source else '')
        audit[f'{key}_units'] = units
        audit[f'{key}_reason'] = reason if value is None else ''

    values = {}
    missing = []
    for key, label, page in IN_PLANE_STATE_FIELDS:
        value = rotor_input_number(getattr(state, key), positive=(key == 'lift_slope_per_rad'))
        if key == 'profile_drag_factor' and value is not None and value < 0:
            value = None
        from_state = source and value is not None
        values[key] = value if from_state else derived.get(key)
        reason = label if value is None else f'source for {label}'
        record(f'in_plane_{key}', values[key], f'NASA CR-166536 {page}', input_source=source,
               status=('PROVIDED / ASSUMPTIONS' if state.assumptions else 'PROVIDED') if from_state else force_status,
               units='rad' if key.endswith('_rad') else '1/rad' if key.endswith('_per_rad') else '1',
               reason=f'Missing or invalid: {reason}')
        if values[key] is None:
            missing.append(reason)

    q6 = audit.get('in_plane_q6_n')
    lift_slope = values['lift_slope_per_rad']
    if 'in_plane_q6_n' not in audit and None not in (density, radius, tip_speed, solidity, lift_slope):
        # A-20/21: DN = rho pi R² (Omega-prime R)² and Q6 = sigma a DN / 2.
        q6 = rotor_input_number(.5 * solidity * lift_slope * density * math.pi * radius**2 * tip_speed**2,
                                positive=True)
    induced = values['induced_inflow']
    inflow = rotor_input_number(lambda_zero + induced) if None not in (lambda_zero, induced) else None
    induced_velocity = (rotor_input_number(induced * tip_speed)
                        if None not in (induced, tip_speed) else None)
    cyclic_a = cyclic_b = None
    if None not in (xi, values['lateral_cyclic_mast_rad'], values['longitudinal_cyclic_mast_rad']):
        a = values['lateral_cyclic_mast_rad']
        b = values['longitudinal_cyclic_mast_rad']
        cyclic_a = rotor_input_number(a * math.cos(xi) - b * math.sin(xi))
        cyclic_b = rotor_input_number(a * math.sin(xi) + b * math.cos(xi))
    for key, value, page, units, reason in (
        ('in_plane_q6_n', q6, 'pp. A-20–A-21', 'N', 'Requires density, radius, effective tip speed, solidity and lift slope'),
        ('in_plane_total_inflow', inflow, 'Fig. A1-3, p. A-24, lambda = lambda_0 + lambda_i', '1',
         'Requires axial and induced inflow ratios'),
        ('in_plane_induced_velocity_mps', induced_velocity,
         'pp. A-20, A-24, induced ratio × effective tip speed', 'm/s',
         'Requires induced inflow ratio and effective tip speed'),
        ('in_plane_lateral_cyclic_wind_rad', cyclic_a, 'p. A-21', 'rad', 'Requires A1, B1 and wind azimuth'),
        ('in_plane_longitudinal_cyclic_wind_rad', cyclic_b, 'p. A-21', 'rad', 'Requires A1, B1 and wind azimuth'),
    ):
        if key not in audit:
            record(key, value, f'NASA CR-166536 {page}', units=units, reason=reason,
                   status=force_status if key in ('in_plane_total_inflow', 'in_plane_induced_velocity_mps')
                   else 'CALCULATED')
    missing.extend(label for value, label in (
        (xi, 'wind azimuth xi from a nonzero mast-plane hub velocity'),
        (mu, 'mast-plane advance ratio mu'),
        (lambda_zero, 'axial hub inflow ratio lambda_0 from W_HM and Omega prime R'),
        (inflow, 'total inflow lambda = lambda_0 + lambda_i'),
        (p_hat, 'normalized wind-mast rate p-hat'),
        (q_hat, 'normalized wind-mast rate q-hat'),
        (tip_speed, 'positive effective tip speed Omega prime R'),
        (radius, 'positive rotor radius R'),
        (density, 'positive air density rho'),
        (solidity, 'geometric solidity sigma from blade count, chord and radius'),
        (q6, 'force scale Q6'),
        (delta3_tan, 'signed pitch-flap coupling tan(delta3)'),
        (cyclic_a, 'wind-mast lateral cyclic A-bar1'),
        (cyclic_b, 'wind-mast longitudinal cyclic B-bar1'),
    ) if value is None)
    # A-30 Section 9: the lambda-term subscript cannot be established from indexed text.
    # A supplied candidate is visible in the audit and cannot enter the formula.
    h_bar = y_bar = None
    candidate = rotor_input_number(state.h_lambda_coefficient) if source else None
    record('in_plane_h_lambda_coefficient', None, 'NASA CR-166536 p. A-30, Section 9', reason=H_BAR_BLOCKER)
    record('in_plane_h_lambda_coefficient_candidate', candidate, 'NASA CR-166536 p. A-30, Section 9',
           status='UNVERIFIED / NOT USED', input_source=source, reason=H_BAR_BLOCKER)
    audit['in_plane_h_lambda_coefficient_candidate_reason'] = H_BAR_BLOCKER
    missing.append(H_BAR_BLOCKER)
    cs0, cs1, cs2, cc1, cc2 = (values[key] for key in ('cs0', 'cs1', 'cs2', 'cc1', 'cc2'))
    coning = values['coning_rad']
    flap_a = values['longitudinal_flap_rad']
    flap_b = values['lateral_flap_rad']
    distribution = values['inflow_distribution']
    drag = values['profile_drag_factor']
    y_bar_dependencies = (
        (q6, 'Q6'), (mu, 'μ'), (inflow, 'λ'), (induced, 'λ_i'), (p_hat, 'p-hat'), (q_hat, 'q-hat'),
        (delta3_tan, 'tan(δ3)'), (cyclic_a, 'A-bar1'), (cyclic_b, 'B-bar1'),
        (cs0, 'CS0'), (cs1, 'CS1'), (cs2, 'CS2'), (cc1, 'CC1'), (cc2, 'CC2'),
        (coning, 'operating coning'), (flap_a, 'a-bar1'), (flap_b, 'b-bar1'),
        (distribution, 'inflow distribution K'), (drag, 'Cdf'),
    )
    y_bar_missing = [label for value, label in y_bar_dependencies if value is None]
    if not y_bar_missing:
        induced_skew = distribution * induced
        cyclic_inflow = (cc1 * inflow + cs0 * inflow * inflow) / 2
        cyclic_coning = (cc2 + cs1 * inflow) * coning / 2
        profile_drag = drag * math.hypot(1, 4 * inflow / 3)
        # A-31 Section 9, continued. Values already use each rotor's local signs.
        y_bar = rotor_input_number(q6 * (
            cs0 * mu * mu * flap_b / 2 + cs2 * (flap_b - 2 * induced_skew / 3 + q_hat / 2)
            - cc1 * inflow * (1.5 * (flap_b - 8 * induced_skew / 9) + q_hat)
            + cc2 * coning / 2 * (flap_a - p_hat) - 1.5 * cs1 * mu * coning
            + cyclic_b * cyclic_coning + cyclic_a * cyclic_inflow
            - delta3_tan * (cc2 * coning * flap_b / 2 + cc1 * inflow * flap_a / 2
                            + 8 * cs2 * inflow * inflow * flap_a / 9)
            + profile_drag * coning * p_hat))
        if y_bar is None:
            missing.append('finite wind-mast Y-bar')
    # A-31 Section 10 uses the same rotation in both local rotor frames.
    # Represent exact cardinal azimuths without a floating-point sine/cosine residue.
    cos_xi = sin_xi = None
    if xi is not None:
        cos_xi = 0.0 if abs(xi) == math.pi / 2 else math.cos(xi)
        sin_xi = 0.0 if xi == 0 or abs(xi) == math.pi else math.sin(xi)

    def rotate(h_weight, y_weight):
        if xi is None:
            return None, 'Requires a defined wind azimuth ξ'
        terms = ((h_bar, h_weight, 'H-bar'), (y_bar, y_weight, 'Y-bar'))
        required = [label for value, weight, label in terms if weight != 0 and value is None]
        if required:
            return None, 'Requires ' + ' and '.join(required)
        value = rotor_input_number(sum(value * weight for value, weight, _ in terms if weight != 0))
        return value, 'Requires a finite rotated force' if value is None else ''

    h, h_reason = rotate(cos_xi, sin_xi)
    y, y_reason = rotate(-sin_xi if sin_xi is not None else None, cos_xi)
    y_bar_reason = 'Missing or invalid: ' + ', '.join(y_bar_missing) if y_bar_missing else 'Requires finite Y-bar'
    record('rotor_h_bar_n', h_bar, 'NASA CR-166536 p. A-30, Section 9', units='N', reason=H_BAR_BLOCKER)
    record('rotor_y_bar_n', y_bar, 'NASA CR-166536 p. A-31, Section 9', units='N', status=force_status,
           reason=y_bar_reason)
    record('nasa_rotor_h_n', h, MAST_FORCE_ROTATION_SOURCE, units='N', status=force_status, reason=h_reason)
    record('nasa_rotor_y_n', y, MAST_FORCE_ROTATION_SOURCE, units='N', status=force_status, reason=y_reason)
    audit['nasa_in_plane_missing_inputs'] = '; '.join(missing)
    return h, y, audit


def audit_rotor_value(audit, key, value, units, source, *, status='CALCULATED', reason=''):
    audit.update({key: value, f'{key}_units': units, f'{key}_source': source,
                  f'{key}_status': status if value is not None else 'UNAVAILABLE',
                  f'{key}_reason': reason})


def illustrative_rotor_status(status):
    return isinstance(status, str) and ('ASSUMED' in status or 'ILLUSTRATIVE' in status)


def calculate_xv15_induced_inflow(*, thrust_n, dn_n, mu, lambda_zero, tip_loss_factor,
                                 ground_effect_factor, washout, side_by_side_factor, tandem_factor):
    """Fig. A1-3, p. A-24 inner loop with supplied axial thrust held fixed."""
    inputs = ((thrust_n, 'T'), (dn_n, 'DN'), (mu, 'μ'), (lambda_zero, 'λ₀'),
              (tip_loss_factor, 'B'), (ground_effect_factor, 'G'), (washout, 'WASHOUT'),
              (side_by_side_factor, 'X_SS'), (tandem_factor, 'X_SF'))
    missing = [label for value, label in inputs if rotor_input_number(value) is None]
    if missing:
        return None, None, 'UNAVAILABLE', 'Missing or invalid: ' + ', '.join(missing)
    if thrust_n < 0 or dn_n <= 0 or mu < 0 or tip_loss_factor <= 0:
        return None, None, 'UNAVAILABLE', 'Requires nonnegative T and μ, positive DN and positive B'
    scale = rotor_input_number(2 * tip_loss_factor * tip_loss_factor * dn_n, positive=True)
    q3 = rotor_input_number(thrust_n / scale) if scale is not None else None
    q5_denominator = rotor_input_number(q3 + 8 * mu * mu, positive=True) if q3 is not None else None
    if q5_denominator is None:
        return None, None, 'UNAVAILABLE', 'Fig. A1-3 initialization has no finite Q3 and positive Q5 denominator'
    root_q3 = math.sqrt(q3)
    q5 = .6 * q3 * (root_q3 / q5_denominator)
    total = lambda_zero + q3 * root_q3  # Fig. A1-3 initialization box for the inner loop.
    numerator = q3 * (1 - (1 - ground_effect_factor) * washout) * (1 + side_by_side_factor + tandem_factor)

    # The printed flowchart tests convergence while I <= 20 and exits after I exceeds 20.
    for count in range(1, 22):
        total_squared = total * total
        correction_denominator = rotor_input_number(q3 + 8 * total_squared, positive=True)
        if correction_denominator is None:
            return None, count, 'UNAVAILABLE', 'Fig. A1-3 has a singular or nonfinite wake-correction denominator'
        correction = q5 * ((q3 + (8 / 3) * total * abs(total)) / correction_denominator)
        denominator = rotor_input_number(math.hypot(mu, math.sqrt(.866) * total) + correction, positive=True)
        if denominator is None:
            return None, count, 'UNAVAILABLE', 'Fig. A1-3 requires a finite positive inflow denominator'
        q1 = lambda_zero + numerator / denominator
        induced = rotor_input_number(q1 - lambda_zero)
        total = .7 * q1 + .3 * total
        if induced is None or not math.isfinite(total):
            return None, count, 'UNAVAILABLE', 'Fig. A1-3 produced nonfinite inflow'
        if count <= 20 and abs(q1 - total) <= .001 * abs(induced) + .0001:
            return induced, count, 'CONVERGED', ''
    return None, count, 'NOT CONVERGED', 'Fig. A1-3 iteration limit exceeded'


def resolve_xv15_induced_inflow(state, derived, audit, *, thrust, mu, lambda_zero, force_status):
    equation = 'NASA CR-166536 Fig. A1-3, p. A-24, induced inflow for supplied thrust'
    source = state.source.strip() if isinstance(state.source, str) else ''
    factor_status = 'PROVIDED / ASSUMPTIONS' if state.assumptions else 'PROVIDED'
    factors = {}
    for key, symbol in (('tip_loss_factor', 'B'), ('ground_effect_factor', 'G'), ('washout', 'WASHOUT'),
                        ('side_by_side_factor', 'X_SS'), ('tandem_factor', 'X_SF')):
        value = rotor_input_number(getattr(state, key), positive=key == 'tip_loss_factor') if source else None
        factors[key] = value
        audit_rotor_value(audit, f'in_plane_{key}', value, '1', f'{equation} | {source}' if source else equation,
                          status=factor_status, reason='' if value is not None else f'Sourced {symbol} required')
    supplied = rotor_input_number(state.induced_inflow) if source else None
    if supplied is not None:
        induced, iterations, status, reason = None, None, 'PROVIDED / NOT ITERATED', 'Sourced operating λ_i supplied'
    else:
        induced, iterations, status, reason = calculate_xv15_induced_inflow(
            thrust_n=thrust, dn_n=audit['in_plane_dn_n'], mu=mu, lambda_zero=lambda_zero, **factors)
    if induced is not None:
        derived['induced_inflow'] = induced
    audit_rotor_value(audit, 'in_plane_induced_iteration_count', iterations, '1', equation,
                      status=force_status, reason=reason)
    audit['in_plane_induced_convergence_status'] = status
    audit['in_plane_induced_convergence_source'] = equation
    audit['in_plane_induced_convergence_reason'] = reason


def resolve_xv15_flapping(state, derived, audit, *, force_status):
    equation = 'NASA CR-166536 pp. A-22, A-26–A-28, C x_dot + A x = B in wind-mast axes'
    source = state.source.strip() if isinstance(state.source, str) else ''
    supplied_status = 'PROVIDED / ASSUMPTIONS' if state.assumptions else 'PROVIDED'
    missing = []

    def supplied(key, value, units, *, positive=False):
        value = rotor_input_number(value, positive=positive) if source else None
        if value is None:
            missing.append(key)
        audit_rotor_value(audit, f'in_plane_{key}', value, units, f'{equation} | {source}' if source else equation,
                          status=supplied_status, reason='' if value is not None else f'Sourced {key} required')
        return value

    def terms(raw, keys, units):
        if not isinstance(raw, (tuple, list)) or len(raw) != len(keys):
            raw = (None,) * len(keys)
        return tuple(supplied(key, value, units) for key, value in zip(keys, raw))

    c11, c12, c21, c22 = terms(state.flapping_c_matrix, ('C11', 'C12', 'C21', 'C22'), 'N m s/rad')
    a11, a12, a21, a22 = terms(state.flapping_a_matrix, ('A11', 'A12', 'A21', 'A22'), 'N m/rad')
    b1, b2 = terms(state.flapping_forcing_nm, ('B1_with_controls', 'B2_with_controls'), 'N m')
    prior_a, prior_b = terms(state.flapping_prior_rad, ('prior_a_bar1', 'prior_b_bar1'), 'rad')
    step = supplied('flapping_step_s', state.flapping_step_s, 's', positive=True)
    current_a = rotor_input_number(state.longitudinal_flap_rad) if source else None
    current_b = rotor_input_number(state.lateral_flap_rad) if source else None
    result_a = result_b = None
    reason = 'Missing or invalid: ' + ', '.join(missing) if missing else ''
    if current_a is not None or current_b is not None:
        reason = ('Sourced current flapping states supplied' if None not in (current_a, current_b) else
                  'Supply both current flapping states, or omit them and provide a complete matrix step')
    elif not missing:
        # Backward Euler with the supplied, fixed step matrices. A-28 requires nonsingular C.
        matrix_det = rotor_input_number(c11 * c22 - c12 * c21)
        m11, m12 = c11 + step * a11, c12 + step * a12
        m21, m22 = c21 + step * a21, c22 + step * a22
        step_det = rotor_input_number(m11 * m22 - m12 * m21)
        if matrix_det and step_det:
            rhs1 = c11 * prior_a + c12 * prior_b + step * b1
            rhs2 = c21 * prior_a + c22 * prior_b + step * b2
            result_a = rotor_input_number((rhs1 * m22 - m12 * rhs2) / step_det)
            result_b = rotor_input_number((m11 * rhs2 - rhs1 * m21) / step_det)
        if None in (result_a, result_b):
            result_a = result_b = None
            reason = 'Finite, nonsingular C and C + dt A and finite updated states required'
        else:
            derived.update(longitudinal_flap_rad=result_a, lateral_flap_rad=result_b)
    for key, value in (('a_bar1', result_a), ('b_bar1', result_b)):
        audit_rotor_value(audit, f'in_plane_integrated_{key}_rad', value, 'rad', equation + ' | backward Euler',
                          status=force_status, reason=reason)


def calculate_xv15_rotor_flow(inputs, aircraft, beta_deg, pitch_deg, forward_mps, up_mps, air_density,
                             *, rotor_mast_angles_deg=None, mast_angle_source=None, temperature_c=None):
    """Apply CR-166536 hub, wind-mast and force-coefficient equations per rotor."""
    def project_load(terms, missing):
        active_terms = [term for term in terms if term[0] != 0]
        missing = list(missing)
        for _, value, label, value_source, _ in active_terms:
            if value is None:
                missing.append(label)
            elif not value_source:
                missing.append(f'source for {label}')
        if missing:
            return None, 'UNAVAILABLE', 'Missing or invalid: ' + ', '.join(missing)
        value = rotor_input_number(sum(weight * force for weight, force, *_ in active_terms))
        if value is None:
            return None, 'UNAVAILABLE', 'Requires a finite result'
        status = ILLUSTRATIVE_CALCULATION if any(
            illustrative_rotor_status(term[4]) for term in active_terms) else 'CALCULATED'
        return value, status, ''

    source = inputs.source.strip() if isinstance(inputs.source, str) else ''
    calculated_status = 'CALCULATED'
    u, v, w = map(rotor_input_number, inputs.body_velocity_mps)
    reported_u, reported_w = u, w
    if (inputs.planar_kinematics or inputs.assumptions) and None not in (pitch_deg, forward_mps, up_mps):
        # The horizontal component is already projected onto aircraft heading.
        # This is the pitch rotation of (forward, 0, -up) into body axes.
        pitch = math.radians(pitch_deg)
        if u is None:
            u = forward_mps * math.cos(pitch) + up_mps * math.sin(pitch)
        if w is None:
            w = forward_mps * math.sin(pitch) - up_mps * math.cos(pitch)
    body_known = None not in (u, v, w)
    body_status = 'UNAVAILABLE' if (u, v, w) == (None, None, None) else 'PARTIAL'
    if body_known:
        body_status = 'CALCULATED / ASSUMED' if inputs.planar_kinematics else 'PROVIDED'
    cg_airspeed = rotor_input_number(math.hypot(u, v, w)) if body_known else None
    if cg_airspeed is None and inputs.planar_kinematics and None not in (forward_mps, up_mps):
        cg_airspeed = rotor_input_number(math.hypot(forward_mps, up_mps))
    sound_speed = speed_of_sound_mps(temperature_c)
    p, q, r = map(rotor_input_number, inputs.body_rates_rad_s)
    rates_known = None not in (p, q, r)
    rates_status = 'ASSUMED' if inputs.planar_kinematics else 'PROVIDED'
    rates_source = ZERO_RATES_SOURCE if inputs.planar_kinematics else source
    beta_deg = rotor_input_number(beta_deg)
    if beta_deg is not None and not 0 <= beta_deg <= 90:
        beta_deg = None
    if rotor_mast_angles_deg is None:
        rotor_mast_angles_deg = (beta_deg, beta_deg)
    phi_deg = rotor_input_number(inputs.lateral_mast_tilt_deg)
    if phi_deg is not None:
        phi = math.radians(phi_deg)
        sin_phi, cos_phi = math.sin(phi), math.cos(phi)
    lever_terms = (None, None, None)
    geometry_status = 'PROVIDED'
    if rotor_mast_angles_deg[0] == rotor_mast_angles_deg[1]:
        lever_terms = tuple(map(rotor_input_number, inputs.hub_offsets_m))
        if None not in (beta_deg, phi_deg) and inputs.hub_geometry is not None:
            lever_terms = inputs.hub_geometry.lever_terms_m(math.radians(beta_deg), phi)
            geometry_status = calculated_status
    diameter = rotor_input_number(aircraft.rotor_diameter_ft, positive=True)
    radius = diameter * 0.3048 / 2 if diameter is not None else None
    blades = rotor_input_number(getattr(aircraft, 'rotor_blades_per_hub', None), positive=True)
    chord_ft = rotor_input_number(getattr(aircraft, 'blade_chord_ft', None), positive=True)
    chord_m = chord_ft * 0.3048 if chord_ft is not None else None
    solidity = rotor_solidity(blades, chord_m, radius)
    delta3_deg = rotor_input_number(getattr(aircraft, 'rotor_delta3_deg', None))
    delta3_tan = math.tan(math.radians(delta3_deg)) if delta3_deg is not None else None
    precone_deg = rotor_input_number(getattr(aircraft, 'rotor_precone_deg', None))
    precone_rad = math.radians(precone_deg) if precone_deg is not None else None
    precone_source = f'rotor.xml: {ROTOR_PRECONE_XML_PATH} (deg × π/180)'
    density = rotor_input_number(air_density, positive=True)
    zero_body_rates = (p, q, r) == (0, 0, 0)
    lxh, lyh, lzh = lever_terms
    audit = dict(body_u_mps=u, body_v_mps=v, body_w_mps=w, body_velocity_status=body_status,
                 body_p_rad_s=p, body_q_rad_s=q, body_r_rad_s=r,
                 body_rates_status=rates_status if rates_known else 'UNAVAILABLE',
                 lateral_mast_tilt_deg=phi_deg, lxh_m=lxh, lyh_m=lyh, lzh_m=lzh,
                 lateral_mast_tilt_status='PROVIDED' if phi_deg is not None else 'UNAVAILABLE',
                 lateral_mast_tilt_source=inputs.lateral_mast_tilt_source or source,
                 hub_geometry_xml_fields_inspected=HUB_XML_INSPECTED if inputs.planar_kinematics else '',
                 rotor_radius_m=radius, rotor_radius_status=calculated_status if radius is not None else 'UNAVAILABLE',
                 rotor_geometry_source=f'rotor.xml: {ROTOR_DIAMETER_XML_PATH} (ft × 0.3048 / 2)',
                 rotor_blade_count=blades, rotor_blade_count_status='XML-SOURCED' if blades is not None else 'UNAVAILABLE',
                 rotor_blade_count_source='rotor.xml: proprotor_and_drive_system/rotor_blades_per_hub',
                 rotor_blade_chord_m=chord_m, rotor_blade_chord_status='CALCULATED' if chord_m is not None else 'UNAVAILABLE',
                 rotor_blade_chord_source='rotor.xml: proprotor_and_drive_system/blade_chord (in × 0.0254)',
                 rotor_solidity=solidity, rotor_solidity_status='CALCULATED' if solidity is not None else 'UNAVAILABLE',
                 rotor_solidity_source='NASA CR-166536 p. A-16, blade count × chord / (pi × radius)',
                 rotor_delta3_deg=delta3_deg, rotor_delta3_status='XML-SOURCED' if delta3_deg is not None else 'UNAVAILABLE',
                 rotor_delta3_source=f'rotor.xml: {ROTOR_DELTA3_XML_PATH}',
                 rotor_delta3_tan=delta3_tan, rotor_delta3_tan_status='CALCULATED' if delta3_tan is not None else 'UNAVAILABLE',
                 rotor_delta3_tan_source='NASA CR-166536 p. A-16, tan(radians(delta3 from rotor.xml))',
                 rotor_cg_airspeed_mps=cg_airspeed,
                 rotor_cg_airspeed_status=calculated_status if cg_airspeed is not None else 'UNAVAILABLE',
                 rotor_cg_airspeed_source='NASA CR-166536 p. A-7, V_T = magnitude of CG air-relative velocity',
                 rotor_cg_airspeed_reason='' if cg_airspeed is not None else 'CG air-relative velocity required',
                 air_speed_of_sound_mps=sound_speed,
                 air_speed_of_sound_status=calculated_status if sound_speed is not None else 'UNAVAILABLE',
                 air_speed_of_sound_source=SOUND_SPEED_SOURCE + ' | environment °C + 273.15, dry-air γ = 1.4 and R = 287.05 J/(kg K)',
                 air_speed_of_sound_reason='' if sound_speed is not None else 'Air temperature above absolute zero required',
                 air_density_status='USER-PRESCRIBED' if density is not None else 'UNAVAILABLE',
                 air_density_source='TMPE environment air-density input held constant',
                 rotor_flow_source=source, rotor_flow_assumptions=inputs.assumptions)
    projection_source = 'Heading-aligned Phase 2 air-relative velocity and prescribed pitch'
    for name, value, status, value_source in (
        ('u', u, 'PROVIDED' if reported_u is not None else calculated_status,
         source if reported_u is not None else projection_source),
        ('v', v, 'ASSUMED' if inputs.planar_kinematics else 'PROVIDED',
         PLANAR_V_SOURCE if inputs.planar_kinematics else source),
        ('w', w, 'PROVIDED' if reported_w is not None else calculated_status,
         source if reported_w is not None else projection_source),
        ('p', p, rates_status, rates_source),
        ('q', q, rates_status, rates_source),
        ('r', r, rates_status, rates_source),
    ):
        audit[f'body_{name}_status'] = status if value is not None else 'UNAVAILABLE'
        audit[f'body_{name}_source'] = value_source
    missing = [label for value, label in (
        (u, 'body-axis U for mast-plane hub velocity and μ'),
        (v, 'body-axis V for mast-plane hub velocity and μ'),
        (w, 'body-axis W for mast-plane hub velocity and μ'),
        (p, 'body roll rate p for hub velocity and Ω′'),
        (q, 'body pitch rate q for hub velocity and Ω′'),
        (r, 'body yaw rate r for hub velocity and Ω′'),
        (phi_deg, 'lateral mast tilt phi_m for μ'),
        (radius, 'positive rotor radius R for μ and C_RF'),
        (density, 'positive air density rho for C_RF'),
    ) if value is None]
    if not source:
        missing.append('source for the rotor operating inputs')

    geometry_statuses = []
    selected_force_assumptions = [inputs.assumptions] if inputs.assumptions else []
    wind_mast_assumptions = (
        'Planar model assumes V = p = q = r = 0 and omits cross-plane wind.'
        if inputs.planar_kinematics else inputs.assumptions
    )
    for side, sign, operating_point, mast_angle_deg, signed_offset in (
        ('left', -1, inputs.left, rotor_mast_angles_deg[0], inputs.left_hub_offset_body_m),
        ('right', 1, inputs.right, rotor_mast_angles_deg[1], inputs.right_hub_offset_body_m),
    ):
        mast_angle_deg = rotor_input_number(mast_angle_deg)
        if mast_angle_deg is not None and not 0 <= mast_angle_deg <= 90:
            mast_angle_deg = None
        axes_known = None not in (mast_angle_deg, phi_deg)
        if mast_angle_deg is not None:
            beta = math.radians(mast_angle_deg)
            sin_beta = math.sin(beta)
            cos_beta = 0.0 if mast_angle_deg == 90 else math.cos(beta)
        offset_status = geometry_status
        if signed_offset is not None:
            x, y_offset, z = map(rotor_input_number, signed_offset)
            offset_status = 'PROVIDED'
        else:
            lx, ly, lz = lever_terms
            if beta_deg is None and axes_known and inputs.hub_geometry is not None:
                lx, ly, lz = inputs.hub_geometry.lever_terms_m(beta, phi)
                offset_status = calculated_status
            x = lx
            y_offset = None if ly is None else sign * ly
            z = None if lz is None else -lz
        offset_known = None not in (x, y_offset, z)
        if not offset_known:
            offset_status = 'UNAVAILABLE'
            if not zero_body_rates:
                missing.extend(f'{side} CG-to-hub body {axis} for rate correction'
                               for axis, value in zip(('x', 'y', 'z'), (x, y_offset, z)) if value is None)
        geometry_statuses.append(offset_status)
        rpm = rotor_input_number(operating_point.rpm, positive=True)
        t, h, y = map(rotor_input_number, (operating_point.t_n, operating_point.h_n, operating_point.y_n))
        missing.extend(f'{side} {label}' for value, label in (
            (mast_angle_deg, 'mast angle beta_m for mast-axis hub velocity and μ'),
            (rpm, 'operating rotor RPM for Ω, μ and C_RF'),
            (t, 'mast-axis T (axial thrust) for C_RF'),
        ) if value is None)
        omega = rpm * (math.tau / 60) if rpm is not None else None
        omega_effective = hub_u = hub_v = hub_w = advance_ratio = force_coefficient = None
        xi = mast_plane_speed = lambda_zero = p_hat = q_hat = tip_speed = None
        u_hb = v_hb = w_hb = None
        if zero_body_rates:
            omega_effective = omega
        elif axes_known and rates_known and omega is not None:
            # A-16 rotor angular velocity in space. The two sides counter-rotate.
            omega_effective = rotor_input_number(
                omega + sign * p * sin_beta * cos_phi
                + q * cos_beta * sin_phi - sign * r * cos_beta * cos_phi, positive=True)
            if omega_effective is None:
                missing.append(f'{side} positive effective rotor speed Omega prime')
        if None not in (omega_effective, radius):
            tip_speed = rotor_input_number(omega_effective * radius, positive=True)
        shaft_tip_speed = rotor_input_number(omega * radius, positive=True) if None not in (omega, radius) else None
        tip_mach_inputs = (
            (cg_airspeed, 'CG airspeed V_T'), (shaft_tip_speed, 'shaft speed Ω and radius'),
            (mast_angle_deg, 'mast angle β_m'), (sound_speed, 'speed of sound'),
        )
        tip_mach_missing = [label for value, label in tip_mach_inputs if value is None]
        tip_mach = None
        if not tip_mach_missing:
            # A-21 uses shaft Ω and CG speed. The norm is the law-of-cosines expression.
            tip_mach = rotor_input_number(math.hypot(cg_airspeed + shaft_tip_speed * cos_beta,
                                                     shaft_tip_speed * sin_beta) / sound_speed)
        body_hub_known = rates_known and body_known and (zero_body_rates or offset_known)
        if body_hub_known:
            u_hb, v_hb, w_hb = u, v, w
            if not zero_body_rates:
                # A-18 and A-19 use omega × (CG to hub) in forward/right/down axes.
                u_hb = u + q * z - r * y_offset
                v_hb = v + r * x - p * z
                w_hb = w + p * y_offset - q * x
        hub_known = body_hub_known and axes_known
        if hub_known:
            hub_u = u_hb * cos_beta - sign * v_hb * sin_beta * sin_phi + w_hb * sin_beta * cos_phi
            hub_v = sign * v_hb * cos_phi + w_hb * sin_phi
            hub_w = -u_hb * sin_beta - sign * v_hb * cos_beta * sin_phi + w_hb * cos_beta * cos_phi
            mast_plane_speed = rotor_input_number(math.hypot(hub_u, hub_v))
            if mast_plane_speed is not None and mast_plane_speed > 0:
                xi = math.atan2(hub_v, hub_u)
            if tip_speed is not None:
                advance_ratio = rotor_input_number(mast_plane_speed / tip_speed) if mast_plane_speed is not None else None
                lambda_zero = rotor_input_number(-hub_w / tip_speed)
                if advance_ratio is None:
                    missing.append(f'{side} finite mast-plane advance ratio')
        if None not in (xi, omega_effective) and axes_known and rates_known:
            # A-16/A-17: the left rotor signs are applied before the local azimuth rotation.
            p_mast = sign * p * cos_beta - q * sin_beta * sin_phi + sign * r * sin_beta * cos_phi
            q_mast = q * cos_phi + sign * r * sin_phi
            cos_xi, sin_xi = math.cos(xi), math.sin(xi)
            p_hat = rotor_input_number((p_mast * cos_xi + q_mast * sin_xi) / omega_effective)
            q_hat = rotor_input_number((-p_mast * sin_xi + q_mast * cos_xi) / omega_effective)
        state = operating_point.in_plane_state or XV15InPlaneState()
        assumed_thrust = illustrative_rotor_status(operating_point.t_status)
        calculated_force_status = ILLUSTRATIVE_CALCULATION if assumed_thrust or state.assumptions else 'CALCULATED'
        derived, derivation_audit = calculate_xv15_blade_performance(
            state, mu=advance_ratio, tip_mach=tip_mach, mast_angle_deg=mast_angle_deg, density=density,
            radius=radius, tip_speed=tip_speed, solidity=solidity, blade_count=blades, thrust_n=t,
            omega_effective=omega_effective, precone_rad=precone_rad, precone_source=precone_source)
        for key in ('in_plane_cr166536_ct', 'in_plane_blade_alpha_r_rad', 'in_plane_blade_cd',
                    'in_plane_derived_cdf', 'in_plane_derived_coning_rad'):
            if derivation_audit[key] is not None:
                derivation_audit[f'{key}_status'] = calculated_force_status
        if state.assumptions:
            for key in ('in_plane_derived_lift_slope_per_rad', 'in_plane_q6_n'):
                if derivation_audit[key] is not None:
                    derivation_audit[f'{key}_status'] = ILLUSTRATIVE_CALCULATION
        resolve_xv15_induced_inflow(state, derived, derivation_audit, thrust=t, mu=advance_ratio,
                                   lambda_zero=lambda_zero, force_status=calculated_force_status)
        resolve_xv15_flapping(state, derived, derivation_audit, force_status=calculated_force_status)
        h_calc, y_calc, force_audit = calculate_xv15_in_plane_forces(
            state, xi=xi, mu=advance_ratio, lambda_zero=lambda_zero, p_hat=p_hat, q_hat=q_hat,
            tip_speed=tip_speed, radius=radius, density=density, solidity=solidity, delta3_tan=delta3_tan,
            derived=derived, derivation_audit=derivation_audit, force_status=calculated_force_status)
        force_audit['in_plane_state_assumptions'] = state.assumptions
        audit.update({f'{side}_{name}': value for name, value in force_audit.items()})
        h_source = y_source = operating_point.in_plane_force_source or source
        h_status = y_status = operating_point.in_plane_force_status
        force_source = WIND_MAST_FORCE_SOURCE + ' and p. A-31, Section 10'
        if assumed_thrust:
            selected_force_assumptions.append('Axial thrust is illustrative or assumed.')
        used_nasa_force = False
        if h is None and h_calc is not None:
            h, h_source, h_status = h_calc, force_source, calculated_force_status
            used_nasa_force = True
        if y is None and y_calc is not None:
            y, y_source, y_status = y_calc, force_source, calculated_force_status
            used_nasa_force = True
        if used_nasa_force and state.assumptions:
            selected_force_assumptions.append(state.assumptions)
        if any(value is not None and illustrative_rotor_status(status) for value, status in ((h, h_status), (y, y_status))):
            selected_force_assumptions.append('Selected mast-axis H/Y include illustrative or assumed inputs.')
        missing.extend(f'{side} mast-axis {name} (in-plane {axis} force) for C_RF'
                       for value, name, axis in ((h, 'H', 'longitudinal'), (y, 'Y', 'lateral')) if value is None)
        forces_known = None not in (t, h, y)
        coefficient_status = ILLUSTRATIVE_CALCULATION if any(map(
            illustrative_rotor_status, (operating_point.t_status, h_status, y_status))) else 'CALCULATED'
        if source and forces_known and None not in (density, shaft_tip_speed, radius):
            # A-38 uses shaft Omega, while A-20 uses angular-rate-corrected Omega prime.
            disk_area = math.pi * radius**2
            denominator = rotor_input_number(density * disk_area * shaft_tip_speed * shaft_tip_speed, positive=True)
            if denominator is not None:
                force_coefficient = rotor_input_number(math.hypot(t, h, y) / denominator)
            if force_coefficient is None:
                missing.append(f'{side} finite rotor force coefficient')
        audit.update({f'{side}_{key}': value for key, value in (
            ('beta_m_deg', mast_angle_deg),
            ('beta_m_status', ('CALCULATED' if mast_angle_source else 'PROVIDED')
             if mast_angle_deg is not None else 'UNAVAILABLE'),
            ('beta_m_source', mast_angle_source or 'Supplied rotor mast angle'),
            ('cg_to_hub_x_m', x), ('cg_to_hub_y_m', y_offset), ('cg_to_hub_z_m', z),
            ('cg_to_hub_status', offset_status), ('cg_to_hub_source', source if offset_known else ''),
            ('hub_u_body_mps', u_hb), ('hub_v_body_mps', v_hb), ('hub_w_body_mps', w_hb),
            ('hub_velocity_body_status', calculated_status if body_hub_known else 'UNAVAILABLE'),
            ('hub_velocity_body_source', 'NASA CR-166536 pp. A-18–A-19, V_hub = V_cg + body_rates × offset'),
            ('hub_u_mast_mps', hub_u), ('hub_v_mast_mps', hub_v), ('hub_w_mast_mps', hub_w),
            ('hub_velocity_status', calculated_status if hub_known else 'UNAVAILABLE'),
            ('hub_velocity_source', 'NASA CR-166536 pp. A-18–A-19'),
            ('wind_azimuth_rad', xi), ('wind_azimuth_status', 'CALCULATED' if xi is not None else 'UNAVAILABLE'),
            ('wind_azimuth_source', WIND_MAST_AZIMUTH_SOURCE),
            ('wind_azimuth_reason', '' if xi is not None else
             'Undefined at zero mast-plane speed' if mast_plane_speed == 0 else
             'Finite mast-plane U_HM and V_HM required'),
            ('wind_mast_flow_assumptions', wind_mast_assumptions),
            ('in_plane_effective_tip_speed_mps', tip_speed),
            ('in_plane_effective_tip_speed_status', 'CALCULATED' if tip_speed is not None else 'UNAVAILABLE'),
            ('in_plane_effective_tip_speed_source', 'NASA CR-166536 p. A-20, Omega prime × R'),
            ('rotor_shaft_tip_speed_mps', shaft_tip_speed),
            ('rotor_shaft_tip_speed_status', calculated_status if shaft_tip_speed is not None else 'UNAVAILABLE'),
            ('rotor_shaft_tip_speed_source', 'NASA CR-166536 p. A-21, shaft Ω × R'),
            ('rotor_tip_mach', tip_mach),
            ('rotor_tip_mach_status', calculated_status if tip_mach is not None else 'UNAVAILABLE'),
            ('rotor_tip_mach_source', 'NASA CR-166536 p. A-21, MTIP from V_T, shaft ΩR, β_m and speed of sound'),
            ('rotor_tip_mach_reason', ', '.join(tip_mach_missing) if tip_mach_missing else
             '' if tip_mach is not None else 'Finite MTIP required'),
            ('in_plane_lambda_zero', lambda_zero),
            ('in_plane_lambda_zero_status', 'CALCULATED' if lambda_zero is not None else 'UNAVAILABLE'),
            ('in_plane_lambda_zero_source', 'NASA CR-166536 p. A-20, -W_HM / (Omega prime × R)'),
            ('wind_mast_p_hat', p_hat), ('wind_mast_q_hat', q_hat),
            ('wind_mast_rates_status', 'CALCULATED' if None not in (p_hat, q_hat) else 'UNAVAILABLE'),
            ('wind_mast_rates_source', 'NASA CR-166536 pp. A-16–A-17, local rates / Omega prime'),
            ('rotor_rpm', rpm), ('rotor_omega_rad_s', omega), ('rotor_omega_effective_rad_s', omega_effective),
            ('rotor_rpm_status', operating_point.rpm_status if rpm is not None else 'UNAVAILABLE'),
            ('rotor_rpm_source', operating_point.rpm_source or source),
            ('rotor_omega_status', calculated_status if omega is not None else 'UNAVAILABLE'),
            ('rotor_omega_source', 'Operating RPM × 2π/60'),
            ('rotor_speed_status', calculated_status if omega_effective is not None else
             'PARTIAL / SHAFT SPEED ONLY' if omega is not None else 'UNAVAILABLE'),
            ('rotor_omega_effective_source', 'NASA CR-166536 p. A-16'),
            ('rotor_t_n', t), ('rotor_h_n', h), ('rotor_y_n', y),
            ('rotor_t_status', operating_point.t_status if t is not None else 'UNAVAILABLE'),
            ('rotor_t_source', operating_point.t_source or source),
            ('rotor_h_status', h_status if h is not None else 'UNAVAILABLE'),
            ('rotor_h_source', h_source if h is not None else MAST_FORCE_ROTATION_SOURCE),
            ('rotor_y_status', y_status if y is not None else 'UNAVAILABLE'),
            ('rotor_y_source', y_source if y is not None else MAST_FORCE_ROTATION_SOURCE),
            ('rotor_force_status', 'COMPLETE' if forces_known else 'UNAVAILABLE'),
            ('advance_ratio_status', calculated_status if advance_ratio is not None else 'UNAVAILABLE'),
            ('advance_ratio_source', 'NASA CR-166536 p. A-20'),
            ('force_coefficient_status', coefficient_status if force_coefficient is not None else 'UNAVAILABLE'),
            ('force_coefficient_source', 'NASA CR-166536 p. A-38'),
        )})
        # Transpose the A-18/A-19 body-to-mast rotation for the local force
        # vector (-H, Y, -T). Body x is forward and body z is down.
        transform_source = (
            'NASA CR-166536 pp. A-10, A-13, A-18–A-19, transposed body-to-mast axes'
        )
        thrust_term = (t, 'mast T', operating_point.t_source or source, operating_point.t_status)
        h_term = (h, 'mast H', h_source, h_status)
        y_term = (y, 'mast Y', y_source, y_status)
        missing_beta = [] if mast_angle_deg is not None else ['mast angle β_m']
        fx_terms = () if missing_beta else ((sin_beta, *thrust_term), (-cos_beta, *h_term))
        fx, fx_status, fx_reason = project_load(fx_terms, missing_beta)
        missing_fz = missing_beta.copy()
        if phi_deg is None:
            missing_fz.append('lateral mast tilt φ_m')
        elif not (inputs.lateral_mast_tilt_source or source):
            missing_fz.append('source for lateral mast tilt φ_m')
        fz_terms = ()
        if not missing_fz:
            cant_cos = 0.0 if abs(phi_deg) == 90 else cos_phi
            fz_terms = ((-cos_beta * cant_cos, *thrust_term),
                        (-sin_beta * cant_cos, *h_term), (sin_phi, *y_term))
        fz, fz_status, fz_reason = project_load(fz_terms, missing_fz)
        missing_arms = [label for value, label in ((x, 'CG-to-hub body x'), (z, 'CG-to-hub body z'))
                        if value is None]
        if not missing_arms and not source:
            missing_arms.append('source for CG-to-hub arms')
        pitch_terms = () if missing_arms else (
            (z, fx, 'body forward force from ' + fx_reason if fx_reason else 'body forward force',
             transform_source, fx_status),
            (-x, fz, 'body down force from ' + fz_reason if fz_reason else 'body down force',
             transform_source, fz_status),
        )
        pitch_arm, pitch_arm_status, pitch_arm_reason = project_load(pitch_terms, missing_arms)
        for name, value, units, equation, load_status, reason in (
            ('rotor_body_forward_force_n', fx, 'N', transform_source + ', T sin β_m − H cos β_m',
             fx_status, fx_reason),
            ('rotor_body_down_force_n', fz, 'N',
             transform_source + ', −(T cos β_m + H sin β_m) cos φ_m + Y sin φ_m',
             fz_status, fz_reason),
            ('rotor_cg_pitch_arm_moment_nm', pitch_arm, 'N·m',
             'NASA CR-166536 pp. A-18–A-19, signed CG-to-hub arms, (r × F)_y = z F_x − x F_z',
             pitch_arm_status, pitch_arm_reason),
        ):
            audit_rotor_value(audit, f'{side}_{name}', value, units, equation,
                              status=load_status, reason=reason)
        audit[f'mu_{side}'] = advance_ratio
        audit[f'c_rf_{side}'] = force_coefficient
        coefficient_dependencies = ', '.join(label for value, label in (
            (t, 'T'), (h, 'H'), (y, 'Y'), (density, 'air density'), (radius, 'rotor radius'),
            (shaft_tip_speed, 'shaft RPM'),
        ) if value is None)
        audit[f'{side}_force_coefficient_reason'] = (
            '' if force_coefficient is not None else f'Missing or invalid: {coefficient_dependencies}'
            if coefficient_dependencies else 'Requires sourced forces and a finite normalization')
        audit[f'{side}_rotor_h_reason'] = '' if h is not None else force_audit.get('nasa_rotor_h_n_reason', '')
        audit[f'{side}_rotor_y_reason'] = '' if y is not None else force_audit.get('nasa_rotor_y_n_reason', '')
        audit[f'{side}_advance_ratio_reason'] = (
            '' if advance_ratio is not None else 'Requires mast-axis U, V and positive effective tip speed')
        audit[f'{side}_in_plane_lambda_zero_reason'] = (
            '' if lambda_zero is not None else 'Requires mast-axis W and positive effective tip speed')
        audit[f'{side}_in_plane_effective_tip_speed_reason'] = (
            '' if tip_speed is not None else 'Requires positive effective rotor speed and radius')
        audit[f'{side}_rotor_rpm_reason'] = '' if rpm is not None else 'Requires finite positive operating RPM'
        audit[f'{side}_rotor_speed_reason'] = (
            '' if omega_effective is not None else 'Requires shaft RPM, body rates and applicable mast angles')
    geometry_known = 'UNAVAILABLE' not in geometry_statuses
    audit.update(hub_geometry_status=' / '.join(dict.fromkeys(geometry_statuses)) if geometry_known else 'UNAVAILABLE',
                 hub_geometry_source=source if geometry_known else '', hub_geometry_note='')
    if not geometry_known:
        effect = ('Zero body rates eliminate the hub-offset velocity terms.' if zero_body_rates else
                  'Required for body-rate corrections to hub velocity and μ.')
        audit['hub_geometry_note'] = f'Need {HUB_GEOMETRY_NEEDED}. {effect}'
    force_assumptions = ' '.join(dict.fromkeys(selected_force_assumptions))
    audit['rotor_flow_assumptions'] = force_assumptions
    resolved = XV15RotorFlow(source, audit['c_rf_left'], audit['c_rf_right'], audit['mu_left'], audit['mu_right'],
                             force_assumptions)
    return resolved, audit, missing


@dataclass(frozen=True)
class XV15WingPylonPolar:
    """NASA report inputs for one fixed flap/flaperon setting."""

    flap_pair: tuple
    lift_tables: tuple
    drag_tables: tuple
    rotor_wing_induction: tuple | None = None

    source = 'NASA CR-166536 Appendix B'
    status = 'NASA REPORT DATA / FREESTREAM ESTIMATE'
    user_source = None
    wing_alpha_source = 'NASA CR-166536 p. 25 and Appendix A pp. A-49–A-55, A-70'
    induction_source = 'NASA CR-166536 Appendix B p. B-33'
    # GTRS area documented in NASA TM-4448, Appendix A, citing CR-166536.
    reference_area_m2 = 181.0 * 0.09290304

    @classmethod
    def load(cls, flap_pair):
        data = runpy.run_path(str(DATA_DIR / 'xv15_wing_pylon_data.py'))
        lift, drag = (tuple(block for block in data[name] if block[3] == flap_pair)
                      for name in ('LIFT_TABLES', 'DRAG_TABLES'))
        return cls(flap_pair, lift, drag, data.get('ROTOR_WING_INDUCTION'))

    def rotor_flow_correction(self, beta_deg, rotor_flow):
        """Return the p. 25 wing AoA correction in radians, with its input audit."""
        rotor_flow = rotor_flow if rotor_flow is not None else XV15RotorFlow('')
        source = rotor_flow.source.strip() if isinstance(rotor_flow.source, str) else ''
        inputs = {}
        missing = []
        for key, value, label in (
            ('c_rf_left', rotor_flow.left_force_coefficient, 'left rotor force coefficient C_RFL'),
            ('c_rf_right', rotor_flow.right_force_coefficient, 'right rotor force coefficient C_RFR'),
            ('mu_left', rotor_flow.left_advance_ratio, 'left mast-plane advance ratio mu_L'),
            ('mu_right', rotor_flow.right_advance_ratio, 'right mast-plane advance ratio mu_R'),
        ):
            value = rotor_input_number(value)
            if value is None or value < 0:
                missing.append(label)
                value = None
            inputs[key] = value
        if any(value is not None for value in inputs.values()) and not source:
            missing.append('source for the supplied rotor coefficients and advance ratios')
        flow_status = ILLUSTRATIVE_ROTOR_FLOW if rotor_flow.assumptions else 'SOURCE-BACKED'
        inputs.update(rotor_flow_source=source, rotor_flow_status='UNAVAILABLE' if missing else flow_status,
                      rotor_flow_assumptions=rotor_flow.assumptions,
                      rotor_wing_induction=None, rotor_wing_induction_status='UNAVAILABLE',
                      rotor_wing_induction_source='', rotor_advance_ratio_mean=None,
                      rotor_advance_ratio_for_correction=None)

        if self.rotor_wing_induction is None:
            missing.append('rotor-to-wing constants K_XRW, X_RW0, X_RW1 and X_RW2')
        else:
            k_xrw, x_rw0, x_rw1, x_rw2 = self.rotor_wing_induction
            inputs['rotor_wing_induction_source'] = self.induction_source
            if beta_deg is not None:
                inputs.update(rotor_wing_induction=x_rw0 + beta_deg * (x_rw1 + beta_deg * x_rw2),
                              rotor_wing_induction_status='CALCULATED FROM SOURCE-BACKED CONSTANTS')
        if beta_deg is None:
            missing.append('common left/right nacelle angle in [0, 90] deg for NASA mast angle beta_m')
        if None not in (inputs['mu_left'], inputs['mu_right']) and source:
            # A-70 §4.B.3 gives the left/right mean. Fig. A4-2 covers wake immersion.
            inputs['rotor_advance_ratio_mean'] = 0.5 * inputs['mu_left'] + 0.5 * inputs['mu_right']
        if missing:
            return None, inputs, missing

        # The squared denominator follows p. 25. A-55 gives radians.
        mu_floor = max(inputs['rotor_advance_ratio_mean'], 0.15)
        inputs['rotor_advance_ratio_for_correction'] = mu_floor
        rotor_force_coefficient_sum = inputs['c_rf_right'] + inputs['c_rf_left']
        correction_rad = (k_xrw * inputs['rotor_wing_induction']
                          * rotor_force_coefficient_sum / (mu_floor * mu_floor))
        if not math.isfinite(correction_rad):
            return None, inputs, ['finite NASA wing AoA rotor-flow correction']
        return correction_rad, inputs, missing

    @staticmethod
    def coefficient_grid(tables, beta, mach):
        samples = {}
        for table, page, mast, _, mach_range, values in tables:
            if mast != beta or not mach_range[0] <= mach <= mach_range[1]:
                continue
            for angle, value in values.items():
                entries = samples.setdefault(angle, {})
                if value is not None:
                    entries.setdefault(value, set()).add(f'Table {table} p. {page}')
        return samples

    @classmethod
    def sample(cls, tables, alpha, beta, mach):
        """Evaluate one coefficient on its original AoA grid."""
        samples = cls.coefficient_grid(tables, beta, mach)
        angles = sorted(samples)
        index = bisect_left(angles, alpha)
        if index < len(angles) and angles[index] == alpha:
            entries = samples[alpha]
            if len(entries) == 1:
                value, sources = next(iter(entries.items()))
                return value, sources, False
            return None
        if not 0 < index < len(angles):
            return None
        lower, upper = angles[index - 1:index + 1]
        if len(samples[lower]) != 1 or len(samples[upper]) != 1:
            return None
        first, first_sources = next(iter(samples[lower].items()))
        last, last_sources = next(iter(samples[upper].items()))
        fraction = (alpha - lower) / (upper - lower)
        return first + fraction * (last - first), first_sources | last_sources, True

    def evaluate(self, alpha, beta, mach):
        if not 0 <= beta <= 90:
            return None, 'Report mast angle outside [0, 90] deg'
        # CR-166536 p. 25 calls for mast-angle interpolation. Evaluate AoA
        # on each eligible endpoint first, then blend at the requested mast angle.
        masts = (beta,) if beta in (0, 90) else (0, 90)
        coefficient_samples = {
            name: [self.sample(tables, alpha, mast, mach) for mast in masts]
            for name, tables in (('CL', self.lift_tables), ('CD', self.drag_tables))
        }
        missing = next(((name, mast) for name, samples in coefficient_samples.items()
                        for mast, sample in zip(masts, samples) if sample is None), None)
        status = 'NASA REPORT DATA'
        if missing is not None:
            name, missing_mast = missing
            tables = self.lift_tables if name == 'CL' else self.drag_tables
            grid = self.coefficient_grid(tables, missing_mast, mach)
            defined = sorted(angle for angle, samples in grid.items() if len(samples) == 1)
            if defined:
                coverage = f'defined AoA span [{defined[0]:g}, {defined[-1]:g}] deg'
                angles = sorted(grid)
                index = bisect_left(angles, alpha)
                if alpha in grid:
                    bracket = (alpha,)
                else:
                    bracket = angles[index - 1:index + 1] if 0 < index < len(angles) else ()
                gaps = [f'{angle:g}' for angle in bracket if len(grid[angle]) != 1]
                if gaps:
                    coverage += '. Blank or conflicting AoA row(s) at ' + ', '.join(gaps) + ' deg'
                elif not bracket:
                    coverage += '. Request is outside the AoA grid'
            else:
                coverage = 'no uniquely defined AoA rows at this flap setting and Mach'
            reason = (f'NASA report {name} unavailable at AoA {alpha:.6g} deg, '
                      f'Mach {mach:.6g}: required table mast {missing_mast:g} deg '
                      f'(requested mast {beta:.6g} deg). {coverage}')
            return None, reason

        coefficients, sources = [], []
        aoa_interpolated = False
        for name, samples in coefficient_samples.items():
            value = samples[0][0]
            if len(samples) == 2:
                value += (beta / 90) * (samples[1][0] - value)
            coefficients.append(value)
            pages = sorted(set().union(*(sample[1] for sample in samples)))
            sources.append(f'{name} ' + ', '.join(pages))
            aoa_interpolated |= any(sample[2] for sample in samples)
        axes = [axis for axis, used in (('AoA', aoa_interpolated), ('mast angle', len(masts) == 2)) if used]
        if axes:
            status += ' / INTERPOLATED (' + ' and '.join(axes) + ')'
        table_mast = masts[0] if len(masts) == 1 else None
        return (coefficients, status, self.source + ' — ' + '; '.join(sources), table_mast), ''


def transition_wing_forces(state, aircraft, environment, polar, *, rotor_flow=None):
    """Heading-aligned inertial plane. Lift points toward up at gamma = 0.

    V_air = V_ground - V_wind. Gamma = atan2(up, forward). With nose-up
    attitude positive, fuselage AoA is pitch minus gamma. The NASA lookup
    requires the rotor-corrected wing AoA. Cross-plane wind is excluded.
    """
    nasa = isinstance(polar, XV15WingPylonPolar)
    area = polar.reference_area_m2 if nasa else aircraft.wing_area_m2
    pitch = state.pitch_attitude_deg
    if isinstance(pitch, bool) or not isinstance(pitch, (int, float)) or not math.isfinite(pitch):
        pitch = None
    heading_deg = rotor_input_number(state.heading_deg)
    row = dict(wind_speed_mps=environment.windspeed, wind_from_deg=environment.winddirection,
               wind_forward_mps=None, wind_east_mps=None, wind_cross_plane_mps=None, air_forward_mps=None,
               air_up_mps=None, airspeed_mps=None, air_relative_velocity_status='UNAVAILABLE',
               heading_deg=heading_deg, heading_source='TMPE Phase 2 fixed aircraft heading',
               heading_status='PRESCRIBED' if heading_deg is not None else 'UNAVAILABLE',
               gamma_deg=None, gamma_status='UNAVAILABLE', pitch_attitude_deg=pitch,
               pitch_attitude_status='UNAVAILABLE' if pitch is None else 'USER-PRESCRIBED',
               pitch_attitude_source='' if pitch is None else 'Phase 2 pitch input',
               fuselage_alpha_deg=None, fuselage_alpha_rad=None, fuselage_alpha_status='UNAVAILABLE',
               wing_alpha_deg=None, wing_alpha_rad=None, wing_alpha_status='UNAVAILABLE', wing_alpha_source='',
               alpha_deg=None, q_pa=None,
               cl=None, cd=None, lift_n=None, drag_n=None, wing_forward_n=None, wing_up_n=None,
               air_density_kg_m3=environment.air_density, wing_area_m2=aircraft.wing_area_m2,
               wing_incidence_deg=aircraft.wing_incidence_deg, aero_reference_area_m2=area,
               aero_source=SIMPLIFIED_TRANSITION_LABEL if polar is None else polar.source,
               aero_user_source=None if polar is None else polar.user_source, aero_validation='UNVALIDATED',
               aero_status='UNAVAILABLE', aero_reason='')
    if nasa:
        flap, flaperon = polar.flap_pair
        row.update(beta_m_deg=None, table_mast_deg=None,
                   mach=None, flap_deg=flap, flaperon_deg=flaperon,
                   rotor_flow_correction_deg=None, rotor_flow_correction_rad=None,
                   rotor_flow_correction_status='UNAVAILABLE',
                   wing_alpha_source=polar.wing_alpha_source, wing_alpha_missing_inputs='',
                   wing_pylon_lookup_status='UNAVAILABLE')
    invalid_flow_inputs = []
    for label, value, minimum in (
        ('Wind speed', environment.windspeed, 0),
        ('Wind FROM direction', environment.winddirection, None),
        ('Aircraft heading', heading_deg, None),
        ('Forward velocity', state.forward_velocity_mps, None),
        ('Vertical velocity', state.vertical_velocity_mps, None),
    ):
        try:
            require_finite(label, value, minimum)
        except (TypeError, ValueError):
            invalid_flow_inputs.append(label)
    forward = up = speed = None
    density = rotor_input_number(environment.air_density)
    if not invalid_flow_inputs:
        wind_north, wind_east, _ = wind_velocity_ned(environment.windspeed, environment.winddirection)
        heading = math.radians(heading_deg)
        cos_heading, sin_heading = math.cos(heading), math.sin(heading)
        wind_forward = wind_north * cos_heading + wind_east * sin_heading
        wind_cross_plane = -wind_north * sin_heading + wind_east * cos_heading
        forward = state.forward_velocity_mps - wind_forward
        up = state.vertical_velocity_mps
        speed = math.hypot(forward, up)
        row.update(wind_forward_mps=wind_forward, wind_east_mps=wind_east,
                   wind_cross_plane_mps=wind_cross_plane, air_forward_mps=forward,
                   air_up_mps=up, airspeed_mps=speed, air_relative_velocity_status='CALCULATED')
        if speed > FLOW_EPS_MPS:
            gamma = math.atan2(up, forward)
            row.update(gamma_deg=math.degrees(gamma), gamma_status='CALCULATED')
            if pitch is not None:
                fuselage_alpha = math.radians(pitch) - gamma
                row.update(fuselage_alpha_deg=math.degrees(fuselage_alpha), fuselage_alpha_rad=fuselage_alpha,
                           fuselage_alpha_status='CALCULATED FROM USER-PRESCRIBED PITCH')
        if density is not None and math.isfinite(density) and density >= 0:
            row['q_pa'] = .5 * density * speed**2
    mast_angles = tuple(90 - angle if angle is not None and 0 <= angle <= 90 else None
                        for angle in (rotor_input_number(state.left_nacelle_angle_deg),
                                      rotor_input_number(state.right_nacelle_angle_deg)))
    beta = mast_angles[0] if mast_angles[0] == mast_angles[1] else None
    row.update(beta_m_deg=beta, beta_m_status='CALCULATED' if beta is not None else 'UNAVAILABLE',
               beta_m_source='90° minus the common Phase 2 nacelle angle')
    if rotor_flow is None:
        rotor_flow = transition_rotor_inputs(state, aircraft)
    rotor_missing = []
    if isinstance(rotor_flow, XV15RotorInputs):
        rotor_flow, input_audit, rotor_missing = calculate_xv15_rotor_flow(
            rotor_flow, aircraft, beta, pitch, forward, up, density, rotor_mast_angles_deg=mast_angles,
            mast_angle_source='90° minus the rotor nacelle angle in Phase 2', temperature_c=environment.temperature)
        row.update(input_audit)
        row['rotor_flow_missing_inputs'] = ', '.join(rotor_missing)
    else:
        row['rotor_flow_input_note'] = 'Externally supplied C_RF and advance ratios; raw rotor inputs unavailable.'
    row.update(transition_environment_audit(environment))
    if invalid_flow_inputs and not nasa:
        row['aero_reason'] = f'{invalid_flow_inputs[0]} is missing or invalid'
        return row
    if polar is None:
        cl, cd = SIMPLIFIED_TRANSITION_CL, SIMPLIFIED_TRANSITION_CD
        row.update(cl=cl, cd=cd)
        for label, value, valid in (
            ('Air density', row['q_pa'], row['q_pa'] is not None),
            ('Wing reference area', area, area is not None and area > 0),
            ('Assumed CL', cl, cl is not None),
            ('Assumed CD', cd, cd is not None and cd >= 0),
        ):
            if not valid or not math.isfinite(value):
                row['aero_reason'] = f'{label} is missing or invalid'
                return row
        lift = row['q_pa'] * area * cl
        drag = row['q_pa'] * area * cd
        row.update(lift_n=lift, drag_n=drag, wing_forward_n=-drag, wing_up_n=lift,
                   aero_status='ILLUSTRATIVE / SOFTWARE-TEST ASSUMPTIONS')
        return row
    if nasa:
        sound_speed = speed_of_sound_mps(environment.temperature)
        if speed is not None and sound_speed is not None:
            row['mach'] = speed / sound_speed
        correction, rotor_audit, correction_missing = polar.rotor_flow_correction(beta, rotor_flow)
        row.update(rotor_audit)
        illustrative = bool(rotor_flow.assumptions)
        missing = rotor_missing + correction_missing
        row['rotor_flow_missing_inputs'] = ', '.join(dict.fromkeys(missing))
        if correction is not None and not rotor_missing:
            row.update(rotor_flow_correction_rad=correction, rotor_flow_correction_deg=math.degrees(correction),
                       rotor_flow_correction_status=(ILLUSTRATIVE_ROTOR_FLOW if illustrative else
                                                     'CALCULATED FROM SOURCE-BACKED INPUTS'))
        if invalid_flow_inputs:
            missing.extend(invalid_flow_inputs)
        elif row['gamma_deg'] is None:
            missing.append('sufficient planar airspeed for flight-path angle gamma')
        if pitch is None:
            missing.append('Phase 2 pitch attitude (--transition-pitch-deg) for fuselage AoA and planar U/W')
        if missing:
            row['wing_alpha_missing_inputs'] = ', '.join(dict.fromkeys(missing))
            row['aero_reason'] = 'NASA-corrected wing AoA unavailable. Missing or invalid: ' + row['wing_alpha_missing_inputs']
            if invalid_flow_inputs:
                row['aero_first_blocker'] = invalid_flow_inputs[0]
            elif pitch is None:
                row['aero_first_blocker'] = 'Phase 2 pitch attitude (--transition-pitch-deg)'
            elif row['gamma_deg'] is None:
                row['aero_first_blocker'] = 'Nonzero planar airspeed for flight-path angle gamma'
            else:
                row['aero_first_blocker'] = missing[0]
            return row
        alpha_rad = row['fuselage_alpha_rad'] - correction
        alpha = math.degrees(alpha_rad)  # Appendix B table lookup uses degrees.
        if not math.isfinite(alpha):
            row['aero_reason'] = 'NASA-corrected wing AoA unavailable. Expected a finite result.'
            return row
        row.update(wing_alpha_deg=alpha, wing_alpha_rad=alpha_rad, alpha_deg=alpha,
                   wing_alpha_status=(ILLUSTRATIVE_ROTOR_FLOW if illustrative else
                                      'CALCULATED FROM SOURCE-BACKED ROTOR INPUTS AND USER-PRESCRIBED PITCH'))
        if row['mach'] is None:
            row['aero_reason'] = 'A finite air temperature above absolute zero is required for Mach'
            return row
        selected, reason = polar.evaluate(alpha, beta, row['mach'])
        if selected is None:
            row['aero_reason'] = reason
            return row
        coefficients, lookup_status, row['aero_source'], table_mast = selected
        if illustrative:
            lookup_status = ILLUSTRATIVE_ROTOR_FLOW + ' / ' + lookup_status.replace(
                'NASA REPORT DATA', 'REPORT TABLE LOOKUP')
        if table_mast is not None:
            row['table_mast_deg'] = table_mast
        row['wing_pylon_lookup_status'] = lookup_status
        row['aero_status'] = lookup_status + ' / FREESTREAM ESTIMATE'
    else:
        if row['gamma_deg'] is None:
            row['aero_reason'] = 'Flight-path angle and AoA undefined at insufficient planar airspeed'
            return row
        incidence = aircraft.wing_incidence_deg
        if incidence is None or not math.isfinite(incidence):
            row['aero_reason'] = 'Wing incidence unavailable'
            return row
        fuselage_alpha = row['fuselage_alpha_deg']
        alpha = incidence + (fuselage_alpha if fuselage_alpha is not None else -row['gamma_deg'])
        basis = 'PRESCRIBED PITCH' if pitch is not None else 'ASSUMED LEVEL ATTITUDE'
        row.update(wing_alpha_deg=alpha, alpha_deg=alpha,
                   wing_alpha_status=f'APPROXIMATE / USER POLAR / {basis}',
                   wing_alpha_source='Geometric AoA with XML wing incidence for the user polar')
        coefficients = polar.coefficients(alpha)
        if coefficients is None:
            row['aero_reason'] = f'AoA outside table range [{polar.rows[0][0]}, {polar.rows[-1][0]}] deg'
            return row
        row['aero_status'] = polar.status
    row['cl'], row['cd'] = coefficients
    if row['q_pa'] is None or area is None or not math.isfinite(area) or area <= 0:
        row.update(aero_status='UNAVAILABLE', aero_reason='Air density or wing reference area unavailable or invalid')
        return row
    lift, drag = (row['q_pa'] * area * coefficient for coefficient in coefficients)
    sine, cosine = up / speed, forward / speed
    row.update(lift_n=lift, drag_n=drag, wing_forward_n=-lift * sine - drag * cosine,
               wing_up_n=lift * cosine - drag * sine)
    return row


PRESCRIBED_TRANSITION = 'PRESCRIBED KINEMATIC / ILLUSTRATIVE / UNVALIDATED'
LONGITUDINAL_3DOF_BLOCKER = 'Total pitching moment about the operating CG and matched pitch inertia through conversion are unavailable.'


def simulate_transition(takeoff, aircraft, environment, polar, duration_s, timestep_s, end_angles, *,
                        pitch_attitude_deg=None, rotor_flow_provider=None, illustrative_rotor_flow=False,
                        heading_deg=0.0, prescribed_velocity_mps=None, prescribed_start_height_m=None,
                        prescribed_start_angles_deg=None):
    """Integrate available forces or evaluate an explicit constant-velocity schedule.

    A rotor_flow_provider(state, environment) may supply XV15RotorInputs for
    each NASA lookup or return resolved XV15RotorFlow. The built-in provider
    uses prescribed pitch and needs the illustrative opt-in to assume H = Y = 0.
    """
    for label, value in (('Phase 2 duration', duration_s), ('Timestep', timestep_s)):
        require_finite(label, value, 0)
        if value == 0:
            raise ValueError(f'{label} must be positive')
    left_end, right_end = map(validate_transition_angle, end_angles)
    if pitch_attitude_deg is not None:
        require_finite('Phase 2 pitch attitude', pitch_attitude_deg)
    require_finite('Phase 2 heading', heading_deg)
    handoff = initialize_transition(takeoff)
    state = replace(handoff, pitch_attitude_deg=pitch_attitude_deg,
                    heading_deg=heading_deg)
    prescribed = prescribed_velocity_mps is not None
    if not prescribed and (prescribed_start_height_m is not None or prescribed_start_angles_deg is not None):
        raise ValueError('Prescribed starting values require prescribed Phase 2 velocity')
    if prescribed:
        vx, vz = prescribed_velocity_mps
        require_finite('Prescribed forward velocity', vx)
        require_finite('Prescribed up velocity', vz)
        height = state.height_agl_m if prescribed_start_height_m is None else prescribed_start_height_m
        require_finite('Prescribed start height AGL', height, 0)
        final_height = height + vz * duration_s
        if not math.isfinite(final_height) or final_height < 0:
            raise ValueError('Prescribed trajectory must end at a finite height at or above ground')
        if not math.isfinite(state.forward_distance_m + vx * duration_s):
            raise ValueError('Prescribed trajectory must end at a finite forward distance')
        state = replace(state, height_agl_m=height, forward_velocity_mps=vx, vertical_velocity_mps=vz)
        if prescribed_start_angles_deg is not None:
            left_start, right_start = map(validate_transition_angle, prescribed_start_angles_deg)
            state = replace(state, left_nacelle_angle_deg=left_start, right_nacelle_angle_deg=right_start)
    origin = state
    trajectory_source = ('Explicit constant forward/up velocity and linear nacelle schedule' if prescribed else
                         'TMPE planar force balance and Phase 1 handoff')
    trajectory_status = PRESCRIBED_TRANSITION if prescribed else 'ILLUSTRATIVE / SOFTWARE-TEST ONLY'
    result = dict(final_state=state, elapsed_s=0.0, requested_s=duration_s, rows=[], simplified=polar is None,
                  status='UNAVAILABLE', reason='', first_blocker='', prescribed=prescribed)
    coefficient_audit = transition_thrust_coefficient_audit(takeoff)
    start_audit = dict(phase1_handoff_height_agl_m=handoff.height_agl_m,
                       phase1_handoff_up_mps=handoff.vertical_velocity_mps,
                       phase1_handoff_status=takeoff['motion']['takeoff_status'],
                       phase1_handoff_left_nacelle_deg=handoff.left_nacelle_angle_deg,
                       phase1_handoff_right_nacelle_deg=handoff.right_nacelle_angle_deg,
                       phase2_start_height_source=('USER-PRESCRIBED' if prescribed_start_height_m is not None else
                                                   'PHASE 1 RETAINED STATE'),
                       phase2_start_velocity_source='USER-PRESCRIBED' if prescribed else 'PHASE 1 RETAINED STATE',
                       phase2_start_nacelle_source=('USER-PRESCRIBED' if prescribed_start_angles_deg is not None else
                                                    'PHASE 1 RETAINED STATE'),
                       phase2_requested_mode='PRESCRIBED KINEMATIC' if prescribed else 'FORCE-BASED',
                       longitudinal_3dof_status='UNAVAILABLE', longitudinal_3dof_reason=LONGITUDINAL_3DOF_BLOCKER)
    if not prescribed and (takeoff['motion']['takeoff_status'] != 'AIRBORNE' or state.height_agl_m <= 0):
        result['reason'] = (f"Phase 1 ended {takeoff['motion']['takeoff_status']}. "
                            f"{takeoff['motion']['reason']}")
        result['first_blocker'] = 'Airborne Phase 1 handoff with positive height AGL'
        preflight = dict(coefficient_audit, **transition_environment_audit(environment), **start_audit,
                         audit_record='PREFLIGHT ONLY', phase2_time_s=None, simulation_time_s=state.elapsed_s,
                         first_force_blocker=result['first_blocker'],
                         phase2_motion_mode='NOT ADVANCED', trajectory_status='PHASE 1 RETAINED STATE',
                         phase2_force_balance_role='UNAVAILABLE FOR MOTION',
                         rotor_model=takeoff['rotor_model'], aircraft_transition_status='UNAVAILABLE',
                         aero_status='UNAVAILABLE', aero_reason=result['reason'],
                         heading_deg=heading_deg, pitch_attitude_deg=pitch_attitude_deg,
                         pitch_attitude_status='USER-PRESCRIBED' if pitch_attitude_deg is not None else 'UNAVAILABLE',
                         body_u_mps=None, body_v_mps=None, body_w_mps=None, body_velocity_status='NOT EVALUATED')
        rpm = rotor_input_number(aircraft.hover_rotor_rpm, positive=True)
        for side in ('left', 'right'):
            preflight.update({f'{side}_{key}': value for key, value in (
                ('thrust_n', takeoff[side]['value']),
                ('rotor_rpm', rpm), ('rotor_rpm_status', HELD_HOVER_RPM_STATUS if rpm is not None else 'UNAVAILABLE'),
                ('rotor_rpm_source', f'rotor.xml: {HOVER_RPM_XML_PATH}'),
                ('hub_u_body_mps', None), ('hub_v_body_mps', None), ('hub_w_body_mps', None),
                ('hub_u_mast_mps', None), ('hub_v_mast_mps', None), ('hub_w_mast_mps', None),
                ('wind_azimuth_rad', None), ('wind_azimuth_status', 'NOT EVALUATED'),
                ('wind_azimuth_source', WIND_MAST_AZIMUTH_SOURCE),
                ('wind_azimuth_reason', result['reason']),
                ('rotor_h_bar_n', None), ('rotor_h_bar_n_status', 'NOT EVALUATED'),
                ('rotor_h_bar_n_source', 'NASA CR-166536 p. A-30, Section 9'),
                ('rotor_y_bar_n', None), ('rotor_y_bar_n_status', 'NOT EVALUATED'),
                ('rotor_y_bar_n_source', 'NASA CR-166536 p. A-31, Section 9'),
                ('rotor_t_n', None), ('rotor_h_n', None), ('rotor_y_n', None),
                ('hub_velocity_status', 'NOT EVALUATED'),
            )})
            preflight[f'mu_{side}'] = preflight[f'c_rf_{side}'] = None
        result['preflight_audit'] = preflight
        return result
    starts = (state.left_nacelle_angle_deg, state.right_nacelle_angle_deg)
    mass = takeoff['takeoff_mass_kg']
    time = 0.0
    step = 0
    landed = False
    while True:
        if time == duration_s:
            angles = (left_end, right_end)
        else:
            fraction = time / duration_s
            angles = tuple(None if start is None else start + fraction * (end - start)
                           for start, end in zip(starts, (left_end, right_end)))
        state = replace(state, left_nacelle_angle_deg=angles[0], right_nacelle_angle_deg=angles[1])
        row = dict(coefficient_audit, **start_audit, audit_record='PHASE 2 EVALUATION',
                   phase2_time_s=time, simulation_time_s=state.elapsed_s, height_agl_m=state.height_agl_m,
                   forward_distance_m=state.forward_distance_m, forward_velocity_mps=state.forward_velocity_mps,
                   vertical_velocity_mps=state.vertical_velocity_mps, mass_kg=mass,
                   aircraft_weight_n=None if mass is None else mass * G, rotor_model=takeoff['rotor_model'],
                   trajectory_status=trajectory_status, trajectory_source=trajectory_source,
                   phase2_motion_mode=start_audit['phase2_requested_mode'],
                   phase2_force_balance_role='DIAGNOSTIC ONLY' if prescribed else 'MOTION INPUT',
                   aircraft_transition_status='UNAVAILABLE')
        rotor_vectors = []
        for side, angle in zip(('left', 'right'), angles):
            rotor = takeoff[side]
            vector = rotor_force_body_n(rotor['value'], angle)
            rotor_vectors.append(vector)
            row.update({f'{side}_nacelle_deg': angle, f'{side}_thrust_n': rotor['value'],
                        f'{side}_rotor_forward_n': None if vector is None else vector[0],
                        f'{side}_rotor_up_n': None if vector is None else -vector[2],
                        f'{side}_rotor_source': rotor.get('source') or rotor['reason'],
                        f'{side}_rotor_configuration_source': rotor.get('configuration_source'),
                        f'{side}_rotor_status': rotor['status'],
                        f'{side}_rotor_reason': rotor['reason'],
                        f'{side}_rotor_calibration': rotor.get('source_status', rotor.get('calibration_status', 'UNAVAILABLE'))})
        rotor_flow = (rotor_flow_provider(state, environment) if rotor_flow_provider is not None else
                      transition_rotor_inputs(state, aircraft, illustrative=illustrative_rotor_flow, takeoff=takeoff))
        row.update(transition_wing_forces(state, aircraft, environment, polar, rotor_flow=rotor_flow))
        rotors_known = all(vector is not None for vector in rotor_vectors)
        rotor_forward = sum(vector[0] for vector in rotor_vectors) if rotors_known else None
        rotor_up = -sum(vector[2] for vector in rotor_vectors) if rotors_known else None
        wing_forward, wing_up, weight = row['wing_forward_n'], row['wing_up_n'], row['aircraft_weight_n']
        net_forward = None if rotor_forward is None or wing_forward is None else rotor_forward + wing_forward
        net_up = None if rotor_up is None or wing_up is None or weight is None else rotor_up + wing_up - weight
        row.update(total_rotor_forward_n=rotor_forward, total_rotor_up_n=rotor_up,
                   net_forward_n=net_forward, net_up_n=net_up)
        acceleration = planar_force_balance(mass, net_forward, net_up)
        row.update(acceleration)
        ax, az = acceleration['forward_acceleration_mps2'], acceleration['vertical_acceleration_mps2']
        balance_reason = ''
        first_force_blocker = ''
        if ax is None or az is None:
            missing = [f'{side} {quantity}' for side in ('left', 'right')
                       for quantity, suffix in (('rotor thrust', 'thrust_n'), ('nacelle angle', 'nacelle_deg'))
                       if row[f'{side}_{suffix}'] is None]
            if mass is None:
                missing.append('aircraft mass')
            reasons = [row['aero_reason']] if row['aero_reason'] else []
            if missing:
                reasons.append('Missing ' + ', '.join(missing))
            balance_reason = '. '.join(reasons) or 'Both net force components and mass are required'
            first_force_blocker = missing[0] if missing else row.get('aero_first_blocker') or balance_reason
        row.update(force_balance_status='UNAVAILABLE' if balance_reason else 'CALCULATED',
                   force_balance_reason=balance_reason, first_force_blocker=first_force_blocker)
        if balance_reason and not prescribed:
            row['phase2_force_balance_role'] = 'UNAVAILABLE FOR MOTION'
            if time == 0:
                row.update(phase2_motion_mode='NOT ADVANCED', trajectory_status='PHASE 1 RETAINED STATE',
                           trajectory_source='Phase 1 retained state')
        result['rows'].append(row)
        result.update(final_state=state, elapsed_s=time)
        if landed:
            result.update(status='GROUND CONTACT', reason='Airborne integration stopped at ground contact')
            break
        if not prescribed and balance_reason:
            result['reason'] = balance_reason
            result['first_blocker'] = first_force_blocker
            break
        if time == duration_s:
            result.update(status='PRESCRIBED' if prescribed else 'CALCULATED', reason='Requested duration reached')
            break

        step += 1
        next_time = min(step * timestep_s, duration_s)
        dt = next_time - time
        if dt <= 0:
            result['reason'] = 'Timestep too small to advance the simulation clock'
            break
        if prescribed:
            state = replace(state, elapsed_s=origin.elapsed_s + next_time,
                            height_agl_m=origin.height_agl_m + origin.vertical_velocity_mps * next_time,
                            forward_distance_m=origin.forward_distance_m + origin.forward_velocity_mps * next_time)
            time = next_time
            continue
        h, vz, vx = state.height_agl_m, state.vertical_velocity_mps, state.forward_velocity_mps
        height = h + vz * dt + .5 * az * dt**2
        # An upward acceleration can bring the aircraft back above ground
        # by the end of the step even if the quadratic reached ground earlier.
        minimum_time = -vz / az if az > 0 and vz < 0 else dt
        minimum_time = min(dt, minimum_time)
        landed = h + vz * minimum_time + .5 * az * minimum_time**2 <= 0
        if landed:
            # Positive ground-intersection time for this constant-acceleration step.
            dt = 2 * h / (math.sqrt(max(0.0, vz**2 - 2 * az * h)) - vz)
            height = 0.0
            next_time = time + dt
        state = replace(state, elapsed_s=takeoff['elapsed_s'] + next_time, height_agl_m=height,
                        forward_distance_m=state.forward_distance_m + vx * dt + .5 * ax * dt**2,
                        forward_velocity_mps=vx + ax * dt, vertical_velocity_mps=vz + az * dt)
        time = next_time
    return result


def report_transition_equations(simplified=False, nasa=False, prescribed=False):
    print('The inertial plane follows the fixed heading. Forward and up are positive. Cross-plane wind is excluded.')
    print('Nacelles: independent linear ramps over requested duration, 0 deg forward / 90 deg up.')
    print('Rotor magnitudes held at Phase 1 values with original calibration status. Prototype T = CT rho A (Omega R)^2.')
    print('V_air = V_ground - V_wind; along-heading wind = -speed*cos(wind_from - heading).')
    print('q = 0.5 rho |V_air|²; L = q S CL; D = q S CD.')
    if simplified:
        print('Constant CL and CD; wing forward = -D; wing up = L.')
    else:
        print('gamma = atan2(V_air_up, V_air_forward).')
        print('Fuselage alpha_F = user-prescribed nose-up pitch - gamma.')
        if nasa:
            print('Body U = V_air_forward*cos(pitch) + V_air_up*sin(pitch).')
            print('Body W = V_air_forward*sin(pitch) - V_air_up*cos(pitch).')
            print('Body V = 0 m/s and p = q = r = 0 rad/s in the planar model.')
            print('NASA report mast beta_m = 90 deg - nacelle angle.')
            print('Appendix A pp. A-18–A-20 project the air-relative hub velocities into each rotor mast plane.')
            print("Each rotor's advance ratio is hypot(U_HM, V_HM) / (Omega′ * R).")
            print('Appendix A p. A-38 uses the resultant of T, H and Y for each rotor force coefficient.')
            print('x_R/W = X_RW0 + beta_m * (X_RW1 + beta_m * X_RW2), with beta_m in degrees.')
            print('NASA p. 25 wing AoA: alpha_W = alpha_F - K_XRW * x_R/W * (C_RFR + C_RFL) / max(mu, 0.15)^2.')
            print('mu = (mu_R + mu_L) / 2, per Appendix A p. A-70.')
            print('Convert the resulting wing AoA from radians to degrees at the table lookup.')
            print('S = 181 ft². CL/CD use matched AoA, flap and Mach coverage at the selected mast endpoints.')
        else:
            print('User polar wing AoA approximation = pitch + incidence - gamma; level attitude if pitch is absent.')
        print('Wing forward = -L*sin(gamma) - D*cos(gamma); wing up = L*cos(gamma) - D*sin(gamma).')
    print('The held-thrust demonstration projects each rotor forward = T*cos(nacelle); up = T*sin(nacelle).')
    print('Net forward = rotor forward + wing forward; net up = rotor up + wing up - mass*g.')
    if prescribed:
        print('Prescribed position = starting position + user-supplied velocity × Phase 2 time.')
        print('Any evaluated forces and net-force accelerations are diagnostics for the prescribed state.')
    else:
        print('a = F_net/m; v_next = v + a*dt; position_next = position + v*dt + 0.5*a*dt².')


AIRCRAFT_TRANSITION_BLOCKER = (
    'Missing installed per-rotor thrust and shaft-power maps across nacelle angle, inflow/airspeed, '
    'RPM or tip Mach, density and blade collective, plus rotor-wake effects on wing-pylon forces.'
)


def report_cell(value):
    return '' if value is None else format(value, '.4g')


def print_report_table(headers, rows):
    table = (headers, *rows)
    widths = [max(map(len, column)) for column in zip(*table)]
    for row in table:
        print('  '.join(cell.rjust(width) for cell, width in zip(row, widths)).rstrip())


def transition_checkpoints(rows):
    if not rows:
        return []
    last_time = rows[-1]['phase2_time_s']
    targets = [2.5 * step for step in range(1, math.ceil(last_time / 2.5))]
    indices = [0, *(min(range(len(rows)), key=lambda i: abs(rows[i]['phase2_time_s'] - target))
                    for target in targets), len(rows) - 1]
    return [rows[i] for i in sorted(set(indices))]


def compact_aero_reason(row):
    details = row.get('wing_alpha_missing_inputs')
    if not details:
        return row['aero_reason']
    missing = [label for key, label in (
        ('pitch_attitude_status', 'prescribed pitch (--transition-pitch-deg)'),
        ('beta_m_status', 'a common mast angle'),
        ('gamma_status', 'a flight-path angle'),
        ('rotor_flow_status', 'complete rotor-flow inputs'),
    ) if row.get(key) == 'UNAVAILABLE']
    if missing:
        return 'NASA-corrected wing AoA requires ' + ', '.join(missing)
    return 'NASA-corrected wing AoA unavailable. Missing or invalid ' + details.split(', ', 1)[0]


def report_timed_transition(result, audit_path=None, *, verbose=False):
    print('\nPHASE 2 — TIMED TRANSITION')
    rows = result['rows']
    checkpoints = transition_checkpoints(rows)
    print(f"Phase 2 elapsed: {result['elapsed_s']:.6g} / {result['requested_s']:.6g} s")
    status, reason = result['status'], result['reason']
    if not verbose and rows and rows[-1]['aero_reason']:
        reason = reason.replace(rows[-1]['aero_reason'], compact_aero_reason(rows[-1]), 1)
    if result.get('prescribed'):
        print(f'Phase 2 demonstration — {PRESCRIBED_TRANSITION}')
        print('Force balance is diagnostic only.')
    elif status == 'UNAVAILABLE':
        print('Phase 2 force-based request — BLOCKED')
    else:
        print(f'Phase 2 force-based — ILLUSTRATIVE / {status}')
    if rows:
        first = rows[0]
        pitch = first['pitch_attitude_deg']
        left = report_cell(first['left_nacelle_deg']) or 'unavailable'
        right = report_cell(first['right_nacelle_deg']) or 'unavailable'
        print(f"Start {first['height_agl_m']:.4g} m AGL ({first['phase2_start_height_source']}) | "
              f"Velocity {first['forward_velocity_mps']:.4g} m/s forward, {first['vertical_velocity_mps']:.4g} m/s up "
              f"({first['phase2_start_velocity_source']})")
        pitch_display = 'UNAVAILABLE' if pitch is None else f'{pitch:g}° (USER-PRESCRIBED)'
        print(f"Start nacelles L/R {left}/{right} deg ({first['phase2_start_nacelle_source']}) | Pitch {pitch_display}")
    first_blocker = result.get('first_blocker')
    if first_blocker and first_blocker != reason:
        print(f'First missing force input — {first_blocker}')
    if reason:
        label = 'Blocker' if status == 'UNAVAILABLE' else 'Outcome'
        print(f'{label} — {reason}')
    if checkpoints:
        print('\nPHASE 2 CHECKPOINTS')
        print('Start, every 2.5 s, and end, using nearest evaluated timesteps.')
        columns = (('t (s)', 'phase2_time_s'), ('Nac L (deg)', 'left_nacelle_deg'),
                   ('Nac R (deg)', 'right_nacelle_deg'), ('AGL (m)', 'height_agl_m'),
                   ('Fwd (m/s)', 'forward_velocity_mps'), ('Up (m/s)', 'vertical_velocity_mps'),
                   ('Lift (N)', 'lift_n'), ('Drag (N)', 'drag_n'), ('Net up (N)', 'net_up_n'))
        print_report_table([label for label, _ in columns],
                           [tuple(report_cell(row.get(key)) for _, key in columns) for row in checkpoints])
        if verbose:
            report_transition_checkpoint_details(checkpoints)
    else:
        print('Phase 2 checkpoints — NOT EVALUATED')
    print('\nCSV OUTPUTS')
    report_transition_csvs(result, checkpoints, audit_path)


def report_transition_checkpoint_details(checkpoints):
    print('Forward and upward values are positive in their named directions.')

    def number(value):
        if value is None or value == '':
            return ''
        if value == 0:
            return '0'
        text = format(value, '.3g')
        if value and 0.001 <= abs(value) < 1e6 and 'e' in text:
            text = format(float(text), f'.{max(0, 2 - math.floor(math.log10(abs(value))))}f')
        return text

    asymmetric = any(row['left_nacelle_deg'] != row['right_nacelle_deg'] for row in checkpoints)

    def angle(row):
        left = number(row['left_nacelle_deg'])
        return left + '/' + number(row['right_nacelle_deg']) if asymmetric else left

    def table(title, columns):
        print('\n' + title)
        values = [[getter(row) for _, getter in columns] for row in checkpoints]
        widths = [max(len(label), *(len(row[i]) for row in values))
                  for i, (label, _) in enumerate(columns)]
        print('  '.join(label.rjust(width) for (label, _), width in zip(columns, widths)))
        print('  '.join('—' * width for width in widths))
        for row in values:
            print('  '.join(value.rjust(width) for value, width in zip(row, widths)))

    def field(key):
        return lambda row: number(row.get(key))

    table('TRANSITION CHECKPOINTS', (
        ('t (s)', field('phase2_time_s')),
        ('Tilt L/R (deg)' if asymmetric else 'Tilt (deg)', angle),
        ('AGL (m)', field('height_agl_m')),
        ('Fwd (m/s)', field('forward_velocity_mps')),
        ('Up (m/s)', field('vertical_velocity_mps')),
        ('Air (m/s)', field('airspeed_mps')),
    ))
    table('FORCE CHECKPOINTS — upward positive', (
        ('t (s)', field('phase2_time_s')),
        ('Rotor up (N)', field('total_rotor_up_n')),
        ('Lift (N)', field('lift_n')),
        ('Net up (N)', field('net_up_n')),
        ('a up (m/s²)', field('vertical_acceleration_mps2')),
    ))
    nasa = 'wing_pylon_lookup_status' in checkpoints[0]
    aoa_columns = (
        ('t (s)', field('phase2_time_s')),
        ('Air fwd (m/s)', field('air_forward_mps')),
        ('Air up (m/s)', field('air_up_mps')),
        ('Pitch (deg)', field('pitch_attitude_deg')),
        ('Gamma (deg)', field('gamma_deg')),
        ('Fuselage AoA (deg)', field('fuselage_alpha_deg')),
    )
    if nasa:
        aoa_columns += (('Rotor correction (deg)', field('rotor_flow_correction_deg')),)
    table('ANGLE OF ATTACK CHECKPOINTS', aoa_columns + (('Wing AoA (deg)', field('wing_alpha_deg')),))
    if nasa:
        table('ROTOR-FLOW CHECKPOINTS', (
            ('t (s)', field('phase2_time_s')),
            ('μL', field('mu_left')),
            ('μR', field('mu_right')),
            ('Mean μ', field('rotor_advance_ratio_mean')),
            ('Floored μ', field('rotor_advance_ratio_for_correction')),
            ('CRFL', field('c_rf_left')),
            ('CRFR', field('c_rf_right')),
        ))
    lookup_columns = (
        ('t (s)', field('phase2_time_s')),
        ('Tilt L/R (deg)' if asymmetric else 'Tilt (deg)', angle),
    )
    if nasa:
        first = checkpoints[0]
        print(f'NASA flap/flaperon setting: {first["flap_deg"]:g}°/{first["flaperon_deg"]:g}° throughout Phase 2.')
        lookup_columns += (('Requested mast (deg)', field('beta_m_deg')),
                           ('Table mast (deg)', field('table_mast_deg')), ('Mach', field('mach')))
    table('WING-PYLON DRAG CHECKPOINTS' if nasa else 'WING DRAG CHECKPOINTS', lookup_columns + (
        ('CL', field('cl')),
        ('CD', field('cd')),
        ('q (Pa)', field('q_pa')),
        ('S (m²)', field('aero_reference_area_m2')),
        ('Drag (N)', field('drag_n')),
    ))
    for status in dict.fromkeys(row['pitch_attitude_status'] for row in checkpoints):
        print('Phase 2 pitch attitude: ' + status + (' for the AoA calculation' if status == 'USER-PRESCRIBED' else ''))
    for status in dict.fromkeys(row['wing_alpha_status'] for row in checkpoints):
        print('Wing AoA status: ' + status)
    for source in dict.fromkeys(row['wing_alpha_source'] for row in checkpoints if row['wing_alpha_source']):
        print('Wing AoA method: ' + source)
    if nasa:
        for key, label in (
            ('rotor_flow_status', 'Rotor inputs for the NASA wing AoA correction'),
            ('rotor_flow_correction_status', 'Rotor-flow correction'),
            ('rotor_flow_input_note', 'Rotor input note'),
            ('wing_pylon_lookup_status', 'Wing-pylon coefficient lookup'),
        ):
            for value in dict.fromkeys(row.get(key) for row in checkpoints if row.get(key)):
                print(f'{label}: {value}')
    for source in dict.fromkeys(row['aero_source'] for row in checkpoints):
        print('Wing coefficient source: ' + source)
    if nasa:
        print('The NASA p. 25 correction applies to freestream AoA. Wake-immersed wing loads are outside this model.')
    print('Tilt is the nacelle direction. L/R shows both sides when they differ.')
    print('Lift is wing lift magnitude; net upward force uses the modeled vertical components.')
    for status in dict.fromkeys(row['aero_status'] for row in checkpoints):
        print('Wing model status: ' + status)
    for reason in dict.fromkeys(row['aero_reason'] for row in checkpoints if row['aero_reason']):
        print('Wing force limitation: ' + reason)


def report_rotor_flow_inputs(row):
    """Report the last evaluated rotor-flow inputs and their provenance."""
    if 'body_rates_status' not in row:
        if row.get('rotor_flow_assumptions'):
            print('Rotor-flow assumptions — ' + row['rotor_flow_assumptions'])
        return
    print(f'\nROTOR-FLOW INPUTS AT t = {row["phase2_time_s"]:g} s')

    def show(label, key, unit, places=3, *, status_key=None, source_key=None):
        raw = row.get(key)
        value = '—'
        if raw is not None:
            value = f'{raw:.6g} {unit}' if unit == 'dimensionless' else format_quantity(raw, unit, places=places)
        status = row[status_key or key + '_status']
        source = row[source_key or key + '_source']
        print(f'  {label}  {value} | {status} | {source}')

    show('Heading', 'heading_deg', 'deg true', status_key='heading_status', source_key='heading_source')
    for name in ('u', 'v', 'w'):
        show(f'Body {name.upper()}', f'body_{name}_mps', 'm/s',
             status_key=f'body_{name}_status', source_key=f'body_{name}_source')
    for name in ('p', 'q', 'r'):
        show(f'Body {name}', f'body_{name}_rad_s', 'rad/s',
             status_key=f'body_{name}_status', source_key=f'body_{name}_source')
    show('Rotor radius', 'rotor_radius_m', 'm', status_key='rotor_radius_status',
         source_key='rotor_geometry_source')
    show('Blade count per rotor', 'rotor_blade_count', 'blades', places=0)
    show('Blade chord', 'rotor_blade_chord_m', 'm', places=4, status_key='rotor_blade_chord_status',
         source_key='rotor_blade_chord_source')
    show('Geometric solidity σ', 'rotor_solidity', 'dimensionless')
    show('Pitch-flap coupling δ3', 'rotor_delta3_deg', 'deg', status_key='rotor_delta3_status',
         source_key='rotor_delta3_source')
    show('Lateral mast tilt', 'lateral_mast_tilt_deg', 'deg', status_key='lateral_mast_tilt_status',
         source_key='lateral_mast_tilt_source')
    geometry = f'  CG-to-hub geometry | {row["hub_geometry_status"]}'
    if row['hub_geometry_source']:
        geometry += f' | {row["hub_geometry_source"]}'
    print(geometry)
    if row['hub_geometry_note']:
        print('  ' + row['hub_geometry_note'])
    if row['hub_geometry_xml_fields_inspected']:
        print('  XML fields inspected — ' + row['hub_geometry_xml_fields_inspected'])
    for side in ('left', 'right'):
        offset = tuple(row[f'{side}_cg_to_hub_{axis}_m'] for axis in ('x', 'y', 'z'))
        if None not in offset:
            print(f'  {side.title()} CG-to-hub (forward/right/down)  {format_vector(offset, "m")}'
                  f' | {row[f"{side}_cg_to_hub_status"]} | {row[f"{side}_cg_to_hub_source"]}')
        show(f'{side.title()} mast angle β', f'{side}_beta_m_deg', 'deg',
             status_key=f'{side}_beta_m_status', source_key=f'{side}_beta_m_source')
        show(f'{side.title()} rotor speed', f'{side}_rotor_rpm', 'rpm', places=0)
        show(f'{side.title()} shaft Ω', f'{side}_rotor_omega_rad_s', 'rad/s', places=4,
             status_key=f'{side}_rotor_omega_status', source_key=f'{side}_rotor_omega_source')
        show(f'{side.title()} effective Ω′', f'{side}_rotor_omega_effective_rad_s', 'rad/s', places=4,
             status_key=f'{side}_rotor_speed_status', source_key=f'{side}_rotor_omega_effective_source')
        for frame, status_key in (('body', 'hub_velocity_body'), ('mast', 'hub_velocity')):
            values = tuple(row[f'{side}_hub_{axis}_{frame}_mps'] for axis in ('u', 'v', 'w'))
            velocity = '—' if None in values else format_vector(values, 'm/s')
            print(f'  {side.title()} hub U/V/W ({frame})  {velocity} | {row[f"{side}_{status_key}_status"]}'
                  f' | {row[f"{side}_{status_key}_source"]}')
        show(f'{side.title()} wind azimuth ξ', f'{side}_wind_azimuth_rad', 'rad',
             status_key=f'{side}_wind_azimuth_status', source_key=f'{side}_wind_azimuth_source')
        if row[f'{side}_wind_azimuth_reason']:
            print('    ' + row[f'{side}_wind_azimuth_reason'])
        show(f'{side.title()} axial hub inflow λ₀', f'{side}_in_plane_lambda_zero', 'dimensionless',
             status_key=f'{side}_in_plane_lambda_zero_status', source_key=f'{side}_in_plane_lambda_zero_source')
        for name in ('h', 'y'):
            key = f'{side}_rotor_{name}_bar_n'
            show(f'{side.title()} wind-mast {name.upper()}-bar', key, 'N',
                 status_key=f'{key}_status', source_key=f'{key}_source')
        for name in ('t', 'h', 'y'):
            show(f'{side.title()} mast {name.upper()}', f'{side}_rotor_{name}_n', 'N',
                 status_key=f'{side}_rotor_{name}_status', source_key=f'{side}_rotor_{name}_source')
        show(f'{side.title()} μ', f'mu_{side}', 'dimensionless',
             status_key=f'{side}_advance_ratio_status', source_key=f'{side}_advance_ratio_source')
        show(f'{side.title()} C_RF', f'c_rf_{side}', 'dimensionless',
             status_key=f'{side}_force_coefficient_status', source_key=f'{side}_force_coefficient_source')
        if row[f'{side}_in_plane_state_assumptions']:
            print(f'  {side.title()} operating inputs — {row[f"{side}_in_plane_state_assumptions"]}')
        if row[f'{side}_nasa_in_plane_missing_inputs']:
            print(f'  {side.title()} NASA in-plane inputs unavailable — {row[f"{side}_nasa_in_plane_missing_inputs"]}')
    # Raw external providers can document assumptions beyond the built-in ones shown above.
    if row.get('rotor_flow_assumptions') and row['body_rates_status'] != 'ASSUMED':
        print('Rotor-flow assumptions — ' + row['rotor_flow_assumptions'])


def report_inputs_and_assumptions(transition):
    print('\nINPUTS AND ASSUMPTIONS')
    row = transition['rows'][-1] if transition['rows'] else transition['preflight_audit']
    for field, key, value_key, unit in (
        ('Air density', 'air_density', 'air_density_kg_m3', 'kg/m³'),
        ('Wind speed', 'wind_speed', 'wind_speed_mps', 'm/s'),
        ('Wind FROM', 'wind_from', 'wind_from_deg', 'deg true'),
        ('Temperature', 'temperature', 'temperature_c', '°C'),
    ):
        value = format_quantity(row[value_key], unit, places=3)
        print(f'  {field}  {value} | {row[f"{key}_status"]} | {row[f"{key}_source"]}')
    for key, label, value in (
        ('rotor_model', 'Rotor model', row['rotor_model']),
        ('timestep', 'Integration timestep', f'{row["timestep_s"]:g} s' if 'timestep_s' in row else None),
        ('transition_end', 'Nacelle targets L/R', row.get('transition_end_deg')),
        ('transition_flaps', 'Wing flap/flaperon', row.get('transition_flaps_deg')),
        ('collective_config', 'Collective configuration', row.get('left_rotor_configuration_source')),
    ):
        if row.get(f'{key}_status') == 'DEFAULTED' and value is not None:
            print(f'  {label}  {value} | DEFAULTED | {row[f"{key}_source"]}')
    pitch = row['pitch_attitude_deg']
    print('  Phase 2 pitch  ' + ('— | UNAVAILABLE | --transition-pitch-deg'
                                 if pitch is None else f'{pitch:g} deg | USER-PRESCRIBED | --transition-pitch-deg'))
    for side in ('left', 'right'):
        ct = row[f'{side}_thrust_coefficient']
        value = '—' if ct is None else format(ct, '.6g')
        print(f'  {side.title()} selected C_T  {value} | {row[f"{side}_ct_status"]}'
              f' | {row[f"{side}_ct_data_kind"]} | {row[f"{side}_ct_source"] or row[f"{side}_ct_reason"]}')
        if ct is None and row[f'{side}_ct_source']:
            print('    ' + row[f'{side}_ct_reason'])
        lower = row[f'{side}_ct_pitch_min_deg']
        if lower is not None:
            print(f'    Model bounds {lower:g} to {row[f"{side}_ct_pitch_max_deg"]:g} deg'
                  f' | {row[f"{side}_ct_pitch_bounds_basis"]} | {row[f"{side}_ct_pitch_bounds_source"]}')
    print('  Selected C_T application — ' + row['selected_ct_application'])
    print(f'  CR-177436 hover C_T | {row["cr177436_hover_ct_status"]} | {row["cr177436_hover_ct_source"]}')
    print('  ' + row['cr177436_hover_ct_coverage'])
    print('  ' + row['cr177436_hover_ct_limitation'])
    print('  ' + row['cr177436_hover_ct_data_kind'])
    if transition['rows']:
        report_rotor_flow_inputs(row)
        if row.get('rotor_flow_missing_inputs'):
            print('  Unavailable rotor inputs or results — ' + row['rotor_flow_missing_inputs'])
    else:
        print(f'  Heading  {row["heading_deg"]:g} deg true | {row.get("heading_status", "PRESCRIBED")}'
              f' | {row.get("heading_source", "TMPE Phase 2 fixed heading")}')
        speed = format_quantity(row['left_rotor_rpm'], 'rpm', places=0)
        print(f'  Accepted rotor-speed input for both rotors  {speed} | {row["left_rotor_rpm_source"]}')
        print('  Phase 2 rotor flow | NOT EVALUATED | ' + transition['reason'])


def write_transition_audit(rows, audit_path):
    # Export-only ordering and rounding; retain full-precision simulation rows.
    numeric_groups = (
        ('audit_record', 'phase2_time_s', 'simulation_time_s', 'height_agl_m', 'forward_distance_m',
         'forward_velocity_mps', 'vertical_velocity_mps', 'mass_kg', 'aircraft_weight_n'),
        ('left_nacelle_deg', 'right_nacelle_deg', 'left_thrust_n', 'right_thrust_n',
         'left_collective_input_deg', 'right_collective_input_deg', 'left_thrust_coefficient',
         'right_thrust_coefficient', 'left_cr177436_hover_ct', 'right_cr177436_hover_ct',
         'left_cr177436_hover_thrust_n', 'right_cr177436_hover_thrust_n',
         'left_rotor_forward_n', 'left_rotor_up_n', 'right_rotor_forward_n', 'right_rotor_up_n',
         'total_rotor_forward_n', 'total_rotor_up_n'),
        ('wind_speed_mps', 'wind_from_deg', 'wind_forward_mps', 'wind_east_mps', 'wind_cross_plane_mps',
         'air_forward_mps', 'air_up_mps', 'airspeed_mps', 'air_density_kg_m3', 'temperature_c',
         'air_speed_of_sound_mps', 'rotor_cg_airspeed_mps'),
        ('heading_deg', 'pitch_attitude_deg', 'body_u_mps', 'body_v_mps', 'body_w_mps',
         'body_p_rad_s', 'body_q_rad_s', 'body_r_rad_s',
         'beta_m_deg', 'lateral_mast_tilt_deg', 'lxh_m', 'lyh_m', 'lzh_m', 'rotor_radius_m',
         'rotor_blade_count', 'rotor_blade_chord_m', 'rotor_solidity', 'rotor_delta3_deg', 'rotor_delta3_tan'),
        ('left_beta_m_deg', 'left_cg_to_hub_x_m', 'left_cg_to_hub_y_m', 'left_cg_to_hub_z_m',
         'left_hub_u_body_mps', 'left_hub_v_body_mps', 'left_hub_w_body_mps',
         'left_hub_u_mast_mps', 'left_hub_v_mast_mps', 'left_hub_w_mast_mps',
         'left_rotor_rpm', 'left_rotor_omega_rad_s', 'left_rotor_omega_effective_rad_s',
         'left_rotor_shaft_tip_speed_mps', 'left_in_plane_effective_tip_speed_mps', 'left_rotor_tip_mach',
         'left_in_plane_dn_n', 'left_in_plane_q6_n', 'left_in_plane_induced_iteration_count',
         'left_wind_azimuth_rad', 'left_wind_mast_p_hat', 'left_wind_mast_q_hat', 'left_in_plane_lambda_zero',
         'left_rotor_h_bar_n', 'left_rotor_y_bar_n', 'left_rotor_t_n', 'left_rotor_h_n', 'left_rotor_y_n', 'c_rf_left', 'mu_left'),
        ('right_beta_m_deg', 'right_cg_to_hub_x_m', 'right_cg_to_hub_y_m', 'right_cg_to_hub_z_m',
         'right_hub_u_body_mps', 'right_hub_v_body_mps', 'right_hub_w_body_mps',
         'right_hub_u_mast_mps', 'right_hub_v_mast_mps', 'right_hub_w_mast_mps',
         'right_rotor_rpm', 'right_rotor_omega_rad_s', 'right_rotor_omega_effective_rad_s',
         'right_rotor_shaft_tip_speed_mps', 'right_in_plane_effective_tip_speed_mps', 'right_rotor_tip_mach',
         'right_in_plane_dn_n', 'right_in_plane_q6_n', 'right_in_plane_induced_iteration_count',
         'right_wind_azimuth_rad', 'right_wind_mast_p_hat', 'right_wind_mast_q_hat', 'right_in_plane_lambda_zero',
         'right_rotor_h_bar_n', 'right_rotor_y_bar_n', 'right_rotor_t_n', 'right_rotor_h_n', 'right_rotor_y_n', 'c_rf_right', 'mu_right'),
        ('rotor_advance_ratio_mean', 'rotor_advance_ratio_for_correction', 'rotor_wing_induction',
         'wing_area_m2', 'wing_incidence_deg', 'gamma_deg', 'fuselage_alpha_rad', 'fuselage_alpha_deg',
         'rotor_flow_correction_rad', 'rotor_flow_correction_deg',
         'wing_alpha_rad', 'wing_alpha_deg', 'alpha_deg',
         'table_mast_deg',
         'flap_deg', 'flaperon_deg', 'mach', 'q_pa', 'aero_reference_area_m2',
         'cl', 'cd', 'lift_n', 'drag_n', 'wing_forward_n', 'wing_up_n'),
        ('net_forward_n', 'net_up_n', 'forward_acceleration_mps2', 'vertical_acceleration_mps2'),
    )
    fields = list(dict.fromkeys(key for row in rows for key in row))
    ordered = [key for group in numeric_groups for key in group if key in fields]
    # Preserve every status/source/validation field, including future additions.
    fieldnames = ordered + [key for key in fields if key not in ordered]
    output_path = None
    try:
        if audit_path is None:
            output_dir = Path('output')
            output_dir.mkdir(exist_ok=True)
            stream = tempfile.NamedTemporaryFile(mode='w', newline='', encoding='utf-8',
                dir=output_dir, prefix='stage2_audit_', suffix='.csv', delete=False)
        else:
            stream = audit_path.open('x', newline='', encoding='utf-8')
        output_path = Path(stream.name)
        with stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(
                {key: format(value, '.4g') if isinstance(value, (int, float))
                 and not isinstance(value, bool) else value
                 for key, value in row.items()}
                for row in rows)
        print(f'Stage 2 audit CSV ({len(rows)} rows): {output_path}')
    except OSError as exc:
        print(f'Stage 2 audit CSV UNAVAILABLE — {exc.filename or output_path or audit_path or Path("output")}'
              f' — {exc.strerror or type(exc).__name__}')
    return output_path


def write_transition_report(path, fieldnames, rows):
    try:
        with path.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                values = {key: row.get(key) for key in fieldnames}
                # Keep exact time keys so the three reports can be joined on time.
                writer.writerow({key: format(value, '.8g') if isinstance(value, float) and not key.endswith('_time_s')
                                 else value for key, value in values.items()})
        print(f'Stage 2 report CSV ({len(rows)} rows): {path}')
    except OSError as exc:
        print(f'Stage 2 report CSV UNAVAILABLE — {path} — {exc.strerror or type(exc).__name__}')


def report_transition_csvs(result, checkpoints, audit_path):
    rows = result['rows']
    if not rows and 'preflight_audit' not in result:
        return
    output_path = write_transition_audit(rows or [result['preflight_audit']], audit_path)
    run_path = output_path or audit_path
    report_parent = run_path.parent if run_path is not None else Path('output')
    try:
        if output_path is None:
            # A failed audit open can leave the requested name belonging to an earlier run.
            report_dir = Path(tempfile.mkdtemp(dir=report_parent, prefix='stage2_reports_'))
        else:
            report_dir = output_path.with_name(f'{output_path.stem}_reports')
            report_dir.mkdir()
    except OSError as exc:
        print(f'Stage 2 report CSVs UNAVAILABLE — {exc.filename or report_parent}'
              f' — {exc.strerror or type(exc).__name__}')
        return

    checkpoint_fields = (
        'phase2_time_s', 'simulation_time_s', 'height_agl_m', 'forward_distance_m',
        'forward_velocity_mps', 'vertical_velocity_mps', 'left_nacelle_deg', 'right_nacelle_deg',
        'airspeed_mps', 'left_thrust_n', 'right_thrust_n', 'total_rotor_forward_n', 'total_rotor_up_n',
        'lift_n', 'drag_n',
        'wing_forward_n', 'wing_up_n',
        'aircraft_weight_n', 'net_forward_n', 'net_up_n',
        'forward_acceleration_mps2', 'vertical_acceleration_mps2',
        'phase2_status', 'phase2_reason', 'aero_status', 'aero_reason',
        'trajectory_status', 'trajectory_source', 'phase2_requested_mode', 'phase2_motion_mode',
        'phase2_force_balance_role', 'force_balance_status', 'first_force_blocker', 'force_balance_reason',
        'phase2_start_height_source', 'phase2_start_velocity_source', 'phase2_start_nacelle_source',
        'longitudinal_3dof_status', 'aircraft_transition_status',
    )
    companion_rows = []
    for row in checkpoints:
        aero_reason = compact_aero_reason(row)
        if row is rows[-1]:
            status, reason = result['status'], result['reason']
            if aero_reason != row['aero_reason']:
                reason = reason.replace(row['aero_reason'], aero_reason, 1)
        else:
            status = 'PRESCRIBED' if result.get('prescribed') else row['force_balance_status']
            reason = '' if result.get('prescribed') else aero_reason
        balance_reason = row['force_balance_reason']
        if row['aero_reason'] and aero_reason != row['aero_reason']:
            balance_reason = balance_reason.replace(row['aero_reason'], aero_reason, 1)
        companion_rows.append(dict(row, phase2_status=status, phase2_reason=reason,
                                   aero_reason=aero_reason, force_balance_reason=balance_reason,
                                   requested_mast_deg=row.get('beta_m_deg'),
                                   requested_mast_status=row.get('beta_m_status')))
    write_transition_report(report_dir / 'stage2_checkpoints.csv', checkpoint_fields, companion_rows)

    rotor_columns = (
        ('nacelle_deg', 'nacelle_deg'), ('mast_angle_deg', 'beta_m_deg'),
        ('rotor_rpm', 'rotor_rpm'), ('rotor_rpm_status', 'rotor_rpm_status'),
        ('rotor_rpm_source', 'rotor_rpm_source'),
        ('effective_tip_speed_mps', 'in_plane_effective_tip_speed_mps'),
        ('tip_mach', 'rotor_tip_mach'), ('tip_mach_status', 'rotor_tip_mach_status'),
        ('axial_inflow_ratio', 'in_plane_lambda_zero'), ('wind_azimuth_rad', 'wind_azimuth_rad'),
        ('dn_n', 'in_plane_dn_n'), ('q6_n', 'in_plane_q6_n'),
        ('induced_inflow_ratio', 'in_plane_induced_inflow'),
        ('induced_velocity_mps', 'in_plane_induced_velocity_mps'),
        ('induced_inflow_iterations', 'in_plane_induced_iteration_count'),
        ('induced_inflow_status', 'in_plane_induced_convergence_status'),
        ('induced_inflow_reason', 'in_plane_induced_convergence_reason'),
        ('wind_mast_h_bar_n', 'rotor_h_bar_n'), ('h_bar_status', 'rotor_h_bar_n_status'),
        ('h_bar_source', 'rotor_h_bar_n_source'), ('h_bar_reason', 'rotor_h_bar_n_reason'),
        ('wind_mast_y_bar_n', 'rotor_y_bar_n'), ('y_bar_status', 'rotor_y_bar_n_status'),
        ('y_bar_source', 'rotor_y_bar_n_source'), ('y_bar_reason', 'rotor_y_bar_n_reason'),
        ('mast_t_n', 'rotor_t_n'), ('mast_t_status', 'rotor_t_status'), ('mast_t_source', 'rotor_t_source'),
        ('mast_h_n', 'rotor_h_n'), ('mast_h_status', 'rotor_h_status'), ('mast_h_source', 'rotor_h_source'),
        ('mast_h_reason', 'rotor_h_reason'),
        ('mast_y_n', 'rotor_y_n'), ('mast_y_status', 'rotor_y_status'), ('mast_y_source', 'rotor_y_source'),
        ('mast_y_reason', 'rotor_y_reason'),
        ('rotor_force_status', 'rotor_force_status'),
        ('held_phase1_thrust_n', 'thrust_n'), ('held_thrust_status', 'rotor_status'),
        ('projected_forward_n', 'rotor_forward_n'), ('projected_up_n', 'rotor_up_n'),
        ('diagnostic_body_forward_force_n', 'rotor_body_forward_force_n'),
        ('diagnostic_body_forward_force_status', 'rotor_body_forward_force_n_status'),
        ('diagnostic_body_forward_force_source', 'rotor_body_forward_force_n_source'),
        ('diagnostic_body_forward_force_reason', 'rotor_body_forward_force_n_reason'),
        ('diagnostic_body_down_force_n', 'rotor_body_down_force_n'),
        ('diagnostic_body_down_force_status', 'rotor_body_down_force_n_status'),
        ('diagnostic_body_down_force_source', 'rotor_body_down_force_n_source'),
        ('diagnostic_body_down_force_reason', 'rotor_body_down_force_n_reason'),
        ('diagnostic_pitch_arm_moment_nm', 'rotor_cg_pitch_arm_moment_nm'),
        ('diagnostic_pitch_arm_moment_status', 'rotor_cg_pitch_arm_moment_nm_status'),
        ('diagnostic_pitch_arm_moment_source', 'rotor_cg_pitch_arm_moment_nm_source'),
        ('diagnostic_pitch_arm_moment_reason', 'rotor_cg_pitch_arm_moment_nm_reason'),
        ('phase1_ct', 'thrust_coefficient'), ('phase1_ct_status', 'ct_status'), ('phase1_ct_source', 'ct_source'),
        ('phase1_calibration', 'rotor_calibration'),
        ('in_plane_assumptions', 'in_plane_state_assumptions'),
    )
    rotor_fields = ('phase2_time_s', 'rotor_side', 'rotor_model', 'trajectory_status', 'trajectory_source',
                    *(name for name, _ in rotor_columns),
                    'advance_ratio', 'advance_ratio_status', 'advance_ratio_source',
                    'c_rf', 'c_rf_status', 'c_rf_source', 'c_rf_reason', 'rotor_flow_source',
                    'rotor_flow_assumptions', 'first_missing_in_plane_input', 'unresolved_in_plane_inputs',
                    'rotor_flow_input_note')
    rotor_rows = []
    for row in checkpoints:
        for side in ('left', 'right'):
            selected = {name: row.get(f'{side}_{key}') for name, key in rotor_columns}
            missing_in_plane = row.get(f'{side}_nasa_in_plane_missing_inputs', '')
            first_missing_input = (missing_in_plane.partition('; ')[0]
                                   if None in (selected['mast_h_n'], selected['mast_y_n']) else '')
            rotor_rows.append(dict(selected, phase2_time_s=row['phase2_time_s'], rotor_side=side,
                                   rotor_model=row['rotor_model'], trajectory_status=row['trajectory_status'],
                                   trajectory_source=row['trajectory_source'], advance_ratio=row.get(f'mu_{side}'),
                                   advance_ratio_status=row.get(f'{side}_advance_ratio_status'),
                                   advance_ratio_source=row.get(f'{side}_advance_ratio_source'),
                                   c_rf=row.get(f'c_rf_{side}'), c_rf_status=row.get(f'{side}_force_coefficient_status'),
                                   c_rf_source=row.get(f'{side}_force_coefficient_source'),
                                   c_rf_reason=row.get(f'{side}_force_coefficient_reason'),
                                   rotor_flow_source=row.get('rotor_flow_source'),
                                   rotor_flow_assumptions=row.get('rotor_flow_assumptions'),
                                   first_missing_in_plane_input=first_missing_input,
                                   unresolved_in_plane_inputs=missing_in_plane,
                                   rotor_flow_input_note=row.get('rotor_flow_input_note')))
    write_transition_report(report_dir / 'stage2_rotors.csv', rotor_fields, rotor_rows)

    aero_fields = (
        'phase2_time_s', 'trajectory_status', 'trajectory_source',
        'air_forward_mps', 'air_up_mps', 'airspeed_mps', 'air_relative_velocity_status',
        'air_density_kg_m3', 'air_density_status', 'pitch_attitude_deg', 'pitch_attitude_status',
        'gamma_deg', 'gamma_status', 'fuselage_alpha_deg', 'fuselage_alpha_status',
        'rotor_flow_correction_deg', 'rotor_flow_correction_status',
        'wing_alpha_deg', 'wing_alpha_status', 'requested_mast_deg', 'requested_mast_status', 'table_mast_deg',
        'flap_deg', 'flaperon_deg', 'transition_flaps_status', 'mach',
        'cl', 'cd', 'q_pa', 'aero_reference_area_m2', 'lift_n', 'drag_n',
        'wing_forward_n', 'wing_up_n', 'wing_pylon_lookup_status',
        'aero_status', 'aero_reason', 'aero_source', 'aero_user_source', 'aero_validation',
    )
    write_transition_report(report_dir / 'stage2_aero.csv', aero_fields, companion_rows)


def report_final_state(transition, takeoff):
    state = transition['final_state'] if transition['elapsed_s'] > 0 else initialize_transition(takeoff)
    if transition['elapsed_s'] > 0 and transition.get('prescribed'):
        phase = 'PHASE 2 (PRESCRIBED KINEMATIC DEMONSTRATION)'
    elif transition['elapsed_s'] > 0:
        phase = 'PHASE 2 (ILLUSTRATIVE DEMONSTRATION)'
    elif takeoff['elapsed_s'] > 0:
        phase = 'PHASE 1 (RETAINED STATE)'
    else:
        phase = 'PHASE 1 (INITIAL STATE RETAINED)'
    print(f'\nFINAL STATE — {phase}')
    values = (state.height_agl_m, state.forward_distance_m, state.forward_velocity_mps,
              state.vertical_velocity_mps)
    print_report_table(('AGL (m)', 'Distance (m)', 'Fwd (m/s)', 'Up (m/s)'),
                       [tuple(map(report_cell, values))])


def sample_nacelle_transition(takeoff_result, angle_pairs):
    """Untimed geometry samples with held Phase 1 thrust; never flight predictions.

    No inflow, RPM, collective or airspeed correction is inferred while tilting.
    Each pair is (left degrees, right degrees); no conversion rate is implied.
    """
    state = initialize_transition(takeoff_result)
    samples = []
    for left_angle, right_angle in angle_pairs:
        validate_transition_angle(left_angle)
        validate_transition_angle(right_angle)
        left = rotor_force_body_n(state.left_proprotor_thrust_n, left_angle)
        right = rotor_force_body_n(state.right_proprotor_thrust_n, right_angle)
        samples.append(dict(left_nacelle_angle_deg=left_angle, right_nacelle_angle_deg=right_angle,
                            left_force_body_n=left, right_force_body_n=right,
                            forward_rotor_force_n=None if left is None or right is None else left[0] + right[0],
                            upward_rotor_force_n=None if left is None or right is None else -left[2] - right[2],
                            wing_lift_status='UNAVAILABLE', drag_status='UNAVAILABLE'))
    mass = takeoff_result['takeoff_mass_kg']
    return dict(starting_state=state, final_state=state, samples=samples,
                aircraft_weight_n=None if mass is None else mass * G,
                trajectory_status='BLOCKED', rotor_model=takeoff_result['rotor_model'])


def report_transition(result):
    print('\nOPTIONAL NACELLE GEOMETRY SAMPLES')
    state = result['starting_state']
    print('\n--- Initial aircraft state ---')
    for label, value, unit in (
        ('STARTING ALTITUDE', state.height_agl_m, 'm'),
        ('STARTING VERTICAL SPEED', state.vertical_velocity_mps, 'm/s'),
        ('STARTING FORWARD SPEED', state.forward_velocity_mps, 'm/s'),
        ('ELAPSED SIMULATION TIME', state.elapsed_s, 's'),
    ):
        print(f'{label:<23}: {format_quantity(value, unit)}')
    for sample in result['samples']:
        print('\n--- Transition configuration ---')
        for label, key in (('LEFT NACELLE ANGLE', 'left_nacelle_angle_deg'),
                           ('RIGHT NACELLE ANGLE', 'right_nacelle_angle_deg')):
            print(f'{label:<23}: {format_number(sample[key])}°')
        print('\n--- Rotor performance ---')
        for label, value in (
            ('LEFT PROPROTOR', state.left_proprotor_thrust_n),
            ('RIGHT PROPROTOR', state.right_proprotor_thrust_n),
            ('TOTAL UPWARD THRUST', sample['upward_rotor_force_n']),
            ('TOTAL FORWARD THRUST', sample['forward_rotor_force_n']),
        ):
            print(f'{label:<23}: {format_quantity(value, "N")}')
        print('\n--- Aircraft response ---')
        print(f'{"AIRCRAFT WEIGHT":<23}: {format_quantity(result["aircraft_weight_n"], "N")}')
        for label, status in (
            ('WING LIFT', sample['wing_lift_status']),
            ('AERODYNAMIC DRAG', sample['drag_status']),
            ('NET FORWARD FORCE', 'UNAVAILABLE'),
            ('NET VERTICAL FORCE', 'UNAVAILABLE'),
            ('FORWARD ACCELERATION', 'UNAVAILABLE'),
            ('VERTICAL ACCELERATION', 'UNAVAILABLE'),
        ):
            print(f'{label:<23}: {status}')
        print('\n--- Simulation status ---')
        print(f'{"TRANSITION STATUS":<23}: FORCE DECOMPOSITION ONLY')
        print(f'{"TRAJECTORY":<23}: NOT SIMULATED')
        print(f'{"SIMULATED TIME":<23}: {format_quantity(state.elapsed_s, "s")}')
    print('\n--- Model limitations ---')
    if result['rotor_model'] == 'prototype-collective':
        print('Collective-to-thrust model: see the selected Phase 1 calibration status for each rotor.')
    elif result['rotor_model'] == 'prototype-fixed-ct':
        print('Fixed-CT model: illustrative assumption; not validated XV-15 performance.')
    else:
        print('Rotor model: see Phase 1 force availability and calibration status.')
    print('Final rotor thrust held fixed during angle sampling; level aircraft assumed.')
    print('Wing lift and drag unavailable, not zero; net forces and accelerations unavailable.')
    print('Aerodynamic effects excluded: inflow corrections, rotor/wing interference and other airframe loads.')
    print('Provisional angle range: 0–90°; untimed samples are not a validated conversion procedure.')
    print('These optional angle samples advance neither time nor motion.')


def report_takeoff(result):
    balance, motion = result['balance'], result['motion']
    print('\n--- Rotor performance ---')
    for label, value in (('LEFT PROPROTOR', result['left']['value']),
                         ('RIGHT PROPROTOR', result['right']['value']),
                         ('TOTAL VERTICAL', balance.get('total_rotor_vertical_force_n'))):
        print(f'{label:<23}: {format_quantity(value, "N", reason="rotor model inputs missing")}')
    print('\n--- Aircraft response (up positive) ---')
    for label, value, unit in (
        ('TAKEOFF MASS', result['takeoff_mass_kg'], 'kg'),
        ('AIRCRAFT WEIGHT', balance.get('aircraft_weight_n'), 'N'),
        ('NET VERTICAL FORCE', balance.get('net_vertical_force_n'), 'N'),
        ('VERTICAL ACCEL.', motion['vertical_acceleration_mps2'], 'm/s²'),
    ):
        print(f'{label:<23}: {format_quantity(value, unit, reason="force balance unavailable")}')
    if result.get('rotor_model') == 'prototype-fixed-ct' and balance.get('net_vertical_force_n') is not None:
        margin_percent = 100 * balance['net_vertical_force_n'] / balance['aircraft_weight_n']
        print(f'Prototype force excess / weight: {margin_percent:.3f}% (excluded-load assumption).')
        if abs(margin_percent) < 1:
            print('Almost weight-balanced; no substantial takeoff margin. Any motion is a simplified mechanics result.')
    print(f"TAKEOFF STATUS         : {motion['takeoff_status']}")
    print(f"Phase 1 force calculation: {motion['status']}")


def report_xv15_calculations(aircraft, takeoff):
    """Reference calculations for the design configuration, independent of the dynamics."""
    def show(label, value, unit, *, status='CALCULATED', missing='required input missing or invalid'):
        if value is None:
            print(f'  {label}: UNAVAILABLE — {missing}')
        else:
            places = 4 if unit == 'dimensionless' else 2
            print(f'  {label}: {status} — {format_quantity(value, unit, places)}')

    print('\nXV-15 DESIGN-CONFIGURATION CALCULATIONS')
    diameter_ft = aircraft.rotor_diameter_ft
    chord_ft = aircraft.blade_chord_ft
    blades = aircraft.rotor_blades_per_hub
    rpm = aircraft.hover_rotor_rpm
    if all(value is not None and value > 0 for value in (diameter_ft, blades, chord_ft, rpm)):
        print(f'  Provisional geometry inputs — {diameter_ft:g} ft diameter, {blades:g} blades per rotor, '
              f'{chord_ft * 12:g} in chord, {rpm:g} hover RPM.')
    radius_m = diameter_ft * .3048 / 2 if diameter_ft is not None and diameter_ft > 0 else None
    omega = math.tau * rpm / 60 if rpm is not None and rpm > 0 else None
    chord_m = chord_ft * .3048 if chord_ft is not None and chord_ft > 0 else None
    show('Disk area per rotor', None if radius_m is None else math.pi * radius_m**2, 'm²',
         missing='rotor diameter missing or invalid')
    show('Hover tip speed', None if radius_m is None or omega is None else omega * radius_m, 'm/s',
         missing='diameter or hover RPM missing or invalid')
    show('Geometric solidity', rotor_solidity(blades, chord_m, radius_m), 'dimensionless',
         missing='diameter, chord or blade count missing or invalid')

    weight = takeoff['balance'].get('aircraft_weight_n')
    download = aircraft.hover_download_fraction
    if weight is None or download is None or not 0 <= download < 1:
        show('Conditional hover thrust baseline', None, 'N', missing='weight or download fraction missing or invalid')
    else:
        total = weight / (1 - download)
        print(f'  Hover baseline — provisional {download:.1%} download, flaps deflected, equal rotor sharing.')
        show('Total rotor thrust required', total, 'N')
        show('Thrust required per rotor', total / 2, 'N')
        show('Downward wing download', total * download, 'N')
        print('  Thrust requirement from weight balance; this estimate does not use the run’s blade collective-pitch inputs.')

    power_limit = aircraft.hover_drive_engine_limit_shp
    if power_limit is None or power_limit <= 0 or omega is None:
        show('Hover shaft torque per rotor', None, 'N·m', missing='installed power reference or hover RPM missing or invalid')
    else:
        total_shp = .65 * (2 * power_limit)
        print(f'  Torque basis — provisional 65% of {2 * power_limit:,.0f} SHP total, treated as mast power with an equal split.')
        show('Hover shaft torque per rotor', total_shp * SHP_TO_W / (2 * omega), 'N·m', status='APPROXIMATE')

    angle = takeoff['nacelle_angle_deg']
    angle_display = 'UNAVAILABLE' if angle is None else f'{angle:g}°'
    print(f'  Phase 1 rotor components (forward/up) at {angle_display}, with 0° forward and 90° vertical.')
    projection_status = 'CALCULATED from selected rotor thrust; calibration shown in model status'
    vectors = [rotor_force_body_n(takeoff[side]['value'], angle) for side in ('left', 'right')]
    for side, vector in zip(('Left', 'Right'), vectors):
        if vector is None:
            print(f'  {side}: UNAVAILABLE — rotor thrust or nacelle angle missing')
        else:
            print(f'  {side}: {projection_status} — {format_number(vector[0])} / {format_number(-vector[2])} N')
    if any(vector is None for vector in vectors):
        print('  Total: UNAVAILABLE — both rotor projections required')
    else:
        forward = sum(vector[0] for vector in vectors)
        up = -sum(vector[2] for vector in vectors)
        print(f'  Total: {projection_status} — {format_number(forward)} / {format_number(up)} N')


def report_data_status(takeoff, transition, aircraft, polar):
    print('\nMODEL RESULTS AND DATA STATUS')
    print('XV-15 flight validation: NOT VALIDATED; calculated results alone do not establish validation.')
    rotors = [takeoff[side] for side in ('left', 'right')]
    model = takeoff['rotor_model']
    statuses = []
    for side, rotor in zip(('Left', 'Right'), rotors):
        value = rotor['value']
        provenance = rotor.get('source_status') or rotor.get('calibration_status') or ''
        status = 'ILLUSTRATIVE; software test only' if 'ILLUSTRATIVE' in provenance else rotor['status']
        statuses.append(status)
        if model == 'prototype-collective':
            print(f'{side} collective-pitch input: '
                  f'{format_quantity(rotor.get("theta_deg"), "deg", places=1)} — blade pitch, separate from nacelle direction.')
            print(f'{side} collective-to-thrust calibration: '
                  f'{rotor.get("calibration_status", "NOT REPORTED")}')
            print(f'  Experimental validation: {rotor.get("experimental_validation_status", "NOT REPORTED")}')
        if value is None:
            print(f'{side} rotor thrust: UNAVAILABLE | model: {model}')
            print(f"  {rotor['reason']}")
        else:
            print(f'{side} rotor thrust: {value:,.2f} N — {status} | model: {model}')

    if all(rotor['value'] is not None for rotor in rotors):
        total = sum(rotor['value'] for rotor in rotors)
        status = 'ILLUSTRATIVE; software test only' if any(label.startswith('ILLUSTRATIVE') for label in statuses) else 'CALCULATED'
        print(f'Total rotor thrust: {total:,.2f} N — {status} | model: {model}')
    else:
        print(f'Total rotor thrust: UNAVAILABLE — both rotor values are required | model: {model}')
    print('  Basis: sum of left and right rotor-axis magnitudes.')
    if model != 'prototype-collective':
        print('Collective-pitch input: NOT USED by the selected rotor model; separate left/right inputs are available in prototype-collective mode.')
    if takeoff['motion']['status'] != 'CALCULATED':
        print(f"Phase 1 blocker: {takeoff['balance']['reason']}")
        if takeoff['elapsed_s'] == 0:
            print('Phase 1 takeoff not simulated.')
    if polar is not None:
        print(f'Wing coefficient table: {polar.status} — {Path(polar.source).name}')
        if polar.user_source:
            print(f'  User-provided source metadata: {polar.user_source}')
    else:
        print('Wing coefficients: ILLUSTRATIVE — Phase 2 uses constant CL/CD assumptions, not an AoA-based XV-15 polar.')
    if transition['rows']:
        row = transition['rows'][-1]
        if row['wing_alpha_deg'] is not None:
            print(f'Wing AoA (last Phase 2 evaluation): {row["wing_alpha_status"]} — '
                  f'{format_quantity(row["wing_alpha_deg"], "deg")}')
        elif polar is None:
            print('Wing AoA (Phase 2): NOT CALCULATED by the constant-coefficient model.')
        else:
            print(f'Wing AoA (last Phase 2 evaluation): NOT CALCULATED — {row["aero_reason"]}')
    else:
        print('Wing AoA (Phase 2): NOT EVALUATED — no Phase 2 steps were evaluated.')
    if transition['status'] == 'UNAVAILABLE':
        print(f"Phase 2 blocker: {transition['reason']}")
    elif transition['rows'] and transition['rows'][-1]['aero_reason']:
        print(f"Last wing force evaluation: UNAVAILABLE — {transition['rows'][-1]['aero_reason']}")
    inputs = ('design_gross', 'nacelle_angle_deg', 'wing_area_ft2')
    if isinstance(polar, TransitionPolar):
        inputs += ('wing_incidence_deg',)
    for name in inputs:
        record = aircraft.diagnostics[name]
        if record['value'] is None:
            print(f"{name}: UNAVAILABLE — {record['file']} :: {record['path']} — {record['reason']}")
    print('\nAIRCRAFT DATA AND MODEL LIMITS')
    if model == 'prototype-collective':
        print('XV-15 collective-to-thrust calibration: see each rotor’s selected calibration above; blade-pitch input alone does not establish calibrated or validated XV-15 thrust.')
    else:
        print('XV-15 collective-to-thrust calibration: NOT USED by the selected rotor model.')
    print(f'XV-15 transition performance maps: MISSING — {AIRCRAFT_TRANSITION_BLOCKER}')
    print('Density coverage: full XV-15 rotor/wing performance across densities is not established; the run accepts density as a separate, constant input.')
    print('Wing force coverage: validated XV-15 forces across AoA and configurations are not established; calculated AoA alone does not supply coefficients.')
    print('Rotor mast power: UNAVAILABLE — commanded shaft power and power split are unknown.')
    print('Phase 1 wing loads, ground effect and wind response: NOT MODELED.')
    print('Phase 2 rotor inflow/RPM response, ground effect, wake-immersed wing loads, other airframe loads, '
          'power limits, lateral motion and attitude dynamics: NOT MODELED.')


def report_takeoff_diagnostics(result, verbose=False):
    print('\nTAKEOFF PHYSICS')
    for name in ('left', 'right', 'balance', 'motion'):
        row = result[name]
        print(f"  {name}: {row['status']} — {row['reason']}")
    if result.get('rotor_model') == 'prototype-fixed-ct':
        print('  Other vertical loads: EXCLUDED — ' + PROTOTYPE_EXCLUSIONS)
        print('  Rotor model: ' + PROTOTYPE_LABEL)
    elif result.get('rotor_model') == 'prototype-collective':
        print('  Other vertical loads: ' + COLLECTIVE_EXCLUSIONS)
    elif result['other_vertical_force_n'] is None:
        print('  Other vertical loads: UNAVAILABLE — ground effect, wing download and drag not established for ground liftoff.')
    else:
        print(f"  Other vertical loads: CALCULATED — explicit force provider: {result['other_vertical_force_n']} N")
    if result.get('rotor_model') == 'prototype-collective':
        print('  Rotor model: prototype-collective — local linear CT; experimental validation: NOT VALIDATED')
    elif result.get('rotor_model') != 'prototype-fixed-ct':
        print('  Rotor model: NOT IMPLEMENTED — supported CT/CQ or equivalent performance map required.')
    print('  Shaft power: UNAVAILABLE — engine rating is not commanded rotor mast power; power split and operating conditions unresolved.')
    print(f'  Nacelle convention: 90° vertical, 0° forward; NASA TM-81244 printed p. 4 / PDF p. 6; {NACELLE_SOURCE}')
    if verbose:
        print('  Selected loading: XML design_gross lb × 0.45359237 = mass kg; weight = mass × 9.80665 N.')
        print('  Thrust body vector = (T cos(beta), 0, -T sin(beta)); level aircraft, vertical translation only.')
        print('  F_net,up = T_L sin(beta_L) + T_R sin(beta_R) + F_other,up - m g.')
        print('  Ground support = max(0, -F_net,up) while grounded and stationary.')
        print('  Constant-force integration: h += v dt + 0.5 a dt²; v += a dt; clamp ground crossings.')
        print('  No OGE 7% download or reported 93% efficiency is silently applied to runway liftoff.')


def report_run_summary(transition, takeoff, end_angles):
    row = transition['rows'][0] if transition['rows'] else transition['preflight_audit']
    print('\nRUN SUMMARY')
    print('XV-15 helicopter-mode vertical takeoff and timed nacelle transition. Flight validation — UNVALIDATED.')
    print(f"Rotor model — {takeoff['rotor_model']} ({row['rotor_model_status']})")
    if takeoff['rotor_model'] == 'prototype-fixed-ct':
        print(PROTOTYPE_LABEL)
    print(f"Timestep — {row['timestep_s']:g} s ({row['timestep_status']}) | "
          f"Target nacelles — {end_angles[0]:g}° L / {end_angles[1]:g}° R ({row['transition_end_status']})")
    inputs = (
        ('Air density (kg/m³)', 'air_density_kg_m3', 'air_density_status'),
        ('Wind speed (m/s)', 'wind_speed_mps', 'wind_speed_status'),
        ('Wind FROM (° true)', 'wind_from_deg', 'wind_from_status'),
        ('Temperature (°C)', 'temperature_c', 'temperature_status'),
    )
    print_report_table(('Input', 'Value', 'Status'),
                       [(label, report_cell(row[key]), row[status]) for label, key, status in inputs])
    print('Blank table cells indicate unavailable values.')


def report_takeoff_summary(result, duration_s):
    balance, motion = result['balance'], result['motion']
    print('\nPHASE 1 — HELICOPTER TAKEOFF')
    print(f"Phase 1 elapsed: {result['elapsed_s']:.6g} / {duration_s:.6g} s")
    print(f"TAKEOFF STATUS         : {motion['takeoff_status']}")
    print(f"Retained state — {motion['height_agl_m']:.4g} m AGL, {motion['vertical_velocity_mps']:.4g} m/s up")
    print(f"Vertical force balance — {balance['status']}")
    values = (result['takeoff_mass_kg'], balance.get('aircraft_weight_n'),
              balance.get('total_rotor_vertical_force_n'), balance.get('net_vertical_force_n'),
              motion['vertical_acceleration_mps2'])
    print_report_table(('Mass (kg)', 'AIRCRAFT WEIGHT (N)', 'Rotor up (N)', 'Net up (N)', 'Accel up (m/s²)'),
                       [tuple(map(report_cell, values))])
    comparisons = []
    for side in ('left', 'right'):
        ct = result[side].get('thrust_coefficient')
        status = cr177436_hover_ct_comparison(ct)
        if status.startswith('ABOVE'):
            comparisons.append(f'{side} C_T={ct:g} ({status.lower()})')
    if comparisons:
        print('CR-177436 isolated metal-blade hover comparison — ' + ', '.join(comparisons))
        print(f'Reported alternating loads rose above C_T={HOVER_CT_ALTERNATING_LOAD_ONSET:g}. '
              f'Suggested hover maximum C_T≈{HOVER_CT_SUGGESTED_MAXIMUM:g} is specific to the hover evidence.')
    if motion['status'] != 'CALCULATED':
        print(f"Phase 1 blocker — {balance['reason']}.")
        if result['elapsed_s'] == 0:
            print('Phase 1 takeoff not simulated.')


def report_status_summary(takeoff, transition, polar):
    evaluated = transition['rows']
    row = evaluated[-1] if evaluated else transition['preflight_audit']
    print('\nROTOR STATUS')
    rotor_rows = []
    for side in ('left', 'right'):
        rotor = takeoff[side]
        provenance = rotor.get('source_status') or rotor.get('calibration_status') or ''
        basis = 'ILLUSTRATIVE' if 'ILLUSTRATIVE' in provenance else provenance
        rotor_rows.append((side.title(), report_cell(rotor['value']), rotor['status'],
                           report_cell(rotor.get('thrust_coefficient')), rotor.get('ct_status', 'UNAVAILABLE'), basis))
    print_report_table(('Rotor', 'Phase 1 T (N)', 'Thrust status', 'C_T', 'C_T status', 'Model basis'), rotor_rows)
    for side in ('left', 'right'):
        rotor = takeoff[side]
        if rotor['value'] is None:
            print(f"{side.title()} thrust blocker — {rotor['reason']}")
    if not evaluated:
        print('Phase 2 rotor flow — NOT EVALUATED')
    elif row.get('rotor_flow_status'):
        geometry = [('Radius (m)', 'rotor_radius_m'), ('Blades per rotor', 'rotor_blade_count'),
                    ('Solidity', 'rotor_solidity')]
        print('Rotor geometry')
        print_report_table([label for label, _ in geometry],
                           [tuple(report_cell(row.get(key)) for _, key in geometry)])
        print('Dynamic flow')
        print_report_table(('Rotor', 'RPM', 'Tip (m/s)', 'Tip Mach', 'μ', 'λ₀'), [
            (side.title(), *(report_cell(row.get(key)) for key in (
                f'{side}_rotor_rpm', f'{side}_in_plane_effective_tip_speed_mps', f'{side}_rotor_tip_mach',
                f'mu_{side}', f'{side}_in_plane_lambda_zero')))
            for side in ('left', 'right')])
        if all(row.get(f'{side}_rotor_rpm_status') == HELD_HOVER_RPM_STATUS for side in ('left', 'right')):
            print('RPM — XML-SOURCED / HELD FIXED (design hover input)')
        print(f"Rotor inputs for wing AoA correction — {row['rotor_flow_status']}")
        print('H/Y forces')
        print_report_table(('Rotor', 'H-bar (N)', 'Y-bar (N)', 'H (N)', 'Y (N)', 'C_RF', 'C_RF status'), [
            (side.title(), *(report_cell(row.get(key)) for key in (
                f'{side}_rotor_h_bar_n', f'{side}_rotor_y_bar_n', f'{side}_rotor_h_n', f'{side}_rotor_y_n',
                f'c_rf_{side}')), row.get(f'{side}_force_coefficient_status', ''))
            for side in ('left', 'right')])
        missing = [f'{side.title()} {force.upper()}' for side in ('left', 'right') for force in ('t', 'h', 'y')
                   if row.get(f'{side}_rotor_{force}_status') == 'UNAVAILABLE']
        if missing:
            print('Unresolved inputs — ' + ', '.join(missing) + ' unavailable. See the rotor CSV and wide audit.')
        if any(illustrative_rotor_status(row.get(f'{side}_rotor_{force}_status'))
               for side in ('left', 'right') for force in ('h', 'y')):
            print('Selected H/Y include illustrative or assumed values. See per-rotor statuses in the CSV.')

    print('\nWING/AERO STATUS')
    if polar is None:
        print('Coefficients — ILLUSTRATIVE / ASSUMED CONSTANTS')
    else:
        print(f'Coefficient source — {Path(polar.source).name} | {polar.status}')
        if polar.user_source:
            print(f'User-provided source metadata: {polar.user_source}')
    if not evaluated:
        print('Phase 2 wing forces — NOT EVALUATED')
    else:
        print(f"Wing forces — {row['aero_status']}")
        angle = row['wing_alpha_deg']
        aoa_status = row['wing_alpha_status']
        print('Wing AoA — ' + (f'{report_cell(angle)}° | ' if angle is not None else '') + aoa_status)
        if row['aero_reason'] and transition['status'] != 'UNAVAILABLE':
            print(f'Wing force blocker — {compact_aero_reason(row)}')

    print('\nMODEL LIMITS')
    print('XV-15 transition performance — UNAVAILABLE. Installed rotor maps and wing wake loads are missing.')
    print(f'Longitudinal 3-DoF — UNAVAILABLE. {LONGITUDINAL_3DOF_BLOCKER}')
    print('Phase 1 is vertical only. Phase 2 uses planar motion and holds Phase 1 axial thrust.')
    print('The model excludes ground effect, rotor RPM response, and wake-immersed wing loads.')
    print('See the wide audit for per-step sources and blockers. Use --verbose for the detailed terminal report.')


def report_run_inputs(aircraft, environment, takeoff, phase1_s, phase2_s, dt, end_angles, pitch_attitude_deg=None,
                      *, rotor_input_mode='planar'):
    model = takeoff['rotor_model']
    illustrative = any('ILLUSTRATIVE' in (takeoff[side].get('source_status') or
                                          takeoff[side].get('calibration_status') or '')
                       for side in ('left', 'right'))
    print('\nRUN SUMMARY')
    print('Scenario: XV-15 helicopter-mode vertical takeoff and nacelle transition.')
    print('Initial state: on the ground, stationary, level, facing north.')
    print(f'Model: {model}')
    if illustrative:
        print('Run type: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED')
    else:
        print('Run type: educational aircraft model; experimental validation: NOT VALIDATED')
    print(f'Requested duration: Phase 1 {phase1_s:g} s | Phase 2 {phase2_s:g} s | timestep {dt:g} s')
    if rotor_input_mode is not None:
        mode = ('ILLUSTRATIVE / UNVALIDATED with H = Y = 0 assumed'
                if rotor_input_mode == 'illustrative' else 'planar model with per-rotor H and Y required for C_RF')
        print(f'Phase 2 rotor-flow mode — {mode}')
        print(f'Accepted XML operating speed for both rotors — {format_quantity(aircraft.hover_rotor_rpm, "rpm", places=0)}')
        print(f'  rotor.xml: {HOVER_RPM_XML_PATH}')

    print('\nRUN INPUTS')
    print(f'Air-density input: {environment.air_density:g} kg/m³ (supplied for this run; held constant)')
    print(f'Wind: {environment.windspeed:g} m/s FROM {environment.winddirection:g}° clockwise from north')
    print(f'Temperature: {environment.temperature:g} °C')
    print(f'Ground elevation (MSL): {environment.altitude:.2f} m (display only)')
    print(f'Design gross (XML): {format_quantity(aircraft.design_gross, "lb")}')
    print(f'Wing area / incidence (XML): {format_quantity(aircraft.wing_area_m2, "m²")} / '
          f'{format_quantity(aircraft.wing_incidence_deg, "°")}')
    print(f'Phase 1 nacelle direction: {format_quantity(aircraft.nacelle_angle_deg, "°")} each (0° forward, 90° up; not blade pitch)')
    print(f'Phase 2 nacelle targets (left/right): {end_angles[0]:g}° / {end_angles[1]:g}° (linear ramps)')
    if pitch_attitude_deg is not None:
        print(f'Phase 2 pitch attitude: {pitch_attitude_deg:g}° (user-prescribed for airflow, nose-up positive)')
    elif rotor_input_mode is not None:
        print('Phase 2 pitch attitude UNAVAILABLE. Set --transition-pitch-deg for planar U/W and wing AoA.')
    if model == 'prototype-collective':
        for side in ('left', 'right'):
            rotor = takeoff[side]
            pitch = rotor.get('theta_deg')
            ct = rotor.get('thrust_coefficient')
            print(f'{side} collective-pitch input: {format_quantity(pitch, "deg", places=1)} (blade pitch); '
                  f'CT = {ct if ct is not None else "UNAVAILABLE"}')
    elif model == 'prototype-fixed-ct':
        print(PROTOTYPE_LABEL)
        ct = takeoff['left']['inputs']['thrust_coefficient']
        print(f'Fixed CT: {ct if ct is not None else "UNAVAILABLE"}; blade collective-pitch input is not used by this model.')


def main(argv=None):
    parser = argparse.ArgumentParser(description='TMPE :: XV-15 HELICOPTER TAKEOFF')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='show detailed checkpoints, model inputs, assumptions, equations and secondary AoA trace')
    parser.add_argument('--diagnostics', action='store_true', help='show XML sources and takeoff blockers')
    parser.add_argument('--duration', '--phase1-duration', type=float, help='Phase 1 duration in seconds')
    parser.add_argument('--transition-duration', type=float, help='Phase 2 duration in seconds')
    parser.add_argument('--transition-pitch-deg', type=float, metavar='DEG',
                        help='nose-up-positive pitch for Phase 2 body-axis airflow and NASA wing AoA')
    parser.add_argument('--transition-end', metavar='LEFT:RIGHT',
                        help='ending nacelle angles in degrees, default 0:0')
    parser.add_argument('--transition-start', metavar='LEFT:RIGHT',
                        help='kinematic start nacelle angles in degrees, default Phase 1 nacelle angles')
    parser.add_argument('--transition-kinematic', metavar='FORWARD:UP',
                        help='run the full Phase 2 schedule with explicit constant velocities in m/s')
    parser.add_argument('--transition-start-height-m', type=float, metavar='METERS',
                        help='prescribed kinematic start height AGL, default retained Phase 1 height')
    parser.add_argument('--trim-benchmark', action='store_true',
                        help='compare the standalone TR-1195-1 p. A-31 trim point')
    wing_model = parser.add_mutually_exclusive_group()
    wing_model.add_argument('--transition-polar', type=Path, help='wing coefficient CSV with header alpha_deg,cl,cd')
    wing_model.add_argument('--illustrative-transition', action='store_true',
                            help='use the legacy constant CL/CD software demonstration')
    parser.add_argument('--illustrative-rotor-flow', action='store_true',
                        help='assume H = Y = 0 for the NASA wing AoA correction (ILLUSTRATIVE / UNVALIDATED)')
    parser.add_argument('--transition-flaps', choices=('0:0', '20:12.5', '40:25', '75:47'),
                        metavar='FLAP:FLAPERON', help='fixed NASA flap/flaperon pair in degrees (default 40:25)')
    parser.add_argument('--polar-source', help='source metadata for the supplied wing coefficient table')
    parser.add_argument('--transition-audit', type=Path, help='new CSV path; default output/stage2_audit_*.csv')
    parser.add_argument('--dt', type=float, help='integration timestep in seconds (default 0.1)')
    parser.add_argument('--ideal-rated-hover', action='store_true', help='separate conditional lossless rated-power hover estimate; not XV-15 predictions')
    parser.add_argument('--rotor-model', choices=('unsupported', 'prototype-fixed-ct', 'prototype-collective'),
                        help='default: prototype-collective with pilot pitch; prototype-fixed-ct is available for regression testing')
    parser.add_argument('--prototype-ct', type=float, help='assumed CT override for both prototype rotors')
    parser.add_argument('--collective-config', type=Path, help='collective model JSON; default: data(v.2)/collective_model.json')
    parser.add_argument('--allow-illustrative-collective', action='store_true', help='opt in to illustrative collective math tests; NOT aircraft predictions')
    parser.add_argument('--nacelle-schedule', nargs='+', default=[],
                        metavar='LEFT:RIGHT', help='untimed illustrative angle pairs in degrees; provisional range 0–90')
    args = parser.parse_args(argv)
    if args.trim_benchmark:
        benchmark = runpy.run_path(str(PROJECT_DIR / 'xv15_trim_benchmark.py'))
        benchmark['report_xv15_trim_benchmark'](Aircraft(), XV15RotorInputs, XV15RotorOperatingPoint,
                                                XV15InPlaneState, calculate_xv15_rotor_flow, XV15_LATERAL_MAST_TILT_DEG,
                                                XV15_MAST_TILT_SOURCE)
        return
    cli_audit = {}
    for key, option, value in (
        ('rotor_model', '--rotor-model', args.rotor_model),
        ('transition_end', '--transition-end', args.transition_end),
        ('timestep', '--dt', args.dt),
    ):
        cli_audit[f'{key}_status'] = 'DEFAULTED' if value is None else 'USER-PRESCRIBED'
        cli_audit[f'{key}_source'] = f'TMPE CLI default for {option}' if value is None else option
    if args.rotor_model is None:
        args.rotor_model = 'prototype-collective'
    if args.transition_end is None:
        args.transition_end = '0:0'
    if args.dt is None:
        args.dt = .1
    cli_audit.update(timestep_s=args.dt, transition_end_deg=args.transition_end,
                     heading_status='DEFAULTED', heading_source='TMPE AircraftState default yaw (north)')
    try:
        prescribed_velocity = None
        prescribed_start_angles = None
        if args.transition_kinematic is not None:
            prescribed_velocity = tuple(map(float, args.transition_kinematic.split(':')))
            if len(prescribed_velocity) != 2 or not all(map(math.isfinite, prescribed_velocity)):
                raise ValueError('--transition-kinematic requires two finite velocities as FORWARD:UP in m/s')
        if args.transition_start_height_m is not None:
            if prescribed_velocity is None:
                raise ValueError('--transition-start-height-m requires --transition-kinematic')
            require_finite('--transition-start-height-m', args.transition_start_height_m, 0)
        if args.transition_start is not None:
            if prescribed_velocity is None:
                raise ValueError('--transition-start requires --transition-kinematic')
            prescribed_start_angles = tuple(map(float, args.transition_start.split(':')))
        angle_pairs = [tuple(map(float, pair.split(':'))) for pair in args.nacelle_schedule]
        end_angles = tuple(map(float, args.transition_end.split(':')))
        angle_inputs = [*angle_pairs, end_angles]
        if prescribed_start_angles is not None:
            angle_inputs.append(prescribed_start_angles)
        for pair in angle_inputs:
            if len(pair) != 2:
                raise ValueError('Nacelle angle pairs must be LEFT:RIGHT')
            for angle in pair:
                validate_transition_angle(angle)
    except ValueError as exc:
        parser.error(str(exc))
    if args.rotor_model != 'prototype-collective' and (args.collective_config is not None or args.allow_illustrative_collective):
        parser.error('collective options require --rotor-model prototype-collective')
    if args.prototype_ct is not None:
        if args.rotor_model != 'prototype-fixed-ct':
            parser.error('--prototype-ct requires --rotor-model prototype-fixed-ct')
        if not math.isfinite(args.prototype_ct) or args.prototype_ct < 0:
            parser.error('--prototype-ct must be finite and nonnegative')
    for flag, value in (('--duration', args.duration), ('--transition-duration', args.transition_duration),
                        ('--dt', args.dt)):
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f'{flag} must be finite and positive')
    if args.transition_pitch_deg is not None and not math.isfinite(args.transition_pitch_deg):
        parser.error('--transition-pitch-deg must be finite')
    if args.polar_source and args.transition_polar is None:
        parser.error('--polar-source requires --transition-polar')
    if args.transition_flaps and (args.transition_polar or args.illustrative_transition):
        parser.error('--transition-flaps applies only to the NASA wing-pylon lookup')
    if args.illustrative_rotor_flow and (args.transition_polar or args.illustrative_transition):
        parser.error('--illustrative-rotor-flow applies only to the NASA wing-pylon lookup')
    polar = None
    if args.transition_polar is not None:
        try:
            polar = TransitionPolar.load(args.transition_polar, args.polar_source)
        except (OSError, ValueError, csv.Error) as exc:
            parser.error(f'Invalid transition polar: {exc}')
    elif not args.illustrative_transition:
        flap_setting = args.transition_flaps or '40:25'
        flap_pair = tuple(map(float, flap_setting.split(':')))
        cli_audit.update(transition_flaps_deg=flap_setting,
                         transition_flaps_status='USER-PRESCRIBED' if args.transition_flaps else 'DEFAULTED',
                         transition_flaps_source='--transition-flaps' if args.transition_flaps else 'TMPE CLI flap default')
        try:
            polar = XV15WingPylonPolar.load(flap_pair)
        except OSError as exc:
            parser.error(f'NASA wing-pylon data unavailable: {exc}')
        except ValueError as exc:
            parser.error(str(exc))
    print('TMPE :: XV-15 HELICOPTER TAKEOFF')
    aircraft = Aircraft()
    state = AircraftState()
    print("\nEnter ground-level environmental conditions.")

    input_statuses = {}

    def scenario_input(key, prompt, **options):
        value, input_statuses[key] = ask_float(prompt, with_status=True, **options)
        return value

    environment = Environment(
        air_density=scenario_input('air_density', 'Air density (kg/m^3)', default=1.225, minimum=0, places=3),
        windspeed=scenario_input('windspeed', 'Wind speed (m/s)', default=5.0, minimum=0),
        winddirection=scenario_input('winddirection', 'Wind FROM direction (degrees clockwise from north)',
                                     default=180.0, places=1),
        temperature=scenario_input('temperature', 'Temperature (degrees C)', default=15.0, places=1),
        altitude=ask_float('Ground elevation above sea level (m)', default=0.0),
        input_statuses=input_statuses,
    )

    collective_options = {}
    if args.rotor_model == 'prototype-collective':
        model = LinearCollectiveModel.load(args.collective_config)
        if args.collective_config is None:
            illustrative_path = PROJECT_DIR.parent / 'tests' / 'fixtures' / 'collective_illustrative.json'
            if not args.allow_illustrative_collective and model.parameters.get('parameter_status') != 'calibrated':
                print('\nXV-15 collective-to-thrust calibration: UNCALIBRATED in the selected configuration; separate left/right blade-pitch inputs are accepted.')
                print(model.evaluate(0)['reason'])
                print('Illustrative coefficients are for SOFTWARE TESTING ONLY; NOT validated XV-15 predictions.')
                while True:
                    answer = input('Enable illustrative collective testing? [y/N]: ').strip().lower()
                    if answer in ('y', 'yes', 'n', 'no', ''):
                        args.allow_illustrative_collective = answer in ('y', 'yes')
                        break
                    print('Please answer yes or no.')
            if args.allow_illustrative_collective:
                model = LinearCollectiveModel.load(illustrative_path)
        cli_audit.update(
            collective_config_status=('USER-PRESCRIBED' if args.collective_config is not None else
                                      'ILLUSTRATIVE OPT-IN' if args.allow_illustrative_collective else 'DEFAULTED'),
            collective_config_source=('--collective-config' if args.collective_config is not None else
                                      'TMPE bundled mathematical-test fixture' if args.allow_illustrative_collective else
                                      'TMPE bundled collective_model.json'),
        )
        pitches = {}
        for side in ('left', 'right'):
            while True:
                theta = ask_float(f'Pilot {side} collective pitch (degrees)')
                check = model.evaluate(theta, allow_illustrative=args.allow_illustrative_collective)
                if check['reason'].startswith('Collective outside'):
                    print(check['reason'])
                    continue
                pitches[side] = theta
                break
        collective_options = dict(collective_deg=pitches, ct_model=model,
                                  allow_illustrative=args.allow_illustrative_collective)
    durations = []
    for stage, duration in (('Phase 1 takeoff', args.duration), ('Phase 2 transition', args.transition_duration)):
        while duration is None or duration <= 0:
            duration = ask_float(f'{stage} duration (s)', minimum=0)
            if duration is None or duration == 0:
                print('Enter a positive duration.')
        durations.append(duration)
    phase1_duration, phase2_duration = durations
    result = simulate_takeoff(aircraft, environment, phase1_duration, args.dt,
                              rotor_model=args.rotor_model, ct_override=args.prototype_ct, **collective_options)
    rotor_input_mode = None
    if isinstance(polar, XV15WingPylonPolar):
        rotor_input_mode = 'illustrative' if args.illustrative_rotor_flow else 'planar'

    try:
        transition = simulate_transition(result, aircraft, environment, polar, phase2_duration, args.dt, end_angles,
                                         pitch_attitude_deg=args.transition_pitch_deg,
                                         illustrative_rotor_flow=args.illustrative_rotor_flow,
                                         heading_deg=math.degrees(state.yaw),
                                         prescribed_velocity_mps=prescribed_velocity,
                                         prescribed_start_height_m=args.transition_start_height_m,
                                         prescribed_start_angles_deg=prescribed_start_angles)
    except ValueError as exc:
        parser.error(str(exc))
    audit_rows = transition['rows'] or [transition['preflight_audit']]
    for row in audit_rows:
        row.update(cli_audit)
    report_run_summary(transition, result, end_angles)
    report_takeoff_summary(result, phase1_duration)
    report_timed_transition(transition, args.transition_audit, verbose=args.verbose)
    report_final_state(transition, result)
    report_status_summary(result, transition, polar)
    if angle_pairs:
        report_transition(sample_nacelle_transition(result, angle_pairs))
    if args.verbose:
        report_run_inputs(aircraft, environment, result, phase1_duration, phase2_duration, args.dt, end_angles,
                          pitch_attitude_deg=args.transition_pitch_deg, rotor_input_mode=rotor_input_mode)
        report_takeoff(result)
        report_xv15_calculations(aircraft, result)
        report_data_status(result, transition, aircraft, polar)
        report_inputs_and_assumptions(transition)
        print('\nTRANSITION EQUATIONS')
        report_transition_equations(simplified=polar is None, nasa=isinstance(polar, XV15WingPylonPolar),
                                    prescribed=transition['prescribed'])
    if args.ideal_rated_hover:
        report_rated_ideal_hover(aircraft, environment)
    if args.diagnostics or args.verbose:
        report_takeoff_diagnostics(result, verbose=args.verbose)
    if args.diagnostics:
        # Preserve the XML ledger and explicitly separate secondary freestream work.
        flow = calculate_aoa(state, environment, aircraft.wing_incidence_deg)
        aero = wing_aerodynamics(flow)
        print('\nSECONDARY FREESTREAM DIAGNOSTICS — excluded from takeoff balance')
        report_diagnostics(aircraft, flow, aero, takeoff_result=result)
    if args.verbose:
        flow = calculate_aoa(state, environment, aircraft.wing_incidence_deg)
        report_details(environment, aircraft, wing_aerodynamics(flow))
        report_aoa(state, aircraft, flow)


if __name__ == "__main__":
    main()


# B. DETAILED VARIABLE WORD BANK
# ============================================================
# Read entries as: variable name = meaning (units, where applicable).
# Some short names are reused locally; their meanings depend on the function.
# Dictionary keys such as flow['air_world'] use the same definitions below.
#
# +--------------------------------------------------------------------+
# | B1. COORDINATES, UNITS AND PHYSICS TERMS
# +--------------------------------------------------------------------+
# NED / world       = Earth-fixed axes: north, east, down, in that order.
# body              = Aircraft-fixed axes: forward, right, down.
# wing              = Wing-fixed axes: forward along chord, right, down.
# chord             = Line from the wing's leading edge to its trailing edge.
# chord plane       = The forward/down (x/z) plane; excludes sideways flow.
# AoA / alpha       = Angle of attack, calculated with atan2(down, forward).
# incidence         = Fixed wing mounting angle relative to the aircraft body;
#                     positive incidence raises the wing's leading edge.
# freestream        = Airflow before local rotor wash or induced-flow effects.
# rad / deg / °     = Radians / degrees; calculations use radians unless named
#                     *_deg, *_degrees, winddirection, or wind_direction.
# ft / ft2 / m2     = Feet / square feet / square metres.
# CL / CD / Cm      = Lift / drag / pitching-moment coefficients. The XML does
#                     not provide a supported model for these coefficients.
# None              = Unknown or unavailable value; it does not mean zero.
#
# +--------------------------------------------------------------------+
# | B2. FILE LOCATIONS AND CONSTANTS
# +--------------------------------------------------------------------+
# PROJECT_DIR       = Folder containing this Python file.
# DATA_DIR          = Folder containing the aircraft XML data.
# G                 = Standard gravitational acceleration (m/s²), used for weight.
# FLOW_EPS_MPS      = Minimum chord-plane speed for a defined AoA (m/s).
#                     This threshold does not turn small forces into zero.
#
# +--------------------------------------------------------------------+
# | B3. XML LOADING
# +--------------------------------------------------------------------+
# filename          = Name of the XML file to read inside DATA_DIR.
# file_path         = Full path to that XML file.
# root              = Top-level XML element in the loaded document.
# element           = XML element currently being inspected.
# name              = Requested XML element's 'name' attribute.
# tag_path          = Path through XML tags to the requested element.
# attribute         = Optional XML attribute to read instead of element text.
# value             = Value being loaded, formatted, or entered by the user.
# loader            = Function used to load one aircraft parameter.
# args              = Arguments passed to loader in try_load(); in main(),
#                     the parsed command-line options instead.
# error             = Caught XML, file-reading, or numeric conversion error.
#
# +--------------------------------------------------------------------+
# | B4. AIRCRAFT DATA (attributes of aircraft / self)
# +--------------------------------------------------------------------+
# aircraft          = Aircraft object holding geometry and mass data from XML.
# self              = The current Aircraft or Environment object in a method.
# design_gross      = Aircraft design gross weight as supplied by XML (lb).
# wing_span_ft      = Wingtip-to-wingtip distance (ft).
# wing_area_ft2     = Wing reference area (ft²).
# wing_area_m2      = The same wing reference area converted to m².
# wing_chord_ft     = Wing chord length from XML (ft).
# wing_aspect_ratio = Wing span squared divided by wing area (dimensionless).
# wing_incidence_deg = Wing mounting angle loaded from XML (degrees).
# fuselage_length_ft / fuselage_width_ft / fuselage_height_ft
#                   = Aircraft fuselage dimensions (ft); scalar width remains unknown.
# rotor_diameter_ft = Diameter of one rotor disk (ft).
# rotor_blades_per_hub = Number of blades attached to each rotor hub.
# rotor_hub_positions = Supplied tilt-axis locations in FS/BL/WL datum feet;
#                       aft/right/up, not CG-centered body coordinates.
#
# +--------------------------------------------------------------------+
# | B5. ENVIRONMENT AND PRESCRIBED AIRCRAFT STATE
# +--------------------------------------------------------------------+
# environment       = Environment object holding the conditions below.
# windspeed         = Horizontal wind speed (m/s), never negative.
# winddirection     = Direction wind comes FROM, clockwise from north (degrees).
# wind_direction    = User-entered value used to set winddirection.
# air_density / density = Air density (kg/m³); density is the user input.
# temperature       = Air temperature (°C).
# altitude          = Ground elevation above sea level (m), not height AGL.
# state             = AircraftState object: prescribed velocity and attitude.
# ground_velocity_ned = Aircraft velocity over the ground: north/east/down (m/s).
# roll / pitch / yaw = Aircraft orientation angles (radians): positive means
#                      right wing down / nose up / turning north toward east.
# angles            = Roll/pitch/yaw tuple in degrees for report_aoa().
#
# +--------------------------------------------------------------------+
# | B6. ROTATIONS AND AIRFLOW CALCULATIONS
# +--------------------------------------------------------------------+
# cr / sr           = Cosine / sine of roll (dimensionless).
# cp / sp           = Cosine / sine of pitch (dimensionless).
# cy / sy           = Cosine / sine of yaw (dimensionless).
# matrix / rotation = Rotation matrix; rotation changes NED to body coordinates.
# row               = One row of a matrix.
# vector            = Three-component quantity being rotated or displayed.
# a / b             = Matching matrix-row/vector components in transform().
# speed             = Wind speed in wind_velocity_ned(); aircraft airspeed
#                     magnitude in a flow dictionary (both m/s).
# from_degrees      = Wind FROM direction passed to wind_velocity_ned() (degrees).
# direction         = The same wind direction converted to radians.
# values            = State/wind numbers collected for finite-value validation.
# v                 = A local number during validation/formatting; also the
#                     wing's sideways velocity when unpacking (u, v, w).
# wind              = Wind velocity toward its destination, in NED axes (m/s).
# g / w             = Ground/wind components in the air_world subtraction.
#                     Elsewhere w means the wing's down-axis airspeed component.
# air_world         = Aircraft velocity relative to air, in NED axes (m/s):
#                     ground_velocity_ned minus wind. Example: northward ground
#                     speed 10 minus northward wind 3 gives airspeed north 7.
# body              = That same air-relative velocity in body axes (m/s).
# body_alpha        = AoA relative to the BODY forward axis (radians); this
#                     does not yet account for wing mounting incidence.
# incidence_deg     = Wing mounting angle passed into calculate_aoa() (degrees).
# i                 = That wing incidence converted to radians.
# wing_rotation     = Matrix changing body coordinates to wing coordinates.
# wing              = Air-relative velocity in wing axes (m/s), or None when
#                     wing incidence is missing.
# u / v / w         = Wing forward / right / down velocity components (m/s).
# alpha             = WING AoA including incidence (radians), or None when
#                     incidence is missing or chord-plane airflow is too small.
# chord_speed       = Magnitude of wing forward/down velocity (m/s).
# _                 = An unpacked component that this calculation does not use.
# result / flow     = Dictionary of airflow vectors, rotations, speed, AoA,
#                     and a status explaining whether wing AoA is available.
#                     These velocities describe aircraft motion relative to air;
#                     incoming airflow points in the opposite direction.
#
# +--------------------------------------------------------------------+
# | B7. AERODYNAMIC RESULTS
# +--------------------------------------------------------------------+
# aero              = Dictionary containing aerodynamic status and force.
# status            = Text explaining availability of AoA or aerodynamic force.
# force_body_n      = Wing freestream force tuple in body axes (N); None when
#                     unavailable. Exactly zero freestream gives a zero tuple.
# regime            = Extra status text for unsupported flow directions.
# airspeed          = Aircraft speed relative to air in calculate_lift() (m/s).
# lift_coefficient  = Supplied CL value (dimensionless); no verified model yet.
# lift              = Display text for wing lift; not a calculated force value.
#
# +--------------------------------------------------------------------+
# | B8. DISPLAY, INPUT AND PROGRAM OPTIONS
# +--------------------------------------------------------------------+
# displayed         = Rounded TEXT made from a number, e.g. '3.14'; used only
#                     for output. The original number keeps its full precision.
# places            = Number of decimal places to show.
# unit              = Unit text appended to a displayed number/vector.
# reason            = Explanation shown when a value is unavailable.
# label             = Heading/name printed beside a value or matrix.
# prompt            = Question shown when requesting user input.
# default           = Value used when the user presses Enter without typing.
# minimum           = Lowest accepted numeric input, when a limit is supplied.
# answer            = User's typed response, with surrounding spaces removed.
# suffix            = Prompt text showing a default or an unknown-value option.
# parser            = Command-line argument parser.
# argv              = Command-line arguments supplied to main(); None uses the
#                     arguments with which the program was launched.
# verbose           = Option enabled by -v or --verbose (args.verbose).
# ============================================================
#
# DIAGNOSTIC WORD BANK
# diagnostics       = Dictionary mapping parameter names to load records.
# records           = Shared diagnostic ledger passed to the XML loader.
# row               = One load record; in matrix calculations, a matrix row.
# name              = Python parameter name used as a diagnostic record key.
# meaning           = Human-readable description of a parameter.
# path              = XML element path, optionally ending in /@attribute.
# unit / source_unit = Output units / original XML units before conversion.
# used              = Whether a parameter is used in the live physics path.
# citation          = Source, location and provenance metadata from XML.
# reason            = Explanation of a load status or calculation blocker.
# dependencies      = Input record names required by a derived field.
# geometry_recovery = Supplied geometry items with their per-item citations.
# LOADED            = Value parsed successfully; not proof of verification.
# UNVERIFIED        = XML explicitly marks the supplied data as unverified.
# MISSING           = Absent data or an intentionally unavailable value.
# INVALID           = Malformed XML, incompatible units or invalid content.
# CALCULATED        = A numerical physics result was produced, including zero.
# UNAVAILABLE       = Required data or supported assumptions are missing.
# UNSUPPORTED       = Operating condition lies outside current model support.
# NOT IMPLEMENTED   = The calculation does not exist yet.
# diagnostics option = --diagnostics prints the ledger and physics blockers.

# VERTICAL TAKEOFF WORD BANK
# left/right_proprotor_thrust_n = Separate rotor-axis thrust magnitudes (N).
# total_rotor_vertical_force_n = Sum of projected upward rotor forces (N).
# aircraft_weight_n = Selected takeoff mass times standard gravity (N).
# net_vertical_force_n = Rotor vertical force + other vertical loads - weight.
# vertical_acceleration_mps2 = Upward acceleration; includes ground support in motion result.
# vertical_velocity_mps = Upward speed (m/s), distinct from NED down velocity.
# height_agl_m = Height above runway (m), never MSL field elevation.
# force_provider = Explicit external force law for integration; not aircraft reference data.
# timestep_s = Seconds per force evaluation and constant-acceleration step.

# FIXED CT PROTOTYPE WORD BANK
# prototype_thrust_coefficient = XML illustrative CT assumption, dimensionless.
# ct_override = Explicit experimental CT assumption for one rotor call or both CLI rotors.
# rotor_model = CLI defaults to prototype-collective; explicit prototype-fixed-ct preserves regression mode.
#               Low-level APIs retain unsupported defaults for explicit model selection.
# angular_speed_rad_s = RPM * 2*pi/60, radians per second.
# disk_area_m2 = pi * radius_m**2; precise circular area, not rounded XML area.
# excluded_loads = Effects intentionally omitted, not measured zero physical loads.
