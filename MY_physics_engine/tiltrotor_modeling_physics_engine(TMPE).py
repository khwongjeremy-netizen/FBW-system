# ============================================================
# TMPE — Tiltrotor Modeling Physics Engine
# Ground-start helicopter takeoff prototype for the XV-15.
#
# This is an educational prototype, NOT a validated flight simulator.
# Scope: rotor lift generation and vertical takeoff from the ground.
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
import xml.etree.ElementTree as ET
import math
from dataclasses import dataclass

PROJECT_DIR = Path(__file__).resolve().parent
DATA_DIR = PROJECT_DIR / "data(v.2)"
G = 9.80665



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


def report_diagnostics(aircraft, flow, aero):
    print('\nTMPE :: DIAGNOSTIC REPORT\nAIRCRAFT DATA')
    for name, row in aircraft.diagnostics.items():
        print(f"  {name}: {row['status']} | {row['meaning']}")
        print(f"    Source: {DATA_DIR / row['file']} :: {row['path']}")
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
            ('rotor_diameter_ft', 'rotor.xml', 'proprotor_and_drive_system/rotor_diameter', 'ft', 'Rotor diameter', False),
            ('rotor_blades_per_hub', 'rotor.xml', 'proprotor_and_drive_system/rotor_blades_per_hub', '1', 'Blades per hub', False),
        )
        for name, filename, path, unit, meaning, used in specs:
            value = load_parameter(self.diagnostics, name, filename, path, unit, meaning, used=used)
            setattr(self, name, value)
        for name, filename, path, unit, meaning in (
            ('engine_takeoff_shp', 'engine.xml', "engine_data/power_rating[@name='takeoff']", 'SHP', 'Per-engine rated takeoff power; not mast power'),
            ('hover_rotor_rpm', 'rotor.xml', "proprotor_and_drive_system/design_rotor_speed[@mode='hover']", 'rpm', 'Design hover rotor speed; not commanded RPM'),
            ('transmission_efficiency', 'relationships.xml', "reported_relationships/relationship[@name='transmission_efficiency']/efficiency", '1', 'Reported mast/engine power ratio; conditional applicability'),
            ('engine_count', 'aircraft.xml', 'aircraft_overview/engine_count', '1', 'Installed engine count'),
            ('rotor_disk_area_ft2', 'rotor.xml', 'proprotor_and_drive_system/rotor_disc_area_per_rotor', 'ft^2', 'Per-rotor disk area'),
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
    ):
        self.windspeed = windspeed
        self.winddirection = winddirection
        self.air_density = air_density
        self.temperature = temperature
        self.altitude = altitude


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


def ask_float(prompt, default=None, minimum=None, places=2):
    while True:
        suffix = f" [{format_number(default, places)}]" if default is not None else " [Enter to leave unknown]"
        answer = input(f"{prompt}{suffix}: ").strip()

        if not answer:
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

        return value


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


def calculate_proprotor(side, aircraft, environment):
    """Evaluate one real-rotor model's readiness; never infer thrust from rating.

    RPM, diameter and engine rating do not establish collective, mast power or
    CT/CQ. A hover momentum estimate also needs explicit induced-power input
    and cannot be reused as a climb/ground-effect thrust law.
    """
    if side not in ('left', 'right'):
        raise ValueError('Proprotor side must be left or right')
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


def simulate_takeoff(aircraft, environment, duration_s=5.0, timestep_s=.1, force_provider=None):
    """Run vertical-only motion; a supplied provider must establish forces each step.

    Provider(time, height, upward_velocity) returns left/right thrust and explicit
    other_vertical_force_n in SI. It is an extension point, not aircraft data.
    Default XV-15 execution stops at the first unsupported force evaluation.
    """
    require_finite('Duration', duration_s, 0)
    require_finite('Timestep', timestep_s, 0)
    if timestep_s == 0:
        raise ValueError('Timestep must be positive')
    mass = takeoff_mass_kg(aircraft.design_gross)
    time, height, velocity = 0.0, 0.0, 0.0
    history = []
    left = calculate_proprotor('left', aircraft, environment)
    right = calculate_proprotor('right', aircraft, environment)
    while True:
        other = None
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
                other_vertical_force_n=other, nacelle_angle_deg=aircraft.nacelle_angle_deg)


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
        ('VERTICAL VELOCITY', motion['vertical_velocity_mps'], 'm/s'),
        ('HEIGHT ABOVE GROUND', motion['height_agl_m'], 'm'),
    ):
        print(f'{label:<23}: {format_quantity(value, unit, reason="force balance unavailable")}')
    print(f"TAKEOFF STATUS         : {motion['takeoff_status']}")
    print(f"Simulated time         : {result['elapsed_s']:.2f} s")
    if motion['status'] != 'CALCULATED':
        if result['elapsed_s'] == 0:
            print('Initial state retained; takeoff not simulated. Initial velocity/height are not trajectory predictions.')
        else:
            print('Simulation stopped; last supported state retained. No motion predicted beyond this time.')
        print('Blockers: rotor mast power, thrust performance maps and supported other vertical loads. See --diagnostics.')
    print('Educational prototype; no validated XV-15 rotor performance model.')


def report_takeoff_diagnostics(result, verbose=False):
    print('\nTAKEOFF PHYSICS')
    for name in ('left', 'right', 'balance', 'motion'):
        row = result[name]
        print(f"  {name}: {row['status']} — {row['reason']}")
    if result['other_vertical_force_n'] is None:
        print('  Other vertical loads: UNAVAILABLE — ground effect, wing download and drag not established for ground liftoff.')
    else:
        print(f"  Other vertical loads: CALCULATED — explicit force provider: {result['other_vertical_force_n']} N")
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


def main(argv=None):
    parser = argparse.ArgumentParser(description='TMPE :: XV-15 HELICOPTER TAKEOFF')
    parser.add_argument('-v', '--verbose', action='store_true', help='show force equations and secondary AoA trace')
    parser.add_argument('--diagnostics', action='store_true', help='show XML sources and takeoff blockers')
    parser.add_argument('--duration', type=float, default=5.0, help='requested simulation duration in seconds')
    parser.add_argument('--dt', type=float, default=.1, help='integration timestep in seconds')
    parser.add_argument('--ideal-rated-hover', action='store_true', help='separate conditional lossless rated-power hover estimate; not XV-15 predictions')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0 or not math.isfinite(args.dt) or args.dt <= 0:
        parser.error('duration must be finite and nonnegative; dt must be finite and positive')
    print('TMPE :: XV-15 HELICOPTER TAKEOFF')
    aircraft = Aircraft()
    state = AircraftState()
    print('Scenario: XV-15 helicopter-mode vertical takeoff.')
    print('Initial state: on the ground, stationary, level, facing north.')
    print('MODE: HELICOPTER; nacelle angle:', format_quantity(aircraft.nacelle_angle_deg, '°', 1))
    print("\nEnter ground-level environmental conditions.")

    density = ask_float(
        "Air density (kg/m^3)",
        default=1.225,
        minimum=0
    )

    windspeed = ask_float(
        "Wind speed (m/s)",
        default=5.0,
        minimum=0
    )

    wind_direction = ask_float(
        "Wind FROM direction (degrees clockwise from north)",
        default=180.0,
        places=1
    )

    temperature = ask_float(
        "Temperature (degrees C)",
        default=15.0,
        places=1
    )

    altitude = ask_float(
        "Ground elevation above sea level (m)",
        default=0.0
    )

    environment = Environment(
        windspeed=windspeed,
        winddirection=wind_direction,
        air_density=density,
        temperature=temperature,
        altitude=altitude
    )

    result = simulate_takeoff(aircraft, environment, args.duration, args.dt)
    report_takeoff(result)
    if args.ideal_rated_hover:
        report_rated_ideal_hover(aircraft, environment)
    if args.diagnostics or args.verbose:
        report_takeoff_diagnostics(result, verbose=args.verbose)
    if args.diagnostics:
        # Preserve the XML ledger and explicitly separate secondary freestream work.
        flow = calculate_aoa(state, environment, aircraft.wing_incidence_deg)
        aero = wing_aerodynamics(flow)
        print('\nSECONDARY FREESTREAM DIAGNOSTICS — excluded from takeoff balance')
        report_diagnostics(aircraft, flow, aero)
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
