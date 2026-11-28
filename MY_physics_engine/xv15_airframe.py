"""Independent XV-15 airframe load terms from the GTRS reports, in SI."""

from bisect import bisect_left
import math


FT_TO_M = 0.3048
FT2_TO_M2 = FT_TO_M ** 2
WING_AREA_M2 = 181.0 * FT2_TO_M2  # CR-166535 p. I-7, XV-15 GTRS data deck.
WING_CHORD_M = 5.225 * FT_TO_M
WING_SPAN_M = 32.17 * FT_TO_M
WING_SWEEP_RAD = math.radians(-6.5)
PYLON_AREA_M2 = 24.05 * FT2_TO_M2  # CR-166535 p. I-9, one nacelle.
GTRS_ROTOR_RADIUS_M = 12.5 * FT_TO_M  # CR-166535 p. I-2.
GTRS_MAST_LENGTH_M = 4.667 * FT_TO_M
WING_WAKE_VELOCITY_FACTOR = 1.6  # CR-166535 p. I-3: K0=1.6, K1 through K4=0.
FUSELAGE_EXTRA_DRAG_AREA_M2 = -0.5 * FT2_TO_M2  # Signed DLANG, CR-166535 p. I-6.
GEAR_POD_DRAG_AREA_M2 = 1.15 * FT2_TO_M2  # CR-166535 p. I-6: DPOD.

# NASA CR-166536, p. B-51, Table 4-VIII. The appendix heading says 1/rad,
# but A-53, A-70–A-71 and CR-166535 I-9 establish dimensionless C_mWP.
PITCH_MAST_DEG = (0, 15, 30, 60, 90)
PITCH_COEFFICIENT = {
    (0, 0): (0.025, 0.070, 0.080, 0.050, -0.025),
    (20, 12.5): (-0.05, -0.01, 0.0, -0.05, -0.11),
    (40, 25): (-0.110, -0.090, -0.060, -0.110, -0.170),
    (75, 47): (-0.115, -0.110, -0.080, -0.130, -0.190),
}

# CR-166536 p. B-50, Tables 4-VI and 4-VII. Rates are per radian.
ROLL_MAST_DEG = (0, 30, 60, 90)
ROLL_STATIC_DERIVATIVES = (
    ((-0.012, 0.089, 0.078, 0.039), (0.09, 0.0, -0.02, -0.05)),
    ((-0.136, 0.064, 0.034, -0.051), (0.09, -0.01, 0.0, 0.0)),
)

# CR-166536 p. B-55, Tables 4-XV and 4-XVI.
PYLON_MAST_DEG = (-5, 0, 15, 30, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90)
PYLON_INTERFERENCE_FT2 = (13.5, 13.5, 13.5, 13.5, 13.5, 13.4, 13.25,
                         13.0, 12.0, 10.5, 8.0, 5.7, 3.4, 1.0)
PYLON_INCIDENCE_DEG = (0, 10, 20, 30, 40, 45, 90)
PYLON_DRAG_FACTOR = (0.0, 0.04, 0.1, 0.5, 0.95, 1.0, 1.0)
AILERON_LIFT_PER_DEG = {  # CR-166536 B-54, Table 4-XII.
    (0, 0): 0.00316, (20, 12.5): 0.00396, (40, 25): 0.00476, (75, 47): 0.0,
}


def interpolate(value, abscissa, ordinate):
    """Linearly interpolate within a source table's printed abscissa."""
    if value is None or not abscissa[0] <= value <= abscissa[-1]:
        return None
    upper = bisect_left(abscissa, value)
    if abscissa[upper] == value:
        return ordinate[upper]
    lower = upper - 1
    fraction = (value - abscissa[lower]) / (abscissa[upper] - abscissa[lower])
    return ordinate[lower] + fraction * (ordinate[upper] - ordinate[lower])


def wind_to_body(load, alpha, beta):
    """Rotate forward/right/down wind components using the A-72 wind basis."""
    x, y, z = load
    c_a, s_a = math.cos(alpha), math.sin(alpha)
    c_b, s_b = math.cos(beta), math.sin(beta)
    along_xz = x * c_b - y * s_b
    return (along_xz * c_a - z * s_a,
            x * s_b + y * c_b,
            along_xz * s_a + z * c_a)


def record_vector(audit, name, values, units):
    for axis, value in zip('xyz', values):
        audit[f'{name}_{axis}_{units}'] = value


def wing_wake_audit(mast_deg, area_m2):
    source = 'NASA CR-166536 pp. A-60 §4 and A-70 §B.1'
    outside_wake = mast_deg is not None and 30 < mast_deg <= 90
    audit = dict(wing_freestream_area_m2=area_m2 if outside_wake else None,
                 left_wing_wake_area_m2=0.0 if outside_wake else None,
                 right_wing_wake_area_m2=0.0 if outside_wake else None,
                 wing_wake_source=source,
                 wing_wake_status='REPORT IMMERSION BYPASS' if outside_wake else 'UNAVAILABLE',
                 wing_wake_reason='' if outside_wake else
                 'Common mast angle and matched wake footprint and local flow are required at mast ≤30°')
    for side in ('left', 'right'):
        for load in ('lift', 'drag'):
            audit[f'{side}_wing_wake_{load}_n'] = 0.0 if outside_wake else None
    return audit


def calculate_airframe_loads(body_velocity, density, mast_deg, flap_pair, rotor_induced_mps,
                            wing_alpha_rad, cl, cd, q_wfs_pa, *, mach=None, aileron_deg=None,
                            rotor_induced_source='', wing_flow_status='', rotor_force_coefficients=None):
    """Return only independently identifiable contributions, with reasons for gaps.

    Angles and controls follow the XV-15 GTRS convention. Force lever arms,
    complete fuselage and tail loads, and aircraft totals depend on other inputs.
    """
    audit = wing_wake_audit(mast_deg, WING_AREA_M2)
    audit.update(wing_cm=None, wing_pitch_wind_nm=None, wing_sideforce_wind_n=None,
                 wing_roll_sideslip_wind_nm=None, wing_yaw_sideslip_wind_nm=None,
                 wing_pitch_status='UNAVAILABLE', wing_pitch_reason='Requires dynamic pressure, common mast angle and documented flap/flaperon setting',
                 wing_pitch_source='NASA CR-166536 pp. A-53, A-70–A-71 and B-51 Table 4-VIII; CR-166535 pp. I-7, I-9',
                 wing_lateral_source='NASA CR-166536 pp. A-71–A-74, B-32 and B-50 Tables 4-VI/VII; CR-166535 pp. I-7–I-8',
                 wing_lateral_status='UNAVAILABLE', wing_lateral_reason='Requires body airflow with U >15 ft/s, valid Mach, corrected wing AoA and lookup lift coefficient',
                 wing_aileron_deg=aileron_deg, wing_aileron_lift_n=None,
                 wing_aileron_source='NASA CR-166536 p. A-70 §B.4 and p. B-54 Table 4-XII',
                 wing_aileron_reason='Requires documented flap setting and explicit aileron deflection',
                 wing_freestream_base_lift_n=None, wing_freestream_drag_n=None,
                 wing_freestream_reason='Requires wake-excluded area, pressure and report coefficients',
                 wing_freestream_source='NASA CR-166536 pp. A-57 Fig. A4-1 and A-70 §B',
                 wing_freestream_status='UNAVAILABLE', airframe_fuselage_status='UNAVAILABLE',
                 airframe_fuselage_reason='CR-166536 A-41–A-44 requires matched dimensioned lift/drag/moment arrays vs fuselage AoA and sideslip',
                 fuselage_q_pa=None, fuselage_sideforce_wind_n=None, fuselage_roll_wind_nm=None,
                 fuselage_yaw_wind_nm=None, fuselage_symmetry_status='UNAVAILABLE',
                 fuselage_symmetry_reason='Requires exactly zero lateral airflow for the identified symmetry case',
                 fuselage_extra_drag_n=None, gear_pod_drag_n=None,
                 fuselage_drag_component_source='NASA CR-166536 pp. A-44 and A-108 §B; CR-166535 p. I-6',
                 fuselage_drag_component_status='UNAVAILABLE', fuselage_drag_component_reason='Requires body airflow and density',
                 airframe_horizontal_tail_status='UNAVAILABLE',
                 airframe_horizontal_tail_reason='CR-166536 A-78–A-88 requires elevator, local rotor/wing wakes and full stabilizer coefficient tables',
                 airframe_vertical_tail_status='UNAVAILABLE',
                 airframe_vertical_tail_reason='CR-166536 A-89–A-102 requires rudder, fin-local flow and matched fin coefficient tables',
                 pylon_interference_drag_n=None, pylon_interference_area_m2=None,
                 pylon_interference_source='NASA CR-166536 pp. A-38–A-39, A-69, A-75 §E and B-55 Table 4-XV',
                 pylon_interference_status='UNAVAILABLE', pylon_interference_reason='Requires local dynamic pressures in both wing wakes',
                 wing_local_wake_source='NASA CR-166536 pp. A-38–A-39 and A-69; CR-166535 pp. I-2–I-3',
                 wing_local_wake_status='UNAVAILABLE', wing_local_wake_reason='Requires rotor induced velocity, body airflow and a common mast angle',
                 spinner_drag_n=None, spinner_q_pa=None, spinner_mast_alpha_deg=None,
                 spinner_alpha_rad=None, spinner_beta_rad=None, spinner_induced_mean_mps=None,
                 spinner_induced_velocity_source=rotor_induced_source,
                 spinner_status='UNAVAILABLE', spinner_reason='Requires matched per-rotor induced velocities, body airflow, density and common mast angle',
                 spinner_source='NASA CR-166536 pp. A-75–A-76 §§F–G and B-33',
                 pylon_sideslip_drag_n=None, pylon_sideslip_area_m2=None, pylon_sideslip_drag_factor=None,
                 pylon_sideslip_status='UNAVAILABLE', pylon_sideslip_reason='Requires spinner flow solution',
                 pylon_sideslip_source='NASA CR-166536 p. A-77 §H and B-55 Table 4-XVI; CR-166535 p. I-9',
                 airframe_moment_arm_status='UNAVAILABLE',
                 airframe_moment_arm_reason='Operating CG and matching aerodynamic force application points are required')
    for name, unit in (('wing_freestream_base_force_body', 'n'), ('wing_freestream_lift_body_force', 'n'),
                       ('wing_freestream_drag_body_force', 'n'), ('wing_aileron_lift_body_force', 'n'),
                       ('wing_pitch_only_body_moment', 'nm'),
                       ('wing_sideslip_body_moment', 'nm'), ('spinner_body_force', 'n'),
                       ('pylon_sideslip_body_force', 'n'), ('fuselage_extra_drag_body_force', 'n'),
                       ('gear_pod_drag_body_force', 'n')):
        record_vector(audit, name, (None, None, None), unit)
    wake_radii = rotor_force_coefficients if rotor_force_coefficients is not None else (None, None)
    for side, coefficient in zip(('left', 'right'), wake_radii):
        audit[f'{side}_wing_wake_radius_m'] = wake_radius(coefficient)
        for quantity in ('induced_mps', 'u_mps', 'w_mps', 'q_pa', 'alpha_deg', 'beta_deg'):
            audit[f'{side}_wing_wake_{quantity}'] = None

    area_ft2 = interpolate(mast_deg, PYLON_MAST_DEG, PYLON_INTERFERENCE_FT2)
    if area_ft2 is not None:
        audit['pylon_interference_area_m2'] = area_ft2 * FT2_TO_M2
    if q_wfs_pa is None and body_velocity is not None and density is not None:
        u, _, w = body_velocity
        q_wfs_pa = 0.5 * density * (u * u + w * w)
    if flap_pair in PITCH_COEFFICIENT:
        cm = interpolate(mast_deg, PITCH_MAST_DEG, PITCH_COEFFICIENT[flap_pair])
        audit['wing_cm'] = cm
        if cm is not None and q_wfs_pa is not None:
            audit.update(wing_pitch_wind_nm=q_wfs_pa * WING_AREA_M2 * WING_CHORD_M * cm,
                         wing_pitch_status='CALCULATED IN WIND AXES',
                         wing_pitch_reason='Body projection requires valid local wing AoA and fuselage sideslip')
    if q_wfs_pa is not None and flap_pair in AILERON_LIFT_PER_DEG:
        derivative = AILERON_LIFT_PER_DEG[flap_pair]
        if derivative == 0 or aileron_deg is not None:
            audit.update(wing_aileron_lift_n=(0.0 if derivative == 0 else
                         -q_wfs_pa * WING_AREA_M2 * derivative * abs(aileron_deg)), wing_aileron_reason='')

    if body_velocity is not None:
        u, v, w = body_velocity
        if density is not None:
            calculate_fuselage_drag_components(audit, body_velocity, density)
        speed_xz = math.hypot(u, w)
        if speed_xz > 0:
            alpha = math.atan2(w, u)
            beta = math.atan2(v, speed_xz)
            calculate_wing_projection(audit, alpha, beta, q_wfs_pa, cl, cd, wing_flow_status)
            aileron_lift = audit['wing_aileron_lift_n']
            if aileron_lift is not None:
                record_vector(audit, 'wing_aileron_lift_body_force',
                              wind_to_body((0.0, 0.0, -aileron_lift), alpha, beta), 'n')
            if wing_alpha_rad is not None:
                pitch = audit['wing_pitch_wind_nm']
                if pitch is not None:
                    record_vector(audit, 'wing_pitch_only_body_moment',
                                  wind_to_body((0.0, pitch, 0.0), wing_alpha_rad, beta), 'nm')
                    audit.update(wing_pitch_status='CALCULATED PITCHING COEFFICIENT TERM', wing_pitch_reason='')
            calculate_wing_sideslip(audit, u, beta, q_wfs_pa, mast_deg, flap_pair, mach, wing_alpha_rad, cl)
        if (mast_deg is not None and 0 <= mast_deg <= 90 and density is not None
                and rotor_induced_mps is not None and len(rotor_induced_mps) == 2):
            calculate_local_wing_wake(audit, body_velocity, density, mast_deg, rotor_induced_mps)
            if all(value is not None and math.isfinite(value) for value in rotor_induced_mps):
                calculate_spinner_pylon(audit, body_velocity, density, mast_deg, rotor_induced_mps)
    return audit


def calculate_wing_projection(audit, alpha, beta, q_wfs_pa, cl, cd, flow_status):
    area = audit['wing_freestream_area_m2']
    if q_wfs_pa is None or area is None:
        return
    lift = q_wfs_pa * area * cl if cl is not None else None
    drag = q_wfs_pa * area * cd if cd is not None else None
    audit.update(wing_freestream_base_lift_n=lift, wing_freestream_drag_n=drag)
    if lift is not None:
        record_vector(audit, 'wing_freestream_lift_body_force', wind_to_body((0.0, 0.0, -lift), alpha, beta), 'n')
    if drag is not None:
        record_vector(audit, 'wing_freestream_drag_body_force', wind_to_body((-drag, 0.0, 0.0), alpha, beta), 'n')
    if lift is not None and drag is not None:
        record_vector(audit, 'wing_freestream_base_force_body',
                      wind_to_body((-drag, 0.0, -lift), alpha, beta), 'n')
        input_status = f' / {flow_status}' if flow_status else ''
        audit.update(wing_freestream_status='CALCULATED BASE LIFT AND DRAG' + input_status,
                     wing_freestream_reason='Aileron lift and wing lateral-directional forces are evaluated separately')
    elif lift is not None or drag is not None:
        available = 'LIFT' if lift is not None else 'DRAG'
        missing = 'CD' if lift is not None else 'CL'
        audit.update(wing_freestream_status=f'CALCULATED BASE {available} CONTRIBUTION',
                     wing_freestream_reason=f'Report {missing} unavailable for the matching condition')


def calculate_wing_sideslip(audit, u, beta, q_wfs_pa, mast_deg, flap_pair, mach, wing_alpha_rad, cl):
    if q_wfs_pa is None or mach is None or not 0 <= mach <= 0.6 or u <= 15 * FT_TO_M:
        return
    # The three model-input derivatives of Y'_WP are explicitly zero (I-7).
    audit['wing_sideforce_wind_n'] = 0.0
    if beta == 0:
        audit.update(wing_roll_sideslip_wind_nm=0.0, wing_yaw_sideslip_wind_nm=0.0,
                     wing_lateral_status='CALCULATED STATIC SIDESLIP TERMS',
                     wing_lateral_reason='Wing roll/yaw rate and aileron terms are separate contributions')
        record_vector(audit, 'wing_sideslip_body_moment', (0.0, 0.0, 0.0), 'nm')
        return
    if cl is None or wing_alpha_rad is None or flap_pair not in PITCH_COEFFICIENT:
        return
    static_roll, static_roll_over_lift = ROLL_STATIC_DERIVATIVES[flap_pair != (0, 0)]
    roll0 = interpolate(mast_deg, ROLL_MAST_DEG, static_roll)
    roll_cl = interpolate(mast_deg, ROLL_MAST_DEG, static_roll_over_lift)
    if roll0 is None:
        return
    cos_sweep = math.cos(WING_SWEEP_RAD)
    compressibility = math.sqrt(1 - (mach * cos_sweep) ** 2)
    aspect_ratio = WING_SPAN_M ** 2 / WING_AREA_M2
    c_beta = (aspect_ratio + 4 * cos_sweep) / (aspect_ratio * compressibility + 4 * cos_sweep)
    roll_beta = c_beta * roll0 + cl * roll_cl
    yaw_beta = c_beta * -0.0315 + cl * cl * 0.057
    roll = q_wfs_pa * WING_AREA_M2 * WING_SPAN_M * roll_beta * math.copysign(min(abs(beta), math.radians(15)), beta)
    yaw = q_wfs_pa * WING_AREA_M2 * WING_SPAN_M * yaw_beta * beta
    audit.update(wing_roll_sideslip_wind_nm=roll, wing_yaw_sideslip_wind_nm=yaw,
                 wing_lateral_status='CALCULATED STATIC SIDESLIP TERMS',
                 wing_lateral_reason='Wing roll/yaw rate and aileron terms are separate contributions')
    record_vector(audit, 'wing_sideslip_body_moment', wind_to_body((roll, 0.0, yaw), wing_alpha_rad, beta), 'nm')


def wake_radius(force_coefficient):
    if force_coefficient is None or not math.isfinite(force_coefficient) or force_coefficient < 0:
        return None
    z = GTRS_MAST_LENGTH_M / GTRS_ROTOR_RADIUS_M
    exponent = -(0.3 + 2 * z * math.sqrt(force_coefficient) + 60 * force_coefficient)
    return GTRS_ROTOR_RADIUS_M * (0.78 + 0.22 * math.exp(exponent))


def calculate_fuselage_drag_components(audit, velocity, density):
    u, v, _ = velocity
    speed = math.hypot(*velocity)
    pressure = 0.5 * density * speed ** 2
    fuselage_drag = pressure * FUSELAGE_EXTRA_DRAG_AREA_M2
    pod_drag = pressure * GEAR_POD_DRAG_AREA_M2
    audit.update(fuselage_q_pa=pressure, fuselage_extra_drag_n=fuselage_drag, gear_pod_drag_n=pod_drag,
                 fuselage_drag_component_status='CALCULATED DRAG CONTRIBUTIONS', fuselage_drag_component_reason='')
    for name, drag in (('fuselage_extra_drag_body_force', fuselage_drag), ('gear_pod_drag_body_force', pod_drag)):
        force = tuple(-drag * component / speed for component in velocity) if speed > 0 else (0.0, 0.0, 0.0)
        record_vector(audit, name, force, 'n')
    if u > 0 and v == 0:
        audit.update(fuselage_sideforce_wind_n=0.0, fuselage_roll_wind_nm=0.0,
                     fuselage_yaw_wind_nm=0.0, fuselage_symmetry_status='SYMMETRIC ZERO SIDESLIP',
                     fuselage_symmetry_reason='')


def calculate_local_wing_wake(audit, velocity, density, mast_deg, rotor_induced_mps):
    u, v, w = velocity
    sin_mast, cos_mast = math.sin(math.radians(mast_deg)), math.cos(math.radians(mast_deg))
    pressures = []
    for side, rotor_induced in zip(('left', 'right'), rotor_induced_mps):
        if rotor_induced is None or not math.isfinite(rotor_induced):
            continue
        local_induced = rotor_induced * WING_WAKE_VELOCITY_FACTOR
        local_u, local_w = u + local_induced * sin_mast, w - local_induced * cos_mast
        speed_xz = math.hypot(local_u, local_w)
        pressure = 0.5 * density * speed_xz ** 2
        pressures.append(pressure)
        audit[f'{side}_wing_wake_induced_mps'] = local_induced
        audit[f'{side}_wing_wake_u_mps'] = local_u
        audit[f'{side}_wing_wake_w_mps'] = local_w
        audit[f'{side}_wing_wake_q_pa'] = pressure
        if speed_xz > 0:
            audit[f'{side}_wing_wake_alpha_deg'] = math.degrees(math.atan2(local_w, local_u))
            audit[f'{side}_wing_wake_beta_deg'] = math.degrees(math.atan2(v, speed_xz))
    if len(pressures) == 2:
        audit.update(wing_local_wake_status='CALCULATED LOCAL FLOW', wing_local_wake_reason='',
                     pylon_interference_drag_n=audit['pylon_interference_area_m2'] * sum(pressures) / 2,
                     pylon_interference_status='CALCULATED MAGNITUDE',
                     pylon_interference_reason='Body force requires its report direction convention')
    elif pressures:
        audit.update(wing_local_wake_status='PARTIAL LOCAL FLOW',
                     wing_local_wake_reason='Requires the other rotor induced velocity for paired loads')


def calculate_spinner_pylon(audit, velocity, density, mast_deg, rotor_induced_mps):
    u, v, w = velocity
    sin_mast = math.sin(math.radians(mast_deg))
    cos_mast = math.cos(math.radians(mast_deg))
    induced = sum(rotor_induced_mps) / 2
    mast_u = u * cos_mast + w * sin_mast
    mast_w = -induced - u * sin_mast + w * cos_mast
    mast_in_plane = math.hypot(mast_u, v)
    q_sp = 0.5 * density * (mast_in_plane ** 2 + mast_w ** 2)
    if mast_in_plane == 0 and mast_w == 0:
        audit.update(spinner_q_pa=q_sp, spinner_induced_mean_mps=induced,
                     spinner_reason='Spinner/pylon flow angles undefined at zero local velocity')
        return
    mast_alpha = math.atan2(mast_in_plane, abs(mast_w))
    # CR-166536 A-76 defines a 0.01 ft/s forward floor for these angles only.
    local_u = max(0.01 * FT_TO_M, u + induced * sin_mast)
    local_w = w - induced * cos_mast
    alpha = math.atan2(local_w, local_u)
    beta = math.atan2(v, math.hypot(local_u, local_w))
    spinner_drag = 2 * q_sp * FT2_TO_M2 * (1.0 + 5.5 * math.sin(mast_alpha) ** 3)
    audit.update(spinner_drag_n=spinner_drag, spinner_q_pa=q_sp, spinner_induced_mean_mps=induced,
                 spinner_mast_alpha_deg=math.degrees(mast_alpha), spinner_alpha_rad=alpha,
                 spinner_beta_rad=beta, spinner_status='CALCULATED FOR TWO SPINNERS', spinner_reason='')
    record_vector(audit, 'spinner_body_force', wind_to_body((-spinner_drag, 0.0, 0.0), alpha, beta), 'n')
    if mast_in_plane == 0:
        audit['pylon_sideslip_reason'] = 'Mast-plane sideslip projection undefined at zero in-plane flow'
        return
    area = PYLON_AREA_M2 * abs(v / mast_in_plane)
    drag_factor = interpolate(math.degrees(mast_alpha), PYLON_INCIDENCE_DEG, PYLON_DRAG_FACTOR)
    drag = 2 * q_sp * area * drag_factor
    audit.update(pylon_sideslip_drag_n=drag, pylon_sideslip_area_m2=area,
                 pylon_sideslip_drag_factor=drag_factor, pylon_sideslip_status='CALCULATED FOR TWO NACELLES',
                 pylon_sideslip_reason='')
    record_vector(audit, 'pylon_sideslip_body_force', wind_to_body((-drag, 0.0, 0.0), alpha, beta), 'n')
