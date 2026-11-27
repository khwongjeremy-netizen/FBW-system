"""One GTRS model trim printout, transcribed from the supplied TR-1195-1 p. A-31 scan."""

import math


TRIM_SOURCE = 'TR-1195-1 p. A-31, Aircraft Trim Flight Conditions'
# Keep the printed precision and units apart from the SI values used by TMPE.
TRIM = dict(airspeed_kt='80.00', calibrated_airspeed_kt='80.01', nacelle_deg='75.0', mast_deg='15.00',
            gross_weight_lb='13000.0', altitude_ft='5000.00', rpm='589.00', flap_deg='40', flaperon_deg='25',
            body_velocity_fps=('135.0', '0.2013E-07', '1.544'), body_rates_deg_s=('0.0000E+00',) * 3,
            alpha_deg='0.655', pitch_deg='0.655', sideslip_deg='0.000', roll_deg='0.000', yaw_deg='0.000',
            density_slug_ft3='0.2377E-02', ambient_temp_printed_deg_r='288.16', outside_temp_c='15.00')
COMMON_PRINTED = dict(h_lbf='147.18', y_lbf='29.77', tip_speed_fps='902.1', tip_mach='0.8080',
                      advance_ratio='0.1697', total_inflow_ratio='0.0611', ct='0.006518', cp='0.000556',
                      cd='0.015399', power_shp='540.4', torque_ft_lbf='4819.', coning_deg='2.2096',
                      longitudinal_flap_deg='0.2945', lateral_flap_deg='3.0553', theta0_deg='42.6151',
                      b1_deg='1.5568', a1_deg='0.0000', governor_pitch_deg='1.717')
TRIM_ROTORS = dict(left=dict(COMMON_PRINTED, thrust_lbf='4520.48', induced_velocity_fps='13.6777'),
                   right=dict(COMMON_PRINTED, thrust_lbf='4520.47', induced_velocity_fps='13.6776'))
FT_M = .3048  # International foot.
LBF_N = .45359237 * 9.80665  # International pound and standard gravity.
SLUG_FT3_KG_M3 = LBF_N / FT_M**4  # One slug = lbf s²/ft.


def evaluate_xv15_trim_benchmark(aircraft, inputs_type, operating_point_type, in_plane_state_type, calculate_rotor_flow,
                                lateral_mast_tilt_deg, lateral_mast_tilt_source):
    """Compare independent kinematics and normalization with the finite printed trim case."""
    rpm = float(TRIM['rpm'])
    density = float(TRIM['density_slug_ft3']) * SLUG_FT3_KG_M3
    body_velocity = tuple(float(value) * FT_M for value in TRIM['body_velocity_fps'])
    body_rates = tuple(math.radians(float(value)) for value in TRIM['body_rates_deg_s'])
    mast = float(TRIM['mast_deg'])
    temperature = float(TRIM['outside_temp_c'])

    def point(side, kinematic_audit):
        reference = TRIM_ROTORS[side]
        values = (tuple(float(reference[key]) * LBF_N for key in ('thrust_lbf', 'h_lbf', 'y_lbf'))
                  if kinematic_audit is not None else (None, None, None))
        tip = kinematic_audit.get(f'{side}_in_plane_effective_tip_speed_mps') if kinematic_audit is not None else None
        state = (None if tip is None else in_plane_state_type(
            source=f'{TRIM_SOURCE}, printed induced velocity divided by effective tip speed',
            induced_inflow=float(reference['induced_velocity_fps']) * FT_M / tip))
        return operating_point_type(rpm, *values, rpm_source=TRIM_SOURCE, t_source=TRIM_SOURCE,
                                    in_plane_force_source=TRIM_SOURCE, in_plane_state=state)

    def evaluate(kinematic_audit=None):
        inputs = inputs_type(TRIM_SOURCE, body_velocity_mps=body_velocity, body_rates_rad_s=body_rates,
                             lateral_mast_tilt_deg=lateral_mast_tilt_deg,
                             lateral_mast_tilt_source=lateral_mast_tilt_source,
                             left=point('left', kinematic_audit), right=point('right', kinematic_audit))
        return calculate_rotor_flow(inputs, aircraft, mast, float(TRIM['pitch_deg']), None, None, density,
                                    temperature_c=temperature)[1]

    independent = evaluate()
    reference_fed = evaluate(independent)
    rows = []

    def comparison(side, quantity, printed, calculated, method):
        rows.append(dict(side=side, quantity=quantity, printed=printed, calculated=calculated,
                         difference=None if calculated is None or printed is None else calculated - float(printed),
                         method=method))

    for side, reported in TRIM_ROTORS.items():
        def value(key):
            return independent[f'{side}_{key}']

        dn = value('in_plane_dn_n')
        shaft_tip = value('rotor_shaft_tip_speed_mps')
        airspeed = independent['rotor_cg_airspeed_mps']
        advancing_tip = None
        if None not in (airspeed, shaft_tip):
            angle = math.radians(mast)
            advancing_tip = math.hypot(airspeed + shaft_tip * math.cos(angle), shaft_tip * math.sin(angle))
        comparison(side, 'shaft tip speed (ft/s)', None,
                   None if shaft_tip is None else shaft_tip / FT_M, 'shaft ΩR')
        comparison(side, 'advancing tip speed (ft/s)', reported['tip_speed_fps'],
                   None if advancing_tip is None else advancing_tip / FT_M, 'CR-166536 A-21')
        comparison(side, 'advancing tip Mach', reported['tip_mach'], value('rotor_tip_mach'), 'CR-166536 A-21')
        comparison(side, 'advance ratio', reported['advance_ratio'], independent[f'mu_{side}'], 'CR-166536 A-20')
        comparison(side, 'total inflow ratio', reported['total_inflow_ratio'], reference_fed[f'{side}_in_plane_total_inflow'],
                   'axial ratio plus printed induced speed / effective tip speed')
        comparison(side, 'axial thrust (lbf)', reported['thrust_lbf'],
                   None if dn is None else float(reported['ct']) * dn / LBF_N, 'printed CT × calculated DN')
        for force in ('h', 'y'):
            calculated = value(f'nasa_rotor_{force}_n')
            comparison(side, f'mast {force.upper()} (lbf)', reported[f'{force}_lbf'],
                       None if calculated is None else calculated / LBF_N, 'independent force law')
        comparison(side, 'CT', reported['ct'], reference_fed[f'{side}_in_plane_cr166536_ct'], 'printed thrust / DN')
        cp = None if None in (dn, shaft_tip) else float(reported['power_shp']) * 550 * FT_M * LBF_N / (dn * shaft_tip)
        comparison(side, 'CP', reported['cp'], cp, 'printed shaft power / (DN × shaft tip speed)')
        omega = value('rotor_omega_rad_s')
        torque = None if omega is None else float(reported['power_shp']) * 550 / omega
        comparison(side, 'shaft torque (ft·lbf)', reported['torque_ft_lbf'], torque, 'printed shaft power / shaft Ω')
        comparison(side, 'C_RF', None, reference_fed[f'c_rf_{side}'], 'A-38 using printed T, H, Y')
    return dict(density_kg_m3=density, radius_m=independent['rotor_radius_m'], rows=rows)


def report_xv15_trim_benchmark(aircraft, inputs_type, operating_point_type, in_plane_state_type, calculate_rotor_flow,
                               lateral_mast_tilt_deg, lateral_mast_tilt_source):
    result = evaluate_xv15_trim_benchmark(aircraft, inputs_type, operating_point_type, in_plane_state_type, calculate_rotor_flow,
                                         lateral_mast_tilt_deg, lateral_mast_tilt_source)
    print(f'XV-15 TRIM BENCHMARK — {TRIM_SOURCE}')
    print('GTRS model trim reference | XV-15 flight validation UNVALIDATED')
    print('80.00 kt | 75.0° nacelle / 15.00° mast | 13,000.0 lb | 589.00 rpm | flap/flaperon 40°/25°')
    print(f"Density {result['density_kg_m3']:.6f} kg/m³ from {TRIM['density_slug_ft3']} slug/ft³. "
          'Speed of sound uses the printed outside air temperature of 15.00°C.')
    radius = '' if result['radius_m'] is None else f"{result['radius_m']:.6g} m"
    print(f'Rotor radius from rotor.xml — {radius or "UNAVAILABLE"} | Lateral mast cant {lateral_mast_tilt_deg:g}°')
    print('The ambient temperature column prints 288.16 with a DEG-R label. The unit label is unresolved.')
    print('H/Y references use each rotor’s local mast axes. Thrust, power, and coefficients are per rotor.')
    print(f"{'Rotor':<6} {'Quantity':<27} {'Printed':>11} {'TMPE':>13} {'Delta':>13}  Method")
    for row in result['rows']:
        printed = row['printed'] or ''
        calculated = '' if row['calculated'] is None else format(row['calculated'], '.8g')
        delta = '' if row['difference'] is None else format(row['difference'], '+.5g')
        method = row['method']
        if row['calculated'] is None:
            method += ' (UNAVAILABLE)'
        print(f"{row['side']:<6} {row['quantity']:<27} {printed:>11} {calculated:>13} {delta:>13}  {method}")
    print('Independent H/Y require the verified CR-166536 A-30 H-bar term and matched blade and rotor-flow inputs.')
