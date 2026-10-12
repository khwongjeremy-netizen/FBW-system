
# Basic parameters for a simplified tiltrotor
#CV-22 osprey


# aircraft_parameters.py
# Source: https://www.af.mil/about-us/fact-sheets/display/article/104531/cv-22-osprey/
# No additional aircraft specifications added

# CV-22 Osprey: basic parameters for a simplified tiltrotor
# Source: https://www.af.mil/about-us/fact-sheets/display/article/104531/cv-22-osprey/
# Operational limits: user-provided summary of AFMAN 11-2CV-22 Vol 3
# Operational limits are not necessarily physics-model inputs.

# Aircraft dimensions
length_ft = 57 + 4 / 12
height_ft = 22 + 1 / 12
wingspan_ft = 83 + 10 / 12
rotor_diameter_ft = 38

# Weight limits
max_gross_weight_self_deployment_lb = 60500
max_gross_weight_stol_lb = 57000
max_gross_weight_vtol_lb = 52600

# Propulsion
number_of_engines = 2
shaft_horsepower_per_engine = 6200  # More than 6,200 HP per engine

# Fuel and weight-related limits
minimum_destination_fuel_reserve_lb = 1500
minimum_fuel_declaration_threshold_lb = 1500
emergency_fuel_declaration_threshold_lb = 1200
descent_approach_missed_approach_fuel_lb = 1200
remote_destination_holding_fuel_reserve_lb = 1500
operational_fuel_buffer_margin_lb = 300  # 1500 - 1200
maximum_mission_navigation_kit_weight_lb = 200
rescue_hoist_load_limit_lb = 600
belay_equipment_mass_threshold_min_lb = 400
belay_equipment_mass_threshold_max_lb = 1000
basic_weight_change_tolerance_percent = 0.3

# Ground and obstacle clearances
minimum_taxiway_width_ft = 20
minimum_runway_width_ft = 30
minimum_180_degree_turn_width_ft = 35
taxi_obstruction_clearance_ft = 25
terminal_horizontal_obstacle_clearance_ft = 25

# Proprotor clearance and formation
proprotor_tip_path_obstacle_clearance_ft = 25
# Formation criterion is qualitative in the source; no numerical separation given
'''
formation_rotor_separation_reference = "Largest rotor span / rotor disk diameter"
'''

# Center of gravity operational limits
basic_weight_cg_tolerance_in = 0.3
maximum_ramp_aft_vtol_cg_excursion_in = 10

# Propulsion and power requirements
power_available_engine_reference_percent = 95
power_available_mast_torque_reference_percent = 117
hover_power_assurance_mgt_limit_percent = 95
power_reconfirmation_margin_percent = 10
"""
vtol_approach_clear_escape_power_reference = "Out of Ground Effect (OGE) hover power"
vtol_approach_restricted_escape_power_reference = "OGE hover power + 5%"
vtol_departure_clear_escape_power_reference = "30-foot hover power"
vtol_departure_restricted_escape_power_reference = "OGE hover power"
"""

# Nacelle mode thresholds
# Helicopter/conversion mode: angle >= 60 deg; airplane mode: angle < 60 deg
helicopter_conversion_mode_min_angle_deg = 60
airplane_mode_angle_threshold_deg = 60

# Operational altitude limits
airplane_mode_operational_altitude_floor_ft_agl = 500
conversion_mode_operational_altitude_floor_ft_agl = 300
mountainous_airplane_mode_low_level_floor_ft_agl = 200
mountainous_conversion_mode_low_level_floor_ft_agl = 100
flat_rolling_airplane_mode_low_level_floor_ft_agl = 100
flat_rolling_conversion_mode_low_level_floor_ft_agl = 50

# Special operational limits
helocast_maximum_altitude_ft_awl = 10
helocast_maximum_ground_speed_kgs = 10
rappelling_maximum_training_altitude_ft = 150