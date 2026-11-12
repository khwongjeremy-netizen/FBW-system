# Collective CLI execution evidence

All illustrative coefficients are arbitrary software-test inputs. Experimental XV-15 validation: NOT VALIDATED.

## Uncalibrated aircraft configuration

```sh
printf '\n\n\n\n\n4\n4\n' | PYTHONDONTWRITEBYTECODE=1 python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --rotor-model prototype-collective --diagnostics
```

Exit code: 0

```text
TMPE :: XV-15 HELICOPTER TAKEOFF
Scenario: XV-15 helicopter-mode vertical takeoff.
Initial state: on the ground, stationary, level, facing north.
MODE: HELICOPTER; nacelle angle: 90.0°

Enter ground-level environmental conditions.
Air density (kg/m^3) [1.23]: Wind speed (m/s) [5.00]: Wind FROM direction (degrees clockwise from north) [180.0]: Temperature (degrees C) [15.0]: Ground elevation above sea level (m) [0.00]: 
Collective pitch bounds: {'minimum_deg': -30.0, 'maximum_deg': 30.0, 'basis': 'provisional-software', 'source': 'TMPE software input guard only; not XV-15 mechanical, aerodynamic or flight limits'}
Calibration source: Repository data audit: no paired collective/thrust measurements supplied
Pilot left collective pitch (degrees) [Enter to leave unknown]: Pilot right collective pitch (degrees) [Enter to leave unknown]: 
Model: prototype-collective
CT(theta) = CT_reference + k_theta * (theta - theta_reference)
Wing download, ground effect, rotor interference and other aerodynamic loads are EXCLUDED, not measured zero. Wind, inflow response, stall and power limits are excluded. Fixed RPM; constant selected pitch per rotor. Software mechanics exercise; NOT XV-15 aircraft predictions.
left: pilot collective = 4.0 deg; CT = None; UNAVAILABLE
  Collective calculation: UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition
  Calibration: UNCALIBRATED; experimental validation: NOT VALIDATED
right: pilot collective = 4.0 deg; CT = None; UNAVAILABLE
  Collective calculation: UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition
  Calibration: UNCALIBRATED; experimental validation: NOT VALIDATED

--- Rotor performance ---
LEFT PROPROTOR         : Unavailable — rotor model inputs missing
RIGHT PROPROTOR        : Unavailable — rotor model inputs missing
TOTAL VERTICAL         : Unavailable — rotor model inputs missing

--- Aircraft response (up positive) ---
TAKEOFF MASS           : 5896.70 kg
AIRCRAFT WEIGHT        : 57826.88 N
NET VERTICAL FORCE     : Unavailable — force balance unavailable
VERTICAL ACCEL.        : Unavailable — force balance unavailable
VERTICAL VELOCITY      : 0.00 m/s
HEIGHT ABOVE GROUND    : 0.00 m
TAKEOFF STATUS         : UNAVAILABLE
Simulated time         : 0.00 s
Initial state retained; takeoff not simulated. Initial velocity/height are not trajectory predictions.
Blockers: UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition; UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition; Both rotor thrusts, nacelle angles and explicit other vertical loads are required
Educational prototype; no validated XV-15 rotor performance model.

TAKEOFF PHYSICS
  left: UNAVAILABLE — UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition
  right: UNAVAILABLE — UNCALIBRATED: missing CT_reference, k_theta_per_deg, theta_reference_deg, reference_condition
  balance: UNAVAILABLE — Both rotor thrusts, nacelle angles and explicit other vertical loads are required
  motion: UNAVAILABLE — Motion not advanced: mass or net vertical force unavailable
  Other vertical loads: Wing download, ground effect, rotor interference and other aerodynamic loads are EXCLUDED, not measured zero. Wind, inflow response, stall and power limits are excluded. Fixed RPM; constant selected pitch per rotor. Software mechanics exercise; NOT XV-15 aircraft predictions.
  Rotor model: prototype-collective — local linear CT; experimental validation: NOT VALIDATED
  Shaft power: UNAVAILABLE — engine rating is not commanded rotor mast power; power split and operating conditions unresolved.
  Nacelle convention: 90° vertical, 0° forward; NASA TM-81244 printed p. 4 / PDF p. 6; https://ntrs.nasa.gov/api/citations/19810001546/downloads/19810001546.pdf

SECONDARY FREESTREAM DIAGNOSTICS — excluded from takeoff balance

TMPE :: DIAGNOSTIC REPORT
AIRCRAFT DATA
  design_gross: LOADED | Selected design-gross takeoff weight
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/mass.xml :: .//*[@name='design_gross']
    Value: 13000.0 lb | used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=p. 5, sec. 3.1.1
  wing_span_ft: LOADED | Wing span
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: geometry/wing/span
    Value: 32.17 ft | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=p. 12, sec. 3.3
  wing_area_ft2: LOADED | Wing reference area
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: geometry/wing/area
    Value: 169.0 ft^2 | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=p. 12, sec. 3.3
  wing_chord_ft: LOADED | Wing chord
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: geometry/wing/chord
    Value: 5.25 ft | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=p. 12, sec. 3.3
  wing_aspect_ratio: LOADED | Wing aspect ratio
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: geometry/wing/aspect_ratio
    Value: 6.12 1 | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=p. 12, sec. 3.3
  wing_incidence_deg: LOADED | Wing mounting incidence
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: geometry/wing/incidence
    Value: 3.0 deg | used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=p. 12, sec. 3.3
  fuselage_length_ft: LOADED | Fuselage length
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: geometry/fuselage/@length
    Value: 42.1 ft | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S1
  rotor_diameter_ft: LOADED | Rotor diameter
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/rotor.xml :: proprotor_and_drive_system/rotor_diameter
    Value: 25.0 ft | used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=p. 17, sec. 3.7
  rotor_blades_per_hub: LOADED | Blades per hub
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/rotor.xml :: proprotor_and_drive_system/rotor_blades_per_hub
    Value: 3.0 1 | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=p. 17, sec. 3.7
  prototype_thrust_coefficient: UNVERIFIED | Derived illustrative hover-equilibrium CT assumption; not verified XV-15 data
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/rotor.xml :: proprotor_and_drive_system/prototype_thrust_model/thrust_coefficient
    Value: 0.0102 1 | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=user-specified prototype assumption; location=; provenance=Illustrative hover-equilibrium assumption; not measured or verified XV-15 rotor data.
  engine_takeoff_shp: LOADED | Per-engine rated takeoff power; not mast power
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/engine.xml :: engine_data/power_rating[@name='takeoff']
    Value: 1550.0 SHP | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=sec. 6.2, p. 37
  hover_rotor_rpm: LOADED | Design hover rotor speed; not commanded RPM
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/rotor.xml :: proprotor_and_drive_system/design_rotor_speed[@mode='hover']
    Value: 565.0 rpm | used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=table 4.5, p. 25; p. 19
  transmission_efficiency: LOADED | Reported mast/engine power ratio; conditional applicability
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/relationships.xml :: reported_relationships/relationship[@name='transmission_efficiency']/efficiency
    Value: 0.93 1 | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=sec. 5.1, p. 26
  engine_count: LOADED | Installed engine count
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/aircraft.xml :: aircraft_overview/engine_count
    Value: 2.0 1 | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S1
  rotor_disk_area_ft2: LOADED | Per-rotor disk area
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/rotor.xml :: proprotor_and_drive_system/rotor_disc_area_per_rotor
    Value: 491.0 ft^2 | not used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=p. 17, sec. 3.7
  nacelle_angle_deg: LOADED | Helicopter nacelle angle; 0 forward / 90 vertical
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/aircraft.xml :: helicopter_configuration/nacelle_angle
    Value: 90.0 deg | used in live physics
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S14; location=printed p. 4 / PDF p. 6
  transmission_relationship: LOADED | Reported efficiency relationship; applicability not established
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/relationships.xml :: reported_relationships/relationship[@name='transmission_efficiency']/equation
    Value: 'P_rotor = eta_xmsn * P_engine; eta_xmsn = 0.93' text | not used in live physics
    XML unit: None
    Loaded successfully; LOADED does not assert independent verification.
    Citation: source=S2; location=sec. 5.1, p. 26
  geometry_recovery: UNVERIFIED | Supplied geometry collection
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery
    Value: '37 items' mixed | not used in live physics
    Collection loaded; see individual item validation and citations.
  geometry_recovery.hub_left_fs_in: UNVERIFIED | Mast conversion tilt-axis centerline; aircraft datum, not CG body coordinates
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='hub_left_fs_in']
    Value: 290.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.hub_left_bl_in: UNVERIFIED | Mast conversion tilt-axis centerline; aircraft datum, not CG body coordinates
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='hub_left_bl_in']
    Value: -193.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.hub_left_wl_in: UNVERIFIED | Mast conversion tilt-axis centerline; aircraft datum, not CG body coordinates
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='hub_left_wl_in']
    Value: 86.75 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.hub_right_fs_in: UNVERIFIED | Mast conversion tilt-axis centerline; aircraft datum, not CG body coordinates
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='hub_right_fs_in']
    Value: 290.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.hub_right_bl_in: UNVERIFIED | Mast conversion tilt-axis centerline; aircraft datum, not CG body coordinates
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='hub_right_bl_in']
    Value: 193.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.hub_right_wl_in: UNVERIFIED | Mast conversion tilt-axis centerline; aircraft datum, not CG body coordinates
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='hub_right_wl_in']
    Value: 86.75 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.rotor_radius_ft: UNVERIFIED | Rotor radius
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='rotor_radius_ft']
    Value: 12.5 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.hover_tip_wing_clearance_ft: UNVERIFIED | Rotor tip clearance to wing in hover
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='hover_tip_wing_clearance_ft']
    Value: 4.8 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.fs_zero_forward_of_pitot_in: UNVERIFIED | FS increases aft; origin forward of pitot tip
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='fs_zero_forward_of_pitot_in']
    Value: 10.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.bl_symmetry_plane_in: UNVERIFIED | BL zero is aircraft symmetry plane; supplied left negative/right positive
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='bl_symmetry_plane_in']
    Value: 0.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.wl_datum_in: UNVERIFIED | WL zero is datum below fuselage; WL increases upward
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='wl_datum_in']
    Value: 0.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 9, 12, 16, 17, 19; sec. 3.3, 3.7; Fig. 2.3, 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.fuselage_interior_max_width_ft: UNVERIFIED | Cabin interior maximum width
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='fuselage_interior_max_width_ft']
    Value: 5.0 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.fuselage_floor_width_ft: UNVERIFIED | Cabin interior floor width
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='fuselage_floor_width_ft']
    Value: 4.0 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.fuselage_exterior_width_min_ft: UNVERIFIED | Lower end of supplied exterior width range near FS 310, WL 60
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='fuselage_exterior_width_min_ft']
    Value: 5.0 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.fuselage_exterior_width_max_ft: UNVERIFIED | Upper end of supplied exterior width range near FS 310, WL 60
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='fuselage_exterior_width_max_ft']
    Value: 5.2 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.fuselage_interior_max_height_ft: UNVERIFIED | Cabin interior maximum height
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='fuselage_interior_max_height_ft']
    Value: 5.0 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.fuselage_underwing_height_ft: UNVERIFIED | Cabin interior underwing height
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='fuselage_underwing_height_ft']
    Value: 4.5 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.fuselage_exterior_depth_ft: UNVERIFIED | Derived from supplied 74 in depth; summary rounds to 6.17 ft
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='fuselage_exterior_depth_ft']
    Value: 6.166666666666667 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.aircraft_total_height_ft: UNVERIFIED | Derived from supplied 152 in total height; conflicts with existing 12.75 ft fin-height inventory
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='aircraft_total_height_ft']
    Value: 12.666666666666666 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.belly_keel_wl_in: UNVERIFIED | Supplied external-shell depth reference
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='belly_keel_wl_in']
    Value: 23.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.cabin_crown_wl_in: UNVERIFIED | Supplied external-shell depth reference
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='cabin_crown_wl_in']
    Value: 97.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.cabin_depth_fs_start_in: UNVERIFIED | Supplied external-shell depth reference
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='cabin_depth_fs_start_in']
    Value: 272.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.cabin_depth_fs_end_in: UNVERIFIED | Supplied external-shell depth reference
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='cabin_depth_fs_end_in']
    Value: 310.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 14, 16; sec. 3.5; Fig. 3.5.1; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.fuselage_length_ft: UNVERIFIED | Fixed geometry; length conflicts with existing 42.1 ft inventory
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='fuselage_length_ft']
    Value: 42.17 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied sec. 3.3 / PDF p. 12; implementation snippet; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.wing_span_ft: UNVERIFIED | Supplied wing geometry
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='wing_span_ft']
    Value: 32.17 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied sec. 3.3 / PDF p. 12; implementation snippet; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.wing_chord_ft: UNVERIFIED | Supplied wing geometry
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='wing_chord_ft']
    Value: 5.25 ft | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied sec. 3.3 / PDF p. 12; implementation snippet; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.wing_area_ft2: UNVERIFIED | Supplied wing geometry
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='wing_area_ft2']
    Value: 169.0 ft^2 | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied sec. 3.3 / PDF p. 12; implementation snippet; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.wing_incidence_deg: UNVERIFIED | Supplied wing geometry
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='wing_incidence_deg']
    Value: 3.0 deg | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied sec. 3.3 / PDF p. 12; implementation snippet; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.wing_dihedral_deg: UNVERIFIED | Supplied wing geometry
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='wing_dihedral_deg']
    Value: 2.0 deg | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied sec. 3.3 / PDF p. 12; implementation snippet; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.wing_quarter_chord_sweep_deg: UNVERIFIED | Supplied wing geometry
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='wing_quarter_chord_sweep_deg']
    Value: -6.5 deg | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied sec. 3.3 / PDF p. 12; implementation snippet; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.wing_airfoil: UNVERIFIED | Existing NACA 64A223 (Mod) retained; modification unresolved
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='wing_airfoil']
    Value: 'NACA_64A223' text | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Implementation snippet; no separate pinpoint supplied; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.candidate_cg_fs_in: UNVERIFIED | Candidate only; FS 295 conflicts with existing nominal FS 296; not selected for dynamics
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='candidate_cg_fs_in']
    Value: 295.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Coordinate transformation section; no pinpoint citation supplied; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.candidate_cg_bl_in: UNVERIFIED | Candidate only; FS 295 conflicts with existing nominal FS 296; not selected for dynamics
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='candidate_cg_bl_in']
    Value: 0.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Coordinate transformation section; no pinpoint citation supplied; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.candidate_cg_wl_in: UNVERIFIED | Candidate only; FS 295 conflicts with existing nominal FS 296; not selected for dynamics
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='candidate_cg_wl_in']
    Value: 45.0 in | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Coordinate transformation section; no pinpoint citation supplied; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.xml_loading_diagnosis: UNVERIFIED | NotebookLM software diagnosis, not a NASA-established parser defect; missing units do not themselves cause float conversion failure
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='xml_loading_diagnosis']
    Value: 'Tag mismatch; units; datum/frame mismatch; missing elements' text | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=Supplied PDF pp. 12–19; sec. 3.1–3.7; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.wing_local_angle_equation: UNVERIFIED | Qualitative supplied relation; existing vector AoA retained, incidence applied once; no wake/downwash model supplied
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='wing_local_angle_equation']
    Value: 'alpha_w = alpha_fuselage + incidence - downwash + wake_delta_alpha' text | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=AoA section; no pinpoint citation supplied; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  geometry_recovery.isolated_nacelle_polars: UNVERIFIED | Full +/-180 degree coefficient arrays not supplied; retain None
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item[@name='isolated_nacelle_polars']
    Value: 'unavailable' text | not used in live physics
    Existing data marked unverified in XML.
    Citation: source=S2; location=NASA CR-166536 / CR-166537; no specific pages supplied; url=https://ntrs.nasa.gov/api/citations/19750016648/downloads/19750016648.pdf; provenance=notebooklm_geometry_recovery_2026-11-10.md
  rotor_hub_positions: UNVERIFIED | rotor hub positions
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item
    Value: {'left': (24.166666666666664, -16.083333333333332, 7.229166666666666), 'right': (24.166666666666664, 16.083333333333332, 7.229166666666666)} ft | not used in live physics
    Derived from cited recovery items; not used by live physics.
    Inputs: geometry_recovery.hub_left_fs_in, geometry_recovery.hub_left_bl_in, geometry_recovery.hub_left_wl_in, geometry_recovery.hub_right_fs_in, geometry_recovery.hub_right_bl_in, geometry_recovery.hub_right_wl_in
  fuselage_width_range_ft: UNVERIFIED | fuselage width range ft
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item
    Value: (5.0, 5.2) ft | not used in live physics
    Derived from cited recovery items; not used by live physics.
    Inputs: geometry_recovery.fuselage_exterior_width_min_ft, geometry_recovery.fuselage_exterior_width_max_ft
  fuselage_height_ft: UNVERIFIED | fuselage height ft
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item
    Value: 6.166666666666667 ft | not used in live physics
    Derived from cited recovery items; not used by live physics.
    Inputs: geometry_recovery.fuselage_exterior_depth_ft
  fuselage_interior_width_ft: UNVERIFIED | fuselage interior width ft
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item
    Value: 5.0 ft | not used in live physics
    Derived from cited recovery items; not used by live physics.
    Inputs: geometry_recovery.fuselage_interior_max_width_ft
  rotor_radius_ft: UNVERIFIED | rotor radius ft
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery/item
    Value: 12.5 ft | not used in live physics
    Derived from cited recovery items; not used by live physics.
    Inputs: geometry_recovery.rotor_radius_ft
  fuselage_width_ft: MISSING | Single exterior width
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/geometry.xml :: notebooklm_geometry_recovery
    Value: None ft | not used in live physics
    Intentionally None: only a 5.0–5.2 ft range was supplied.
  isolated_nacelle_polars: MISSING | Isolated nacelle aerodynamic polars
    Source: /Users/JeremyWong/Desktop/FBW-system/MY_physics_engine/data(v.2)/aerodynamics.xml :: aerodynamics/wing_pylon_coefficients/numerical_lookup_arrays
    Value: None 1 | not used in live physics
    Intentionally None: supported numerical coefficient arrays are not supplied.

PHYSICS
  Airspeed: CALCULATED — 5.00 m/s
  Wing AoA: CALCULATED — -177.0°
  CL coefficient: UNAVAILABLE — No supported CL model available; approximate XML descriptors are not lookup arrays.
  Wing freestream force: UNSUPPORTED — unavailable: CL/CD/Cm coefficient model missing; wing AoA available; unsupported reverse/near-normal/spanwise flow
  Wing lift: UNSUPPORTED — unavailable: CL/CD/Cm coefficient model missing; wing AoA available; unsupported reverse/near-normal/spanwise flow
  Rotor thrust: UNAVAILABLE — prototype-collective; see primary rotor results and assumptions

BLOCKERS
  CL coefficient: No supported CL model available; approximate XML descriptors are not lookup arrays.
  Wing freestream force: unavailable: CL/CD/Cm coefficient model missing; wing AoA available; unsupported reverse/near-normal/spanwise flow
  Wing lift: unavailable: CL/CD/Cm coefficient model missing; wing AoA available; unsupported reverse/near-normal/spanwise flow
  Rotor thrust: prototype-collective; see primary rotor results and assumptions
```

## Illustrative pitch 4 degrees

```sh
printf '\n\n\n\n\n4\n4\n' | PYTHONDONTWRITEBYTECODE=1 python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --rotor-model prototype-collective --collective-config tests/fixtures/collective_illustrative.json --allow-illustrative-collective
```

Exit code: 0

```text
TMPE :: XV-15 HELICOPTER TAKEOFF
Scenario: XV-15 helicopter-mode vertical takeoff.
Initial state: on the ground, stationary, level, facing north.
MODE: HELICOPTER; nacelle angle: 90.0°

Enter ground-level environmental conditions.
Air density (kg/m^3) [1.23]: Wind speed (m/s) [5.00]: Wind FROM direction (degrees clockwise from north) [180.0]: Temperature (degrees C) [15.0]: Ground elevation above sea level (m) [0.00]: 
Collective pitch bounds: {'minimum_deg': -10.0, 'maximum_deg': 20.0, 'basis': 'provisional-software', 'source': 'Synthetic fixture input guard; NOT aircraft limits'}
Calibration source: Arbitrary synthetic software-test fixture; NOT measured XV-15 data
Pilot left collective pitch (degrees) [Enter to leave unknown]: Pilot right collective pitch (degrees) [Enter to leave unknown]: 
Model: prototype-collective
CT(theta) = CT_reference + k_theta * (theta - theta_reference)
Wing download, ground effect, rotor interference and other aerodynamic loads are EXCLUDED, not measured zero. Wind, inflow response, stall and power limits are excluded. Fixed RPM; constant selected pitch per rotor. Software mechanics exercise; NOT XV-15 aircraft predictions.
left: pilot collective = 4.0 deg; CT = 0.009; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED
right: pilot collective = 4.0 deg; CT = 0.009; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED

--- Rotor performance ---
LEFT PROPROTOR         : 25549.51 N
RIGHT PROPROTOR        : 25549.51 N
TOTAL VERTICAL         : 51099.01 N

--- Aircraft response (up positive) ---
TAKEOFF MASS           : 5896.70 kg
AIRCRAFT WEIGHT        : 57826.88 N
NET VERTICAL FORCE     : -6727.87 N
VERTICAL ACCEL.        : 0.00 m/s²
VERTICAL VELOCITY      : 0.00 m/s
HEIGHT ABOVE GROUND    : 0.00 m
TAKEOFF STATUS         : GROUNDED
Simulated time         : 5.00 s
Educational prototype; no validated XV-15 rotor performance model.
```

## Illustrative pitch 6 degrees

```sh
printf '\n\n\n\n\n6\n6\n' | PYTHONDONTWRITEBYTECODE=1 python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --rotor-model prototype-collective --collective-config tests/fixtures/collective_illustrative.json --allow-illustrative-collective
```

Exit code: 0

```text
TMPE :: XV-15 HELICOPTER TAKEOFF
Scenario: XV-15 helicopter-mode vertical takeoff.
Initial state: on the ground, stationary, level, facing north.
MODE: HELICOPTER; nacelle angle: 90.0°

Enter ground-level environmental conditions.
Air density (kg/m^3) [1.23]: Wind speed (m/s) [5.00]: Wind FROM direction (degrees clockwise from north) [180.0]: Temperature (degrees C) [15.0]: Ground elevation above sea level (m) [0.00]: 
Collective pitch bounds: {'minimum_deg': -10.0, 'maximum_deg': 20.0, 'basis': 'provisional-software', 'source': 'Synthetic fixture input guard; NOT aircraft limits'}
Calibration source: Arbitrary synthetic software-test fixture; NOT measured XV-15 data
Pilot left collective pitch (degrees) [Enter to leave unknown]: Pilot right collective pitch (degrees) [Enter to leave unknown]: 
Model: prototype-collective
CT(theta) = CT_reference + k_theta * (theta - theta_reference)
Wing download, ground effect, rotor interference and other aerodynamic loads are EXCLUDED, not measured zero. Wind, inflow response, stall and power limits are excluded. Fixed RPM; constant selected pitch per rotor. Software mechanics exercise; NOT XV-15 aircraft predictions.
left: pilot collective = 6.0 deg; CT = 0.011; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED
right: pilot collective = 6.0 deg; CT = 0.011; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED

--- Rotor performance ---
LEFT PROPROTOR         : 31227.17 N
RIGHT PROPROTOR        : 31227.17 N
TOTAL VERTICAL         : 62454.35 N

--- Aircraft response (up positive) ---
TAKEOFF MASS           : 5896.70 kg
AIRCRAFT WEIGHT        : 57826.88 N
NET VERTICAL FORCE     : 4627.47 N
VERTICAL ACCEL.        : 0.78 m/s²
VERTICAL VELOCITY      : 3.92 m/s
HEIGHT ABOVE GROUND    : 9.81 m
TAKEOFF STATUS         : AIRBORNE
Simulated time         : 5.00 s
Educational prototype; no validated XV-15 rotor performance model.
```

## Original fixed CT

```sh
printf '\n\n\n\n\n' | PYTHONDONTWRITEBYTECODE=1 python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --rotor-model prototype-fixed-ct
```

Exit code: 0

```text
TMPE :: XV-15 HELICOPTER TAKEOFF
Scenario: XV-15 helicopter-mode vertical takeoff.
Initial state: on the ground, stationary, level, facing north.
MODE: HELICOPTER; nacelle angle: 90.0°

Enter ground-level environmental conditions.
Air density (kg/m^3) [1.23]: Wind speed (m/s) [5.00]: Wind FROM direction (degrees clockwise from north) [180.0]: Temperature (degrees C) [15.0]: Ground elevation above sea level (m) [0.00]: 
PROTOTYPE ESTIMATE — FIXED CT, FIXED RPM
Model: prototype-fixed-ct; CT source status: illustrative assumption, not verified XV-15 data.
Wing download, ground effect, rotor interference and other aerodynamic loads are excluded from this force balance, not established as physically zero. Entered wind is excluded from rotor thrust and motion; no windy-hover claim. Fixed CT/RPM do not respond to collective, inflow, climb speed or active control. No realistic climb performance or engine power demand is predicted.
left: rotor.xml: proprotor_and_drive_system/prototype_thrust_model/thrust_coefficient | T = CT rho A (Omega R)^2; PROTOTYPE ESTIMATE — FIXED CT, FIXED RPM
  Inputs: {'rotor_diameter_ft': 25.0, 'hover_rotor_rpm': 565.0, 'air_density': 1.225, 'thrust_coefficient': 0.0102}
right: rotor.xml: proprotor_and_drive_system/prototype_thrust_model/thrust_coefficient | T = CT rho A (Omega R)^2; PROTOTYPE ESTIMATE — FIXED CT, FIXED RPM
  Inputs: {'rotor_diameter_ft': 25.0, 'hover_rotor_rpm': 565.0, 'air_density': 1.225, 'thrust_coefficient': 0.0102}

--- Rotor performance ---
LEFT PROPROTOR         : 28956.11 N
RIGHT PROPROTOR        : 28956.11 N
TOTAL VERTICAL         : 57912.21 N

--- Aircraft response (up positive) ---
TAKEOFF MASS           : 5896.70 kg
AIRCRAFT WEIGHT        : 57826.88 N
NET VERTICAL FORCE     : 85.33 N
VERTICAL ACCEL.        : 0.01 m/s²
VERTICAL VELOCITY      : 0.07 m/s
HEIGHT ABOVE GROUND    : 0.18 m
Prototype force excess / weight: 0.148% (excluded-load assumption).
Almost weight-balanced; no substantial takeoff margin. Any motion is a simplified mechanics result.
TAKEOFF STATUS         : AIRBORNE
Simulated time         : 5.00 s
Educational prototype; no validated XV-15 rotor performance model.
```

## Rejected input then recovery

```sh
printf '\n\n\n\n\nnan\n21\n4\n6\n' | PYTHONDONTWRITEBYTECODE=1 python3 'MY_physics_engine/tiltrotor_modeling_physics_engine(TMPE).py' --rotor-model prototype-collective --collective-config tests/fixtures/collective_illustrative.json --allow-illustrative-collective
```

Exit code: 0

```text
TMPE :: XV-15 HELICOPTER TAKEOFF
Scenario: XV-15 helicopter-mode vertical takeoff.
Initial state: on the ground, stationary, level, facing north.
MODE: HELICOPTER; nacelle angle: 90.0°

Enter ground-level environmental conditions.
Air density (kg/m^3) [1.23]: Wind speed (m/s) [5.00]: Wind FROM direction (degrees clockwise from north) [180.0]: Temperature (degrees C) [15.0]: Ground elevation above sea level (m) [0.00]: 
Collective pitch bounds: {'minimum_deg': -10.0, 'maximum_deg': 20.0, 'basis': 'provisional-software', 'source': 'Synthetic fixture input guard; NOT aircraft limits'}
Calibration source: Arbitrary synthetic software-test fixture; NOT measured XV-15 data
Pilot left collective pitch (degrees) [Enter to leave unknown]: Please enter a finite number.
Pilot left collective pitch (degrees) [Enter to leave unknown]: Collective outside provisional-software bounds [-10.0, 20.0] deg
Pilot left collective pitch (degrees) [Enter to leave unknown]: Pilot right collective pitch (degrees) [Enter to leave unknown]: 
Model: prototype-collective
CT(theta) = CT_reference + k_theta * (theta - theta_reference)
Wing download, ground effect, rotor interference and other aerodynamic loads are EXCLUDED, not measured zero. Wind, inflow response, stall and power limits are excluded. Fixed RPM; constant selected pitch per rotor. Software mechanics exercise; NOT XV-15 aircraft predictions.
left: pilot collective = 4.0 deg; CT = 0.009; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED
right: pilot collective = 6.0 deg; CT = 0.011; CALCULATED
  Collective calculation: CT = CT_reference + k_theta * (theta - theta_reference); T = CT rho A (Omega R)^2
  Calibration: ILLUSTRATIVE — MATHEMATICAL TEST ONLY; experimental validation: NOT VALIDATED

--- Rotor performance ---
LEFT PROPROTOR         : 25549.51 N
RIGHT PROPROTOR        : 31227.17 N
TOTAL VERTICAL         : 56776.68 N

--- Aircraft response (up positive) ---
TAKEOFF MASS           : 5896.70 kg
AIRCRAFT WEIGHT        : 57826.88 N
NET VERTICAL FORCE     : -1050.20 N
VERTICAL ACCEL.        : 0.00 m/s²
VERTICAL VELOCITY      : 0.00 m/s
HEIGHT ABOVE GROUND    : 0.00 m
TAKEOFF STATUS         : GROUNDED
Simulated time         : 5.00 s
Educational prototype; no validated XV-15 rotor performance model.
```

## Automated suite

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -q
```

Exit code: 0

```text
----------------------------------------------------------------------
Ran 54 tests in 0.548s

OK
```
