#==================================================
# TMPE(Tiltrotor Modeling Physics Engine)
# Phase 1 objective: 
# Model the physics around the XV-15 taking off from the ground
#==================================================

#importing libraries 
#==================================================
from zipfile import Path

import numpy as np 
import scipy as sp 
import matplotlib.pyplot as plt 
from pathlib import Path
import xml.etree.ElementTree as ET
import re
import math
'''
pip3 install numpy
pip3 install scipy
pip3 install matplotlib
'''
#==================================================

#importing necessary data(Import)
#==================================================

PROJECTDIR = Path(__file__).resolve().parent
DATA_DIR = PROJECTDIR / "data(v.2)"

#function for sourcing values 
def fetchvalue(filename, name):
    file_path = DATA_DIR / filename

    tree = ET.parse(file_path)
    root = tree.getroot()

    for element in root.iter():
        if element.get("name") == name:
            if element.text is None:
                raise ValueError(f"Missing XML value: {name}")

            return float(element.text)

    raise ValueError(f"Missing XML parameter: {name}")

def fetchpara(filename, tag_path, attribute=None):
    file_path = DATA_DIR / filename

    tree = ET.parse(file_path)
    root = tree.getroot()

    element = root.find(tag_path)

    if element is None:
        raise ValueError(f"Missing XML tag: {tag_path}")

    if attribute is not None:
        value = element.get(attribute)
    else:
        value = element.text

    if value is None:
        raise ValueError(f"Missing value for: {tag_path}")

    try:
        return float(value)
    except ValueError:
        return value.strip()
#==================================================

#==================================================
#Physics engine logic(PEL)
#==================================================

#Environment variables(External values)
#==================================================
g = 9.80665 #m/s^2 gravity 
class Environment: 
    def __init__(self):
        self.windspeed = 5 #m\s
        self.winddirection = 180.0
        self.Airdensity = 1.225 #kg/m^3
        self.temperature = 15 #degrees
        self.altitude = 0 #m above sea level 

#Aircraft physical varaibles
#==================================================
class aircraft_values: 
    def __init__(self):
        #designed weight of xv-15
        self.designgross = fetchvalue("mass.xml", "design_gross")
        # Wing geometry — geometry.xml
        self.wing_span = fetchpara("geometry.xml", "geometry/wing/span")  # ft
        self.wing_reference_area = fetchpara("geometry.xml", "geometry/wing/area")  # ft^2
        self.wing_mean_chord = fetchpara("geometry.xml", "geometry/wing/chord")  # ft
        self.wing_aspect_ratio = fetchpara("geometry.xml", "geometry/wing/aspect_ratio")
        self.wing_incidence = fetchpara("geometry.xml", "geometry/wing/incidence")  # deg
        self.wing_dihedral = fetchpara("geometry.xml", "geometry/wing/dihedral")  # deg
        self.wing_sweep = fetchpara("geometry.xml", "geometry/wing/quarter_chord_sweep")  # deg

        # Fuselage geometry
        self.fuselage_length = fetchpara("geometry.xml", "geometry/fuselage", "length")  # ft
        self.fuselage_width = None   # Not supplied
        self.fuselage_height = None  # Not supplied
        self.rotor_tip_to_tip_span = fetchpara("geometry.xml", "geometry/rotor_tip_to_tip_span", "value")  # ft; definition needs verification
        self.maximum_fin_height = fetchpara("geometry.xml", "geometry/maximum_fin_height", "value")  # ft

        # Rotor geometry — rotor.xml
        self.rotor_diameter = fetchpara("rotor.xml", "proprotor_and_drive_system/rotor_diameter")  # ft
        self.rotor_radius = self.rotor_diameter / 2  # ft
        self.rotor_blades_per_hub = fetchpara("rotor.xml", "proprotor_and_drive_system/rotor_blades_per_hub")
        self.rotor_disc_area_per_rotor = fetchpara("rotor.xml", "proprotor_and_drive_system/rotor_disc_area_per_rotor")  # ft^2
        self.rotor_blade_chord = fetchpara("rotor.xml", "proprotor_and_drive_system/blade_chord")  # inches
        self.rotor_solidity = fetchpara("rotor.xml", "proprotor_and_drive_system/rotor_solidity")
        self.rotor_hub_precone_angle = fetchpara("rotor.xml", "proprotor_and_drive_system/hub_precone_angle")  # deg
        self.rotor_hub_positions = None  # Exact 3D coordinates not supplied

#Pilot commands
#==================================================
pilot_command = {
    "request": "takeoff",
    "flight_mode": "helicopter",
    "target_vertical_speed": 2.0,  # m/s; simulator test target
}

#running calculations and physics
#==================================================
def calculate_lift(
    air_density,
    airspeed,
    reference_area,
    lift_coefficient,
):
    """
    Calculate aerodynamic lift.

    Equation:
        L = 0.5 * rho * V**2 * S * CL

    Missing-data policy:
        Do not invent CL when the required aerodynamic
        coefficient is unavailable.
    """

    if lift_coefficient is None:
        raise NotImplementedError(
            "Missing lift coefficient: "
            "a verified XV-15 aerodynamic model is required."
        )

    return (
        0.5
        * air_density
        * airspeed**2
        * reference_area
        * lift_coefficient
    )

# Physics output -> Text response
#==================================================
def explain_lift(
    lift_force,
    airspeed,
    air_density,
    reference_area,
    lift_coefficient,
):
    """
    Convert the aerodynamic lift calculation into a
    readable explanation.

    Missing values remain unknown; they are not guessed.
    """

    # Check whether the required physical variables exist.
    missing_variables = []

    if airspeed is None:
        missing_variables.append("airspeed")
    if air_density is None:
        missing_variables.append("air density")
    if reference_area is None:
        missing_variables.append("reference area")
    if lift_coefficient is None:
        missing_variables.append("lift coefficient")

    # Do not report a numerical lift value if inputs are missing.
    if missing_variables:
        missing_text = ", ".join(missing_variables)

        return (
            "Aerodynamic lift cannot currently be calculated "
            f"because these variables are missing: {missing_text}. "
            "The lift equation is defined, but the numerical "
            "result remains unknown until these values are supplied."
        )

    # The inputs exist, but the calculation might not have been run.
    if lift_force is None:
        return (
            "The variables required for lift are available, "
            "but no lift force has been calculated yet."
        )

    # Convert the force from newtons to kilonewtons for readability.
    lift_kN = lift_force / 1000

    # Describe the result.
    response = (
        f"The calculated aerodynamic lift is {lift_force:.2f} N "
        f"({lift_kN:.3f} kN).\n"
        f"This result uses an airspeed of {airspeed:.2f} m/s, "
        f"an air density of {air_density:.3f} kg/m^3, "
        f"a reference area of {reference_area:.3f} m^2, "
        f"and a lift coefficient of {lift_coefficient:.4f}.\n"
    )

    # Lift is not necessarily enough to keep the aircraft airborne.
    response += (
        "This is the aerodynamic lift represented by the equation. "
        "It does not include rotor thrust, and it does not by itself "
        "determine whether the aircraft can take off."
    )

    return response