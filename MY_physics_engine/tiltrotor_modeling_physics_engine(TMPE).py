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