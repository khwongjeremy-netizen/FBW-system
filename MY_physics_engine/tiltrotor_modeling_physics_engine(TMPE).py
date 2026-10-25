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
'''
print("Looking for data in:", DATA_DIR)
print("Folder exists:", DATA_DIR.exists())
print("XML files found:", len(list(DATA_DIR.rglob("*.xml"))))

def inspect_xml_files():

    xml_files = sorted(DATA_DIR.rglob("*.xml"))

    if not xml_files:
        raise FileNotFoundError(
            f"No XML files found in: {DATA_DIR}"
        )

    for file_path in xml_files:
        print(f"\n --- {file_path.relative_to(DATA_DIR)} ---")

        tree = ET.parse(file_path)
        root = tree.getroot()

        for element in root.iter():
            if element is root:
                continue

        value = (element.text or "").strip()

        if value:
            print(
                f"Tag: {element.tag} | " 
                f"Value: {value} |"
                f" Attributes: {element.attrib}"
                )
'''
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
        # Wing geometry
        self.wing_span = fetchpara("geometry.xml", "geometry/wing/span")
        self.wing_reference_area = fetchpara("geometry.xml", "geometry/wing/area")
        self.wing_mean_chord = fetchpara("geometry.xml", "geometry/wing/chord")

        # Fuselage geometry
        self.fuselage_length = fetchpara("geometry.xml", "geometry/fuselage", "length")
        self.fuselage_width = None   # Not provided in the supplied XML
        self.fuselage_height = None  # Not provided in the supplied XML

        # Rotor geometry
        self.rotor_radius = None          # Rotor diameter not established by this XML
        self.rotor_hub_positions = None   # Exact 3D coordinates not provided

        # Tail geometry
        self.horizontal_tail_area = fetchpara(
            "geometry.xml", "geometry/horizontal_tail/area"
        )
        self.vertical_tail_area = fetchpara(
            "geometry.xml", "geometry/vertical_tail/total_area"
        )
                                                
