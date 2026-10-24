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
DATA_DIR = PROJECTDIR / "data"

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
#==================================================

#Physics engine logic(PEL)
#==================================================
if __name__ == "__main__":
    inspect_xml_files()
