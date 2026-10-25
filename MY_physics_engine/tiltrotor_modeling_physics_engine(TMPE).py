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
'''
PROJECTDIR = Path(__file__).resolve().parent
DATA_DIR = PROJECTDIR / "data(v.2)"

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

#Physics engine logic(PEL)
#==================================================
g = 9.80665 #m/s^2 gravity 
class Environment: 
    def __init__(self):
        self.windspeed = 5 #m\s
        self.winddirection = 180.0
        self.Airdensity = 1.225 #kg/m^3
        self.temperature = 15 #degrees
        self.altitude = 0 #m above sea level 

#TEST
#==================================================
# Time samples
t = np.linspace(0, 10, 200)

# Example path in 3D space (metres)
x = 2 * t
y = np.sin(t)
z = 0.5 * t**2

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.plot(x, y, z)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Example 3D Trajectory")

plt.show()
