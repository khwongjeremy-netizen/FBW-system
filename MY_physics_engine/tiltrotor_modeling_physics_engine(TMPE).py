#
#     try:
#         generate_report(aircraft, user_input)
#     except ValueError as error:
#         print(f"\n[CALCULATION ERROR] {error}")
#
#
# if __name__ == "__main__":
#     main()

# ============================================================
# TMPE — Tiltrotor Modeling Physics Engine
# Version 0.1: interactive input, XML loading, lift calculation,
# and readable output.
#
# This is an educational prototype, NOT a validated flight simulator.
# It does not yet calculate rotor thrust, aircraft motion, or FBW laws.
# ============================================================

from pathlib import Path
import xml.etree.ElementTree as ET

PROJECT_DIR = Path(__file__).resolve().parent
DATA_DIR = PROJECT_DIR / "data(v.2)"
G = 9.80665  # m/s²


# ============================================================
# 1. XML DATA LOADING
# ============================================================

def fetchvalue(filename, name):
    """Load a numeric XML element identified by its name attribute."""
    file_path = DATA_DIR / filename
    root = ET.parse(file_path).getroot()

    for element in root.iter():
        if element.get("name") == name:
            if element.text is None or not element.text.strip():
                raise ValueError(f"XML value '{name}' is empty.")
            return float(element.text.strip())

    raise ValueError(f"XML parameter '{name}' was not found in {filename}.")


def fetchpara(filename, tag_path, attribute=None):
    """Load a value from an XML tag path, optionally reading an attribute."""
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
    """Load a parameter if possible; otherwise keep it unknown and warn."""
    try:
        return loader(*args)
    except (OSError, ET.ParseError, ValueError) as error:
        print(f"[DATA WARNING] Could not load {args}: {error}")
        return None


# ============================================================
# 2. AIRCRAFT PARAMETERS
# ============================================================

class Aircraft:
    """Selected XV-15 parameters loaded from XML."""

    def __init__(self):
        self.design_gross = try_load(fetchvalue, "mass.xml", "design_gross")

        self.wing_span_ft = try_load(
            fetchpara, "geometry.xml", "geometry/wing/span"
        )
        self.wing_area_ft2 = try_load(
            fetchpara, "geometry.xml", "geometry/wing/area"
        )
        self.wing_chord_ft = try_load(
            fetchpara, "geometry.xml", "geometry/wing/chord"
        )
        self.wing_aspect_ratio = try_load(
            fetchpara, "geometry.xml", "geometry/wing/aspect_ratio"
        )
        self.wing_incidence_deg = try_load(
            fetchpara, "geometry.xml", "geometry/wing/incidence"
        )
        self.fuselage_length_ft = try_load(
            fetchpara, "geometry.xml", "geometry/fuselage", "length"
        )
        self.rotor_diameter_ft = try_load(
            fetchpara, "rotor.xml",
            "proprotor_and_drive_system/rotor_diameter"
        )
        self.rotor_blades_per_hub = try_load(
            fetchpara, "rotor.xml",
            "proprotor_and_drive_system/rotor_blades_per_hub"
        )

        # Not supplied/confirmed in the current data framework:
        self.rotor_hub_positions = None
        self.fuselage_width_ft = None
        self.fuselage_height_ft = None

    @property
    def wing_area_m2(self):
        """Convert square feet to square metres for SI calculations."""
        if self.wing_area_ft2 is None:
            return None
        return self.wing_area_ft2 * 0.09290304


# ============================================================
# 3. ENVIRONMENT AND USER INPUT
# ============================================================

class Environment:
    def __init__(
        self,
        windspeed=5.0,
        winddirection=180.0,
        air_density=1.225,
        temperature=15.0,
        altitude=0.0,
    ):
        self.windspeed = windspeed          # m/s
        self.winddirection = winddirection  # degrees; convention must be defined
        self.air_density = air_density      # kg/m³
        self.temperature = temperature      # °C
        self.altitude = altitude            # m


def ask_text(prompt, default):
    answer = input(f"{prompt} [{default}]: ").strip()
    return answer if answer else default


def ask_float(prompt, default=None, minimum=None):
    while True:
        suffix = f" [{default}]" if default is not None else " [Enter to leave unknown]"
        answer = input(f"{prompt}{suffix}: ").strip()

        if not answer:
            return default

        try:
            value = float(answer)
        except ValueError:
            print("Please enter a valid number.")
            continue

        if minimum is not None and value < minimum:
            print(f"Please enter a value of at least {minimum}.")
            continue

        return value