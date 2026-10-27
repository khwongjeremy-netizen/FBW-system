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


# ============================================================
# 4. MAIN PROGRAM — CONNECT THE COMPONENTS
# ============================================================

def calculate_lift(air_density, airspeed, wing_area_m2, lift_coefficient):
    """
    Calculate wing lift using:
    L = 0.5 * rho * V^2 * S * CL

    Returns None if required parameters are unknown.
    """
    if None in (air_density, airspeed, wing_area_m2, lift_coefficient):
        return None

    if air_density < 0 or airspeed < 0 or wing_area_m2 <= 0:
        raise ValueError("Density and airspeed must be non-negative; wing area must be positive.")

    return 0.5 * air_density * airspeed**2 * wing_area_m2 * lift_coefficient


def main():
    print("\n" + "=" * 55)
    print(" TMPE — TILTROTOR MODELING PHYSICS ENGINE")
    print("=" * 55)

    # 1. Load aircraft parameters from XML.
    print("\n[1/3] Loading XV-15 aircraft data...")
    aircraft = Aircraft()

    # 2. Collect pilot commands.
    print("\n[2/3] Enter pilot commands.")

    command = ask_text(
        "Command (hover / takeoff / move_forward / land)",
        "hover"
    ).lower()

    flight_mode = ask_text(
        "Flight mode (helicopter / transition / airplane)",
        "helicopter"
    ).lower()

    airspeed = ask_float(
        "Airspeed (m/s; Enter if unknown)",
        default=None,
        minimum=0
    )

    lift_coefficient = ask_float(
        "Wing lift coefficient CL (Enter if unknown)",
        default=None
    )

    # 3. Collect environmental conditions.
    print("\n[3/3] Enter environmental conditions.")

    density = ask_float(
        "Air density (kg/m^3)",
        default=1.225,
        minimum=0
    )

    windspeed = ask_float(
        "Wind speed (m/s)",
        default=5.0,
        minimum=0
    )

    wind_direction = ask_float(
        "Wind direction (degrees)",
        default=180.0
    )

    temperature = ask_float(
        "Temperature (degrees C)",
        default=15.0
    )

    altitude = ask_float(
        "Altitude (m)",
        default=0.0
    )

    environment = Environment(
        windspeed=windspeed,
        winddirection=wind_direction,
        air_density=density,
        temperature=temperature,
        altitude=altitude
    )

    # Calculate wing lift only when the required inputs are known.
    lift = calculate_lift(
        environment.air_density,
        airspeed,
        aircraft.wing_area_m2,
        lift_coefficient
    )

    # Display results.
    print("\n" + "=" * 55)
    print(" TMPE — SIMULATION INPUT REPORT")
    print("=" * 55)

    print(f"Pilot command:       {command}")
    print(f"Selected mode:       {flight_mode}")
    print(f"Air density:         {environment.air_density} kg/m^3")
    print(f"Wind speed:          {environment.windspeed} m/s")
    print(f"Wind direction:      {environment.winddirection} degrees")
    print(f"Temperature:         {environment.temperature} C")
    print(f"Altitude:            {environment.altitude} m")

    print("\n--- Aircraft data ---")
    print(f"Design gross:        {aircraft.design_gross}")
    print(f"Wing span:           {aircraft.wing_span_ft} ft")
    print(f"Wing area:           {aircraft.wing_area_ft2} ft^2")
    print(f"Wing area (SI):      {aircraft.wing_area_m2} m^2")
    print(f"Rotor diameter:      {aircraft.rotor_diameter_ft} ft")
    print(f"Rotor blades/hub:    {aircraft.rotor_blades_per_hub}")

    print("\n--- Calculation ---")
    if lift is None:
        print("Wing lift: NOT CALCULATED")
        print("Reason: airspeed, wing area, or CL is unknown.")
    else:
        print(f"Wing lift:           {lift:.2f} N")
        print(f"Wing lift:           {lift / 1000:.3f} kN")

    print("\n--- Model limitations ---")
    print("Rotor thrust:        Not implemented")
    print("Aircraft dynamics:   Not implemented")
    print("Flight-by-wire:      Not implemented")
    print("Actual trajectory:   Not calculated")

    print("\nTMPE run complete.")
    print("This is an educational prototype, not a validated flight simulator.")


if __name__ == "__main__":
    main()
