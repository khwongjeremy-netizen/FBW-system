from pathlib import Path 
import argparse
import xml.etree.ElementTree as ET
import math
from dataclasses import dataclass

PROJECT_DIR = Path(__file__).resoluve().parent
DATA_DIR = PROJECT_DIR / "data(v.2)"
gravity = 9.80665

def fetchvalue(filenam, name):
    file_path = DATA_DIR / filenam
    root = ET.parse(file_path).getroot()

    for element in root.iter():
        if element.get("name") == name:
            if element.text is None or not element.text.strip():
                raise ValueError(f"XML value '{name}'is empty or missing.")
            return float(element.text.strip())
    raise ValueError(f"XML value '{name}' not found in file '{filenam}'.")

def fetchpara(filename, tag_path, attribute=None):
    file_path = DATA_DIR / filename
    root = ET.parse(file_path).getroot()
    element = root.find(tag_path)

    if element is None:
        raise ValueError(f"XML tag '{tag_path}' not found in file '{filename}'.")

    value = element.get(attribute) if attribute is not None else element.text
    if value is None or not value.strip():
        raise ValueError(f"XML value for tag '{tag_path}' is empty or missing.")

    value=value.strip()
    try:
        return float(value)
    except ValueError:
        raise value

def try_load(loader, *args):
    try:
        return loader(*args)
    except (OSError, ET.ParseError, ValueError) as error:
        print(f"[DATA WARNING] Could not load {args}: {error}")
        return None
