# XV-15 XML Data Library

This folder splits the original XV-15 XML dataset into smaller, topic-based XML files.

## Files

- `data/sources.xml` — source records S1 and S2.
- `data/aircraft.xml` — broad aircraft overview and capability inventory.
- `data/mass.xml` — reported weights, useful-load breakdown, component weights, and mass moments of inertia.
- `data/geometry.xml` — fuselage, wing, horizontal-tail, and vertical-tail geometry.
- `data/controls.xml` — control-surface dimensions and pilot control travel.
- `data/control_response.xml` — extracted hover control sensitivity and damping values.
- `data/rotor.xml` — rotor geometry, blade properties, rotor speeds, and drivetrain ratios.
- `data/engine.xml` — engine description, power ratings, and specific fuel consumption.
- `data/relationships.xml` — extracted equations and engineering relationships.
- `data/performance.xml` — performance inventory and summaries of report figures.
- `data/systems.xml` — qualitative system behaviour notes.
- `data/data_gaps.xml` — parameters not identified in the current extraction.
- `data/review_flags.xml` — unresolved discrepancies and verification tasks.

## How to read these files

Each file is a standalone XML document with the same `<xv15_dataset>` root metadata. The original section tag names, value attributes, source IDs, page/section pointers, and pointer comments have been retained. Some files contain more than one original section where the sections belong together.

The XML comments are short navigation aids; they do not verify a value or change its meaning. Values and notes have been split and sorted, not independently revalidated. In particular, keep the `review_flags.xml` issues unresolved until you check the cited source pages.

## Python example

```python
import xml.etree.ElementTree as ET

tree = ET.parse("data/mass.xml")
root = tree.getroot()

# Find the original mass-moments section and its airplane-mode roll value.
value = root.find(
    "./mass_moments_of_inertia/inertia[@axis='roll'][@mode='airplane']"
)

if value is not None:
    print(float(value.text))
```

Run the Python script from the folder that contains the `data` directory, or adjust the path accordingly.
