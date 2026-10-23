"""Load and validate the XV-15 XML dataset.

This module deliberately does not interpret aircraft-specific parameters or
fill missing values. It loads XML documents and reports structural problems.
Uses only Python's standard library.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass
class LoadReport:
    """Summary of what was loaded and any problems found."""

    loaded_files: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        """True when no errors occurred. Warnings may still be present."""
        return not self.errors


@dataclass
class AircraftData:
    """Parsed XML documents, indexed by path relative to the data folder."""

    documents: dict[str, ET.ElementTree]
    report: LoadReport

    def root(self, relative_path: str) -> ET.Element:
        """Return an XML document's root element.

        Example: data.root("inertia.xml")
        Nested files can be addressed by relative path, e.g.
        "review/re-interation/data_gaps.xml".
        """
        key = _normalise_relative_path(relative_path)
        try:
            return self.documents[key].getroot()
        except KeyError as exc:
            available = ", ".join(sorted(self.documents)) or "(none)"
            raise KeyError(
                f"XML file {key!r} was not loaded. Available files: {available}"
            ) from exc

    def find_text(
        self,
        relative_path: str,
        element_path: str,
        *,
        required: bool = False,
    ) -> str | None:
        """Find an XML element and return its trimmed text.

        `element_path` is an ElementTree path relative to the document root.
        This is a generic helper; callers must use paths that match the
        actual XML schema. Missing optional elements return None.
        """
        element = self.root(relative_path).find(element_path)
        value = element.text.strip() if element is not None and element.text else None

        if value is None and required:
            raise ValueError(
                f"Required XML value not found: file={relative_path!r}, "
                f"element_path={element_path!r}"
            )
        return value


def _normalise_relative_path(path: str) -> str:
    """Use consistent forward-slash keys on every operating system."""
    return Path(path).as_posix().lstrip("./")


def load_aircraft_data(data_dir: str | Path) -> AircraftData:
    """Load every .xml file under `data_dir`, including nested folders.

    Invalid XML files are reported and skipped. Missing directories and
    directories with no XML files are errors. No data values are inferred.
    """
    directory = Path(data_dir)
    report = LoadReport()
    documents: dict[str, ET.ElementTree] = {}

    if not directory.exists():
        report.errors.append(f"Data directory does not exist: {directory}")
        return AircraftData(documents, report)

    if not directory.is_dir():
        report.errors.append(f"Data path is not a directory: {directory}")
        return AircraftData(documents, report)

    xml_files = sorted(
        (path for path in directory.rglob("*") if path.is_file() and path.suffix.lower() == ".xml"),
        key=lambda path: path.as_posix().lower(),
    )

    if not xml_files:
        report.errors.append(f"No XML files found in: {directory}")
        return AircraftData(documents, report)

    for file_path in xml_files:
        relative_path = file_path.relative_to(directory).as_posix()

        try:
            tree = ET.parse(file_path)
        except (ET.ParseError, OSError) as exc:
            report.errors.append(f"{relative_path}: could not parse XML ({exc})")
            continue

        root_element = tree.getroot()
        if not root_element.tag:
            report.warnings.append(f"{relative_path}: XML root has no tag name")

        documents[relative_path] = tree
        report.loaded_files.append(relative_path)

    if report.errors:
        report.warnings.append(
            "Some files could not be loaded. Do not treat the dataset as fully "
            "validated until those errors are reviewed."
        )

    return AircraftData(documents, report)


def print_load_report(data: AircraftData) -> None:
    """Print a readable summary for development and debugging."""
    print(f"XML files loaded: {len(data.report.loaded_files)}")

    for relative_path in data.report.loaded_files:
        root_tag = data.documents[relative_path].getroot().tag
        print(f"  OK  {relative_path} (root: <{root_tag}>)")

    for warning in data.report.warnings:
        print(f"WARNING: {warning}")

    for error in data.report.errors:
        print(f"ERROR: {error}")


if __name__ == "__main__":
    # Running this file directly checks the project's data/ directory.
    # Expected layout: project_root/engine/data_loader.py and project_root/data/
    project_root = Path(__file__).resolve().parent.parent
    aircraft_data = load_aircraft_data(project_root / "data")
    print_load_report(aircraft_data)

    if not aircraft_data.report.ok:
        raise SystemExit(1)
