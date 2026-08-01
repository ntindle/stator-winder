"""Immutable official NBK P30-3GT-BLP-6C-10 flyer occurrence.

The downloaded CC-BY-ND STEP is imported byte-for-byte and only transformed
as an assembly occurrence.  This module has no STEP export function.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Iterable

from build123d import Part, Pos, Rot, import_step


HERE = Path(__file__).resolve().parent
SOURCE_STEP = (
    HERE / "models" / "upgrades" / "NBK_P30_D10_download" /
    "P30-3GT-BLP-6C-10.stp"
)
SOURCE_STEP_SHA256 = (
    "780110e1d59a988661f5ae80e9ebbe5d2eb324b9037d33a481809c939fa4c9f1"
)
SOURCE_STEP_BYTES = 57130
CADENAS_ORDER_ID = "22026071121383341311079d0b6156e"
CADENAS_EXPRESSION = "{CN=P30-3GT-BLP-6C},{D=10}"
PRODUCT_URL = (
    "https://www.nbk1560.com/products/pulley/timingpulley/"
    "3GT-BLP-6C/P30-3GT-BLP-6C/"
)

OFFICIAL_PART_NUMBER = "P30-3GT-BLP-6C-10"
OFFICIAL_MASS_G = 28.0
OFFICIAL_MASS_KG = 0.028
OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2 = 3.0e-6
OFFICIAL_CLAMP_BOLT = "M2 socket-head bolt supplied with pulley"
OFFICIAL_CLAMP_TORQUE_NM = 0.5
OFFICIAL_SHAFT_TOLERANCE = "h6 or h7"
OFFICIAL_BODY_MATERIAL = "A2017"
OFFICIAL_BOLT_MATERIAL = "SCM435 black oxide"

SOURCE_SHAFT_AXIS = (1.0, 0.0, 0.0)
MACHINE_SHAFT_AXIS_HUB_REAR = (0.0, 0.0, -1.0)
SOURCE_AXIAL_MIN_MM = -5.5
SOURCE_AXIAL_MAX_MM = 13.0
SOURCE_BORE_DIAMETER_MM = 10.0
SOURCE_FLANGE_DIAMETER_MM = 32.0
SOURCE_TOOTH_ENVELOPE_DIAMETER_MM = 27.9
SOURCE_TOOTH_BAND_WIDTH_MM = 7.3
STOCK_CLAMP_LENGTH_MM = 7.5
STOCK_LABEL = "NBK_P30_3GT_BLP_6C_10_stock_hub_rear_vendor_occurrence"


@dataclass(frozen=True)
class OfficialMassProperties:
    mass_g: float = OFFICIAL_MASS_G
    mass_kg: float = OFFICIAL_MASS_KG
    axial_moment_of_inertia_kg_m2: float = (
        OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2
    )
    authority: str = "NBK P30-3GT-BLP-6C product table at maximum bore"


@dataclass
class PlacedD10:
    stock_occurrence: Part
    center_xyz_mm: tuple[float, float, float]
    stock_roll_deg: float
    source_sha256_before: str
    source_sha256_after: str

    @property
    def official_mass_properties(self) -> OfficialMassProperties:
        return OfficialMassProperties()


def source_sha256() -> str:
    return hashlib.sha256(SOURCE_STEP.read_bytes()).hexdigest()


def _finite_triplet(values: Iterable[float]) -> tuple[float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError("center_xyz_mm must contain three finite values")
    return result  # type: ignore[return-value]


def import_official() -> Part:
    digest = source_sha256()
    if digest != SOURCE_STEP_SHA256:
        raise RuntimeError("official NBK D10 STEP hash drift")
    if SOURCE_STEP.stat().st_size != SOURCE_STEP_BYTES:
        raise RuntimeError("official NBK D10 STEP byte-count drift")
    source = import_step(str(SOURCE_STEP))
    if len(source.solids()) != 1 or not source.is_valid:
        raise RuntimeError("official NBK D10 STEP must be one valid solid")
    return source


def place_hub_rear(
    center_xyz_mm: Iterable[float], *, stock_roll_deg: float = 0.0
) -> PlacedD10:
    """Map source +X to machine -Z without modifying source geometry."""

    center = _finite_triplet(center_xyz_mm)
    roll = float(stock_roll_deg)
    if not math.isfinite(roll):
        raise ValueError("stock_roll_deg must be finite")
    digest_before = source_sha256()
    source_mtime = SOURCE_STEP.stat().st_mtime_ns
    source = import_official()
    stock = Pos(*center) * (
        Rot(0.0, 0.0, roll) * (Rot(0.0, 90.0, 0.0) * source)
    )
    stock.label = STOCK_LABEL
    digest_after = source_sha256()
    if digest_after != digest_before or SOURCE_STEP.stat().st_mtime_ns != source_mtime:
        raise RuntimeError("official NBK D10 STEP changed during placement")
    return PlacedD10(
        stock_occurrence=stock,
        center_xyz_mm=center,
        stock_roll_deg=roll,
        source_sha256_before=digest_before,
        source_sha256_after=digest_after,
    )


__all__ = [
    "CADENAS_EXPRESSION",
    "CADENAS_ORDER_ID",
    "MACHINE_SHAFT_AXIS_HUB_REAR",
    "OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2",
    "OFFICIAL_CLAMP_BOLT",
    "OFFICIAL_CLAMP_TORQUE_NM",
    "OFFICIAL_MASS_G",
    "OFFICIAL_MASS_KG",
    "OFFICIAL_PART_NUMBER",
    "OFFICIAL_SHAFT_TOLERANCE",
    "OfficialMassProperties",
    "PRODUCT_URL",
    "PlacedD10",
    "SOURCE_AXIAL_MAX_MM",
    "SOURCE_AXIAL_MIN_MM",
    "SOURCE_BORE_DIAMETER_MM",
    "SOURCE_STEP",
    "SOURCE_STEP_BYTES",
    "SOURCE_STEP_SHA256",
    "STOCK_CLAMP_LENGTH_MM",
    "import_official",
    "place_hub_rear",
    "source_sha256",
]
