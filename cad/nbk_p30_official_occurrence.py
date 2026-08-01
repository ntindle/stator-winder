"""Immutable official NBK P30 occurrence and separate BNW review witnesses.

This module deliberately has no ``gen_step`` function.  It imports the
byte-identical vendor STEP as an in-memory assembly occurrence, applies only a
placement transform, and returns optional review witnesses as separate child
solids.  It never cuts, fuses, heals, or re-exports the CC BY-ND vendor model.

Coordinate contract
-------------------
The official source uses local +X as the shaft axis, with the tooth midplane at
local X=0 and the bore centered at local Y=Z=0.  ``place_for_m2`` maps local +X
to machine +Z.  ``center_xyz_mm`` is the world position of that local origin.

BNW boundary
------------
NBK publishes BNW as two additional set-screw holes at 90 degrees, but does
not publish this part's configured hole size, screw size, or axial station.
The returned M3x12 and hole-path solids are deliberately oversized upper-bound
witnesses.  They are positive, separately labeled geometry and are not holes
in the vendor occurrence.  Each screw witness may be translated inward along
its radial thread axis without changing its 12 mm upper-bound length.
Production still requires the configured NBK drawing/RFQ.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Iterable

from build123d import Align, Compound, Cylinder, Part, Pos, Rot, import_step


HERE = Path(__file__).resolve().parent
SOURCE_STEP = HERE / "models" / "upgrades" / "NBK_P30-3GT-BLP-6C-5_AP214.step"
SOURCE_STEP_SHA256 = "996449b7d9ec7703e7b38c6f75eff00a1174e3e1f088c05f0f1460b205169df9"

# Governing purchase-part properties from the NBK P30-3GT-BLP-6C-5 table.
OFFICIAL_MASS_G = 28.0
OFFICIAL_MASS_KG = OFFICIAL_MASS_G / 1000.0
OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2 = 3.0e-6

# Source geometry facts pinned by the provenance audit.
SOURCE_SHAFT_AXIS = (1.0, 0.0, 0.0)
MACHINE_SHAFT_AXIS = (0.0, 0.0, 1.0)
SOURCE_AXIAL_MIN_MM = -5.5
SOURCE_AXIAL_MAX_MM = 13.0
SOURCE_BORE_DIAMETER_MM = 5.0
SOURCE_FLANGE_DIAMETER_MM = 32.0

# Conservative review-only BNW envelopes.  These are not delivered hardware.
BNW_WITNESS_COUNT = 2
BNW_WITNESS_ANGLE_DEG = 90.0
BNW_WITNESS_DEFAULT_LOCAL_X_MM = 10.25
BNW_WITNESS_SCREW_DIAMETER_MM = 3.0
BNW_WITNESS_SCREW_LENGTH_MM = 12.0
BNW_WITNESS_SCREW_RADIAL_START_MM = 2.5
BNW_WITNESS_SCREW_RADIAL_END_MM = (
    BNW_WITNESS_SCREW_RADIAL_START_MM + BNW_WITNESS_SCREW_LENGTH_MM
)
BNW_WITNESS_HOLE_DIAMETER_MM = 3.2
BNW_WITNESS_HOLE_RADIAL_START_MM = 2.3
BNW_WITNESS_HOLE_RADIAL_END_MM = 16.5

STOCK_LABEL = "NBK_P30_3GT_BLP_6C_5_stock_split_clamp_vendor_occurrence"
REVIEW_ASSEMBLY_LABEL = "NBK_P30_stock_plus_separate_BNW_upper_bound_witnesses"


@dataclass(frozen=True)
class OfficialMassProperties:
    """NBK-table mass properties; never inferred from the union B-rep."""

    mass_g: float = OFFICIAL_MASS_G
    mass_kg: float = OFFICIAL_MASS_KG
    axial_moment_of_inertia_kg_m2: float = OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2
    authority: str = "NBK P30-3GT-BLP-6C-5 product table"


@dataclass
class PlacedP30Review:
    """Placed stock occurrence plus non-destructive BNW review geometry."""

    stock_occurrence: Part
    bnw_hole_witnesses: tuple[Part, Part]
    bnw_set_screw_witnesses: tuple[Part, Part]
    center_xyz_mm: tuple[float, float, float]
    stock_roll_deg: float
    bnw_first_azimuth_deg: float
    bnw_local_x_mm: float
    bnw_screw_inward_adjustments_mm: tuple[float, float]
    source_sha256_before: str
    source_sha256_after: str

    @property
    def official_mass_properties(self) -> OfficialMassProperties:
        return OfficialMassProperties()

    def review_assembly(self) -> Compound:
        """Return a labeled compound without booleaning any child together."""

        children = [
            self.stock_occurrence,
            *self.bnw_hole_witnesses,
            *self.bnw_set_screw_witnesses,
        ]
        result = Compound(children=children)
        result.label = REVIEW_ASSEMBLY_LABEL
        return result

    def parts_by_role(self) -> dict[str, Part]:
        """Stable integration keys for downstream assembly code."""

        return {
            "stock_pulley": self.stock_occurrence,
            "bnw_hole_witness_0": self.bnw_hole_witnesses[0],
            "bnw_hole_witness_1": self.bnw_hole_witnesses[1],
            "bnw_set_screw_witness_0": self.bnw_set_screw_witnesses[0],
            "bnw_set_screw_witness_1": self.bnw_set_screw_witnesses[1],
        }


def source_sha256() -> str:
    """Return the current source digest without mutating or opening the STEP."""

    return hashlib.sha256(SOURCE_STEP.read_bytes()).hexdigest()


def _finite_triplet(values: Iterable[float]) -> tuple[float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError("center_xyz_mm must contain exactly three finite values")
    return result  # type: ignore[return-value]


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _finite_nonnegative_pair(
    values: Iterable[float], name: str,
) -> tuple[float, float]:
    result = tuple(float(value) for value in values)
    if (
        len(result) != BNW_WITNESS_COUNT
        or not all(math.isfinite(value) for value in result)
        or any(value < 0.0 for value in result)
        or any(value >= BNW_WITNESS_SCREW_RADIAL_START_MM for value in result)
    ):
        raise ValueError(
            f"{name} must contain two finite values in "
            f"[0,{BNW_WITNESS_SCREW_RADIAL_START_MM})"
        )
    return result  # type: ignore[return-value]


def _radial_cylinder(
    *,
    center_xyz_mm: tuple[float, float, float],
    axial_local_x_mm: float,
    azimuth_deg: float,
    diameter_mm: float,
    radial_start_mm: float,
    radial_end_mm: float,
    label: str,
) -> Part:
    """Make a +X radial cylinder, rotate about machine Z, then place it."""

    length = radial_end_mm - radial_start_mm
    radial_mid = (radial_start_mm + radial_end_mm) / 2.0
    local = Pos(radial_mid, 0.0, axial_local_x_mm) * (
        Rot(0.0, 90.0, 0.0)
        * Cylinder(
            diameter_mm / 2.0,
            length,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
    )
    result = Pos(*center_xyz_mm) * (Rot(0.0, 0.0, azimuth_deg) * local)
    result.label = label
    return result


def _bnw_witness_pair(
    *,
    center_xyz_mm: tuple[float, float, float],
    axial_local_x_mm: float,
    first_azimuth_deg: float,
    kind: str,
    screw_inward_adjustments_mm: tuple[float, float] = (0.0, 0.0),
) -> tuple[Part, Part]:
    if kind == "hole":
        diameter = BNW_WITNESS_HOLE_DIAMETER_MM
        radial_start = BNW_WITNESS_HOLE_RADIAL_START_MM
        radial_end = BNW_WITNESS_HOLE_RADIAL_END_MM
        label_base = "NBK_BNW_unreleased_M3_upper_bound_hole_path_witness"
    elif kind == "screw":
        diameter = BNW_WITNESS_SCREW_DIAMETER_MM
        radial_start = BNW_WITNESS_SCREW_RADIAL_START_MM
        radial_end = BNW_WITNESS_SCREW_RADIAL_END_MM
        label_base = "NBK_BNW_unreleased_M3x12_set_screw_envelope_witness"
    else:  # pragma: no cover - internal contract
        raise ValueError(f"unsupported witness kind: {kind}")

    result: list[Part] = []
    for index in range(BNW_WITNESS_COUNT):
        inward = (
            screw_inward_adjustments_mm[index]
            if kind == "screw"
            else 0.0
        )
        result.append(_radial_cylinder(
            center_xyz_mm=center_xyz_mm,
            axial_local_x_mm=axial_local_x_mm,
            azimuth_deg=first_azimuth_deg + index * BNW_WITNESS_ANGLE_DEG,
            diameter_mm=diameter,
            radial_start_mm=radial_start - inward,
            radial_end_mm=radial_end - inward,
            label=f"{label_base}_{index}",
        ))
    return tuple(result)  # type: ignore[return-value]


def place_for_m2(
    center_xyz_mm: Iterable[float],
    *,
    stock_roll_deg: float = 0.0,
    bnw_first_azimuth_deg: float = 0.0,
    bnw_local_x_mm: float = BNW_WITNESS_DEFAULT_LOCAL_X_MM,
    bnw_screw_inward_adjustments_mm: Iterable[float] = (0.0, 0.0),
) -> PlacedP30Review:
    """Place the immutable stock pulley on M2 and add separate BNW witnesses.

    ``stock_roll_deg`` rotates the stock split-clamp details around machine Z.
    ``bnw_first_azimuth_deg`` independently indexes the first review witness;
    the second is exactly 90 degrees away.  ``bnw_local_x_mm`` is an assumed
    source-axis station and remains an unreleased review parameter.
    ``bnw_screw_inward_adjustments_mm`` translates each full-length screw
    witness inward along its radial thread axis; it never modifies the stock
    occurrence or the separate hole paths.
    """

    center = _finite_triplet(center_xyz_mm)
    stock_roll = _finite(stock_roll_deg, "stock_roll_deg")
    bnw_first = _finite(bnw_first_azimuth_deg, "bnw_first_azimuth_deg")
    bnw_station = _finite(bnw_local_x_mm, "bnw_local_x_mm")
    screw_adjustments = _finite_nonnegative_pair(
        bnw_screw_inward_adjustments_mm,
        "bnw_screw_inward_adjustments_mm",
    )
    if not SOURCE_AXIAL_MIN_MM <= bnw_station <= SOURCE_AXIAL_MAX_MM:
        raise ValueError("bnw_local_x_mm must lie inside the stock axial envelope")

    digest_before = source_sha256()
    if digest_before != SOURCE_STEP_SHA256:
        raise RuntimeError("official NBK STEP no longer matches its pinned source hash")

    source = import_step(str(SOURCE_STEP))
    if len(source.solids()) != 1:
        raise RuntimeError("official NBK STEP must import as one solid")
    stock = Pos(*center) * (
        Rot(0.0, 0.0, stock_roll)
        * (Rot(0.0, -90.0, 0.0) * source)
    )
    stock.label = STOCK_LABEL

    holes = _bnw_witness_pair(
        center_xyz_mm=center,
        axial_local_x_mm=bnw_station,
        first_azimuth_deg=bnw_first,
        kind="hole",
    )
    screws = _bnw_witness_pair(
        center_xyz_mm=center,
        axial_local_x_mm=bnw_station,
        first_azimuth_deg=bnw_first,
        kind="screw",
        screw_inward_adjustments_mm=screw_adjustments,
    )

    digest_after = source_sha256()
    if digest_after != digest_before:
        raise RuntimeError("official NBK STEP changed during occurrence construction")

    return PlacedP30Review(
        stock_occurrence=stock,
        bnw_hole_witnesses=holes,
        bnw_set_screw_witnesses=screws,
        center_xyz_mm=center,
        stock_roll_deg=stock_roll,
        bnw_first_azimuth_deg=bnw_first,
        bnw_local_x_mm=bnw_station,
        bnw_screw_inward_adjustments_mm=screw_adjustments,
        source_sha256_before=digest_before,
        source_sha256_after=digest_after,
    )


__all__ = [
    "BNW_WITNESS_DEFAULT_LOCAL_X_MM",
    "BNW_WITNESS_HOLE_DIAMETER_MM",
    "BNW_WITNESS_SCREW_DIAMETER_MM",
    "BNW_WITNESS_SCREW_LENGTH_MM",
    "MACHINE_SHAFT_AXIS",
    "OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2",
    "OFFICIAL_MASS_G",
    "OFFICIAL_MASS_KG",
    "OfficialMassProperties",
    "PlacedP30Review",
    "SOURCE_STEP",
    "SOURCE_STEP_SHA256",
    "place_for_m2",
    "source_sha256",
]
