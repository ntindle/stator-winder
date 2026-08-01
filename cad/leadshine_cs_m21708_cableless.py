"""Provenance-bound cable-free derivative of Leadshine CS-M21708 CAD.

The official manufacturer STEP contains the exact motor BREP plus long cable
and free-connector geometry.  The cable geometry is not useful in the winder
assembly because its installed routing is not yet defined.  This module keeps
the manufacturer's solid motor bodies, removes only the side cable/connector
solids that extend outside the 42.3 mm NEMA-17 frame, and rebases the exact
mounting face to ``z=0`` with the output shaft pointing toward ``+z``.

This is a filtered vendor model, not a dimensioned envelope.  No retained
solid is remodeled or scaled.  The exposed shaft remains the manufacturer's
5 mm D-profile (4.5 mm across the flat), which is an intentionally open
pulley-retention gate for the normal GOAL drive selection.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from build123d import Align, Box, Compound, Part, Pos, Solid, import_step, export_step


CAD_DIR = Path(__file__).resolve().parent
ROOT = CAD_DIR.parent
SOURCE_STEP = CAD_DIR / "models" / "upgrades" / "CS-M21708.STEP"
OUTPUT_STEP = CAD_DIR / "models" / "upgrades" / "CS-M21708_cableless.step"
REPORT_PATH = ROOT / "out" / "reports" / "leadshine_cs_m21708_cableless.json"

PRODUCT_URL = (
    "https://www.leadshine.com/product-detail/closed-loop-stepper-drive/"
    "closed-loop-stepper/CS-M21708.html"
)
SOURCE_ARCHIVE_URL = (
    "https://www.leadshine.com/upfiles/downloads/"
    "3c4df9dc7e3237fbdafee94f4142513a_1651890419697.zip"
)
SOURCE_SHA256 = "7e995e724fc7e019278e0a919ba1db8c8abb3333f156c64eb6e62485e0f6662b"

# Coordinates measured from exact major planar faces in the vendor STEP.
SOURCE_MOUNT_FACE_Z_MM = -252.315163
SOURCE_NOMINAL_REAR_Z_MM = -335.315163
SOURCE_EXACT_REAR_Z_MM = -335.515163
MOUNT_REBASE_Z_MM = -SOURCE_MOUNT_FACE_Z_MM

FRAME_HALF_MM = 21.15
FILTER_TOL_MM = 1.0e-6
EXPECTED_SOURCE_SOLIDS = 27
EXPECTED_RETAINED_SOLIDS = 18
EXPECTED_DROPPED_SOLID_INDICES = (4, 5, 9, 10, 11, 18, 19, 20, 21)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_solids():
    if _sha256(SOURCE_STEP) != SOURCE_SHA256:
        raise RuntimeError("CS-M21708 source STEP hash does not match the approved vendor file")
    model = import_step(SOURCE_STEP)
    solids = list(model.solids())
    if len(solids) != EXPECTED_SOURCE_SOLIDS:
        raise RuntimeError(
            f"CS-M21708 source solid count changed: {len(solids)} != "
            f"{EXPECTED_SOURCE_SOLIDS}"
        )
    return solids


def _partition_source_solids():
    """Split exact motor solids from side cable/connector solids.

    The vendor's NEMA-17 frame is exactly x/y=+/-21.15 mm.  Cable/connector
    solids are the source solids whose +Y bound exceeds that documented frame
    plane.  Wire/surface-only cable entities are inherently excluded by the
    explicit ``solids()`` selection.
    """
    retained = []
    dropped = []
    for index, solid in enumerate(_source_solids(), start=1):
        if solid.bounding_box().max.Y > FRAME_HALF_MM + FILTER_TOL_MM:
            dropped.append((index, solid))
        else:
            retained.append((index, solid))
    dropped_indices = tuple(index for index, _ in dropped)
    if dropped_indices != EXPECTED_DROPPED_SOLID_INDICES:
        raise RuntimeError(
            "CS-M21708 cable/connector partition changed: "
            f"{dropped_indices} != {EXPECTED_DROPPED_SOLID_INDICES}"
        )
    if len(retained) != EXPECTED_RETAINED_SOLIDS:
        raise RuntimeError("CS-M21708 retained-solid count changed")
    return retained, dropped


def gen_step() -> Part:
    """Return the exact retained vendor solids in the winder mount frame."""
    retained, _ = _partition_source_solids()
    # Apply the rebase to every retained child.  A location carried only by a
    # parent Compound is visible to bounding-box queries but can be omitted by
    # some STEP assembly exporters when they serialize the child occurrences.
    result = Compound.make_composite(
        [
            # Detach the sub-solid from the imported source Compound before
            # placing it.  This prevents the viewer hierarchy exporter from
            # following ``topo_parent`` back to discarded cable geometry.
            Pos(0.0, 0.0, MOUNT_REBASE_Z_MM) * Solid(solid.wrapped)
            for _, solid in retained
        ]
    )
    result.label = "Leadshine_CS-M21708_cableless_exact_vendor_body"
    return result


def _bounds(shape) -> dict[str, list[float]]:
    box = shape.bounding_box()
    return {
        "min_mm": [box.min.X, box.min.Y, box.min.Z],
        "max_mm": [box.max.X, box.max.Y, box.max.Z],
        "size_mm": [box.size.X, box.size.Y, box.size.Z],
    }


def _shaft_section_area_mm2(shape, z_mm: float) -> float:
    thickness_mm = 0.01
    slab = Pos(0.0, 0.0, z_mm - thickness_mm / 2.0) * Box(
        8.0,
        8.0,
        thickness_mm,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    return (shape & slab).volume / thickness_mm


def audit() -> dict:
    retained, dropped = _partition_source_solids()
    shape = gen_step()
    # Source solid 27 is the exact rotor/shaft BREP.  Section that solid alone;
    # intersecting a non-fused multi-solid compound can suppress the section in
    # some OpenCascade builds.
    shaft = Pos(0.0, 0.0, MOUNT_REBASE_Z_MM) * _source_solids()[26]
    bounds = _bounds(shape)
    round_section_area = _shaft_section_area_mm2(shaft, -2.0)
    exposed_section_area = _shaft_section_area_mm2(shaft, 17.0)
    return {
        "artifact": "Leadshine CS-M21708 cableless exact-vendor-body derivative",
        "source": {
            "product_url": PRODUCT_URL,
            "archive_url": SOURCE_ARCHIVE_URL,
            "step_path": str(SOURCE_STEP.relative_to(ROOT)).replace("\\", "/"),
            "step_sha256": SOURCE_SHA256,
            "solid_count": EXPECTED_SOURCE_SOLIDS,
        },
        "filter": {
            "rule": "drop source solids with max_y > +21.15 mm frame plane",
            "retained_solid_indices_1_based": [index for index, _ in retained],
            "dropped_cable_connector_solid_indices_1_based": [
                index for index, _ in dropped
            ],
            "retained_solid_count": len(retained),
            "dropped_solid_count": len(dropped),
            "retained_geometry_remodeled": False,
        },
        "mount_frame": {
            "mount_face_z_mm": 0.0,
            "shaft_direction": "+Z",
            "shaft_tip_z_mm": 24.0,
            "nominal_body_rear_z_mm": SOURCE_NOMINAL_REAR_Z_MM + MOUNT_REBASE_Z_MM,
            "exact_feature_rear_z_mm": SOURCE_EXACT_REAR_Z_MM + MOUNT_REBASE_Z_MM,
            "bounds": bounds,
        },
        "shaft_interface": {
            "profile": "D",
            "diameter_mm": 5.0,
            "across_flat_mm": 4.5,
            "internal_round_section_area_mm2": round_section_area,
            "exposed_d_section_area_mm2": exposed_section_area,
            "stock_round_bore_split_clamp_authorized": False,
        },
        "gates": {
            "source_hash": _sha256(SOURCE_STEP) == SOURCE_SHA256,
            "solid_partition": (
                tuple(index for index, _ in dropped)
                == EXPECTED_DROPPED_SOLID_INDICES
            ),
            "mount_face_rebased": abs(bounds["max_mm"][2] - 24.0) < 1.0e-3,
            "frame_xy": (
                abs(bounds["size_mm"][0] - 42.3) < 1.0e-3
                and abs(bounds["size_mm"][1] - 42.3) < 1.0e-3
            ),
            "d_shaft_proven": exposed_section_area < round_section_area - 0.5,
        },
    }


def main() -> None:
    shape = gen_step()
    OUTPUT_STEP.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    export_step(shape, OUTPUT_STEP)
    REPORT_PATH.write_text(json.dumps(audit(), indent=2) + "\n", encoding="utf-8")
    print(OUTPUT_STEP)
    print(REPORT_PATH)


if __name__ == "__main__":
    main()
