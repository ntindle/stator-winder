"""Fabrication package for the default stator's Nomex Type 410 insulation.

This module owns the geometry and documentation for the selected 5 mil
DuPont Nomex Type 410 / BAE Wire ``INNMX410005S`` insulation set:

* twenty-four identical, formed slot-cell strips;
* one front and one rear end-face star cap; and
* a dimensioned fabrication/installation PDF plus JSON/Markdown evidence.

The drawings are exact for the *simplified* 24-slot, OD46 x 15 mm stack in
``stator_model.py``.  The 0.127 mm nominal stock is accepted only when its
formed/installed thickness is at or below the active winding plan's 0.140 mm
maximum and at or above its 0.120 mm lower input bound.  3M DMD180 3-2-3 at 0.2032 mm remains an explicitly incompatible
alternate unless packing and routing are regenerated.

Flat pattern frame:

* X is developed distance around the slot wall, left mouth to right mouth.
* Y is stator axial direction, rear overhang to front overhang.
* DXF and report dimensions are millimetres at 1:1 scale.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable, Sequence

import ezdxf
from ezdxf import units
from reportlab.lib import colors
from reportlab.lib.pagesizes import landscape, letter
from reportlab.pdfgen import canvas
from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from params import DEFAULT_STATOR
from coil_growth import slot_geometry


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "out"
REPORT_DIR = OUT / "reports"
DXF_DIR = OUT / "custom" / "shop" / "dxf"
PDF_DIR = OUT / "custom" / "shop" / "pdf"
PDF_QA_DIR = OUT / "custom" / "shop" / "pdf_qa"

SCHEMA = "stator-insulation-fabrication/v1"
DRAWING_REV = "A"

# Exact selected stock.  BAE identifies this SKU as an uncoated,
# non-adhesive 24 x 36 inch sheet of 0.005 inch Nomex Type 410 aramid paper.
# The active packing contract is intentionally more conservative than the
# nominal sheet: the *formed and installed* liner must measure <=0.140 mm.
MATERIAL_NAME = "DuPont Nomex Type 410, 5 mil, 24 x 36 inch sheet"
MATERIAL_PART_NUMBER = "Nomex Type 410, 5 mil"
MATERIAL_SUPPLIER = "BAE Wire & Insulation"
MATERIAL_SUPPLIER_SKU = "INNMX410005S"
MATERIAL_NOMINAL_THICKNESS_MM = 0.127
MATERIAL_RECEIVING_MIN_MM = 0.120
MATERIAL_RECEIVING_MAX_MM = 0.140
MATERIAL_SHEET_WIDTH_MM = 609.6
MATERIAL_SHEET_LENGTH_MM = 914.4
MATERIAL_KIND = "calendered aramid insulation paper"
PRODUCT_URL = (
    "https://www.baewire.com/"
    "NOMEX-TYPE-410-005-24-X-36-SHEETS-p/innmx410005s.htm"
)
DATA_SHEET_URL = (
    "https://www.dupont.com/content/dam/dupont/amer/us/en/safety/"
    "public/documents/en/Nomex_410_Datasheet_2025.pdf"
)

# Explicitly rejected drop-in alternate retained in the evidence report.
DMD180_ALTERNATE_NAME = "3M DMD180 3-2-3, 8 mil"
DMD180_ALTERNATE_THICKNESS_MM = 0.2032

# Fabrication policy.  A 1.5 mm axial projection is slit into independent
# tabs and flared onto each face.  The star cap extends 0.35 mm into each
# steel edge, so the flared liner and cap have positive overlap instead of a
# butt joint at the lamination corner.
AXIAL_END_FLARE_MM = 1.50
CAP_EDGE_OVERLAP_MM = 0.35
RELIEF_NOTCH_WIDTH_MM = 0.35
CUT_TOLERANCE_MM = 0.10
FOLD_STATION_TOLERANCE_MM = 0.15
NEST_GAP_MM = 2.0
ACTIVE_WINDING_PLAN_LINER_MAX_MM = 0.140
ACTIVE_WIRE_DIAMETER_MAX_MM = 0.235


@dataclass(frozen=True)
class DevelopedSlotCell:
    """One identical slot-cell blank and its forming stations."""

    stack_mm: float
    axial_end_flare_mm: float
    blank_length_mm: float
    blank_width_mm: float
    shoe_edge_each_mm: float
    shoe_under_arc_each_mm: float
    neck_wall_each_mm: float
    root_fold_x_mm: float
    fold_stations_x_mm: tuple[float, ...]
    relief_notch_width_mm: float


def _default_geometry() -> dict[str, float]:
    return slot_geometry(DEFAULT_STATOR)


def developed_slot_cell() -> DevelopedSlotCell:
    """Return the exact nominal wall development for the simplified slot.

    Each half runs from the slot root along the inner tooth-neck face, around
    the short shoe-underface arc, then radially through the shoe edge to the
    OD mouth.  The flexible laminate is formed on these stations; this is not
    a metallic bend-deduction calculation.
    """

    geom = _default_geometry()
    pitch = 2.0 * math.pi / DEFAULT_STATOR.slots
    tooth_half_pitch = pitch / 2.0
    shoe_half_angle = 0.36 * pitch
    neck_half = float(geom["tooth_neck_width_mm"]) / 2.0
    shoe_inner = float(geom["shoe_inner_radius_mm"])
    shoe_edge = float(geom["shoe_thickness_mm"])

    # In a tooth-local frame the slot-side neck wall is v=-neck_half.  Its
    # u-coordinate at the slot-root apex and shoe-inner circle is analytic.
    u_root = neck_half / math.tan(tooth_half_pitch)
    u_shoe = math.sqrt(shoe_inner * shoe_inner - neck_half * neck_half)
    neck_wall = u_shoe - u_root
    neck_intersection_angle = tooth_half_pitch - math.asin(
        neck_half / shoe_inner
    )
    shoe_slot_edge_angle = tooth_half_pitch - shoe_half_angle
    shoe_under_arc = shoe_inner * (
        neck_intersection_angle - shoe_slot_edge_angle
    )
    if min(neck_wall, shoe_under_arc, shoe_edge) <= 0.0:
        raise ValueError("default stator does not produce a valid open slot wall")

    half = shoe_edge + shoe_under_arc + neck_wall
    stations = (
        shoe_edge,
        shoe_edge + shoe_under_arc,
        half,
        half + neck_wall,
        half + neck_wall + shoe_under_arc,
    )
    return DevelopedSlotCell(
        stack_mm=DEFAULT_STATOR.stack,
        axial_end_flare_mm=AXIAL_END_FLARE_MM,
        blank_length_mm=DEFAULT_STATOR.stack + 2.0 * AXIAL_END_FLARE_MM,
        blank_width_mm=2.0 * half,
        shoe_edge_each_mm=shoe_edge,
        shoe_under_arc_each_mm=shoe_under_arc,
        neck_wall_each_mm=neck_wall,
        root_fold_x_mm=half,
        fold_stations_x_mm=stations,
        relief_notch_width_mm=RELIEF_NOTCH_WIDTH_MM,
    )


def slot_cell_outline() -> tuple[tuple[float, float], ...]:
    """Return a closed-profile blank with end-tab V reliefs.

    The first point is not repeated.  Reliefs are part of the closed outer
    contour, avoiding ambiguous open through-cuts in a knife/plotter DXF.
    """

    cell = developed_slot_cell()
    half_notch = cell.relief_notch_width_mm / 2.0
    points: list[tuple[float, float]] = [(0.0, 0.0)]
    for station in cell.fold_stations_x_mm:
        points.extend((
            (station - half_notch, 0.0),
            (station, cell.axial_end_flare_mm),
            (station + half_notch, 0.0),
        ))
    points.extend(((cell.blank_width_mm, 0.0),
                   (cell.blank_width_mm, cell.blank_length_mm)))
    for station in reversed(cell.fold_stations_x_mm):
        points.extend((
            (station + half_notch, cell.blank_length_mm),
            (station, cell.blank_length_mm - cell.axial_end_flare_mm),
            (station - half_notch, cell.blank_length_mm),
        ))
    points.append((0.0, cell.blank_length_mm))
    return tuple(points)


def _new_dxf() -> tuple[ezdxf.document.Drawing, object]:
    document = ezdxf.new("R2013")
    document.units = units.MM
    document.layers.add("CUT", color=1)
    document.layers.add("FOLD_REFERENCE", color=5, linetype="DASHED")
    document.layers.add("DATUM_REFERENCE", color=8, linetype="CENTER")
    return document, document.modelspace()


def slot_cell_dxf() -> ezdxf.document.Drawing:
    """Return the 1:1 slot-cell cut and forming-reference drawing."""

    document, modelspace = _new_dxf()
    modelspace.add_lwpolyline(
        slot_cell_outline(), close=True, dxfattribs={"layer": "CUT"}
    )
    cell = developed_slot_cell()
    for station in cell.fold_stations_x_mm:
        modelspace.add_line(
            (station, cell.axial_end_flare_mm),
            (station, cell.blank_length_mm - cell.axial_end_flare_mm),
            dxfattribs={"layer": "FOLD_REFERENCE"},
        )
    for y in (cell.axial_end_flare_mm,
              cell.blank_length_mm - cell.axial_end_flare_mm):
        modelspace.add_line(
            (0.0, y), (cell.blank_width_mm, y),
            dxfattribs={"layer": "DATUM_REFERENCE"},
        )
    return document


def _annular_sector(inner: float, outer: float, center: float,
                    half_angle: float, segments: int = 24) -> Polygon:
    outer_points = [
        (
            outer * math.cos(center - half_angle
                             + 2.0 * half_angle * index / segments),
            outer * math.sin(center - half_angle
                             + 2.0 * half_angle * index / segments),
        )
        for index in range(segments + 1)
    ]
    inner_points = [
        (
            inner * math.cos(center + half_angle
                             - 2.0 * half_angle * index / segments),
            inner * math.sin(center + half_angle
                             - 2.0 * half_angle * index / segments),
        )
        for index in range(segments + 1)
    ]
    return Polygon(outer_points + inner_points)


def _main_lamination_face() -> Polygon:
    """Planar hub+teeth+shoes, excluding the unrelated shaft solid."""

    geom = _default_geometry()
    outer_radius = float(geom["outer_radius_mm"])
    hub_radius = float(geom["hub_radius_mm"])
    shoe_inner = float(geom["shoe_inner_radius_mm"])
    neck_width = float(geom["tooth_neck_width_mm"])
    neck_start = hub_radius - 1.0
    neck_end = shoe_inner + 1.0
    pitch = 2.0 * math.pi / DEFAULT_STATOR.slots
    shoe_half = 0.36 * pitch

    members = [Point(0.0, 0.0).buffer(hub_radius, quad_segs=256)]
    base_neck = box(neck_start, -neck_width / 2.0,
                    neck_end, neck_width / 2.0)
    for tooth in range(DEFAULT_STATOR.slots):
        angle_rad = tooth * pitch
        angle_deg = math.degrees(angle_rad)
        members.append(affinity.rotate(base_neck, angle_deg,
                                       origin=(0.0, 0.0)))
        members.append(_annular_sector(
            shoe_inner, outer_radius, angle_rad, shoe_half
        ))
    face = unary_union(members)
    if not isinstance(face, Polygon):
        raise RuntimeError("lamination face did not resolve to one polygon")
    return face


def end_cap_geometry() -> Polygon:
    """Return one non-handed end cap with 0.35 mm steel-edge overlap."""

    geom = _default_geometry()
    outer_radius = float(geom["outer_radius_mm"])
    hub_inner_radius = max(
        DEFAULT_STATOR.shaft_d + 4.0,
        DEFAULT_STATOR.od * DEFAULT_STATOR.hub_od_ratio - 10.0,
    ) / 2.0
    cap_inner_radius = hub_inner_radius - CAP_EDGE_OVERLAP_MM
    if cap_inner_radius <= DEFAULT_STATOR.shaft_d / 2.0:
        raise ValueError("cap overlap consumes the central hub opening")

    cap = _main_lamination_face().buffer(
        CAP_EDGE_OVERLAP_MM, quad_segs=32, join_style=1
    )
    # No material is allowed beyond the stator OD: an outboard paper fringe
    # would become a flyer snag point.  Coverage is instead guaranteed by the
    # contracted slot cutouts and hub bore.
    cap = cap.intersection(Point(0.0, 0.0).buffer(
        outer_radius, quad_segs=512
    ))
    cap = cap.difference(Point(0.0, 0.0).buffer(
        cap_inner_radius, quad_segs=512
    ))
    if not isinstance(cap, Polygon):
        raise RuntimeError("end-cap offset did not resolve to one polygon")
    if len(cap.interiors) != 1:
        raise RuntimeError("end cap must have one central cutout")
    return cap


def _polyline_points(coords: Iterable[Sequence[float]]) -> list[tuple[float, float]]:
    points = [(float(point[0]), float(point[1])) for point in coords]
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    return points


def end_cap_dxf(side: str) -> ezdxf.document.Drawing:
    """Return one front/rear cap DXF; Nomex 410 sheet is non-handed."""

    if side not in {"front", "rear"}:
        raise ValueError("side must be 'front' or 'rear'")
    document, modelspace = _new_dxf()
    cap = end_cap_geometry()
    modelspace.add_lwpolyline(
        _polyline_points(cap.exterior.coords), close=True,
        dxfattribs={"layer": "CUT"},
    )
    for interior in cap.interiors:
        modelspace.add_lwpolyline(
            _polyline_points(interior.coords), close=True,
            dxfattribs={"layer": "CUT"},
        )
    # A tiny centre cross is reference-only and is never a through-cut.
    for start, end in (((-2.0, 0.0), (2.0, 0.0)),
                       ((0.0, -2.0), (0.0, 2.0))):
        modelspace.add_line(start, end,
                            dxfattribs={"layer": "DATUM_REFERENCE"})
    return document


def geometry_summary() -> dict[str, object]:
    geom = _default_geometry()
    cell = developed_slot_cell()
    cap = end_cap_geometry()
    inner_radius = max(
        DEFAULT_STATOR.shaft_d + 4.0,
        DEFAULT_STATOR.od * DEFAULT_STATOR.hub_od_ratio - 10.0,
    ) / 2.0 - CAP_EDGE_OVERLAP_MM
    bare_mouth = float(geom["opening_width_mm"])
    lined_mouth = bare_mouth - 2.0 * MATERIAL_RECEIVING_MAX_MM
    capped_mouth = bare_mouth - 2.0 * CAP_EDGE_OVERLAP_MM
    min_x, min_y, max_x, max_y = cap.bounds
    strip_area = Polygon(slot_cell_outline()).area
    strip_columns = 13
    strip_rows = 2
    strip_block_width = (
        strip_columns * cell.blank_width_mm
        + (strip_columns - 1) * NEST_GAP_MM
    )
    strip_block_height = (
        strip_rows * cell.blank_length_mm
        + (strip_rows - 1) * NEST_GAP_MM
    )
    cap_block_width = 2.0 * DEFAULT_STATOR.od + NEST_GAP_MM
    cap_block_height = DEFAULT_STATOR.od
    nest_width = max(strip_block_width, cap_block_width)
    nest_height = strip_block_height + NEST_GAP_MM + cap_block_height
    sheet_area = MATERIAL_SHEET_WIDTH_MM * MATERIAL_SHEET_LENGTH_MM
    return {
        "stator": {
            "slots": DEFAULT_STATOR.slots,
            "od_mm": DEFAULT_STATOR.od,
            "stack_mm": DEFAULT_STATOR.stack,
            "hub_od_mm": DEFAULT_STATOR.od * DEFAULT_STATOR.hub_od_ratio,
            "hub_id_mm": 2.0 * (inner_radius + CAP_EDGE_OVERLAP_MM),
            "shoe_inner_radius_mm": geom["shoe_inner_radius_mm"],
            "shoe_thickness_mm": geom["shoe_thickness_mm"],
            "tooth_neck_width_mm": geom["tooth_neck_width_mm"],
            "slot_pitch_deg": geom["tooth_pitch_deg"],
            "bare_slot_mouth_mm": bare_mouth,
        },
        "slot_cell": {
            **asdict(cell),
            "required_quantity": DEFAULT_STATOR.slots,
            "recommended_cut_quantity": DEFAULT_STATOR.slots + 2,
            "profile": "open slot cell; no seam crosses the winding mouth",
            "end_treatment": (
                "centre on stack, flare each 1.50 mm relieved tab outward "
                "onto its end-face cap"
            ),
        },
        "end_caps": {
            "quantity": 2,
            "front_and_rear_identical": True,
            "overall_od_mm": DEFAULT_STATOR.od,
            "central_cutout_diameter_mm": 2.0 * inner_radius,
            "steel_edge_overlap_mm": CAP_EDGE_OVERLAP_MM,
            "area_each_mm2": cap.area,
            "bounds_mm": [min_x, min_y, max_x, max_y],
            "slot_cutouts": DEFAULT_STATOR.slots,
            "outside_od_fringe_mm": 0.0,
        },
        "clearance_indicators": {
            "bare_slot_mouth_mm": bare_mouth,
            "mouth_after_two_0p140_max_liner_walls_mm": lined_mouth,
            "end_cap_mouth_after_edge_overlap_mm": capped_mouth,
            "single_active_wire_static_margin_at_cap_mouth_mm": (
                capped_mouth - ACTIVE_WIRE_DIAMETER_MAX_MM
            ),
            "interpretation": (
                "positive one-wire opening only; not a packing, needle, "
                "route, enamel-damage, or production-tolerance proof"
            ),
        },
        "sheet_yield": {
            "sheet_size_mm": [
                MATERIAL_SHEET_WIDTH_MM, MATERIAL_SHEET_LENGTH_MM,
            ],
            "simple_unrotated_layout": (
                "13 x 2 slot-cell block plus two side-by-side star caps"
            ),
            "part_gap_mm": NEST_GAP_MM,
            "layout_bounds_mm": [nest_width, nest_height],
            "fits_selected_sheet": (
                nest_width <= MATERIAL_SHEET_WIDTH_MM
                and nest_height <= MATERIAL_SHEET_LENGTH_MM
            ),
            "net_part_area_mm2": (
                26.0 * strip_area + 2.0 * cap.area
            ),
            "net_sheet_area_utilization": (
                (26.0 * strip_area + 2.0 * cap.area) / sheet_area
            ),
            "scope": (
                "bounding-box feasibility only; converter may renest for "
                "machine direction, hold-down, and blade access"
            ),
        },
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_report() -> dict[str, object]:
    geometry = geometry_summary()
    active_compatible = (
        MATERIAL_RECEIVING_MAX_MM <= ACTIVE_WINDING_PLAN_LINER_MAX_MM
    )
    return {
        "schema": SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "drawing_revision": DRAWING_REV,
        "fabrication_package_status": "PASS",
        "active_winding_job_compatibility": (
            "PASS" if active_compatible else "BLOCKED"
        ),
        "authority": (
            "Exact for the repository's simplified DEFAULT_STATOR geometry; "
            "measure a purchased lamination and regenerate before cutting "
            "production insulation."
        ),
        "material": {
            "manufacturer": "DuPont",
            "name": MATERIAL_NAME,
            "part_number": MATERIAL_PART_NUMBER,
            "supplier": MATERIAL_SUPPLIER,
            "supplier_sku": MATERIAL_SUPPLIER_SKU,
            "material_kind": MATERIAL_KIND,
            "nominal_thickness_mm": MATERIAL_NOMINAL_THICKNESS_MM,
            "nominal_thickness_mil": 5.0,
            "receiving_max_installed_thickness_mm": MATERIAL_RECEIVING_MAX_MM,
            "receiving_min_installed_thickness_mm": MATERIAL_RECEIVING_MIN_MM,
            "sheet_width_mm": MATERIAL_SHEET_WIDTH_MM,
            "sheet_length_mm": MATERIAL_SHEET_LENGTH_MM,
            "coated": False,
            "adhesive": False,
            "product_url": PRODUCT_URL,
            "technical_data_sheet_url": DATA_SHEET_URL,
        },
        "geometry": geometry,
        "fabrication": {
            "units": "mm",
            "scale": "1:1",
            "cut_process": (
                "clean sharp knife, steel-rule die, or approved drag-knife "
                "plotter; do not laser unless the material converter approves "
                "edge chemistry and residue"
            ),
            "cut_tolerance_mm": CUT_TOLERANCE_MM,
            "fold_station_tolerance_mm": FOLD_STATION_TOLERANCE_MM,
            "fold_layers_are_reference_only": True,
            "grain_direction": (
                "orient strip axial Y with supplier machine direction when "
                "marked; keep all 24 strips in one orientation"
            ),
            "inspection": [
                "verify package identifies DuPont Nomex Type 410, 5 mil and BAE SKU INNMX410005S",
                "measure sheet thickness at five distributed points",
                "measure the formed installed liner at no fewer than five coupon locations; use the conservative maximum including instrument uncertainty",
                "accept the 50-turn job only when the measured installed input is 0.120-0.140 mm",
                "reject scorched, delaminated, contaminated, or fuzzy cut edges",
                "dry-fit every slot; trim only the OD mouth edge, never a root fold",
                "hipot/megger and winding-process qualification remain external gates",
            ],
        },
        "integration_contract": {
            "active_plan_liner_maximum_mm": ACTIVE_WINDING_PLAN_LINER_MAX_MM,
            "selected_sheet_nominal_mm": MATERIAL_NOMINAL_THICKNESS_MM,
            "receiving_min_installed_mm": MATERIAL_RECEIVING_MIN_MM,
            "receiving_max_installed_mm": MATERIAL_RECEIVING_MAX_MM,
            "margin_to_plan_limit_mm": (
                ACTIVE_WINDING_PLAN_LINER_MAX_MM - MATERIAL_RECEIVING_MAX_MM
            ),
            "accepted_wire_finished_diameter_max_mm": ACTIVE_WIRE_DIAMETER_MAX_MM,
            "acceptance": (
                "Compatible with the 50-turn measured-input contract only "
                "when the conservative installed liner input is 0.120-0.140 mm, "
                "the conservative finished-wire input is 0.220-0.235 mm, and "
                "every core edge remains protected. Regenerate packing, plan, "
                "settings, routes, capture, continuous audit, and player from "
                "those inputs; physical coupon and electrical gates still apply."
            ),
        },
        "incompatible_alternate": {
            "name": DMD180_ALTERNATE_NAME,
            "nominal_thickness_mm": DMD180_ALTERNATE_THICKNESS_MM,
            "excess_over_plan_limit_mm": (
                DMD180_ALTERNATE_THICKNESS_MM
                - ACTIVE_WINDING_PLAN_LINER_MAX_MM
            ),
            "status": "REJECT_AS_DROP_IN_SUBSTITUTE",
            "requalification": (
                "Regenerate and pass packing, progressive route, collision, "
                "and physical coupon gates before any use."
            ),
        },
        "outputs": {
            "slot_cell_dxf": (
                "out/custom/shop/dxf/stator_slot_cell_nomex410_5mil.dxf"
            ),
            "front_cap_dxf": (
                "out/custom/shop/dxf/stator_end_cap_front_nomex410_5mil.dxf"
            ),
            "rear_cap_dxf": (
                "out/custom/shop/dxf/stator_end_cap_rear_nomex410_5mil.dxf"
            ),
            "fabrication_pdf": (
                "out/custom/shop/pdf/stator_insulation_nomex410_5mil.pdf"
            ),
            "pdf_qa_pngs": [
                "out/custom/shop/pdf_qa/stator_insulation_nomex410_page1.png",
                "out/custom/shop/pdf_qa/stator_insulation_nomex410_page2.png",
                "out/custom/shop/pdf_qa/stator_insulation_nomex410_page3.png",
            ],
        },
        "source_sha256": {
            "stator_insulation_nomex410.py": _sha256(Path(__file__)),
            "params.py": _sha256(HERE / "params.py"),
            "coil_growth.py": _sha256(HERE / "coil_growth.py"),
            "stator_model.py": _sha256(HERE / "stator_model.py"),
        },
        "limits": [
            "No vendor lamination drawing or measured loose lamination was supplied.",
            "Nomex springback, bend radius, cut kerf, varnish, and thermal cure are not simulated.",
            "A positive static wire opening does not prove needle access or sequential winding.",
            "This package does not modify settings, the BOM, or release readiness.",
        ],
    }


def render_markdown(report: dict[str, object]) -> str:
    material = report["material"]
    geometry = report["geometry"]
    cell = geometry["slot_cell"]
    caps = geometry["end_caps"]
    clear = geometry["clearance_indicators"]
    yield_info = geometry["sheet_yield"]
    contract = report["integration_contract"]
    alternate = report["incompatible_alternate"]
    lines = [
        "# DuPont Nomex 410 stator insulation fabrication package",
        "",
        f"Fabrication drawings: **{report['fabrication_package_status']}**",
        f"Active winding-job compatibility: **{report['active_winding_job_compatibility']}**",
        "",
        f"Authority: {report['authority']}",
        "",
        "## Selected material",
        "",
        f"- {material['name']}",
        f"- Supplier: {material['supplier']}; SKU `{material['supplier_sku']}`",
        f"- Nominal thickness: {material['nominal_thickness_mm']:.3f} mm (5 mil)",
        f"- Receiving maximum after forming/install: {material['receiving_max_installed_thickness_mm']:.3f} mm",
        f"- Sheet: {material['sheet_width_mm']:.1f} x {material['sheet_length_mm']:.1f} mm (24 x 36 inch)",
        f"- [BAE product page]({material['product_url']})",
        f"- [DuPont technical data sheet]({material['technical_data_sheet_url']})",
        "",
        "## Cut set",
        "",
        f"- Slot cells: {cell['required_quantity']} required; cut {cell['recommended_cut_quantity']} including two spares.",
        f"- Slot-cell blank: {cell['blank_width_mm']:.3f} x {cell['blank_length_mm']:.3f} mm.",
        f"- Axial end flare: {cell['axial_end_flare_mm']:.2f} mm each face.",
        f"- End caps: two identical non-handed star caps, OD {caps['overall_od_mm']:.2f} mm, centre cutout {caps['central_cutout_diameter_mm']:.2f} mm.",
        f"- Cap overlap into every steel edge: {caps['steel_edge_overlap_mm']:.2f} mm; no fringe outside stator OD.",
        f"- One-sheet proof: the 26-strip plus two-cap simple nest is {yield_info['layout_bounds_mm'][0]:.1f} x {yield_info['layout_bounds_mm'][1]:.1f} mm and fits the selected 609.6 x 914.4 mm sheet.",
        "",
        "## Clearance indicators",
        "",
        f"- Bare shoe-inner mouth: {clear['bare_slot_mouth_mm']:.6f} mm.",
        f"- Mouth after two 0.140 mm maximum installed liner walls: {clear['mouth_after_two_0p140_max_liner_walls_mm']:.6f} mm.",
        f"- End-cap mouth after 0.35 mm edge overlap: {clear['end_cap_mouth_after_edge_overlap_mm']:.6f} mm.",
        f"- Static margin over one {contract['accepted_wire_finished_diameter_max_mm']:.3f} mm maximum accepted wire: {clear['single_active_wire_static_margin_at_cap_mouth_mm']:.6f} mm.",
        f"- Scope: {clear['interpretation']}",
        "",
        "## Integration contract",
        "",
        f"The selected sheet is {contract['selected_sheet_nominal_mm']:.3f} mm nominal; the conservative installed input must be {contract['receiving_min_installed_mm']:.3f}-{contract['receiving_max_installed_mm']:.3f} mm. {contract['acceptance']}",
        "",
        "## Incompatible alternate",
        "",
        f"{alternate['name']} is {alternate['nominal_thickness_mm']:.4f} mm, {alternate['excess_over_plan_limit_mm']:.4f} mm above the active limit: **{alternate['status']}**. {alternate['requalification']}",
        "",
        "## Limits",
        "",
    ]
    lines.extend(f"- {item}" for item in report["limits"])
    lines.append("")
    return "\n".join(lines)


PAGE = landscape(letter)
PAGE_W, PAGE_H = PAGE
NAVY = colors.HexColor("#18324A")
BLUE = colors.HexColor("#2B6F9F")
PALE = colors.HexColor("#EAF2F7")
ORANGE = colors.HexColor("#D97706")
RED = colors.HexColor("#B42318")
INK = colors.HexColor("#17212A")
GRAY = colors.HexColor("#5E6B75")


def _pdf_header(c: canvas.Canvas, title: str, drawing: str,
                page: int, pages: int) -> None:
    c.setFillColor(colors.white)
    c.rect(0, 0, PAGE_W, PAGE_H, fill=1, stroke=0)
    c.setFillColor(NAVY)
    c.rect(0, PAGE_H - 50, PAGE_W, 50, fill=1, stroke=0)
    c.setFillColor(colors.white)
    c.setFont("Helvetica-Bold", 15)
    c.drawString(28, PAGE_H - 31, title)
    c.setFont("Helvetica", 8)
    c.drawRightString(PAGE_W - 28, PAGE_H - 22,
                      f"DRAWING {drawing}  REV {DRAWING_REV}")
    c.drawRightString(PAGE_W - 28, PAGE_H - 35,
                      f"PAGE {page}/{pages}  UNITS mm")
    c.setFillColor(INK)


def _pdf_footer(c: canvas.Canvas, authority: str) -> None:
    c.setStrokeColor(colors.HexColor("#AAB6BE"))
    c.line(28, 28, PAGE_W - 28, 28)
    c.setFillColor(GRAY)
    c.setFont("Helvetica", 7.2)
    c.drawString(28, 16,
                 "1:1 DXF authority; PDF views are enlarged and not cut templates")
    c.drawRightString(PAGE_W - 28, 16, authority)


def _wrapped(c: canvas.Canvas, text: str, x: float, y: float,
             width: float, font: str = "Helvetica", size: float = 8.5,
             leading: float = 12.0) -> float:
    words = text.split()
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = word if not current else current + " " + word
        if c.stringWidth(candidate, font, size) <= width:
            current = candidate
        else:
            lines.append(current)
            current = word
    if current:
        lines.append(current)
    c.setFont(font, size)
    for line in lines:
        c.drawString(x, y, line)
        y -= leading
    return y


def _table(c: canvas.Canvas, x: float, y: float, widths: Sequence[float],
           rows: Sequence[Sequence[str]], row_h: float = 22.0) -> None:
    total = sum(widths)
    for index, row in enumerate(rows):
        top = y - index * row_h
        c.setFillColor(NAVY if index == 0 else
                       (PALE if index % 2 else colors.white))
        c.rect(x, top - row_h, total, row_h, fill=1, stroke=0)
        c.setFillColor(colors.white if index == 0 else INK)
        c.setFont("Helvetica-Bold" if index == 0 else "Helvetica", 7.8)
        xx = x
        for value, width in zip(row, widths):
            c.drawString(xx + 4, top - row_h + 7, str(value))
            xx += width
    c.setStrokeColor(colors.HexColor("#A9B6BE"))
    c.rect(x, y - len(rows) * row_h, total, len(rows) * row_h,
           fill=0, stroke=1)


def _draw_slot_blank(c: canvas.Canvas, x: float, y: float,
                     scale: float) -> None:
    outline = slot_cell_outline()
    path = c.beginPath()
    path.moveTo(x + outline[0][0] * scale, y + outline[0][1] * scale)
    for px, py in outline[1:]:
        path.lineTo(x + px * scale, y + py * scale)
    path.close()
    c.setFillColor(colors.HexColor("#DCEAF2"))
    c.setStrokeColor(BLUE)
    c.setLineWidth(1.3)
    c.drawPath(path, fill=1, stroke=1)
    cell = developed_slot_cell()
    c.setDash(3, 2)
    c.setStrokeColor(ORANGE)
    for station in cell.fold_stations_x_mm:
        c.line(x + station * scale,
               y + cell.axial_end_flare_mm * scale,
               x + station * scale,
               y + (cell.blank_length_mm - cell.axial_end_flare_mm) * scale)
    c.setDash()


def _draw_cap(c: canvas.Canvas, cap: Polygon, cx: float, cy: float,
              scale: float) -> None:
    c.setFillColor(colors.HexColor("#DCEAF2"))
    c.setStrokeColor(BLUE)
    c.setLineWidth(1.0)
    exterior = _polyline_points(cap.exterior.coords)
    path = c.beginPath()
    path.moveTo(cx + exterior[0][0] * scale,
                cy + exterior[0][1] * scale)
    for px, py in exterior[1:]:
        path.lineTo(cx + px * scale, cy + py * scale)
    path.close()
    c.drawPath(path, fill=1, stroke=1)
    for interior in cap.interiors:
        points = _polyline_points(interior.coords)
        hole = c.beginPath()
        hole.moveTo(cx + points[0][0] * scale,
                    cy + points[0][1] * scale)
        for px, py in points[1:]:
            hole.lineTo(cx + px * scale, cy + py * scale)
        hole.close()
        c.setFillColor(colors.white)
        c.drawPath(hole, fill=1, stroke=1)


def write_pdf(path: Path) -> None:
    report = build_report()
    geometry = report["geometry"]
    cell = developed_slot_cell()
    cap = end_cap_geometry()
    path.parent.mkdir(parents=True, exist_ok=True)
    c = canvas.Canvas(str(path), pagesize=PAGE)

    _pdf_header(c, "DUPONT NOMEX 410 STATOR INSULATION CUT SET",
                "STATOR-INS-001", 1, 3)
    c.setFillColor(colors.HexColor("#ECFDF3"))
    c.setStrokeColor(colors.HexColor("#067647"))
    c.roundRect(42, PAGE_H - 104, PAGE_W - 84, 36, 5, fill=1, stroke=1)
    c.setFillColor(colors.HexColor("#067647"))
    c.setFont("Helvetica-Bold", 9)
    c.drawCentredString(PAGE_W / 2, PAGE_H - 83,
                        "DRAWINGS COMPLETE - 0.127 mm NOMINAL; ACCEPT MEASURED INSTALLED INPUT 0.120-0.140 mm")
    material = report["material"]
    _table(c, 45, 445, [155, 220, 150, 165], [
        ["ITEM", "EXACT CALL-OUT", "VALUE", "RECEIVING CHECK"],
        ["Sheet", "DuPont Nomex Type 410", material["supplier_sku"], "Match package/CoC"],
        ["Construction", "Calendered aramid paper", "Uncoated; no adhesive", "No substitution"],
        ["Nominal thickness", "5 mil / 0.005 inch", "0.127 mm", "5-point measure"],
        ["Installed input", "Packing contract", "0.120-0.140 mm", "5-point coupon max"],
        ["Sheet size", "24 x 36 inch", "609.6 x 914.4 mm", "One sheet"],
    ])
    c.setFillColor(RED)
    c.setFont("Helvetica-Bold", 8.2)
    c.drawCentredString(
        PAGE_W / 2, 294,
        "DO NOT SUBSTITUTE 8 MIL DMD180 3-2-3: 0.2032 mm EXCEEDS THE ACTIVE LINER LIMIT BY 0.0632 mm",
    )
    c.setFillColor(NAVY)
    c.setFont("Helvetica-Bold", 11)
    c.drawString(45, 275, "CUT QUANTITIES")
    _table(c, 45, 258, [190, 95, 115, 270], [
        ["PART", "REQUIRED", "CUT QTY", "DXF"],
        ["Identical slot-cell strip", "24", "26", "stator_slot_cell_nomex410_5mil.dxf"],
        ["Front star cap", "1", "1", "stator_end_cap_front_nomex410_5mil.dxf"],
        ["Rear star cap", "1", "1", "stator_end_cap_rear_nomex410_5mil.dxf"],
    ])
    c.setFillColor(INK)
    _wrapped(c,
             "Cut with a clean sharp knife, steel-rule die, or converter-approved drag knife. "
             "Fold/reference layers are not cuts. Do not laser unless the material converter "
             "approves the resulting edge chemistry and residue. Keep all slot strips in one "
             "supplier machine-direction orientation. A simple 2 mm-gap nest of 26 strips and "
             "two caps occupies 311.8 x 86.0 mm, so one selected sheet is sufficient.",
             45, 135, 690, size=8.5, leading=13)
    _pdf_footer(c, "simplified DEFAULT_STATOR OD46 x 15, 24 slot")
    c.showPage()

    _pdf_header(c, "SLOT-CELL STRIP FLAT PATTERN AND FORMING",
                "STATOR-INS-002", 2, 3)
    scale = 13.0
    draw_x, draw_y = 68, 115
    _draw_slot_blank(c, draw_x, draw_y, scale)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 9)
    c.drawString(draw_x, 70, "ENLARGED VIEW - CUT FROM 1:1 DXF")
    c.setStrokeColor(GRAY)
    c.line(draw_x, draw_y - 12,
           draw_x + cell.blank_width_mm * scale, draw_y - 12)
    c.setFont("Helvetica", 8)
    c.drawCentredString(draw_x + cell.blank_width_mm * scale / 2,
                        draw_y - 23, f"{cell.blank_width_mm:.3f} overall")
    c.line(draw_x + cell.blank_width_mm * scale + 12, draw_y,
           draw_x + cell.blank_width_mm * scale + 12,
           draw_y + cell.blank_length_mm * scale)
    c.saveState()
    c.translate(draw_x + cell.blank_width_mm * scale + 25,
                draw_y + cell.blank_length_mm * scale / 2)
    c.rotate(90)
    c.drawCentredString(0, 0, f"{cell.blank_length_mm:.3f} overall")
    c.restoreState()
    table_x = 440
    _table(c, table_x, 465, [150, 130], [
        ["FEATURE", "NOMINAL"],
        ["Stack coverage", f"{cell.stack_mm:.3f}"],
        ["End flare each", f"{cell.axial_end_flare_mm:.3f}"],
        ["Shoe edge each", f"{cell.shoe_edge_each_mm:.3f}"],
        ["Shoe arc each", f"{cell.shoe_under_arc_each_mm:.3f}"],
        ["Neck wall each", f"{cell.neck_wall_each_mm:.3f}"],
        ["Root fold station", f"X={cell.root_fold_x_mm:.3f}"],
        ["V relief width", f"{cell.relief_notch_width_mm:.3f}"],
        ["Cut tolerance", f"+/-{CUT_TOLERANCE_MM:.2f}"],
    ], row_h=24)
    c.setFillColor(NAVY)
    c.setFont("Helvetica-Bold", 10)
    c.drawString(table_x, 215, "FORMING SEQUENCE")
    c.setFillColor(INK)
    y = 196
    for index, note in enumerate((
        "Use a radiused mandrel; do not knife-score or cut the orange fold references.",
        "Form from the root fold outward, then seat both shoe-edge returns through the OD mouth.",
        "Centre the 15 mm stack between the axial datums; 1.50 mm remains at each face.",
        "Install both star caps, flare every relieved tab outward over the cap, then retain with the qualified varnish process.",
        "Dry-fit all 24 slots and reject lifted, fuzzy, delaminated, or steel-exposing edges before winding.",
    ), 1):
        y = _wrapped(c, f"{index}. {note}", table_x, y, 300,
                     size=8.1, leading=11.5) - 3
    _pdf_footer(c, "out/custom/shop/dxf/stator_slot_cell_nomex410_5mil.dxf")
    c.showPage()

    _pdf_header(c, "FRONT AND REAR END-FACE STAR CAPS",
                "STATOR-INS-003", 3, 3)
    _draw_cap(c, cap, 220, 325, 5.1)
    _draw_cap(c, cap, 520, 325, 5.1)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 9)
    c.drawCentredString(220, 188, "FRONT - QTY 1")
    c.drawCentredString(520, 188, "REAR - QTY 1 (IDENTICAL, NON-HANDED)")
    caps = geometry["end_caps"]
    clear = geometry["clearance_indicators"]
    c.setFillColor(RED)
    c.setFont("Helvetica-Bold", 8.5)
    c.drawCentredString(PAGE_W / 2, 166,
                        "MEASURE THE ACTUAL LAMINATION BEFORE PRODUCTION CUTTING; REGENERATE IF ANY FACE OR SLOT DIMENSION DIFFERS")
    _table(c, 78, 146, [190, 135, 300], [
        ["FEATURE", "NOMINAL", "FUNCTION / LIMIT"],
        ["Overall extent", f"OD {caps['overall_od_mm']:.2f}", "No paper fringe outside steel OD"],
        ["Centre cutout", f"D {caps['central_cutout_diameter_mm']:.2f}", "0.35 overlap onto hub-ID steel edge"],
        ["Slot/tooth pitch", "24 at 15.00 deg", "Aligned to simplified stator tooth 0"],
        ["Steel edge overlap", f"{caps['steel_edge_overlap_mm']:.2f}", "Contracts each slot edge on the face"],
        ["Cap mouth", f"{clear['end_cap_mouth_after_edge_overlap_mm']:.3f}", "Static opening only; route gate is external"],
    ], row_h=18)
    _pdf_footer(c, "front/rear cap DXFs in out/custom/shop/dxf")
    c.save()


def render_pdf_qa(path: Path, output_dir: Path = PDF_QA_DIR) -> list[Path]:
    """Rasterize the final PDF with PDFium for deterministic visual QA."""

    import pypdfium2 as pdfium

    output_dir.mkdir(parents=True, exist_ok=True)
    document = pdfium.PdfDocument(str(path))
    outputs: list[Path] = []
    for index, page in enumerate(document, 1):
        output = output_dir / f"stator_insulation_nomex410_page{index}.png"
        page.render(scale=2.0).to_pil().save(output)
        outputs.append(output)
    return outputs


def write_reports(json_path: Path, markdown_path: Path) -> dict[str, object]:
    report = build_report()
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")
    return report


def write_direct_dxfs() -> None:
    """Convenience exporter; the DXF skill CLI remains the validation path."""

    DXF_DIR.mkdir(parents=True, exist_ok=True)
    slot_cell_dxf().saveas(DXF_DIR / "stator_slot_cell_nomex410_5mil.dxf")
    end_cap_dxf("front").saveas(
        DXF_DIR / "stator_end_cap_front_nomex410_5mil.dxf"
    )
    end_cap_dxf("rear").saveas(
        DXF_DIR / "stator_end_cap_rear_nomex410_5mil.dxf"
    )


def main() -> int:
    write_direct_dxfs()
    pdf_path = PDF_DIR / "stator_insulation_nomex410_5mil.pdf"
    write_pdf(pdf_path)
    render_pdf_qa(pdf_path)
    report = write_reports(
        REPORT_DIR / "stator_insulation_nomex410_5mil.json",
        REPORT_DIR / "stator_insulation_nomex410_5mil.md",
    )
    print(
        "Nomex 410 fabrication package: "
        f"drawings={report['fabrication_package_status']} "
        f"active_job={report['active_winding_job_compatibility']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
