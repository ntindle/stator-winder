"""Fail-closed felt-tensioner preload and catalog-spring audit.

The felt stack is a *base drag* device: the compression spring creates a
normal clamp load and the two felt/wire interfaces turn that load into drag.
It is not the dancer.  The dancer stores wire and reacts to transient tension;
this module deliberately does not credit the felt stack with that function.

Felt friction against finished magnet wire is strongly dependent on pad lot,
enamel, contamination, speed, humidity, and felt compression.  Consequently
the Coulomb relation used here is a sizing envelope, not a calibration:

    drag = 2 * mu * normal_preload

The selected spring and wing nut cover 1..10 N for an explicit, deliberately
broad ``mu=0.15..0.45`` tuning assumption.  Final drag must be set with a pull
gauge on the actual wire.  The report remains failed until the selected spring
and corrected stack geometry are present in the assembly/hardware schedule.

Run from ``machine/cad``::

    ..\\.venv\\Scripts\\python.exe felt_loads.py

Outputs are written to ``out/reports/felt_loads.{json,md}``.  A non-zero exit
status means the current shared CAD/BOM integration is not order-ready.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any

from build123d import Align, Cylinder, Pos
import hardware
import hardware_placements
from params import DEFAULT_STATOR, PARAMS as P
import printed
import wire_geometry


LBF_TO_N = 4.4482216152605
M4_COARSE_PITCH_MM = 0.7
CONTACT_COUNT = 2

# This is a transparent sizing assumption, not a material-property claim.
# The report requires pull-gauge calibration and also shows results outside the
# assumed band so an unexpectedly slick or grabby pad cannot be mistaken for a
# proven 1..10 N setting.
MU_DESIGN_MIN = 0.15
MU_NOMINAL = 0.30
MU_DESIGN_MAX = 0.45
MU_SENSITIVITY = (0.10, 0.15, 0.30, 0.45, 0.60)
DRAG_MIN_N = 1.0
DRAG_NOMINAL_N = 5.0
DRAG_MAX_N = 10.0

MIN_THREAD_ENGAGEMENT_MM = 5.0  # full DIN 315 hub thread, with tolerance reserve
MIN_COIL_BIND_MARGIN_MM = 2.0
MIN_BACKING_EDGE_MARGIN_MM = 1.0
MAX_WIRE_CENTERING_ERROR_MM = 0.05
MAX_STACK_CONTACT_GAP_MM = 0.05

SPRING_THRUST_WASHER = {
    "vendor": "McMaster-Carr",
    "sku": "91116A130",
    "description": "18-8 stainless M4 oversized washer",
    "id_mm": 4.3,
    "od_mm": 12.0,
    "thickness_range_mm": [0.9, 1.1],
    "specification": "DIN 9021",
    "package_qty": 100,
    "source_url": "https://www.mcmaster.com/91116A130/",
    "source_category_url": (
        "https://www.mcmaster.com/products/oversized-washers/"
        "system-of-measurement~metric/"
    ),
    "source_checked": "2026-07-10",
}

BACKING_DISC = {
    "quantity": 2,
    "od_mm": 20.0,
    "id_mm": 4.5,
    "thickness_mm": 1.0,
    "material": "304 stainless steel",
    "route": "laser-cut/custom flat disc",
}


@dataclass(frozen=True)
class CompressionSpring:
    vendor: str
    sku: str
    description: str
    material: str
    free_length_mm: float
    od_mm: float
    id_mm: float
    wire_d_mm: float
    compressed_length_at_max_load_mm: float
    max_load_n: float
    rate_n_per_mm: float
    end_type: str
    specifications: tuple[str, ...]
    package_qty: int
    source_url: str
    source_category_url: str
    source_checked: str

    def force(self, installed_length_mm: float) -> float:
        """Ideal catalog-rate force; negative compression is clamped to zero."""
        return max(0.0, self.rate_n_per_mm
                   * (self.free_length_mm - installed_length_mm))

    def installed_length(self, force_n: float) -> float:
        if force_n < 0.0:
            raise ValueError("compression-spring force cannot be negative")
        return self.free_length_mm - force_n / self.rate_n_per_mm


# McMaster's current table row (checked 2026-07-10): 22 mm long, 9.25 mm OD,
# 6.75 mm ID, 1.25 mm wire, 14.1 mm compressed length at 16 lb maximum load,
# 2 lbf/mm rate, closed and ground, package of five.
SELECTED_SPRING = CompressionSpring(
    vendor="McMaster-Carr",
    sku="94125K614",
    description="metric spring-steel compression spring",
    material="spring steel",
    free_length_mm=22.0,
    od_mm=9.25,
    id_mm=6.75,
    wire_d_mm=1.25,
    compressed_length_at_max_load_mm=14.1,
    max_load_n=16.0 * LBF_TO_N,
    rate_n_per_mm=2.0 * LBF_TO_N,
    end_type="closed and ground",
    specifications=("DIN 2095", "DIN 2098", "DIN 17223"),
    package_qty=5,
    source_url="https://www.mcmaster.com/94125K614/",
    source_category_url=(
        "https://www.mcmaster.com/products/precision-compression-springs/"
    ),
    source_checked="2026-07-10",
)


def drag_from_preload(normal_force_n: float, mu: float) -> float:
    """Two-face straight-pass Coulomb sizing relation."""
    if normal_force_n < 0.0 or mu < 0.0:
        raise ValueError("normal force and friction coefficient must be nonnegative")
    return CONTACT_COUNT * mu * normal_force_n


def preload_for_drag(drag_n: float, mu: float) -> float:
    if drag_n < 0.0 or mu <= 0.0:
        raise ValueError("drag must be nonnegative and mu must be positive")
    return drag_n / (CONTACT_COUNT * mu)


def design_preload_band() -> dict[str, float]:
    """Normal-force/length band covering the full drag and design-mu box."""
    minimum_force = preload_for_drag(DRAG_MIN_N, MU_DESIGN_MAX)
    maximum_force = preload_for_drag(DRAG_MAX_N, MU_DESIGN_MIN)
    nominal_force = preload_for_drag(DRAG_NOMINAL_N, MU_NOMINAL)
    return {
        "minimum_normal_force_n": minimum_force,
        "nominal_normal_force_n": nominal_force,
        "maximum_normal_force_n": maximum_force,
        "maximum_installed_length_mm": SELECTED_SPRING.installed_length(
            minimum_force),
        "nominal_installed_length_mm": SELECTED_SPRING.installed_length(
            nominal_force),
        "minimum_installed_length_mm": SELECTED_SPRING.installed_length(
            maximum_force),
        "wingnut_travel_mm": ((maximum_force - minimum_force)
                              / SELECTED_SPRING.rate_n_per_mm),
        "wingnut_turns": ((maximum_force - minimum_force)
                          / SELECTED_SPRING.rate_n_per_mm
                          / M4_COARSE_PITCH_MM),
    }


def _bbox(part) -> dict[str, list[float]]:
    box = part.bounding_box()
    return {"min": list(tuple(box.min)), "max": list(tuple(box.max))}


def _extent(box: dict[str, list[float]], axis: int) -> float:
    return box["max"][axis] - box["min"][axis]


def _schedule_item(item_id: str) -> dict[str, Any]:
    matches = [item for item in hardware.HARDWARE_SCHEDULE
               if item["id"] == item_id]
    if len(matches) != 1:
        raise AssertionError(f"expected one schedule item {item_id!r}, got {len(matches)}")
    return matches[0]


def _printed_felt_boss_front_z() -> float:
    """Measure the fixed stack's printed annular seat, not a prose constant."""
    # The center has an M4 bore, so probe an annulus inside the OD14 boss.
    probe = Pos(P.rear_post_x, P.felt_y, -200.0) * (
        Cylinder(6.0, 100.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        - Cylinder(3.0, 102.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    )
    contact = printed.felt_tensioner() & probe
    if not contact.solids():
        raise AssertionError("could not measure printed felt-boss contact seat")
    return contact.bounding_box().max.Z


def current_stack_snapshot() -> dict[str, Any]:
    """Read the actual placed stack and authoritative wire centerline."""
    occurrences = {
        occurrence.label: occurrence
        for occurrence in hardware_placements.hardware_occurrences_by_link(P)["static"]
        if occurrence.label.startswith("felt_")
    }
    stud_occurrences = [occurrence for occurrence in occurrences.values()
                        if occurrence.schedule_id == "felt_stud"]
    if len(stud_occurrences) != 1:
        raise AssertionError(
            f"expected one felt_stud occurrence, got {len(stud_occurrences)}")
    stud_label = stud_occurrences[0].label
    required = (
        stud_label, "felt_backing_fixed", "felt_pad_fixed",
        "felt_pad_moving", "felt_backing_moving",
        "felt_compression_spring", "felt_m4_wingnut",
    )
    missing = [label for label in required if label not in occurrences]
    if missing:
        raise AssertionError(f"felt placement occurrences missing: {missing}")

    boxes = {label: _bbox(occurrences[label].build()) for label in required}
    thrust_label = "felt_spring_thrust_washer"
    thrust_present = thrust_label in occurrences
    if thrust_present:
        boxes[thrust_label] = _bbox(occurrences[thrust_label].build())
    landmarks = wire_geometry.static_path_spec()["landmarks"]
    wire = landmarks["felt_contact"]
    wire_d = DEFAULT_STATOR.wire_d
    fixed_face = boxes["felt_pad_fixed"]["max"][2]
    moving_face = boxes["felt_pad_moving"]["min"][2]
    gap_mid = (fixed_face + moving_face) / 2.0
    spring_seat = boxes["felt_backing_moving"]["max"][2]
    spring_face = boxes["felt_compression_spring"]["max"][2]
    wingnut_face = boxes["felt_m4_wingnut"]["min"][2]
    stud_end = boxes[stud_label]["max"][2]

    pad_radius = _extent(boxes["felt_pad_fixed"], 0) / 2.0
    backing_radius = _extent(boxes["felt_backing_fixed"], 0) / 2.0
    radial_offset = math.hypot(wire[0] - P.rear_post_x,
                               wire[1] - P.felt_y)
    spring_length = _extent(boxes["felt_compression_spring"], 2)
    schedule = _schedule_item("felt_compression_spring")
    printed_boss_front = _printed_felt_boss_front_z()
    fixed_backing_rear = boxes["felt_backing_fixed"]["min"][2]

    return {
        "wire_contact_xyz_mm": list(wire),
        "wire_diameter_mm": wire_d,
        "stud_center_xy_mm": [P.rear_post_x, P.felt_y],
        "wire_radial_offset_from_stud_mm": radial_offset,
        "pad_radius_mm": pad_radius,
        "backing_radius_mm": backing_radius,
        "pad_wire_edge_margin_mm": pad_radius - radial_offset - wire_d / 2.0,
        "backing_wire_edge_margin_mm": (
            backing_radius - radial_offset - wire_d / 2.0),
        "fixed_pad_wire_face_z_mm": fixed_face,
        "moving_pad_wire_face_z_mm": moving_face,
        "printed_fixed_stack_boss_front_z_mm": printed_boss_front,
        "fixed_backing_rear_face_z_mm": fixed_backing_rear,
        "fixed_backing_to_boss_gap_mm": fixed_backing_rear - printed_boss_front,
        "unloaded_pad_gap_mm": moving_face - fixed_face,
        "unloaded_pad_gap_mid_z_mm": gap_mid,
        "wire_centering_error_mm": wire[2] - gap_mid,
        "moving_backing_spring_seat_z_mm": spring_seat,
        "modeled_spring_installed_length_mm": spring_length,
        "modeled_spring_front_face_z_mm": spring_face,
        "modeled_wingnut_bearing_face_z_mm": wingnut_face,
        "spring_to_wingnut_face_gap_mm": wingnut_face - spring_face,
        "spring_thrust_washer_present": thrust_present,
        "spring_thrust_washer_label": thrust_label if thrust_present else None,
        "spring_to_thrust_washer_gap_mm": (
            boxes[thrust_label]["min"][2] - spring_face
            if thrust_present else None),
        "thrust_washer_to_wingnut_gap_mm": (
            wingnut_face - boxes[thrust_label]["max"][2]
            if thrust_present else None),
        "stud_label": stud_label,
        "stud_end_z_mm": stud_end,
        "modeled_thread_engagement_mm": stud_end - wingnut_face,
        "selected_spring_force_at_current_modeled_length_n": (
            SELECTED_SPRING.force(spring_length)),
        "selected_spring_coil_bind_margin_at_current_length_mm": (
            spring_length
            - SELECTED_SPRING.compressed_length_at_max_load_mm),
        "hardware_schedule": schedule,
        "component_bounding_boxes_mm": boxes,
    }


def recommended_geometry(snapshot: dict[str, Any] | None = None) -> dict[str, Any]:
    if snapshot is None:
        snapshot = current_stack_snapshot()
    band = design_preload_band()
    target_fixed_face = (snapshot["wire_contact_xyz_mm"][2]
                         - snapshot["unloaded_pad_gap_mm"] / 2.0)
    target_moving_face = (snapshot["wire_contact_xyz_mm"][2]
                          + snapshot["unloaded_pad_gap_mm"] / 2.0)
    pad_thickness = _extent(
        snapshot["component_bounding_boxes_mm"]["felt_pad_fixed"], 2)
    target_fixed_backing_front = target_fixed_face - pad_thickness
    target_fixed_backing_rear = (
        target_fixed_backing_front - BACKING_DISC["thickness_mm"])
    target_moving_backing_rear = target_moving_face + pad_thickness
    seat = target_moving_backing_rear + BACKING_DISC["thickness_mm"]
    pad_shift = target_fixed_face - snapshot["fixed_pad_wire_face_z_mm"]
    seat_shift = seat - snapshot["moving_backing_spring_seat_z_mm"]
    stud_end = snapshot["stud_end_z_mm"]
    washer_t = max(SPRING_THRUST_WASHER["thickness_range_mm"])
    spring_end_at_min_preload = seat + band["maximum_installed_length_mm"]
    spring_end_at_nominal = seat + band["nominal_installed_length_mm"]
    spring_end_at_max_preload = seat + band["minimum_installed_length_mm"]
    face_at_min_preload = spring_end_at_min_preload + washer_t
    face_at_nominal = spring_end_at_nominal + washer_t
    face_at_max_preload = spring_end_at_max_preload + washer_t
    minimum_stud_end = face_at_min_preload + MIN_THREAD_ENGAGEMENT_MM
    current_stud_start = snapshot["component_bounding_boxes_mm"][
        snapshot["stud_label"]]["min"][2]
    return {
        "pad_axial_shift_from_current_mm": pad_shift,
        "spring_seat_axial_shift_from_current_mm": seat_shift,
        "target_fixed_pad_wire_face_z_mm": target_fixed_face,
        "target_moving_pad_wire_face_z_mm": target_moving_face,
        "target_fixed_backing_front_face_z_mm": target_fixed_backing_front,
        "target_fixed_backing_rear_face_z_mm": target_fixed_backing_rear,
        "target_moving_backing_rear_face_z_mm": target_moving_backing_rear,
        "required_printed_boss_front_z_mm": target_fixed_backing_rear,
        "spring_bearing_seat_z_mm": seat,
        "spring_front_face_range_z_mm": [
            spring_end_at_max_preload, spring_end_at_min_preload],
        "nominal_spring_front_face_z_mm": spring_end_at_nominal,
        "spring_thrust_washer_thickness_design_mm": washer_t,
        "wingnut_bearing_face_range_z_mm": [
            face_at_max_preload, face_at_min_preload],
        "nominal_wingnut_bearing_face_z_mm": face_at_nominal,
        "minimum_thread_engagement_over_adjustment_mm": (
            stud_end - face_at_min_preload),
        "minimum_required_stud_end_z_mm": minimum_stud_end,
        "minimum_required_stud_length_mm": minimum_stud_end - current_stud_start,
        "recommended_standard_stud_length_mm": 50.0,
        "nominal_modeled_spring_length_mm": (
            band["nominal_installed_length_mm"]),
        "nominal_modeled_spring_force_n": band["nominal_normal_force_n"],
        "minimum_backing_od_mm": 2.0 * (
            snapshot["wire_radial_offset_from_stud_mm"]
            + snapshot["wire_diameter_mm"] / 2.0
            + MIN_BACKING_EDGE_MARGIN_MM),
        "recommended_backing_disc": {
            **BACKING_DISC,
            "reason": (
                "the current ISO 7093 OD12 washer ends at the wire centerline; "
                "OD20 restores 3.88 mm radial support margin"
            ),
        },
    }


def selected_spring_checks() -> list[dict[str, Any]]:
    band = design_preload_band()
    spring = SELECTED_SPRING
    maximum_force = band["maximum_normal_force_n"]
    minimum_length = band["minimum_installed_length_mm"]
    checks = [
        ("spring ID clears M4 stud", spring.id_mm > 4.0,
         spring.id_mm - 4.0, "> 0 mm diametral clearance"),
        ("spring OD fits current OD10 envelope", spring.od_mm <= 10.0,
         10.0 - spring.od_mm, ">= 0 mm radial-envelope margin"),
        ("design maximum preload below catalog maximum load",
         maximum_force <= spring.max_load_n,
         spring.max_load_n - maximum_force, ">= 0 N reserve"),
        ("design minimum length above catalog compressed length",
         minimum_length >= spring.compressed_length_at_max_load_mm,
         minimum_length - spring.compressed_length_at_max_load_mm,
         ">= 0 mm catalog reserve"),
        ("coil-bind margin meets design minimum",
         minimum_length - spring.compressed_length_at_max_load_mm
         >= MIN_COIL_BIND_MARGIN_MM,
         minimum_length - spring.compressed_length_at_max_load_mm,
         f">= {MIN_COIL_BIND_MARGIN_MM:g} mm"),
    ]
    return [{"name": name, "pass": passed, "value": value, "requirement": req}
            for name, passed, value, req in checks]


def audit() -> dict[str, Any]:
    snapshot = current_stack_snapshot()
    band = design_preload_band()
    geometry = recommended_geometry(snapshot)
    spring_checks = selected_spring_checks()
    schedule = snapshot["hardware_schedule"]
    schedule_model = schedule.get("model") or {}
    schedule_kwargs = schedule_model.get("kwargs") or {}
    schedule_geometry_matches = (
        schedule_model.get("factory") == "spring_envelope"
        and math.isclose(schedule_kwargs.get("od", math.nan),
                         SELECTED_SPRING.od_mm, abs_tol=1e-3)
        and math.isclose(schedule_kwargs.get("bore", math.nan),
                         SELECTED_SPRING.id_mm, abs_tol=1e-3)
        and math.isclose(schedule_kwargs.get("length", math.nan),
                         band["nominal_installed_length_mm"], abs_tol=0.02)
    )
    thrust_schedule = [item for item in hardware.HARDWARE_SCHEDULE
                       if item["id"] == "felt_spring_thrust_washer"]
    thrust_schedule_matches = (
        len(thrust_schedule) == 1
        and thrust_schedule[0].get("sku") == SPRING_THRUST_WASHER["sku"]
        and thrust_schedule[0].get("qty") == 1
        and thrust_schedule[0].get("status") not in {
            "selection_pending", "pending", None}
    )

    current_checks = [
        ("wire centered between unloaded felt faces",
         abs(snapshot["wire_centering_error_mm"])
         <= MAX_WIRE_CENTERING_ERROR_MM,
         abs(snapshot["wire_centering_error_mm"]),
         f"<= {MAX_WIRE_CENTERING_ERROR_MM:g} mm"),
        ("felt OD supports wire contact",
         snapshot["pad_wire_edge_margin_mm"] >= MIN_BACKING_EDGE_MARGIN_MM,
         snapshot["pad_wire_edge_margin_mm"],
         f">= {MIN_BACKING_EDGE_MARGIN_MM:g} mm edge margin"),
        ("fixed backing is seated against printed boss",
         abs(snapshot["fixed_backing_to_boss_gap_mm"])
         <= MAX_STACK_CONTACT_GAP_MM,
         snapshot["fixed_backing_to_boss_gap_mm"],
         f"absolute gap <= {MAX_STACK_CONTACT_GAP_MM:g} mm"),
        ("metal backing supports wire contact",
         snapshot["backing_wire_edge_margin_mm"] >= MIN_BACKING_EDGE_MARGIN_MM,
         snapshot["backing_wire_edge_margin_mm"],
         f">= {MIN_BACKING_EDGE_MARGIN_MM:g} mm edge margin"),
        ("spring touches moving backing",
         abs(snapshot["component_bounding_boxes_mm"]
             ["felt_compression_spring"]["min"][2]
             - snapshot["moving_backing_spring_seat_z_mm"])
         <= MAX_STACK_CONTACT_GAP_MM,
         abs(snapshot["component_bounding_boxes_mm"]
             ["felt_compression_spring"]["min"][2]
             - snapshot["moving_backing_spring_seat_z_mm"]),
         f"<= {MAX_STACK_CONTACT_GAP_MM:g} mm"),
        ("separate spring thrust washer is placed under wingnut",
         snapshot["spring_thrust_washer_present"],
         snapshot["spring_thrust_washer_present"],
         "True; spring OD9.25 exceeds wingnut hub OD8"),
        ("spring touches thrust washer",
         snapshot["spring_thrust_washer_present"]
         and abs(snapshot["spring_to_thrust_washer_gap_mm"])
         <= MAX_STACK_CONTACT_GAP_MM,
         snapshot["spring_to_thrust_washer_gap_mm"],
         f"<= {MAX_STACK_CONTACT_GAP_MM:g} mm"),
        ("thrust washer touches wingnut bearing face",
         snapshot["spring_thrust_washer_present"]
         and abs(snapshot["thrust_washer_to_wingnut_gap_mm"])
         <= MAX_STACK_CONTACT_GAP_MM,
         snapshot["thrust_washer_to_wingnut_gap_mm"],
         f"<= {MAX_STACK_CONTACT_GAP_MM:g} mm"),
        ("selected spring is not beyond catalog compression in modeled pose",
         snapshot["selected_spring_coil_bind_margin_at_current_length_mm"]
         >= 0.0,
         snapshot["selected_spring_coil_bind_margin_at_current_length_mm"],
         ">= 0 mm"),
        ("modeled spring length lies in the designed adjustment band",
         band["minimum_installed_length_mm"] - 1e-6
         <= snapshot["modeled_spring_installed_length_mm"]
         <= band["maximum_installed_length_mm"] + 1e-6,
         snapshot["modeled_spring_installed_length_mm"],
         (f"{band['minimum_installed_length_mm']:.3f}.."
          f"{band['maximum_installed_length_mm']:.3f} mm")),
        ("M4 thread engagement covers low-preload end",
         geometry["minimum_thread_engagement_over_adjustment_mm"]
         >= MIN_THREAD_ENGAGEMENT_MM,
         geometry["minimum_thread_engagement_over_adjustment_mm"],
         f">= {MIN_THREAD_ENGAGEMENT_MM:g} mm"),
        ("hardware schedule selects exact spring SKU",
         schedule.get("sku") == SELECTED_SPRING.sku,
         schedule.get("sku"), SELECTED_SPRING.sku),
        ("hardware schedule no longer marks spring pending",
         schedule.get("status") not in {"selection_pending", "pending", None},
         schedule.get("status"), "selected/order_ready"),
        ("hardware schedule uses selected spring dimensions",
         schedule_geometry_matches,
         schedule_model, "spring_envelope with exact OD/ID and nominal length"),
        ("hardware schedule includes exact spring thrust washer",
         thrust_schedule_matches,
         thrust_schedule[0] if len(thrust_schedule) == 1 else thrust_schedule,
         f"1x McMaster {SPRING_THRUST_WASHER['sku']} selected/order_ready"),
    ]
    current_checks = [
        {"name": name, "pass": passed, "value": value, "requirement": req}
        for name, passed, value, req in current_checks
    ]

    sensitivity = []
    for mu in MU_SENSITIVITY:
        sensitivity.append({
            "mu": mu,
            "drag_at_minimum_design_preload_n": drag_from_preload(
                band["minimum_normal_force_n"], mu),
            "drag_at_nominal_design_preload_n": drag_from_preload(
                band["nominal_normal_force_n"], mu),
            "drag_at_maximum_design_preload_n": drag_from_preload(
                band["maximum_normal_force_n"], mu),
            "preload_for_1n_drag_n": preload_for_drag(1.0, mu),
            "preload_for_10n_drag_n": preload_for_drag(10.0, mu),
        })

    selected_ok = all(row["pass"] for row in spring_checks)
    current_ok = all(row["pass"] for row in current_checks)
    return {
        "schema": 1,
        "status": "PASS" if selected_ok and current_ok else "FAIL",
        "scope": {
            "modeled": [
                "straight-pass two-face Coulomb sizing",
                "catalog spring load, travel, OD/ID, M4 adjustment",
                "current placed stack faces, wire centering, and backing support",
            ],
            "not_modeled": [
                "felt friction hysteresis, wear, contamination, or speed dependence",
                "felt compression stiffness and pad-to-pad lot variation",
                "transient dancer response, wire sag, or spool inertia",
            ],
        },
        "role_separation": {
            "felt_tensioner": (
                "adjustable baseline drag from a spring-clamped straight felt pass"
            ),
            "dancer": (
                "separate moving pulley/spring system that stores wire and absorbs "
                "transient tension excursions"
            ),
        },
        "friction_model": {
            "equation": "drag_N = 2 * mu * normal_preload_N",
            "design_mu_range": [MU_DESIGN_MIN, MU_DESIGN_MAX],
            "nominal_mu": MU_NOMINAL,
            "basis": (
                "explicit tuning assumption only; no stable catalog coefficient "
                "exists for the actual felt/enamel/environment combination"
            ),
            "required_hardware_validation": (
                "calibrate drag with a pull gauge on production wire at operating "
                "speed; do not use wingnut turns as a force certificate"
            ),
        },
        "selected_spring": asdict(SELECTED_SPRING),
        "selected_spring_thrust_washer": SPRING_THRUST_WASHER,
        "selected_spring_checks": spring_checks,
        "design_preload_band": band,
        "per_wingnut_turn": {
            "normal_force_change_n": (
                SELECTED_SPRING.rate_n_per_mm * M4_COARSE_PITCH_MM),
            "drag_change_at_design_mu_min_n": drag_from_preload(
                SELECTED_SPRING.rate_n_per_mm * M4_COARSE_PITCH_MM,
                MU_DESIGN_MIN),
            "drag_change_at_nominal_mu_n": drag_from_preload(
                SELECTED_SPRING.rate_n_per_mm * M4_COARSE_PITCH_MM,
                MU_NOMINAL),
            "drag_change_at_design_mu_max_n": drag_from_preload(
                SELECTED_SPRING.rate_n_per_mm * M4_COARSE_PITCH_MM,
                MU_DESIGN_MAX),
        },
        "friction_sensitivity": sensitivity,
        "current_stack": snapshot,
        "recommended_geometry": geometry,
        "current_integration_checks": current_checks,
        "current_integration_ready": current_ok,
        "selected_spring_sizing_ready": selected_ok,
        "shared_changes_required": [
            {
                "files": ["cad/hardware.py", "cad/cots.py"],
                "change": (
                    "replace SPRING-TBD-FELT/OD10x12 placeholder with McMaster "
                    "94125K614; model OD9.25, ID6.75 at the nominal compressed "
                    f"length {band['nominal_installed_length_mm']:.3f} mm"
                ),
            },
            {
                "files": ["cad/hardware.py", "cad/hardware_placements.py", "cad/assembly.py"],
                "change": (
                    f"add 1x McMaster {SPRING_THRUST_WASHER['sku']} M4 OD12 "
                    "load washer between spring and wingnut; move the wingnut "
                    "bearing face from z=-140.400 to nominal "
                    f"z={geometry['nominal_wingnut_bearing_face_z_mm']:.3f}; "
                    "allow the full reported adjustment-face range"
                ),
            },
            {
                "files": ["cad/printed.py", "cad/hardware_placements.py"],
                "change": (
                    "re-center the unloaded 0.5 mm felt gap about wire z=-157.0 "
                    f"(faces z={geometry['target_fixed_pad_wire_face_z_mm']:.2f} "
                    f"and {geometry['target_moving_pad_wire_face_z_mm']:.2f}); "
                    f"shift both pads {geometry['pad_axial_shift_from_current_mm']:.2f} "
                    "mm, use the specified 1.0 mm backing discs (corrected spring "
                    f"seat z={geometry['spring_bearing_seat_z_mm']:.2f}), and "
                    "extend the printed annular boss to z="
                    f"{geometry['required_printed_boss_front_z_mm']:.2f} so the "
                    "fixed backing is actually seated"
                ),
            },
            {
                "files": ["cad/hardware.py", "cad/hardware_placements.py", "BOM.csv"],
                "change": (
                    "replace OD12 ISO7093 backing washers with two OD20 x ID4.5 "
                    "x 1.0 mm 304 stainless flat discs (custom/laser cut); the "
                    "existing cots OD20 envelope is geometrically correct"
                ),
            },
            {
                "files": ["BOM.csv", "out/order lists"],
                "change": (
                    "order one McMaster package (5 springs) of 94125K614 and "
                    f"one package of {SPRING_THRUST_WASHER['sku']} M4 load washers; "
                    "record four springs as spares; change the stud to DIN 976 "
                    "M4x55 so the low-preload end retains full wingnut engagement"
                ),
            },
        ],
    }


def _fmt(value: Any, digits: int = 3) -> str:
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def markdown_report(report: dict[str, Any]) -> str:
    spring = report["selected_spring"]
    band = report["design_preload_band"]
    stack = report["current_stack"]
    geom = report["recommended_geometry"]
    thrust = report["selected_spring_thrust_washer"]
    integration_note = (
        "The selected spring, OD20 backing discs, thrust washer, M4x55 stud, "
        "and corrected stack coordinates are present in the shared CAD and "
        "hardware schedule."
        if report["current_integration_ready"] else
        "The spring selection is valid, but the machine is not order-ready "
        "until every failed integration check below is corrected."
    )
    lines = [
        "# Felt tensioner spring and preload audit",
        "",
        f"**{report['status']} — selected spring sizing: "
        f"{'PASS' if report['selected_spring_sizing_ready'] else 'FAIL'}; "
        f"current CAD/BOM integration: "
        f"{'PASS' if report['current_integration_ready'] else 'FAIL'}.**",
        "",
        integration_note,
        "",
        "## Exact spring",
        "",
        f"- McMaster-Carr `{spring['sku']}` — {spring['description']}",
        f"- Source: {spring['source_url']} (table checked {spring['source_checked']})",
        (f"- Free length {_fmt(spring['free_length_mm'])} mm; OD "
         f"{_fmt(spring['od_mm'])} mm; ID {_fmt(spring['id_mm'])} mm; wire "
         f"{_fmt(spring['wire_d_mm'])} mm"),
        (f"- Rate {_fmt(spring['rate_n_per_mm'])} N/mm; maximum load "
         f"{_fmt(spring['max_load_n'])} N; catalog compressed length "
         f"{_fmt(spring['compressed_length_at_max_load_mm'])} mm"),
        f"- {spring['end_type']}; package quantity {spring['package_qty']}",
        (f"- Spring load washer: McMaster `{thrust['sku']}`, M4, ID "
         f"{_fmt(thrust['id_mm'])} mm, OD {_fmt(thrust['od_mm'])} mm, "
         f"{_fmt(thrust['thickness_range_mm'][0])}.."
         f"{_fmt(thrust['thickness_range_mm'][1])} mm thick"),
        "",
        "## Designed adjustment band",
        "",
        (f"For `mu=0.15..0.45`, 1..10 N drag requires "
         f"{_fmt(band['minimum_normal_force_n'])}.."
         f"{_fmt(band['maximum_normal_force_n'])} N spring preload."),
        (f"That is {_fmt(band['minimum_installed_length_mm'])}.."
         f"{_fmt(band['maximum_installed_length_mm'])} mm installed spring "
         f"length across {_fmt(band['wingnut_turns'])} M4 wingnut turns."),
        (f"The maximum design preload uses "
         f"{100 * band['maximum_normal_force_n'] / spring['max_load_n']:.1f}% "
         f"of catalog load and leaves "
         f"{band['minimum_installed_length_mm'] - spring['compressed_length_at_max_load_mm']:.3f} "
         "mm before the catalog compressed length."),
        (f"Nominal drawing pose: 5 N drag at assumed `mu=0.30`, "
         f"{_fmt(band['nominal_normal_force_n'])} N preload, "
         f"{_fmt(band['nominal_installed_length_mm'])} mm spring length."),
        "",
        "## Integrated geometry findings",
        "",
        (f"- Wire is {_fmt(stack['wire_radial_offset_from_stud_mm'])} mm off the "
         f"stud axis inside an OD20 felt pad; pad edge margin is "
         f"{_fmt(stack['pad_wire_edge_margin_mm'])} mm."),
        (f"- The OD20 backing-disc wire margin is "
         f"{_fmt(stack['backing_wire_edge_margin_mm'])} mm."),
        (f"- The unloaded felt-gap midpoint is z="
         f"{_fmt(stack['unloaded_pad_gap_mid_z_mm'])} mm while the wire is z="
         f"{_fmt(stack['wire_contact_xyz_mm'][2])} mm."),
        (f"- The fixed backing starts z={_fmt(stack['fixed_backing_rear_face_z_mm'])} "
         f"while the printed boss ends z="
         f"{_fmt(stack['printed_fixed_stack_boss_front_z_mm'])}, leaving an unloaded "
         f"{_fmt(stack['fixed_backing_to_boss_gap_mm'])} mm axial gap."),
        (f"- Modeled selected-spring length is "
         f"{_fmt(stack['modeled_spring_installed_length_mm'])} mm. Installing the "
         f"real spring leaves "
         f"{stack['selected_spring_coil_bind_margin_at_current_length_mm']:.3f} "
         "mm above its catalog compressed length."),
        (f"- The OD12 spring load washer and wingnut bearing face are at nominal z="
         f"{_fmt(geom['nominal_wingnut_bearing_face_z_mm'])} mm; adjustment range "
         f"z={_fmt(geom['wingnut_bearing_face_range_z_mm'][0])}.."
         f"{_fmt(geom['wingnut_bearing_face_range_z_mm'][1])} mm."),
        (f"- The M4x55 stud retains "
         f"{_fmt(geom['minimum_thread_engagement_over_adjustment_mm'])} mm engagement "
         "at minimum preload."),
        "",
        "## Current integration checks",
        "",
        "| Check | Result | Value | Requirement |",
        "|---|---:|---:|---|",
    ]
    for row in report["current_integration_checks"]:
        lines.append(
            f"| {row['name']} | {'PASS' if row['pass'] else 'FAIL'} | "
            f"{_fmt(row['value'])} | {row['requirement']} |"
        )
    lines += [
        "",
        "## Friction sensitivity",
        "",
        "| Assumed mu | Drag at min preload (N) | Drag at nominal preload (N) | Drag at max preload (N) |",
        "|---:|---:|---:|---:|",
    ]
    for row in report["friction_sensitivity"]:
        lines.append(
            f"| {row['mu']:.2f} | "
            f"{row['drag_at_minimum_design_preload_n']:.3f} | "
            f"{row['drag_at_nominal_design_preload_n']:.3f} | "
            f"{row['drag_at_maximum_design_preload_n']:.3f} |"
        )
    lines += [
        "",
        "## Felt versus dancer",
        "",
        "The felt stack supplies adjustable baseline drag. It does not regulate "
        "tension and it does not store wire. The separate spring-loaded dancer "
        "does those transient jobs. Set felt drag with a pull gauge on the actual "
        "wire at operating speed; do not infer drag from wingnut turns alone.",
        "",
        ("## Integrated shared-source changes" if
         report["current_integration_ready"] else
         "## Required shared-source changes"),
        "",
    ]
    if report["current_integration_ready"]:
        lines.append("All enumerated source changes are present; the machine-readable checks above are authoritative.")
    else:
        for change in report["shared_changes_required"]:
            lines.append(f"- `{', '.join(change['files'])}`: {change['change']}")
    lines += [
        "",
        "## Limits",
        "",
        "This is a catalog/static sizing audit. It does not certify felt friction, "
        "wear, hysteresis, spring fatigue, transient dancer behavior, spool inertia, "
        "wire sag, or snagging. Those require the documented hardware pull-gauge and "
        "running-machine tests.",
        "",
    ]
    return "\n".join(lines)


def write_reports(report: dict[str, Any] | None = None) -> tuple[Path, Path]:
    if report is None:
        report = audit()
    root = Path(__file__).resolve().parents[1]
    out = root / "out" / "reports"
    out.mkdir(parents=True, exist_ok=True)
    json_path = out / "felt_loads.json"
    md_path = out / "felt_loads.md"
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    md_path.write_text(markdown_report(report), encoding="utf-8")
    return json_path, md_path


def main() -> int:
    report = audit()
    json_path, md_path = write_reports(report)
    spring = report["selected_spring"]
    band = report["design_preload_band"]
    print(f"felt preload audit: {report['status']}")
    print(f"spring {spring['sku']}: free {spring['free_length_mm']:.3f} mm, "
          f"rate {spring['rate_n_per_mm']:.3f} N/mm")
    print(f"preload {band['minimum_normal_force_n']:.3f} .. "
          f"{band['maximum_normal_force_n']:.3f} N")
    print(f"installed length {band['minimum_installed_length_mm']:.3f} .. "
          f"{band['maximum_installed_length_mm']:.3f} mm")
    for row in report["current_integration_checks"]:
        print(f"{'PASS' if row['pass'] else 'FAIL'} {row['name']}: "
              f"{row['value']} ({row['requirement']})")
    print(json_path)
    print(md_path)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
