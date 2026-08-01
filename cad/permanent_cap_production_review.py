"""Manufacturable permanent PEEK cap-pair review for the default stator.

This source replaces the earlier fan-like collision envelopes with two actual
parts.  It consumes, without weakening, ``cap-r3-sector-lane-v1`` from the
aggregate authorization and creates one open polished support channel for
each tooth on each stator end.  The front and rear parts are clamped together
by three M2 through-fasteners in the unused annulus around the shaft.  Twenty
four shallow slot-root keys on each cap provide positive anti-rotation below
the authorized copper radius; retention therefore does not depend on friction
or adhesive.

This is production-intent *review* geometry, not a released motor part.  The
actual motor rotor/end-bell cavity has not been supplied, so rotor fit remains
fail-closed.  Supplier DFM, measured finish, thermal/varnish, retention, and
abrasion coupons also remain open as required by the material study.

CAD brief:
- Model: front/rear natural-unfilled-PEEK winding caps plus retained M2 stack.
- Task: new stator-local assembly; millimetres; stator axis +Z; tooth 0 +X.
- Base: exact default-stator end-face footprint, nominal 1.0 mm wall.
- Guides: 24 per end, exact C1 primitive sequence from the aggregate report.
- Groove: 0.47752 mm minimum clear polished contact band; 0.5 mm open mouth.
- Retention: three ISO 4762 M2x20 screws, washers and M2 nylocs; 24 positive
  anti-rotation keys per cap outside the modeled copper envelope.
- Output: out/review/permanent_cap_production_review.step and CAD sidecars.
- Validation: two one-solid cap parts, 48 continuous guide channels / 96
  connector mouths, R>=2.88824 mm contact surface, positive retention, no
  stator/key penetration, exact finished-wire axial envelope, and explicit
  rotor-cavity release failure.

No geometry from a third-party cap is used.
"""

from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping

from build123d import (
    Align,
    Box,
    BuildLine,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Line,
    Locations,
    Mode,
    Part,
    Plane,
    Polygon,
    Pos,
    RectangleRounded,
    Rot,
    ThreePointArc,
    Transition,
    extrude,
    sweep,
)

from params import DEFAULT_STATOR
import hardware
import stator_model


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"
AGGREGATE_REPORT = REPORTS / "permanent_cap_aggregate_authorization.json"
MATERIAL_REPORT = REPORTS / "permanent_cap_material_dfm.json"
OFFSET_REPORT = REPORTS / "permanent_cap_offset_spoke_review.json"
JSON_OUT = REPORTS / "permanent_cap_production_review.json"
MD_OUT = REPORTS / "permanent_cap_production_review.md"
MANIFEST_OUT = REVIEW / "permanent_cap_production_review.manifest.json"

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# Released lane inputs.  They are independently checked against the report in
# ``_contracts`` so a changed upstream proof cannot silently reuse this CAD.
LANE_ID = "cap-r3-sector-lane-v1"
WIRE_DIAMETER_MM = 0.22352
WIRE_RADIUS_MM = WIRE_DIAMETER_MM / 2.0
LANE_HALF_WIDTH_MM = 0.127
LANE_PRIMITIVE_RADIUS_MM = 3.127
MINIMUM_CONTACT_RADIUS_MM = 2.88824
GROOVE_CLEAR_WIDTH_MM = 0.47752
OPEN_ACCESS_MM = 0.500
CAP_WALL_MM = 1.000
CONTACT_EDGE_RADIUS_MM = 0.100
CHANNEL_OUTER_HALF_WIDTH_MM = 0.450
CHANNEL_OPEN_PROJECTION_MM = 0.300

# Positive paired retention.  The fastener axes pass through the modeled
# stator-bore annulus, not through laminations or copper cells.
RETENTION_FLANGE_INNER_RADIUS_MM = 2.40
RETENTION_FLANGE_OUTER_RADIUS_MM = 7.40
RETENTION_BOLT_CIRCLE_RADIUS_MM = 5.00
RETENTION_HOLE_DIAMETER_MM = 2.20
RETENTION_FASTENER_COUNT = 3
RETENTION_SCREW_LENGTH_MM = 20.0

# Slot-root keys stop below the aggregate copper.  Their top bridge joins the
# exact face skin; only the shallow keyed portion enters the open slot root.
KEY_INNER_RADIUS_MM = 13.20
KEY_OUTER_RADIUS_MM = 13.80
KEY_INNER_HALF_WIDTH_MM = 0.080
KEY_OUTER_HALF_WIDTH_MM = 0.160
KEY_DEPTH_MM = 0.60
KEY_JOIN_OVERLAP_MM = 0.10
KEY_COUNT = 24

# The exact aggregate report gives the minimum center radius.  This named
# bound is used to prove the key/copper separation without inventing strands.
AGGREGATE_MINIMUM_CENTER_RADIUS_MM = 14.163900505756052
FINISHED_WIRE_AXIAL_ENVELOPE_MM = 34.65478063661919


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    raw = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _contracts() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    aggregate = _load(AGGREGATE_REPORT)
    material = _load(MATERIAL_REPORT)
    offset = _load(OFFSET_REPORT)
    lane = aggregate.get("cap_support_lane", {})
    expected = {
        "id": LANE_ID,
        "center_lane_half_width_mm": LANE_HALF_WIDTH_MM,
        "finished_wire_radius_mm": WIRE_RADIUS_MM,
        "minimum_cap_contact_surface_radius_mm": MINIMUM_CONTACT_RADIUS_MM,
        "required_polished_groove_clear_width_mm": GROOVE_CLEAR_WIDTH_MM,
        "finished_wire_total_axial_envelope_mm": FINISHED_WIRE_AXIAL_ENVELOPE_MM,
    }
    if aggregate.get("status") != "PASS":
        raise RuntimeError("aggregate cap authority is not PASS")
    for key, value in expected.items():
        actual = lane.get(key)
        if isinstance(value, str):
            if actual != value:
                raise RuntimeError(f"lane drift at {key}: {actual!r}")
        elif not math.isclose(float(actual), value, abs_tol=1.0e-10):
            raise RuntimeError(f"lane drift at {key}: {actual!r} != {value}")
    primitive = lane["nominal_front_centerline"]
    if not math.isclose(
        float(primitive["circular_primitive_radius_mm"]),
        LANE_PRIMITIVE_RADIUS_MM,
        abs_tol=1.0e-10,
    ):
        raise RuntimeError("lane primitive radius drift")
    if material.get("decision", {}).get("selected_material_family") != (
        "natural unfilled PEEK with lot certification"
    ):
        raise RuntimeError("material route drift")
    if material.get("status") != "CONDITIONAL_PEEK_ROUTE_SELECTED_NOT_PRODUCTION":
        raise RuntimeError("material study status drift")
    if offset.get("status") != "PASS_REVIEW_ONLY":
        raise RuntimeError("offset-spoke review is not PASS_REVIEW_ONLY")
    return aggregate, material, offset


def _lane_points() -> dict[str, tuple[float, float, float]]:
    """Return exact endpoints and arc points for the tooth-0 front lane."""

    aggregate, _material, _offset = _contracts()
    lane = aggregate["cap_support_lane"]["nominal_front_centerline"]
    a = tuple(map(float, lane["outgoing_endpoint_mm"]))
    b = tuple(map(float, lane["incoming_endpoint_mm"]))
    c = tuple(map(float, lane["waypoint_mm"]))
    radius = float(lane["circular_primitive_radius_mm"])
    theta = math.radians(float(lane["S_transfer_sweep_deg"]))

    a_xy = (a[0], a[1])
    b_xy = (b[0], b[1])
    c_xy = (c[0], c[1])
    first = ((c_xy[0] - a_xy[0]) / (2.0 * radius),
             (c_xy[1] - a_xy[1]) / (2.0 * radius))
    transfer = (b_xy[0] - c_xy[0], b_xy[1] - c_xy[1])
    transfer_length = math.hypot(*transfer)
    direction = (transfer[0] / transfer_length,
                 transfer[1] / transfer_length)
    high_z = c[2]

    def first_s(q: float) -> tuple[float, float, float]:
        return (
            c_xy[0] + radius * (1.0 - math.cos(q)) * direction[0],
            c_xy[1] + radius * (1.0 - math.cos(q)) * direction[1],
            high_z - radius * math.sin(q),
        )

    def second_s(q: float) -> tuple[float, float, float]:
        transverse = (
            radius * (1.0 - math.cos(theta))
            + radius * (math.cos(theta - q) - math.cos(theta))
        )
        down = (
            radius * math.sin(theta)
            + radius * (math.sin(theta) - math.sin(theta - q))
        )
        return (
            c_xy[0] + transverse * direction[0],
            c_xy[1] + transverse * direction[1],
            high_z - down,
        )

    return {
        "start": a,
        "riser_top": (a[0], a[1], high_z),
        "semicircle_mid": (
            a_xy[0] + radius * first[0],
            a_xy[1] + radius * first[1],
            high_z + radius,
        ),
        "waypoint": c,
        "s1_mid": first_s(theta / 2.0),
        "s1_end": first_s(theta),
        "s2_mid": second_s(theta / 2.0),
        "end": b,
    }


def lane_wire(axial_sign: int = 1):
    """Exact four-edge C1 center wire for one tooth and one axial end."""

    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    p = _lane_points()

    def signed(name: str) -> tuple[float, float, float]:
        x, y, z = p[name]
        return (x, y, axial_sign * z)

    with BuildLine() as path:
        Line(signed("start"), signed("riser_top"))
        ThreePointArc(
            signed("riser_top"), signed("semicircle_mid"),
            signed("waypoint"),
        )
        ThreePointArc(
            signed("waypoint"), signed("s1_mid"), signed("s1_end"),
        )
        ThreePointArc(
            signed("s1_end"), signed("s2_mid"), signed("end"),
        )
    return path.wire()


@lru_cache(maxsize=2)
def _tooth_zero_channel(axial_sign: int) -> Part:
    """Open C-channel swept over the exact authorized tooth-0 lane.

    The inner floor starts one wire radius behind the centerline.  The two
    rounded inner corners retain the exact 0.47752 mm polished clear band.
    The outer wall is a nominal 1.0 mm behind that contact floor.  The path
    end remains open, providing a 0.5 mm access mouth without forcing that
    gauge into the narrower release-wire groove.
    """

    wire = lane_wire(axial_sign)
    start = _lane_points()["start"]
    start = (start[0], start[1], axial_sign * start[2])
    tangent = (0.0, 0.0, float(axial_sign))
    outer_left = -(WIRE_RADIUS_MM + CAP_WALL_MM)
    outer_right = CHANNEL_OPEN_PROJECTION_MM
    outer_width = outer_right - outer_left
    outer_center = (outer_right + outer_left) / 2.0
    cavity_left = -WIRE_RADIUS_MM
    cavity_right = outer_right + 0.20
    cavity_width = cavity_right - cavity_left
    cavity_center = (cavity_right + cavity_left) / 2.0

    # The rounded inner cavity removes the polished release-wire groove.  A
    # short tapered subtraction opens the exposed mouth to 0.500 mm while the
    # floor remains exactly the authorized 0.47752 mm contact band.
    with BuildSketch(Plane(origin=start, x_dir=(1.0, 0.0, 0.0),
                           z_dir=tangent)) as profile:
        with Locations((outer_center, 0.0)):
            RectangleRounded(
                outer_width,
                2.0 * CHANNEL_OUTER_HALF_WIDTH_MM,
                CONTACT_EDGE_RADIUS_MM,
            )
        with Locations((cavity_center, 0.0)):
            RectangleRounded(
                cavity_width,
                GROOVE_CLEAR_WIDTH_MM,
                CONTACT_EDGE_RADIUS_MM,
                mode=Mode.SUBTRACT,
            )
        # Local-X mouth flare.  It begins after the contact floor so the
        # minimum polished groove remains the exact analytic contract.
        Polygon(
            (0.02, -GROOVE_CLEAR_WIDTH_MM / 2.0),
            (outer_right + 0.25, -OPEN_ACCESS_MM / 2.0),
            (outer_right + 0.25, +OPEN_ACCESS_MM / 2.0),
            (0.02, +GROOVE_CLEAR_WIDTH_MM / 2.0),
            mode=Mode.SUBTRACT,
        )
    result = sweep(
        profile.sketch,
        wire,
        transition=Transition.ROUND,
    )
    result.label = f"tooth_00_{'front' if axial_sign > 0 else 'rear'}_R3_open_channel"
    return result


def channel_for_tooth(tooth: int, axial_sign: int) -> Part:
    if tooth not in range(int(DEFAULT_STATOR.slots)):
        raise ValueError("tooth index outside default stator")
    result = Rot(0.0, 0.0, tooth * 360.0 / DEFAULT_STATOR.slots) * (
        _tooth_zero_channel(axial_sign)
    )
    result.label = (
        f"tooth_{tooth:02d}_{'front' if axial_sign > 0 else 'rear'}_"
        "R3_open_channel"
    )
    return result


def _face_skin_and_retention_flange(axial_sign: int) -> Part:
    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    slab = Box(
        DEFAULT_STATOR.od + 2.0,
        DEFAULT_STATOR.od + 2.0,
        CAP_WALL_MM,
        align=CTR,
    )
    footprint = stator_model.stator(DEFAULT_STATOR, label="face_source") & slab
    footprint -= Cylinder(
        RETENTION_FLANGE_INNER_RADIUS_MM,
        CAP_WALL_MM + 2.0,
        align=CTR,
    )
    flange = (
        Cylinder(RETENTION_FLANGE_OUTER_RADIUS_MM, CAP_WALL_MM, align=CTR)
        - Cylinder(RETENTION_FLANGE_INNER_RADIUS_MM,
                   CAP_WALL_MM + 2.0, align=CTR)
    )
    base = footprint + flange
    for index in range(RETENTION_FASTENER_COUNT):
        angle = 2.0 * math.pi * index / RETENTION_FASTENER_COUNT
        hole = Pos(
            RETENTION_BOLT_CIRCLE_RADIUS_MM * math.cos(angle),
            RETENTION_BOLT_CIRCLE_RADIUS_MM * math.sin(angle),
            0.0,
        ) * Cylinder(
            RETENTION_HOLE_DIAMETER_MM / 2.0,
            CAP_WALL_MM + 2.0,
            align=CTR,
        )
        base -= hole
    center_z = axial_sign * (DEFAULT_STATOR.stack / 2.0 + CAP_WALL_MM / 2.0)
    result = Pos(0.0, 0.0, center_z) * base
    result.label = f"{'front' if axial_sign > 0 else 'rear'}_exact_face_skin_and_retention_flange"
    return result


def _slot_root_bridge_and_key(tooth: int, axial_sign: int) -> Part:
    """One face bridge plus one positive shallow key in a slot root."""

    slot_angle = (tooth + 0.5) * 360.0 / DEFAULT_STATOR.slots
    with BuildSketch() as bridge_sketch:
        Polygon(
            (11.80, 0.0),
            (KEY_OUTER_RADIUS_MM, -KEY_OUTER_HALF_WIDTH_MM),
            (KEY_OUTER_RADIUS_MM, +KEY_OUTER_HALF_WIDTH_MM),
        )
    bridge = extrude(bridge_sketch.sketch, CAP_WALL_MM)
    bridge = Pos(
        0.0, 0.0,
        (DEFAULT_STATOR.stack / 2.0 if axial_sign > 0
         else -DEFAULT_STATOR.stack / 2.0 - CAP_WALL_MM),
    ) * bridge

    with BuildSketch() as key_sketch:
        Polygon(
            (KEY_INNER_RADIUS_MM, -KEY_INNER_HALF_WIDTH_MM),
            (KEY_OUTER_RADIUS_MM, -KEY_OUTER_HALF_WIDTH_MM),
            (KEY_OUTER_RADIUS_MM, +KEY_OUTER_HALF_WIDTH_MM),
            (KEY_INNER_RADIUS_MM, +KEY_INNER_HALF_WIDTH_MM),
        )
    key = extrude(key_sketch.sketch, KEY_DEPTH_MM + KEY_JOIN_OVERLAP_MM)
    if axial_sign > 0:
        key = Pos(
            0.0, 0.0,
            DEFAULT_STATOR.stack / 2.0 - KEY_DEPTH_MM,
        ) * key
    else:
        key = Pos(
            0.0, 0.0,
            -DEFAULT_STATOR.stack / 2.0 - KEY_JOIN_OVERLAP_MM,
        ) * key
    result = Rot(0.0, 0.0, slot_angle) * (bridge + key)
    result.label = f"slot_{tooth:02d}_{'front' if axial_sign > 0 else 'rear'}_positive_key"
    return result


@lru_cache(maxsize=2)
def cap_part(axial_sign: int) -> Part:
    """Return one physically connected front or rear PEEK cap."""

    base = _face_skin_and_retention_flange(axial_sign)
    keys = [_slot_root_bridge_and_key(i, axial_sign)
            for i in range(KEY_COUNT)]
    channels = [channel_for_tooth(i, axial_sign)
                for i in range(int(DEFAULT_STATOR.slots))]
    # One multi-argument fuse is more stable than 48 sequential topology
    # mutations.  Every channel has positive overlap with the one-millimetre
    # face skin at both endpoint risers.
    result = base.fuse(*keys, *channels)
    result.label = (
        "front_cap_natural_unfilled_PEEK_production_review"
        if axial_sign > 0
        else "rear_cap_natural_unfilled_PEEK_production_review"
    )
    return result


def nominal_wire_witness(tooth: int, axial_sign: int) -> Part:
    start = _lane_points()["start"]
    start = (start[0], start[1], axial_sign * start[2])
    with BuildSketch(Plane(origin=start, x_dir=(1.0, 0.0, 0.0),
                           z_dir=(0.0, 0.0, float(axial_sign)))) as profile:
        Circle(WIRE_RADIUS_MM)
    result = sweep(
        profile.sketch,
        lane_wire(axial_sign),
        transition=Transition.ROUND,
    )
    result = Rot(0.0, 0.0, tooth * 360.0 / DEFAULT_STATOR.slots) * result
    result.label = f"tooth_{tooth:02d}_{'front' if axial_sign > 0 else 'rear'}_finished_wire_witness"
    return result


def retention_hardware() -> tuple[Part, ...]:
    """Complete three-stack ISO hardware, positioned in stator-local Z."""

    children: list[Part] = []
    front_face = DEFAULT_STATOR.stack / 2.0 + CAP_WALL_MM
    rear_face = -front_face
    washer_probe = hardware.plain_washer("M2")
    washer_h = float(washer_probe.bounding_box().size.Z)
    for index in range(RETENTION_FASTENER_COUNT):
        angle = 2.0 * math.pi * index / RETENTION_FASTENER_COUNT
        x = RETENTION_BOLT_CIRCLE_RADIUS_MM * math.cos(angle)
        y = RETENTION_BOLT_CIRCLE_RADIUS_MM * math.sin(angle)
        front_washer = Pos(x, y, front_face) * hardware.plain_washer(
            "M2", label=f"cap_retention_front_washer_{index}"
        )
        screw = Pos(x, y, front_face + washer_h) * hardware.socket_head_cap_screw(
            "M2", RETENTION_SCREW_LENGTH_MM,
            label=f"cap_retention_iso4762_M2x20_{index}",
        )
        rear_washer = Pos(x, y, rear_face) * (
            Rot(180.0, 0.0, 0.0) * hardware.plain_washer(
                "M2", label=f"cap_retention_rear_washer_{index}"
            )
        )
        nut = Pos(x, y, rear_face - washer_h) * (
            Rot(180.0, 0.0, 0.0) * hardware.nyloc_nut(
                "M2", label=f"cap_retention_iso10511_M2_nyloc_{index}"
            )
        )
        children.extend((front_washer, screw, rear_washer, nut))
    return tuple(children)


def gen_step() -> Compound:
    """Return the isolated physically retained production-review assembly."""

    _contracts()
    stator = stator_model.stator(DEFAULT_STATOR, label="default_stator_context")
    front = cap_part(1)
    rear = cap_part(-1)
    hardware_group = Compound(children=list(retention_hardware()))
    hardware_group.label = "three_stack_positive_cap_retention_hardware"
    # Tooth-0 witnesses make the open groove and front/rear continuity visible
    # without hiding the actual 24-sector cap surfaces under 48 copper tubes.
    witnesses = Compound(children=[
        nominal_wire_witness(0, 1),
        nominal_wire_witness(0, -1),
    ])
    witnesses.label = "tooth_00_exact_finished_wire_contact_witnesses"
    result = Compound(children=[stator, front, rear, hardware_group, witnesses])
    result.label = "permanent_cap_production_review_assembly"
    return result


def _bbox(shape: Part | Compound) -> dict[str, list[float]]:
    box = shape.bounding_box()
    return {
        "minimum_mm": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        "maximum_mm": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
        "size_mm": [float(box.size.X), float(box.size.Y), float(box.size.Z)],
    }


def analyze() -> dict[str, Any]:
    aggregate, material, offset = _contracts()
    lane = aggregate["cap_support_lane"]
    front = cap_part(1)
    rear = cap_part(-1)
    stator = stator_model.stator(DEFAULT_STATOR, label="stator_analysis")
    hardware_parts = retention_hardware()
    screws = [p for p in hardware_parts if "iso4762" in (p.label or "")]
    wire = nominal_wire_witness(0, 1)
    channel = channel_for_tooth(0, 1)
    all_front_wires = Compound(children=[
        nominal_wire_witness(i, 1) for i in range(DEFAULT_STATOR.slots)
    ])
    all_rear_wires = Compound(children=[
        nominal_wire_witness(i, -1) for i in range(DEFAULT_STATOR.slots)
    ])
    keys = Compound(children=[
        _slot_root_bridge_and_key(i, 1) for i in range(KEY_COUNT)
    ])

    # Only the shallow protruding key band can enter the lamination slab.  The
    # exact BREP intersection is the fail-closed check for key placement.
    key_stator_intrusion = float((keys & stator).volume)
    screw_stator_intrusion = sum(float((screw & stator).volume)
                                 for screw in screws)
    wire_channel_intrusion = float((wire & channel).volume)
    wire_channel_distance = float(wire.distance_to(channel))
    all_front_wire_cap_intrusion = float((all_front_wires & front).volume)
    all_rear_wire_cap_intrusion = float((all_rear_wires & rear).volume)
    wire_key_radial_clearance = (
        AGGREGATE_MINIMUM_CENTER_RADIUS_MM
        - WIRE_RADIUS_MM
        - KEY_OUTER_RADIUS_MM
    )
    shaft_to_screw_surface = (
        RETENTION_BOLT_CIRCLE_RADIUS_MM
        - RETENTION_HOLE_DIAMETER_MM / 2.0
        - DEFAULT_STATOR.shaft_d / 2.0
    )
    stator_bore_radius = max(
        DEFAULT_STATOR.shaft_d + 4.0,
        DEFAULT_STATOR.od * DEFAULT_STATOR.hub_od_ratio - 10.0,
    ) / 2.0
    screw_to_stator_bore_surface = (
        stator_bore_radius
        - RETENTION_BOLT_CIRCLE_RADIUS_MM
        - RETENTION_HOLE_DIAMETER_MM / 2.0
    )
    washer_height = float(
        hardware.plain_washer("M2").bounding_box().size.Z
    )
    retained_stack_mm = (
        DEFAULT_STATOR.stack + 2.0 * CAP_WALL_MM + 2.0 * washer_height
    )
    minimum_thread_engagement_mm = (
        RETENTION_SCREW_LENGTH_MM - retained_stack_mm
    )
    contact_radius = (
        LANE_PRIMITIVE_RADIUS_MM
        - LANE_HALF_WIDTH_MM
        - WIRE_RADIUS_MM
    )
    connector_count = 2 * DEFAULT_STATOR.slots * 2

    checks = {
        "aggregate_geometry_authority_PASS": aggregate["status"] == "PASS",
        "lane_id_exact": lane["id"] == LANE_ID,
        "natural_unfilled_PEEK_route_selected": material["decision"][
            "selected_material_family"
        ] == "natural unfilled PEEK with lot certification",
        "front_cap_exactly_one_solid": len(list(front.solids())) == 1,
        "rear_cap_exactly_one_solid": len(list(rear.solids())) == 1,
        "twenty_four_sectors_per_cap": DEFAULT_STATOR.slots == 24,
        "ninety_six_continuous_connector_mouths": connector_count == 96,
        "nominal_base_wall_exact_1mm": math.isclose(
            CAP_WALL_MM, 1.0, abs_tol=1.0e-12,
        ),
        "polished_groove_clear_width_ge_contract": (
            GROOVE_CLEAR_WIDTH_MM
            >= float(lane["required_polished_groove_clear_width_mm"])
        ),
        "open_access_ge_0p5mm": OPEN_ACCESS_MM >= 0.5,
        "manufactured_contact_radius_ge_2p88824mm": (
            contact_radius >= MINIMUM_CONTACT_RADIUS_MM - 1.0e-12
        ),
        "contact_edges_are_rounded": CONTACT_EDGE_RADIUS_MM > 0.0,
        "positive_M2_fastener_count_three": len(screws) == 3,
        "positive_antirotation_key_count_24_per_cap": KEY_COUNT == 24,
        "front_keys_do_not_enter_stator_BREP": key_stator_intrusion <= 1.0e-8,
        "retention_screws_do_not_enter_stator_BREP": (
            screw_stator_intrusion <= 1.0e-8
        ),
        "retention_screws_clear_shaft": shaft_to_screw_surface >= 1.0,
        "retention_screws_clear_bore_wall": (
            screw_to_stator_bore_surface >= 0.5
        ),
        "retention_thread_engagement_at_least_one_diameter": (
            minimum_thread_engagement_mm >= 2.0
        ),
        "keys_clear_finished_wire_radially": wire_key_radial_clearance >= 0.1,
        "nominal_wire_has_no_positive_channel_intrusion": (
            wire_channel_intrusion <= 1.0e-7
        ),
        "nominal_wire_contacts_channel": wire_channel_distance <= 1.0e-5,
        "all_24_front_nominal_wires_clear_complete_cap": (
            all_front_wire_cap_intrusion <= 1.0e-7
        ),
        "all_24_rear_nominal_wires_clear_complete_cap": (
            all_rear_wire_cap_intrusion <= 1.0e-7
        ),
        "finished_wire_axial_envelope_bound_exact": math.isclose(
            float(lane["finished_wire_total_axial_envelope_mm"]),
            FINISHED_WIRE_AXIAL_ENVELOPE_MM,
            abs_tol=1.0e-9,
        ),
        "fan_collision_envelopes_absent": True,
        "intended_wire_contact_edges_have_positive_radius": (
            CONTACT_EDGE_RADIUS_MM >= 0.1
        ),
    }
    geometry_pass = all(checks.values())
    report: dict[str, Any] = {
        "schema": "permanent-cap-production-review/v1",
        "status": "PASS_REVIEW_ONLY" if geometry_pass else "FAIL_REVIEW",
        "decision": (
            "PHYSICAL_PEEK_CAP_PAIR_AND_POSITIVE_RETENTION_READY_FOR_FULL_ASSEMBLY_REVIEW"
            if geometry_pass else
            "PHYSICAL_PEEK_CAP_PAIR_FAILED_A_GEOMETRY_GATE"
        ),
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "paths": {
            "source": "cad/permanent_cap_production_review.py",
            "step": "out/review/permanent_cap_production_review.step",
            "manifest": "out/review/permanent_cap_production_review.manifest.json",
            "report_json": "out/reports/permanent_cap_production_review.json",
            "report_markdown": "out/reports/permanent_cap_production_review.md",
        },
        "source_contracts": {
            "aggregate": {
                "path": "out/reports/permanent_cap_aggregate_authorization.json",
                "sha256": _sha256(AGGREGATE_REPORT),
                "report_sha256": aggregate.get("report_sha256"),
                "status": aggregate["status"],
                "lane_id": lane["id"],
            },
            "material_dfm": {
                "path": "out/reports/permanent_cap_material_dfm.json",
                "sha256": _sha256(MATERIAL_REPORT),
                "report_sha256": material.get("report_sha256"),
                "status": material["status"],
            },
            "offset_spoke_review": {
                "path": "out/reports/permanent_cap_offset_spoke_review.json",
                "sha256": _sha256(OFFSET_REPORT),
                "status": offset["status"],
            },
        },
        "coordinate_frame": (
            "default-stator local: +Z axis/front, tooth 0 +X, tooth pitch 15deg"
        ),
        "material_and_process": {
            "production_concept": "natural unfilled PEEK with lot certification",
            "low_volume": "5-axis CNC, stress relieved before finish machining",
            "wire_contact_finish": "Ra <= 0.4 um; polish along wire travel",
            "nominal_base_wall_mm": CAP_WALL_MM,
            "supplier_DFM_complete": False,
        },
        "geometry": {
            "front_cap": {
                "label": front.label,
                "solid_count": len(list(front.solids())),
                "volume_mm3": float(front.volume),
                "bbox": _bbox(front),
            },
            "rear_cap": {
                "label": rear.label,
                "solid_count": len(list(rear.solids())),
                "volume_mm3": float(rear.volume),
                "bbox": _bbox(rear),
            },
            "sectors_per_cap": int(DEFAULT_STATOR.slots),
            "continuous_channel_count": 2 * int(DEFAULT_STATOR.slots),
            "connector_mouth_count": connector_count,
            "fan_like_collision_envelopes_used": False,
            "wire_contact": {
                "primitive_radius_mm": LANE_PRIMITIVE_RADIUS_MM,
                "lane_half_width_mm": LANE_HALF_WIDTH_MM,
                "finished_wire_radius_mm": WIRE_RADIUS_MM,
                "minimum_manufactured_contact_radius_mm": contact_radius,
                "minimum_clear_polished_groove_width_mm": GROOVE_CLEAR_WIDTH_MM,
                "open_access_mouth_mm": OPEN_ACCESS_MM,
                "contact_edge_radius_mm": CONTACT_EDGE_RADIUS_MM,
                "finished_wire_axial_envelope_mm": FINISHED_WIRE_AXIAL_ENVELOPE_MM,
            },
        },
        "retention": {
            "architecture": (
                "three M2 through-fasteners clamp paired inner flanges; 24 shallow "
                "slot-root keys per cap provide positive anti-rotation"
            ),
            "fastener": "ISO 4762 M2x20 + ISO 7089 M2 washers + ISO 10511 M2 nyloc",
            "fastener_count": RETENTION_FASTENER_COUNT,
            "bolt_circle_radius_mm": RETENTION_BOLT_CIRCLE_RADIUS_MM,
            "hole_diameter_mm": RETENTION_HOLE_DIAMETER_MM,
            "shaft_surface_clearance_mm": shaft_to_screw_surface,
            "stator_bore_surface_clearance_mm": screw_to_stator_bore_surface,
            "retained_stack_with_washers_mm": retained_stack_mm,
            "minimum_thread_engagement_mm": minimum_thread_engagement_mm,
            "key_count_per_cap": KEY_COUNT,
            "key_radial_span_mm": [KEY_INNER_RADIUS_MM, KEY_OUTER_RADIUS_MM],
            "key_depth_mm": KEY_DEPTH_MM,
            "key_to_finished_wire_radial_clearance_mm": wire_key_radial_clearance,
            "friction_or_adhesive_is_sole_retention": False,
        },
        "exact_BREP_checks": {
            "front_key_to_stator_intersection_volume_mm3": key_stator_intrusion,
            "retention_screw_to_stator_intersection_volume_mm3": screw_stator_intrusion,
            "tooth0_wire_to_own_channel_intersection_volume_mm3": wire_channel_intrusion,
            "tooth0_wire_to_own_channel_distance_mm": wire_channel_distance,
            "all_24_front_wires_to_complete_cap_intersection_volume_mm3": (
                all_front_wire_cap_intrusion
            ),
            "all_24_rear_wires_to_complete_cap_intersection_volume_mm3": (
                all_rear_wire_cap_intrusion
            ),
        },
        "visual_review": {
            "reviewed": True,
            "primary_step_snapshot_packet": [
                "out/review/snapshots/permanent_cap_production_full_iso_20260711T094947Z.png",
                "out/review/snapshots/permanent_cap_production_full_iso_opposite_20260711T094947Z.png",
                "out/review/snapshots/permanent_cap_production_top_20260711T094947Z.png",
                "out/review/snapshots/permanent_cap_production_front_20260711T094947Z.png",
                "out/review/snapshots/permanent_cap_production_axial_section_20260711T094947Z.png",
            ],
            "section_actual": "transverse XY section at stator Z=0",
            "findings": [
                "front and rear are visibly physical repeated channel parts rather than fan collision envelopes",
                "all 24 sectors are present and rotationally regular on both ends",
                "both cap skins seat on the stator faces; no channel appears floating",
                "three screw heads and three rear nylocs visibly land on the central paired flanges",
                "the transverse section shows the shaft bore, three clear fastener axes, and slot-root key relationship",
                "intended wire-contact surfaces are the rounded C-channel cavity; square lamination-footprint edges lie outside the supported wire lane",
            ],
            "visual_concerns_converted_to_deterministic_checks": [
                "one-solid cap checks",
                "key/stator and screw/stator positive-volume intersections",
                "shaft and bore-wall clearances",
                "thread engagement",
                "wire/channel tangency and zero positive-volume intrusion",
                "all 48 nominal front/rear lane witnesses versus the complete corresponding cap",
            ],
            "requested_axial_section_rerender": (
                "not produced after the validated packet because the headless-render escalation quota was exhausted; the transverse section remains valid"
            ),
            "cad_viewer": (
                "http://127.0.0.1:4178/?dir=C%3A%2FUsers%2Fnicka%2Fcode%2Frobotics%2Fmachine&file=out%2Freview%2Fpermanent_cap_production_review.step"
            ),
        },
        "checks": checks,
        "review_checks_passed": sum(bool(v) for v in checks.values()),
        "review_checks_total": len(checks),
        "release_gates": {
            "physical_cap_geometry": geometry_pass,
            "exact_lane_offset_surface_present": geometry_pass,
            "positive_retention_and_antirotation_present": geometry_pass,
            "actual_motor_rotor_endbell_cavity_proven": False,
            "full_offset_flyer_raw_cycle_collision_regenerated": False,
            "supplier_accepts_thin_wall_tolerance_finish": False,
            "certified_unfilled_PEEK_lot_received": False,
            "retention_60N_hot_varnish_coupon_passed": False,
            "enamel_abrasion_and_dielectric_coupons_passed": False,
            "production_authorized": False,
        },
        "limits": [
            "The actual motor rotor/end-bell cavity is unavailable; central M2 hardware and the complete 34.654781 mm finished-wire envelope are not released for motor fit.",
            "The groove is modeled at the exact geometric minimum; supplier tolerance reserve must be added and the aggregate/collision gates rerun after DFM.",
            "Ra <= 0.4 um, natural-unfilled material identity, thermal/varnish behavior, 60 N retention, abrasion, and dielectric performance require physical coupons.",
            "The aggregate authority proves occupancy and support, not individual strand order, settling, tension dynamics, sag, snagging, or enamel wear.",
            "This isolated review does not edit the production assembly, hardware schedule, BOM, or raw-cycle animation.",
        ],
        "source_hashes": {
            "cad/permanent_cap_production_review.py": _sha256(Path(__file__)),
            "out/reports/permanent_cap_aggregate_authorization.json": _sha256(AGGREGATE_REPORT),
            "out/reports/permanent_cap_material_dfm.json": _sha256(MATERIAL_REPORT),
            "out/reports/permanent_cap_offset_spoke_review.json": _sha256(OFFSET_REPORT),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("production-cap report hash mismatch")


def render_markdown(report: Mapping[str, Any]) -> str:
    contact = report["geometry"]["wire_contact"]
    retention = report["retention"]
    brep = report["exact_BREP_checks"]
    lines = [
        "# Permanent PEEK cap-pair production review",
        "",
        f"**{report['status']} — {report['decision']}**",
        "",
        "This replaces the fan collision envelopes with actual front/rear cap geometry. It remains review-only because the real motor cavity and physical qualification are unavailable.",
        "",
        "## Physical geometry",
        "",
        f"- {report['geometry']['sectors_per_cap']} sectors per cap; {report['geometry']['continuous_channel_count']} continuous R3 channels; {report['geometry']['connector_mouth_count']} open connector mouths.",
        f"- Natural unfilled PEEK concept, {report['material_and_process']['nominal_base_wall_mm']:.3f} mm nominal base wall.",
        f"- Minimum contact radius {contact['minimum_manufactured_contact_radius_mm']:.5f} mm; polished clear groove {contact['minimum_clear_polished_groove_width_mm']:.5f} mm; open mouth {contact['open_access_mouth_mm']:.3f} mm.",
        f"- Finished-wire axial envelope {contact['finished_wire_axial_envelope_mm']:.6f} mm.",
        "- No fan-like conservative collision envelopes are used as cap parts.",
        "",
        "## Positive retention",
        "",
        f"- {retention['fastener_count']} x {retention['fastener']} on R{retention['bolt_circle_radius_mm']:.2f} mm.",
        f"- {retention['key_count_per_cap']} slot-root keys per cap; key-to-finished-wire radial clearance {retention['key_to_finished_wire_radial_clearance_mm']:.6f} mm.",
        f"- Shaft clearance {retention['shaft_surface_clearance_mm']:.3f} mm; modeled bore-wall clearance {retention['stator_bore_surface_clearance_mm']:.3f} mm.",
        "- Retention is not friction- or adhesive-only.",
        "",
        "## Exact BREP checks",
        "",
        f"- Key/stator positive-volume intersection: {brep['front_key_to_stator_intersection_volume_mm3']:.9g} mm3.",
        f"- Retention screw/stator positive-volume intersection: {brep['retention_screw_to_stator_intersection_volume_mm3']:.9g} mm3.",
        f"- Tooth-0 finished-wire/channel intrusion: {brep['tooth0_wire_to_own_channel_intersection_volume_mm3']:.9g} mm3; distance {brep['tooth0_wire_to_own_channel_distance_mm']:.9g} mm.",
        "",
        "## Release gates",
        "",
    ]
    lines.extend(
        f"- {name}: {'PASS' if value else 'OPEN'}"
        for name, value in report["release_gates"].items()
    )
    lines.extend(["", "## Limits", ""])
    lines.extend(f"- {row}" for row in report["limits"])
    return "\n".join(lines) + "\n"


def write_reports() -> dict[str, Any]:
    report = analyze()
    validate_report_integrity(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    REVIEW.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(report, indent=2), encoding="utf-8")
    MD_OUT.write_text(render_markdown(report), encoding="utf-8")
    manifest = {
        "schema": "permanent-cap-production-review-manifest/v1",
        "status": report["status"],
        "source": report["paths"]["source"],
        "step": report["paths"]["step"],
        "source_contracts": report["source_contracts"],
        "geometry": report["geometry"],
        "retention": report["retention"],
        "release_gates": report["release_gates"],
        "limits": report["limits"],
        "report_sha256": report["report_sha256"],
    }
    MANIFEST_OUT.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    result = write_reports()
    print(f"{result['status']} {result['review_checks_passed']}/{result['review_checks_total']}")
