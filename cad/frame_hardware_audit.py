"""Source-BREP audit for the winder's static frame hardware.

This audit deliberately works from :mod:`assembly` and
:mod:`hardware_placements`, not from tessellated export files.  OpenCascade
common-volume booleans decide whether two solids genuinely interpenetrate;
bounding boxes are only a broad-phase accelerator.

Scope
-----

* all proposed HBKTST5 frame brackets, their M5 screws and slot nuts;
* every original static-link host from ``assembly.static_link()``;
* every other placed static-link hardware occurrence, so a bracket cannot
  silently occupy an endstop screw or another fastener stack;
* exact semantic classification of the intended bracket seats, T-slot nut
  capture, and screw/T-nut thread engagement.

The lightweight T-nut and thread models intentionally omit helical detail.
Positive common volume is therefore allowed only for explicitly classified
``thread_engagement`` or ``tslot_capture`` pairs.  Face seating, clearance
holes, and all unclassified pairs must have zero positive common volume.

This module does not modify the shared machine sources.  ``proposed_layout``
applies the smallest audit-only bracket removals/repositions so the report can
distinguish a diagnosed current fault from a geometry-proven repair contract.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from enum import Enum
import json
from pathlib import Path
from typing import Iterable, Mapping, Sequence

from build123d import Part

import assembly
import cots
import hardware
import hardware_placements as placements
import printed
from params import PARAMS as P


POSITIVE_VOLUME_TOL_MM3 = 1.0e-4
CONTACT_DISTANCE_TOL_MM = 1.0e-7
REPORT_PATH = Path(__file__).with_suffix(".report.md")
JSON_PATH = Path(__file__).with_suffix(".report.json")

FRAME_SCHEDULE_IDS = frozenset({
    "frame_brackets",
    "frame_bracket_screws",
    "frame_bracket_tnuts",
})


class ContactClass(str, Enum):
    """Mechanical meaning of one potential solid pair."""

    SEATED_FACE = "seated_face"
    CLEARANCE_PATH = "clearance_path"
    THREAD_ENGAGEMENT = "thread_engagement"
    TSLOT_CAPTURE = "tslot_capture"
    UNCLASSIFIED = "unclassified"


@dataclass(frozen=True)
class ContactPolicy:
    classification: ContactClass
    positive_volume_allowed: bool
    rationale: str


@dataclass(frozen=True)
class SolidRecord:
    label: str
    shape: Part
    source: str
    occurrence: placements.HardwareOccurrence | None = None


@dataclass(frozen=True)
class ContactFinding:
    a: str
    b: str
    source_a: str
    source_b: str
    classification: str
    positive_volume_allowed: bool
    common_volume_mm3: float
    bbox_overlap_mm: tuple[float, float, float]
    status: str
    rationale: str


@dataclass(frozen=True)
class LayoutResult:
    name: str
    bracket_count: int
    tested_broadphase_pairs: int
    positive_volume_pairs: int
    allowed_positive_volume_pairs: int
    forbidden_positive_volume_pairs: int
    findings: tuple[ContactFinding, ...]


@dataclass(frozen=True)
class ShoeAuditResult:
    printed_connectors: int
    selected_hardware: int
    shoe_cross_common_volume_mm3: float
    shoe_post_common_volume_mm3: float
    base_rail_gap_mm: float
    nearest_repositioned_bracket_gap_mm: float
    floor_bore_min_ligament_mm: float
    upright_bore_min_ligament_mm: float
    floor_head_min_edge_margin_mm: float
    upright_head_min_edge_margin_mm: float
    scallop_back_wall_mm: float
    scallop_to_upright_bore_ligament_mm: float
    allowed_positive_volume_pairs: tuple[tuple[str, str, float, str], ...]
    forbidden_positive_volume_pairs: tuple[tuple[str, str, float], ...]


def _bbox(shape: Part) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    bb = shape.bounding_box()
    return ((float(bb.min.X), float(bb.min.Y), float(bb.min.Z)),
            (float(bb.max.X), float(bb.max.Y), float(bb.max.Z)))


def bbox_overlap(a: Part, b: Part) -> tuple[float, float, float]:
    """Signed axis overlaps; any non-positive component means no volume."""
    amin, amax = _bbox(a)
    bmin, bmax = _bbox(b)
    return tuple(
        min(amax[i], bmax[i]) - max(amin[i], bmin[i])
        for i in range(3)
    )  # type: ignore[return-value]


def common_volume_mm3(a: Part, b: Part) -> float:
    """Return the exact OpenCascade common volume for two source solids."""
    overlaps = bbox_overlap(a, b)
    if any(value <= 0.0 for value in overlaps):
        return 0.0
    common = a.intersect(b)
    if common is None:
        return 0.0
    return float(sum(max(0.0, float(getattr(item, "volume", 0.0)))
                     for item in common))


def source_static_hosts() -> dict[str, SolidRecord]:
    """Build every authored host that can touch a frame bracket stack.

    These are the exact expressions from ``assembly.static_link``.  Building
    only the ten extrusion hosts plus the two specifically disputed parts
    avoids importing unrelated motors/bearings for a local frame audit.
    """
    ext = cots.extrusion_2020
    shapes = [
        assembly._at(ext(450), -P.base_rail_x, -215, P.frame_z0,
                     label="base_rail_L"),
        assembly._at(ext(450), P.base_rail_x, -215, P.frame_z0,
                     label="base_rail_R"),
        assembly._at(ext(180), -90, -235, -180, ry=90,
                     label="cross_rear"),
        assembly._at(ext(180), -90, -235, -50, ry=90,
                     label="cross_mid"),
        assembly._at(ext(180), -90, -235, 160, ry=90,
                     label="cross_front"),
        assembly._at(ext(P.stringer_len), -P.rail_x, -215, P.stringer_z0,
                     label="stringer_L"),
        assembly._at(ext(P.stringer_len), P.rail_x, -215, P.stringer_z0,
                     label="stringer_R"),
        assembly._at(ext(235), -P.post_x, -205,
                     sum(P.post_z) / 2.0, rx=-90, label="post_L"),
        assembly._at(ext(235), P.post_x, -205,
                     sum(P.post_z) / 2.0, rx=-90, label="post_R"),
        assembly._at(ext(305), P.rear_post_x, -225, P.rear_post_z,
                     rx=-90, label="rear_post"),
        assembly._at(cots.t8_screw(P.screw_len), P.screw_x, P.screw_y,
                     P.screw_z0, label="t8_screw"),
        printed.endstop_mount(),
    ]
    records: dict[str, SolidRecord] = {}
    for shape in shapes:
        label = str(shape.label)
        if label in records:
            raise AssertionError(f"duplicate static host label: {label}")
        records[label] = SolidRecord(label, shape, "assembly.static_link")
    return records


def _occurrence_records(
    geometry: placements.PlacementGeometry,
) -> tuple[dict[str, SolidRecord], dict[str, SolidRecord]]:
    frame: dict[str, SolidRecord] = {}
    other: dict[str, SolidRecord] = {}
    relevant_other_ids = {
        "endstop_pedestal_screws", "endstop_pedestal_tnuts",
        "endstop_switch_screws", "endstop_switch_washers",
        "endstop_switch_nylocs",
    }
    for occurrence in placements.static_occurrences(P, geometry):
        if (occurrence.schedule_id not in FRAME_SCHEDULE_IDS
                and occurrence.schedule_id not in relevant_other_ids):
            continue
        record = SolidRecord(
            occurrence.label,
            occurrence.build(),
            "hardware_placements.static_occurrences",
            occurrence,
        )
        target = frame if occurrence.schedule_id in FRAME_SCHEDULE_IDS else other
        if record.label in target:
            raise AssertionError(f"duplicate hardware label: {record.label}")
        target[record.label] = record
    return frame, other


def bracket_host_pairs(
    geometry: placements.PlacementGeometry,
) -> dict[str, tuple[str, str]]:
    """Map each bracket label to (floor host, upright host)."""
    result: dict[str, tuple[str, str]] = {}
    for frame in geometry.frame_brackets:
        fields = frame.joint.split(":")
        if fields[1] in {"rear", "mid", "front"}:
            cross, member = fields[1], fields[2]
            cross_host = f"cross_{cross}"
            member_host = {
                "base_L": "base_rail_L",
                "base_R": "base_rail_R",
                "stringer_L": "stringer_L",
                "stringer_R": "stringer_R",
            }[member]
            # Audit-only corrected frames are underslung at a cross front
            # face: their floor leg seats on the upper member's underside and
            # their upright seats against the cross.  Current top-corner
            # frames use the opposite assignment.
            if frame.z_dir == (0.0, -1.0, 0.0):
                floor, upright = member_host, cross_host
            else:
                floor, upright = cross_host, member_host
        elif fields[1] in {"post_L", "post_R"}:
            floor = "base_rail_L" if fields[1] == "post_L" else "base_rail_R"
            upright = fields[1]
        elif fields[1] == "rear_post":
            floor, upright = "cross_rear", "rear_post"
        else:  # pragma: no cover - protects future frame schema changes
            raise AssertionError(f"unknown bracket joint {frame.joint!r}")
        result[frame.label] = (floor, upright)
    return result


def _mate_members(
    records: Mapping[str, SolidRecord],
) -> dict[str, tuple[str, ...]]:
    grouped: dict[str, list[str]] = {}
    for record in records.values():
        if record.occurrence is not None:
            grouped.setdefault(record.occurrence.mate_id, []).append(record.label)
    return {key: tuple(sorted(value)) for key, value in grouped.items()}


def classify_pair(
    a: SolidRecord,
    b: SolidRecord,
    geometry: placements.PlacementGeometry,
    frame_records: Mapping[str, SolidRecord],
) -> ContactPolicy:
    """Classify one host/contact pair from exact occurrence metadata.

    This is the public host-contact classification API used by the tests and
    by downstream integration.  It does not infer permission from overlap:
    permission comes only from a named mechanical interface.
    """
    pair = {a.label, b.label}
    hosts = bracket_host_pairs(geometry)

    for bracket, (floor_host, upright_host) in hosts.items():
        if pair == {bracket, floor_host}:
            return ContactPolicy(
                ContactClass.SEATED_FACE, False,
                f"{bracket} floor leg seats on {floor_host}; positive volume is forbidden",
            )
        if pair == {bracket, upright_host}:
            return ContactPolicy(
                ContactClass.SEATED_FACE, False,
                f"{bracket} upright leg seats on {upright_host}; positive volume is forbidden",
            )

    # Each bracket owns two named stacks.  The screw crosses the clearance
    # hole; only the same-stack screw/T-nut interface represents threads.
    frame_mates = _mate_members(frame_records)
    for mate_id, labels in frame_mates.items():
        members = set(labels)
        if pair <= members and len(pair) == 2:
            schedule_ids = {
                frame_records[label].occurrence.schedule_id  # type: ignore[union-attr]
                for label in pair
            }
            if schedule_ids == {"frame_bracket_screws", "frame_bracket_tnuts"}:
                return ContactPolicy(
                    ContactClass.THREAD_ENGAGEMENT, True,
                    f"same mating stack {mate_id}: simplified nominal screw/T-nut thread",
                )

    # A frame T-nut is intentionally captive in one specific host slot.
    for frame in geometry.frame_brackets:
        floor_host, upright_host = hosts[frame.label]
        floor_tnut = f"{frame.label}_floor_tnut"
        upright_tnut = f"{frame.label}_upright_tnut"
        if pair == {floor_tnut, floor_host}:
            return ContactPolicy(
                ContactClass.TSLOT_CAPTURE, True,
                f"{floor_tnut} simplified envelope captured in {floor_host} slot",
            )
        if pair == {upright_tnut, upright_host}:
            return ContactPolicy(
                ContactClass.TSLOT_CAPTURE, True,
                f"{upright_tnut} simplified envelope captured in {upright_host} slot",
            )

        screw_labels = {
            f"{frame.label}_floor_m5x10",
            f"{frame.label}_upright_m5x10",
        }
        if frame.label in pair and pair & screw_labels:
            return ContactPolicy(
                ContactClass.CLEARANCE_PATH, False,
                f"{next(iter(pair & screw_labels))} must pass the bracket OD5.5 hole without solid overlap",
            )

    return ContactPolicy(
        ContactClass.UNCLASSIFIED, False,
        "no documented threaded, T-slot, press, or seated interface",
    )


def _candidate_pairs(
    frame: Mapping[str, SolidRecord],
    hosts: Mapping[str, SolidRecord],
    other_hardware: Mapping[str, SolidRecord],
) -> Iterable[tuple[SolidRecord, SolidRecord]]:
    # Frame hardware against every authored static host.
    for record in frame.values():
        for host in hosts.values():
            yield record, host

    # Frame hardware against frame hardware, once per unordered pair.
    values = list(frame.values())
    for index, left in enumerate(values):
        for right in values[index + 1:]:
            yield left, right

    # Other placed hardware can collide with a frame bracket/stack too.
    for record in frame.values():
        for other in other_hardware.values():
            yield record, other


def audit_layout(
    name: str,
    geometry: placements.PlacementGeometry,
) -> LayoutResult:
    hosts = source_static_hosts()
    frame, other_hardware = _occurrence_records(geometry)
    findings: list[ContactFinding] = []
    broadphase = 0
    positive = 0
    allowed = 0
    forbidden = 0

    for a, b in _candidate_pairs(frame, hosts, other_hardware):
        overlap = bbox_overlap(a.shape, b.shape)
        if any(value <= 0.0 for value in overlap):
            continue
        broadphase += 1
        # Exact OCC distance cheaply rejects bounding-box false positives.
        # A common-volume Boolean is still mandatory for touching/overlapping
        # source solids.
        if float(a.shape.distance_to(b.shape)) > CONTACT_DISTANCE_TOL_MM:
            continue
        volume = common_volume_mm3(a.shape, b.shape)
        if volume <= POSITIVE_VOLUME_TOL_MM3:
            continue
        positive += 1
        policy = classify_pair(a, b, geometry, frame)
        status = "allowed" if policy.positive_volume_allowed else "forbidden"
        if policy.positive_volume_allowed:
            allowed += 1
        else:
            forbidden += 1
        findings.append(ContactFinding(
            a=a.label,
            b=b.label,
            source_a=a.source,
            source_b=b.source,
            classification=policy.classification.value,
            positive_volume_allowed=policy.positive_volume_allowed,
            common_volume_mm3=round(volume, 6),
            bbox_overlap_mm=tuple(round(value, 6) for value in overlap),
            status=status,
            rationale=policy.rationale,
        ))

    findings.sort(key=lambda row: (row.status, -row.common_volume_mm3,
                                   row.a, row.b))
    return LayoutResult(
        name=name,
        bracket_count=sum(
            1 for record in frame.values()
            if record.occurrence is not None
            and record.occurrence.schedule_id == "frame_brackets"
        ),
        tested_broadphase_pairs=broadphase,
        positive_volume_pairs=positive,
        allowed_positive_volume_pairs=allowed,
        forbidden_positive_volume_pairs=forbidden,
        findings=tuple(findings),
    )


def audit_bracket_bodies(
    name: str,
    geometry: placements.PlacementGeometry,
) -> LayoutResult:
    """Fast exact pass over bracket bodies versus hosts and one another."""
    hosts = source_static_hosts()
    frame: dict[str, SolidRecord] = {}
    for bracket in geometry.frame_brackets:
        shape = bracket.location * hardware.angle_bracket_2020(bracket.label)
        shape.label = bracket.label
        occurrence = placements.HardwareOccurrence(
            link="static", label=bracket.label, schedule_id="frame_brackets",
            origin=bracket.origin, axis=None, mate_id=bracket.joint,
            mate_center=bracket.origin, bracket_frame=bracket,
        )
        frame[bracket.label] = SolidRecord(
            bracket.label, shape, "hardware_placements.BracketFrame", occurrence)

    findings: list[ContactFinding] = []
    broadphase = 0
    pairs: list[tuple[SolidRecord, SolidRecord]] = []
    for bracket in frame.values():
        pairs.extend((bracket, host) for host in hosts.values())
    values = list(frame.values())
    for index, left in enumerate(values):
        pairs.extend((left, right) for right in values[index + 1:])

    for a, b in pairs:
        overlap = bbox_overlap(a.shape, b.shape)
        if any(value <= 0.0 for value in overlap):
            continue
        broadphase += 1
        volume = common_volume_mm3(a.shape, b.shape)
        if volume <= POSITIVE_VOLUME_TOL_MM3:
            continue
        policy = classify_pair(a, b, geometry, frame)
        findings.append(ContactFinding(
            a.label, b.label, a.source, b.source,
            policy.classification.value, policy.positive_volume_allowed,
            round(volume, 6), tuple(round(value, 6) for value in overlap),
            "allowed" if policy.positive_volume_allowed else "forbidden",
            policy.rationale,
        ))
    findings.sort(key=lambda row: (-row.common_volume_mm3, row.a, row.b))
    allowed = sum(row.positive_volume_allowed for row in findings)
    return LayoutResult(
        name=name,
        bracket_count=len(frame),
        tested_broadphase_pairs=broadphase,
        positive_volume_pairs=len(findings),
        allowed_positive_volume_pairs=allowed,
        forbidden_positive_volume_pairs=len(findings) - allowed,
        findings=tuple(findings),
    )


def rejected_front_z_rear_post_volume_mm3() -> float:
    """Exact proof that the suggested 16th front-Z HBKT embeds the post."""
    frame = placements.BracketFrame(
        "frame_bracket_rear_post_front", "frame:rear_post:front",
        (-45.0, -225.0, -170.0),
        (1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0),
    )
    bracket = frame.location * hardware.angle_bracket_2020(frame.label)
    return common_volume_mm3(bracket, source_static_hosts()["rear_post"].shape)


def rear_post_left_shoe(*, corrected: bool = True) -> Part:
    """Custom printed second rear-post shoe supplied by root.

    The 14 mm floor fits the 15 mm corridor with a verified 1 mm rail gap.
    It replaces, rather than disguises, the impossible 25 mm HBKTST5.
    """
    floor = printed._box(-69.0, -55.0, -225.0, -219.0, -190.0, -170.0)
    upright = printed._box(-61.0, -55.0, -225.0, -200.0, -190.0, -170.0)
    shoe = floor + upright
    shoe -= printed._cyl_y(2.7, -226.0, -218.0,
                           x=-63.0, z=-180.0)
    upright_hole_y = -208.0 if corrected else -211.0
    shoe -= printed._cyl_x(2.7, -62.0, -54.0,
                           y=upright_hole_y, z=-180.0)
    if corrected:
        # The originally requested floor head overlaps the upright by
        # 62.827142 mm3.  This OD9.2 scallop provides 0.24 mm radial head
        # clearance while leaving a 3.4 mm back wall at the tightest section.
        shoe -= printed._cyl_y(4.6, -219.2, -213.2,
                               x=-63.0, z=-180.0)
    shoe.label = "rear_post_left_shoe"
    return shoe


def rear_post_left_shoe_hardware(*, corrected: bool = True) -> dict[str, Part]:
    """Two M5x12/HNTA5-5 stacks for :func:`rear_post_left_shoe`."""
    return {
        "rear_post_left_shoe_floor_m5x12": hardware.place(
            hardware.socket_head_cap_screw("M5", 12.0),
            (-63.0, -219.0, -180.0), axis="+y",
            label="rear_post_left_shoe_floor_m5x12",
        ),
        "rear_post_left_shoe_floor_tnut": hardware.place(
            hardware.tnut_slot6("M5"),
            (-63.0, -225.0, -180.0), axis="-y",
            label="rear_post_left_shoe_floor_tnut",
        ),
        "rear_post_left_shoe_upright_m5x12": hardware.place(
            hardware.socket_head_cap_screw("M5", 12.0),
            (-61.0, -208.0 if corrected else -211.0, -180.0), axis="-x",
            label="rear_post_left_shoe_upright_m5x12",
        ),
        "rear_post_left_shoe_upright_tnut": hardware.place(
            hardware.tnut_slot6("M5"),
            (-55.0, -208.0 if corrected else -211.0, -180.0), axis="+x",
            label="rear_post_left_shoe_upright_tnut",
        ),
    }


def audit_rear_post_left_shoe(
    geometry: placements.PlacementGeometry | None = None,
) -> ShoeAuditResult:
    """Exact source-BREP and edge-distance audit of the custom shoe."""
    geometry = geometry or proposed_layout()
    hosts = source_static_hosts()
    shoe = rear_post_left_shoe()
    hardware_parts = rear_post_left_shoe_hardware()
    brackets = {
        frame.label: frame.location * hardware.angle_bracket_2020(frame.label)
        for frame in geometry.frame_brackets
    }

    records: dict[str, Part] = {"rear_post_left_shoe": shoe, **hardware_parts}
    targets: dict[str, Part] = {**hosts_as_shapes(hosts), **brackets}
    allowed_exact = {
        frozenset(("rear_post_left_shoe_floor_m5x12",
                   "rear_post_left_shoe_floor_tnut")): "thread_engagement",
        frozenset(("rear_post_left_shoe_upright_m5x12",
                   "rear_post_left_shoe_upright_tnut")): "thread_engagement",
        frozenset(("rear_post_left_shoe_floor_tnut", "cross_rear")):
            "tslot_capture",
        frozenset(("rear_post_left_shoe_upright_tnut", "rear_post")):
            "tslot_capture",
        frozenset(("rear_post_left_shoe_floor_m5x12", "cross_rear")):
            "tslot_passage_envelope",
        frozenset(("rear_post_left_shoe_upright_m5x12", "rear_post")):
            "tslot_passage_envelope",
    }

    allowed: list[tuple[str, str, float, str]] = []
    forbidden: list[tuple[str, str, float]] = []
    tested: set[frozenset[str]] = set()

    # Shoe module versus every host/repositioned bracket.
    pair_rows: list[tuple[str, Part, str, Part]] = []
    for label, shape in records.items():
        pair_rows.extend((label, shape, target_label, target)
                         for target_label, target in targets.items())
    # Internal shoe-module pairs.
    values = list(records.items())
    for index, (left_label, left) in enumerate(values):
        pair_rows.extend((left_label, left, right_label, right)
                         for right_label, right in values[index + 1:])

    for a_label, a, b_label, b in pair_rows:
        key = frozenset((a_label, b_label))
        if len(key) < 2 or key in tested:
            continue
        tested.add(key)
        if any(value <= 0.0 for value in bbox_overlap(a, b)):
            continue
        if float(a.distance_to(b)) > CONTACT_DISTANCE_TOL_MM:
            continue
        volume = common_volume_mm3(a, b)
        if volume <= POSITIVE_VOLUME_TOL_MM3:
            continue
        if key in allowed_exact:
            allowed.append((a_label, b_label, round(volume, 6),
                            allowed_exact[key]))
        else:
            forbidden.append((a_label, b_label, round(volume, 6)))

    # The placed M5 head's transverse half-envelope is 4.36 mm.  Derive it
    # from the actual solid instead of duplicating the catalog dimension.
    floor_head_bb = _bbox(hardware_parts["rear_post_left_shoe_floor_m5x12"])
    floor_head_r = max(
        -63.0 - floor_head_bb[0][0], floor_head_bb[1][0] + 63.0,
        -180.0 - floor_head_bb[0][2], floor_head_bb[1][2] + 180.0,
    )
    upright_head_bb = _bbox(hardware_parts["rear_post_left_shoe_upright_m5x12"])
    upright_head_r = max(
        -208.0 - upright_head_bb[0][1], upright_head_bb[1][1] + 208.0,
        -180.0 - upright_head_bb[0][2], upright_head_bb[1][2] + 180.0,
    )

    nearest_bracket_gap = min(float(shoe.distance_to(shape))
                              for shape in brackets.values())
    return ShoeAuditResult(
        printed_connectors=1,
        selected_hardware=len(hardware_parts),
        shoe_cross_common_volume_mm3=round(
            common_volume_mm3(shoe, hosts["cross_rear"].shape), 6),
        shoe_post_common_volume_mm3=round(
            common_volume_mm3(shoe, hosts["rear_post"].shape), 6),
        base_rail_gap_mm=round(float(shoe.distance_to(
            hosts["base_rail_L"].shape)), 6),
        nearest_repositioned_bracket_gap_mm=round(nearest_bracket_gap, 6),
        floor_bore_min_ligament_mm=round(6.0 - 2.7, 6),
        upright_bore_min_ligament_mm=round(8.0 - 2.7, 6),
        floor_head_min_edge_margin_mm=round(6.0 - floor_head_r, 6),
        upright_head_min_edge_margin_mm=round(8.0 - upright_head_r, 6),
        scallop_back_wall_mm=round(-55.0 - (-63.0 + 4.6), 6),
        scallop_to_upright_bore_ligament_mm=round(
            (-208.0 - 2.7) - (-213.2), 6),
        allowed_positive_volume_pairs=tuple(sorted(allowed)),
        forbidden_positive_volume_pairs=tuple(sorted(forbidden)),
    )


def hosts_as_shapes(hosts: Mapping[str, SolidRecord]) -> dict[str, Part]:
    return {label: record.shape for label, record in hosts.items()}


def proposed_layout(
    geometry: placements.PlacementGeometry | None = None,
) -> placements.PlacementGeometry:
    """Return the audit-only candidate after exact bracket disposition.

    Five base/cross brackets and the two front/stringer brackets are moved to
    the cross front face/upper-member underside.  The impossible rear-post
    left HBKTST5 is removed: its 25 mm floor cannot fit in the 15 mm corridor
    between the rear post and left base rail.  ``rear_base_R`` is left in its
    already-clear current top-corner position to keep this correction minimal.
    """
    base = geometry or placements.current_geometry(P)
    underslung = {
        "frame_bracket_rear_base_L": (-P.base_rail_x, -225.0, -170.0),
        "frame_bracket_mid_base_L": (-P.base_rail_x, -225.0, -40.0),
        "frame_bracket_mid_base_R": (P.base_rail_x, -225.0, -40.0),
        "frame_bracket_front_base_L": (-P.base_rail_x, -225.0, 170.0),
        "frame_bracket_front_base_R": (P.base_rail_x, -225.0, 170.0),
        "frame_bracket_front_stringer_L": (-P.rail_x, -225.0, 170.0),
        "frame_bracket_front_stringer_R": (P.rail_x, -225.0, 170.0),
    }
    corrected: list[placements.BracketFrame] = []
    for frame in base.frame_brackets:
        if frame.label == "frame_bracket_rear_post_left":
            continue
        if frame.label in underslung:
            frame = replace(
                frame,
                origin=underslung[frame.label],
                x_dir=(1.0, 0.0, 0.0),
                y_dir=(0.0, 0.0, 1.0),
                z_dir=(0.0, -1.0, 0.0),
            )
        corrected.append(frame)
    return replace(base, frame_brackets=tuple(corrected))


def _layout_json(result: LayoutResult) -> dict[str, object]:
    data = asdict(result)
    data["findings"] = [asdict(row) for row in result.findings]
    return data


def write_reports(results: Sequence[LayoutResult], shoe: ShoeAuditResult) -> None:
    JSON_PATH.write_text(json.dumps({
        "schema": 1,
        "positive_volume_tolerance_mm3": POSITIVE_VOLUME_TOL_MM3,
        "layouts": [_layout_json(result) for result in results],
        "rejected_front_z_rear_post_common_volume_mm3":
            round(rejected_front_z_rear_post_volume_mm3(), 6),
        "rear_post_left_shoe": asdict(shoe),
    }, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# Static frame hardware source-BREP audit",
        "",
        ("OpenCascade common-volume booleans on the authored build123d "
         "solids; STL/FCL meshes are not used."),
        "",
    ]
    for result in results:
        lines += [
            f"## {result.name}",
            "",
            f"- Brackets: {result.bracket_count}",
            f"- Broad-phase candidate pairs: {result.tested_broadphase_pairs}",
            f"- Positive-volume pairs: {result.positive_volume_pairs}",
            f"- Allowed thread/T-slot engagements: {result.allowed_positive_volume_pairs}",
            f"- Forbidden collisions: {result.forbidden_positive_volume_pairs}",
            "",
        ]
        for row in result.findings:
            lines.append(
                f"- `{row.status}` `{row.a}` / `{row.b}`: "
                f"{row.common_volume_mm3:.6f} mm3, `{row.classification}` — "
                f"{row.rationale}"
            )
        lines.append("")
    lines += [
        "## Custom rear-post left shoe",
        "",
        f"- Shoe/cross common volume: {shoe.shoe_cross_common_volume_mm3:.6f} mm3",
        f"- Shoe/post common volume: {shoe.shoe_post_common_volume_mm3:.6f} mm3",
        f"- Gap to left base rail: {shoe.base_rail_gap_mm:.6f} mm",
        ("- Nearest repositioned HBKT gap: "
         f"{shoe.nearest_repositioned_bracket_gap_mm:.6f} mm"),
        f"- Floor bore minimum ligament: {shoe.floor_bore_min_ligament_mm:.3f} mm",
        f"- Upright bore minimum ligament: {shoe.upright_bore_min_ligament_mm:.3f} mm",
        f"- Floor head minimum edge margin: {shoe.floor_head_min_edge_margin_mm:.3f} mm",
        f"- Upright head minimum edge margin: {shoe.upright_head_min_edge_margin_mm:.3f} mm",
        f"- Scallop back wall: {shoe.scallop_back_wall_mm:.3f} mm",
        ("- Scallop-to-upright-bore ligament: "
         f"{shoe.scallop_to_upright_bore_ligament_mm:.3f} mm"),
        ("- Forbidden positive-volume pairs: "
         f"{len(shoe.forbidden_positive_volume_pairs)}"),
        "",
        "Allowed positive volumes are limited to the documented T-slot envelope:",
        "",
        *[
            f"- `{a}` / `{b}`: {volume:.6f} mm3, `{classification}`"
            for a, b, volume, classification in shoe.allowed_positive_volume_pairs
        ],
        "",
        "## Exact disposition",
        "",
        ("- Replace `frame_bracket_rear_post_left` and its four stack "
         "occurrences with the custom 14 mm-floor shoe and two M5x12/HNTA5-5 "
         "stacks. A 25 mm HBKT leg cannot fit the 15 mm X corridor."),
        ("- The requested shoe fastener centers initially failed: floor screw "
         "/ upright shoe = 62.827142 mm3 and the two screw envelopes = "
         "12.713730 mm3. Add the OD9.2 scallop y=-219.2..-213.2 and move "
         "the upright axis-X stack from y=-211 to y=-208."),
        ("- Reposition `rear_base_L`, `mid_base_L/R`, and "
         "`front_base_L/R` to their cross front faces/upper-member "
         "undersides with `x=(1,0,0), y=(0,0,1), z=(0,-1,0)`."),
        ("- Reposition `front_stringer_L/R` the same way at origins "
         "`(-45,-225,170)` and `(45,-225,170)`."),
        ("- The proposed 16th front-Z rear-post HBKT is rejected: exact "
         f"common volume with the rear post is "
         f"{rejected_front_z_rear_post_volume_mm3():.6f} mm3."),
        ("- `post_L_front` / T8 has zero common volume. It is a tangent "
         "manufacturing-risk contact, not an allowed engagement; add a 2 mm "
         "corner relief if tolerance clearance is required."),
        "",
    ]
    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    current_geometry = placements.current_geometry(P)
    current = audit_bracket_bodies(
        "production 15-HBKT plus custom-shoe layout (bracket bodies)",
        current_geometry,
    )
    corrected_geometry = proposed_layout(current_geometry)
    candidate = audit_bracket_bodies(
        "independently normalized 15-HBKT plus custom-shoe layout (HBKT bodies)",
        corrected_geometry,
    )
    shoe = audit_rear_post_left_shoe(corrected_geometry)
    write_reports((current, candidate), shoe)
    for result in (current, candidate):
        print(
            f"{result.name}: brackets={result.bracket_count} "
            f"positive={result.positive_volume_pairs} "
            f"allowed={result.allowed_positive_volume_pairs} "
            f"FORBIDDEN={result.forbidden_positive_volume_pairs}"
        )
        for row in result.findings:
            if row.status == "forbidden":
                print(f"  {row.a} <-> {row.b}: {row.common_volume_mm3:.6f} mm3")
    return (candidate.forbidden_positive_volume_pairs
            + len(shoe.forbidden_positive_volume_pairs))


if __name__ == "__main__":
    raise SystemExit(main())
