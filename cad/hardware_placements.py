"""Parametric hardware occurrences for the four winder kinematic links.

This module is deliberately isolated from :mod:`assembly` and
:mod:`printed`.  It consumes the dimensioned catalog in :mod:`hardware` and a
small, explicit geometry contract derived from ``params.PARAMS``.  Assembly
can therefore add the returned labeled parts without creating an import
cycle, and retention redesigns can pass a replacement :class:`PlacementGeometry`
without changing this file.

Coordinate convention follows the machine CAD: millimetres; X horizontal, Y
up, Z along the flyer axis.  For screws, ``axis`` points from the shank toward
the head (the local +Z convention in ``hardware.place``), so the shank extends
opposite ``axis``.  Nuts, washers, inserts and studs use ``axis`` as their
local +Z direction.  Every item in a mating stack carries one ``mate_id`` and
one common ``mate_center`` on its hole axis; deterministic tests use these to
catch transverse placement drift.

Hardware-only corrections are already reflected here for the unchanged frame,
M0, M1 and passive wire-supply baseline.  Final flyer, paired-cap and
active-sector hardware is owned by ``integrated_release_candidate.py`` and its
two source-level successor modules; it is intentionally not duplicated into
this legacy assembly helper.  Schedule rows declare that separate placement
authority explicitly.

The dancer extension spring remains a procurement selection rather than a
false exact occurrence; its fixed and moving anchor holes are recorded as an
issue until a spring and working pose are selected.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence

from build123d import Part, Plane

import hardware
import carriage_endstop_flag
from params import PARAMS as DEFAULT_PARAMS


Vec3 = tuple[float, float, float]

AXIS_VECTOR: dict[str, Vec3] = {
    "+x": (1.0, 0.0, 0.0),
    "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0),
    "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0),
    "-z": (0.0, 0.0, -1.0),
}


def _vadd(*vectors: Sequence[float]) -> Vec3:
    return tuple(float(sum(v[i] for v in vectors)) for i in range(3))  # type: ignore[return-value]


def _vscale(vector: Sequence[float], scale: float) -> Vec3:
    return tuple(float(scale * value) for value in vector)  # type: ignore[return-value]


def _axis_name(vector: Sequence[float]) -> str:
    rounded = tuple(int(round(float(value))) for value in vector)
    for name, candidate in AXIS_VECTOR.items():
        if rounded == candidate:
            return name
    raise ValueError(f"hardware axes must be cardinal; got {tuple(vector)!r}")


def _opposite(axis: str) -> str:
    return ("-" if axis[0] == "+" else "+") + axis[1]


def _radial_point(x: float, y: float, radius: float, angle_deg: float) -> Vec3:
    angle = math.radians(angle_deg)
    return (x + radius * math.cos(angle), y + radius * math.sin(angle), 0.0)


@dataclass(frozen=True)
class BracketFrame:
    """Right-handed placed frame for one HBKTST5 angle bracket.

    The catalog bracket's local X is its 20 mm width, local Y is the floor
    leg, and local Z is the upright leg.  ``origin`` is the outside corner at
    the supporting extrusion surface.
    """

    label: str
    joint: str
    origin: Vec3
    x_dir: Vec3
    y_dir: Vec3
    z_dir: Vec3

    def point(self, x: float, y: float, z: float) -> Vec3:
        return _vadd(
            self.origin,
            _vscale(self.x_dir, x),
            _vscale(self.y_dir, y),
            _vscale(self.z_dir, z),
        )

    @property
    def location(self):
        return Plane(origin=self.origin, x_dir=self.x_dir,
                     z_dir=self.z_dir).location


@dataclass(frozen=True)
class PlacementGeometry:
    """Current hole/surface contract consumed by the placement functions.

    Values are separated from BREP generation on purpose.  If a retention
    feature moves, root can call :func:`current_geometry`, replace only the
    affected field(s), and pass the resulting object to every public builder.
    """

    frame_brackets: tuple[BracketFrame, ...]
    rail_hole_z: tuple[float, ...]
    rail_counterbore_y: float
    rail_slot_surface_y: float
    plate_top_y: float
    plate_bottom_y: float
    tower_top_y: float
    nut_bracket_top_y: float
    t8_flange_rear_z: float
    t8_bracket_front_z: float
    m0_mount_top_y: float
    m0_support_top_y: float
    endstop_pedestal_hole_x: tuple[float, float]
    endstop_pedestal_head_y: float
    endstop_switch_front_y: float
    endstop_switch_nut_y: float
    post_front_z: float
    post_rear_z: float
    tensioner_base_front_z: float
    felt_stud_start_z: float
    felt_contact_z: float
    dancer_arm_rear_z: float
    dancer_arm_front_z: float
    dancer_pulley_rear_z: float
    dancer_pulley_front_z: float


@dataclass(frozen=True)
class HardwareOccurrence:
    """One labeled, buildable occurrence plus its mating-axis evidence."""

    link: str
    label: str
    schedule_id: str
    origin: Vec3
    axis: str | None
    mate_id: str
    mate_center: Vec3
    plausible: bool = True
    issue: str = ""
    bracket_frame: BracketFrame | None = None

    @property
    def axis_vector(self) -> Vec3 | None:
        return None if self.axis is None else AXIS_VECTOR[self.axis]

    def build(self) -> Part:
        """Instantiate and place the catalog solid with its occurrence label."""
        part = hardware.make_scheduled_part(self.schedule_id, label=self.label)
        if self.bracket_frame is not None:
            result = self.bracket_frame.location * part
            result.label = self.label
            return result
        if self.axis is None:
            raise ValueError(f"{self.label}: non-bracket occurrence lacks axis")
        return hardware.place(part, self.origin, axis=self.axis,
                              label=self.label)


@dataclass(frozen=True)
class PlacementIssue:
    code: str
    mate_id: str
    affected_labels: tuple[str, ...]
    reason: str


@dataclass(frozen=True)
class SharedGeometryPatch:
    """Exact source-level edit required outside this isolated module."""

    feature: str
    source: str
    current: str
    required: str
    selected_hardware: str


PRINTED_GEOMETRY_PATCHES: tuple[SharedGeometryPatch, ...] = (
    SharedGeometryPatch(
        feature="endstop_pedestal_side_ears",
        source="cad/printed.py:endstop_mount",
        current=(
            "M5 centers x=-6,+6 z=166; bore y=-233..-213 stops inside the "
            "x=+/-13.5 pedestal and has no accessible head face"
        ),
        required=(
            "union foot box x=-25..25, y=-225..-217, z=157..175; move both "
            "OD5.4 through bores to x=-19,+19, z=166, y=-233..-216"
        ),
        selected_hardware="2x ISO 4762 M5x12 + 2x HNTA5-5 slot nuts",
    ),
    SharedGeometryPatch(
        feature="endstop_switch_rear_nut_pockets",
        source="cad/printed.py:endstop_mount",
        current=(
            "switch pocket rear is y=-203.2; solid pedestal continues to "
            "y=-225, leaving no rear access for the scheduled nylocs"
        ),
        required=(
            "at x=-3.25,+3.25 z=160.35 add OD6 contact bosses from "
            "y=-203.2..-202.02 and OD8 rear-access counterbores from "
            "y=-226..-207.0; retain OD2.2 through bores y=-226..-193"
        ),
        selected_hardware=(
            "2x ISO 4762 M2x16 + 2x ISO 7089 M2 washers + 2x ISO 10511 M2 nylocs"
        ),
    ),
    SharedGeometryPatch(
        feature="felt_captive_jam_nut",
        source="cad/printed.py:felt_tensioner",
        current=(
            "OD5.4 stud seat z=-165..-157 is blind; no M4 nut pocket and "
            "the 35 mm stud starts at z=-164"
        ),
        required=(
            "make OD4.5 stud bore continuous z=-171..-157; add rear-opening "
            "M4 hex trap AF7.2 x 3.4 deep at z=-170..-166.6; trap is captive "
            "against rear-post face and stud starts z=-170"
        ),
        selected_hardware="DIN 976 M4x45 stud + ISO 4032 M4 nut",
    ),
)


def required_geometry_patches() -> tuple[SharedGeometryPatch, ...]:
    """Return the exact shared-source patch table for root integration."""
    return PRINTED_GEOMETRY_PATCHES


def _frame_brackets(params) -> tuple[BracketFrame, ...]:
    """Collision-free 15-HBKT layout; a printed shoe is the 16th joint."""
    frames: list[BracketFrame] = []

    # Horizontal members sit on the three lower cross rails.  One bracket on
    # the inward side of each longitudinal member avoids the outside envelope.
    horizontal = (
        ("base_L", -params.base_rail_x),
        ("base_R", params.base_rail_x),
    )
    for cross_name, cross_z in (
        ("rear", -180.0), ("mid", -50.0), ("front", 160.0),
    ):
        for member_name, member_x in horizontal:
            left = member_x < 0
            side_x = member_x + (10.0 if left else -10.0)
            frame = BracketFrame(
                label=f"frame_bracket_{cross_name}_{member_name}",
                joint=f"frame:{cross_name}:{member_name}",
                origin=(side_x, params.base_bot_y + params.extrusion,
                        cross_z),
                x_dir=(0.0, 0.0, 1.0 if left else -1.0),
                y_dir=(1.0 if left else -1.0, 0.0, 0.0),
                z_dir=(0.0, 1.0, 0.0),
            )
            # Five formerly inward/top-mounted brackets intersected a
            # stringer, rear post, or adjacent bracket.  Seat them on the
            # upper member underside and cross front face instead.
            if not (cross_name == "rear" and member_name == "base_R"):
                frame = BracketFrame(
                    label=frame.label, joint=frame.joint,
                    origin=(member_x, params.base_top_y, cross_z + 10.0),
                    x_dir=(1.0, 0.0, 0.0),
                    y_dir=(0.0, 0.0, 1.0),
                    z_dir=(0.0, -1.0, 0.0),
                )
            frames.append(frame)

    for cross_name, cross_z in (("mid", -50.0), ("front", 160.0)):
        for member_name, member_x in (
            ("stringer_L", -params.rail_x),
            ("stringer_R", params.rail_x),
        ):
            left = member_x < 0
            side_x = member_x + (10.0 if left else -10.0)
            frame = BracketFrame(
                label=f"frame_bracket_{cross_name}_{member_name}",
                joint=f"frame:{cross_name}:{member_name}",
                origin=(side_x, params.base_bot_y + params.extrusion,
                        cross_z),
                x_dir=(0.0, 0.0, 1.0 if left else -1.0),
                y_dir=(1.0 if left else -1.0, 0.0, 0.0),
                z_dir=(0.0, 1.0, 0.0),
            )
            if cross_name == "front":
                frame = BracketFrame(
                    label=frame.label, joint=frame.joint,
                    origin=(member_x, params.base_top_y, cross_z + 10.0),
                    x_dir=(1.0, 0.0, 0.0),
                    y_dir=(0.0, 0.0, 1.0),
                    z_dir=(0.0, -1.0, 0.0),
                )
            frames.append(frame)

    # Two braces per vertical post.  The front posts brace fore/aft along the
    # base rails; the rear tensioner post braces left/right along its cross.
    for side_name, post_x in (("L", -params.post_x), ("R", params.post_x)):
        post_z = sum(params.post_z) / 2.0
        for face_name, y_dir, x_dir in (
            ("front", (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
            ("rear", (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
        ):
            frames.append(BracketFrame(
                label=f"frame_bracket_post_{side_name}_{face_name}",
                joint=f"frame:post_{side_name}:{face_name}",
                origin=(post_x, params.stringer_top_y,
                        post_z + (10.0 if face_name == "front" else -10.0)),
                x_dir=x_dir, y_dir=y_dir, z_dir=(0.0, 1.0, 0.0),
            ))

    # The left 25 mm HBKT cannot fit the 15 mm base-rail corridor.  The
    # exact printed rear_post_left_shoe() replaces it; keep the clear right.
    for face_name, y_dir, x_dir in (
        ("right", (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ):
        frames.append(BracketFrame(
            label=f"frame_bracket_rear_post_{face_name}",
            joint=f"frame:rear_post:{face_name}",
            origin=(params.rear_post_x + (10.0 if face_name == "right" else -10.0),
                    params.base_top_y, params.rear_post_z),
            x_dir=x_dir, y_dir=y_dir, z_dir=(0.0, 1.0, 0.0),
        ))

    if len(frames) != 15:
        raise AssertionError(f"frame layout must produce 15 HBKTs, got {len(frames)}")
    return tuple(frames)


def current_geometry(params=DEFAULT_PARAMS) -> PlacementGeometry:
    """Return the corrected integration contract.

    Five values depend on the exact shared-source edits enumerated by
    :func:`required_geometry_patches`; all other values match current source.
    """
    plate_top = params.plate_top_y
    plate_bottom = plate_top - params.plate_t
    return PlacementGeometry(
        frame_brackets=_frame_brackets(params),
        # HIWIN MGN12R-150: six holes at 25 mm pitch. The selected cut uses
        # E1=10 mm and E2=15 mm, both within HIWIN's 5..20 mm limits.
        rail_hole_z=tuple(params.rail_z0 + 10.0 + 25.0 * i for i in range(6)),
        # Counterbore depth is 4.5 mm from the 8 mm rail top.
        rail_counterbore_y=params.stringer_top_y + params.rail_h - 4.5,
        rail_slot_surface_y=params.stringer_top_y,
        plate_top_y=plate_top,
        plate_bottom_y=plate_bottom,
        tower_top_y=plate_top + 6.0,
        nut_bracket_top_y=plate_top + 8.0,
        t8_flange_rear_z=params.m0_home_standoff - 18.0 - 3.8,
        t8_bracket_front_z=params.m0_home_standoff - 10.0,
        m0_mount_top_y=params.stringer_top_y + 8.0,
        m0_support_top_y=params.stringer_top_y + 8.0,
        endstop_pedestal_hole_x=(-19.0, 19.0),
        endstop_pedestal_head_y=-217.0,
        endstop_switch_front_y=-196.0,
        endstop_switch_nut_y=-207.0,
        post_front_z=params.post_z[1],
        post_rear_z=params.post_z[0],
        tensioner_base_front_z=params.rear_post_z + 16.0,
        felt_stud_start_z=params.rear_post_z + 10.0,
        felt_contact_z=-157.0,
        dancer_arm_rear_z=params.rear_post_z + 17.0,
        dancer_arm_front_z=params.rear_post_z + 19.5,
        dancer_pulley_rear_z=-160.0,
        dancer_pulley_front_z=-150.0,
    )


def _resolve_geometry(params, geometry: PlacementGeometry | Mapping[str, object] | None) -> PlacementGeometry:
    if geometry is None:
        return current_geometry(params)
    if isinstance(geometry, PlacementGeometry):
        return geometry
    if isinstance(geometry, Mapping):
        values = current_geometry(params).__dict__ | dict(geometry)
        return PlacementGeometry(**values)
    raise TypeError("geometry must be PlacementGeometry, mapping, or None")


def _occ(
    link: str,
    label: str,
    schedule_id: str,
    origin: Sequence[float],
    axis: str | None,
    mate_id: str,
    mate_center: Sequence[float],
    *,
    plausible: bool = True,
    issue: str = "",
    bracket_frame: BracketFrame | None = None,
) -> HardwareOccurrence:
    return HardwareOccurrence(
        link=link, label=label, schedule_id=schedule_id,
        origin=tuple(map(float, origin)), axis=axis, mate_id=mate_id,
        mate_center=tuple(map(float, mate_center)), plausible=plausible,
        issue=issue, bracket_frame=bracket_frame,
    )


def _add_stack(
    result: list[HardwareOccurrence],
    *,
    link: str,
    mate_id: str,
    center: Vec3,
    rows: Iterable[tuple[str, str, Vec3, str]],
    plausible: bool = True,
    issue: str = "",
) -> None:
    for label, schedule_id, origin, axis in rows:
        result.append(_occ(
            link, label, schedule_id, origin, axis, mate_id, center,
            plausible=plausible, issue=issue,
        ))


def _frame_occurrences(geometry: PlacementGeometry) -> list[HardwareOccurrence]:
    result: list[HardwareOccurrence] = []
    for frame in geometry.frame_brackets:
        result.append(_occ(
            "static", frame.label, "frame_brackets", frame.origin, None,
            frame.joint, frame.origin, bracket_frame=frame,
        ))

        floor_center = frame.point(0.0, 12.0, 0.0)
        floor_head = frame.point(0.0, 12.0, 5.0)
        floor_axis = _axis_name(frame.z_dir)
        floor_mate = f"{frame.joint}:floor"
        _add_stack(result, link="static", mate_id=floor_mate,
                   center=floor_center, rows=(
            (f"{frame.label}_floor_m5x10", "frame_bracket_screws",
             floor_head, floor_axis),
            (f"{frame.label}_floor_tnut", "frame_bracket_tnuts",
             floor_center, _opposite(floor_axis)),
        ))

        upright_center = frame.point(0.0, 0.0, 12.0)
        upright_head = frame.point(0.0, 5.0, 12.0)
        upright_axis = _axis_name(frame.y_dir)
        upright_mate = f"{frame.joint}:upright"
        _add_stack(result, link="static", mate_id=upright_mate,
                   center=upright_center, rows=(
            (f"{frame.label}_upright_m5x10", "frame_bracket_screws",
             upright_head, upright_axis),
            (f"{frame.label}_upright_tnut", "frame_bracket_tnuts",
             upright_center, _opposite(upright_axis)),
        ))
    return result


def static_occurrences(
    params=DEFAULT_PARAMS,
    geometry: PlacementGeometry | Mapping[str, object] | None = None,
) -> list[HardwareOccurrence]:
    """Return all fixed-link hardware at the reference pose."""
    g = _resolve_geometry(params, geometry)
    result = _frame_occurrences(g)

    # Custom printed left shoe supplies the second rear-post anchor in the
    # 15 mm corridor where an HBKTST5 cannot fit.
    _add_stack(result, link="static", mate_id="frame:rear_post:left_shoe:floor",
               center=(-63.0, -225.0, -180.0), rows=(
        ("rear_post_left_shoe_floor_m5x12", "rear_post_shoe_screws",
         (-63.0, -219.0, -180.0), "+y"),
        ("rear_post_left_shoe_floor_tnut", "rear_post_shoe_tnuts",
         (-63.0, -225.0, -180.0), "-y"),
    ))
    _add_stack(result, link="static",
               mate_id="frame:rear_post:left_shoe:upright",
               center=(-55.0, -208.0, -180.0), rows=(
        ("rear_post_left_shoe_upright_m5x12", "rear_post_shoe_screws",
         (-61.0, -208.0, -180.0), "-x"),
        ("rear_post_left_shoe_upright_tnut", "rear_post_shoe_tnuts",
         (-55.0, -208.0, -180.0), "+x"),
    ))

    # Four controlled 35 mm support stacks mount to underside slots away from
    # every cross bracket: 18 mm Würth female/female standoff, 17 mm Elesa
    # rubber foot, and a bonded M5x12 set screw with 4.8 mm rail-side
    # projection. The HNTA5 body is shifted 1.5 mm into the real slot cavity.
    for index, (x, z) in enumerate(((-params.base_rail_x, -120.0),
                                    (params.base_rail_x, -120.0),
                                    (-params.base_rail_x, 120.0),
                                    (params.base_rail_x, 120.0))):
        mate = f"frame:foot:{index}"
        center = (x, params.base_top_y, z)
        standoff_bottom_y = params.base_top_y - params.foot_standoff_h
        set_screw_top_y = (
            params.base_top_y + params.foot_set_screw_projection
        )
        tnut_y = params.base_top_y + params.foot_tnut_slot_depth
        _add_stack(result, link="static", mate_id=mate, center=center,
                   rows=(
            (f"foot_{index}", "machine_feet",
             (x, standoff_bottom_y, z), "+y"),
            (f"foot_{index}_standoff", "machine_foot_standoffs",
             center, "-y"),
            (f"foot_{index}_set_screw", "machine_foot_set_screws",
             (x, set_screw_top_y, z), "+y"),
            (f"foot_{index}_tnut", "machine_foot_tnuts",
             (x, tnut_y, z), "+y"),
        ))

    # MGN12 rail screws and slot nuts.
    for side, x in (("L", -params.rail_x), ("R", params.rail_x)):
        for index, z in enumerate(g.rail_hole_z, start=1):
            mate = f"m0:rail_{side}:hole_{index}"
            center = (x, g.rail_slot_surface_y, z)
            _add_stack(result, link="static", mate_id=mate, center=center,
                       rows=(
                (f"rail_{side}_m3x8_{index}", "rail_screws",
                 (x, g.rail_counterbore_y, z), "+y"),
                (f"rail_{side}_tnut_m3_{index}", "rail_tnuts",
                 center, "-y"),
            ))

    # M0 motor mount and fixed-end support feet, both on the left stringer.
    for group, schedule_screw, schedule_nut, top_y, centers in (
        ("m0_mount", "m0_mount_screws", "m0_mount_tnuts", g.m0_mount_top_y,
         ((-42.0, params.m0_motor_z + 12.0),
          (-42.0, params.m0_motor_z + 30.0))),
        ("m0_support", "m0_support_screws", "m0_support_tnuts",
         g.m0_support_top_y, ((-45.0, 132.0), (-45.0, 144.0))),
    ):
        for index, (x, z) in enumerate(centers, start=1):
            mate = f"m0:{group}:hole_{index}"
            center = (x, params.stringer_top_y, z)
            _add_stack(result, link="static", mate_id=mate, center=center,
                       rows=(
                (f"{group}_m5x12_{index}", schedule_screw,
                 (x, top_y, z), "+y"),
                (f"{group}_tnut_{index}", schedule_nut,
                 center, "-y"),
            ))

    # M0 NEMA17 screws: head on the front of the 6 mm plate, shank +Z.
    hg = 31.0 / 2.0
    for ix, dx in enumerate((-hg, hg)):
        for iy, dy in enumerate((-hg, hg)):
            x, y = params.screw_x + dx, params.screw_y + dy
            mate = f"motor:m0:{ix}:{iy}"
            center = (x, y, params.m0_motor_z)
            result.append(_occ(
                "static", f"m0_motor_m3x10_{ix}_{iy}", "motor_screws",
                (x, y, params.m0_motor_z - 6.0), "-z", mate, center,
            ))

    # Corrected side-foot ears: 8 mm printed grip plus 4 mm slot engagement.
    for index, x in enumerate(g.endstop_pedestal_hole_x, start=1):
        mate = f"m0:endstop_pedestal:hole_{index}"
        center = (x, params.base_top_y, 166.0)
        _add_stack(result, link="static", mate_id=mate, center=center,
                   rows=(
            (f"endstop_pedestal_m5x12_{index}", "endstop_pedestal_screws",
             (x, g.endstop_pedestal_head_y, 166.0), "+y"),
            (f"endstop_pedestal_tnut_{index}", "endstop_pedestal_tnuts",
             center, "-y"),
        ))

    # Omron D2F switch stack after the rear contact bosses and nut-access pockets
    # in required_geometry_patches().  The front washer sits under the head;
    # the M2 nyloc bears on the y=-207 counterbore shoulder.
    for index, x in enumerate(params.endstop_switch_hole_x, start=1):
        mate = f"m0:endstop_switch:hole_{index}"
        center = (x, -199.0, params.endstop_switch_hole_z)
        _add_stack(result, link="static", mate_id=mate, center=center,
                   rows=(
            (f"endstop_switch_m2x16_{index}", "endstop_switch_screws",
             (x, g.endstop_switch_front_y + 0.35,
              params.endstop_switch_hole_z), "+y"),
            (f"endstop_switch_washer_{index}", "endstop_switch_washers",
             (x, g.endstop_switch_front_y,
              params.endstop_switch_hole_z), "+y"),
            (f"endstop_switch_nyloc_{index}", "endstop_switch_nylocs",
             (x, g.endstop_switch_nut_y,
              params.endstop_switch_hole_z), "-y"),
        ))

    # Flyer bearing block to the post front slots.
    for sx in (-1, 1):
        side = "L" if sx < 0 else "R"
        for dy in (-12.0, 12.0):
            y = dy
            suffix = "low" if dy < 0 else "high"
            mate = f"m2:flyer_block:{side}:{suffix}"
            center = (sx * params.post_x, y, g.post_front_z)
            _add_stack(result, link="static", mate_id=mate, center=center,
                       rows=(
                (f"flyer_block_{side}_{suffix}_m5x16",
                 "flyer_block_screws",
                 (sx * params.post_x, y, g.post_front_z + 9.0), "+z"),
                (f"flyer_block_{side}_{suffix}_tnut",
                 "flyer_block_tnuts", center, "-z"),
            ))

    # M2 motor mount to post rear slots.
    for sx in (-1, 1):
        side = "L" if sx < 0 else "R"
        for y in (params.m2_motor_axis_y - 12.0, 8.0):
            suffix = "low" if y < 0 else "high"
            mate = f"m2:motor_mount:{side}:{suffix}"
            center = (sx * params.post_x, y, g.post_rear_z)
            _add_stack(result, link="static", mate_id=mate, center=center,
                       rows=(
                (f"m2_mount_{side}_{suffix}_m5x12", "m2_mount_screws",
                 (sx * params.post_x, y, g.post_rear_z - 6.0), "-z"),
                (f"m2_mount_{side}_{suffix}_tnut", "m2_mount_tnuts",
                 center, "+z"),
            ))

    # McMaster 6627T421 M2 motor screws through the 6 mm tensioning plate.
    for ix, dx in enumerate((-hg, hg)):
        for iy, dy in enumerate((-hg, hg)):
            x, y = dx, params.m2_motor_axis_y + dy
            mate = f"motor:m2:{ix}:{iy}"
            center = (x, y, params.m2_motor_face_z)
            result.append(_occ(
                "static", f"m2_motor_m3x10_{ix}_{iy}", "motor_screws",
                (x, y, params.m2_motor_face_z + 6.0), "+z", mate, center,
            ))

    # Four rear-post bases, two M5 slot fasteners each.
    base_patterns = (
        ("spool", params.spool_y, (-18.0, 18.0), "m3_base_tnuts"),
        ("felt", params.felt_y, (-9.0, 9.0), "m3_base_tnuts"),
        ("dancer", params.dancer_y, params.dancer_base_mount_offsets,
         "dancer_base_tnuts"),
        ("entry", 0.0, (-8.0, 8.0), "m3_base_tnuts"),
    )
    for base_name, center_y, offsets, tnut_schedule in base_patterns:
        for index, offset in enumerate(offsets, start=1):
            y = center_y + offset
            mate = f"m3:{base_name}_base:hole_{index}"
            center = (params.rear_post_x, y, params.rear_post_z + 10.0)
            _add_stack(result, link="static", mate_id=mate, center=center,
                       rows=(
                (f"{base_name}_base_m5x12_{index}", "m3_base_screws",
                 (params.rear_post_x, y, g.tensioner_base_front_z), "+z"),
                (f"{base_name}_base_tnut_{index}", tnut_schedule,
                 center, "-z"),
            ))

    # Spool axle, washers and nyloc, axis along X.
    spool_center = (
        params.rear_post_x, params.spool_y, params.rear_post_z + 60.0,
    )
    left_outer = params.rear_post_x - 27.0
    right_outer = params.rear_post_x + 27.0
    m8_washer_t = 1.8
    _add_stack(result, link="static", mate_id="m3:spool_axle",
               center=spool_center, rows=(
        ("spool_axle_m8x75", "spool_axle",
         (left_outer - m8_washer_t, spool_center[1], spool_center[2]), "-x"),
        ("spool_axle_washer_left", "spool_axle_washers",
         (left_outer - m8_washer_t, spool_center[1], spool_center[2]), "+x"),
        ("spool_axle_washer_right", "spool_axle_washers",
         (right_outer, spool_center[1], spool_center[2]), "+x"),
        ("spool_axle_nyloc", "spool_axle_nyloc",
         (right_outer + m8_washer_t, spool_center[1], spool_center[2]), "+x"),
    ))

    # Felt stud stack.  The corrected rear M4 nut trap is captive against the
    # rear post and the longer stud preserves the existing front stack.
    felt_center = (params.rear_post_x, params.felt_y, g.felt_contact_z)
    result.append(_occ(
        "static", "felt_m4x55_stud", "felt_stud",
        (params.rear_post_x, params.felt_y, g.felt_stud_start_z), "+z",
        "m3:felt_stud", felt_center,
    ))
    nut_angle = math.radians(30.0)
    nut_frame = BracketFrame(
        label="felt_m4_jam_nut_frame", joint="m3:felt_stud",
        origin=(params.rear_post_x, params.felt_y, g.felt_stud_start_z),
        x_dir=(math.cos(nut_angle), math.sin(nut_angle), 0.0),
        y_dir=(-math.sin(nut_angle), math.cos(nut_angle), 0.0),
        z_dir=(0.0, 0.0, 1.0),
    )
    result.append(_occ(
        "static", "felt_m4_jam_nut", "felt_jam_nut",
        nut_frame.origin, None, "m3:felt_stud", felt_center,
        bracket_frame=nut_frame,
    ))
    felt_rows = (
        ("felt_backing_fixed", "felt_backing_washers",
         (params.rear_post_x, params.felt_y, -161.25), "+z"),
        ("felt_pad_fixed", "felt_pads",
         (params.rear_post_x, params.felt_y, -160.25), "+z"),
        ("felt_pad_moving", "felt_pads",
         (params.rear_post_x, params.felt_y, -156.75), "+z"),
        ("felt_backing_moving", "felt_backing_washers",
         (params.rear_post_x, params.felt_y, -153.75), "+z"),
        ("felt_compression_spring", "felt_compression_spring",
         (params.rear_post_x, params.felt_y, -152.75), "+z"),
        ("felt_spring_thrust_washer", "felt_spring_thrust_washer",
         (params.rear_post_x, params.felt_y, -131.6867039296), "+z"),
        ("felt_m4_wingnut", "felt_wingnut",
         (params.rear_post_x, params.felt_y, -130.5867039296), "+z"),
    )
    _add_stack(result, link="static", mate_id="m3:felt_stud",
               center=felt_center, rows=felt_rows)

    # Dancer pivot OD5 shoulder: head/front shim, 10 mm shoulder through arm
    # and base, M4 thread and nyloc behind the base.
    pivot_center = (params.rear_post_x, params.dancer_y, -165.0)
    _add_stack(result, link="static", mate_id="m3:dancer_pivot",
               center=pivot_center, rows=(
        ("dancer_pivot_shim_front", "dancer_pivot_shims",
         (pivot_center[0], pivot_center[1], g.dancer_arm_front_z), "+z"),
        ("dancer_pivot_shoulder_m4", "dancer_pivot_shoulder",
         (pivot_center[0], pivot_center[1], g.dancer_arm_front_z + 0.5), "+z"),
        ("dancer_pivot_shim_rear", "dancer_pivot_shims",
         (pivot_center[0], pivot_center[1], -164.0), "+z"),
        ("dancer_pivot_tnut_m4", "dancer_pivot_tnut",
         (pivot_center[0], pivot_center[1], -170.0), "-z"),
    ))

    # Dancer pulley OD3 shoulder through front shim, pulley/623, rear shim and
    # arm boss; the M2.5 nyloc sits behind the arm.
    pulley_center = (params.dancer_pulley_x, params.dancer_pulley_y, -155.0)
    # Five additional 0.5 mm DIN 988 shims behind the arm fill the former
    # 2.5 mm overlong shoulder.  The nyloc now starts exactly at the M2.5
    # thread transition rather than bottoming on OD3.
    pulley_rows = [
        ("dancer_pulley_shim_front", "dancer_pulley_shims",
         (pulley_center[0], pulley_center[1], g.dancer_pulley_front_z), "+z"),
        ("dancer_pulley_shoulder_m2p5", "dancer_pulley_shoulder",
         (pulley_center[0], pulley_center[1], g.dancer_pulley_front_z + 0.5), "+z"),
        ("dancer_pulley_shim_rear", "dancer_pulley_shims",
         (pulley_center[0], pulley_center[1], g.dancer_pulley_rear_z - 0.5), "+z"),
    ]
    for index in range(5):
        pulley_rows.append((
            f"dancer_pulley_shim_arm_rear_{index + 1}",
            "dancer_pulley_shims",
            (pulley_center[0], pulley_center[1],
             g.dancer_arm_rear_z - 0.5 * index),
            "-z",
        ))
    pulley_rows.append((
        "dancer_pulley_nyloc_m2p5", "dancer_pulley_nyloc",
        (pulley_center[0], pulley_center[1], g.dancer_arm_rear_z - 2.5),
        "-z",
    ))
    _add_stack(result, link="static", mate_id="m3:dancer_pulley",
               center=pulley_center, rows=pulley_rows)

    # Two hard stops.  Printed Ø9 support bosses stay behind the moving arm;
    # only the OD5 steel sleeve spans z=-164..-160 in the arm plane.
    for index, (x, y) in enumerate(params.dancer_stop_centers, start=1):
        mate = f"m3:dancer_stop:{index}"
        _add_stack(result, link="static", mate_id=mate,
                   center=(x, y, -162.0), rows=(
            (f"dancer_stop_{index}_m3x10", "dancer_stop_screws",
             (x, y, -159.5), "+z"),
            (f"dancer_stop_{index}_od5_sleeve", "dancer_stop_sleeves",
             (x, y, -164.0), "+z"),
            (f"dancer_stop_{index}_m3_washer", "dancer_stop_washers",
             (x, y, -160.0), "+z"),
            (f"dancer_stop_{index}_m3_insert", "dancer_stop_inserts",
             (x, y, -170.0), "+z"),
        ))

    # Spring anchor pins hold both loops in the audited z=-154.25 plane.
    fx, fy = params.dancer_spring_fixed_x, params.dancer_spring_fixed_y
    _add_stack(result, link="static", mate_id="m3:dancer_spring_fixed",
               center=(fx, fy, params.dancer_spring_plane_z), rows=(
        ("dancer_spring_fixed_m2x12", "dancer_fixed_anchor_screw",
         (fx, fy, -159.5), "-z"),
        ("dancer_spring_fixed_sleeve", "dancer_fixed_anchor_sleeve",
         (fx, fy, -156.5), "+z"),
        ("dancer_spring_fixed_washer_rear", "dancer_anchor_washers",
         (fx, fy, -155.0), "+z"),
        ("dancer_spring_fixed_washer_front", "dancer_anchor_washers",
         (fx, fy, -154.0), "+z"),
        ("dancer_spring_fixed_m2_nyloc", "dancer_anchor_nylocs",
         (fx, fy, -153.5), "+z"),
    ))

    dx = params.dancer_pulley_x - params.rear_post_x
    dy = params.dancer_pulley_y - params.dancer_y
    arm_length = math.hypot(dx, dy)
    mx = params.rear_post_x + params.dancer_spring_moving_r * dx / arm_length
    my = params.dancer_y + params.dancer_spring_moving_r * dy / arm_length
    _add_stack(result, link="static", mate_id="m3:dancer_spring_moving",
               center=(mx, my, params.dancer_spring_plane_z), rows=(
        ("dancer_spring_moving_m2x16_flush", "dancer_moving_anchor_screw",
         (mx, my, -163.0), "-z"),
        ("dancer_spring_moving_sleeve", "dancer_moving_anchor_sleeve",
         (mx, my, -160.5), "+z"),
        ("dancer_spring_moving_washer_rear", "dancer_anchor_washers",
         (mx, my, -156.5), "+z"),
        ("dancer_spring_moving_washer_front", "dancer_anchor_washers",
         (mx, my, -155.5), "+z"),
        ("dancer_spring_moving_m2_nyloc", "dancer_anchor_nylocs",
         (mx, my, -155.0), "+z"),
    ))

    spring_dx, spring_dy = mx - fx, my - fy
    spring_length = math.hypot(spring_dx, spring_dy)
    spring_z_dir = (spring_dx / spring_length, spring_dy / spring_length, 0.0)
    spring_x_dir = (0.0, 0.0, 1.0)
    spring_y_dir = (spring_z_dir[1], -spring_z_dir[0], 0.0)
    spring_frame = BracketFrame(
        label="dancer_extension_spring_frame",
        joint="m3:dancer_extension_spring",
        origin=(fx, fy, params.dancer_spring_plane_z),
        x_dir=spring_x_dir, y_dir=spring_y_dir, z_dir=spring_z_dir,
    )
    result.append(_occ(
        "static", "dancer_extension_spring", "dancer_extension_spring",
        spring_frame.origin, None, "m3:dancer_extension_spring",
        ((fx + mx) / 2.0, (fy + my) / 2.0,
         params.dancer_spring_plane_z), bracket_frame=spring_frame,
    ))
    return result


def carriage_occurrences(
    params=DEFAULT_PARAMS,
    geometry: PlacementGeometry | Mapping[str, object] | None = None,
) -> list[HardwareOccurrence]:
    """Return moving-carriage hardware at the HOME reference pose."""
    g = _resolve_geometry(params, geometry)
    result: list[HardwareOccurrence] = []
    zc = params.m0_home_standoff

    # MGN12H block screws on exact 20x20 tapped grids.
    for side, rail_x in (("L", -params.rail_x), ("R", params.rail_x)):
        for dx in (-10.0, 10.0):
            for dz in (-10.0, 10.0):
                x, z = rail_x + dx, zc + dz
                mate = f"m0:block_{side}:{dx:+g}:{dz:+g}"
                result.append(_occ(
                    "carriage", f"block_{side}_m3x10_{dx:+g}_{dz:+g}",
                    "block_screws", (x, g.plate_top_y, z), "+y", mate,
                    (x, g.plate_bottom_y, z),
                ))

    # Front tower row uses M4x20 through tower + 0.250 in plate.  The rear
    # row uses M4x25 because it also captures the 6 mm printable endstop flag.
    for dx in (-31.0, 31.0):
        for dz in (-31.0, 31.0):
            x, z = dx, zc + dz
            mate = f"m1:tower:{dx:+g}:{dz:+g}"
            screw_id = ("carriage_flag_m4_screws" if dz > 0
                        else "carriage_m4_screws")
            screw_length = "25" if dz > 0 else "20"
            bearing_y = (carriage_endstop_flag.FLAG_BOTTOM_Y
                         if dz > 0 else g.plate_bottom_y)
            _add_stack(result, link="carriage", mate_id=mate,
                       center=(x, bearing_y, z), rows=(
                (f"tower_m4x{screw_length}_{dx:+g}_{dz:+g}", screw_id,
                 (x, g.tower_top_y, z), "+y"),
                (f"tower_washer_m4_{dx:+g}_{dz:+g}",
                 "carriage_m4_washers", (x, bearing_y, z), "-y"),
                (f"tower_nyloc_m4_{dx:+g}_{dz:+g}",
                 "carriage_m4_nylocs", (x, bearing_y - 0.9, z), "-y"),
            ))

    # Nut-bracket uses M4x25: 14 mm printed grip + washer + full nyloc with
    # roughly 5 mm protrusion for inspection.
    for dz in (2.0, 12.0):
        x, z = -78.0, zc + dz
        mate = f"m0:nut_bracket:{dz:+g}"
        _add_stack(result, link="carriage", mate_id=mate,
                   center=(x, g.plate_bottom_y, z), rows=(
            (f"nut_bracket_m4x25_{dz:+g}", "nut_bracket_m4_screws",
             (x, g.nut_bracket_top_y, z), "+y"),
            (f"nut_bracket_washer_m4_{dz:+g}", "carriage_m4_washers",
             (x, g.plate_bottom_y, z), "-y"),
            (f"nut_bracket_nyloc_m4_{dz:+g}", "carriage_m4_nylocs",
             (x, g.plate_bottom_y - 0.9, z), "-y"),
        ))

    # The Zyltech drawing calls out four M3 threaded flange holes. Install
    # M3x12 screws from the bracket +Z face through one washer and into the
    # 4 mm Delrin flange; no rear head/nyloc may occupy the spring annulus.
    for index, angle in enumerate((45.0, 135.0, 225.0, 315.0), start=1):
        radial = _radial_point(params.screw_x, params.screw_y, 8.0, angle)
        x, y = radial[0], radial[1]
        mate = f"m0:t8_flange:{index}"
        center = (x, y, zc - 18.0)
        _add_stack(result, link="carriage", mate_id=mate, center=center,
                   rows=(
            (f"t8_flange_m3x12_{index}", "t8_nut_screws",
             (x, y, g.t8_bracket_front_z + 0.55), "+z"),
            (f"t8_flange_washer_m3_{index}", "t8_nut_washers",
             (x, y, g.t8_bracket_front_z), "+z"),
        ))

    # M1 motor screws from flange top downward into the motor face.
    hg = 31.0 / 2.0
    for ix, dx in enumerate((-hg, hg)):
        for iz, dz in enumerate((-hg, hg)):
            x, z = dx, zc + dz
            mate = f"motor:m1:{ix}:{iz}"
            result.append(_occ(
                "carriage", f"m1_motor_m3x10_{ix}_{iz}", "motor_screws",
                (x, g.tower_top_y, z), "+y", mate,
                (x, params.m1_motor_top_y, z),
            ))
    return result


def spindle_occurrences(
    params=DEFAULT_PARAMS,
    geometry: PlacementGeometry | Mapping[str, object] | None = None,
) -> list[HardwareOccurrence]:
    """Return spindle-link audit hardware.

    The current audit schedule assigns the spindle's rings, spacers, collar
    and coupling to existing COTS occurrences rather than the new fastener
    catalog, so this list is intentionally empty.  Accepting both parameters
    keeps the same integration signature as the other links.
    """
    _resolve_geometry(params, geometry)
    return []


def flyer_occurrences(
    params=DEFAULT_PARAMS,
    geometry: PlacementGeometry | Mapping[str, object] | None = None,
) -> list[HardwareOccurrence]:
    """Return flyer-link clamp and counterweight hardware."""
    _resolve_geometry(params, geometry)
    result: list[HardwareOccurrence] = []

    # Two radial collar set screws and their heat-set inserts.
    collar_z = sum(params.hub_z) / 2.0
    collar_specs = (
        ("neg_y", (0.0, -14.0, collar_z), "-y",
         (0.0, -14.0, collar_z), "+y"),
        ("pos_x", (14.0, 0.0, collar_z), "+x",
         (14.0, 0.0, collar_z), "-x"),
    )
    for name, screw_origin, screw_axis, insert_origin, insert_axis in collar_specs:
        mate = f"m2:flyer_arm_clamp:{name}"
        center = (0.0, 0.0, collar_z)
        _add_stack(result, link="flyer", mate_id=mate, center=center,
                   rows=(
            (f"flyer_arm_{name}_m3x8", "flyer_set_screws",
             screw_origin, screw_axis),
            (f"flyer_arm_{name}_m3_insert", "flyer_set_inserts",
             insert_origin, insert_axis),
        ))

    return result


def hardware_occurrences_by_link(
    params=DEFAULT_PARAMS,
    geometry: PlacementGeometry | Mapping[str, object] | None = None,
) -> dict[str, list[HardwareOccurrence]]:
    """Return labeled metadata occurrences for all four kinematic links."""
    g = _resolve_geometry(params, geometry)
    return {
        "static": static_occurrences(params, g),
        "carriage": carriage_occurrences(params, g),
        "spindle": spindle_occurrences(params, g),
        "flyer": flyer_occurrences(params, g),
    }


def hardware_parts_by_link(
    params=DEFAULT_PARAMS,
    geometry: PlacementGeometry | Mapping[str, object] | None = None,
    *,
    include_flagged: bool = True,
) -> dict[str, list[Part]]:
    """Build the labeled occurrence solids for direct assembly integration."""
    result: dict[str, list[Part]] = {}
    for link, occurrences in hardware_occurrences_by_link(params, geometry).items():
        selected = occurrences if include_flagged else [
            occurrence for occurrence in occurrences if occurrence.plausible
        ]
        result[link] = [occurrence.build() for occurrence in selected]
    return result


def occurrence_counts(
    params=DEFAULT_PARAMS,
    geometry: PlacementGeometry | Mapping[str, object] | None = None,
) -> Counter[str]:
    """Count assembly occurrences by hardware schedule ID."""
    links = hardware_occurrences_by_link(params, geometry)
    return Counter(
        occurrence.schedule_id
        for occurrences in links.values()
        for occurrence in occurrences
    )


def unmodeled_schedule_items(
    params=DEFAULT_PARAMS,
    geometry: PlacementGeometry | Mapping[str, object] | None = None,
) -> dict[str, int]:
    """Return scheduled quantities lacking an occurrence in this module."""
    counts = occurrence_counts(params, geometry)
    return {
        item["id"]: item["qty"] - counts[item["id"]]
        for item in hardware.HARDWARE_SCHEDULE
        if item.get("placement_authority", "hardware_placements")
        == "hardware_placements"
        if item["qty"] != counts[item["id"]]
    }


def placement_issues(
    params=DEFAULT_PARAMS,
    geometry: PlacementGeometry | Mapping[str, object] | None = None,
) -> list[PlacementIssue]:
    """Return de-duplicated mechanical issues, including unselected springs."""
    by_key: dict[tuple[str, str], list[str]] = {}
    issue_text: dict[tuple[str, str], str] = {}
    for occurrences in hardware_occurrences_by_link(params, geometry).values():
        for occurrence in occurrences:
            if occurrence.plausible:
                continue
            code = "unresolved_hardware_contract"
            key = (code, occurrence.mate_id)
            by_key.setdefault(key, []).append(occurrence.label)
            issue_text[key] = occurrence.issue

    result = [
        PlacementIssue(code, mate_id, tuple(sorted(labels)), issue_text[(code, mate_id)])
        for (code, mate_id), labels in sorted(by_key.items())
    ]
    return result


if __name__ == "__main__":
    links = hardware_occurrences_by_link()
    for link, occurrences in links.items():
        print(f"{link}: {len(occurrences)} occurrences")
    print("unmodeled:", unmodeled_schedule_items())
    for issue in placement_issues():
        print(f"ISSUE {issue.code} {issue.mate_id}: {issue.reason}")
