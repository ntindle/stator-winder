"""M0-following, M1-static active-sector terminal guide successor.

One front and one rear shared capture shoe follow the M0 carriage but do not
rotate with M1.  Raw tooth indexing brings every one of the 24 identical cap
sectors into this same machine-space guide.  Each fixed shoe feeds two open
R3.50 selection bowls; a short free radial handoff crosses a 2.50 mm nominal
rigid gap to 24-fold spindle-owned open R3.50 cap lead-ins.

All wire channels are externally accessible.  There are no hidden curved
bores, no new commanded axis and no deposition load path through M1 at R39.2.
"""

from __future__ import annotations

from functools import lru_cache
import math
from pathlib import Path

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
    Pos,
    Rectangle,
    Rot,
    Sphere,
    ThreePointArc,
    sweep,
)

import hardware
from params import DEFAULT_STATOR, PARAMS
import permanent_cap_production_review as cap
import printed
import shared_annular_terminal_crown as predecessor
import stator_model


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REVIEW = ROOT / "out" / "review"
STEP_OUT = REVIEW / "carriage_active_sector_terminal_guide.step"

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
SLOTS = 24
PITCH_DEG = 15.0
PEEK_DENSITY_G_MM3 = 1.30e-3
ALUMINUM_DENSITY_G_MM3 = 2.70e-3

# A 0.80 mm inboard shift from the first R40 layout restores a physical
# tolerance/deflection reserve to the repaired-flyer clearance.
CAPTURE_RADIUS_MM = 39.20
CAPTURE_POINT_AXIAL_MM = 21.0
FIXED_BOWL_X_MM = 29.70
FIXED_BOWL_AXIAL_MM = 21.35
FIXED_BOWL_SURFACE_RADIUS_MM = 3.50
FIXED_BOWL_OUTER_RADIUS_MM = 4.25
FIXED_RAIL_TANGENTIAL_MM = 3.35
FIXED_RAIL_WIDTH_MM = 0.60

HANDOFF_X_MM = 21.75
LEADIN_CENTERLINE_RADIUS_MM = 3.50
LEADIN_CLEAR_RADIUS_MM = 0.45
# Outer positives are fused before every intended groove/access negative is
# subtracted globally.  This retains the R1.20 structural shell and a real
# separator web without allowing one adjacent positive to refill the other's
# R0.45 wire groove.
LEADIN_OUTER_RADIUS_MM = 1.20
LEADIN_OPENING_WIDTH_MM = 0.75
RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM = 2.0 * LEADIN_OUTER_RADIUS_MM
RIGHT_SEAM_MOUTH_TANGENTIAL_WIDTH_MM = LEADIN_OPENING_WIDTH_MM
RIGHT_SEAM_MOUTH_AXIAL_SPAN_MM = 0.90
RIGHT_SEAM_MOUTH_CAP_SIDE_OVERLAP_MM = (
    RIGHT_SEAM_MOUTH_AXIAL_SPAN_MM / 2.0
)
LEADIN_BEND_X_MM = cap._lane_points()["start"][0] + LEADIN_CENTERLINE_RADIUS_MM
LEADIN_HIGH_AXIAL_MM = 21.35
LEADIN_BEND_CENTER_AXIAL_MM = (
    LEADIN_HIGH_AXIAL_MM - LEADIN_CENTERLINE_RADIUS_MM
)
# ``PORT_*`` remains the fixed-bowl handoff and the unchanged left cap-port
# contract.  The production cap does not have a matching right riser_top: its
# exact C1 centerline passes through ``waypoint`` before descending to the
# low right slot endpoint.  The right short lead-in therefore starts at that
# existing high waypoint and uses a two-arc S-bend to recover the unchanged
# fixed-bowl handoff and +X handoff tangent.
PORT_X_MM = 18.20
PORT_TANGENTIAL_MM = 2.05
LEFT_CAP_ENDPOINT_NAME = "riser_top"
RIGHT_CAP_ENDPOINT_NAME = "waypoint"
_CAP_LANE_POINTS = cap._lane_points()
PORT_AXIAL_MM = float(_CAP_LANE_POINTS[LEFT_CAP_ENDPOINT_NAME][2])
RIGHT_CAP_ENDPOINT_X_MM = float(
    _CAP_LANE_POINTS[RIGHT_CAP_ENDPOINT_NAME][0]
)
RIGHT_CAP_ENDPOINT_TANGENTIAL_MM = float(
    _CAP_LANE_POINTS[RIGHT_CAP_ENDPOINT_NAME][1]
)
RIGHT_CAP_ENDPOINT_AXIAL_MM = float(
    _CAP_LANE_POINTS[RIGHT_CAP_ENDPOINT_NAME][2]
)
if not math.isclose(
    RIGHT_CAP_ENDPOINT_AXIAL_MM, PORT_AXIAL_MM, abs_tol=1.0e-12,
):
    raise RuntimeError("right cap waypoint axial height drift")

RIGHT_QUARTER_END_X_MM = (
    RIGHT_CAP_ENDPOINT_X_MM + LEADIN_CENTERLINE_RADIUS_MM
)
RIGHT_S_BEND_X_RUN_MM = HANDOFF_X_MM - RIGHT_QUARTER_END_X_MM
RIGHT_S_BEND_TANGENTIAL_SHIFT_MM = (
    PORT_TANGENTIAL_MM - RIGHT_CAP_ENDPOINT_TANGENTIAL_MM
)
if RIGHT_S_BEND_X_RUN_MM <= 0.0:
    raise RuntimeError("right lead-in S-bend requires positive X run")
if RIGHT_S_BEND_TANGENTIAL_SHIFT_MM <= 0.0:
    raise RuntimeError("right lead-in S-bend requires positive tangential shift")
RIGHT_S_BEND_SWEEP_RAD = 2.0 * math.atan2(
    RIGHT_S_BEND_TANGENTIAL_SHIFT_MM,
    RIGHT_S_BEND_X_RUN_MM,
)
if not (0.0 < RIGHT_S_BEND_SWEEP_RAD < math.pi / 2.0):
    raise RuntimeError("right lead-in S-bend sweep must be in (0, 90deg)")
RIGHT_S_BEND_SWEEP_DEG = math.degrees(RIGHT_S_BEND_SWEEP_RAD)
RIGHT_S_BEND_RADIUS_MM = RIGHT_S_BEND_X_RUN_MM / (
    2.0 * math.sin(RIGHT_S_BEND_SWEEP_RAD)
)
if RIGHT_S_BEND_RADIUS_MM < LEADIN_CENTERLINE_RADIUS_MM:
    raise RuntimeError("right lead-in S-bend fell below the R3.50 contract")
_RIGHT_RECONSTRUCTED_HANDOFF_X_MM = (
    RIGHT_QUARTER_END_X_MM
    + 2.0 * RIGHT_S_BEND_RADIUS_MM
    * math.sin(RIGHT_S_BEND_SWEEP_RAD)
)
_RIGHT_RECONSTRUCTED_HANDOFF_TANGENTIAL_MM = (
    RIGHT_CAP_ENDPOINT_TANGENTIAL_MM
    + 2.0 * RIGHT_S_BEND_RADIUS_MM
    * (1.0 - math.cos(RIGHT_S_BEND_SWEEP_RAD))
)
if not (
    math.isclose(
        _RIGHT_RECONSTRUCTED_HANDOFF_X_MM,
        HANDOFF_X_MM,
        abs_tol=1.0e-12,
    )
    and math.isclose(
        _RIGHT_RECONSTRUCTED_HANDOFF_TANGENTIAL_MM,
        PORT_TANGENTIAL_MM,
        abs_tol=1.0e-12,
    )
):
    raise RuntimeError("right lead-in S-bend does not reconstruct handoff")

ROTATING_LEADIN_OUTER_ENVELOPE_MM = HANDOFF_X_MM + LEADIN_OUTER_RADIUS_MM
FIXED_GUIDE_INNER_ENVELOPE_MM = FIXED_BOWL_X_MM - FIXED_BOWL_OUTER_RADIUS_MM
ARBITRARY_M1_RADIAL_CLEARANCE_MM = (
    FIXED_GUIDE_INNER_ENVELOPE_MM - ROTATING_LEADIN_OUTER_ENVELOPE_MM
)

# Positive M0-carriage bracket.  Two side rails stay outside the full 6 mm
# wire mouth.  Their inboard faces mate to (but do not overlap) the PEEK pads;
# one M3 stack on each side of each guide clamps through the aluminum rail
# into a short heat-set insert in the pad.
GUIDE_PAD_CONTACT_TANGENTIAL_MM = 10.10
GUIDE_SEAT_TANGENTIAL_MM = 15.10
GUIDE_SEAT_OUTER_TANGENTIAL_MM = 20.10
# Keep the long forward-plane rails and their radial crossmembers outside the
# complete R26 conservative wound-coil body.  With the 10 mm member width,
# the inner corner is (x=6, |y|=29.5), giving 4.104 mm nominal radial
# clearance and 3.688 mm after the current 0.416 mm adverse tolerance sum.
YOKE_TANGENTIAL_MM = 34.50
YOKE_BAR_WIDTH_MM = 10.00
# The full-length rails sit in the forward plane, ahead of every rotating
# flyer occurrence at the deepest M0 pose.  The +axial guide cannot connect
# straight to that plane because the physical bell-to-shaft chord crosses the
# direct radial member.  Its short inboard dogleg therefore drops to local
# axial z=5 before turning forward; the rails stop at z=8.7, below the wire's
# z=12 shaft endpoint.  The -axial guide can connect directly at z=-21.35.
YOKE_RADIAL_X_MM = 13.00
YOKE_AXIAL_MIN_MM = -90.0
YOKE_FRONT_DOGLEG_AXIAL_MM = 5.0
YOKE_SEAT_MEMBER_AXIAL_MM = 7.4
YOKE_AXIAL_MAX_MM = (
    YOKE_FRONT_DOGLEG_AXIAL_MM + YOKE_SEAT_MEMBER_AXIAL_MM / 2.0
)
YOKE_AXIAL_SPAN_MM = YOKE_AXIAL_MAX_MM - YOKE_AXIAL_MIN_MM
YOKE_BAR_RADIAL_LENGTH_MM = 14.0
YOKE_FRONT_PLANE_X_MIN_MM = (
    YOKE_RADIAL_X_MM - YOKE_BAR_RADIAL_LENGTH_MM / 2.0
)
# The keyed seat itself retains the full x=33.4..40.4 pad.  Its structural
# dogleg is shifted 1.4 mm forward to x=32..39: this still overlaps the seat
# by 5.6 mm while opening the exact full-M2 flyer-screw witness above 4 mm.
YOKE_GUIDE_CONNECTOR_X_MAX_MM = 39.0
YOKE_GUIDE_DOGLEG_X_MM = YOKE_GUIDE_CONNECTOR_X_MAX_MM - 3.5
M3_HOLE_RADIUS_MM = 1.70
M3_BOLT_RADIAL_X_MM = 37.10
YOKE_GUIDE_SEAT_X_MIN_MM = 33.4
# The seat ends at the exact outer radial edge of the 5.50 mm PEEK pad.  The
# former extra 0.55 mm aluminum lip was not part of the clamp interface and
# unnecessarily governed full-M2 clearance to a flyer guide screw.
YOKE_GUIDE_SEAT_X_MAX_MM = M3_BOLT_RADIAL_X_MM + 5.50 / 2.0
M3_INSERT_PILOT_RADIUS_MM = 2.35
M3_INSERT_PILOT_DEPTH_MM = 5.50
M3_SHORT_INSERT_LENGTH_MM = 4.30
M3_WASHER_THICKNESS_MM = 0.55
M4_WASHER_THICKNESS_MM = 0.90
GUIDE_DATUM_KEY_RADIAL_MM = 1.20
GUIDE_DATUM_KEY_AXIAL_MM = 1.20
GUIDE_DATUM_KEY_DEPTH_MM = 0.80
GUIDE_DATUM_CLEARANCE_MM = 0.05
GUIDE_DATUM_X_MM = 34.70
GUIDE_DATUM_AXIAL_OFFSET_MM = 2.20
TOWER_DATUM_MACHINE_X_MM = (-10.0, 10.0)
TOWER_DATUM_MACHINE_Z_MM = 61.0
TOWER_DATUM_KEY_X_MM = 3.0
TOWER_DATUM_KEY_Z_MM = 2.0
TOWER_DATUM_KEY_DEPTH_MM = 1.5
TOWER_DATUM_CLEARANCE_MM = 0.05

TOWER_ADAPTER_LOCAL_X_MM = (26.0, 38.0)
TOWER_ADAPTER_LOCAL_Y_MM = (-28.0, 28.0)
TOWER_ADAPTER_LOCAL_Z_MM = (-114.0, -110.0)
TOWER_M4_MACHINE_X_MM = (-21.0, 21.0)
TOWER_M4_MACHINE_Z_MM = (60.0, 66.0)
TOWER_M4_INSERT_DEPTH_MM = 6.0
TOWER_M4_CLEAR_RADIUS_MM = 2.25
TOWER_M4_PILOT_RADIUS_MM = 2.85


def carriage_local_to_machine_reference(point: tuple[float, float, float]):
    """Map active-stator local to M0=0 machine; owner then translates on M0."""

    x, y, z = map(float, point)
    return (-y, z, float(PARAMS.m0_home_standoff) - x)


def to_machine_reference(shape: Part | Compound):
    """Exact assembly map for an active-local shape at M0=M1=0."""

    return (
        Pos(0.0, 0.0, float(PARAMS.m0_home_standoff))
        * Rot(0.0, 90.0, 0.0)
        * Rot(-90.0, 0.0, 0.0)
        * shape
    )


def _validate_sign(axial_sign: int) -> int:
    if int(axial_sign) not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    return int(axial_sign)


def cap_lane_endpoint_name(side: int) -> str:
    """Return the actual production-cap lane point used by one branch."""

    side_value = int(side)
    if side_value not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    return (
        LEFT_CAP_ENDPOINT_NAME if side_value < 0
        else RIGHT_CAP_ENDPOINT_NAME
    )


def cap_lane_endpoint(
    side: int, axial_sign: int,
) -> tuple[float, float, float]:
    """Return the exact BREP-lane binding point for one lead-in branch."""

    sign = _validate_sign(axial_sign)
    point = _CAP_LANE_POINTS[cap_lane_endpoint_name(side)]
    return (float(point[0]), float(point[1]), sign * float(point[2]))


def leadin_handoff(
    side: int, axial_sign: int,
) -> tuple[float, float, float]:
    """Return the unchanged fixed-bowl handoff point for one branch."""

    side_value = int(side)
    sign = _validate_sign(axial_sign)
    if side_value not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    return (
        HANDOFF_X_MM,
        side_value * PORT_TANGENTIAL_MM,
        sign * LEADIN_HIGH_AXIAL_MM,
    )


def _right_s_bend_points(
    axial_sign: int,
) -> dict[str, tuple[float, float, float]]:
    """Exact two equal/opposite XY arcs from the quarter turn to handoff."""

    sign = _validate_sign(axial_sign)
    theta = RIGHT_S_BEND_SWEEP_RAD
    radius = RIGHT_S_BEND_RADIUS_MM
    z = sign * LEADIN_HIGH_AXIAL_MM
    x0 = RIGHT_QUARTER_END_X_MM
    y0 = RIGHT_CAP_ENDPOINT_TANGENTIAL_MM
    join = (
        x0 + radius * math.sin(theta),
        y0 + radius * (1.0 - math.cos(theta)),
        z,
    )
    first_mid = (
        x0 + radius * math.sin(theta / 2.0),
        y0 + radius * (1.0 - math.cos(theta / 2.0)),
        z,
    )
    second_mid = (
        join[0] + radius * (
            math.sin(theta) - math.sin(theta / 2.0)
        ),
        join[1] + radius * (
            math.cos(theta / 2.0) - math.cos(theta)
        ),
        z,
    )
    return {
        "start": (x0, y0, z),
        "first_mid": first_mid,
        "join": join,
        "second_mid": second_mid,
        "end": leadin_handoff(1, sign),
    }


def _xz_quarter_midpoint(
    start: tuple[float, float, float], axial_sign: int,
) -> tuple[float, float, float]:
    """Explicit 45-degree point for a true XZ R3.50 quarter circle."""

    sign = _validate_sign(axial_sign)
    radius = LEADIN_CENTERLINE_RADIUS_MM
    return (
        float(start[0]) + radius * (1.0 - 1.0 / math.sqrt(2.0)),
        float(start[1]),
        sign * (
            LEADIN_BEND_CENTER_AXIAL_MM + radius / math.sqrt(2.0)
        ),
    )


def _leadin_centerline(side: int, axial_sign: int):
    side_value = int(side)
    sign = _validate_sign(axial_sign)
    if side_value not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    start = cap_lane_endpoint(side_value, sign)
    handoff = leadin_handoff(side_value, sign)
    with BuildLine() as path:
        if side_value < 0:
            tangent = side_value * PORT_TANGENTIAL_MM
            bend_start = (
                PORT_X_MM, tangent, sign * LEADIN_BEND_CENTER_AXIAL_MM,
            )
            bend_end = (
                LEADIN_BEND_X_MM, tangent, sign * LEADIN_HIGH_AXIAL_MM,
            )
            Line(
                start,
                bend_start,
            )
            ThreePointArc(
                bend_start,
                _xz_quarter_midpoint(bend_start, sign),
                bend_end,
            )
            Line(
                bend_end,
                handoff,
            )
        else:
            bend_start = (
                start[0], start[1], sign * LEADIN_BEND_CENTER_AXIAL_MM,
            )
            points = _right_s_bend_points(sign)
            Line(start, bend_start)
            ThreePointArc(
                bend_start,
                _xz_quarter_midpoint(bend_start, sign),
                points["start"],
            )
            ThreePointArc(
                points["start"], points["first_mid"], points["join"],
            )
            ThreePointArc(
                points["join"], points["second_mid"], points["end"],
            )
    return path.wire()


def _channel_profile_plane(side: int, axial_sign: int) -> Plane:
    side_value = int(side)
    sign = _validate_sign(axial_sign)
    if side_value not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    start = cap_lane_endpoint(side_value, sign)
    # Before tooth rotation +X is radial-outward.  Both access slots therefore
    # stay externally open while their negative tools remain separated from
    # every same-tooth, adjacent-side and adjacent-tooth groove.
    return Plane(
        origin=start,
        x_dir=(1.0, 0.0, 0.0),
        z_dir=(0.0, 0.0, float(sign)),
    )


def _outer_channel(side: int, axial_sign: int) -> Part:
    side_value = int(side)
    sign = _validate_sign(axial_sign)
    with BuildSketch(_channel_profile_plane(side_value, sign)) as profile:
        Circle(LEADIN_OUTER_RADIUS_MM)
    result = sweep(profile.sketch, _leadin_centerline(side_value, sign))
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_"
        f"{'right' if side_value > 0 else 'left'}_R1p20_positive_shell"
    )
    return result


def _channel_negative(side: int, axial_sign: int) -> Part:
    side_value = int(side)
    sign = _validate_sign(axial_sign)
    with BuildSketch(_channel_profile_plane(side_value, sign)) as profile:
        Circle(LEADIN_CLEAR_RADIUS_MM)
        with Locations((LEADIN_OUTER_RADIUS_MM, 0.0)):
            Rectangle(
                2.0 * LEADIN_OUTER_RADIUS_MM,
                LEADIN_OPENING_WIDTH_MM,
            )
    result = sweep(profile.sketch, _leadin_centerline(side_value, sign))
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_"
        f"{'right' if side_value > 0 else 'left'}_"
        "R0p45_groove_plus_radial_access_negative"
    )
    return result


def _open_channel(side: int, axial_sign: int) -> Part:
    side_value = int(side)
    sign = _validate_sign(axial_sign)
    result = _outer_channel(side_value, sign).cut(
        _channel_negative(side_value, sign)
    )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_"
        f"{'right' if side_value > 0 else 'left'}_open_R3p50_cap_leadin"
    )
    return result


def leadin_for_tooth(tooth: int, side: int, axial_sign: int) -> Part:
    if int(tooth) not in range(SLOTS):
        raise ValueError("tooth outside 0..23")
    return Rot(0.0, 0.0, int(tooth) * PITCH_DEG) * (
        _open_channel(side, axial_sign)
    )


def _outer_channel_for_tooth(tooth: int, side: int, axial_sign: int) -> Part:
    if int(tooth) not in range(SLOTS):
        raise ValueError("tooth outside 0..23")
    return Rot(0.0, 0.0, int(tooth) * PITCH_DEG) * (
        _outer_channel(side, axial_sign)
    )


def _channel_negative_for_tooth(
    tooth: int, side: int, axial_sign: int,
) -> Part:
    if int(tooth) not in range(SLOTS):
        raise ValueError("tooth outside 0..23")
    return Rot(0.0, 0.0, int(tooth) * PITCH_DEG) * (
        _channel_negative(side, axial_sign)
    )


def _right_seam_overlap_mouth_negative(axial_sign: int) -> Part:
    """Radial breakout spanning 0.45 mm into each side of the right seam."""

    sign = _validate_sign(axial_sign)
    x, y, z = cap_lane_endpoint(1, sign)
    result = Pos(x, y, z) * Box(
        RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM,
        RIGHT_SEAM_MOUTH_TANGENTIAL_WIDTH_MM,
        RIGHT_SEAM_MOUTH_AXIAL_SPAN_MM,
        align=(Align.MIN, Align.CENTER, Align.CENTER),
    )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_right_seam_"
        "radial_overlap_mouth_negative"
    )
    return result


def _right_seam_overlap_mouth_for_tooth(
    tooth: int, axial_sign: int,
) -> Part:
    if int(tooth) not in range(SLOTS):
        raise ValueError("tooth outside 0..23")
    return Rot(0.0, 0.0, int(tooth) * PITCH_DEG) * (
        _right_seam_overlap_mouth_negative(axial_sign)
    )


@lru_cache(maxsize=2)
def _cap_with_short_leadins_before_right_seam_mouth(
    axial_sign: int,
) -> Part:
    """Return globally grooved cap before the 24 right seam breakouts."""

    sign = _validate_sign(axial_sign)
    positives = [
        _outer_channel_for_tooth(tooth, side, sign)
        for tooth in range(SLOTS) for side in (-1, 1)
    ]
    negatives = [
        _channel_negative_for_tooth(tooth, side, sign)
        for tooth in range(SLOTS) for side in (-1, 1)
    ]
    result = cap.cap_part(sign).fuse(*positives).cut(*negatives)
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_globally_grooved_"
        "cap_before_right_seam_mouths"
    )
    return result


@lru_cache(maxsize=2)
def cap_with_short_leadins(axial_sign: int) -> Part:
    """One positively retained cap plus 48 accessible short lead-ins."""

    sign = _validate_sign(axial_sign)
    right_seam_mouths = [
        _right_seam_overlap_mouth_for_tooth(tooth, sign)
        for tooth in range(SLOTS)
    ]
    result = _cap_with_short_leadins_before_right_seam_mouth(sign).cut(
        *right_seam_mouths
    )
    solids = list(result.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"short-leadin crowned cap must be one solid; {len(solids)}"
        )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_one_solid_PEEK_cap_with_short_open_leadins"
    )
    return result


def _active_shoe(axial_sign: int) -> Part:
    sign = _validate_sign(axial_sign)
    result = predecessor._tooth_zero_shoe(sign, CAPTURE_RADIUS_MM)
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_single_shared_R40_capture_shoe"
    )
    return result


@lru_cache(maxsize=2)
def active_sector_guide(axial_sign: int) -> Part:
    """One carriage-owned PEEK capture shoe with left/right open bowls."""

    sign = _validate_sign(axial_sign)
    positives: list[Part] = [_active_shoe(sign)]
    cuts: list[Part] = []
    for side in (-1, 1):
        center = (
            FIXED_BOWL_X_MM,
            side * PORT_TANGENTIAL_MM,
            sign * FIXED_BOWL_AXIAL_MM,
        )
        positives.append(Pos(*center) * Sphere(FIXED_BOWL_OUTER_RADIUS_MM))
        cuts.append(Pos(*center) * Sphere(FIXED_BOWL_SURFACE_RADIUS_MM))
    barrel_x = CAPTURE_RADIUS_MM - predecessor.CAPTURE_SURFACE_RADIUS_MM
    rail_length = barrel_x - FIXED_BOWL_X_MM + 0.8
    for tangent in (-FIXED_RAIL_TANGENTIAL_MM, FIXED_RAIL_TANGENTIAL_MM):
        positives.append(Pos(
            FIXED_BOWL_X_MM + rail_length / 2.0 - 0.4,
            tangent,
            sign * FIXED_BOWL_AXIAL_MM,
        ) * Box(
            rail_length,
            FIXED_RAIL_WIDTH_MM,
            predecessor.BACKBONE_AXIAL_THICKNESS_MM,
            align=CTR,
        ))
    # Broad outer side webs make the bowl-to-shoe and pad-to-shoe load paths
    # positive-volume unions while staying outside the 6 mm wire mouth.
    for side in (-1, 1):
        positives.append(Pos(
            34.0,
            side * 4.40,
            sign * FIXED_BOWL_AXIAL_MM,
        ) * Box(8.0, 2.0, 2.0, align=CTR))
    # Open the inboard handoff face.  The retained outward hemisphere is the
    # polished R3.50 selection surface; the wire leaves radially toward the
    # rotating cap lead-in without a hidden passage.
    cuts.append(Pos(
        (HANDOFF_X_MM + FIXED_BOWL_X_MM) / 2.0,
        0.0,
        sign * FIXED_BOWL_AXIAL_MM,
    ) * Box(
        FIXED_BOWL_X_MM - HANDOFF_X_MM,
        2.0 * (FIXED_RAIL_TANGENTIAL_MM - FIXED_RAIL_WIDTH_MM),
        2.0 * FIXED_BOWL_OUTER_RADIUS_MM + 1.0,
        align=CTR,
    ))

    # Two integral pads end exactly at the inboard faces of the aluminum
    # rails.  They are face-mated, never volume-overlapped.  A short M3
    # heat-set insert is installed from that accessible outer pad face.
    for tangent_sign in (-1, 1):
        positives.append(Pos(
            M3_BOLT_RADIAL_X_MM,
            tangent_sign * (
                GUIDE_PAD_CONTACT_TANGENTIAL_MM - 4.60 / 2.0
            ),
            sign * FIXED_BOWL_AXIAL_MM,
        ) * Box(5.50, 4.60, 6.0, align=CTR))
        # Full-section bridge into the outer bowl/web.  Its inboard corner
        # stays >2 mm from the rotating lead-in envelope; no thin collision
        # notch or decorative point connection is used.
        positives.append(Pos(
            30.0,
            tangent_sign * 5.55,
            sign * FIXED_BOWL_AXIAL_MM,
        ) * Box(8.8, 1.1, 2.0, align=CTR))
        # A positive rectangular key controls radial and axial location;
        # the M3 stack supplies clamp force only.  The matching aluminum
        # pocket has 0.05 mm clearance per side.
        positives.append(Pos(
            GUIDE_DATUM_X_MM,
            tangent_sign * (
                GUIDE_PAD_CONTACT_TANGENTIAL_MM
                + GUIDE_DATUM_KEY_DEPTH_MM / 2.0
            ),
            sign * (
                FIXED_BOWL_AXIAL_MM + GUIDE_DATUM_AXIAL_OFFSET_MM
            ),
        ) * Box(
            GUIDE_DATUM_KEY_RADIAL_MM,
            GUIDE_DATUM_KEY_DEPTH_MM,
            GUIDE_DATUM_KEY_AXIAL_MM,
            align=CTR,
        ))
        cuts.append(hardware.place(
            Cylinder(
                M3_INSERT_PILOT_RADIUS_MM,
                M3_INSERT_PILOT_DEPTH_MM,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ),
            (
                M3_BOLT_RADIAL_X_MM,
                tangent_sign * GUIDE_PAD_CONTACT_TANGENTIAL_MM,
                sign * FIXED_BOWL_AXIAL_MM,
            ),
            axis="-y" if tangent_sign > 0 else "+y",
        ))
    result = positives[0].fuse(*positives[1:]).cut(*cuts)
    solids = list(result.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"active-sector PEEK guide must be one solid; {len(solids)}"
        )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_M0_following_M1_static_PEEK_active_sector"
    )
    return result


@lru_cache(maxsize=1)
def carriage_yoke() -> Part:
    """One aluminum split-yoke collision/load body for the two guide ends."""

    pieces: list[Part] = []
    for tangent in (-YOKE_TANGENTIAL_MM, YOKE_TANGENTIAL_MM):
        pieces.append(Pos(
            YOKE_RADIAL_X_MM,
            tangent,
            (YOKE_AXIAL_MIN_MM + YOKE_AXIAL_MAX_MM) / 2.0,
        ) * Box(
            YOKE_BAR_RADIAL_LENGTH_MM,
            YOKE_BAR_WIDTH_MM,
            YOKE_AXIAL_SPAN_MM,
            align=CTR,
        ))
    # Thick local seats connect the guide pads to the forward-plane beams.
    # The +axial seat first drops to z=5 at x=36.5, where it is clear of the
    # wire chord.  Both axial branches then travel tangentially outward while
    # still at x=33..40 before crossing radially at |y|=34.5.  The prior
    # inboard radial/tangent pair crossed the cap and R26 coil bodies.
    for axial_sign in (-1, 1):
        for tangent_sign in (-1, 1):
            pieces.append(Pos(
                (
                    YOKE_GUIDE_SEAT_X_MIN_MM
                    + YOKE_GUIDE_SEAT_X_MAX_MM
                ) / 2.0,
                tangent_sign * GUIDE_SEAT_TANGENTIAL_MM,
                axial_sign * FIXED_BOWL_AXIAL_MM,
            ) * Box(
                YOKE_GUIDE_SEAT_X_MAX_MM - YOKE_GUIDE_SEAT_X_MIN_MM,
                YOKE_BAR_WIDTH_MM,
                YOKE_SEAT_MEMBER_AXIAL_MM,
                    align=CTR,
                ))
            connector_axial = (
                YOKE_FRONT_DOGLEG_AXIAL_MM
                if axial_sign > 0 else -FIXED_BOWL_AXIAL_MM
            )
            if axial_sign > 0:
                # Full-section axial dogleg.  Its ends overlap the keyed seat
                # and forward radial member by half their 7.4 mm depth.
                pieces.append(Pos(
                    YOKE_GUIDE_DOGLEG_X_MM,
                    tangent_sign * GUIDE_SEAT_TANGENTIAL_MM,
                    (FIXED_BOWL_AXIAL_MM + connector_axial) / 2.0,
                ) * Box(
                    7.0,
                    YOKE_BAR_WIDTH_MM,
                    FIXED_BOWL_AXIAL_MM - connector_axial
                    + YOKE_SEAT_MEMBER_AXIAL_MM,
                    align=CTR,
                ))
            # Tangential leg stays wholly outside the cap/coil radius while
            # joining the keyed seat/dogleg to the outboard radial beam.
            outboard_tangent_center = (
                GUIDE_SEAT_TANGENTIAL_MM + YOKE_TANGENTIAL_MM
            ) / 2.0
            outboard_tangent_span = (
                YOKE_TANGENTIAL_MM - GUIDE_SEAT_TANGENTIAL_MM
                + YOKE_BAR_WIDTH_MM
            )
            pieces.append(Pos(
                YOKE_GUIDE_DOGLEG_X_MM,
                tangent_sign * outboard_tangent_center,
                connector_axial,
            ) * Box(
                7.0,
                outboard_tangent_span,
                YOKE_SEAT_MEMBER_AXIAL_MM,
                align=CTR,
            ))
            # Radial beam crosses only at the outboard |y|=34.5 plane and
            # overlaps both the tangential leg and long forward rail.
            pieces.append(Pos(
                (
                    YOKE_FRONT_PLANE_X_MIN_MM
                    + YOKE_GUIDE_CONNECTOR_X_MAX_MM
                ) / 2.0,
                tangent_sign * YOKE_TANGENTIAL_MM,
                connector_axial,
            ) * Box(
                (
                    YOKE_GUIDE_CONNECTOR_X_MAX_MM
                    - YOKE_FRONT_PLANE_X_MIN_MM
                ),
                YOKE_BAR_WIDTH_MM,
                YOKE_SEAT_MEMBER_AXIAL_MM,
                align=CTR,
            ))
    # Two rear struts leave the U-plate beside the tower bridge.  Broad
    # full-section bridges at local axial -95..-89 reach the forward-plane
    # rails entirely behind every wrap, deposition and flyer sweep witness.
    rear_tangent = 24.50
    for tangent_sign in (-1, 1):
        tangent = tangent_sign * rear_tangent
        pieces.append(Pos(
            32.0,
            tangent,
            -100.0,
        ) * Box(6.0, YOKE_BAR_WIDTH_MM, 24.0, align=CTR))
        pieces.append(Pos(
            (YOKE_FRONT_PLANE_X_MIN_MM + 35.0) / 2.0,
            tangent_sign * ((19.5 + 36.3) / 2.0),
            -92.0,
        ) * Box(
            35.0 - YOKE_FRONT_PLANE_X_MIN_MM,
            36.3 - 19.5,
            6.0,
            align=CTR,
        ))
    # Actual tower adapter plate.  In machine reference the plate is
    # x=+/-28, y=-114..-110, z=57..69 at M0 reference.
    pieces.append(Pos(
        sum(TOWER_ADAPTER_LOCAL_X_MM) / 2.0,
        sum(TOWER_ADAPTER_LOCAL_Y_MM) / 2.0,
        sum(TOWER_ADAPTER_LOCAL_Z_MM) / 2.0,
    ) * Box(
        TOWER_ADAPTER_LOCAL_X_MM[1] - TOWER_ADAPTER_LOCAL_X_MM[0],
        TOWER_ADAPTER_LOCAL_Y_MM[1] - TOWER_ADAPTER_LOCAL_Y_MM[0],
        TOWER_ADAPTER_LOCAL_Z_MM[1] - TOWER_ADAPTER_LOCAL_Z_MM[0],
        align=CTR,
    ))
    result = pieces[0].fuse(*pieces[1:])
    # Guide-key pockets.  They are cut after union so no rail seam can refill
    # a pocket, and their depth leaves >9 mm aluminum behind the key.
    for axial_sign in (-1, 1):
        for tangent_sign in (-1, 1):
            result -= Pos(
                GUIDE_DATUM_X_MM,
                tangent_sign * (
                    GUIDE_PAD_CONTACT_TANGENTIAL_MM
                    + (GUIDE_DATUM_KEY_DEPTH_MM + GUIDE_DATUM_CLEARANCE_MM)
                    / 2.0
                ),
                axial_sign * (
                    FIXED_BOWL_AXIAL_MM + GUIDE_DATUM_AXIAL_OFFSET_MM
                ),
            ) * Box(
                GUIDE_DATUM_KEY_RADIAL_MM + 2.0 * GUIDE_DATUM_CLEARANCE_MM,
                GUIDE_DATUM_KEY_DEPTH_MM + GUIDE_DATUM_CLEARANCE_MM,
                GUIDE_DATUM_KEY_AXIAL_MM + 2.0 * GUIDE_DATUM_CLEARANCE_MM,
                align=CTR,
            )
    # The tower's central bearing bridge projects forward at machine
    # x=+/-17, z>=65.  An upper-center window makes the adapter a U-shaped
    # face plate: its two M4 ears land only on the real front-column face,
    # while the lower crossbar keeps the yoke one solid.
    result -= Pos(
        27.75,
        0.0,
        (TOWER_ADAPTER_LOCAL_Z_MM[0] + TOWER_ADAPTER_LOCAL_Z_MM[1]) / 2.0,
    ) * Box(
        5.5,
        35.0,
        TOWER_ADAPTER_LOCAL_Z_MM[1] - TOWER_ADAPTER_LOCAL_Z_MM[0] + 1.0,
        align=CTR,
    )
    # Twin aluminum adapter keys project 1.5 mm into matching tower pockets.
    # Their wide spacing positively locates the U-plate; M4 clearance is no
    # longer part of the guide position tolerance stack.
    for machine_x in TOWER_DATUM_MACHINE_X_MM:
        pieces_key = Pos(
            float(PARAMS.m0_home_standoff) - TOWER_DATUM_MACHINE_Z_MM,
            -machine_x,
            -114.65,
        ) * Box(
            TOWER_DATUM_KEY_Z_MM,
            TOWER_DATUM_KEY_X_MM,
            TOWER_DATUM_KEY_DEPTH_MM + 0.2,
            align=CTR,
        )
        result = result.fuse(pieces_key)
    for axial_sign in (-1, 1):
        for tangent_sign in (-1, 1):
            # M3 clearance through the aluminum rail.  The screw is installed
            # from the fully accessible outboard face and engages the PEEK
            # insert across the exact rail/pad mating face.
            result -= hardware.place(
                Cylinder(
                    M3_HOLE_RADIUS_MM,
                    YOKE_BAR_WIDTH_MM + 1.0,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                ),
                (
                    M3_BOLT_RADIAL_X_MM,
                    tangent_sign * (
                        GUIDE_SEAT_OUTER_TANGENTIAL_MM + 0.5
                    ),
                    axial_sign * FIXED_BOWL_AXIAL_MM,
                ),
                axis="-y" if tangent_sign > 0 else "+y",
            )
    # Four M4 clearance holes through the actual tower adapter plate.
    for machine_x in TOWER_M4_MACHINE_X_MM:
        for machine_z in TOWER_M4_MACHINE_Z_MM:
            result -= Pos(
                float(PARAMS.m0_home_standoff) - machine_z,
                -machine_x,
                TOWER_ADAPTER_LOCAL_Z_MM[0] - 0.5,
            ) * Cylinder(
                TOWER_M4_CLEAR_RADIUS_MM,
                TOWER_ADAPTER_LOCAL_Z_MM[1] - TOWER_ADAPTER_LOCAL_Z_MM[0] + 1.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )
    solids = list(result.solids())
    if len(solids) != 1:
        bounds = [
            (
                (s.bounding_box().min.X, s.bounding_box().min.Y, s.bounding_box().min.Z),
                (s.bounding_box().max.X, s.bounding_box().max.Y, s.bounding_box().max.Z),
            )
            for s in solids
        ]
        raise RuntimeError(
            f"carriage yoke must be one solid; {len(solids)} {bounds}"
        )
    result.label = "M0_carriage_owned_aluminum_active_sector_split_yoke"
    return result


def guide_retention_hardware() -> tuple[Part, ...]:
    result: list[Part] = []
    for axial_sign in (-1, 1):
        for tangent_sign in (-1, 1):
            outward_axis = "+y" if tangent_sign > 0 else "-y"
            inward_axis = "-y" if tangent_sign > 0 else "+y"
            rail_outer = tangent_sign * (
                GUIDE_SEAT_OUTER_TANGENTIAL_MM
            )
            washer_origin = (
                M3_BOLT_RADIAL_X_MM,
                rail_outer,
                axial_sign * FIXED_BOWL_AXIAL_MM,
            )
            washer = hardware.place(
                hardware.plain_washer(
                    "M3",
                    label=(
                        f"active_sector_M3_washer_{axial_sign:+d}_{tangent_sign:+d}"
                    ),
                ),
                washer_origin,
                axis=outward_axis,
            )
            screw = hardware.place(
                hardware.socket_head_cap_screw(
                    "M3", 14.0,
                    label=(
                        f"active_sector_M3x14_{axial_sign:+d}_{tangent_sign:+d}"
                    ),
                ),
                (
                    M3_BOLT_RADIAL_X_MM,
                    rail_outer + tangent_sign * M3_WASHER_THICKNESS_MM,
                    axial_sign * FIXED_BOWL_AXIAL_MM,
                ),
                axis=outward_axis,
            )
            insert = hardware.place(
                hardware.heat_set_insert(
                    "M3", length="short",
                    label=(
                        f"active_sector_M3_heat_insert_{axial_sign:+d}_{tangent_sign:+d}"
                    ),
                ),
                (
                    M3_BOLT_RADIAL_X_MM,
                    tangent_sign * GUIDE_PAD_CONTACT_TANGENTIAL_MM,
                    axial_sign * FIXED_BOWL_AXIAL_MM,
                ),
                axis=inward_axis,
            )
            result.extend((screw, washer, insert))
    return tuple(result)


@lru_cache(maxsize=1)
def revised_spindle_tower() -> Part:
    """Real tower with four front-installed M4 heat-set insert pilots."""

    result = printed.spindle_tower()
    # Front face of the low column is machine y=-114.  Pilots extend rearward
    # six millimetres and stay wholly inside x+/-30,z57..69 column material.
    for machine_x in TOWER_M4_MACHINE_X_MM:
        for machine_z in TOWER_M4_MACHINE_Z_MM:
            tool = hardware.place(
                Cylinder(
                    TOWER_M4_PILOT_RADIUS_MM,
                    TOWER_M4_INSERT_DEPTH_MM,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                ),
                (machine_x, -114.0, machine_z),
                axis="-y",
            )
            result -= tool
    for machine_x in TOWER_DATUM_MACHINE_X_MM:
        result -= Pos(
            machine_x,
            -114.75,
            TOWER_DATUM_MACHINE_Z_MM,
        ) * Box(
            TOWER_DATUM_KEY_X_MM + 2.0 * TOWER_DATUM_CLEARANCE_MM,
            TOWER_DATUM_KEY_DEPTH_MM + 0.2,
            TOWER_DATUM_KEY_Z_MM + 2.0 * TOWER_DATUM_CLEARANCE_MM,
            align=CTR,
        )
    result.label = "spindle_tower_with_active_sector_M4_insert_pilots"
    return result


def tower_adapter_hardware_reference() -> tuple[Part, ...]:
    """Four M4x10/washer/heat-insert stacks in machine reference frame."""

    result: list[Part] = []
    for machine_x in TOWER_M4_MACHINE_X_MM:
        for machine_z in TOWER_M4_MACHINE_Z_MM:
            suffix = f"{'P' if machine_x > 0 else 'N'}X_{machine_z:.0f}Z"
            washer = hardware.place(
                hardware.plain_washer(
                    "M4", label=f"active_sector_tower_M4_washer_{suffix}",
                ),
                (machine_x, -110.0, machine_z), axis="+y",
            )
            screw = hardware.place(
                hardware.socket_head_cap_screw(
                    "M4", 10.0,
                    label=f"active_sector_tower_ISO4762_M4x10_{suffix}",
                ),
                (machine_x, -110.0 + M4_WASHER_THICKNESS_MM, machine_z),
                axis="+y",
            )
            insert = hardware.place(
                hardware.heat_set_insert(
                    "M4", length="short",
                    label=f"active_sector_tower_M4_heat_insert_{suffix}",
                ),
                (machine_x, -114.0, machine_z), axis="-y",
            )
            result.extend((screw, washer, insert))
    return tuple(result)


def carriage_link_parts() -> list[Part]:
    return [
        active_sector_guide(1), active_sector_guide(-1),
        carriage_yoke(), *guide_retention_hardware(),
    ]


def carriage_link_reference_parts() -> list[Part]:
    result = [to_machine_reference(part) for part in carriage_link_parts()]
    result.append(revised_spindle_tower())
    result.extend(tower_adapter_hardware_reference())
    return result


def spindle_link_parts() -> list[Part]:
    return [
        cap_with_short_leadins(1), cap_with_short_leadins(-1),
        *cap.retention_hardware(),
    ]


def gen_step() -> Compound:
    stator = to_machine_reference(
        stator_model.stator(DEFAULT_STATOR, label="default_stator_context")
    )
    spindle = to_machine_reference(Compound(children=spindle_link_parts()))
    spindle.label = "M1_rotating_caps_with_short_open_leadins"
    carriage = Compound(children=carriage_link_reference_parts())
    carriage.label = "M0_following_M1_static_active_sector_guide"
    result = Compound(children=[stator, spindle, carriage])
    result.label = "carriage_active_sector_terminal_guide_review"
    return result


if __name__ == "__main__":
    front = active_sector_guide(1)
    rear = active_sector_guide(-1)
    cap_front = cap_with_short_leadins(1)
    print(
        "active sector",
        len(list(front.solids())), len(list(rear.solids())),
        len(list(cap_front.solids())),
        f"M1gap={ARBITRARY_M1_RADIAL_CLEARANCE_MM:.3f}mm",
    )
