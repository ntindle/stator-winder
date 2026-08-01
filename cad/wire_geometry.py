"""Shared analytical wire centerlines for CAD, animation, and validation.

The path is defined once here and consumed by ``wire_vis.py`` and exported
through ``out/links/manifest.json`` for ``sim/wirepath.py``.  Coordinates are
the machine frame from ``params.py`` (millimetres).

The static chain is deliberately physical rather than a guide-centre
polyline:

* the spool payoff, felt contact, and dancer entry tangent are collinear;
* the felt contact is outside the M4 stud but inside the felt discs;
* the dancer follows the outside of the pulley for 80 degrees using exact
  tangent points; and
* a 4 mm-radius, tangent 90-degree elbow turns the wire into the axial entry
  passage.

The flyer elbow is also a true tangent circular fillet (5 mm radius) between
the hollow-shaft axis and the straight run to the tip eyelet.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

from params import PARAMS as P

Vec3 = tuple[float, float, float]

WIRE_DIAMETER_MAX = 0.5
WIRE_RADIUS_MAX = WIRE_DIAMETER_MAX / 2.0
DANCER_WRAP_DEG = 80.0
DANCER_BODY_RADIUS = 8.0
DANCER_PATH_RADIUS = DANCER_BODY_RADIUS + WIRE_RADIUS_MAX
ENTRY_BEND_RADIUS = 4.0
FLYER_ELBOW_RADIUS = 5.0
FLYER_ELBOW_BODY_RADIUS = 3.2
ARC_STEP_DEG = 5.0
# The wire exits the tip eyelet at z=-1.5 and lays onto the tooth 3.5 mm
# behind it. Z=+2.0 keeps the free-span angle under 6 degrees while reducing
# required stator insertion enough for the 28..65 mm launch OD range. The old
# arbitrary -0.5 plane overran the finite coil prism and made OD28/36/65
# unreachable at the root end.
TOOTH_CONTACT_Z = 2.0

# The flyer reverses the wire by roughly 120..150 degrees at its tip.  A
# straight ceramic eyelet cannot supply that turn without a sharp, unmodelled
# lip contact.  The release geometry therefore uses a polished ceramic torus:
# the wire approaches through its centre, follows one meridian, and leaves on
# a tangent toward the work.  Offsetting the Ø6 tube by the maximum wire radius
# gives a 3.25 mm centreline bend, while the 6.5 mm major radius leaves the
# same 3.25 mm minimum radius around the inside of the torus.
TIP_GUIDE_MAJOR_RADIUS = 6.5
TIP_GUIDE_TUBE_RADIUS = 3.0
# Keep the complete R5 elbow body behind the longest launch shaft at the
# deepest captured M0 pose.  For the maximum 20 mm launch stack, the shaft-tip
# envelope reaches z=-10.915 mm; the elbow reaches BODY_RADIUS in front of
# this plane.  z=-17 therefore leaves 2.885 mm while preserving every
# bend/tangent radius and keeping the radial feed inside the flyer-arm window.
TIP_GUIDE_CENTER_Z = -17.0
TIP_GUIDE_FEED_Y = 12.0

# A seamless, polished sleeve remains on the exposed upper work shaft for the
# inter-phase wraps.  OD8 is the minimum production size; larger shafts retain
# at least a 1 mm radial sleeve wall.  This is a configured changeover part,
# not a generic set-screw collar (a slit or screw would be a snag feature).
SHAFT_WRAP_MIN_OD = 8.0
SHAFT_WRAP_MIN_RADIAL_WALL = 1.0
SHAFT_WRAP_LENGTH = 6.0
SHAFT_WRAP_BORE_CLEARANCE = 0.05


def tooth_contact_spec(stator, coil: dict) -> dict:
    """Shared finite final-wound tooth contact envelope for settings/twin.

    The winding free span is tangent to the wire-centre offset of the final
    rectangular coil envelope. M0 traverses only the collision model's finite
    radial winding span at ``TOOTH_CONTACT_Z``; this prevents the former
    0.33/1.82 mm overtravel beyond the coil prism ends.
    """
    growth = float(coil["bundle"]["collision_growth_mm"])
    envelope = coil["bundle"]["end_turn_envelope"]
    radial_start = float(coil["bundle"]["radial_winding_start_mm"])
    radial_end = float(coil["bundle"]["radial_winding_end_mm"])
    deep = stator.od / 2.0 - radial_start - TOOTH_CONTACT_Z
    shallow = stator.od / 2.0 - radial_end - TOOTH_CONTACT_Z
    return {
        "model": (
            "support tangent to maximum-wire offset of smooth elliptical "
            "final-coil envelope"
        ),
        "physical_tangential_radius_mm": float(
            envelope["tangential_radius_mm"]
        ),
        "physical_axial_radius_mm": float(envelope["axial_radius_mm"]),
        "wire_offset_radius_mm": WIRE_RADIUS_MAX,
        "tangential_half_extent_mm": (
            float(envelope["tangential_radius_mm"]) + WIRE_RADIUS_MAX
        ),
        "axial_half_extent_mm": (
            float(envelope["axial_radius_mm"]) + WIRE_RADIUS_MAX
        ),
        "minimum_max_wire_deposited_curvature_radius_mm": float(
            envelope["minimum_deposited_wire_center_curvature_radius_mm"]
            - float(envelope["validation_wire_radius_mm"])
            + WIRE_RADIUS_MAX
        ),
        "minimum_small_wire_deposited_curvature_radius_mm": float(
            envelope["minimum_deposited_wire_center_curvature_radius_mm"]
        ),
        "deposited_bend_gate": False,
        "bend_rule_scope": envelope["bend_rule_scope"],
        "z_mm": TOOTH_CONTACT_Z,
        "radial_winding_span_mm": [radial_start, radial_end],
        "insertion_depth_range_mm": [shallow, deep],
    }


def shaft_wrap_sleeve_spec(stator) -> dict:
    """Configured seamless phase-lead sleeve on the exposed upper shaft."""
    outer_d = max(
        SHAFT_WRAP_MIN_OD,
        stator.shaft_d + 2.0 * SHAFT_WRAP_MIN_RADIAL_WALL,
    )
    shaft_top = stator.stack / 2.0 + stator.shaft_above
    # Keep the complete 6 mm contact band inside the finite exposed stub and
    # away from both the lamination face and the shaft end.
    axial_y = shaft_top - SHAFT_WRAP_LENGTH / 2.0 - 0.5
    axial_span = [
        axial_y - SHAFT_WRAP_LENGTH / 2.0,
        axial_y + SHAFT_WRAP_LENGTH / 2.0,
    ]
    if axial_span[0] <= stator.stack / 2.0:
        raise ValueError("upper shaft stub is too short for wrap sleeve")
    return {
        "model": "seamless polished permanent phase-lead wrap sleeve",
        "outer_diameter_mm": outer_d,
        "bore_diameter_mm": stator.shaft_d + SHAFT_WRAP_BORE_CLEARANCE,
        "length_mm": SHAFT_WRAP_LENGTH,
        "axial_y_mm": axial_y,
        "axial_span_mm": axial_span,
        "shaft_top_y_mm": shaft_top,
        "minimum_edge_radius_mm": 0.75,
        "finish_ra_um_max": 0.2,
    }


def shaft_contact_spec(stator) -> dict:
    sleeve = shaft_wrap_sleeve_spec(stator)
    return {
        "model": "line tangent to finite polished shaft-wrap sleeve",
        "radius_to_wire_center_mm": (
            sleeve["outer_diameter_mm"] / 2.0 + WIRE_RADIUS_MAX
        ),
        "axial_y_mm": sleeve["axial_y_mm"],
        "axial_span_mm": sleeve["axial_span_mm"],
        "sleeve": sleeve,
    }


def tip_guide_spec() -> dict:
    """Shared polished toroidal flyer-tip guide and its fixed feed point."""
    wire_path_radius = TIP_GUIDE_TUBE_RADIUS + WIRE_RADIUS_MAX
    inside_path_radius = TIP_GUIDE_MAJOR_RADIUS - wire_path_radius
    if min(wire_path_radius, inside_path_radius) < 3.0:
        raise ValueError("tip torus violates the 3 mm wire bend-radius rule")
    return {
        "model": "tangent-arc-tangent path over polished ceramic torus",
        "center_local_mm": [0.0, P.flyer_tip_r, TIP_GUIDE_CENTER_Z],
        "axis_local": [0.0, 1.0, 0.0],
        "feed_local_mm": [0.0, TIP_GUIDE_FEED_Y, TIP_GUIDE_CENTER_Z],
        "major_radius_mm": TIP_GUIDE_MAJOR_RADIUS,
        "tube_radius_mm": TIP_GUIDE_TUBE_RADIUS,
        "wire_path_radius_mm": wire_path_radius,
        "inside_wire_path_radius_mm": inside_path_radius,
        "minimum_wire_center_bend_radius_mm": min(
            wire_path_radius, inside_path_radius
        ),
        "material": "99.8% alumina ceramic",
        "finish_ra_um_max": 0.2,
    }


def _add(a: Vec3, b: Vec3) -> Vec3:
    return tuple(a[i] + b[i] for i in range(3))  # type: ignore[return-value]


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return tuple(a[i] - b[i] for i in range(3))  # type: ignore[return-value]


def _mul(a: Vec3, s: float) -> Vec3:
    return tuple(v * s for v in a)  # type: ignore[return-value]


def _dot(a: Vec3, b: Vec3) -> float:
    return sum(a[i] * b[i] for i in range(3))


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _norm(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Vec3) -> Vec3:
    length = _norm(a)
    if length < 1e-9:
        raise ValueError("zero-length wire direction")
    return _mul(a, 1.0 / length)


def _rotate(v: Vec3, axis: Vec3, angle: float) -> Vec3:
    """Rodrigues rotation of ``v`` about unit ``axis``."""
    c, s = math.cos(angle), math.sin(angle)
    return _add(_add(_mul(v, c), _mul(_cross(axis, v), s)),
                _mul(axis, _dot(axis, v) * (1.0 - c)))


def _dedupe(points: Iterable[Vec3], tol: float = 1e-8) -> list[Vec3]:
    result: list[Vec3] = []
    for p in points:
        q = tuple(float(v) for v in p)  # type: ignore[assignment]
        if not result or _norm(_sub(q, result[-1])) > tol:
            result.append(q)
    return result


def _circular_fillet(corner: Vec3, incoming: Vec3, outgoing: Vec3,
                     radius: float, step_deg: float = ARC_STEP_DEG):
    """Return a tangent circular fillet for a path through ``corner``.

    ``incoming`` and ``outgoing`` are unit path directions toward and away
    from the virtual sharp corner.  The returned arc includes both tangent
    points and carries deterministic metadata used by the validator.
    """
    u = _unit(incoming)
    v = _unit(outgoing)
    turn = math.acos(max(-1.0, min(1.0, _dot(u, v))))
    if not math.radians(1.0) < turn < math.radians(179.0):
        raise ValueError(f"unsupported fillet turn {math.degrees(turn):.3f} deg")
    trim = radius * math.tan(turn / 2.0)
    start = _sub(corner, _mul(u, trim))
    end = _add(corner, _mul(v, trim))
    axis = _unit(_cross(u, v))
    inward = _unit(_cross(axis, u))
    center = _add(start, _mul(inward, radius))
    r0 = _sub(start, center)
    count = max(2, math.ceil(math.degrees(turn) / step_deg))
    points = [_add(center, _rotate(r0, axis, turn * i / count))
              for i in range(count + 1)]
    if _norm(_sub(points[-1], end)) > 1e-6:
        raise AssertionError("fillet endpoint construction drift")
    return points, {
        "radius": radius,
        "turn_deg": math.degrees(turn),
        "trim": trim,
        "start": list(start),
        "end": list(end),
        "center": list(center),
        "axis": list(axis),
        "incoming_direction": list(u),
        "outgoing_direction": list(v),
    }


def _point_on_tangent_at_y(tangent: Vec3, theta: float, y: float) -> Vec3:
    # (-sin(theta), cos(theta)) is tangent to the circle in the XY plane.
    dx, dy = -math.sin(theta), math.cos(theta)
    if abs(dy) < 1e-9:
        raise ValueError("dancer tangent is horizontal; cannot hit requested Y")
    scale = (y - tangent[1]) / dy
    return (tangent[0] + dx * scale, y, tangent[2])


def static_path_spec() -> dict:
    """Complete fixed-frame centerline from supply spool to flyer bore."""
    z_plane = P.rear_post_z + 23.0
    dancer_center = (P.dancer_pulley_x, P.dancer_pulley_y, z_plane)
    entry_corner = (0.0, 0.0, z_plane)

    cx, cy = dancer_center[:2]
    theta_in = math.pi
    theta_out = theta_in - math.radians(DANCER_WRAP_DEG)

    def on_pulley(theta: float) -> Vec3:
        return (cx + DANCER_PATH_RADIUS * math.cos(theta),
                cy + DANCER_PATH_RADIUS * math.sin(theta), z_plane)

    tangent_in = on_pulley(theta_in)
    tangent_out = on_pulley(theta_out)
    # The clockwise tangent at theta=180 deg points exactly +Y, so the
    # supply, felt contact, and dancer entry share one straight X/Z line.
    felt = (tangent_in[0], P.felt_y, z_plane)
    spool = (tangent_in[0], P.spool_y, z_plane)
    felt_guide_in = (tangent_in[0], P.felt_y - 15.0, z_plane)

    arc_count = max(2, math.ceil(DANCER_WRAP_DEG / ARC_STEP_DEG))
    dancer_arc = [on_pulley(theta_in + (theta_out - theta_in) * i / arc_count)
                  for i in range(arc_count + 1)]

    entry_incoming = _unit(_sub(entry_corner, tangent_out))
    entry_arc, entry_meta = _circular_fillet(
        entry_corner, entry_incoming, (0.0, 0.0, 1.0), ENTRY_BEND_RADIUS)

    entry_eyelet = (0.0, 0.0, P.wire_entry_z + 3.0)
    bore_rear = (0.0, 0.0, P.flyer_shaft_rear_z)
    points = _dedupe([spool, felt_guide_in, felt, tangent_in,
                      *dancer_arc[1:],
                      *entry_arc, entry_eyelet, bore_rear])

    # A longer version of the elbow path is used to cut an open, curved
    # passage through the entry bracket.  It extends beyond the housing on
    # the incoming side and overlaps the axial bore on the outgoing side.
    entry_start = entry_arc[0]
    channel_open = _sub(entry_start, _mul(entry_incoming, 7.0))
    channel_axial = _add(entry_arc[-1], (0.0, 0.0, 7.0))

    return {
        "points": [list(p) for p in points],
        "landmarks": {
            "spool_payoff": list(spool),
            "felt_contact": list(felt),
            "felt_guide_in": list(felt_guide_in),
            "dancer_center": list(dancer_center),
            "dancer_tangent_in": list(tangent_in),
            "dancer_tangent_out": list(tangent_out),
            "entry_corner": list(entry_corner),
            "entry_eyelet": list(entry_eyelet),
            "bore_rear": list(bore_rear),
        },
        "dancer": {
            "body_radius": DANCER_BODY_RADIUS,
            "path_radius": DANCER_PATH_RADIUS,
            "theta_in_deg": math.degrees(theta_in),
            "theta_out_deg": math.degrees(theta_out),
            "wrap_deg": DANCER_WRAP_DEG,
            "direction": "clockwise",
            "tangent_in_direction": [math.sin(theta_in),
                                     -math.cos(theta_in), 0.0],
            "tangent_out_direction": [math.sin(theta_out),
                                      -math.cos(theta_out), 0.0],
        },
        "entry_bend": entry_meta,
        "entry_channel_points": [list(p) for p in _dedupe(
            [channel_open, entry_start, *entry_arc[1:], channel_axial])],
        "felt_offset_from_stud": math.hypot(felt[0] - P.rear_post_x,
                                             felt[1] - P.felt_y),
        "spool_pack_radius": math.hypot(spool[1] - P.spool_y,
                                         spool[2] - (P.rear_post_z + 60.0)),
    }


def flyer_path_spec() -> dict:
    """Fixed flyer-frame centreline from bore rear to torus feed.

    The work-facing portion after ``guide_feed`` is configuration-dependent
    and is constructed analytically by the simulator as a tangent/arc/tangent
    path over :func:`tip_guide_spec`.  Keeping that span dynamic avoids the old
    fictitious kink at the centre of a sharp annular eyelet.
    """
    corner = (0.0, 0.0, TIP_GUIDE_CENTER_Z)
    feed = (0.0, TIP_GUIDE_FEED_Y, TIP_GUIDE_CENTER_Z)
    outgoing = (0.0, 1.0, 0.0)
    elbow_arc, elbow_meta = _circular_fillet(
        corner, (0.0, 0.0, 1.0), outgoing, FLYER_ELBOW_RADIUS)
    bore_rear = (0.0, 0.0, P.flyer_shaft_rear_z)
    points = _dedupe([bore_rear, elbow_arc[0], *elbow_arc[1:], feed])

    # Extend both ends so the subtraction opens into the shaft bore and the
    # straight radial feed tube.
    channel_axial = _sub(elbow_arc[0], (0.0, 0.0, 12.0))
    channel_out = _add(feed, _mul(outgoing, 2.0))
    return {
        "points": [list(p) for p in points],
        "landmarks": {
            "bore_rear": list(bore_rear),
            "elbow_corner": list(corner),
            "elbow_exit": list(elbow_arc[-1]),
            "guide_feed": list(feed),
            "tip_guide_center": tip_guide_spec()["center_local_mm"],
        },
        "elbow_bend": elbow_meta,
        "elbow_channel_points": [list(p) for p in _dedupe(
            [channel_axial, elbow_arc[0], *elbow_arc[1:], channel_out])],
    }


def polyline_length(points: Sequence[Sequence[float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))
