"""Isolated four-occurrence aggregate-boundary replacement carriage.

This source is a replacement prototype, not an additive assembly patch.  It
reuses the collision-cleared U-window/outboard-rail corridor of the current
M0 carriage yoke, restores its obsolete guide M3 cuts, subsumes the obsolete
guide-key volumes into parked-follower reliefs, and adds four
integral coarse-selection bays.  Exactly four handed follower occurrences are
carriage-owned.  M1 and M2 select an identity but never transform a part.

Active-local frame (mm): +X radial/outward, +Y tangential, +Z stator axis.
At M0 home ``machine=(-local_y, local_z, 95-local_x)``.

The positive 8.90 mm coarse-selection solids are blocker envelopes only.  No
actuator/linkage is claimed, and this file grants no assembly-integration,
collision, route, load, buildability, procurement, BOM, production, or release
authority.  It is intentionally not imported by :mod:`assembly`.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from functools import lru_cache
import hashlib
import json
from pathlib import Path
from typing import Any

from build123d import Align, Box, Compound, Cylinder, Plane, Pos, Rot, Vector

import aggregate_boundary_floating_follower as follower
import carriage_active_sector_terminal_guide as active
import hardware
import m1_selector_alternating_former as selector


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

ROOT = Path(__file__).resolve().parents[1]
STEP_OUT = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_replacement_carriage.step"
)
MANIFEST_OUT = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_replacement_carriage_manifest.json"
)

REFERENCE_M0_STANDOFF_MM = 95.0
FIXED_AXIAL_DATUM_MM = 21.35
SOURCE_GIMBAL_AXIAL_MM = 24.0
SOURCE_TO_CARRIER_X_SHIFT_MM = -0.30
SOURCE_TO_FRONT_Z_SHIFT_MM = FIXED_AXIAL_DATUM_MM - SOURCE_GIMBAL_AXIAL_MM

COARSE_ACTIVE_BASE_MM = 2.05
COARSE_PARKED_BASE_MM = 10.95
COARSE_SELECTION_TRAVEL_MM = (
    COARSE_PARKED_BASE_MM - COARSE_ACTIVE_BASE_MM
)
PASSIVE_TANGENTIAL_USABLE_MM = 0.50

# Exact downstream-clearance repair for the inherited front doglegs.  Each
# cutter removes only the inboard corner beside one parked follower; the
# outboard X=36.20..39.00 web and the outer tangential half remain continuous.
# The axial band clears every inner-pivot leaf while stopping 1.15 mm above
# the unchanged front tangential member at Z=8.70 mm.
PARKED_FOLLOWER_RELIEF_X_MIN_MM = 25.00
PARKED_FOLLOWER_RELIEF_X_MAX_MM = 36.20
PARKED_FOLLOWER_RELIEF_TANGENTIAL_INNER_MM = 5.45
PARKED_FOLLOWER_RELIEF_TANGENTIAL_OUTER_MM = 16.50
PARKED_FOLLOWER_RELIEF_AXIAL_INNER_MM = 9.85
PARKED_FOLLOWER_RELIEF_AXIAL_OUTER_MM = 27.85

# Trim only the axial-outboard 1.00 mm of each selection-bay wall.  This
# preserves its floor/connector overlap while closing the exact 2.00 mm
# cartridge-to-carrier transition envelope at the parked endpoint.
SELECTION_WALL_AXIAL_INNER_MM = 2.85
SELECTION_WALL_AXIAL_OUTER_MM = 12.85
SELECTION_BAY_TANGENTIAL_CLEARANCE_MM = 0.50

RADIAL_HARD_CENTER_MIN_MM = 13.80
RADIAL_HARD_CENTER_MAX_MM = 20.20

M0_GATE_STATES = (
    "ENGAGED_LOCKED",
    "FORCED_RETRACTION_RAMP",
    "ALL_RETRACTED_DISCONNECTED",
)
M1_LAWS = selector.LAW_CODES
M2_TRACK_RADII_MM = selector.CAM_TRACK_RADII_MM

DIRECT_PULSE_ANGLES_DEG = (15.0, 30.0, 195.0, 210.0)
REVERSE_PULSE_ANGLES_DEG = (345.0, 330.0, 165.0, 150.0)

PRIMARY_M4_STACK_COUNT = 4
PRIMARY_M4_LEAF_COUNT = 8
MOVING_OCCURRENCE_COUNT = 4
MOVING_CUSTOM_BODY_COUNT_PER_OCCURRENCE = 4
MOVING_PIVOT_LEAF_COUNT_PER_OCCURRENCE = 11
MOVING_LEAF_COUNT_PER_OCCURRENCE = (
    MOVING_CUSTOM_BODY_COUNT_PER_OCCURRENCE
    + MOVING_PIVOT_LEAF_COUNT_PER_OCCURRENCE
)
COARSE_BLOCKER_COUNT = 4
EXPECTED_LEAF_SOLID_COUNT = (
    1
    + MOVING_OCCURRENCE_COUNT * MOVING_LEAF_COUNT_PER_OCCURRENCE
    + PRIMARY_M4_LEAF_COUNT
    + COARSE_BLOCKER_COUNT
)

AUTHORITY = {
    "review_only": True,
    "assembly_integration_authorized": False,
    "collision_authorized": False,
    "wire_route_authorized": False,
    "load_authorized": False,
    "buildability_authorized": False,
    "procurement_authorized": False,
    "production_authorized": False,
    "BOM_released": False,
    "release_authorized": False,
}

BLOCKERS = (
    "UNMODELED_positive_volume_8p90mm_coarse_selection_linkage",
    "UNMODELED_positive_M0_retraction_linkage",
    "PROVISIONAL_four_occurrence_route_and_clearance_placement",
    "OPEN_MISUMI_SCCG5_10_pivot_pin_retention_load_and_wear",
    "OPEN_5p52Nm_primary_mount_structural_proof",
    "OPEN_tangential_bearing_return_spring_and_wear_qualification",
)

# Exact compact replacement for the rejected inward McMaster shoulder-screw /
# nyloc stack.  MISUMI's configurable SCCG family permits D=5 mm and
# L=10.0..60.0 mm in 0.1 mm increments and includes two stainless E-rings.
# NETWS4 is the catalog No.4 stainless E-ring: 9 mm radial envelope, 0.6 mm
# thick, for the 4.0 mm groove.  The closed annulus below is deliberately a
# conservative collision envelope for the real E-profile.
OUTER_PIVOT_PIN_SKU = "MISUMI_SCCG5-10"
OUTER_PIVOT_RING_SKU = "MISUMI_NETWS4_included_2pcs"
OUTER_PIVOT_PIN_DIAMETER_MM = 5.0
OUTER_PIVOT_PIN_LENGTH_MM = 10.0
OUTER_PIVOT_GROOVE_DIAMETER_MM = 4.0
OUTER_PIVOT_GROOVE_WIDTH_MM = 0.70
OUTER_PIVOT_GROOVE_CENTER_ABS_Y_MM = 4.65
OUTER_PIVOT_RING_OD_MM = 9.0
OUTER_PIVOT_RING_THICKNESS_MM = 0.60
OUTER_PIVOT_SHIM_HUB_FACE_ABS_Y_MM = 2.00
OUTER_PIVOT_STACK_HALF_ENVELOPE_MM = OUTER_PIVOT_PIN_LENGTH_MM / 2.0
ENGAGED_OPPOSED_CENTER_SPACING_MM = (
    COARSE_ACTIVE_BASE_MM + COARSE_PARKED_BASE_MM
)
ENGAGED_YOKE_BODY_CLEARANCE_MM = (
    ENGAGED_OPPOSED_CENTER_SPACING_MM - 10.0
)
ENGAGED_OUTER_PIVOT_STACK_CLEARANCE_MM = (
    ENGAGED_OPPOSED_CENTER_SPACING_MM
    - 2.0 * OUTER_PIVOT_STACK_HALF_ENVELOPE_MM
)

REJECTED_INWARD_NYLOC_WITNESS = {
    "stack": "McMaster_96654A127_OD5x10_M4_plus_inward_M4_nyloc",
    "all_parked_opposed_pair_count": 2,
    "all_parked_screw_tip_overlap_each_mm": 0.10,
    "all_parked_screw_tip_common_volume_each_mm3": 1.2566370614359,
    "engaged_cross_occurrence_positive_pair_count_each_identity": 14,
    "engaged_cross_occurrence_common_volume_each_identity_mm3": 161.463338,
    "status": "REJECTED_REPLACED_BY_COMPACT_GROOVED_PIN_STACK",
}

# Primary mount replacement pattern.  The inherited same-side axes were only
# 6 mm apart, so ISO 7089 OD9 washers and ISO 4762 OD7 heads were impossible.
# These diagonals retain the proof-critical X-row span of 6 mm and increase
# the Y span.  Recessed NBK ultra-low small heads carry a nylon locking patch,
# so the invalid washer occurrences are removed rather than hidden.
PRIMARY_M4_LOCAL_X_MM = (29.0, 35.0)
PRIMARY_M4_OUTER_ABS_Y_MM = 24.5
PRIMARY_M4_INNER_ABS_Y_MM = 17.5
PRIMARY_M4_DIAGONAL_CENTER_DISTANCE_MM = (
    6.0 ** 2 + 7.0 ** 2
) ** 0.5
PRIMARY_M4_SCREW_SKU = "NBK_SSHS-M4-10-SD-ALK"
PRIMARY_M4_HEAD_DIAMETER_MM = 6.0
PRIMARY_M4_HEAD_HEIGHT_MM = 1.5
PRIMARY_M4_SCREW_LENGTH_MM = 10.0
PRIMARY_M4_BEARING_PLANE_Z_MM = -111.5
PRIMARY_M4_COUNTERBORE_DIAMETER_MM = 6.2
PRIMARY_M4_CLEARANCE_DIAMETER_MM = 4.5
PRIMARY_M4_INSERT_SKU = "McMaster_short_M4_heat_set_insert"
REJECTED_PRIMARY_M4_WITNESS = {
    "pattern": "local_x_29_35_at_each_y_plus_minus_21",
    "same_side_center_pitch_mm": 6.0,
    "ISO4762_head_diameter_mm": 7.0,
    "ISO7089_washer_diameter_mm": 9.0,
    "positive_pair_count_reference_state": 18,
    "positive_common_volume_reference_state_mm3": 323.6953723851857,
    "status": "REJECTED_REPLACED_BY_DIAGONAL_RECESSED_SMALL_HEAD_PATTERN",
}


@dataclass(frozen=True)
class OccurrenceIdentity:
    index: int
    name: str
    axial_sign: int
    tangential_sign: int


OCCURRENCE_IDENTITIES = (
    OccurrenceIdentity(
        0, "front_left", axial_sign=1, tangential_sign=-1,
    ),
    OccurrenceIdentity(
        1, "front_right", axial_sign=1, tangential_sign=1,
    ),
    OccurrenceIdentity(
        2, "rear_right", axial_sign=-1, tangential_sign=1,
    ),
    OccurrenceIdentity(
        3, "rear_left", axial_sign=-1, tangential_sign=-1,
    ),
)


def _label(shape, label: str):
    shape.label = label
    return shape


def _cylinder_y(radius_mm: float, length_mm: float):
    return Rot(90.0, 0.0, 0.0) * Cylinder(
        radius_mm, length_mm, align=CTR,
    )


def active_local_to_machine_reference(
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Map this source's active-local point to the M0-home machine frame."""

    x, y, z = map(float, point)
    return (-y, z, REFERENCE_M0_STANDOFF_MM - x)


def _primary_m4_local_locations() -> tuple[tuple[float, float, float], ...]:
    result = []
    for tangential_sign in (-1, 1):
        result.extend((
            (
                PRIMARY_M4_LOCAL_X_MM[0],
                tangential_sign * PRIMARY_M4_OUTER_ABS_Y_MM,
                -114.0,
            ),
            (
                PRIMARY_M4_LOCAL_X_MM[1],
                tangential_sign * PRIMARY_M4_INNER_ABS_Y_MM,
                -114.0,
            ),
        ))
    return tuple(result)


def physical_occurrence_index(m1_law: str, m2_track: int) -> int:
    """Return the physical follower identity selected by one law/track.

    Direct and reverse-zero preserve track identity.  Reverse-180 exchanges
    the opposite identities (0<->2 and 1<->3).  This is a state mapping only;
    no M1/M2 transform is applied to carriage-owned geometry.
    """

    if m1_law not in M1_LAWS:
        raise ValueError(f"m1_law must be one of {M1_LAWS!r}")
    if isinstance(m2_track, bool) or not isinstance(m2_track, int):
        raise ValueError("m2_track must be an integer from 0 through 3")
    if m2_track not in range(MOVING_OCCURRENCE_COUNT):
        raise ValueError("m2_track must be an integer from 0 through 3")
    if m1_law == selector.LAW_REVERSE_180:
        return (m2_track + 2) % MOVING_OCCURRENCE_COUNT
    return m2_track


def selected_occurrences(
    m1_law: str,
    m2_track: int,
    m0_gate_state: str,
) -> tuple[dict[str, Any], ...]:
    """Return all four physical occurrence states for one selector state.

    `ENGAGED_LOCKED` produces one active, radially floating reference
    occurrence and three parked/retracted occurrences.  Both positive-M0 gate
    states keep all four parked and retracted because their common retraction
    linkage is still unresolved.
    """

    selected_index = physical_occurrence_index(m1_law, m2_track)
    if m0_gate_state not in M0_GATE_STATES:
        raise ValueError(f"m0_gate_state must be one of {M0_GATE_STATES!r}")
    engaged = m0_gate_state == "ENGAGED_LOCKED"
    states: list[dict[str, Any]] = []
    for identity in OCCURRENCE_IDENTITIES:
        selected = engaged and identity.index == selected_index
        coarse_base = (
            COARSE_ACTIVE_BASE_MM if selected else COARSE_PARKED_BASE_MM
        )
        radial_state = "mid" if selected else "retracted"
        local_nose = occurrence_nose_center(
            identity,
            radial_state=radial_state,
            coarse_base_mm=coarse_base,
            passive_tangential_mm=0.0,
        )
        states.append({
            **asdict(identity),
            "owner": "M0_carriage",
            "M1_spatial_transform": False,
            "M2_spatial_transform": False,
            "m1_law": m1_law,
            "m2_track_index": m2_track,
            "m2_track_radius_mm": M2_TRACK_RADII_MM[m2_track],
            "m0_gate_state": m0_gate_state,
            "selected": selected,
            "coarse_state": "active" if selected else "parked",
            "coarse_base_abs_y_mm": coarse_base,
            "coarse_selection_travel_mm": COARSE_SELECTION_TRAVEL_MM,
            "radial_reference_state": radial_state,
            "passive_tangential_reference_state": "center",
            "local_nose_center_mm": list(local_nose),
            "machine_reference_nose_center_mm": list(
                active_local_to_machine_reference(local_nose)
            ),
        })
    return tuple(states)


def _restore_obsolete_guide_cuts(body):
    """Return current yoke BREP with old guide pockets/M3 holes filled."""

    fills = []
    for axial_sign in (-1, 1):
        for tangential_sign in (-1, 1):
            fills.append(Pos(
                active.GUIDE_DATUM_X_MM,
                tangential_sign * (
                    active.GUIDE_PAD_CONTACT_TANGENTIAL_MM
                    + (active.GUIDE_DATUM_KEY_DEPTH_MM
                       + active.GUIDE_DATUM_CLEARANCE_MM) / 2.0
                ),
                axial_sign * (
                    active.FIXED_BOWL_AXIAL_MM
                    + active.GUIDE_DATUM_AXIAL_OFFSET_MM
                ),
            ) * Box(
                active.GUIDE_DATUM_KEY_RADIAL_MM
                + 2.0 * active.GUIDE_DATUM_CLEARANCE_MM,
                active.GUIDE_DATUM_KEY_DEPTH_MM
                + active.GUIDE_DATUM_CLEARANCE_MM,
                active.GUIDE_DATUM_KEY_AXIAL_MM
                + 2.0 * active.GUIDE_DATUM_CLEARANCE_MM,
                align=CTR,
            ))
            fills.append(Pos(
                active.M3_BOLT_RADIAL_X_MM,
                tangential_sign * active.GUIDE_SEAT_TANGENTIAL_MM,
                axial_sign * active.FIXED_BOWL_AXIAL_MM,
            ) * _cylinder_y(
                active.M3_HOLE_RADIUS_MM,
                active.YOKE_BAR_WIDTH_MM,
            ))
    return body.fuse(*fills)


def _migrate_primary_m4_pattern(body):
    """Fill the impossible 6 mm-pitch stacks and cut four diagonal pockets."""

    original_fills = []
    for x in (29.0, 35.0):
        for y in (-21.0, 21.0):
            original_fills.append(Pos(x, y, -112.0) * Cylinder(
                active.TOWER_M4_CLEAR_RADIUS_MM,
                4.0,
                align=CTR,
            ))
    result = body.fuse(*original_fills)
    cuts = []
    for x, y, _z in _primary_m4_local_locations():
        cuts.append(Pos(x, y, -114.5) * Cylinder(
            PRIMARY_M4_CLEARANCE_DIAMETER_MM / 2.0,
            5.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ))
        cuts.append(Pos(x, y, PRIMARY_M4_BEARING_PLANE_Z_MM - 0.10)
                    * Cylinder(
            PRIMARY_M4_COUNTERBORE_DIAMETER_MM / 2.0,
            PRIMARY_M4_HEAD_HEIGHT_MM + 0.20,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ))
    return result.cut(*cuts)


def _selection_bay_pieces() -> tuple:
    """Four integral floors/walls tied into the proven outboard rails."""

    pieces = []
    floor_center_abs_y = (
        COARSE_ACTIVE_BASE_MM + COARSE_PARKED_BASE_MM
    ) / 2.0
    follower_half_width_y = 5.0
    bay_clearance_y = SELECTION_BAY_TANGENTIAL_CLEARANCE_MM
    floor_span_y = (
        COARSE_SELECTION_TRAVEL_MM
        + 2.0 * follower_half_width_y
        + 2.0 * bay_clearance_y
    )
    wall_thickness_y = 1.50
    outer_wall_abs_y = (
        COARSE_PARKED_BASE_MM + follower_half_width_y
        + bay_clearance_y + wall_thickness_y / 2.0
    )
    for identity in OCCURRENCE_IDENTITIES:
        a = identity.axial_sign
        t = identity.tangential_sign
        floor = Pos(16.0, t * floor_center_abs_y, a * 2.35) * Box(
            20.0, floor_span_y, 1.0, align=CTR,
        )
        wall_axial_center = (
            SELECTION_WALL_AXIAL_INNER_MM
            + SELECTION_WALL_AXIAL_OUTER_MM
        ) / 2.0
        wall_axial_span = (
            SELECTION_WALL_AXIAL_OUTER_MM
            - SELECTION_WALL_AXIAL_INNER_MM
        )
        outer_wall = Pos(
            16.0, t * outer_wall_abs_y, a * wall_axial_center,
        ) * Box(
            20.0, wall_thickness_y, wall_axial_span, align=CTR,
        )
        connector_min_abs_y = outer_wall_abs_y - wall_thickness_y / 2.0 - 0.10
        connector_max_abs_y = (
            active.YOKE_TANGENTIAL_MM
            - active.YOKE_BAR_WIDTH_MM / 2.0 + 0.10
        )
        connector = Pos(
            13.0,
            t * (connector_min_abs_y + connector_max_abs_y) / 2.0,
            a * 5.0,
        ) * Box(
            14.0,
            connector_max_abs_y - connector_min_abs_y,
            5.5,
            align=CTR,
        )
        # There is intentionally no central/inner wall.  The two opposed
        # selection lanes share the floor: an inner wall for either parked
        # bay occupies the other identity's selected Y=+/-2.05 sweep.
        pieces.extend((floor, outer_wall, connector))
    return tuple(pieces)


def _obsolete_guide_seat_reliefs() -> tuple:
    """Clear the four parked-follower envelopes beside the old guide seats.

    The cutter follows the exact transition-clearance trade: X=25.00..36.20,
    |Y|=5.45..16.50 and |Z|=9.85..27.85 mm.  It removes the inherited inner
    dogleg corner that approached the inner yoke/nose/pivot stack, while the
    X=36.20..39.00 outboard web and outer tangential half retain a continuous
    one-solid structural path.  Load authority remains explicitly open.
    """

    cuts = []
    for identity in OCCURRENCE_IDENTITIES:
        cuts.append(Pos(
            (
                PARKED_FOLLOWER_RELIEF_X_MIN_MM
                + PARKED_FOLLOWER_RELIEF_X_MAX_MM
            ) / 2.0,
            identity.tangential_sign * (
                PARKED_FOLLOWER_RELIEF_TANGENTIAL_INNER_MM
                + PARKED_FOLLOWER_RELIEF_TANGENTIAL_OUTER_MM
            ) / 2.0,
            identity.axial_sign * (
                PARKED_FOLLOWER_RELIEF_AXIAL_INNER_MM
                + PARKED_FOLLOWER_RELIEF_AXIAL_OUTER_MM
            ) / 2.0,
        ) * Box(
            (
                PARKED_FOLLOWER_RELIEF_X_MAX_MM
                - PARKED_FOLLOWER_RELIEF_X_MIN_MM
            ),
            (
                PARKED_FOLLOWER_RELIEF_TANGENTIAL_OUTER_MM
                - PARKED_FOLLOWER_RELIEF_TANGENTIAL_INNER_MM
            ),
            (
                PARKED_FOLLOWER_RELIEF_AXIAL_OUTER_MM
                - PARKED_FOLLOWER_RELIEF_AXIAL_INNER_MM
            ),
            align=CTR,
        ))
    return tuple(cuts)


@lru_cache(maxsize=1)
def replacement_carrier():
    """One shared U-windowed 6061 carrier with four integral bays."""

    body = _restore_obsolete_guide_cuts(active.carriage_yoke())
    body = _migrate_primary_m4_pattern(body)
    body = body.fuse(*_selection_bay_pieces())
    body = body.cut(*_obsolete_guide_seat_reliefs())
    solids = list(body.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"replacement carrier must be one solid; got {len(solids)}"
        )
    return _label(
        body,
        "M0_carriage_owned_shared_U_window_replacement_carrier_6061",
    )


def _handed_source_part(
    part,
    identity: OccurrenceIdentity,
    coarse_base_mm: float,
    label: str,
):
    """Apply the provisional canonical-source handed occurrence transform."""

    result = part
    if identity.tangential_sign < 0:
        result = result.mirror(Plane.XZ)
    if identity.axial_sign < 0:
        result = result.mirror(Plane.XY)
    result = Pos(
        SOURCE_TO_CARRIER_X_SHIFT_MM,
        identity.tangential_sign * float(coarse_base_mm),
        identity.axial_sign * SOURCE_TO_FRONT_Z_SHIFT_MM,
    ) * result
    return _label(result, f"{identity.name}:{label}")


def _compact_outer_cartridge(
    radial_state: str,
    tangential_state: str = "center",
):
    """Monolithic cartridge with internal shim and recessed E-ring reliefs."""

    body = follower.tangential_slide_outer_gimbal_cartridge(
        radial_state, tangential_state,
    )
    x, y, z = follower._gimbal_center(radial_state, tangential_state)
    for sign in (-1, 1):
        shim_cutter = hardware.place(
            Cylinder(
                5.10, 0.52,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ),
            (x, y + sign * OUTER_PIVOT_SHIM_HUB_FACE_ABS_Y_MM, z),
            axis="+y" if sign > 0 else "-y",
        )
        ring_cutter = Pos(
            x,
            y + sign * OUTER_PIVOT_GROOVE_CENTER_ABS_Y_MM,
            z,
        ) * _cylinder_y(
            OUTER_PIVOT_RING_OD_MM / 2.0 + 0.10,
            OUTER_PIVOT_RING_THICKNESS_MM + 0.04,
        )
        body = body.cut(shim_cutter, ring_cutter)
    body.label = (
        "monolithic_7075_tangential_slide_outer_yoke_compact_pin_reliefs:"
        f"{radial_state}:{tangential_state}"
    )
    return body


def occurrence_nose_center(
    identity: OccurrenceIdentity,
    *,
    radial_state: str,
    coarse_base_mm: float,
    passive_tangential_mm: float = 0.0,
) -> tuple[float, float, float]:
    if abs(float(passive_tangential_mm)) > PASSIVE_TANGENTIAL_USABLE_MM:
        raise ValueError("passive tangential offset exceeds +/-0.50 mm")
    source_nose_x = follower.radial_position(radial_state) + 16.0
    return (
        source_nose_x + SOURCE_TO_CARRIER_X_SHIFT_MM,
        identity.tangential_sign * (
            float(coarse_base_mm) + float(passive_tangential_mm)
        ),
        identity.axial_sign * FIXED_AXIAL_DATUM_MM,
    )


def _outer_pivot_compact_catalog_stack(
    radial_state: str,
    tangential_state: str = "center",
) -> tuple:
    """MISUMI SCCG5-10 pin, two NETWS4 rings and two DIN988 shims.

    The two E-rings are conservative closed-annulus envelopes.  Their actual
    E-profile removes material, so a clear annulus audit is conservative.
    """

    x, y, z = follower._gimbal_center(radial_state, tangential_state)
    pin = _cylinder_y(
        OUTER_PIVOT_PIN_DIAMETER_MM / 2.0,
        OUTER_PIVOT_PIN_LENGTH_MM,
    )
    groove_tools = []
    for sign in (-1, 1):
        outer = Pos(
            0.0, sign * OUTER_PIVOT_GROOVE_CENTER_ABS_Y_MM, 0.0,
        ) * _cylinder_y(
            OUTER_PIVOT_PIN_DIAMETER_MM / 2.0 + 0.2,
            OUTER_PIVOT_GROOVE_WIDTH_MM,
        )
        core = Pos(
            0.0, sign * OUTER_PIVOT_GROOVE_CENTER_ABS_Y_MM, 0.0,
        ) * _cylinder_y(
            OUTER_PIVOT_GROOVE_DIAMETER_MM / 2.0,
            OUTER_PIVOT_GROOVE_WIDTH_MM + 0.2,
        )
        groove_tools.append(outer - core)
    pin = Pos(x, y, z) * pin.cut(*groove_tools)
    pin.label = "outer_pivot_MISUMI_SCCG5-10_D5_grooved_pin"

    result = [pin]
    for sign, side in ((1, "positive"), (-1, "negative")):
        shim = hardware.place(
            hardware.thrust_washer(
                5.0, 10.0, 0.5,
                label=f"outer_pivot_DIN988_5x10x0p5_{side}_shim",
            ),
            (x, y + sign * OUTER_PIVOT_SHIM_HUB_FACE_ABS_Y_MM, z),
            axis="+y" if sign > 0 else "-y",
        )
        ring = Pos(
            x,
            y + sign * OUTER_PIVOT_GROOVE_CENTER_ABS_Y_MM,
            z,
        ) * _cylinder_y(
            OUTER_PIVOT_RING_OD_MM / 2.0,
            OUTER_PIVOT_RING_THICKNESS_MM,
        )
        ring -= Pos(
            x,
            y + sign * OUTER_PIVOT_GROOVE_CENTER_ABS_Y_MM,
            z,
        ) * _cylinder_y(
            OUTER_PIVOT_GROOVE_DIAMETER_MM / 2.0,
            OUTER_PIVOT_RING_THICKNESS_MM + 0.2,
        )
        ring.label = (
            f"outer_pivot_MISUMI_NETWS4_{side}_"
            "conservative_closed_annulus"
        )
        result.extend((shim, ring))
    return tuple(result)


@lru_cache(maxsize=16)
def moving_occurrence(
    identity: OccurrenceIdentity,
    *,
    radial_state: str = "retracted",
    coarse_base_mm: float = COARSE_PARKED_BASE_MM,
) -> Compound:
    """Build one handed follower occurrence without a local carrier/context."""

    raw_parts = (
        (
            follower.radial_slide(radial_state),
            f"radial_slide_7075:{radial_state}",
        ),
        (
            _compact_outer_cartridge(radial_state, "center"),
            f"monolithic_7075_tangential_slide_outer_yoke:{radial_state}",
        ),
        (
            follower.inner_gimbal_yoke(radial_state, "center"),
            "inner_gimbal_yoke_6061",
        ),
        (
            follower.nose_insert(radial_state, "center"),
            "virgin_unfilled_PEEK_R3_open_groove_nose",
        ),
    )
    children = [
        _handed_source_part(part, identity, coarse_base_mm, label)
        for part, label in raw_parts
    ]
    pivot_parts = (
        *_outer_pivot_compact_catalog_stack(radial_state, "center"),
        *(
            part for part in follower.gimbal_pin_hardware(
                radial_state, "center",
            )
            if str(part.label).startswith("inner_pivot_")
        ),
    )
    for index, part in enumerate(pivot_parts):
        role = str(part.label or f"pivot_hardware_{index:02d}")
        children.append(_handed_source_part(
            part, identity, coarse_base_mm, role,
        ))
    result = Compound(children=children)
    result.label = (
        f"moving_follower_occurrence_{identity.index}:{identity.name}:"
        f"a{identity.axial_sign:+d}:t{identity.tangential_sign:+d}"
    )
    return result


def coarse_selection_linkage_blocker(
    identity: OccurrenceIdentity,
):
    """Positive-volume envelope for the unresolved 8.90 mm linkage travel."""

    center_abs_y = (
        COARSE_ACTIVE_BASE_MM + COARSE_PARKED_BASE_MM
    ) / 2.0
    body = Pos(
        3.75,
        identity.tangential_sign * center_abs_y,
        identity.axial_sign * 8.35,
    ) * Box(4.0, COARSE_SELECTION_TRAVEL_MM, 4.0, align=CTR)
    return _label(
        body,
        f"BLOCKER_ONLY_8p90mm_coarse_selection_linkage_envelope:"
        f"{identity.name}",
    )


@lru_cache(maxsize=1)
def primary_tower_m4_hardware() -> tuple:
    """Four recessed NBK small-head M4 screws and four short inserts."""

    parts = []
    for index, (x, y, _z) in enumerate(_primary_m4_local_locations()):
        suffix = f"{index:02d}"
        shank = Pos(0.0, 0.0, -PRIMARY_M4_SCREW_LENGTH_MM) * Cylinder(
            2.0,
            PRIMARY_M4_SCREW_LENGTH_MM,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        head = Cylinder(
            PRIMARY_M4_HEAD_DIAMETER_MM / 2.0,
            PRIMARY_M4_HEAD_HEIGHT_MM,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        screw = Pos(x, y, PRIMARY_M4_BEARING_PLANE_Z_MM) * (shank + head)
        screw.label = f"primary_tower_NBK_SSHS_M4x10_SD_ALK_{suffix}"
        insert = hardware.place(
            hardware.heat_set_insert(
                "M4",
                length="short",
                label=f"primary_tower_M4_short_heat_insert_{suffix}",
            ),
            (x, y, -114.0),
            axis="-z",
        )
        parts.extend((screw, insert))
    parts = tuple(parts)
    if len(parts) != PRIMARY_M4_LEAF_COUNT:
        raise RuntimeError(
            f"expected {PRIMARY_M4_LEAF_COUNT} primary M4 leaves, "
            f"got {len(parts)}"
        )
    return parts


@dataclass(frozen=True)
class ManufacturedLeaf:
    group: str
    label: str
    shape: Any


def manufactured_leaves_for_state(
    m1_law: str,
    m2_track: int,
    m0_gate_state: str,
    *,
    include_primary_mount: bool = True,
) -> tuple[ManufacturedLeaf, ...]:
    """Return the exact manufactured leaves for one selector/gate state.

    The four blocker envelopes are excluded because they are explicitly not a
    manufactured linkage.  The primary mount is included by default and the
    four follower occurrences always include their full pivot hardware.
    """

    states = selected_occurrences(m1_law, m2_track, m0_gate_state)
    leaves = [ManufacturedLeaf(
        "carrier",
        str(replacement_carrier().label),
        replacement_carrier(),
    )]
    for identity, state in zip(OCCURRENCE_IDENTITIES, states):
        occurrence = moving_occurrence(
            identity,
            radial_state=state["radial_reference_state"],
            coarse_base_mm=state["coarse_base_abs_y_mm"],
        )
        leaves.extend(
            ManufacturedLeaf(
                f"occurrence:{identity.name}",
                str(child.label),
                child,
            )
            for child in occurrence.children
        )
    if include_primary_mount:
        leaves.extend(
            ManufacturedLeaf("primary_mount", str(part.label), part)
            for part in primary_tower_m4_hardware()
        )
    labels = [leaf.label for leaf in leaves]
    if len(labels) != len(set(labels)):
        raise RuntimeError("manufactured leaf labels must be unique")
    return tuple(leaves)


def _bbox_bounds(shape) -> tuple[float, float, float, float, float, float]:
    """Materialize one shape's AABB once for a pair-audit pass.

    ``Shape.bounding_box()`` asks OCC to traverse the BREP.  Calling it for
    both leaves in every pair dominated the audit even though a leaf's box is
    invariant within a selector-state signature.  Keep the cached value as
    plain floats so no later loop iteration can trigger another traversal.
    """

    box = shape.bounding_box()
    return (
        float(box.min.X), float(box.max.X),
        float(box.min.Y), float(box.max.Y),
        float(box.min.Z), float(box.max.Z),
    )


def _bbox_bounds_intersect(
    one: tuple[float, float, float, float, float, float],
    two: tuple[float, float, float, float, float, float],
    tolerance_mm: float = 1.0e-7,
) -> bool:
    """Return true only for a positive-volume AABB candidate."""

    t = float(tolerance_mm)
    return (
        min(one[1], two[1]) - max(one[0], two[0]) > t
        and min(one[3], two[3]) - max(one[2], two[2]) > t
        and min(one[5], two[5]) - max(one[4], two[4]) > t
    )


def _bbox_intersects(one, two, tolerance_mm: float = 1.0e-7) -> bool:
    """Return true only for a positive-volume AABB candidate.

    Face, edge, and point contact cannot produce positive common volume and
    must not trigger an expensive OCC boolean.  This audit is explicitly an
    interpenetration test; clearance/distance authority remains separate.
    """

    return _bbox_bounds_intersect(
        _bbox_bounds(one), _bbox_bounds(two), tolerance_mm,
    )


@lru_cache(maxsize=5)
def _manufactured_pair_audit_geometry(
    selected_occurrence_index: int,
) -> dict[str, Any]:
    """Audit one of the five unique geometries: all parked or one selected."""

    if selected_occurrence_index == -1:
        law = selector.LAW_DIRECT
        track = 0
        gate = "ALL_RETRACTED_DISCONNECTED"
        signature = "all_parked_retracted"
    elif selected_occurrence_index in range(MOVING_OCCURRENCE_COUNT):
        law = selector.LAW_DIRECT
        track = selected_occurrence_index
        gate = "ENGAGED_LOCKED"
        signature = f"engaged_selected_{selected_occurrence_index}"
    else:
        raise ValueError("selected_occurrence_index must be -1 or 0..3")

    leaves = manufactured_leaves_for_state(law, track, gate)
    pair_count = len(leaves) * (len(leaves) - 1) // 2
    follower_leaf_count = sum(
        leaf.group != "primary_mount" for leaf in leaves
    )
    follower_pair_count = follower_leaf_count * (follower_leaf_count - 1) // 2
    # AABBs are deliberately precomputed once per leaf.  The exact BREP
    # distance stage is a safe broad-phase after the AABB: positive-volume
    # intersection implies zero shape distance.  Only zero/near-zero-distance
    # candidates reach the authoritative OCC common-volume boolean.  Any
    # distance-kernel exception fails closed into that boolean as well.
    leaf_bounds = tuple(_bbox_bounds(leaf.shape) for leaf in leaves)
    full_overlaps = []
    follower_overlaps = []
    bbox_candidate_count = 0
    follower_bbox_candidate_count = 0
    exact_distance_candidate_count = 0
    follower_exact_distance_candidate_count = 0
    exact_boolean_candidate_count = 0
    follower_exact_boolean_candidate_count = 0
    for index, first in enumerate(leaves):
        for second_index in range(index + 1, len(leaves)):
            second = leaves[second_index]
            follower_pair = (
                first.group != "primary_mount"
                and second.group != "primary_mount"
            )
            if not _bbox_bounds_intersect(
                leaf_bounds[index], leaf_bounds[second_index],
            ):
                continue
            bbox_candidate_count += 1
            if follower_pair:
                follower_bbox_candidate_count += 1
            exact_distance_candidate_count += 1
            if follower_pair:
                follower_exact_distance_candidate_count += 1
            try:
                exact_distance_mm = float(
                    first.shape.distance_to(second.shape)
                )
            except Exception:
                # The common-volume boolean is the authority.  A distance
                # failure may cost time, but it must never suppress it.
                exact_distance_mm = 0.0
            if exact_distance_mm > 1.0e-7:
                continue
            exact_boolean_candidate_count += 1
            if follower_pair:
                follower_exact_boolean_candidate_count += 1
            common = first.shape & second.shape
            common_volume = (
                0.0 if common is None else float(common.volume)
            )
            if common_volume <= 1.0e-7:
                continue
            row = {
                "first_group": first.group,
                "first_label": first.label,
                "second_group": second.group,
                "second_label": second.label,
                "common_volume_mm3": common_volume,
            }
            full_overlaps.append(row)
            if follower_pair:
                follower_overlaps.append(row)

    def scope(
        rows, count, bbox_candidates, distance_candidates,
        boolean_candidates, leaf_count,
    ):
        return {
            "leaf_count": leaf_count,
            "pair_count": count,
            "bbox_candidate_count": bbox_candidates,
            "exact_distance_candidate_count": distance_candidates,
            "exact_common_boolean_count": boolean_candidates,
            "positive_overlap_count": len(rows),
            "positive_common_volume_mm3": sum(
                row["common_volume_mm3"] for row in rows
            ),
            "positive_overlaps": rows,
            "status": "PASS_ZERO_POSITIVE" if not rows else "FAIL_POSITIVE",
        }

    return {
        "geometry_signature": signature,
        "selected_occurrence_index": (
            None if selected_occurrence_index == -1
            else selected_occurrence_index
        ),
        "follower_carrier_scope": scope(
            follower_overlaps,
            follower_pair_count,
            follower_bbox_candidate_count,
            follower_exact_distance_candidate_count,
            follower_exact_boolean_candidate_count,
            follower_leaf_count,
        ),
        "complete_installed_scope": scope(
            full_overlaps,
            pair_count,
            bbox_candidate_count,
            exact_distance_candidate_count,
            exact_boolean_candidate_count,
            len(leaves),
        ),
    }


def manufactured_leaf_pair_audit_for_state(
    m1_law: str,
    m2_track: int,
    m0_gate_state: str,
) -> dict[str, Any]:
    states = selected_occurrences(m1_law, m2_track, m0_gate_state)
    selected = [state["index"] for state in states if state["selected"]]
    signature_index = selected[0] if selected else -1
    geometry = _manufactured_pair_audit_geometry(signature_index)
    return {
        "m1_law": m1_law,
        "m2_track_index": m2_track,
        "m0_gate_state": m0_gate_state,
        "selected_occurrence_index": selected[0] if selected else None,
        **geometry,
    }


@lru_cache(maxsize=1)
def all_selector_state_pair_audit() -> dict[str, Any]:
    states = []
    for gate in M0_GATE_STATES:
        for law in M1_LAWS:
            for track in range(MOVING_OCCURRENCE_COUNT):
                states.append(manufactured_leaf_pair_audit_for_state(
                    law, track, gate,
                ))
    follower_failures = [
        state for state in states
        if state["follower_carrier_scope"]["positive_overlap_count"]
    ]
    complete_failures = [
        state for state in states
        if state["complete_installed_scope"]["positive_overlap_count"]
    ]
    return {
        "schema": "replacement-carriage-manufactured-pair-audit/v1",
        "state_count": len(states),
        "engaged_state_count": sum(
            state["m0_gate_state"] == "ENGAGED_LOCKED" for state in states
        ),
        "all_parked_state_count": sum(
            state["m0_gate_state"] != "ENGAGED_LOCKED" for state in states
        ),
        "unique_geometry_signature_count": len({
            state["geometry_signature"] for state in states
        }),
        "blocker_envelopes_excluded_as_non_manufactured": COARSE_BLOCKER_COUNT,
        "full_pivot_hardware_included": True,
        "primary_mount_hardware_included": True,
        "follower_carrier_failure_state_count": len(follower_failures),
        "complete_installed_failure_state_count": len(complete_failures),
        "all_follower_carrier_states_zero_positive": not follower_failures,
        "all_complete_installed_states_zero_positive": not complete_failures,
        "clearance_authority": False,
        "states": states,
    }


def reference_parts() -> tuple:
    """Leaf geometry for the all-parked/all-retracted reference STEP."""

    return (
        replacement_carrier(),
        *(
            moving_occurrence(identity)
            for identity in OCCURRENCE_IDENTITIES
        ),
        *primary_tower_m4_hardware(),
        *(
            coarse_selection_linkage_blocker(identity)
            for identity in OCCURRENCE_IDENTITIES
        ),
    )


def _leaf_solids(shape) -> list:
    return list(shape.solids())


def geometry_contract() -> dict[str, Any]:
    return {
        "model": "aggregate_boundary_follower_replacement_carriage",
        "frame": {
            "local_axes": {
                "+X": "radial_outward",
                "+Y": "tangential",
                "+Z": "stator_axis",
            },
            "M0_home_transform": "machine=(-local_y,local_z,95-local_x)",
            "owner": "M0_carriage",
            "M1_spatial_transform": False,
            "M2_spatial_transform": False,
        },
        "carrier": {
            "count": 1,
            "material": "6061-T6 aluminum",
            "source_corridor": (
                "carriage_active_sector_terminal_guide.carriage_yoke"
            ),
            "one_solid_required": True,
            "tower_adapter_local_bounds_mm": {
                "x": [26.0, 38.0],
                "y": [-28.0, 28.0],
                "z": [-114.0, -110.0],
            },
            "U_window_local_bounds_mm": {
                "x": [25.0, 30.5],
                "y": [-17.5, 17.5],
                "z": [-114.5, -109.5],
            },
            "legacy_guide_M3_holes_present": False,
            "legacy_guide_key_pockets_functionally_present": False,
            "legacy_guide_key_fill_subsumed_by_clearance_relief": True,
            "integral_selection_bay_count": 4,
            "parked_follower_relief_bounds_local_mm": {
                "x": [
                    PARKED_FOLLOWER_RELIEF_X_MIN_MM,
                    PARKED_FOLLOWER_RELIEF_X_MAX_MM,
                ],
                "abs_y": [
                    PARKED_FOLLOWER_RELIEF_TANGENTIAL_INNER_MM,
                    PARKED_FOLLOWER_RELIEF_TANGENTIAL_OUTER_MM,
                ],
                "abs_z": [
                    PARKED_FOLLOWER_RELIEF_AXIAL_INNER_MM,
                    PARKED_FOLLOWER_RELIEF_AXIAL_OUTER_MM,
                ],
            },
            "selection_wall_abs_z_bounds_mm": [
                SELECTION_WALL_AXIAL_INNER_MM,
                SELECTION_WALL_AXIAL_OUTER_MM,
            ],
            "selection_bay_tangential_clearance_mm": (
                SELECTION_BAY_TANGENTIAL_CLEARANCE_MM
            ),
            "outboard_dogleg_web_min_radial_thickness_mm": (
                active.YOKE_GUIDE_CONNECTOR_X_MAX_MM
                - PARKED_FOLLOWER_RELIEF_X_MAX_MM
            ),
            "original_same_side_6mm_pitch_M4_holes_filled": True,
            "diagonal_primary_M4_local_locations_mm": [
                list(point) for point in _primary_m4_local_locations()
            ],
        },
        "occurrences": {
            "count": MOVING_OCCURRENCE_COUNT,
            "identities": [asdict(item) for item in OCCURRENCE_IDENTITIES],
            "custom_bodies_per_occurrence": (
                MOVING_CUSTOM_BODY_COUNT_PER_OCCURRENCE
            ),
            "pivot_hardware_leaves_per_occurrence": (
                MOVING_PIVOT_LEAF_COUNT_PER_OCCURRENCE
            ),
            "leaf_solids_per_occurrence": MOVING_LEAF_COUNT_PER_OCCURRENCE,
            "reference_pose": "all_parked_and_radially_retracted",
            "canonical_source_nose_retracted_mm": [30.0, 0.0, 24.0],
            "provisional_source_map": (
                "X=x-0.30; Y=t*Ybase+t*y; "
                "Z=a*21.35+a*(z-24)"
            ),
            "outer_pivot_catalog_stack": {
                "pin_sku": OUTER_PIVOT_PIN_SKU,
                "included_ring_sku": OUTER_PIVOT_RING_SKU,
                "pin_diameter_mm": OUTER_PIVOT_PIN_DIAMETER_MM,
                "pin_length_mm": OUTER_PIVOT_PIN_LENGTH_MM,
                "ring_thickness_mm": OUTER_PIVOT_RING_THICKNESS_MM,
                "ring_radial_envelope_modeled_as": (
                    "conservative_closed_annulus"
                ),
                "DIN988_internal_shim_count": 2,
                "inward_screw_or_nyloc_projection": False,
                "catalog_source": (
                    "https://us.misumi-ec.com/vona2/detail/110300095320/"
                ),
            },
        },
        "selection": {
            "M1_laws": list(M1_LAWS),
            "M2_track_radii_mm": list(M2_TRACK_RADII_MM),
            "M0_gate_states": list(M0_GATE_STATES),
            "direct_track_to_occurrence": [0, 1, 2, 3],
            "reverse_zero_track_to_occurrence": [0, 1, 2, 3],
            "reverse_180_track_to_occurrence": [2, 3, 0, 1],
            "direct_pulse_angles_deg": list(DIRECT_PULSE_ANGLES_DEG),
            "reverse_pulse_angles_deg": list(REVERSE_PULSE_ANGLES_DEG),
            "active_base_abs_y_mm": COARSE_ACTIVE_BASE_MM,
            "parked_base_abs_y_mm": COARSE_PARKED_BASE_MM,
            "coarse_selection_travel_mm": COARSE_SELECTION_TRAVEL_MM,
            "passive_tangential_usable_mm": [
                -PASSIVE_TANGENTIAL_USABLE_MM,
                PASSIVE_TANGENTIAL_USABLE_MM,
            ],
            "coarse_linkage_geometry_mode": "BLOCKER_ENVELOPE_ONLY",
            "coarse_blocker_count": COARSE_BLOCKER_COUNT,
            "engaged_opposed_center_spacing_mm": (
                ENGAGED_OPPOSED_CENTER_SPACING_MM
            ),
            "engaged_yoke_body_clearance_mm": (
                ENGAGED_YOKE_BODY_CLEARANCE_MM
            ),
            "engaged_complete_outer_pivot_envelope_clearance_mm": (
                ENGAGED_OUTER_PIVOT_STACK_CLEARANCE_MM
            ),
            "inward_q_complete_outer_pivot_envelope_clearance_mm": (
                ENGAGED_OUTER_PIVOT_STACK_CLEARANCE_MM
                - PASSIVE_TANGENTIAL_USABLE_MM
            ),
            "nominal_reserve_above_2mm_requirement_mm": (
                ENGAGED_OUTER_PIVOT_STACK_CLEARANCE_MM
                - PASSIVE_TANGENTIAL_USABLE_MM
                - 2.0
            ),
        },
        "radial": {
            "hard_center_range_mm": [
                RADIAL_HARD_CENTER_MIN_MM, RADIAL_HARD_CENTER_MAX_MM,
            ],
            "reference_retracted_center_mm": (
                follower.RADIAL_CENTER_RETRACTED_MM
            ),
            "stroke_mm": follower.RADIAL_STROKE_MM,
        },
        "fasteners": {
            "primary_tower_M4_stack_count": PRIMARY_M4_STACK_COUNT,
            "primary_tower_M4_leaf_count": PRIMARY_M4_LEAF_COUNT,
            "duplicate_primary_M4_stack_present": False,
            "legacy_guide_M3_hardware_count": 0,
            "primary_M4_screw_sku": PRIMARY_M4_SCREW_SKU,
            "primary_M4_insert_sku": PRIMARY_M4_INSERT_SKU,
            "primary_M4_washer_count": 0,
            "primary_M4_locking_feature": "factory_nylon_patch",
            "primary_M4_recessed_head_diameter_mm": (
                PRIMARY_M4_HEAD_DIAMETER_MM
            ),
            "primary_M4_recessed_head_height_mm": (
                PRIMARY_M4_HEAD_HEIGHT_MM
            ),
            "primary_M4_same_side_diagonal_center_distance_mm": (
                PRIMARY_M4_DIAGONAL_CENTER_DISTANCE_MM
            ),
            "proof_basis_X_row_span_mm": 6.0,
            "proof_basis_X_row_span_preserved": True,
            "key_locations_unchanged": True,
            "catalog_source": (
                "https://www.nbk1560.com/en-US/products/specialscrew/"
                "nedzicom/miniaturescrew/SSHS-SD-ALK/SSHS-M4-SD-ALK/"
                "SSHS-M4-10-SD-ALK/"
            ),
        },
        "exact_counts": {
            "carrier_leaf_solids": 1,
            "moving_occurrence_count": MOVING_OCCURRENCE_COUNT,
            "moving_leaf_solids": (
                MOVING_OCCURRENCE_COUNT
                * MOVING_LEAF_COUNT_PER_OCCURRENCE
            ),
            "primary_M4_leaf_solids": PRIMARY_M4_LEAF_COUNT,
            "coarse_blocker_leaf_solids": COARSE_BLOCKER_COUNT,
            "total_leaf_solids": EXPECTED_LEAF_SOLID_COUNT,
        },
        "forbidden_content": [
            "old_PEEK_active_sector_guides",
            "old_guide_M3_hardware_or_holes",
            "aggregate_boundary_floating_follower.carrier",
            "aggregate_boundary_floating_follower.mounting_backer_context",
            "integrated_machine_context",
            "duplicate_primary_tower_M4_stack",
        ],
        "rejected_overlap_witnesses": {
            "inward_outer_pivot_nyloc_stack": dict(
                REJECTED_INWARD_NYLOC_WITNESS
            ),
            "six_mm_pitch_primary_M4_stack": dict(
                REJECTED_PRIMARY_M4_WITNESS
            ),
        },
        "authority": dict(AUTHORITY),
        "blockers": list(BLOCKERS),
    }


def manifest(step_path: Path | str = STEP_OUT) -> dict[str, Any]:
    step = Path(step_path)
    payload = geometry_contract()
    payload["manufactured_leaf_pair_audit"] = all_selector_state_pair_audit()
    payload["artifacts"] = {
        "source": str(Path(__file__).resolve()),
        "step": str(step.resolve()),
        "step_exists": step.exists(),
        "step_size_bytes": step.stat().st_size if step.exists() else None,
        "step_sha256": (
            hashlib.sha256(step.read_bytes()).hexdigest()
            if step.exists() else None
        ),
    }
    return payload


def write_manifest(
    path: Path | str = MANIFEST_OUT,
    step_path: Path | str = STEP_OUT,
) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(manifest(step_path), indent=2) + "\n",
        encoding="utf-8",
    )
    return target


def gen_step() -> Compound:
    carrier = replacement_carrier()
    occurrences = [
        moving_occurrence(identity)
        for identity in OCCURRENCE_IDENTITIES
    ]
    occurrence_group = Compound(children=occurrences)
    occurrence_group.label = "four_handed_M0_carriage_owned_followers"

    m4_group = Compound(children=list(primary_tower_m4_hardware()))
    m4_group.label = "one_shared_primary_tower_M4_stack_set"

    blocker_group = Compound(children=[
        coarse_selection_linkage_blocker(identity)
        for identity in OCCURRENCE_IDENTITIES
    ])
    blocker_group.label = (
        "BLOCKER_ONLY_four_8p90mm_coarse_selection_linkage_envelopes"
    )

    result = Compound(children=[
        carrier,
        occurrence_group,
        m4_group,
        blocker_group,
    ])
    result.label = "aggregate_boundary_follower_replacement_carriage_REVIEW_ONLY"
    solids = list(result.solids())
    if len(solids) != EXPECTED_LEAF_SOLID_COUNT:
        raise RuntimeError(
            f"expected {EXPECTED_LEAF_SOLID_COUNT} leaf solids, "
            f"got {len(solids)}"
        )
    if any(float(solid.volume) <= 0.0 for solid in solids):
        raise RuntimeError("all exported leaf solids must have positive volume")
    return result


if __name__ == "__main__":
    from build123d import export_step

    STEP_OUT.parent.mkdir(parents=True, exist_ok=True)
    export_step(gen_step(), str(STEP_OUT))
    write_manifest()
    print(STEP_OUT)
    print(MANIFEST_OUT)
