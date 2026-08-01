"""Fail-closed geometry review for the exact-1:1 M2 drive successor.

The current McMaster 6627T421 / 40T-2GT-6 drive does not retain the required
2x torque margin at the conservative GOAL OD65 wire-force bound.  This module
does *not* silently approve a replacement.  It proves the packaging of the
smallest orderable exact-1:1 successor while leaving the motor running-torque
gate open until the manufacturer curve is extracted at 300 RPM.

Candidate hardware
------------------
* StepperOnline 17HS24-2004D-E1K closed-loop NEMA17, the strongest published
  drop-in frame candidate found for the normal GOAL.  The review uses its
  manufacturer-dimensioned cable-free envelope; its exact STEP and dynamic
  torque curve remain explicit acquisition gates.
* 2x NBK P30-3GT-BLP-6C, 30 teeth, 3 mm pitch, 6 mm belt.  Motor pulley is the
  stock 5 mm bore split clamp-hub with its supplied M2 clamp bolt; it is not a
  radial set-screw/D-flat part.  The flyer-side pulley is an equivalent
  dimensioned custom envelope around the existing 12 mm hollow shaft.
* 210-3GT-6 belt.  Equal 30T pulleys at the frozen 60 mm center distance give
  exactly 210 mm pitch length and preserve the upstream 1:1 radians contract.

The STEP intentionally includes the actual static wire tube and nearby frame
context.  Belt/pulley mesh is an intended contact; every other reported pair
must remain separated.  The pulley solids are supplier-dimensioned collision
envelopes (teeth are not cosmetic BREP detail), matching the project's stated
simplified-belt scope.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from build123d import Align, Box, Compound, Cylinder, Part, Pos, Rot, export_step

import assembly
import cots
import hardware
from params import PARAMS as P
import printed
import wire_vis


ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out"
REVIEW = OUT / "review"
REPORTS = OUT / "reports"
FORCE_REPORT = REPORTS / "permanent_cap_offset_spoke_wire_force_torque.json"

MOTOR_MODEL = "17HS24-2004D-E1K"
MOTOR_STEP = Path(__file__).resolve().parent / "models" / "upgrades" / (
    MOTOR_MODEL + ".step"
)
MOTOR_PRODUCT_URL = (
    "https://www.stepperonline.ca/"
    "nema-17-closed-loop-stepper-motor-80ncm-113-29oz-in-with-encoder-"
    "1000ppr-4000cpr-17hs24-2004d-e1k.html"
)
MOTOR_STEP_URL = (
    "https://www.omc-stepperonline.com/index.php?route=product/product/"
    "get_file&file=3166/17HS24-2004D-E1K.STEP"
)
MOTOR_TORQUE_CURVE_URL = (
    "https://www.omc-stepperonline.com/index.php?route=product/product/"
    "get_file&file=3166/17HS24-2004D-E1K_Torque_Curve.pdf"
)

PULLEY_MODEL_MOTOR = "P30-3GT-BLP-6C-5"
PULLEY_MODEL_FLYER = "P30-3GT-BLP-6C-12.05-interface"
PULLEY_URL = (
    "https://www.nbk1560.com/products/pulley/timingpulley/"
    "3GT-BLP-6C/P30-3GT-BLP-6C/"
)
GATES_PM_MANUAL_URL = (
    "https://www.gates.com/content/dam/gates/home/knowledge-center/"
    "resource-library/operating-manuals/preventive-maintenance-manual-en.pdf"
)
GATES_CATALOG_URL = (
    "https://www.gates.com/content/dam/documents-library/catalogs/"
    "industrial-power-transmission-catalogue-en.pdf"
)

MOTOR_TEETH = 30
FLYER_TEETH = 30
PITCH_MM = 3.0
PITCH_DIAMETER_MM = 28.7
TOOTH_OD_MM = 27.9
FLANGE_OD_MM = 32.0
PULLEY_OVERALL_W_MM = 11.0
PULLEY_CHANNEL_W_MM = 7.3
PULLEY_FLANGE_T_MM = (PULLEY_OVERALL_W_MM - PULLEY_CHANNEL_W_MM) / 2.0
NBK_CLAMP_HUB_DB_MM = 20.0
NBK_CLAMP_ENVELOPE_E_MM = 23.0
NBK_CLAMP_HUB_L_MM = 7.5
NBK_CLAMP_BOLT_F_MM = 2.75
NBK_CLAMP_BOLT_G_MM = 7.5
NBK_CLAMP_BOLT_SIZE = "M2"
NBK_CLAMP_BOLT_TIGHTENING_NM = 0.5
NBK_MOTOR_PULLEY_MASS_G = 28.0
NBK_MOTOR_PULLEY_INERTIA_KGM2 = 3.0e-6
# The primary table/drawing identifies a tangential cap-screw clamp but the
# exact CAD is not local.  M2x12 is a conservative visible witness inside the
# published E23 clamp envelope, not a released fastener-length selection.
NBK_CLAMP_BOLT_WITNESS_LENGTH_MM = 12.0
FLYER_COLLAR_OD_MM = 23.0
FLYER_COLLAR_LENGTH_MM = 6.0
FLYER_SET_SCREW_LENGTH_MM = 6.0
FLYER_SET_SCREW_PROUD_MM = 0.2
FLYER_SHAFT_FLAT_RADIUS_MM = 5.7
BELT_WIDTH_MM = 6.0
# Gates gives the 3MGT total height and tooth height.  For this selected NBK
# pulley, the installed pitch-line-to-pulley-OD offset is derived directly
# from its Dp=28.7 and De=27.9 dimensions: (Dp-De)/2 = 0.400 mm.  Therefore the
# physical belt envelope is asymmetric about the pitch line at the wrap:
# 1.520 mm inward to the tooth tips and 0.890 mm outward to the backing.
BELT_TOTAL_HEIGHT_MM = 2.41
BELT_TOOTH_HEIGHT_MM = 1.12
PITCH_TO_PULLEY_OD_MM = (PITCH_DIAMETER_MM - TOOTH_OD_MM) / 2.0
BELT_INWARD_FROM_PITCH_MM = PITCH_TO_PULLEY_OD_MM + BELT_TOOTH_HEIGHT_MM
BELT_OUTWARD_FROM_PITCH_MM = (
    BELT_TOTAL_HEIGHT_MM - BELT_INWARD_FROM_PITCH_MM
)
CENTER_DISTANCE_MM = 60.0
BELT_PITCH_LENGTH_MM = 210.0
BELT_MODEL = "210-3GT-6"
BELT_LENGTH_FACTOR = 0.9  # NBK: 191..260 mm
BELT_MESH_FACTOR = 1.0    # equal pulleys engage far more than six teeth
PULLEY_BASE_TORQUE_300RPM_NM = 2.06
PULLEY_ALLOWABLE_TORQUE_300RPM_NM = (
    PULLEY_BASE_TORQUE_300RPM_NM * BELT_LENGTH_FACTOR * BELT_MESH_FACTOR
)

MOTOR_AXIS_Y = -60.0
MOTOR_FACE_Z = -102.0
# The 11 mm P30 is 0.7 mm wider than the released 10.3 mm P40.  Moving the
# complete pulley/belt plane 0.5 mm toward the flyer shares the available
# axial gap instead of leaving the motor pulley exactly at the 2.0 mm gate.
DRIVE_PLANE_SHIFT_MM = 0.75
PULLEY_CENTER_Z = sum(P.pulley_z) / 2.0 + DRIVE_PLANE_SHIFT_MM
PULLEY_REAR_Z = PULLEY_CENTER_Z - PULLEY_OVERALL_W_MM / 2.0
FLYER_COLLAR_Z0 = PULLEY_REAR_Z - FLYER_COLLAR_LENGTH_MM
FLYER_COLLAR_CENTER_Z = FLYER_COLLAR_Z0 + FLYER_COLLAR_LENGTH_MM / 2.0
FLYER_COLLAR_RADIAL_ENGAGEMENT_MM = (
    FLYER_COLLAR_OD_MM / 2.0 - FLYER_SHAFT_FLAT_RADIUS_MM
)
FLYER_COLLAR_TAPPED_WALL_MM = (
    FLYER_COLLAR_OD_MM / 2.0 - 12.05 / 2.0
)
MOUNT_Z0 = MOTOR_FACE_Z
MOUNT_Z1 = MOTOR_FACE_Z + 6.0
BOOLEAN_TOL_MM3 = 1.0e-5
MIN_RUNNING_CLEARANCE_MM = 2.0


def _box(x0: float, x1: float, y0: float, y1: float,
         z0: float, z1: float) -> Part:
    return Pos(x0, y0, z0) * Box(
        x1 - x0, y1 - y0, z1 - z0,
        align=(Align.MIN, Align.MIN, Align.MIN),
    )


def _at(part: Part, x: float = 0.0, y: float = 0.0, z: float = 0.0,
        rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
        label: str | None = None) -> Part:
    result = Pos(x, y, z) * (Rot(rx, ry, rz) * part)
    result.label = label or getattr(part, "label", "part")
    return result


def successor_motor() -> Part:
    """Manufacturer-dimensioned cable-free NEMA17 candidate envelope.

    Local mounting face is z=0, body/encoder extends 80 mm toward -Z, the
    register is OD22 x 2, and the 5 mm D shaft extends 24 mm toward +Z.
    """
    body = Box(
        42.3, 42.3, 80.0,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    )
    boss = Cylinder(
        11.0, 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    shaft = Cylinder(
        2.5, 24.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    envelope = body + boss + shaft
    result = _at(
        envelope, 0.0, MOTOR_AXIS_Y, MOTOR_FACE_Z,
        label="m2_successor_17HS24_2004D_E1K",
    )
    return result


def successor_motor_mount() -> Part:
    """Existing validated NEMA17 post-spanning tensioning bracket."""
    result = printed.m2_motor_mount()
    result.label = "m2_successor_nema17_mount"
    return result


def successor_pulley(
    bore_d_mm: float,
    label: str,
    *,
    stock_clamp_witness: bool = False,
    flyer_clamp_ports: bool = False,
) -> Part:
    """Supplier-dimensioned P30-3GT-BLP-6C collision envelope."""
    z0 = PULLEY_CENTER_Z - PULLEY_OVERALL_W_MM / 2.0
    channel_z0 = z0 + PULLEY_FLANGE_T_MM
    tooth_band = Pos(0.0, 0.0, channel_z0) * Cylinder(
        TOOTH_OD_MM / 2.0,
        PULLEY_CHANNEL_W_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    rear_flange = Pos(0.0, 0.0, z0) * Cylinder(
        FLANGE_OD_MM / 2.0,
        PULLEY_FLANGE_T_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    front_flange = Pos(
        0.0, 0.0, channel_z0 + PULLEY_CHANNEL_W_MM
    ) * Cylinder(
        FLANGE_OD_MM / 2.0,
        PULLEY_FLANGE_T_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    # The NBK split-clamp drawing gives Db20 with a maximum E23 clamp/ear
    # envelope.  The E23 body retained here is conservative for collision;
    # exact split/ear topology remains bound to the missing vendor CAD gate.
    hub = Pos(0.0, 0.0, z0) * Cylinder(
        NBK_CLAMP_ENVELOPE_E_MM / 2.0,
        PULLEY_OVERALL_W_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    bore = Pos(0.0, 0.0, z0 - 1.0) * Cylinder(
        bore_d_mm / 2.0,
        PULLEY_OVERALL_W_MM + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    result = tooth_band + rear_flange + front_flange + hub - bore
    if stock_clamp_witness:
        # Conservative, explicitly inferred placement of the supplier's
        # tangential M2 clamp bolt.  F/G locate the visible review witness;
        # the exact split and counterbore topology must come from vendor CAD.
        bolt_z = z0 + NBK_CLAMP_BOLT_F_MM
        hub_surface_y = math.sqrt(
            (NBK_CLAMP_HUB_DB_MM / 2.0) ** 2
            - NBK_CLAMP_BOLT_G_MM ** 2
        )
        through_port = Pos(
            NBK_CLAMP_BOLT_G_MM, 0.0, bolt_z
        ) * (
            Rot(90.0, 0.0, 0.0) * Cylinder(
                1.05,
                NBK_CLAMP_ENVELOPE_E_MM + 2.0,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            )
        )
        access_length = (
            NBK_CLAMP_ENVELOPE_E_MM / 2.0 + 1.0 - hub_surface_y
        )
        access_mid_y = (
            NBK_CLAMP_ENVELOPE_E_MM / 2.0 + 1.0 + hub_surface_y
        ) / 2.0
        head_access = Pos(
            NBK_CLAMP_BOLT_G_MM, access_mid_y, bolt_z
        ) * (
            Rot(90.0, 0.0, 0.0) * Cylinder(
                2.05,
                access_length,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            )
        )
        split = _box(
            bore_d_mm / 2.0 - 0.2,
            FLANGE_OD_MM / 2.0 + 1.0,
            -0.25,
            0.25,
            z0 - 0.5,
            z0 + PULLEY_OVERALL_W_MM + 0.5,
        )
        result = result - through_port - head_access - split
    if flyer_clamp_ports:
        # Retention belongs on an axial collar behind the rear flange, clear
        # of the toothed/belt channel.  The previous study incorrectly bored
        # radial ports under the loaded belt teeth.  These two tapped-port
        # envelopes run from accessible outside faces to the shaft flats.
        collar_outer = Pos(0.0, 0.0, FLYER_COLLAR_Z0) * Cylinder(
            FLYER_COLLAR_OD_MM / 2.0,
            FLYER_COLLAR_LENGTH_MM,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        collar_bore = Pos(0.0, 0.0, FLYER_COLLAR_Z0 - 1.0) * Cylinder(
            bore_d_mm / 2.0,
            FLYER_COLLAR_LENGTH_MM + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        port_span = FLYER_COLLAR_RADIAL_ENGAGEMENT_MM + 0.4
        port_mid = (
            FLYER_COLLAR_OD_MM / 2.0 + FLYER_SHAFT_FLAT_RADIUS_MM
        ) / 2.0
        port_y = Pos(0.0, -port_mid, FLYER_COLLAR_CENTER_Z) * (
            Rot(90.0, 0.0, 0.0) * Cylinder(
                1.55, port_span,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            )
        )
        port_x = Pos(port_mid, 0.0, FLYER_COLLAR_CENTER_Z) * (
            Rot(0.0, 90.0, 0.0) * Cylinder(
                1.55, port_span,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            )
        )
        result = result + collar_outer - collar_bore - port_y - port_x
    result.label = label
    return result


def successor_belt() -> Part:
    """210-3GT-6 racetrack collision body at the supplier pitch circles."""
    if MOTOR_TEETH != FLYER_TEETH:
        raise ValueError("focused racetrack model requires equal pulley teeth")
    pitch_r = MOTOR_TEETH * PITCH_MM / (2.0 * math.pi)
    outer_r = pitch_r + BELT_OUTWARD_FROM_PITCH_MM
    inner_r = pitch_r - BELT_INWARD_FROM_PITCH_MM
    z0 = PULLEY_CENTER_Z - BELT_WIDTH_MM / 2.0

    def capsule(radius: float) -> Part:
        end_a = Pos(0.0, 0.0, z0) * Cylinder(
            radius, BELT_WIDTH_MM,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        end_b = Pos(0.0, -CENTER_DISTANCE_MM, z0) * Cylinder(
            radius, BELT_WIDTH_MM,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        bridge = _box(
            -radius, radius, -CENTER_DISTANCE_MM, 0.0,
            z0, z0 + BELT_WIDTH_MM,
        )
        return end_a + end_b + bridge

    result = capsule(outer_r) - capsule(inner_r)
    result.label = "m2_successor_210_3gt_6_belt"
    return result


def motor_screws() -> list[Part]:
    """Four M3x10 motor screws with heads on the mount front face."""
    result: list[Part] = []
    half = 31.0 / 2.0
    for ix, x in enumerate((-half, half)):
        for iy, y in enumerate((MOTOR_AXIS_Y - half, MOTOR_AXIS_Y + half)):
            result.append(hardware.place(
                hardware.socket_head_cap_screw("M3", 10.0),
                (x, y, MOUNT_Z1), axis="+z",
                label=f"m2_successor_motor_m3x10_{ix}_{iy}",
            ))
    return result


def motor_pulley_clamp_bolt_witness() -> Part:
    """Visible inferred M2 clamp bolt; exact vendor CAD is still mandatory."""
    hub_surface_y = math.sqrt(
        (NBK_CLAMP_HUB_DB_MM / 2.0) ** 2
        - NBK_CLAMP_BOLT_G_MM ** 2
    )
    return hardware.place(
        hardware.socket_head_cap_screw(
            NBK_CLAMP_BOLT_SIZE,
            NBK_CLAMP_BOLT_WITNESS_LENGTH_MM,
        ),
        (
            NBK_CLAMP_BOLT_G_MM,
            MOTOR_AXIS_Y + hub_surface_y,
            PULLEY_REAR_Z + NBK_CLAMP_BOLT_F_MM,
        ),
        axis="+y",
        label="nbk_P30_stock_m2_clamp_bolt_inferred_witness",
    )


def flyer_pulley_set_screws() -> list[Part]:
    """Two accessible rear-collar M3x6 screws onto machined shaft flats."""
    datum = FLYER_COLLAR_OD_MM / 2.0 + FLYER_SET_SCREW_PROUD_MM
    return [
        hardware.place(
            hardware.set_screw("M3", FLYER_SET_SCREW_LENGTH_MM),
            (0.0, -datum, FLYER_COLLAR_CENTER_Z), axis="-y",
            label="m2_successor_flyer_pulley_m3x6_neg_y",
        ),
        hardware.place(
            hardware.set_screw("M3", FLYER_SET_SCREW_LENGTH_MM),
            (datum, 0.0, FLYER_COLLAR_CENTER_Z), axis="+x",
            label="m2_successor_flyer_pulley_m3x6_pos_x",
        ),
    ]


def successor_shaft_with_pulley_flats() -> Part:
    """Existing hollow shaft with two 0.3 mm pulley-clamp flats."""
    shaft = assembly.alu_tube()
    flat_half_z = 3.0
    shaft -= _box(
        -4.0, 4.0, -6.1, -5.7,
        FLYER_COLLAR_CENTER_Z - flat_half_z,
        FLYER_COLLAR_CENTER_Z + flat_half_z,
    )
    shaft -= _box(
        5.7, 6.1, -4.0, 4.0,
        FLYER_COLLAR_CENTER_Z - flat_half_z,
        FLYER_COLLAR_CENTER_Z + flat_half_z,
    )
    shaft.label = "flyer_hollow_shaft_two_pulley_flats_context"
    return shaft


def context_parts() -> dict[str, Part]:
    """Nearby production geometry needed for the focused fit audit."""
    post_l = _at(
        cots.extrusion_2020(235.0), -P.post_x, -205.0,
        sum(P.post_z) / 2.0, rx=-90.0, label="post_L_context",
    )
    post_r = _at(
        cots.extrusion_2020(235.0), P.post_x, -205.0,
        sum(P.post_z) / 2.0, rx=-90.0, label="post_R_context",
    )
    static_wire = wire_vis.wire_static()
    static_wire.label = "wire_static_exact_finished_diameter"
    shaft = successor_shaft_with_pulley_flats()
    block = printed.flyer_block()
    block.label = "flyer_block_context"
    return {
        "post_l": post_l,
        "post_r": post_r,
        "wire_static": static_wire,
        "shaft": shaft,
        "flyer_block": block,
    }


def review_parts() -> dict[str, Part]:
    context = context_parts()
    screws = motor_screws()
    flyer_clamps = flyer_pulley_set_screws()
    result = {
        "mount": successor_motor_mount(),
        "motor": successor_motor(),
        "motor_pulley": _at(
            successor_pulley(
                5.0,
                "m2_successor_motor_pulley_local",
                stock_clamp_witness=True,
            ),
            0.0, MOTOR_AXIS_Y, 0.0,
            label="m2_successor_P30_3GT_BLP_6C_5",
        ),
        "motor_pulley_clamp_bolt": motor_pulley_clamp_bolt_witness(),
        "flyer_pulley": successor_pulley(
            12.05, "m2_successor_flyer_P30_3GT_6C",
            flyer_clamp_ports=True,
        ),
        "belt": successor_belt(),
        **{f"motor_screw_{index}": screw
           for index, screw in enumerate(screws)},
        **{f"flyer_set_screw_{index}": screw
           for index, screw in enumerate(flyer_clamps)},
        **context,
    }
    return result


def _distance(a: Part, b: Part) -> float:
    return float(a.distance_to(b))


def _overlap(a: Part, b: Part) -> float:
    return float((a & b).volume)


def _sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_audit() -> dict:
    parts = review_parts()
    force = json.loads(FORCE_REPORT.read_text(encoding="utf-8"))
    launch = force["duty_cases"]["GOAL_launch_OD65"]
    required = float(launch["known_load_required_torque_nm"])
    minimum_capacity = float(
        launch["minimum_pulley_and_belt_output_capacity_for_2x_nm"]
    )

    ratio = FLYER_TEETH / MOTOR_TEETH
    pitch_length = (
        2.0 * CENTER_DISTANCE_MM
        + (MOTOR_TEETH + FLYER_TEETH) * PITCH_MM / 2.0
    )
    motor_screw_keys = sorted(
        key for key in parts if key.startswith("motor_screw_")
    )
    flyer_set_screw_keys = sorted(
        key for key in parts if key.startswith("flyer_set_screw_")
    )
    clearances = {
        "wire_to_candidate_motor_mm": _distance(parts["wire_static"], parts["motor"]),
        "wire_to_belt_mm": _distance(parts["wire_static"], parts["belt"]),
        "wire_to_motor_pulley_mm": _distance(
            parts["wire_static"], parts["motor_pulley"]
        ),
        "wire_to_flyer_pulley_mm": _distance(
            parts["wire_static"], parts["flyer_pulley"]
        ),
        "belt_to_mount_mm": _distance(parts["belt"], parts["mount"]),
        "motor_pulley_to_mount_mm": _distance(
            parts["motor_pulley"], parts["mount"]
        ),
        "flyer_pulley_to_block_mm": _distance(
            parts["flyer_pulley"], parts["flyer_block"]
        ),
        "belt_to_block_mm": _distance(parts["belt"], parts["flyer_block"]),
        "motor_to_left_post_mm": _distance(parts["motor"], parts["post_l"]),
        "motor_to_right_post_mm": _distance(parts["motor"], parts["post_r"]),
        "belt_to_closest_motor_screw_mm": min(
            _distance(parts["belt"], parts[key]) for key in motor_screw_keys
        ),
        "belt_to_closest_flyer_set_screw_mm": min(
            _distance(parts["belt"], parts[key])
            for key in flyer_set_screw_keys
        ),
        "flyer_set_screw_to_mount_min_mm": min(
            _distance(parts[key], parts["mount"])
            for key in flyer_set_screw_keys
        ),
    }
    unintended_overlaps = {
        "wire_vs_motor_mm3": _overlap(parts["wire_static"], parts["motor"]),
        "wire_vs_belt_mm3": _overlap(parts["wire_static"], parts["belt"]),
        "wire_vs_motor_pulley_mm3": _overlap(
            parts["wire_static"], parts["motor_pulley"]
        ),
        "wire_vs_flyer_pulley_mm3": _overlap(
            parts["wire_static"], parts["flyer_pulley"]
        ),
        "belt_vs_mount_mm3": _overlap(parts["belt"], parts["mount"]),
        "motor_pulley_vs_mount_mm3": _overlap(
            parts["motor_pulley"], parts["mount"]
        ),
        "flyer_pulley_vs_block_mm3": _overlap(
            parts["flyer_pulley"], parts["flyer_block"]
        ),
        "belt_vs_block_mm3": _overlap(parts["belt"], parts["flyer_block"]),
        "belt_vs_motor_screws_max_mm3": max(
            _overlap(parts["belt"], parts[key]) for key in motor_screw_keys
        ),
        "belt_vs_flyer_set_screws_max_mm3": max(
            _overlap(parts["belt"], parts[key])
            for key in flyer_set_screw_keys
        ),
        "flyer_set_screws_vs_pulley_max_mm3": max(
            _overlap(parts[key], parts["flyer_pulley"])
            for key in flyer_set_screw_keys
        ),
        "motor_clamp_bolt_vs_belt_mm3": _overlap(
            parts["motor_pulley_clamp_bolt"], parts["belt"]
        ),
        "motor_clamp_bolt_vs_mount_mm3": _overlap(
            parts["motor_pulley_clamp_bolt"], parts["mount"]
        ),
        "motor_clamp_bolt_vs_wire_mm3": _overlap(
            parts["motor_pulley_clamp_bolt"], parts["wire_static"]
        ),
        "motor_clamp_bolt_vs_pulley_mm3": _overlap(
            parts["motor_pulley_clamp_bolt"], parts["motor_pulley"]
        ),
    }
    intended_contacts = {
        "belt_vs_motor_pulley_mm3": _overlap(
            parts["belt"], parts["motor_pulley"]
        ),
        "belt_vs_flyer_pulley_mm3": _overlap(
            parts["belt"], parts["flyer_pulley"]
        ),
        "flyer_pulley_vs_shaft_mm3": _overlap(
            parts["flyer_pulley"], parts["shaft"]
        ),
        "flyer_set_screw_to_flat_distances_mm": [
            _distance(parts[key], parts["shaft"])
            for key in flyer_set_screw_keys
        ],
        "motor_clamp_bolt_to_inferred_port_mm": _distance(
            parts["motor_pulley_clamp_bolt"], parts["motor_pulley"]
        ),
    }
    geometry_checks = {
        "normal_GOAL_nema17_frame_contract": (
            MOTOR_MODEL.startswith("17")
            and math.isclose(parts["motor"].bounding_box().size.X, 42.3,
                             abs_tol=1.0e-6)
            and math.isclose(parts["motor"].bounding_box().size.Y, 42.3,
                             abs_tol=1.0e-6)
        ),
        "exact_1_to_1_ratio": (
            MOTOR_TEETH == FLYER_TEETH
            and math.isclose(ratio, 1.0, abs_tol=1.0e-12)
            and math.isclose(P.m2_gear_ratio, ratio, abs_tol=1.0e-12)
        ),
        "exact_210mm_pitch_length": math.isclose(
            pitch_length, BELT_PITCH_LENGTH_MM, abs_tol=1.0e-9
        ),
        "supplier_belt_envelope_is_asymmetric_about_pitch_line": (
            math.isclose(PITCH_TO_PULLEY_OD_MM, 0.400, abs_tol=1.0e-9)
            and math.isclose(BELT_INWARD_FROM_PITCH_MM, 1.520,
                             abs_tol=1.0e-9)
            and math.isclose(BELT_OUTWARD_FROM_PITCH_MM, 0.890,
                             abs_tol=1.0e-9)
            and math.isclose(
                BELT_INWARD_FROM_PITCH_MM + BELT_OUTWARD_FROM_PITCH_MM,
                BELT_TOTAL_HEIGHT_MM,
                abs_tol=1.0e-9,
            )
        ),
        "mount_is_one_solid": len(parts["mount"].solids()) == 1,
        "belt_is_one_solid": len(parts["belt"].solids()) == 1,
        "motor_pulley_is_one_solid": len(parts["motor_pulley"].solids()) == 1,
        "flyer_pulley_is_one_solid": len(parts["flyer_pulley"].solids()) == 1,
        "stock_motor_pulley_clamp_bolt_witness_is_present": (
            parts["motor_pulley_clamp_bolt"].volume > 0.0
            and parts["motor_pulley_clamp_bolt"].is_valid
            and unintended_overlaps["motor_clamp_bolt_vs_belt_mm3"]
            <= BOOLEAN_TOL_MM3
        ),
        "all_unintended_overlap_is_zero": max(unintended_overlaps.values()) <= BOOLEAN_TOL_MM3,
        "all_running_clearances_ge_2mm": min(clearances.values()) >= MIN_RUNNING_CLEARANCE_MM - 1.0e-6,
        "belt_mesh_contacts_both_pulleys": min(
            intended_contacts["belt_vs_motor_pulley_mm3"],
            intended_contacts["belt_vs_flyer_pulley_mm3"],
        ) > BOOLEAN_TOL_MM3,
        "pulley_capacity_ge_2x_requirement": (
            PULLEY_ALLOWABLE_TORQUE_300RPM_NM >= minimum_capacity
        ),
        "custom_flyer_pulley_has_two_positive_clamp_screws": (
            len(flyer_set_screw_keys) == 2
            and max(intended_contacts[
                "flyer_set_screw_to_flat_distances_mm"
            ]) <= 1.0e-6
            and unintended_overlaps[
                "flyer_set_screws_vs_pulley_max_mm3"
            ] <= BOOLEAN_TOL_MM3
            and clearances[
                "belt_to_closest_flyer_set_screw_mm"
            ] >= MIN_RUNNING_CLEARANCE_MM
            and FLYER_COLLAR_CENTER_Z < PULLEY_REAR_Z
            and FLYER_COLLAR_RADIAL_ENGAGEMENT_MM >= 4.5
        ),
    }
    geometry_pass = all(geometry_checks.values())
    motor_curve_at_300rpm_nm = None
    motor_curve_gate = False
    motor_pulley_vendor_cad_gate = False
    motor_pulley_to_d_shaft_gate = False
    flyer_pulley_retention_torque_gate = False
    status = (
        "GEOMETRY_PASS_INTERFACE_MOTOR_AND_HUB_GATES_OPEN" if geometry_pass
        else "GEOMETRY_FAIL_INTERFACE_MOTOR_AND_HUB_GATES_OPEN"
    )
    return {
        "schema": "m2-drive-successor-review/v1",
        "status": status,
        "production_authorized": False,
        "geometry_passed": geometry_pass,
        "motor_running_torque_gate_passed": motor_curve_gate,
        "motor_pulley_vendor_cad_gate_passed": motor_pulley_vendor_cad_gate,
        "motor_pulley_to_motor_shaft_gate_passed": (
            motor_pulley_to_d_shaft_gate
        ),
        "flyer_pulley_retention_torque_gate_passed": (
            flyer_pulley_retention_torque_gate
        ),
        "coordinate_frame": "machine reference pose; millimetres; M2=0",
        "candidate": {
            "motor": {
                "model": MOTOR_MODEL,
                "holding_torque_nm": 0.8,
                "rated_current_A": 2.0,
                "body_mm": [42.3, 42.3, 80.0],
                "shaft_mm": [5.0, 24.0],
                "shaft_form": "15 mm D-cut on 24 mm protrusion",
                "shaft_flat_depth_mm": None,
                "encoder_ppr": 1000,
                "local_step": str(MOTOR_STEP.relative_to(ROOT)).replace("\\", "/"),
                "local_step_present": MOTOR_STEP.exists(),
                "local_step_sha256": _sha256(MOTOR_STEP),
                "product_url": MOTOR_PRODUCT_URL,
                "step_url": MOTOR_STEP_URL,
                "torque_curve_url": MOTOR_TORQUE_CURVE_URL,
                "available_torque_at_300rpm_nm": motor_curve_at_300rpm_nm,
                "gate": (
                    "OPEN: manufacturer curve must prove >= "
                    f"{minimum_capacity:.9f} Nm at 300 RPM at the selected "
                    "driver voltage/current, including rotor inertia in the "
                    "recomputed acceleration load"
                ),
            },
            "drive": {
                "motor_pulley": PULLEY_MODEL_MOTOR,
                "flyer_pulley": PULLEY_MODEL_FLYER,
                "pulley_url": PULLEY_URL,
                "teeth_each": (
                    MOTOR_TEETH if MOTOR_TEETH == FLYER_TEETH else None
                ),
                "motor_teeth": MOTOR_TEETH,
                "flyer_teeth": FLYER_TEETH,
                "pitch_mm": PITCH_MM,
                "pitch_diameter_mm": PITCH_DIAMETER_MM,
                "flange_od_mm": FLANGE_OD_MM,
                "overall_width_mm": PULLEY_OVERALL_W_MM,
                "belt_channel_width_mm": PULLEY_CHANNEL_W_MM,
                "belt": BELT_MODEL,
                "belt_width_mm": BELT_WIDTH_MM,
                "belt_total_height_mm": BELT_TOTAL_HEIGHT_MM,
                "belt_tooth_height_mm": BELT_TOOTH_HEIGHT_MM,
                "selected_pulley_pitch_to_od_radial_mm": (
                    PITCH_TO_PULLEY_OD_MM
                ),
                "belt_inward_from_pitch_line_mm": (
                    BELT_INWARD_FROM_PITCH_MM
                ),
                "belt_outward_from_pitch_line_mm": (
                    BELT_OUTWARD_FROM_PITCH_MM
                ),
                "belt_envelope_derivation": (
                    "Gates total height 2.41 and tooth height 1.12; "
                    "selected NBK P30 Dp/De gives (28.7-27.9)/2=0.400; "
                    "inward=0.400+1.120=1.520, "
                    "outward=2.410-1.120-0.400=0.890"
                ),
                "center_distance_mm": CENTER_DISTANCE_MM,
                "axial_plane_shift_toward_flyer_mm": DRIVE_PLANE_SHIFT_MM,
                "calculated_pitch_length_mm": pitch_length,
                "ratio": ratio,
                "base_allowable_torque_300rpm_nm": PULLEY_BASE_TORQUE_300RPM_NM,
                "belt_length_factor": BELT_LENGTH_FACTOR,
                "mesh_factor": BELT_MESH_FACTOR,
                "allowable_torque_300rpm_nm": PULLEY_ALLOWABLE_TORQUE_300RPM_NM,
                "allowable_margin_over_known_load": (
                    PULLEY_ALLOWABLE_TORQUE_300RPM_NM / required
                ),
                "stock_motor_pulley_clamp": {
                    "type": "split clamp hub; tangential hex-socket bolt",
                    "body_material": "A2017",
                    "bolt_material": "SCM435 black oxide",
                    "bolt_size": NBK_CLAMP_BOLT_SIZE,
                    "tightening_torque_nm": (
                        NBK_CLAMP_BOLT_TIGHTENING_NM
                    ),
                    "published_mass_g": NBK_MOTOR_PULLEY_MASS_G,
                    "published_inertia_kgm2": (
                        NBK_MOTOR_PULLEY_INERTIA_KGM2
                    ),
                    "recommended_shaft_tolerance": "round h6 or h7",
                    "witness_bolt_length_mm_not_released": (
                        NBK_CLAMP_BOLT_WITNESS_LENGTH_MM
                    ),
                    "witness_placement": (
                        "inferred tangential placement from published "
                        "Db20/E23/L7.5/F2.75/G7.5/M2 drawing labels"
                    ),
                    "exact_vendor_CAD_gate": False,
                    "selected_motor_D_cut_interface_gate": False,
                    "gate": (
                        "OPEN: selected motor's 15 mm D-cut lies under most "
                        "of the pulley at this axial placement, while NBK "
                        "specifies a round h6/h7 shaft. Obtain exact CAD and "
                        "supplier authorization, or select/prove a D-shaft "
                        "compatible clamp interface."
                    ),
                },
                "flyer_pulley_retention": {
                    "geometry": (
                        "RFQ machined A2017/6061 P30-equivalent, OD12.05 bore, "
                        "integral rear OD23x6 collar outside the belt channel, "
                        "two orthogonal accessible tapped M3x6 flat-point "
                        "screws onto 0.3 mm shaft flats"
                    ),
                    "rear_collar_od_mm": FLYER_COLLAR_OD_MM,
                    "rear_collar_length_mm": FLYER_COLLAR_LENGTH_MM,
                    "screw_reach_to_flat_mm": (
                        FLYER_COLLAR_RADIAL_ENGAGEMENT_MM
                    ),
                    "tapped_aluminum_wall_mm": FLYER_COLLAR_TAPPED_WALL_MM,
                    "minimum_screw_to_belt_clearance_mm": clearances[
                        "belt_to_closest_flyer_set_screw_mm"
                    ],
                    "maximum_screw_to_pulley_interference_mm3": (
                        unintended_overlaps[
                            "flyer_set_screws_vs_pulley_max_mm3"
                        ]
                    ),
                    "torque_capacity_nm": None,
                    "gate": (
                        "OPEN: supplier confirmation or measured slip test "
                        f">= {minimum_capacity:.9f} Nm is only the present "
                        "pre-drive-inertia floor; final proof must include "
                        "belt, both pulleys, every set screw, candidate rotor, "
                        "tube crush/bore distortion, and reversing-cycle "
                        "retention at the recomputed threshold"
                    ),
                },
            },
        },
        "GOAL_OD65_duty": {
            "source_report": str(FORCE_REPORT.relative_to(ROOT)).replace("\\", "/"),
            "known_load_required_torque_nm": required,
            "minimum_2x_motor_running_torque_nm": minimum_capacity,
            "minimum_2x_pulley_capacity_nm": minimum_capacity,
            "motor_rotor_inertia_included": False,
            "successor_motor_pulley_inertia_included": False,
            "successor_flyer_pulley_inertia_included": False,
            "successor_belt_and_fastener_inertia_included": False,
        },
        "geometry": {
            "clearances_mm": clearances,
            "minimum_running_clearance_mm": min(clearances.values()),
            "unintended_overlaps_mm3": unintended_overlaps,
            "intended_contacts_mm3": intended_contacts,
            "solid_counts": {
                key: len(part.solids()) for key, part in parts.items()
            },
            "checks": geometry_checks,
        },
        "limitations": [
            "Motor running torque at 300 RPM has not been extracted from the manufacturer curve.",
            "Candidate NEMA17 rotor inertia has not yet been added to the OD65 acceleration load.",
            "The successor belt, both P30 pulleys, and all pulley fasteners have not yet been folded into the OD65 acceleration load; 0.72506264 Nm is therefore only a lower-bound gate.",
            "The candidate's exact vendor STEP is published but is not yet present locally; the review uses its datasheet envelope.",
            "The stock NBK motor pulley is a split clamp-hub, not a set-screw pulley. Its exact CAD is absent, so the visible M2 clamp-bolt placement is an inferred witness rather than released geometry.",
            "NBK recommends a round h6/h7 shaft, while the selected NEMA17 has a 15 mm D-cut under most of the installed pulley span. The motor-pulley shaft interface is not authorized.",
            "The P30 flyer-side OD12.05 pulley is custom; catalog belt capacity cannot authorize its hub. Supplier confirmation or a reversing-cycle slip-torque test is mandatory.",
            "The two 0.3 mm flyer-shaft flats leave 1.2 mm local wall in the hollow OD12/ID9 tube. Tube crush, bore distortion, and cyclic point loading under both M3 screws remain unproven.",
            "The enlarged mount still needs material/load/buildability authority and installed belt-tension proof.",
            "The pulley teeth and belt teeth are collision envelopes; supplier dimensions and torque tables are authoritative.",
            "Full-machine collision and raw-cycle regeneration remain mandatory after main-assembly integration.",
        ],
    }


def gen_step() -> Compound:
    parts = review_parts()
    children = [
        parts["mount"], parts["motor"], parts["motor_pulley"],
        parts["motor_pulley_clamp_bolt"], parts["flyer_pulley"], parts["belt"],
        *(parts[key] for key in sorted(parts)
          if key.startswith("motor_screw_") or key.startswith("flyer_set_screw_")),
        parts["shaft"], parts["flyer_block"], parts["wire_static"],
        parts["post_l"], parts["post_r"],
    ]
    result = Compound(children=children)
    result.label = "m2_exact_1to1_nema17_3gt_successor_review"
    return result


def write_report(report: dict) -> tuple[Path, Path]:
    REPORTS.mkdir(parents=True, exist_ok=True)
    json_path = REPORTS / "m2_drive_successor_review.json"
    md_path = REPORTS / "m2_drive_successor_review.md"
    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    drive = report["candidate"]["drive"]
    duty = report["GOAL_OD65_duty"]
    clear = report["geometry"]["clearances_mm"]
    lines = [
        "# M2 exact-1:1 drive successor review",
        "",
        f"Status: **{report['status']}**. Production authorization is false.",
        "",
        "The geometry package is the smallest normal-GOAL replacement studied: "
        "StepperOnline 17HS24-2004D-E1K plus equal 30T 3GT pulleys and a "
        "210-3GT-6 belt at the unchanged 60 mm centre distance.",
        "",
        f"* Exact ratio: {drive['ratio']:.1f}:1",
        f"* Pitch length: {drive['calculated_pitch_length_mm']:.3f} mm",
        f"* Corrected NBK allowable torque at 300 RPM: {drive['allowable_torque_300rpm_nm']:.3f} Nm",
        f"* Required 2x motor/pulley capacity before rotor inertia: {duty['minimum_2x_motor_running_torque_nm']:.6f} Nm",
        f"* Minimum modeled unintended running clearance: {report['geometry']['minimum_running_clearance_mm']:.3f} mm",
        f"* Static wire to candidate NEMA17 motor: {clear['wire_to_candidate_motor_mm']:.3f} mm",
        "",
        "## Open gate",
        "",
        "The motor is not approved from holding torque. Its manufacturer "
        "curve must show the required dynamic torque at 300 RPM at the chosen "
        "driver voltage/current, and the candidate NEMA17 rotor inertia must be folded "
        "back into the acceleration requirement. The stock NBK clamp pulley "
        "is also not authorized on the selected motor's 15 mm D-cut because "
        "NBK specifies a round h6/h7 shaft, and its exact CAD is absent. The "
        "custom flyer-side P30 hub needs supplier confirmation or a reversing-"
        "cycle slip/crush test at the recomputed minimum 2x torque.",
        "",
        "## Sources",
        "",
        f"* Motor: {MOTOR_PRODUCT_URL}",
        f"* NBK pulley: {PULLEY_URL}",
        f"* Gates 3MGT dimensions: {GATES_PM_MANUAL_URL}",
        f"* Gates pitch-line definition: {GATES_CATALOG_URL}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return json_path, md_path


def write_snapshot_job() -> Path:
    REVIEW.mkdir(parents=True, exist_ok=True)
    path = REVIEW / "m2_drive_successor_snapshot_job.json"
    job = {
        "input": "out/review/m2_drive_successor_review.step",
        "mode": "view",
        "outputs": [
            {"path": "out/review/snapshots/m2_drive_successor_iso.png", "camera": "iso"},
            {"path": "out/review/snapshots/m2_drive_successor_front.png", "camera": "front"},
            {"path": "out/review/snapshots/m2_drive_successor_top.png", "camera": "top"},
            {"path": "out/review/snapshots/m2_drive_successor_right.png", "camera": "right"},
        ],
        "display": {"mode": "solid", "projection": "orthographic"},
        "render": {
            "viewLabels": True,
            "padding": 0.1,
            "sizeProfile": "assembly-large",
        },
    }
    path.write_text(json.dumps(job, indent=2), encoding="utf-8")
    return path


def main() -> None:
    REVIEW.mkdir(parents=True, exist_ok=True)
    step_path = REVIEW / "m2_drive_successor_review.step"
    export_step(gen_step(), step_path)
    report = run_audit()
    json_path, md_path = write_report(report)
    snapshot_job = write_snapshot_job()
    print(step_path)
    print(json_path)
    print(md_path)
    print(snapshot_job)
    print(report["status"])


if __name__ == "__main__":
    main()
