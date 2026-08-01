"""Source-level carriage fastener and retention audit.

This module is deliberately isolated from :mod:`assembly`.  It reproduces the
current carriage stack, builds corrected candidate solids, and uses build123d
booleans for the exact failure modes found during the Goal-1 hardware pass.
It does not mutate production CAD.

CAD brief
---------

* Task: inspect the moving MIC6 carriage, printed tower/T8 bracket, M1 motor,
  endstop flag, MGN12H screws, and their complete fastener stacks.
* Units/frame: machine millimetres; X/Z are the MIC6 sheet plane, +Y is up.
* Datum: the MIC6 bottom/top faces and spindle axis ``X=0, Z=95`` remain fixed.
* Manufacturing constraint: retain a 1:1 through-cut-only ALUMIC6-250 DXF;
  the existing T8 edge relief may extend to the lower plate edge.
* Validation: every known current positive-volume interference is reproduced;
  the corrected candidates remove it, retain head-bearing material, and give
  explicit thread/grip arithmetic.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
from typing import Iterable

from build123d import Align, Box, Cylinder, Part, Pos, Rot

import carriage_endstop_flag
import cots
import fabricated_carriage
import hardware
import hardware_placements
from params import PARAMS as P
import printed


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "out" / "reports" / \
    "carriage_hardware_audit.json"

# Exact corrected geometry contract.  These values are intentionally gathered
# here so root can copy one table into shared source without reverse-engineering
# the audit solids.
BLOCK_HEAD_RELIEF_CENTERS_XZ = (
    (-35.0, 85.0), (-35.0, 105.0),
    (35.0, 85.0), (35.0, 105.0),
)
BLOCK_HEAD_RELIEF_D = 6.4
BLOCK_HEAD_RELIEF_DEPTH = 3.25

M1_MOTOR_HEAD_Y = P.plate_top_y + 6.0

NUT_FOOT_WEB_H = 2.4
NUT_BOSS_X = (-84.0, -72.0)
NUT_BOSS_Z = (92.0, 112.0)
NUT_BOSS_H = 8.0
T8_ACCESS_D = 7.2
T8_ACCESS_Z = (85.0, 92.0)

FIXED_MOUNT_FOOT_X_MAX = -38.0

M4_WASHER_T = 0.9
M3_WASHER_T = 0.55


def _box(x0: float, x1: float, y0: float, y1: float,
         z0: float, z1: float) -> Part:
    return Pos((x0 + x1) / 2.0, (y0 + y1) / 2.0,
               (z0 + z1) / 2.0) * Box(
        x1 - x0, y1 - y0, z1 - z0, align=CTR,
    )


def _cyl_y(radius: float, y0: float, y1: float,
           *, x: float, z: float) -> Part:
    return Pos(x, (y0 + y1) / 2.0, z) * (
        Rot(90.0, 0.0, 0.0) * Cylinder(radius, y1 - y0, align=CTR)
    )


def _cyl_z(radius: float, z0: float, z1: float,
           *, x: float, y: float) -> Part:
    return Pos(x, y, (z0 + z1) / 2.0) * Cylinder(
        radius, z1 - z0, align=CTR,
    )


def common_volume(a: Part, b: Part) -> float:
    """Positive-volume overlap in mm^3; face contact returns zero."""
    return float((a & b).volume)


def proposed_spindle_tower() -> Part:
    """Current tower plus four printable block-head reliefs.

    The existing M1 pocket is already correct: it rises only two millimetres
    from the flange underside and leaves a four-millimetre roof.  This function
    deliberately preserves that roof, the spindle/bearing geometry, and all
    four M4-to-MIC6 datums.
    """
    tower = printed.spindle_tower()
    yb = P.plate_top_y

    # M3 ISO 4762 head OD is 5.68 mm and height is 3.00 mm in hardware.py.
    # Ø6.4 gives 0.36 mm radial print allowance; 3.25 deep leaves a 2.75 mm
    # roof in the existing 6 mm flange (above P.min_wall=2.4).
    for x, z in BLOCK_HEAD_RELIEF_CENTERS_XZ:
        tower -= _cyl_y(BLOCK_HEAD_RELIEF_D / 2.0, yb - 0.5,
                         yb + BLOCK_HEAD_RELIEF_DEPTH, x=x, z=z)

    tower.label = "spindle_tower_corrected_candidate"
    return tower


def _t8_radial_centers() -> tuple[tuple[float, float], ...]:
    return tuple(
        (P.screw_x + 8.0 * math.cos(math.radians(angle)),
         P.screw_y + 8.0 * math.sin(math.radians(angle)))
        for angle in (45.0, 135.0, 225.0, 315.0)
    )


def proposed_nut_bracket() -> Part:
    """Printable T8 bracket with four usable flange holes.

    The old 8 mm-high full-width foot buried the two lower washer/nyloc stacks.
    The candidate retains an 8 mm M4 bearing rail only at x=-84..-72,
    bridges it with a 2.4 mm web, and opens two Ø7.2 access channels between
    z=85..92.  All T8 washers still bear on the unchanged z=85 wall face.
    """
    zc = P.m0_home_standoff
    y0 = P.plate_top_y
    wall = _box(-84.0, -60.0, y0, P.screw_y + 14.0,
                zc - 18.0, zc - 10.0)
    web = _box(-84.0, -60.0, y0, y0 + NUT_FOOT_WEB_H,
               zc - 10.0, zc + 14.0)
    lower_centers = _t8_radial_centers()[2:]
    for x, y in lower_centers:
        web -= _cyl_z(T8_ACCESS_D / 2.0, *T8_ACCESS_Z, x=x, y=y)
    boss = _box(NUT_BOSS_X[0], NUT_BOSS_X[1], y0,
                y0 + NUT_BOSS_H, NUT_BOSS_Z[0], NUT_BOSS_Z[1])
    bracket = wall + web + boss

    bracket -= _cyl_z(5.5, zc - 19.0, zc - 9.0,
                       x=P.screw_x, y=P.screw_y)
    for x, y in _t8_radial_centers():
        bracket -= _cyl_z(1.6, zc - 19.0, zc - 9.0, x=x, y=y)
    for z in (zc + 2.0, zc + 12.0):
        bracket -= _cyl_y(2.2, y0 - 8.0, y0 + 9.0, x=-78.0, z=z)
    bracket.label = "nut_bracket_corrected_candidate"
    return bracket


def proposed_fixed_end_mount() -> Part:
    """Trim only the non-functional right edge of the M0 support foot."""
    mount = printed.m0_fixed_end_mount()
    mount -= _box(FIXED_MOUNT_FOOT_X_MAX, -34.0,
                  P.stringer_top_y - 1.0, P.stringer_top_y + 9.0,
                  121.0, 151.0)
    mount.label = "m0_fixed_end_mount_corrected_candidate"
    return mount


def _placed(part: Part, origin: tuple[float, float, float], axis: str,
            label: str) -> Part:
    return hardware.place(part, origin, axis=axis, label=label)


def proposed_hardware() -> dict[str, Part]:
    """Exact corrected carriage stacks, keyed by stable occurrence label."""
    result: dict[str, Part] = {}
    zc = P.m0_home_standoff
    plate_bottom = P.plate_top_y - P.plate_t
    tower_top = P.plate_top_y + 6.0

    # MGN12H screws remain standard ISO4762 M3x10.  The printed relief, not
    # a sheet countersink, solves the tower overlap.
    for side, rail_x in (("L", -P.rail_x), ("R", P.rail_x)):
        for dx in (-10.0, 10.0):
            for dz in (-10.0, 10.0):
                label = f"block_{side}_m3x10_{dx:+g}_{dz:+g}"
                result[label] = _placed(
                    hardware.socket_head_cap_screw("M3", 10.0),
                    (rail_x + dx, P.plate_top_y, zc + dz), "+y", label,
                )

    # Tower/plate/flag stacks.  The front row bears below the MIC6 plate;
    # the rear row bears below the separate 6 mm flag.
    for dx in (-31.0, 31.0):
        for dz in (-31.0, 31.0):
            rear = dz > 0.0
            length = 25.0 if rear else 20.0
            y_bearing = (carriage_endstop_flag.FLAG_BOTTOM_Y
                         if rear else plate_bottom)
            screw_label = f"tower_m4x{int(length)}_{dx:+g}_{dz:+g}"
            washer_label = f"tower_washer_m4_{dx:+g}_{dz:+g}"
            nut_label = f"tower_nyloc_m4_{dx:+g}_{dz:+g}"
            result[screw_label] = _placed(
                hardware.socket_head_cap_screw("M4", length),
                (dx, tower_top, zc + dz), "+y", screw_label,
            )
            result[washer_label] = _placed(
                hardware.plain_washer("M4"),
                (dx, y_bearing, zc + dz), "-y", washer_label,
            )
            result[nut_label] = _placed(
                hardware.nyloc_nut("M4"),
                (dx, y_bearing - M4_WASHER_T, zc + dz), "-y", nut_label,
            )

    # Nut bracket to MIC6 remains M4x25: 14.35 grip + 0.9 washer + 5 nyloc
    # leaves 4.75 mm visible thread.  The local boss rail retains its 8 mm
    # bearing height even though the rest of the foot becomes a 2.4 mm web.
    for dz in (2.0, 12.0):
        x, z = -78.0, zc + dz
        for label, part, origin, axis in (
            (f"nut_bracket_m4x25_{dz:+g}",
             hardware.socket_head_cap_screw("M4", 25.0),
             (x, P.plate_top_y + NUT_BOSS_H, z), "+y"),
            (f"nut_bracket_washer_m4_{dz:+g}",
             hardware.plain_washer("M4"), (x, plate_bottom, z), "-y"),
            (f"nut_bracket_nyloc_m4_{dz:+g}",
             hardware.nyloc_nut("M4"),
             (x, plate_bottom - M4_WASHER_T, z), "-y"),
        ):
            result[label] = _placed(part, origin, axis, label)

    # The supplier drawing calls out M3 threaded flange holes. M3x12 enters
    # from the bracket front through a 0.55 mm washer and 8 mm wall, leaving
    # 3.45 mm Delrin engagement without hardware in the spring annulus.
    for index, (x, y) in enumerate(_t8_radial_centers(), start=1):
        rows = (
            (f"t8_flange_m3x12_{index}",
             hardware.socket_head_cap_screw("M3", 12.0),
             (x, y, zc - 10.0 + M3_WASHER_T), "+z"),
            (f"t8_flange_washer_m3_{index}",
             hardware.plain_washer("M3"),
             (x, y, zc - 10.0), "+z"),
        )
        for label, part, origin, axis in rows:
            result[label] = _placed(part, origin, axis, label)

    # The existing four-millimetre pocket roof produces exactly six
    # millimetres of M3x10 engagement in the motor's tapped mounting holes.
    half_grid = 31.0 / 2.0
    for ix, dx in enumerate((-half_grid, half_grid)):
        for iz, dz in enumerate((-half_grid, half_grid)):
            label = f"m1_motor_m3x10_{ix}_{iz}"
            result[label] = _placed(
                hardware.socket_head_cap_screw("M3", 10.0),
                (dx, M1_MOTOR_HEAD_Y, zc + dz), "+y", label,
            )
    return result


def current_hardware() -> dict[str, Part]:
    """Build current production carriage hardware without importing assembly."""
    return {
        occurrence.label: occurrence.build()
        for occurrence in hardware_placements.carriage_occurrences(P)
    }


def _support_annulus_y(x: float, z: float, y0: float, y1: float,
                       outer_r: float, bore_r: float) -> Part:
    return (_cyl_y(outer_r, y0, y1, x=x, z=z)
            - _cyl_y(bore_r, y0 - 0.1, y1 + 0.1, x=x, z=z))


def m1_head_support_volume(tower: Part, head_y: float) -> float:
    """Material under all four M1 screw heads, sampled 0.10 mm deep."""
    volume = 0.0
    half_grid = 31.0 / 2.0
    for x in (-half_grid, half_grid):
        for dz in (-half_grid, half_grid):
            probe = _support_annulus_y(
                x, P.m0_home_standoff + dz, head_y - 0.10, head_y,
                outer_r=2.84, bore_r=1.70,
            )
            volume += common_volume(tower, probe)
    return volume


@dataclass(frozen=True)
class AuditResult:
    check: str
    baseline_mm3: float
    live_source_mm3: float
    corrected_mm3: float
    pass_rule: str
    passed: bool


def audit_results() -> tuple[AuditResult, ...]:
    """Run the targeted baseline/live/corrected BREP checks.

    ``baseline_mm3`` preserves the measured pre-fix evidence even after root
    copies these recommendations into shared production source.  The live
    value then falls to zero while the corrected candidate remains green.
    """
    current_hw = current_hardware()
    corrected_hw = proposed_hardware()
    current_tower = printed.spindle_tower()
    corrected_tower = proposed_spindle_tower()
    current_flag = carriage_endstop_flag.endstop_flag()
    current_mount = printed.m0_fixed_end_mount()
    corrected_mount = proposed_fixed_end_mount()

    current_block = sum(
        common_volume(current_tower, part)
        for label, part in current_hw.items()
        if label.startswith("block_") and (
            "_+10_" in label and "block_L" in label
            or "_-10_" in label and "block_R" in label
        )
    )
    corrected_block = sum(
        common_volume(corrected_tower, part)
        for label, part in corrected_hw.items()
        if label.startswith("block_") and (
            "_+10_" in label and "block_L" in label
            or "_-10_" in label and "block_R" in label
        )
    )

    rear_labels = tuple(
        label for label in current_hw
        if label.startswith(("tower_washer", "tower_nyloc"))
        and label.endswith("_+31")
    )
    current_rear = sum(common_volume(current_flag, current_hw[label])
                       for label in rear_labels)
    corrected_rear = sum(common_volume(current_flag, corrected_hw[label])
                         for label in rear_labels)

    t8_spring = Pos(P.screw_x, P.screw_y, P.m0_home_standoff - 18.0) * (
        Rot(180.0, 0.0, 0.0) * cots.t8_nut_spring_envelope())
    t8_secondary = Pos(
        P.screw_x, P.screw_y, P.m0_home_standoff - 18.0) * (
            Rot(180.0, 0.0, 0.0) * cots.t8_nut_secondary())
    plate = fabricated_carriage.carriage_plate()
    t8_screw_labels = tuple(
        f"t8_flange_m3x12_{index}" for index in range(1, 5))
    current_t8 = (
        common_volume(plate, t8_spring)
        + common_volume(plate, t8_secondary)
        + sum(common_volume(t8_spring, current_hw[label])
              for label in t8_screw_labels)
    )
    corrected_t8 = (
        common_volume(plate, t8_spring)
        + common_volume(plate, t8_secondary)
        + sum(common_volume(t8_spring, corrected_hw[label])
              for label in t8_screw_labels)
    )

    current_flag_mount = common_volume(current_flag, current_mount)
    corrected_flag_mount = common_volume(current_flag, corrected_mount)

    current_support = m1_head_support_volume(current_tower, M1_MOTOR_HEAD_Y)
    corrected_support = m1_head_support_volume(
        corrected_tower, M1_MOTOR_HEAD_Y,
    )

    return (
        AuditResult("four inner MGN screw heads vs tower",
                    275.1583977626061, current_block, corrected_block,
                    "corrected == 0", corrected_block < 1e-6),
        AuditResult("rear flag washer/nyloc vs flag",
                    294.61415187794927, current_rear, corrected_rear,
                    "corrected == 0", corrected_rear < 1e-6),
        AuditResult("complete T8 set vs plate/front-side screws",
                    177.692996, current_t8, corrected_t8,
                    "corrected == 0", corrected_t8 < 1e-6),
        AuditResult("endstop flag vs M0 fixed-end mount",
                    12.0, current_flag_mount, corrected_flag_mount,
                    "corrected == 0", corrected_flag_mount < 1e-6),
        AuditResult("M1 screw-head bearing material (0.1 mm probes)",
                    6.5038507751674395, current_support, corrected_support,
                    "current and corrected > 0",
                    current_support > 1.0 and corrected_support > 1.0),
    )


PATCH_TABLE: tuple[dict[str, str], ...] = (
    {
        "source": "cad/printed.py:spindle_tower",
        "feature": "MGN12H inner screw-head reliefs",
        "exact_patch": (
            "subtract four axis-Y cylinders Ø6.4, y=plate_top-0.5.."
            "plate_top+3.25, at X/Z=(-35,85),(-35,105),(35,85),(35,105)"
        ),
        "hardware": "retain ISO4762 M3x10 x8; no MIC6 countersinks",
    },
    {
        "source": "cad/printed.py:nut_bracket",
        "feature": "T8 lower-stack access and M4 bearing rail",
        "exact_patch": (
            "replace 8 mm full foot with 2.4 mm web z=85..109; add local "
            "boss rail x=-84..-72,y=plate_top..+8,z=92..112; subtract "
            "Ø7.2 access channels at the 225/315 degree T8 holes, z=85..92"
        ),
        "hardware": "T8 ISO4762 M3x12 x4; bracket ISO4762 M4x25 x2",
    },
    {
        "source": "cad/printed.py:m0_fixed_end_mount",
        "feature": "endstop-flag running clearance",
        "exact_patch": "trim foot x maximum from -35 to -38 for 2.0 mm gap",
        "hardware": "no change; M5 foot-hole edge ligament remains 4.3 mm",
    },
    {
        "source": "cad/hardware_placements.py:carriage_occurrences",
        "feature": "rear flag washer and nyloc datums",
        "exact_patch": (
            "rear washer y=FLAG_BOTTOM_Y=-198.00; rear nyloc y=-198.90; "
            "front row stays at plate_bottom and plate_bottom-0.90"
        ),
        "hardware": "front M4x20 x2; rear M4x25 x2; ISO7089/ISO10511 M4",
    },
    {
        "source": "cad/hardware.py + cad/hardware_placements.py",
        "feature": "T8 threaded-flange screw direction and length",
        "exact_patch": (
            "install ISO4762-M3x12 from bracket +Z through washer at z=85; "
            "remove rear-side heads and through-nylocs from spring annulus"
        ),
        "hardware": "M3x12 gives 3.45 mm engagement in threaded Delrin",
    },
    {
        "source": "cad/fabricated_carriage.py:T8_RELIEF",
        "feature": "complete anti-backlash set edge clearance",
        "exact_patch": (
            "retain x=-82..-60 and z_max=91.5; extend z_min from 69.0 "
            "to the plate lower edge z=52.5"
        ),
        "hardware": "unchanged Zyltech 22.4 mm configured nut set",
    },
)


INTENDED_CONTACTS: tuple[dict[str, str], ...] = (
    {"pair": "carriage_plate / spindle_tower",
     "intent": "printed tower seats on MIC6 top face"},
    {"pair": "carriage_plate / endstop_flag",
     "intent": "flag seats on MIC6 bottom face at rear M4 row"},
    {"pair": "MGN block screws / carriage_plate / MGN12H tapped blocks",
     "intent": "head bearing, through clearance, 3.65 mm thread engagement"},
    {"pair": "tower screws / tower / plate [/ flag] / washer / nyloc",
     "intent": "front M4x20 leaves 1.75 mm; rear M4x25 leaves 0.75 mm"},
    {"pair": "nut-bracket M4x25 / boss rail / plate / washer / nyloc",
     "intent": "4.75 mm visible thread; head bears on 8 mm local rail"},
    {"pair": "T8 M3x12 / washer / 8 mm bracket wall / threaded flange",
     "intent": "3.45 mm Delrin engagement; spring side remains unobstructed"},
    {"pair": "M1 M3x10 / 4 mm flange roof / motor tapped hole",
     "intent": "6.0 mm thread engagement; existing roof bears on motor face"},
    {"pair": "m1_inner_race_spacer / m1_outer_race_spacer",
     "intent": "explicit dynamic fit exemption; concentric rotating/stationary race spacers, 0.20 mm radial clearance, zero positive overlap"},
    {"pair": "m1_lower_inner_spacer / m1_din472_22_lower",
     "intent": "explicit dynamic fit exemption; concentric inner-race spacer/ring, about 0.60 mm radial clearance, zero positive overlap"},
)


def report_dict() -> dict[str, object]:
    results = audit_results()
    plate = fabricated_carriage.carriage_plate()
    return {
        "schema": 1,
        "passed": all(result.passed for result in results),
        "checks": [asdict(result) for result in results],
        "patch_table": PATCH_TABLE,
        "intended_contacts": INTENDED_CONTACTS,
        "sendcutsend": {
            "plate_geometry_changed": True,
            "stock": "ALUMIC6-250, 6.35 mm",
            "through_cut_profile_only": True,
            "plate_volume_mm3": round(float(plate.volume), 6),
            "reason": (
                "existing T8 rectangular relief extends to lower plate edge "
                "to clear the complete 22.4 mm anti-backlash set"),
        },
        "stator_datum": {
            "axis_x_mm": 0.0,
            "axis_z_home_mm": P.m0_home_standoff,
            "plate_bottom_y_mm": P.block_top_y,
            "plate_top_y_mm": P.plate_top_y,
            "changed": False,
        },
    }


def write_json(path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report_dict(), indent=2) + "\n",
                      encoding="utf-8")
    return target


if __name__ == "__main__":
    data = report_dict()
    DEFAULT_REPORT.parent.mkdir(parents=True, exist_ok=True)
    DEFAULT_REPORT.write_text(json.dumps(data, indent=2) + "\n",
                              encoding="utf-8")
    print(json.dumps(data, indent=2))
    print(f"wrote {DEFAULT_REPORT}")
    if not data["passed"]:
        raise SystemExit(1)
