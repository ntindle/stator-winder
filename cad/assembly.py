"""Full machine assembly, organized as four kinematic links.

Links and their pose transforms (model-space motor values, radians):
  static   : identity
  carriage : translate (0, 0, M0 * mm_per_rad)          [M0 <= 0]
  spindle  : carriage ∘ rotate about vertical axis X=0, Z=axis_z by M1
  flyer    : rotate about machine Z axis by M2

Parts are authored in machine coordinates at the reference pose
(M0=0 home, M1=0, M2=0 = arm at 12 o'clock). See printed.py header for the
documented in-place modeling decision. gen_step() exports the labeled
assembly at the reference pose; build_links() feeds the digital twin and
collision checker.
"""

import math
from pathlib import Path
from build123d import (
    Part, Cylinder, Cone, Pos, Rot, Align, Compound, Location, Axis,
    export_step,
)

from params import (
    DEFAULT_SPINDLE_ID,
    PARAMS as P,
    SpindleOption,
    StatorSpec,
    DEFAULT_STATOR,
    spindle_option,
)
import printed
import fabricated_carriage
import carriage_endstop_flag
import cots
import stator_model
import coil_growth
import hardware_placements
import wire_vis
import wire_geometry

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)


def _at(part, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0, label=None):
    q = Pos(x, y, z) * (Rot(rx, ry, rz) * part)
    q.label = label or getattr(part, "label", "part")
    return q


def alu_tube() -> Part:
    """Flyer hollow shaft: Ø12x1.5 wall aluminum tube, cut to length."""
    z0, z1 = P.flyer_shaft_rear_z, P.flyer_shaft_front_z
    p = Cylinder(P.flyer_shaft_od / 2, z1 - z0, align=CTR) - \
        Cylinder(P.flyer_shaft_id / 2, z1 - z0 + 2, align=CTR)
    p = Pos(0, 0, (z0 + z1) / 2) * p
    p.label = "alu_tube"
    return p


def shaft8_socket_holder(label="spindle_holder") -> Part:
    """Drawing geometry for the dedicated 8 mm stator-shaft holder.

    This is a project-defined machined part, not a claimed catalog SKU.  Its
    OD8 x 100 shank is a drop-in match for the existing 608 bearing stack and
    5x8 coupling.  The work end is OD16 x 16 with an ID8.10 x 14 socket, a
    0.5 mm mouth lead-in, and two orthogonal radial M4x0.7 tapped ports
    represented by their 3.30 mm tap-drill bores.  M4 cup-point set screws
    clamp the shaft at axial depths 5 and 10 mm; the 2 mm socket floor prevents
    a stator shaft from entering the bearing shank.

    Local frame matches the ER11 model: axis +Z, holder top datum z=0, shank
    extending toward -Z.
    """
    option = spindle_option("shaft8")
    body_r, body_len = option.neck_segments[0]
    ctr_min = (Align.CENTER, Align.CENTER, Align.MIN)
    body = Cylinder(body_r, body_len, align=(Align.CENTER, Align.CENTER,
                                             Align.MAX))
    shank = Pos(0, 0, -body_len) * Cylinder(
        option.shank_d / 2.0,
        option.shank_len,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    )
    socket = Cylinder(4.05, 14.0,
                      align=(Align.CENTER, Align.CENTER, Align.MAX))
    mouth = Pos(0, 0, -1.0) * Cone(4.05, 4.30, 1.0, align=ctr_min)
    # Blind tap-drill representations entering from +X and +Y.  Each extends
    # 0.55 mm into the socket so the eventual M4 screw reaches the shaft.
    tap_x = Pos(8.5, 0, -5.0) * (
        Rot(0, -90, 0) * Cylinder(1.65, 5.0, align=ctr_min)
    )
    tap_y = Pos(0, 8.5, -10.0) * (
        Rot(90, 0, 0) * Cylinder(1.65, 5.0, align=ctr_min)
    )
    p = (body + shank) - socket - mouth - tap_x - tap_y
    p.label = label
    return p


def _spindle_holder(option: str | SpindleOption) -> Part:
    resolved = spindle_option(option)
    if resolved.id == "er11":
        return cots.er11_chuck_c8_hifi(label="spindle_holder")
    if resolved.id == "shaft8":
        return shaft8_socket_holder()
    raise ValueError(f"no CAD factory for spindle option {resolved.id!r}")


# ------------------------------------------------------------------------

def static_link() -> list:
    zc = P.m0_home_standoff
    ext = cots.extrusion_2020
    wire_lm = wire_geometry.static_path_spec()["landmarks"]
    parts = [
        # frame: base long rails, crosses, stringers, posts
        _at(ext(450), -P.base_rail_x, -215, P.frame_z0, label="base_rail_L"),
        _at(ext(450), P.base_rail_x, -215, P.frame_z0, label="base_rail_R"),
        _at(ext(180), -90, -235, -180, ry=90, label="cross_rear"),
        _at(ext(180), -90, -235, -50, ry=90, label="cross_mid"),
        _at(ext(180), -90, -235, 160, ry=90, label="cross_front"),
        _at(ext(P.stringer_len), -P.rail_x, -215, P.stringer_z0,
            label="stringer_L"),
        _at(ext(P.stringer_len), P.rail_x, -215, P.stringer_z0,
            label="stringer_R"),
        _at(ext(235), -P.post_x, -205, (P.post_z[0] + P.post_z[1]) / 2,
            rx=-90, label="post_L"),
        _at(ext(235), P.post_x, -205, (P.post_z[0] + P.post_z[1]) / 2,
            rx=-90, label="post_R"),
        _at(ext(305), P.rear_post_x, -225, P.rear_post_z, rx=-90,
            label="rear_post"),
        printed.rear_post_left_shoe(),
        # Rubber feet and their slot nuts are exact hardware occurrences.
        # M0 axis: rails, screw, drive
        _mgn_rail(-P.rail_x), _mgn_rail(P.rail_x),
        _at(cots.t8_screw(P.screw_len), P.screw_x, P.screw_y, P.screw_z0,
            label="t8_screw"),
        printed.m0_fixed_end_mount(),
        _at(cots.bearing_688(), P.screw_x, P.screw_y, 136.5,
            label="m0_688_2rs"),
        _at(cots.m0_fixed_clamp_collar(), P.screw_x, P.screw_y, 125.0,
            label="m0_fixed_collar"),
        _at(cots.tube_spacer(12.0, 8.1, 1.0), P.screw_x, P.screw_y,
            139.5, label="m0_inner_shim"),
        _at(cots.din472_internal_ring(16.0, 16.8, 1.0),
            P.screw_x, P.screw_y, 139.5, label="m0_din472_16"),
        printed.m0_motor_mount(),
        _at(cots.nema17(), P.screw_x, P.screw_y, P.m0_motor_z, rx=180,
            label="m0_motor"),
        # The selected 27 mm Ruland coupling spans z142.5..169.5 at this
        # datum: 12.5 mm engagement on each shaft, below its 12.7 mm maximum,
        # with the intended 2 mm gap between shaft ends.  The collision model
        # conservatively retains the former 32 mm envelope.
        _at(cots.beam_coupling_5x8(), P.screw_x, P.screw_y, 156.0,
            label="m0_coupling"),
        printed.endstop_mount(),
        # Lever faces -Z toward the tab. The controlled Omron envelope puts
        # the free roller's near edge at z=144.60; the flag ends at z=142.00,
        # leaving a 2.60 mm home gap before deliberate homing overtravel.
        _at(cots.endstop(), 0, -199, P.endstop_switch_origin_z,
            ry=180, label="endstop"),
        # M2 drive (static side)
        printed.flyer_block(),
        _at(cots.bearing_6001(), 0, 0, -52, label="flyer_6001_front"),
        _at(cots.bearing_6001(), 0, 0, -71, label="flyer_6001_rear"),
        _at(cots.tube_spacer(27.8, 22.0, 11.0), 0, 0, -61.5,
            label="m2_outer_race_spacer"),
        _at(cots.din472_internal_ring(28.0, 29.4, 1.3), 0, 0, -75.65,
            label="m2_din472_28"),
        printed.m2_motor_mount(),
        _at(cots.nema17_mcmaster_6627t421(), 0, P.m2_motor_axis_y,
            P.m2_motor_face_z,
            label="m2_motor"),
        _at(cots.gt2_pulley_40t_b5(), 0, P.m2_motor_axis_y,
            sum(P.pulley_z) / 2.0, label="m2_motor_pulley"),
        _belt(),
        # M3 tensioner
        printed.spool_bracket(), printed.spool_drum(),
        printed.felt_tensioner(),
        _at(cots.ceramic_eyelet(), *wire_lm["felt_guide_in"], rx=-90,
            label="felt_guide_in"),
        printed.dancer_base(),
        printed.dancer_arm(), printed.dancer_pulley(),
        _at(cots.bearing_623(), P.dancer_pulley_x, P.dancer_pulley_y,
            -156.0, label="dancer_623"),
        printed.entry_bracket(),
        _at(cots.ceramic_eyelet(), 0, 0, P.wire_entry_z + 3,
            label="entry_eyelet"),
    ]
    return parts


def _mgn_rail(x) -> Part:
    """REAL HIWIN rail (raw frame: runs Z ±75, rail bottom y=-5.5)."""
    p = Pos(x, P.stringer_top_y + 5.5,
            P.rail_z0 + P.rail_len / 2) * cots.mgn12_rail()
    p.label = f"mgn12_rail_{'L' if x < 0 else 'R'}"
    return p


def _mgn_block(x, zc) -> Part:
    """REAL HIWIN block centered on the carriage mounting grid."""
    p = Pos(x, P.stringer_top_y + 5.5, zc) * cots.mgn12h_block_real()
    p.label = f"mgn12h_{'L' if x < 0 else 'R'}"
    return p


def _belt():
    """200-2GT belt loop solid (models/upgrades/gt2_belt.py geometry,
    imported STEP): local loop XY centers (0,0)/(0,-60), extruded z
    0..6; placed in the belt plane."""
    from build123d import import_step
    from pathlib import Path as _P
    p = import_step(str(_P(__file__).parent / "models" / "upgrades" /
                        "gt2_belt_200.step"))
    p = Pos(0, 0, (P.pulley_z[0] + P.pulley_z[1]) / 2 - 3.0) * p
    p.label = "gt2_belt"
    return p


def carriage_link() -> list:
    zc = P.m0_home_standoff
    parts = [
        fabricated_carriage.carriage_plate(),
        carriage_endstop_flag.endstop_flag(),
        printed.spindle_tower(),
        printed.nut_bracket(),
        _mgn_block(-P.rail_x, zc), _mgn_block(P.rail_x, zc),
        _at(cots.nema17(), 0, P.m1_motor_top_y, zc, rx=-90,
            label="m1_motor"),
        _at(cots.bearing_608(), 0, -98.5, zc, rx=-90, label="spindle_608_top"),
        _at(cots.bearing_608(), 0, -121.5, zc, rx=-90,
            label="spindle_608_bot"),
        _at(cots.tube_spacer(21.8, 18.2, 16.0), 0, -110.0, zc,
            rx=-90, label="m1_outer_race_spacer"),
        _at(cots.din472_internal_ring(22.0, 23.0, 1.1), 0, -125.55,
            zc, rx=-90, label="m1_din472_22_lower"),
        _at(cots.din472_internal_ring(22.0, 23.0, 1.1), 0, -94.45,
            zc, rx=-90, label="m1_din472_22_upper"),
        # Complete Zyltech T8x8 anti-backlash set.  The main flange seats on
        # the bracket rear face; spring and secondary nut extend away from
        # the bracket inside the explicitly reserved 22.4 mm envelope.
        _at(cots.t8_nut(), P.screw_x, P.screw_y, zc - 18, rx=180,
            label="t8_nut_main"),
        _at(cots.t8_nut_spring_envelope(), P.screw_x, P.screw_y,
            zc - 18, rx=180, label="t8_nut_spring"),
        _at(cots.t8_nut_secondary(), P.screw_x, P.screw_y,
            zc - 18, rx=180, label="t8_nut_secondary"),
    ]
    return parts


def spindle_link(spec: StatorSpec = DEFAULT_STATOR,
                 final_wound_collision: bool = False,
                 spindle: str | SpindleOption = DEFAULT_SPINDLE_ID) -> list:
    zc = P.m0_home_standoff
    option = spindle_option(spindle)
    holder = _at(_spindle_holder(option), 0,
                 -spec.stack / 2 - P.grip_gap, zc, rx=-90,
                 label="spindle_holder")
    coupling = _at(cots.beam_coupling_5x8(), 0, -153, zc, rx=-90,
                   label="m1_coupling")
    inner_spacer = _at(cots.tube_spacer(17.8, 8.05, 16.0),
                       0, -110.0, zc, rx=-90,
                       label="m1_inner_race_spacer")
    lower_spacer = _at(cots.tube_spacer(17.8, 8.05, 12.0),
                       0, -131.0, zc, rx=-90,
                       label="m1_lower_inner_spacer")
    upper_collar = _at(cots.shaft_clamp_collar(16.0, 8.05, 9.0),
                       0, -90.5, zc, rx=-90,
                       label="m1_upper_shaft_collar")
    # sequential composition (a single Rot(x,y,z) composes differently!):
    # shaft +Z -> machine +Y, then tooth-0 +X -> machine -Z
    stator_local = (coil_growth.wound_stator_collision_model(spec)
                    if final_wound_collision else stator_model.stator(spec))
    st = Pos(0, 0, zc) * (Rot(0, 90, 0) * (Rot(-90, 0, 0) *
                                           stator_local))
    st.label = ('stator_final_wound_envelope' if final_wound_collision
                else 'stator')
    sleeve_spec = wire_geometry.shaft_wrap_sleeve_spec(spec)
    wrap_sleeve = _at(
        cots.ceramic_shaft_wrap_sleeve(spec),
        0, sleeve_spec["axial_y_mm"], zc, rx=-90,
        label="shaft_wrap_sleeve",
    )
    return [holder, coupling, inner_spacer, lower_spacer, upper_collar,
            st, wrap_sleeve]


def flyer_link() -> list:
    parts = [
        alu_tube(),
        printed.flyer_arm(),
        printed.flyer_pulley(),
        printed.wire_elbow(),
        _at(cots.ceramic_toroid_guide(), 0, P.flyer_tip_r,
            wire_geometry.TIP_GUIDE_CENTER_Z, rx=-90,
            label="tip_toroid_guide"),
        _at(cots.tube_spacer(18.0, 12.05, 0.5), 0, 0, -75.25,
            label="m2_inner_rear_shim"),
        # OD17.8 retains the 6001 inner race while giving 2.1 mm exact radial
        # clearance inside the OD27.8/ID22 outer-race spacer.
        _at(cots.tube_spacer(17.8, 12.05, 11.0), 0, 0, -61.5,
            label="m2_inner_center_spacer"),
        _at(cots.tube_spacer(18.0, 12.05, 4.0), 0, 0, -46.0,
            label="m2_inner_front_spacer"),
    ]
    return parts


# ------------------------------------------------------------------------

def link_location(link: str, m0=0.0, m1=0.0, m2=0.0) -> Location:
    """World transform for a link at a pose (model-space radians)."""
    dz = m0 * P.mm_per_rad
    if link == "static":
        return Location()
    if link == "carriage":
        return Pos(0, 0, dz)
    if link == "spindle":
        axis_z = P.m0_home_standoff + dz
        # rotate about vertical axis through (0, *, axis_z), then it is
        # already carried by the carriage translation
        return (Pos(0, 0, axis_z) *
                Rot(0, math.degrees(m1), 0) *
                Pos(0, 0, -P.m0_home_standoff))
    if link == "flyer":
        return Rot(0, 0, math.degrees(m2))
    raise ValueError(link)


def build_links(spec: StatorSpec = DEFAULT_STATOR,
                final_wound_collision: bool = False,
                spindle: str | SpindleOption = DEFAULT_SPINDLE_ID) -> dict:
    """Reference-pose link solids for export; the twin applies
    link_location() transforms numerically to the meshed versions."""
    links = {
        "static": static_link(),
        "carriage": carriage_link(),
        "spindle": spindle_link(spec, final_wound_collision, spindle),
        "flyer": flyer_link(),
    }
    # Every audited bracket, screw, nut, washer, T-nut, insert, axle, and
    # tensioner stack item is a labeled occurrence.  Unselected items fail the
    # procurement gate and are not represented by fictitious geometry.
    placed = hardware_placements.hardware_parts_by_link(
        P, include_flagged=False)
    for link, parts in placed.items():
        links[link].extend(parts)
    return links


def machine(spec: StatorSpec = DEFAULT_STATOR,
            m0=0.0, m1=0.0, m2=0.0,
            spindle: str | SpindleOption = DEFAULT_SPINDLE_ID) -> Compound:
    links = build_links(spec, spindle=spindle)
    children = []
    for name, parts in links.items():
        loc = link_location(name, m0, m1, m2)
        moved = [loc * p for p in parts]
        for orig, m in zip(parts, moved):
            m.label = orig.label
        sub = Compound(children=moved)
        sub.label = name
        children.append(sub)
    # wire visualization (deliverable STEP/GLB only; not a collision body)
    ws = link_location("static", m0, m1, m2) * wire_vis.wire_static()
    wf = link_location("flyer", m0, m1, m2) * wire_vis.wire_flyer()
    ws.label = "wire_static"
    wf.label = "wire_flyer"
    wire_grp = Compound(children=[ws, wf])
    wire_grp.label = "wire"
    children.append(wire_grp)
    asm = Compound(children=children)
    asm.label = "winder_machine"
    return asm


def gen_step():
    return machine()


def main() -> None:
    """Regenerate the review STEP promised by the repository workflow."""
    target = Path(__file__).with_name("assembly.step")
    export_step(gen_step(), target)
    print(target)


if __name__ == "__main__":
    main()
