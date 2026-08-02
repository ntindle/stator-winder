"""COTS component geometry: verified step.parts imports + datasheet envelopes.

Every function returns a build123d Part/Solid in a documented LOCAL frame.
Provenance policy (GOAL.md): imported models are simplified reference solids;
mounting interfaces and outer envelopes are set from manufacturer datasheets.
Two step.parts models (MGN12 rail & carriage) measured dimensionally wrong vs
the HIWIN datasheet, so rail/block are parametric here (see bom.csv notes).

step.parts imports used (sha256-verified at download):
  stepper_motor_nema17_l0040_single_shaft, bearing_608zz,
  bearing_6001_zz_shielded_simple, t8_p2_flange_nut,
  t8_p2_lead_screw_l0200_simple, gt2_pulley_40t_bore5_w6,
  beam_coupling_bore5_to_8
Local frames of the imports (measured via inspect refs --facts):
  NEMA17: body 42.3^2, z 0..40.3 body, boss+shaft up to z 54.8 (+Z = shaft)
  608ZZ: axis Z, z -3.5..3.5, Ø22 / Ø8
  6001ZZ: axis Z, z 0..8, Ø28 / Ø12
  T8 flange nut: axis X, x -12..12 (envelope longer than generic 15mm nut)
  T8 screw L200: axis X, x -100..100, Ø8
  GT2 40T bore5 w6: axis Z, z -5..5, Ø31 envelope (incl. flanges)
  beam coupling 5->8: axis X, x -16..16, Ø24
"""

from pathlib import Path
from functools import lru_cache
import math

from build123d import (
    Part, Box, Cylinder, Pos, Rot, import_step, Align, Compound,
    BuildPart, BuildSketch, Circle, Rectangle, Polygon, extrude, Location,
    Plane, Mode, RegularPolygon, Torus,
)

MODELS = Path(__file__).parent / "models" / "parts"

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# Exact supplier/catalog geometry is useful for the project's local fit and
# collision audits, but most of those files cannot be redistributed.  Keep
# exact mode as the verification default and expose an explicit envelope mode
# for public, source-only assembly generation.
_REFERENCE_MODE = "exact"


def set_reference_mode(mode: str) -> None:
    """Select ``exact`` cached CAD or redistributable parametric envelopes."""
    normalized = str(mode).strip().lower()
    if normalized not in {"exact", "envelope"}:
        raise ValueError("reference mode must be 'exact' or 'envelope'")
    global _REFERENCE_MODE
    _REFERENCE_MODE = normalized


def reference_mode() -> str:
    return _REFERENCE_MODE


def using_reference_envelopes() -> bool:
    return _REFERENCE_MODE == "envelope"


@lru_cache(maxsize=None)
def _imp(name: str) -> Compound:
    return import_step(str(MODELS / f"{name}.step"))


def _copy(name: str) -> Part:
    # imported compounds are cached; return located copies via moved()
    return _imp(name)


def nema17(label="nema17") -> Part:
    """17HS19-2004D-E1K closed-loop NEMA17 envelope, dimensions VERIFIED
    against the current vendor STEP
    (cad/models/upgrades/17HS19-2004D-E1K.step; see motors.report.md):
    faceplate 42.3^2, body+encoder 68.0 deep, boss Ø22x2, shaft Ø5x24.
    Envelope solid per GOAL simplified-COTS rule; the raw vendor STEP is kept
    as the interface/reference authority while its loose 500 mm cable is not
    treated as a rigid collision solid.
    Local: mounting face z=0, body -Z (to -68), shaft +Z."""
    body = Box(42.3, 42.3, 68.0, align=(Align.CENTER, Align.CENTER,
                                        Align.MAX))
    boss = Cylinder(11.0, 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    shaft = Cylinder(2.5, 24.0, align=(Align.CENTER, Align.CENTER,
                                       Align.MIN))
    p = body + boss + shaft
    p.label = label
    return p


def nema23(label="nema23") -> Part:
    """NEMA23 56 mm envelope (23HS22-2804S-E1000 class). step.parts had no
    NEMA23 at search time; parametric per datasheet: body 57^2 x 56,
    boss Ø38.1 x 1.6, shaft Ø8 x 21, holes 47.14 grid (modeled in mount).
    Local: mounting face z=0, body -Z, shaft +Z."""
    body = Box(57.3, 57.3, 81.0, align=(Align.CENTER, Align.CENTER,
                                         Align.MAX))
    boss = Cylinder(19.05, 1.6, align=(Align.CENTER, Align.CENTER,
                                       Align.MIN))
    shaft = Cylinder(4.0, 21.0, align=(Align.CENTER, Align.CENTER,
                                       Align.MIN))
    p = body + boss + shaft
    p.label = label
    return p


def bearing_608(label="608zz") -> Part:
    if using_reference_envelopes():
        p = (Cylinder(11.0, 7.0, align=CTR)
             - Cylinder(4.0, 9.0, align=CTR))
    else:
        p = _copy("bearing_608zz")  # centered, axis Z, ±3.5
    p.label = label
    return p


def bearing_6001(label="6001zz") -> Part:
    if using_reference_envelopes():
        p = (Cylinder(14.0, 8.0, align=CTR)
             - Cylinder(6.0, 10.0, align=CTR))
    else:
        p = Pos(0, 0, -4.0) * _copy(
            "bearing_6001_zz_shielded_simple")  # center it
    p.label = label
    return p


def bearing_623(label="623zz") -> Part:
    """623ZZ miniature bearing envelope: Ø10 / Ø3 x 4, axis Z."""
    p = Cylinder(5.0, 4.0, align=CTR) - Cylinder(1.5, 6.0, align=CTR)
    p.label = label
    return p


def bearing_688(label="688_2rs") -> Part:
    """688-2RS miniature bearing envelope: 8 x 16 x 5, axis +Z."""
    p = Cylinder(8.0, 5.0, align=CTR) - Cylinder(4.0, 7.0, align=CTR)
    p.label = label
    return p


def tube_spacer(od, bore, length, label="tube_spacer") -> Part:
    """Dimensioned metal spacer/shim envelope, centered on local +Z."""
    p = Cylinder(od / 2.0, length, align=CTR) - \
        Cylinder(bore / 2.0, length + 2.0, align=CTR)
    p.label = label
    return p


def shaft_clamp_collar(od=18.0, bore=8.05, length=9.0,
                       label="shaft_collar") -> Part:
    """Set-screw shaft collar envelope, centered on local +Z.

    The radial M3 tapped hole is represented as a clearance cylinder so the
    collar remains useful in fit/collision checks without modeling threads.
    """
    p = tube_spacer(od, bore, length, label)
    p -= Pos(0, 0, 0) * (Rot(0, 90, 0) *
                         Cylinder(1.5, od / 2.0 + 1.0,
                                  align=(Align.CENTER, Align.CENTER,
                                         Align.MIN)))
    p.label = label
    return p


def m0_fixed_clamp_collar(label="m0_fixed_clamp_collar") -> Part:
    """9 mm M0 clamp collar with an inner-race pilot nose.

    Local z=0..9.  The Ø18 clamp body occupies the first 7 mm and the final
    Ø11.8 nose passes through the mount's Ø12.1 throat to bear only on the
    688 inner race.  This preserves the specified clamp envelope without
    embedding it in the printed bearing shoulder.
    """
    body = Pos(0, 0, 3.5) * (
        Cylinder(9.0, 7.0, align=CTR) - Cylinder(4.025, 9.0, align=CTR))
    nose = Pos(0, 0, 8.0) * (
        Cylinder(5.9, 2.0, align=CTR) - Cylinder(4.025, 4.0, align=CTR))
    set_hole = Pos(0, 0, 3.5) * (Rot(0, 90, 0) *
               Cylinder(1.5, 10.0,
                        align=(Align.CENTER, Align.CENTER, Align.MIN)))
    p = body + nose - set_hole
    p.label = label
    return p


def din472_internal_ring(nominal_bore, groove_d, thickness,
                         label="din472_ring") -> Part:
    """Installed DIN 472 internal retaining-ring envelope, axis +Z.

    ``groove_d`` is the installed outside diameter.  The conservative inside
    diameter leaves a 1.5 mm radial ring section; a 3 mm opening makes this a
    physically split, single solid rather than a decorative full washer.
    """
    ro = groove_d / 2.0 - 0.15
    ri = nominal_bore / 2.0 - 1.5
    p = Cylinder(ro, thickness, align=CTR) - \
        Cylinder(ri, thickness + 2.0, align=CTR)
    gap = Pos(0, ro / 2.0, 0) * Box(3.0, ro + 1.0, thickness + 2.0,
                                    align=CTR)
    p -= gap
    p.label = label
    return p


def heat_set_insert_m3(length=5.0, od=4.6,
                       label="m3_heat_set_insert") -> Part:
    """Brass M3 heat-set insert mass/fit envelope, axis +Z."""
    p = Cylinder(od / 2.0, length, align=CTR) - \
        Cylinder(1.5, length + 2.0, align=CTR)
    p.label = label
    return p


def din125_washer_m3(label="din125_m3_washer") -> Part:
    """DIN 125 M3 washer: 7 OD x 3.2 ID x 0.5."""
    p = Cylinder(3.5, 0.5, align=CTR) - Cylinder(1.6, 2.5, align=CTR)
    p.label = label
    return p


def socket_head_cap_screw_m3(length=12.0,
                             label="m3_socket_head_screw") -> Part:
    """DIN 912 M3 screw envelope; local head underside is z=0.

    Thread length extends toward -Z and the 3 mm head extends toward +Z.
    The socket recess is included because its removed steel mass matters to
    the flyer counterbalance calculation.
    """
    shaft = Pos(0, 0, -length / 2.0) * Cylinder(1.5, length, align=CTR)
    head = Pos(0, 0, 1.5) * Cylinder(2.75, 3.0, align=CTR)
    socket = Pos(0, 0, 2.2) * Cylinder(1.45, 1.6, align=CTR)
    p = shaft + head - socket
    p.label = label
    return p


def nema17_mcmaster_6627t421(label="nema17_6627t421") -> Compound:
    """McMaster 6627T421 high-torque closed-loop NEMA17, cable removed.

    The authoritative vendor STEP supplied from the McMaster product page is
    kept at ``models/upgrades/6627T421.step`` (SHA-256 recorded by the CAD
    inspection report).  Its raw +Z mounting face is 28.0126 mm and the shaft
    tip is at 50.0126 mm, giving the catalog 22 mm protrusion.  The encoder rear
    is at -50.0096 mm, or 78.0222 mm behind the mounting face.

    McMaster ships the model with its loose 500 mm cable coiled beside the
    motor.  That shipping pose is not an installed machine part and would make
    collision results arbitrary, so retain the motor, encoder/connector body,
    shaft and register solids (volume > 500 mm^3) while omitting only the loose
    cable/individual pin detail.  The asymmetric connector envelope remains.
    Local frame: mounting face z=0, body/encoder -Z, shaft +Z.
    """
    if using_reference_envelopes():
        body = Box(42.3, 42.3, 78.1,
                   align=(Align.CENTER, Align.CENTER, Align.MAX))
        boss = Cylinder(11.0, 2.0,
                        align=(Align.CENTER, Align.CENTER, Align.MIN))
        shaft = Cylinder(2.5, 22.0,
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
        connector = Pos(0, -23.0, -58.0) * Box(
            18.0, 8.0, 18.0, align=CTR)
        p = body + boss + shaft + connector
    else:
        raw = _imp_upgrade("6627T421")
        keep = [solid for solid in raw.solids() if solid.volume > 500.0]
        p = Compound(children=[Pos(0, 0, -28.0126) * solid
                               for solid in keep])
    p.label = label
    return p


def m4_stud(length=35.0, label="m4_stud") -> Part:
    p = Cylinder(2.0, length,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    p.label = label
    return p


def felt_washer(od=20.0, bore=4.5, thickness=3.0,
                label="felt_washer") -> Part:
    p = Cylinder(od / 2.0, thickness, align=CTR) - \
        Cylinder(bore / 2.0, thickness + 2.0, align=CTR)
    p.label = label
    return p


def m4_backing_washer(label="m4_backing_washer") -> Part:
    p = Cylinder(10.0, 1.0, align=CTR) - Cylinder(2.25, 3.0, align=CTR)
    p.label = label
    return p


def compression_spring_envelope(length=12.0,
                                label="compression_spring") -> Part:
    """Collision envelope for an OD10 compression spring around M4."""
    p = Cylinder(5.0, length, align=CTR) - Cylinder(2.35, length + 2,
                                                    align=CTR)
    p.label = label
    return p


def m4_wingnut_envelope(label="m4_wingnut") -> Part:
    hub = Cylinder(4.0, 4.0, align=CTR) - Cylinder(2.0, 6.0, align=CTR)
    wings = Box(20.0, 4.0, 3.0, align=CTR) - Cylinder(2.0, 5.0, align=CTR)
    p = hub + wings
    p.label = label
    return p


T8_AB_INSTALLED_LENGTH = 22.4
T8_AB_MAIN_LENGTH = 15.0
T8_AB_FLANGE_T = 4.0
T8_AB_BODY_D = 10.3
T8_AB_SECONDARY_FLANGE_D = 14.0
T8_AB_TIP_SLOT_DEPTH = 3.8


def t8_nut(label="t8_nut_main") -> Part:
    """Zyltech HW-SC-SMALLFL-P T8x8 main Delrin nut half.

    The local +Z axis is the screw axis, with the bracket-contacting outer
    flange face at z=0.  Dimensions are from the supplier drawing saved in
    ``tmp/zyltech_t8_drawing1.jpg``: 15 overall, 4 mm flange, Ø22 flange,
    Ø10.3 body, four M3 threaded holes on a Ø16 bolt circle, and a 3.8 mm
    deep anti-rotation/interlock tip.  Threads are represented by the
    conservative Ø8.2 clearance bore; the exact T8x8 thread is a purchasing
    contract rather than collision geometry.
    """
    body = Pos(0, 0, T8_AB_FLANGE_T) * Cylinder(
        T8_AB_BODY_D / 2.0,
        T8_AB_MAIN_LENGTH - T8_AB_FLANGE_T,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    flange = Cylinder(11.0, T8_AB_FLANGE_T,
                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    holes = []
    for a in (45, 135, 225, 315):
        x = 8 * math.cos(math.radians(a))
        y = 8 * math.sin(math.radians(a))
        holes.append(Pos(x, y, -1) * Cylinder(
            1.5, 6, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    p = body + flange - holes
    # The official view shows opposed 3.6 mm-wide slots over the final
    # 3.8 mm.  Cutting their diametric envelope avoids pretending that the
    # molded interlock is a solid collision wall.
    tip_slot = Pos(0, 0, T8_AB_MAIN_LENGTH - T8_AB_TIP_SLOT_DEPTH) * Box(
        T8_AB_BODY_D + 1.0, 3.6, T8_AB_TIP_SLOT_DEPTH + 1.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    p -= tip_slot
    bore = Cylinder(4.1, 40, align=CTR)
    p = p - bore
    p.label = label
    return p


def t8_nut_secondary(label="t8_nut_secondary") -> Part:
    """Zyltech anti-backlash secondary Delrin half at its installed pose.

    Supplier drawing 2 defines a 15 mm part with Ø14 x 4 mm flange,
    Ø10.3 body, and the complementary 3.8 mm interlock.  The selected
    22.4 mm overall installed envelope is the geometric minimum implied by
    two 15 mm halves with both 3.8 mm interlocks engaged.  Receipt inspection
    must confirm this configured envelope and spring preload before release.
    """
    z_end = T8_AB_INSTALLED_LENGTH
    z_flange = z_end - T8_AB_FLANGE_T
    z_body = z_end - T8_AB_MAIN_LENGTH
    body = Pos(0, 0, z_body) * Cylinder(
        T8_AB_BODY_D / 2.0,
        T8_AB_MAIN_LENGTH - T8_AB_FLANGE_T,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    flange = Pos(0, 0, z_flange) * Cylinder(
        T8_AB_SECONDARY_FLANGE_D / 2.0,
        T8_AB_FLANGE_T,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    tip_slot = Pos(0, 0, z_body - 0.5) * Box(
        3.6, T8_AB_BODY_D + 1.0, T8_AB_TIP_SLOT_DEPTH + 1.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    p = body + flange - tip_slot
    p -= Cylinder(4.1, 60, align=CTR)
    p.label = label
    return p


def t8_nut_spring_envelope(label="t8_nut_spring") -> Part:
    """Installed spring swept envelope between the two Delrin flanges.

    Zyltech supplies the spring but does not publish wire/rate dimensions.
    Ø14 overbounds the product photographs and the Ø10.3 inner cut clears
    both molded stems.  It is intentionally an annular collision envelope,
    not invented spring-rate or fatigue authority.
    """
    z0 = T8_AB_FLANGE_T
    length = T8_AB_INSTALLED_LENGTH - 2.0 * T8_AB_FLANGE_T
    p = Pos(0, 0, z0) * (
        Cylinder(T8_AB_SECONDARY_FLANGE_D / 2.0, length,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
        - Cylinder(T8_AB_BODY_D / 2.0 + 0.05, length + 1.0,
                   align=(Align.CENTER, Align.CENTER, Align.MIN))
    )
    p.label = label
    return p


def t8_screw(length=200.0, label="t8_screw") -> Part:
    p = Cylinder(4.0, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p.label = label
    return p


def gt2_pulley_40t_b5(label="gt2_40t_b5") -> Part:
    """NBK P40-2GT-BLP-6C-5 installed collision envelope.

    The verified step.parts tooth model supplies the true 40T/2 mm profile,
    Ø5 bore, and conservative Ø30.965 flange.  The selected NBK pulley is
    Ø30 flange by 10.3 overall with a 7 mm channel for the 6 mm belt; add
    0.15 mm annular end caps to the 10 mm reference model so its axial
    envelope cannot understate the orderable part.
    """
    if using_reference_envelopes():
        body = Cylinder(13.0, 10.0, align=CTR)
        rear_flange = Pos(0, 0, -5.15) * Cylinder(
            15.5, 0.15, align=(Align.CENTER, Align.CENTER, Align.MIN))
        front_flange = Pos(0, 0, 5.0) * Cylinder(
            15.5, 0.15, align=(Align.CENTER, Align.CENTER, Align.MIN))
        p = body + rear_flange + front_flange
        p -= Cylinder(2.5, 12.0, align=CTR)
    else:
        p = _copy("gt2_pulley_40t_bore5_w6")  # axis Z centered ±5
        cap = Cylinder(15.5, 0.15,
                       align=(Align.CENTER, Align.CENTER, Align.MIN)) - \
            Cylinder(2.5, 0.25,
                     align=(Align.CENTER, Align.CENTER, Align.MIN))
        p = p + Pos(0, 0, 5.0) * cap + Pos(0, 0, -5.15) * cap
    p.label = label
    return p


def beam_coupling_5x8(label="coupling_5x8") -> Part:
    """Conservative installed envelope for Ruland PCMR22-8-5-A.

    The exact selected clamp coupling is OD22.2 x 27 mm with 8 mm and 5 mm
    bores.  This verified reference STEP is OD24 x 32 mm, so retaining it for
    collision checks overbounds the supplier body by 0.9 mm radially and
    2.5 mm at each end.
    """
    if using_reference_envelopes():
        p = Cylinder(12.0, 32.0, align=CTR)
        p -= Cylinder(4.0, 34.0, align=CTR)
    else:
        p = Rot(0, 90, 0) * _copy("beam_coupling_bore5_to_8")
        # after Rot about Y, former X axis -> Z; envelope Ø24 x32
    p.label = label
    return p


# ---------------- datasheet-parametric envelopes ------------------------


@lru_cache(maxsize=None)
def _imp_upgrade(name: str) -> Compound:
    return import_step(str(MODELS.parent / "upgrades" / f"{name}.step"))


def mgn12_rail(length: float = 150.0, label="mgn12_rail") -> Part:
    """HIWIN MGN12R-150 reference rail corrected to the official hole table.

    The mirrored configurable vendor STEP has the first five holes on the
    required 25 mm pitch but places its final hole at local Z=+65 instead of
    +60 mm. For a 150 mm rail the selected HIWIN end distances are E1=10 mm
    and E2=15 mm, giving centers ``[-65,-40,-15,10,35,60]``. Preserve the
    authentic rail body, fill only that erroneous final bore, and cut the
    dimensioned M3 through-hole/counterbore at +60. See mgn12.report.md.

    Raw frame: X centered, Y=-5.5..+2.5, Z=-75..+75.
    """
    if abs(length - 150.0) > 1e-6:
        raise ValueError("corrected MGN12 rail source currently supports 150 mm only")

    if using_reference_envelopes():
        # Same shared frame as the cached reference: X centered, Y from
        # -5.5 to +2.5, and rail length along Z about zero.
        p = Pos(0, -1.5, 0) * Box(12.0, 8.0, length, align=CTR)
        for z in (-65.0, -40.0, -15.0, 10.0, 35.0, 60.0):
            through = Pos(0, 3.0, z) * (Rot(90, 0, 0) * Cylinder(
                1.75, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN)))
            counterbore = Pos(0, 3.0, z) * (Rot(90, 0, 0) * Cylinder(
                3.0, 5.0, align=(Align.CENTER, Align.CENTER, Align.MIN)))
            p -= through + counterbore
        p.label = label
        return p

    p = _imp_upgrade("mgn12r_rail")

    # Copy an unperforated 6.2 mm wafer of the authentic rail profile from
    # local Z=55 to Z=65. Unlike a cylindrical plug, this preserves the small
    # underside reliefs and cannot add material outside the vendor envelope.
    source_wafer = p & (
        Pos(0, 0, 55.0) * Box(
            20.0, 20.0, 6.2, align=(Align.CENTER, Align.CENTER, Align.CENTER)
        )
    )
    fill_bad_hole = Pos(0, 0, 10.0) * source_wafer
    through_hole = Pos(0, 2.6, 60.0) * (
        Rot(90, 0, 0) * Cylinder(
            1.75, 8.2, align=(Align.CENTER, Align.CENTER, Align.MIN)
        )
    )
    counterbore = Pos(0, 2.6, 60.0) * (
        Rot(90, 0, 0) * Cylinder(
            3.0, 4.6, align=(Align.CENTER, Align.CENTER, Align.MIN)
        )
    )
    p = (p + fill_bad_hole) - through_hole - counterbore
    p.label = label
    return p


def mgn12h_block_real(label="mgn12h") -> Part:
    """REAL HIWIN MGN12H, rebased to its 20x20 mounting-grid center.

    The vendor occurrence sits at Z=-26.033 on its sample rail.  Assembly
    placement is defined by the carriage axis, so keeping that occurrence
    offset misaligned every plate screw and ran the block off the rail at
    deep travel.  Y remains in the shared rail frame; local Z=0 is now the
    block mounting-grid center.
    """
    if using_reference_envelopes():
        # Match the cached occurrence frame: X is width, Y is height above
        # the rail datum, and Z runs along the rail through the 20x20 grid.
        p = Pos(0, 2.5, 0) * Box(27.0, 10.0, 45.4, align=CTR)
        for x in (-10.0, 10.0):
            for z in (-10.0, 10.0):
                p -= Pos(x, 8.0, z) * (Rot(90, 0, 0) * Cylinder(
                    1.7, 12.0,
                    align=(Align.CENTER, Align.CENTER, Align.MIN)))
    else:
        p = Pos(0, 0, 26.033) * _imp_upgrade("mgn12h_block")
    p.label = label
    return p


def mgn12h_block(label="mgn12h") -> Part:
    """HIWIN MGN12H: L=45.4 (with seals), W=27, assembled H=13 over rail
    bottom; block bottom sits 3 mm above rail bottom (envelope: solid block
    from z=3..13 over W27, with 20x20 M3 grid in top). Local frame matches
    the rail's: z=0 at RAIL bottom, X along rail, centered X/Y."""
    body = Box(45.4, 27, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))
    body = Pos(0, 0, 3) * body
    holes = []
    for dx in (-10, 10):
        for dy in (-10, 10):
            holes.append(Pos(dx, dy, 7) * Cylinder(
                1.25, 8, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    p = body - holes
    p.label = label
    return p


def er11_chuck_c8(shank_len=100.0, label="er11_c8"):
    """C8-ER11A-{len}L straight-shank collet chuck. step.parts MISS
    (searched 'ER11', 'collet' 2026-07-08) -> documented envelope from
    vendor drawings: shank Ø8 x shank_len, head Ø18 x16, nut Ø19 x19
    (conservative outer envelope incl. collet nose).
    Local frame: axis +Z, nut TOP face at z=0, shank extends -Z."""
    nut = Cylinder(9.5, 19, align=(Align.CENTER, Align.CENTER, Align.MAX))
    head = Pos(0, 0, -19) * Cylinder(
        9.0, 16, align=(Align.CENTER, Align.CENTER, Align.MAX))
    shank = Pos(0, 0, -35) * Cylinder(
        4.0, shank_len, align=(Align.CENTER, Align.CENTER, Align.MAX))
    p = nut + head + shank
    # collet bore: gripped stator shafts (3-8 mm) sit inside, not in solid
    p -= Pos(0, 0, 1) * Cylinder(4.1, 26,
                                 align=(Align.CENTER, Align.CENTER, Align.MAX))
    p.label = label
    return p


@lru_cache(maxsize=1)
def _profile_2020_face():
    """True B-type slot-6 cross-section (174.3 mm^2), sectioned from the
    step.parts sample profile_2020_b_slot6_a_l50 (sha256-verified)."""
    from build123d import section, Plane
    sample = _imp("profile_2020_b_slot6_a_l50")   # runs -Y, section in XZ
    return section(sample, section_by=Plane.XZ.offset(25))


def extrusion_2020(length: float, label="2020") -> Part:
    """2020 T-slot with the REAL B-type profile; runs +Z 0..length,
    centered XY. Falls back to a grooved-box envelope if the sample or
    section op is unavailable."""
    try:
        from build123d import extrude
        face = _profile_2020_face()
        p = extrude(face, amount=-length)      # section plane faces -Y
        p = Rot(90, 0, 0) * p                  # +Y run -> +Z run
        bb = p.bounding_box()
        p = Pos(0, 0, -bb.min.Z) * p           # base at z=0
    except Exception:
        p = Box(20, 20, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
        for ang in (0, 90, 180, 270):
            groove = Box(6.2, 4, length + 2,
                         align=(Align.CENTER, Align.MIN, Align.MIN))
            groove = Rot(0, 0, ang) * (Pos(0, 8, -1) * groove)
            p = p - groove
    p.label = label
    return p


def endstop(label="endstop") -> Part:
    """Omron D2F-01L2-D3 controlled-drawing collision geometry.

    Local body is centered in X/Y with its bottom at Z=0. The hinge roller
    extends toward +Z; assembly rotates this frame 180 degrees about Y so the
    roller faces the moving home flag. Body, 6.50 pitch mounting holes,
    solder terminals, lever and OD4.8 roller are all explicit. Cosmetic case
    seams and internal contacts are intentionally omitted.
    """
    body = Box(12.8, 5.8, 6.5,
               align=(Align.CENTER, Align.CENTER, Align.MIN))
    for x in (-3.25, 3.25):
        hole = Rot(90, 0, 0) * Cylinder(
            1.06, 7.8, align=(Align.CENTER, Align.CENTER, Align.CENTER)
        )
        body -= Pos(x, 0.0, 3.15) * hole

    # Three -D3 solder lugs on the 5.08 mm terminal pitch. They touch the
    # body underside and conservatively reserve the wiring/solder envelope.
    terminals = None
    for x in (-5.08, 0.0, 5.08):
        lug = Pos(x, 0.0, -1.7) * Box(
            1.2, 0.8, 3.4,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        terminals = lug if terminals is None else terminals + lug

    lever = Pos(0.0, 0.0, 11.4) * Box(
        4.0, 0.30, 9.8,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    roller = Pos(0.0, 0.0, 16.5) * (
        Rot(90, 0, 0) * Cylinder(
            2.4, 2.8, align=(Align.CENTER, Align.CENTER, Align.CENTER)
        )
    )
    p = body + terminals + lever + roller
    p.label = label
    return p


def gt2_pulley_40t_b8(label="gt2_40t_b8") -> Part:
    """REAL GT2 40T bore-8 pulley STEP (verified smallparts.report.md).
    Raw: axis +X 0..17, Ø28 flanges. Rebased local: axis +Z, z 0..17."""
    p = Rot(0, -90, 0) * _imp_upgrade("gt2_40t_b8")
    bb = p.bounding_box()
    p = Pos(0, 0, -bb.min.Z) * p
    p.label = label
    return p


def er11_chuck_c8_hifi(label="er11_c8"):
    """High-fidelity ER11-A chuck (drawing-faithful; er11.report.md).
    Same drop-in frame as er11_chuck_c8: axis +Z, nut top z=0, shank -Z."""
    if using_reference_envelopes():
        from models.upgrades.er11_c8_hifi import er11_chuck_c8 as source_model
        p = source_model(label=label)
    else:
        p = _imp_upgrade("er11_c8_hifi")
    p.label = label
    return p


def ceramic_eyelet(bore_r=2.0, ring_r=4.5, t=3.0, label="eyelet") -> Part:
    """Drawing-controlled polished alumina fixed guide; axis ``+Z``.

    The OD and thickness are press-seat interfaces.  Both bore rims carry the
    release drawing's R0.75 wire-contact blend; this is no longer a generic
    square-edged fishing-guide placeholder.
    """
    p = Cylinder(ring_r, t, align=CTR) - Cylinder(bore_r, t + 2, align=CTR)
    bore_edges = [
        edge for edge in p.edges()
        if edge.geom_type.name == "CIRCLE"
        and abs(edge.length - 2.0 * math.pi * bore_r) < 1e-5
    ]
    if len(bore_edges) != 2:
        raise RuntimeError(
            f"fixed ceramic guide expected two bore rims, got {len(bore_edges)}"
        )
    p = p.fillet(0.75, bore_edges)
    p.label = label
    return p


def ceramic_toroid_guide(major_r=6.5, tube_r=3.0,
                          label="tip_toroid_guide") -> Part:
    """Polished ceramic flyer fairlead, local symmetry axis ``+Z``.

    Unlike a flat eyelet, the torus provides a real smooth surface for the
    flyer's 120..150 degree wire reversal.  The release part is a custom
    99.8% alumina RFQ item; ``wire_geometry.tip_guide_spec`` is the shared
    dimensional and finish contract.
    """
    p = Torus(major_r, tube_r, align=CTR)
    p.label = label
    return p


def ceramic_shaft_wrap_sleeve(stator,
                               label="shaft_wrap_sleeve") -> Part:
    """Seamless radiused ceramic sleeve, local shaft axis ``+Z``.

    This is the KEIR-Series-45-class configured phase-lead contact sleeve,
    not a set-screw collar.  Both exposed OD rims are modeled at the RFQ's
    R0.75 minimum; the wire contacts the cylindrical mid-band.
    """
    import wire_geometry

    spec = wire_geometry.shaft_wrap_sleeve_spec(stator)
    ro = float(spec["outer_diameter_mm"]) / 2.0
    ri = float(spec["bore_diameter_mm"]) / 2.0
    length = float(spec["length_mm"])
    p = (Cylinder(ro, length, align=CTR)
         - Cylinder(ri, length + 2.0, align=CTR))
    outer_edges = [edge for edge in p.edges()
                   if edge.geom_type.name == "CIRCLE"
                   and abs(edge.length - 2.0 * math.pi * ro) < 1e-5]
    p = p.fillet(float(spec["minimum_edge_radius_mm"]), outer_edges)
    p.label = label
    return p
