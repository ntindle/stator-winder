"""All 3D-printed parts, modeled IN-PLACE in machine coordinates at the
reference pose: M0 = 0 (carriage at HOME, stator axis z = m0_home_standoff),
M1 = 0, M2 = 0 (flyer arm at 12 o'clock, +Y).

Design note (documented deviation from part-local-origin practice): parts are
authored in machine coordinates so the assembly, digital twin, and collision
links are pure axis transforms of these solids. STL export re-orients each
part to its print orientation (see export_stls.py). All interfaces are
parametric from params.PARAMS.

Machine frame: Z = flyer axis (Z=0 flyer plane, +Z toward carriage), Y up.
"""

import math
from pathlib import Path
from build123d import (Part, Box, Cone, Cylinder, Sphere, Pos, Rot, Align, BuildLine,
                       BuildSketch, Circle, Line, Plane, Polyline, RadiusArc,
                       Transition, Polygon, Torus, extrude, sweep)

from params import PARAMS as P, StatorSpec
import wire_geometry

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
ZMIN = (Align.CENTER, Align.CENTER, Align.MIN)


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * \
        Box(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), align=CTR)


def _cyl_z(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, abs(z1 - z0), align=CTR)


def _cyl_y(r, y0, y1, x=0.0, z=0.0):
    c = Cylinder(r, abs(y1 - y0), align=CTR)
    return Pos(x, (y0 + y1) / 2, z) * (Rot(90, 0, 0) * c)


def _hex_z(across_flats, z0, z1, x=0.0, y=0.0):
    """Regular-hex clearance prism on +Z, sized across flats."""
    radius = across_flats / math.sqrt(3.0)
    points = [
        (radius * math.cos(math.radians(30.0 + 60.0 * i)),
         radius * math.sin(math.radians(30.0 + 60.0 * i)))
        for i in range(6)
    ]
    section = Polygon(*points, align=None)
    prism = extrude(Plane.XY * section, amount=abs(z1 - z0) / 2.0,
                    both=True)
    return Pos(x, y, (z0 + z1) / 2.0) * prism


def _m5_flush_countersink_z(x, y, surface_z=-164.0):
    """ISO 10642 M5 printable-clearance seat, opening toward machine +Z."""
    depth = (10.0 - 5.4) / 2.0
    return Pos(x, y, surface_z - depth) * Cone(
        2.7, 5.0, depth, align=ZMIN,
    )


def _bar_xy(start, end, width, z0, z1):
    """Rectangular bridge between two XY points over a Z thickness."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    angle = math.degrees(math.atan2(dy, dx))
    local = Box(length, width, abs(z1 - z0),
                align=(Align.MIN, Align.CENTER, Align.MIN))
    return Pos(start[0], start[1], min(z0, z1)) * (Rot(0, 0, angle) * local)


def _cyl_x(r, x0, x1, y=0.0, z=0.0):
    c = Cylinder(r, abs(x1 - x0), align=CTR)
    return Pos((x0 + x1) / 2, y, z) * (Rot(0, 90, 0) * c)


def _poly_tube(points, radius, label="tube") -> Part:
    """Single swept tube following a sampled machine-frame centerline."""
    if len(points) < 2:
        raise ValueError("tube path requires at least two points")
    clean = [tuple(float(v) for v in point) for point in points]
    direction = tuple(clean[1][i] - clean[0][i] for i in range(3))
    with BuildLine() as path:
        Polyline(*clean)
    with BuildSketch(Plane(origin=clean[0], z_dir=direction)) as profile:
        Circle(radius)
    # TRANSFORMED is robust for tight internal channels; the analytical
    # centerline already contains the required tangent circular fillets.
    result = sweep(profile.sketch, path.line,
                   transition=Transition.TRANSFORMED)
    result.label = label
    return result


# ======================= M2 flyer module ==================================

def flyer_block() -> Part:
    """Flyer bearing block: front plate bolted to the two tower posts'
    front faces (z = post_z[1] = -40), bearing tube rearward to z -80.
    Bore Ø28 seats 2x 6001ZZ; Ø34 clearance hole through the plate for the
    rotating hub collar."""
    zf = P.post_z[1]                      # -40 post front face
    # 9 mm plate: front face at -31 gives 3 mm to the spoke plane (-28)
    plate = _box(-P.post_x - 10, P.post_x + 10, -25, 25, zf, zf + 9)
    tube = _cyl_z(19.0, -80, zf)          # Ø38 tube
    p = plate + tube
    # Full outer-race retention stack.  The rear DIN472-28 ring sits in the
    # Ø29.4 groove; rear bearing, outer spacer and front bearing share the
    # Ø28.1 precision bore.  A 2 mm front shoulder captures the front outer
    # race while the Ø25 through-clearance passes the rotating inner stack.
    p -= _cyl_z(13.0, -81, -29)           # Ø26 running clearance
    p -= _cyl_z(14.05, -75.0, -48.0)      # Ø28.1 bearing/spacer bore
    p -= _cyl_z(14.70, -76.3, -75.0)      # DIN472-28 groove
    p -= _cyl_z(17.0, -46.0, zf + 11)     # Ø34 hub clearance through plate
    # post bolt holes: 2 per side, M5 into T-nuts (vertical pair)
    for sx in (-1, 1):
        for dy in (-12, 12):
            p -= _cyl_z(2.7, zf - 22, zf + 11, x=sx * P.post_x, y=dy)
    p.label = "flyer_block"
    return p


def flyer_arm() -> Part:
    """One-piece flyer: hub collar (clamps Ø12 tube), spoke web, axial
    finger, eyelet boss at tip, counterweight boss opposite. At M2=0 the
    finger/eyelet is at +Y (12 o'clock)."""
    hz0, hz1 = P.hub_z
    sz0, sz1 = P.spoke_z
    w, t = P.flyer_arm_w, P.flyer_arm_t
    R = P.flyer_tip_r

    # collar must stay inside the flyer_block plate bore (Ø34) it rotates
    # through: no external clamp boss. Two radial M3 grub screws (heat-set
    # inserts) onto filed flats on the tube carry the (tiny) arm drag.
    # collar runs all the way to the spoke rear face (sz0) so collar,
    # spoke, finger and CW boss are ONE connected body; r14 stays inside
    # the block's Ø34 bore with 3 mm radial clearance
    collar = _cyl_z(14.0, hz0, sz0) - _cyl_z(6.05, hz0 - 1, sz0 + 1)
    zm = (hz0 + hz1) / 2
    collar -= _cyl_y(2.0, -15, 0, z=zm)            # grub 1 (-Y)
    collar -= _cyl_x(2.0, 0, 15, z=zm)             # grub 2 (+X)

    # spoke: from collar out to tip radius, at +Y (no gusset: a gusset
    # corner would orbit inside the block's Ø34 bore with <0.5 mm; the
    # 14x8 web section alone is ample for a ~30 g arm)
    spoke = _box(-w / 2, w / 2, 0, R + w / 2, sz0, sz1)
    # finger: axial run from spoke to eyelet plane, radially w x t
    # Stop at the rear of the torus instead of spearing through its contact
    # surface; the rear cradle below provides the remaining connection.
    finger = _box(-w / 2, w / 2, R - t / 2, R - 3.0, sz1 - 1, -2)
    # Rear cradle for the polished ceramic torus.  The fairlead's complete
    # front half remains exposed for the physical wire reversal; a concave
    # R3.15 epoxy seat supports only its rear surface.  The Ø8 open throat
    # clears every analytically validated approach tangent without presenting
    # a lip to the enamel.
    guide = wire_geometry.tip_guide_spec()
    gz = guide["center_local_mm"][2]
    boss2 = _cyl_y(11.5, R - 5.0, R - 2.35, z=gz)
    boss2 -= _cyl_y(4.0, R - 6.0, R - 1.5, z=gz)
    seat = Pos(0, R, gz) * (Rot(-90, 0, 0) * Torus(6.5, 3.15))
    boss2 -= seat
    # Counterweight boss at -Y.  A brass M3 heat-set insert is installed from
    # the front (z -18.3..-14); the remainder is Ø3.4 screw clearance.  The old
    # Ø6.5 through-hole and fictitious M6 slug overstated counterweight mass.
    # The 17.4 mm rear reach is mass-property tuned for the released
    # three-washer stack; it remains inside the larger tip-guide sweep.
    cw_outer_y = -P.counterweight_r - 17.4
    cw = _box(-11, 11, cw_outer_y, -10, sz0, sz1 + 6)
    cw -= _cyl_z(1.70, sz0 - 1, -18.3, y=-P.counterweight_r)
    cw -= _cyl_z(2.00, -18.3, -14.0, y=-P.counterweight_r)

    # A dedicated negative-Y web makes the boss a volumetric continuation of
    # the hub, rather than relying on the old coincident rear faces.  It
    # overlaps the collar by 0.8 mm in Z and the boss by 15 mm in Y.  Its
    # inner edge stays outside the Ø12.1 hub passage, while its rear face is
    # 2.2 mm forward of the stationary flyer-block face at z=-31.  The two
    # misleading unloaded trim bores have been removed; final balance is set
    # only by the retained central M3 washer stack.
    cw_web_inner_y = -(6.05 + P.min_wall)  # 2.4 mm wall outside Ø12.1
    cw_web = _box(-w / 2, w / 2, -P.counterweight_r,
                  cw_web_inner_y, sz0 - 0.8, sz1)

    p = collar + spoke + finger + boss2 + cw_web + cw
    # ``cw_web`` is united after the boss-local holes above. Recut the
    # complete counterweight fastener passage through the final union so the
    # web cannot silently refill the M3 clearance bore.
    p -= _cyl_z(1.70, sz0 - 1, -18.3, y=-P.counterweight_r)
    p -= _cyl_z(2.00, -18.3, -14.0, y=-P.counterweight_r)
    # Recut the ceramic envelope after the cradle is united with the finger
    # and spoke.  Cutting only ``boss2`` let the torus intersect those parent
    # solids when the guide plane moved rearward for the 20 mm-stack shaft
    # clearance.  The additional 0.20 mm is the epoxy bond line.
    guide_clearance = Pos(0, R, gz) * (
        Rot(-90, 0, 0) * Torus(6.5, 3.20)
    )
    p -= guide_clearance
    # wire window through the spoke web: the elbow->tip wire run crosses
    # the web at y 10..24 (see wirepath.py); leave 3.5 mm side rails
    p -= _box(-3.5, 3.5, -1, 27, sz0 - 2, sz1 + 2)
    # Open approach throat from the spoke side into the torus cradle.
    p -= _cyl_y(4.0, R - 12.0, R - 1.5, z=gz)
    # During both captured two-turn shaft wraps the taut wire crosses the
    # finger's front/top corner by 0.21 mm (the two directions are mirror
    # images).  Open only the central 6 mm of that last millimetre; the two
    # 4 mm side rails remain continuous and the relief gives the maximum
    # 0.5 mm wire a manufacturing-tolerant passage without moving the torus.
    p -= _box(-3.0, 3.0, R - 4.1, R - 2.9, -3.0, 0.0)
    p.label = "flyer_arm"
    return p


def flyer_pulley() -> Part:
    """Printed GT2 40T pulley with the REAL 2mm-pitch tooth profile
    (models/upgrades/gt2_profile.py, dimensionally verified: tooth OD
    24.96). Keeps the mounting contract: bore Ø12.05 clamp-on, rear
    flange r14.5, front flange r11.5 (enters block bore contact-free),
    OD20.8 hub with radial M3. The 0.1 mm hub-radius reduction preserves
    clamp wall while giving the adjacent DIN472 ring >2 mm running clearance.
    """
    import sys
    up = str(Path(__file__).parent / "models" / "upgrades")
    if up not in sys.path:
        sys.path.insert(0, up)
    from gt2_profile import gt2_pulley
    z0, z1 = P.pulley_z
    # gt2_pulley local: teeth band z 0..width, rear flange below 0
    p = Pos(0, 0, z0) * gt2_pulley(teeth=40, bore_d=12.05,
                                   width=abs(z1 - z0),
                                   flange_r_rear=14.5, flange_r_front=11.5,
                                   hub_len=6.5, hub_r=10.4)
    # Short McMaster 94459A769 insert fits the radial wall: Ø4.0 pilot from
    # the hub OD, then the generator's Ø3.4 clearance toward the tube.  The
    # 1.075 mm closed end behind the insert prevents breakthrough.
    clamp_z = z0 + abs(z1 - z0) + 1.5 + 6.5 / 2.0
    p -= _cyl_y(2.0, -10.4, -7.0, z=clamp_z)
    p.label = "flyer_pulley"
    return p


def m2_motor_mount() -> Part:
    """Bracket from the rear posts carrying McMaster 6627T421 NEMA17 M2.

    The standard NEMA17 interface is a Ø22 pilot and four M3 screws on a
    31 mm square.  Vertical slots give the 200-2GT loop its small tensioning
    adjustment without changing the required 1:1 40T:40T transmission.
    """
    zr = P.post_z[0]                       # -60
    my = P.m2_motor_axis_y                 # -60
    zf = P.m2_motor_face_z                 # -100
    # two side arms from posts to the compact NEMA17 motor plate
    plate_half = 27.0
    plate = _box(-plate_half, plate_half, my - plate_half,
                 my + plate_half, zf, zf + 6)
    plate -= _cyl_z(11.25, zf - 1, zf + 7, y=my)       # Ø22.5 pilot clearance
    hg17 = 31.0 / 2
    for dx in (-hg17, hg17):
        for dy in (my - hg17, my + hg17):
            p_slot = _box(dx - 1.8, dx + 1.8, dy - 5, dy + 5,
                          zf - 1, zf + 7)
            plate -= p_slot
    arm_l = _box(-P.post_x - 10, -plate_half, my - plate_half,
                 my + plate_half, zf, zf + 6)
    arm_r = _box(plate_half, P.post_x + 10, my - plate_half,
                 my + plate_half, zf, zf + 6)
    # riser webs up the post rear faces
    web_l = _box(-P.post_x - 10, -P.post_x + 10, my - 33, 20, zr - 6, zr)
    web_r = _box(P.post_x - 10, P.post_x + 10, my - 33, 20, zr - 6, zr)
    join_l = _box(-P.post_x - 10, -P.post_x + 4,
                  my - plate_half, my + plate_half, zf, zr)
    join_r = _box(P.post_x - 4, P.post_x + 10,
                  my - plate_half, my + plate_half, zf, zr)
    p = plate + arm_l + arm_r + web_l + web_r + join_l + join_r
    # bolt to posts (M5 T-nuts), through the riser webs
    for sx in (-1, 1):
        for dy in (my - 12, 8):
            p -= _cyl_z(2.7, zr - 7, zr + 1, x=sx * P.post_x, y=dy)
    # Rear tool/head access for the low post pair.  Their bearing plane is
    # z=-66 but the join blocks continue to z=-102; these Ø10 tunnels make
    # the installed screws reachable without weakening the 6 mm post webs.
    for x in (-P.post_x, P.post_x):
        p -= _cyl_z(5.0, -103.0, -66.0,
                    x=x, y=P.m2_motor_axis_y - 12.0)
    p.label = "m2_motor_mount"
    return p


def wire_elbow() -> Part:
    """PTFE/printed guide press-fit in the flyer tube front end.

    An exact tangent R5 quarter-turn redirects the axial bore onto the radial
    feed toward the toroidal tip guide.  The complete part rotates with M2.
    """
    zf = P.flyer_shaft_front_z
    sleeve = _cyl_z(P.wire_elbow_sleeve_r, zf - 8, zf + 0.5)
    # A true R5 quarter-turn brings the wire onto a radial straight feed before
    # the toroidal tip guide.  The former diagonal horn ended at a flat eyelet
    # and concealed a 120..150 degree kink.
    path = wire_geometry.flyer_path_spec()
    bend = path["elbow_bend"]
    bend_start_y, bend_start_z = bend["start"][1:]
    bend_exit_y, bend_exit_z = bend["end"][1:]
    # Build the exact R5 quarter-torus and its tangent straight runs as one
    # analytic sweep.  Keeping the G1-continuous centreline in a single pipe
    # avoids the tiny torus/cylinder Boolean seam faces that OCC could export
    # as a valid BREP but omit from the STL tessellation.
    def elbow_tube(radius: float, axial_start_z: float,
                   radial_end_y: float) -> Part:
        with BuildLine(Plane.YZ) as centerline:
            Line((bend_start_y, axial_start_z),
                 (bend_start_y, bend_start_z))
            RadiusArc((bend_start_y, bend_start_z),
                      (bend_exit_y, bend_exit_z), bend["radius"])
            Line((bend_exit_y, bend_exit_z),
                 (radial_end_y, bend_exit_z))
        start_plane = Plane(
            origin=(0.0, bend_start_y, axial_start_z),
            x_dir=(1.0, 0.0, 0.0),
            z_dir=(0.0, 0.0, 1.0),
        )
        with BuildSketch(start_plane) as profile:
            Circle(radius)
        return sweep(profile.sketch, centerline.wire())

    body_r = wire_geometry.FLYER_ELBOW_BODY_RADIUS
    outer_tube = elbow_tube(
        body_r, zf - 7.5, wire_geometry.TIP_GUIDE_FEED_Y + 2.0,
    )
    outer = sleeve + outer_tube

    # Overshoot both open ends of the Ø3.2 channel.  The subtraction therefore
    # opens cleanly into the flyer shaft and beyond the radial outlet without
    # relying on coincident end faces.
    inner_tube = elbow_tube(
        1.6, zf - 9.0, wire_geometry.TIP_GUIDE_FEED_Y + 3.0,
    )
    p = outer - inner_tube
    p.label = "wire_elbow"
    return p


# ======================= M0/M1 carriage module ============================

def carriage_plate() -> Part:
    """Carries both MGN12H blocks, the spindle tower, the hanging M1 motor
    (through a 44x44 window) and the T8 nut bracket. Modeled at HOME pose
    (spindle axis at z = m0_home_standoff). Local plate spans z +/-45 about
    the spindle axis, x -88..+58."""
    zc = P.m0_home_standoff
    y1 = P.plate_top_y
    y0 = y1 - P.plate_t
    p = _box(-88, 60, y0, y1, zc - 45, zc + 45)
    p -= _box(-22, 22, y0 - 1, y1 + 1, zc - 22, zc + 22)   # motor window
    # MGN12H bolt grids (M3) at rail_x
    for sx in (-1, 1):
        for dz in (-10, 10):
            for dx in (-10, 10):
                p -= _cyl_y(1.7, y0 - 1, y1 + 1, x=sx * P.rail_x + dx,
                            z=zc + dz)
    # tower bolts (M4 x4 on 62x62)
    for dx in (-31, 31):
        for dz in (-31, 31):
            p -= _cyl_y(2.2, y0 - 1, y1 + 1, x=dx, z=zc + dz)
    # nut bracket bolts (M4 x2)
    for dz in (2, 12):
        p -= _cyl_y(2.2, y0 - 1, y1 + 1, x=-78, z=zc + dz)
    # relief cutout: T8 nut flange (r11 at screw_y) dips below plate top
    p -= _box(-82, -58, y0 - 1, y1 + 1, zc - 24, zc - 16)
    # endstop trigger tab hanging below the plate at the rear edge
    p += _box(-8, 8, y0 - 6, y0, zc + 39, zc + 45)
    # Rear-left notch clears both the M0 coupling line and the fixed-end
    # mount's stringer web at home/homing overtravel.  It begins behind the
    # rear MGN block screws, so the two block grids and tower grid stay intact.
    p -= _box(-89, -36, y0 - 7, y1 + 1, zc + 20, zc + 46)
    p.label = "carriage_plate"
    return p


def spindle_tower() -> Part:
    """Portal tower on the carriage plate: base flange straddling the motor
    window, twin columns, integral bearing tube (2x 608ZZ) holding the
    spindle. The M1 NEMA17 bolts UP into the flange: 43.5^2 body pocket
    2 mm deep from below, Ø23 boss bore, M3 grid 31x31. Modeled at HOME
    pose."""
    zc = P.m0_home_standoff
    yb = P.plate_top_y
    brg_top = P.spindle_brg_top_y                     # -95
    tube_top = brg_top + 3
    tube_bot = brg_top - 7 - P.spindle_brg_gap - 7 - 3   # 2 brgs + gap + lips
    R = P.spindle_housing_r

    flange = _box(-38, 38, yb, yb + 6, zc - 38, zc + 38)
    # M1 motor interface (motor flange face at m1_motor_top_y = yb + 2)
    flange -= _box(-21.8, 21.8, yb - 1, P.m1_motor_top_y, zc - 21.8,
                   zc + 21.8)                          # body pocket
    flange -= _cyl_y(11.5, yb - 1, yb + 7, z=zc)       # Ø23 boss bore
    hg = 31.0 / 2
    for dx in (-hg, hg):
        for dz in (-hg, hg):
            flange -= _cyl_y(1.7, yb - 1, yb + 7, x=dx, z=zc + dz)
    col_f = _box(-30, 30, yb, tube_bot + 14, zc - 38, zc - 26)
    col_r = _box(-30, 30, yb, tube_bot + 14, zc + 26, zc + 38)
    tube = _cyl_y(R, tube_bot, tube_top, z=zc)
    bridge_f = _box(-R, R, tube_bot + 4, tube_top, zc - 30, zc)
    bridge_r = _box(-R, R, tube_bot + 4, tube_top, zc, zc + 30)
    p = flange + col_f + col_r + tube + bridge_f + bridge_r
    # bearing bores: Ø22 seats, Ø18 waist
    p -= _cyl_y(11.0, brg_top - 7, brg_top + 4, z=zc)
    p -= _cyl_y(11.0, tube_bot - 1, brg_top - 7 - P.spindle_brg_gap, z=zc)
    p -= _cyl_y(11.05, -118.0, -102.0, z=zc)  # Ø22.1 outer spacer seat
    p -= _cyl_y(9.0, tube_bot - 1, tube_top + 1, z=zc)
    # DIN472-22 grooves capture both outer races.  The lower ring is below
    # y=-125; the upper ring is above y=-95, leaving the 16 mm outer spacer
    # between the two 608 bearings fully constrained.
    p -= _cyl_y(11.5, -126.1, -125.0, z=zc)
    p -= _cyl_y(11.5, -95.0, -93.9, z=zc)
    # flange bolts M4
    for dx in (-31, 31):
        for dz in (-31, 31):
            p -= _cyl_y(2.2, yb - 1, yb + 7, x=dx, z=zc + dz)
            # The columns begin at x=±30 and otherwise clip the adjacent
            # half of each M4 head above the flange.  A shallow OD7.6 scallop
            # preserves the flange bearing face and clears that column edge.
            p -= _cyl_y(3.8, yb + 6, yb + 11, x=dx, z=zc + dz)
    # The four inboard MGN12H M3 heads sit directly below this flange.
    # Printable Ø6.4 x 3.25-deep underside pockets clear their Ø5.68 x
    # 3.0 heads while retaining a 2.75 mm roof (above the 2.4 mm wall rule).
    for x in (-35.0, 35.0):
        for z in (zc - 10.0, zc + 10.0):
            p -= _cyl_y(3.2, yb - 0.5, yb + 3.25, x=x, z=z)
    p.label = "spindle_tower"
    return p


def nut_bracket() -> Part:
    """Riser from the plate carrying the T8 flange nut at the screw axis
    (screw_x, screw_y). Nut axis along Z, flange bolts to the bracket's
    +Z face. Modeled at HOME pose."""
    zc = P.m0_home_standoff
    y1 = P.plate_top_y
    # Keep the vertical T8 wall, but replace the old full-width 8 mm foot
    # (which buried the two lower flange stacks) with a printable 2.4 mm web
    # and a local 8 mm-high rail under the two M4 plate screws.
    wall = _box(-84, -60, y1, P.screw_y + 14, zc - 18, zc - 10)
    web = _box(-84, -60, y1, y1 + 2.4, zc - 10, zc + 14)
    boss = _box(-84, -72, y1, y1 + 8, zc - 3, zc + 17)
    p = wall + web + boss
    p -= _cyl_z(5.5, zc - 19, zc - 9, x=P.screw_x, y=P.screw_y)
    lower_holes = []
    for a in (45, 135, 225, 315):
        hx = P.screw_x + 8 * math.cos(math.radians(a))
        hy = P.screw_y + 8 * math.sin(math.radians(a))
        p -= _cyl_z(1.6, zc - 19, zc - 9, x=hx, y=hy)
        if a in (225, 315):
            lower_holes.append((hx, hy))
    # Ø7.2 access channels let the lower washer/nyloc stacks occupy the
    # z=85..92 volume without weakening the local M4 boss rail at z>=92.
    for hx, hy in lower_holes:
        p -= _cyl_z(3.6, zc - 10, zc - 3, x=hx, y=hy)
    for dz in (2, 12):
        p -= _cyl_y(2.2, y1 - 8, y1 + 9, x=-78, z=zc + dz)
    p.label = "nut_bracket"
    return p


def m0_motor_mount() -> Part:
    """L-bracket on the -X base rail carrying the M0 NEMA17 (axis along Z,
    face at m0_motor_z, body +Z behind)."""
    x, y = P.screw_x, P.screw_y
    zf = P.m0_motor_z
    plate = _box(x - 24, x + 24, y - 24, y + 24, zf - 6, zf)
    plate -= _cyl_z(11.5, zf - 7, zf + 1, x=x, y=y)
    hg = 31.0 / 2
    for dx in (-hg, hg):
        for dy in (-hg, hg):
            plate -= _cyl_z(1.7, zf - 7, zf + 1, x=x + dx, y=y + dy)
    # foot stays behind the motor-face plane; motor hangs on its flange
    # True perpendicular foot along +Z.  A prior truncation ended it at the
    # motor plate, leaving both extrusion holes in empty space.
    # Foot sits only over the left stringer; keeping it outboard of the
    # motor envelope prevents the encoder body from passing through it.
    foot_x = -42.0
    foot = _box(-48.0, -32.0,
                P.stringer_top_y, P.stringer_top_y + 8,
                zf - 6, zf + 34)
    web1 = _box(-48.0, -42.0,
                P.stringer_top_y, y + 24, zf - 6, zf)
    web2 = _box(-40.0, -34.0,
                P.stringer_top_y, y + 24, zf - 6, zf)
    p = plate + foot + web1 + web2
    for dz in (12, 30):
        p -= _cyl_y(2.7, P.stringer_top_y - 8, P.stringer_top_y + 9,
                    x=foot_x, z=zf + dz)
    p.label = "m0_motor_mount"
    return p


def m0_fixed_end_mount() -> Part:
    """Fixed-end support for the T8 screw's turned Ø8 journal.

    A 688-2RS bearing seats at z=134..139 against a printed shoulder and is
    retained by a DIN472-16 ring.  The foot bolts to the left MGN stringer;
    the side web keeps the bearing housing clear of the translating carriage.
    """
    x, y = P.screw_x, P.screw_y
    # Bearing housing and dimensioned stepped bore.
    housing = _cyl_z(14.0, 132.0, 140.1, x=x, y=y)

    # M5 foot over the left stringer and a fused side web/neck to the housing.
    # Stop at x=-38 to retain 2 mm running clearance to the separately
    # printable home-endstop flag at the home/homing-overtravel pose.
    foot = _box(-55.0, -38.0, -205.0, -197.0, 122.0, 150.0)
    web = _box(-55.0, -45.0, -197.2, -188.0, 122.0, 150.0)
    neck = _box(-70.0, -45.0, -197.2, -188.0, 132.0, 140.1)
    p = housing + foot + web + neck
    # Cut after the neck union so the support web cannot refill one side of
    # the bearing pocket or retaining-ring groove.
    p -= _cyl_z(6.05, 131.0, 134.0, x=x, y=y)   # Ø12.1 throat
    p -= _cyl_z(8.05, 134.0, 139.0, x=x, y=y)   # Ø16.1 pocket
    p -= _cyl_z(8.40, 139.0, 140.1, x=x, y=y)   # DIN472-16 groove
    # The coupling begins at z=140 and clamps the 1 mm inner-race shim.  This
    # shallow Ø24.2 relief clears its nose while retaining a 1.9 mm radial
    # front lip on the Ø28 housing.
    p -= _cyl_z(12.10, 140.0, 140.2, x=x, y=y)
    # The MGN12 rail ends at z=125 and briefly shares the stringer's top
    # plane. Relieve only that 4 mm tail; both M5 foot screws remain beyond
    # the rail end at z=132/144 with at least 2 mm radial head clearance.
    p -= _box(-51.2, -38.8, -206.0, -196.0, 121.0, 126.0)
    for z in (132.0, 144.0):
        p -= _cyl_y(2.7, -206.0, -196.0, x=-45.0, z=z)
        # ISO 10642 M5 flush seat opens toward +Y at the foot top y=-197.
        p -= Pos(-45.0, -197.0, z) * (
            Rot(90, 0, 0) * Cone(5.0, 2.7, 2.3, align=ZMIN)
        )
    p.label = "m0_fixed_end_mount"
    return p


def endstop_mount() -> Part:
    """Home endstop pedestal in the inter-rail trench, footed on the front
    cross member (z=160). The switch body sits below the carriage plate;
    the plate's hanging trigger tab meets its face at ~+2 mm overtravel
    past HOME."""
    # Omron D2F body sits at z=157.0..163.5. Its solder lugs extend rearward
    # to z=166.9 while the roller faces the moving flag at lower Z.
    p = _box(-13.5, 13.5, P.base_top_y, -194, 157, 175)
    # Low foot puts the cross-member screws outside the switch cavity, so
    # short M5x12 screws engage the slot T-nuts directly.
    p += _box(-25, 25, P.base_top_y, P.base_top_y + 8, 157, 175)
    p -= _box(-8.0, 8.0, -203.2, -194.4, 155.5, 168.5)
    # Controlled Omron holes: x=-3.25/+3.25 and z=160.35, axes along Y.
    for hx in P.endstop_switch_hole_x:
        # Pocket-side bosses and rear nut-access counterbores form a real
        # M2x16 + washer + nyloc clamp around the 6.02 mm switch body.
        p += _cyl_y(3.0, -203.2, -202.02, x=hx,
                    z=P.endstop_switch_hole_z)
        p -= _cyl_y(4.0, P.base_top_y - 1, -207.0, x=hx,
                    z=P.endstop_switch_hole_z)
        p -= _cyl_y(1.1, P.base_top_y - 1, -193.0, x=hx,
                    z=P.endstop_switch_hole_z)
        # Clear the M2 socket head from the 0.4 mm pocket-front rim while the
        # washer continues to bear on the switch body at y=-196.
        p -= _cyl_y(2.2, -194.5, -193.0, x=hx,
                    z=P.endstop_switch_hole_z)
    # T-nut bolts into the cross member (z 150..170 span)
    for dx in (-19, 19):
        p -= _cyl_y(2.7, P.base_top_y - 8, P.base_top_y + 9,
                    x=dx, z=166)
    p.label = "endstop_mount"
    return p


# ======================= M3 tensioner module ==============================

def spool_bracket() -> Part:
    """Two-ear axle holder on the rear post front face; M8 axle along X."""
    zp = P.rear_post_z + 10               # post front face
    y = P.spool_y
    x0 = P.rear_post_x
    base = _box(x0 - 27, x0 + 27, y - 25, y + 25, zp, zp + 6)
    # axle raised to zp+50: drum flanges (r40) clear the base plate by 4mm
    ear_l = _box(x0 - 27, x0 - 21, y - 8, y + 8, zp + 6, zp + 60)
    ear_r = _box(x0 + 21, x0 + 27, y - 8, y + 8, zp + 6, zp + 60)
    p = base + ear_l + ear_r
    p -= _cyl_x(4.1, x0 - 28, x0 + 28, y=y, z=zp + 50)   # M8 axle
    for dy in (-18, 18):
        p -= _cyl_z(2.7, zp - 8, zp + 7, x=x0, y=y + dy)
        p -= _m5_flush_countersink_z(x0, y + dy, zp + 6)
    p.label = "spool_bracket"
    return p


def spool_drum() -> Part:
    """Printed wire spool: Ø80 flanges, Ø30 core, 40 wide, on the M8 axle."""
    y = P.spool_y
    x0 = P.rear_post_x
    zc = P.rear_post_z + 10 + 50
    core = _cyl_x(15, x0 - 20, x0 + 20, y=y, z=zc)
    fl1 = _cyl_x(40, x0 - 20, x0 - 17, y=y, z=zc)
    fl2 = _cyl_x(40, x0 + 17, x0 + 20, y=y, z=zc)
    p = core + fl1 + fl2
    p -= _cyl_x(4.3, x0 - 21, x0 + 21, y=y, z=zc)
    p.label = "spool_drum"
    return p


def felt_tensioner() -> Part:
    """Straight-pass felt pinch on the post front face: the vertical
    spool->dancer wire run passes between two Ø20 felt washers on an M4
    stud pointing +Z; spring + wingnut set the drag (1-10 N).
    Wire deflection here is <5 deg — a true straight pass."""
    zp = P.rear_post_z + 10       # post front face
    x0 = P.rear_post_x
    y = P.felt_y
    base = _box(x0 - 15, x0 + 15, y - 15, y + 15, zp, zp + 6)
    # Front face is the audited seat for the OD20 fixed backing disc.
    insert_boss = _cyl_z(7.0, zp, -161.25, x=x0, y=y)
    path = wire_geometry.static_path_spec()["landmarks"]
    wire_x, wire_z = path["felt_contact"][0], path["felt_contact"][2]
    guide_ys = (path["felt_guide_in"][1],)
    ears = [_box(wire_x - 7, wire_x + 7, gy - 4, gy + 4,
                 zp, wire_z + 7) for gy in guide_ys]
    p = base + insert_boss + ears[0]
    # M4x55 stud is captured by a jam nut loaded into this rear hex trap
    # before the base is bolted to the post.  The through bore keeps the
    # stack serviceable instead of relying on an unmodeled printed thread.
    p -= _cyl_z(2.25, zp - 1, zp + 13, x=x0, y=y)
    p -= _hex_z(7.2, zp, zp + 3.4, x=x0, y=y)
    for gy in guide_ys:
        p -= _cyl_y(P.eyelet_seat_r, gy - 5, gy + 5,
                    x=wire_x, z=wire_z)
    for dy in (-9, 9):
        p -= _cyl_z(2.7, zp - 8, zp + 7, x=x0, y=y + dy)
        p -= _m5_flush_countersink_z(x0, y + dy, zp + 6)
    p.label = "felt_tensioner"
    return p


def dancer_base() -> Part:
    """Fixed rear-post plate with pivot and two audited hard-stop pins."""
    zp = P.rear_post_z + 10
    y = P.dancer_y
    x0 = P.rear_post_x
    mount_ys = [y + offset for offset in P.dancer_base_mount_offsets]
    p = _box(x0 - 15, x0 + 15, min(mount_ys) - 5.0,
             max(mount_ys) + 5.0, zp, zp + 6)
    p -= _cyl_z(2.6, zp - 1, zp + 7, x=x0, y=y)  # M5 shoulder pivot
    # The fixed bosses remain entirely behind the arm.  Only separate Ø5
    # steel sleeves cross the arm plane, preserving the audited -3/+5.5 deg
    # contact geometry instead of locking the arm against a Ø9 boss.
    for sx, sy in P.dancer_stop_centers:
        p += _cyl_z(4.5, -170.0, -164.0, x=sx, y=sy)
        p -= _cyl_z(1.7, -171.0, -159.0, x=sx, y=sy)
        # Ø4.0 x3.5 rear pilot for McMaster 94459A769 (OD4.7 x3.4)
        # heat-set insert.  This keeps each stop independent of the crowded
        # extrusion slot and lets M3x10 terminate inside the base.
        p -= _cyl_z(2.0, -170.1, -166.6, x=sx, y=sy)
    for mount_y in mount_ys:
        p -= _cyl_z(2.7, zp - 8, zp + 7, x=x0, y=mount_y)
        p -= _m5_flush_countersink_z(x0, mount_y, zp + 6)
    p.label = "dancer_base"
    return p


def dancer_arm() -> Part:
    """Moving dancer arm, axially behind the pulley wire groove.

    Keeping the arm behind the groove is essential: the center-to-pivot
    direction lies inside the 80 degree wire-wrap sector.  Separate pivot
    and pulley bores make this a real moving component instead of the former
    fused decorative solid.
    """
    zp = P.rear_post_z + 10
    x0, y0 = P.rear_post_x, P.dancer_y
    pulley_x, pulley_y = P.dancer_pulley_x, P.dancer_pulley_y
    dx, dy = pulley_x - x0, pulley_y - y0
    arm_len = math.hypot(dx, dy)
    arm_angle = math.degrees(math.atan2(dy, dx))
    arm_local = Box(arm_len, 10, 2.5,
                    align=(Align.MIN, Align.CENTER, Align.MIN))
    arm = Pos(x0, y0, zp + 7) * (Rot(0, 0, arm_angle) * arm_local)
    pivot_boss = _cyl_z(6.0, zp + 7, zp + 9.5, x=x0, y=y0)
    pulley_boss = _cyl_z(6.0, zp + 7, zp + 9.5,
                         x=pulley_x, y=pulley_y)
    p = arm + pivot_boss + pulley_boss
    p -= _cyl_z(2.6, zp + 6, zp + 11, x=x0, y=y0)
    p -= _cyl_z(1.7, zp + 6, zp + 11, x=pulley_x, y=pulley_y)
    # One selected spring anchor.  Earlier 15/25/35 mm holes were removed:
    # they could not balance 1..10 N and weakened the lower hard-stop contact.
    ux, uy = dx / arm_len, dy / arm_len
    distance = P.dancer_spring_moving_r
    p -= _cyl_z(1.2, zp + 6, zp + 11,
                x=x0 + ux * distance, y=y0 + uy * distance)
    # Flush ISO 14581 M2 anchor seat on the arm rear face (z=-163).
    p -= Pos(x0 + ux * distance, y0 + uy * distance, zp + 7) * Cone(
        2.4, 1.2, 1.2, align=ZMIN,
    )
    p.label = "dancer_arm"
    return p


def dancer_pulley() -> Part:
    """Separate grooved Ø16 dancer pulley with a 623ZZ bearing pocket."""
    x, y = P.dancer_pulley_x, P.dancer_pulley_y
    core = _cyl_z(8.0, -158.0, -152.0, x=x, y=y)
    rear_flange = _cyl_z(9.0, -160.0, -158.0, x=x, y=y)
    front_flange = _cyl_z(9.0, -152.0, -150.0, x=x, y=y)
    p = core + rear_flange + front_flange
    p -= _cyl_z(5.05, -159.0, -153.0, x=x, y=y)  # Ø10 bearing pocket
    p -= _cyl_z(1.7, -161.0, -149.0, x=x, y=y)   # M3 shoulder axle
    p.label = "dancer_pulley"
    return p


def entry_bracket() -> Part:
    """Bracket from the (x-offset) rear post holding the fixed entry eyelet
    exactly on the flyer axis at (0, 0, wire_entry_z): base on the post
    front face, horizontal arm to x=0, then axial arm to the ring.  The rear
    boss contains a sampled 4 mm-radius side-entry elbow so the wire joins
    the axial passage tangentially instead of kinking at a drilled mouth."""
    zp = P.rear_post_z + 10
    ze = P.wire_entry_z                   # -115
    x0 = P.rear_post_x
    base = _box(x0 - 15, x0 + 15, -12, 12, zp, zp + 6)
    # Rear, +Y-offset support leaves the negative-Y wire tangent completely
    # open while still overlapping both the post base and elbow boss.
    arm_x = _box(x0, 6, 6, 16, zp, zp + 6)
    arm_z = _box(-6, 6, -6, 6, zp + 6, ze + 6)        # axial run at x=0
    # arm_x and arm_z otherwise meet only along the y=6, z=zp+6 edge.
    # The elbow boss overlaps the middle of that edge, but leaves two short
    # coincident seams at its sides; STL tessellation represents each seam
    # with four incident faces.  This local 1 x 1 mm corner key overlaps both
    # arms by 0.5 mm in Y and Z, making the connection volumetric without
    # encroaching on the open negative-Y wire tangent.
    arm_corner_key = _box(-6, 6, 5.5, 6.5, zp + 5.5, zp + 6.5)
    elbow_boss = _cyl_z(8.0, zp + 5, zp + 25)
    ring = _cyl_z(8.0, ze, ze + 6)
    # Spring overpass: a riser clears the arm sweep, then a front bridge puts
    # the fixed M2 eye at (-42,9) in the z=-154.25 spring plane.
    riser = _cyl_z(4.5, -170.0, -156.5, x=-22.0, y=11.5)
    bridge = _bar_xy((-22.0, 11.5),
                     (P.dancer_spring_fixed_x, P.dancer_spring_fixed_y),
                     6.0, -159.5, -156.5)
    anchor = _cyl_z(4.5, -159.5, -156.5,
                    x=P.dancer_spring_fixed_x,
                    y=P.dancer_spring_fixed_y)
    anchor -= _cyl_z(1.2, -160.0, -156.0,
                     x=P.dancer_spring_fixed_x,
                     y=P.dancer_spring_fixed_y)
    anchor -= Pos(P.dancer_spring_fixed_x, P.dancer_spring_fixed_y,
                  -159.5) * Cone(2.4, 1.2, 1.2, align=ZMIN)
    p = (base + arm_x + arm_z + arm_corner_key + elbow_boss + ring + riser
         + bridge + anchor)
    # Recut the complete fixed-anchor seat after union. The bridge overlaps
    # the anchor boss and otherwise refills one side of both the M2 bore and
    # the flush-head cone, producing a real screw/print interference.
    p -= _cyl_z(1.2, -160.0, -156.0,
                x=P.dancer_spring_fixed_x,
                y=P.dancer_spring_fixed_y)
    p -= Pos(P.dancer_spring_fixed_x, P.dancer_spring_fixed_y,
             -159.5) * Cone(2.4, 1.2, 1.2, align=ZMIN)
    # Clear the dancer pulley rear nyloc/excess shoulder shims through the
    # complete hard-stop sweep; this corner is outside the post slot, fixed
    # spring bridge, and wire passage.
    p -= _box(-38.5, -27.0, -22.0, -7.0, -171.0, -163.5)
    p -= _cyl_z(P.eyelet_seat_r, ze - 1, ze + 7)
    p -= _cyl_z(3.0, zp + 5, ze)          # generous axial passage
    channel = wire_geometry.static_path_spec()["entry_channel_points"]
    p -= _poly_tube(channel, 1.6, "entry_elbow_passage")
    for dy in (-8, 8):
        p -= _cyl_z(2.7, zp - 8, zp + 7, x=x0, y=dy)
        p -= _m5_flush_countersink_z(x0, dy, zp + 6)
    p.label = "entry_bracket"
    return p


# ======================= frame helpers ====================================

def rear_post_left_shoe() -> Part:
    """Printed second anchor for the tensioner post's constrained left side.

    A 25 mm HBKTST5 leg cannot fit the 15 mm corridor between the rear post
    and left base rail.  This 14 mm shoe supplies the second orthogonal M5
    connection while retaining a verified 1 mm rail/bracket gap.
    """
    floor = _box(-69.0, -55.0, -225.0, -219.0, -190.0, -170.0)
    upright = _box(-61.0, -55.0, -225.0, -200.0, -190.0, -170.0)
    p = floor + upright
    p -= _cyl_y(2.7, -226.0, -218.0, x=-63.0, z=-180.0)
    p -= _cyl_x(2.7, -62.0, -54.0, y=-208.0, z=-180.0)
    # Scallop clears the floor M5 head from the upright while leaving 3.4 mm
    # back wall and 2.5 mm to the shifted upright bore.
    p -= _cyl_y(4.6, -219.2, -213.2, x=-63.0, z=-180.0)
    p.label = "rear_post_left_shoe"
    return p


def corner_brace(label="corner_brace") -> Part:
    """Generic printed 2020 corner brace (cosmetic; steel angle in BOM)."""
    p = _box(0, 20, 0, 6, 0, 20) + _box(0, 20, 0, 20, 0, 6)
    p.label = label
    return p
