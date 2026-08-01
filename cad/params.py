"""Master parametric configuration for the 4-axis stator flyer winder.

Single source of truth. Every CAD module, the digital twin, the collision
checker, and the settings.yml generator derive from this object. All
dimensions in mm, angles in radians, unless suffixed otherwise.

Machine coordinate frame (matches docs/requirements.md §2):
  Z  — flyer rotation axis, horizontal. Z=0 is the FLYER PLANE (forward face
       of the tip eyelet). +Z points from the flyer toward the carriage/home.
  Y  — up. Y=0 is the flyer axis height (= stator stack mid-height).
  X  — horizontal, completing the right-hand frame.

M0 model-space convention (matches upstream settings semantics):
  M0 = 0 rad  -> carriage at HOME (fully retracted, load/unload).
  M0 more negative -> deeper insertion (stator axis moves toward -Z).
  stator_axis_z(M0) = m0_home_standoff + M0 * mm_per_rad
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SpindleOption:
    """A physically distinct M1 workholding option.

    ``neck_segments`` are ``(outer_radius_mm, axial_length_mm)`` pairs,
    starting at the holder's top datum and proceeding down toward the shared
    diameter-8 shank.  Keeping this profile beside the shaft capacity makes
    it impossible for settings generation to silently use one holder while
    CAD exports use another.
    """

    id: str
    artifact_id: str
    description: str
    shaft_d_min: float
    shaft_d_max: float
    shank_d: float
    shank_len: float
    neck_segments: tuple[tuple[float, float], ...]
    geometry_kind: str
    model_path: str | None
    sourcing: str

    def supports_shaft(self, diameter: float, tol: float = 1e-9) -> bool:
        return (self.shaft_d_min - tol <= diameter <=
                self.shaft_d_max + tol)

    def manifest_record(self) -> dict:
        return {
            "id": self.id,
            "artifact_id": self.artifact_id,
            "description": self.description,
            "shaft_range_mm": [self.shaft_d_min, self.shaft_d_max],
            "shank_d_mm": self.shank_d,
            "shank_len_mm": self.shank_len,
            "neck_segments": [
                {"outer_d_mm": 2.0 * radius, "length_mm": length}
                for radius, length in self.neck_segments
            ],
            "geometry_kind": self.geometry_kind,
            "model_path": self.model_path,
            "sourcing": self.sourcing,
            "changeover_interface_id": "m1-c8-8mm-stack-v1",
        }


# ER11 capacity is 3..7 mm in the verified DIN/REGO-FIX source used by the
# checked-in high-fidelity model.  The former "ER8 slim" option had no part
# model and changed only an analytical nut radius, so it is intentionally not
# represented here.  The 8 mm endpoint uses a project-defined socket holder:
# OD16 x 16 clamp body, ID8.1 x 14 socket, two radial M4 tapped ports, and the
# same OD8 x 100 bearing/coupling shank as the ER11 holder.
SPINDLE_OPTIONS = {
    "er11": SpindleOption(
        id="er11",
        artifact_id="m1-spindle-er11-c8-100l",
        description="C8 ER11-A straight-shank collet holder",
        shaft_d_min=3.0,
        shaft_d_max=7.0,
        shank_d=8.0,
        shank_len=100.0,
        neck_segments=((9.75, 19.0), (9.0, 16.0)),
        geometry_kind="verified_parametric_model",
        model_path="cad/models/upgrades/er11_c8_hifi.step",
        sourcing="Ounuowei C8-ER11A-100L RFQ plus MariTool ER11-13PIECE set; inspect received nut OD",
    ),
    "shaft8": SpindleOption(
        id="shaft8",
        artifact_id="m1-spindle-custom-d8-c8-100l",
        description="Project-defined dedicated diameter-8 shaft socket holder",
        shaft_d_min=8.0,
        shaft_d_max=8.0,
        shank_d=8.0,
        shank_len=100.0,
        neck_segments=((8.0, 16.0),),
        geometry_kind="custom_drawing_geometry",
        model_path=None,
        sourcing="custom machine from the project geometry; no supplier SKU claimed",
    ),
}
DEFAULT_SPINDLE_ID = "er11"


def spindle_option(value: str | SpindleOption = DEFAULT_SPINDLE_ID
                   ) -> SpindleOption:
    if isinstance(value, SpindleOption):
        return value
    try:
        return SPINDLE_OPTIONS[value]
    except KeyError as exc:
        raise ValueError(
            f"unknown spindle option {value!r}; expected one of "
            f"{', '.join(sorted(SPINDLE_OPTIONS))}"
        ) from exc


@dataclass(frozen=True)
class StatorSpec:
    """One stator winding job. Drives the generated settings.yml."""

    slots: int = 24
    od: float = 46.0                # stator lamination outer diameter
    stack: float = 15.0             # lamination stack height
    shaft_d: float = 4.0            # shaft diameter held by selected M1 option
    shaft_below: float = 15.0       # exposed shaft below stator bottom face
    shaft_above: float = 8.0        # shaft stub above stator top face
    hub_od_ratio: float = 0.52      # hub OD / stator OD (tooth root circle)
    # Planning diameter for the exact release wire (Remington 32SNSP.125):
    # the supplier's 0.0088 in nominal finished OD converted to millimetres.
    # Receipt measurements regenerate the schedule; the packing audit proves
    # the fixed topology over its separate accepted measurement interval.
    wire_d: float = 0.22352
    turns: int = 50
    winding_config: str = "AaAabBbBCcCcaAaABbBbcCcC"  # 24n22p

    @property
    def tooth_len(self) -> float:
        """Radial tooth length from hub to tip."""
        return (self.od - self.od * self.hub_od_ratio) / 2.0


@dataclass(frozen=True)
class MachineParams:
    # ---- Supported stator envelope (GOAL.md design envelope) -------------
    stator_od_min: float = 28.0
    stator_od_max: float = 65.0        # launch; parametric to 90
    stator_od_max_param: float = 90.0
    stack_min: float = 5.0
    stack_max: float = 20.0
    shaft_d_min: float = 3.0
    shaft_d_max: float = 8.0

    # ---- Global clearance policy ------------------------------------------
    dyn_clearance: float = 2.0         # minimum dynamic clearance, any pose
    wire_bundle_allow: float = 3.0     # radial growth of wound tooth
    min_bend_radius: float = 3.0       # wire path, GOAL DoD #3

    # ---- Frame vertical levels (Y) ------------------------------------------
    base_bot_y: float = -245.0         # bottom of cross-member 2020 layer
    # Selected support stack: Würth 970180581 M5 F/F standoff L18 plus
    # Elesa 432001 rubber foot H17. The uncompressed support plane remains
    # y=-260 and preserves 8.35 mm below the hanging M1 motor.
    foot_h: float = 35.0
    foot_standoff_h: float = 18.0
    foot_rubber_h: float = 17.0
    foot_set_screw_projection: float = 4.8
    foot_tnut_slot_depth: float = 1.5
    base_top_y: float = -225.0         # top of cross layer = bottom of rails
    stringer_top_y: float = -205.0     # top of long rails & rail stringers
    frame_len: float = 450.0           # Z budget (base rails z -190..+260)
    frame_z0: float = -190.0
    frame_w: float = 300.0             # X budget
    base_rail_x: float = 80.0          # long base 2020 at x = +/- this
    extrusion: float = 20.0
    stringer_z0: float = -60.0         # spans both z=-50 and z=160 crosses
    stringer_len: float = 280.0

    # ---- M0 linear axis -----------------------------------------------------
    m0_lead: float = 8.0               # T8x8 (4-start): 8 mm/rev
    m0_travel: float = 87.0            # usable carriage travel (axis z 8..95)
    m0_home_standoff: float = 95.0     # stator axis Z at M0=0 (home/load)
    # The complete 22.4 mm anti-backlash set remains fully on the screw at
    # this stop with 0.6 mm end margin.  The former 5 mm value described a
    # rail-only pose that would run the newly modeled secondary nut off the
    # z=-33 screw end and was therefore not a valid machine limit.
    m0_axis_z_min: float = 8.0         # hard mechanical stop
    rail_x: float = 45.0               # two MGN12 rails at x = +/- this
    rail_len: float = 150.0            # MGN12 rail cut length, z -25..125
    rail_z0: float = -25.0
    rail_h: float = 8.0                # HIWIN datasheet
    rail_w: float = 12.0
    block_l: float = 45.4              # MGN12H datasheet
    block_w: float = 27.0
    block_h_over_rail_bot: float = 13.0
    plate_t: float = 6.35             # 0.250 in MIC6 SendCutSend plate
    # Omron D2F-01L2-D3 controlled drawing: two OD2.00 +0.12/0 mounting
    # holes on 6.50 +/-0.10 mm centers. The switch's local rear/body datum
    # is placed at z=163.50; after the 180-degree assembly rotation the hole
    # centers are z=160.35 and the free roller's near edge is z=144.60.
    endstop_switch_hole_x: tuple[float, float] = (-3.25, 3.25)
    endstop_switch_hole_z: float = 160.35
    endstop_switch_origin_z: float = 163.50
    screw_x: float = -70.0             # T8 screw offset beside rails
    screw_y: float = -180.0
    # Start 2 mm forward of the left-post front HBKT instead of tangent to it.
    # The hard-stop nut still has 20 mm of thread behind it; shortening the
    # stock cut keeps the proven journal/coupling end at z=155 unchanged.
    screw_z0: float = -33.0
    screw_len: float = 188.0
    # The final 30 mm is a turned/smooth journal.  A 688-2RS bearing carries
    # radial load while the clamp collar and motor coupling capture the screw
    # axially; this is a fixed-free screw arrangement (the carriage end floats).
    m0_journal_z: tuple = (125.0, 155.0)
    m0_fixed_bearing_z: tuple = (134.0, 139.0)
    m0_fixed_collar_z: tuple = (125.0, 134.0)
    m0_fixed_shim_z: tuple = (139.0, 140.0)
    m0_motor_z: float = 181.0          # NEMA17 mounting face Z (body +Z)
    # Raw upstream starts the first flyer move after fixed setup delays rather
    # than waiting for M0 arrival.  20 rad/s (25.46 mm/s on the T8x8 screw)
    # reaches the deepest winding target inside that unmodified timing window
    # with acceleration margin; cad/loads.py certifies the complete move.
    m0_velocity_max_rad: float = 20.0

    # ---- M1 indexing spindle (vertical, on carriage) ------------------------
    # Both explicit holder options use the same Ø8x100 shank in 2x 608ZZ
    # and the same 5->8 beam coupling.  Their workholding capacity and neck
    # profiles live in SPINDLE_OPTIONS above.
    grip_gap: float = 3.0              # stator bottom face -> nut top face
    spindle_brg_top_y: float = -95.0   # top 608ZZ upper face
    spindle_brg_gap: float = 16.0      # gap between the two 608ZZ
    spindle_outer_spacer_y: tuple = (-118.0, -102.0)
    spindle_inner_spacer_y: tuple = (-118.0, -102.0)
    spindle_lower_spacer_y: tuple = (-137.0, -125.0)
    spindle_upper_collar_y: tuple = (-95.0, -86.0)
    spindle_housing_r: float = 17.0
    m1_coupling_top_y: float = -139.5  # selected 27 mm coupling top
    m1_motor_top_y: float = -183.65    # NEMA17 face (2 mm into tower flange pocket)
    # Raw upstream likewise gives each between-phase shaft move a fixed 1.5 s
    # window.  The longest current 24-slot absolute move is 17.541 rad;
    # 20 rad/s clears it in 1.277 s at the audited 50 rad/s^2 acceleration.
    m1_velocity_max_rad: float = 20.0

    # Exact selected M0/M1 coupling: Ruland PCMR22-8-5-A.  The imported
    # Ø24 x 32 reference STEP remains a deliberately larger collision body.
    coupling_5x8_od: float = 22.2
    coupling_5x8_length: float = 27.0
    coupling_5x8_shaft_penetration: float = 12.7
    coupling_5x8_dynamic_reversing_nm: float = 0.45

    # ---- M2 flyer (rotary about Z, hollow shaft) -----------------------------
    flyer_tip_r: float = 45.0          # eyelet circle radius
    flyer_shaft_od: float = 12.0       # Ø12 aluminum tube
    flyer_shaft_id: float = 9.0        # wire bore
    flyer_shaft_rear_z: float = -100.0
    flyer_shaft_front_z: float = -30.0
    flyer_block_z: tuple = (-80.0, -30.0)   # bearing tube z-extent (plate -40..-30)
    flyer_brg_front_z: float = -48.0   # front 6001ZZ front face
    flyer_brg_rear_z: float = -67.0    # rear 6001ZZ front face
    flyer_outer_spacer_z: tuple = (-67.0, -56.0)
    flyer_inner_rear_shim_z: tuple = (-75.5, -75.0)
    flyer_inner_center_spacer_z: tuple = (-67.0, -56.0)
    flyer_inner_front_spacer_z: tuple = (-48.0, -44.0)
    hub_z: tuple = (-44.0, -32.0)      # hub clamp on tube
    spoke_z: tuple = (-28.0, -20.0)    # spoke web axial extent
    finger_len: float = 18.0           # z -20 .. -2
    eyelet_face_z: float = 0.0         # flyer plane datum
    flyer_arm_w: float = 14.0
    flyer_arm_t: float = 8.0
    pulley_z: tuple = (-93.5, -83.5)   # belt teeth plane
    m2_motor_pulley_width: float = 10.3  # NBK P40-2GT-BLP-6C-5 overall W
    m2_motor_pulley_channel: float = 7.0 # A dimension; 0.5/side for 6 mm belt
    m2_motor_pulley_capacity_nm: float = 0.688  # 0..3200 rpm supplier rating
    m2_motor_axis_y: float = -60.0     # belt center distance 60 -> 200-2GT
    # McMaster 6627T421 NEMA17 mounting face.  Its verified STEP extends
    # 78.022 mm rearward to z=-180.022 and its 22 mm shaft reaches z=-80.
    m2_motor_face_z: float = -102.0
    post_x: float = 80.0               # flyer tower posts on base rails
    post_z: tuple = (-60.0, -40.0)
    post_top_y: float = 30.0
    flyer_rpm_design: float = 300.0
    m2_gear_ratio: float = 1.0         # MUST stay 1.0 (src/constants.py)
    m2_velocity_max_rad: float = 20.0
    counterweight_r: float = 25.0
    eyelet_seat_r: float = 4.45        # Ø8.9 press fit for Ø9 guide ring
    wire_elbow_sleeve_r: float = 4.48 # Ø8.96 light fit in Ø9 shaft ID

    # ---- M3 tensioner (passive v1) --------------------------------------------
    tension_min_n: float = 1.0
    tension_max_n: float = 10.0
    rear_post_z: float = -180.0        # vertical 2020
    rear_post_x: float = -45.0         # clears M2 encoder/connector envelope
    rear_post_top_y: float = 80.0
    spool_y: float = -135.0            # spool axle height (clears M2 body)
    felt_y: float = -40.0              # felt center height (side-mounted on rear post +X face)
    dancer_y: float = 45.0             # dancer arm pivot height
    # Keep the two base T-nuts clear of the central pivot nut and both
    # embedded hard-stop inserts when using the real 15 mm HNTA5 envelope.
    dancer_base_mount_offsets: tuple = (-24.0, 24.0)
    dancer_pulley_x: float = -30.75    # pulley center; exact tangent construction
    dancer_pulley_y: float = -13.799324205
    # Dancer spring/stop geometry selected by cad/dancer_loads.py for stable
    # 1..10 N equilibria with Lee Spring LEM050AB 01.
    dancer_spring_fixed_x: float = -42.0
    dancer_spring_fixed_y: float = 9.0
    dancer_spring_moving_r: float = 45.0
    dancer_stop_offsets_deg: tuple = (-3.0, 5.5)
    dancer_stop_centers: tuple = ((-49.606283, 28.874487),
                                  (-32.999919, 33.284709))
    dancer_spring_plane_z: float = -154.25
    wire_entry_z: float = -115.0       # fixed entry eyelet, on flyer axis

    # ---- Print constraints (GOAL) ------------------------------------------
    bed: tuple = (220.0, 220.0, 250.0)
    min_wall: float = 2.4
    max_overhang_deg: float = 55.0

    # ======================= derived quantities ===========================

    @property
    def mm_per_rad(self) -> float:
        """M0 transmission ratio. T8x8: 8/(2*pi) = 1.27324 mm/rad."""
        return self.m0_lead / (2.0 * math.pi)

    @property
    def rail_top_y(self) -> float:
        return self.stringer_top_y + self.rail_h

    @property
    def block_top_y(self) -> float:
        return self.stringer_top_y + self.block_h_over_rail_bot

    @property
    def plate_top_y(self) -> float:
        return self.block_top_y + self.plate_t

    def stator_axis_z(self, m0_rad: float) -> float:
        """Stator vertical-axis Z position for a model-space M0 target."""
        return self.m0_home_standoff + m0_rad * self.mm_per_rad

    def m0_rad_for_axis_z(self, z: float) -> float:
        return (z - self.m0_home_standoff) / self.mm_per_rad

    def chuck_neck_profile(
        self,
        stator: StatorSpec,
        spindle: str | SpindleOption = DEFAULT_SPINDLE_ID,
    ):
        """Exposed rotating-column profile below the stator: list of
        (outer_radius, y_top, y_bottom), stator bottom face downward.
        This is what the flyer swept cylinder must clear."""
        option = spindle_option(spindle)
        y0 = -stator.stack / 2.0 - self.grip_gap
        profile = [
            (stator.shaft_d / 2.0, -stator.stack / 2.0, y0),
        ]
        cursor = y0
        for radius, length in option.neck_segments:
            profile.append((radius, cursor, cursor - length))
            cursor -= length
        profile.append((option.shank_d / 2.0, cursor,
                        self.spindle_brg_top_y))
        return profile

    def max_insertion(
        self,
        stator: StatorSpec,
        spindle: str | SpindleOption = DEFAULT_SPINDLE_ID,
    ) -> float:
        """Maximum tooth insertion depth d past the flyer plane (mm).

        Every rotating-column element within the flyer's swept cylinder
        (radius flyer_tip_r + dyn_clearance about Z) must stay
        >= dyn_clearance forward (+Z) of the flyer plane. Also capped by
        the hub/shaft-exit region and tooth length."""
        band = self.flyer_tip_r + self.dyn_clearance
        d_limit = math.inf
        for r, y_top, y_bot in self.chuck_neck_profile(stator, spindle):
            if y_top <= -band:
                continue
            d_limit = min(d_limit, stator.od / 2.0 - r - self.dyn_clearance)
        d_hub_cap = -self.flyer_shaft_front_z - 15.0
        return max(0.0, min(d_limit, d_hub_cap, stator.tooth_len + 4.0))

    def validate(
        self,
        stator: StatorSpec,
        spindle: str | SpindleOption = DEFAULT_SPINDLE_ID,
    ) -> list:
        """Return basic envelope/workholding violations.

        Required insertion is a property of the finite coil pack, not stator
        OD alone.  ``settings_gen.derive`` performs that check after building
        the authoritative coil/contact model.
        """
        errs = []
        option = spindle_option(spindle)
        if not (self.stator_od_min <= stator.od <= self.stator_od_max_param):
            errs.append(f"stator OD {stator.od} outside "
                        f"[{self.stator_od_min}, {self.stator_od_max_param}]")
        if not (self.stack_min <= stator.stack <= self.stack_max):
            errs.append(f"stack {stator.stack} outside envelope")
        if not (self.shaft_d_min <= stator.shaft_d <= self.shaft_d_max):
            errs.append(f"shaft {stator.shaft_d} outside envelope")
        if not option.supports_shaft(stator.shaft_d):
            errs.append(
                f"spindle {option.id} supports shaft diameter "
                f"{option.shaft_d_min:g}..{option.shaft_d_max:g} mm, not "
                f"{stator.shaft_d:g} mm"
            )
        return errs


PARAMS = MachineParams()
DEFAULT_STATOR = StatorSpec()

if __name__ == "__main__":
    p = PARAMS
    print(f"mm_per_rad = {p.mm_per_rad:.5f}   rail_top={p.rail_top_y} "
          f"block_top={p.block_top_y} plate_top={p.plate_top_y}")
    for od in (28, 36, 46, 65, 90):
        st = StatorSpec(od=od, stack=min(20.0, od * 0.3))
        d = p.max_insertion(st, DEFAULT_SPINDLE_ID)
        errs = p.validate(st, DEFAULT_SPINDLE_ID)
        print(f"OD {od:3d}: tooth {st.tooth_len:4.1f}  d_max {d:4.1f} "
              f"({DEFAULT_SPINDLE_ID})  "
              f"{'OK' if not errs else '; '.join(errs)}")
