"""Printed-part buildability audit (GOAL DoD #6).

Per part: chosen print orientation, bed fit (220x220x250), wall-thickness
statement (parametric design rule), overhang analysis in the chosen
orientation (fraction of down-facing area steeper than max_overhang_deg),
support flag, and mass estimate. Also exports print-oriented STLs to
../out/stl/.

Orientation table: rotation applied to the machine-frame solid so the
chosen face is DOWN (-Z print frame = build plate).
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import trimesh

from build123d import Pos, Rot, export_step, export_stl
from params import PARAMS as P
import printed
import carriage_endstop_flag
import assembly
import carriage_active_sector_terminal_guide as active_sector
import integrated_release_candidate as release_candidate
import m2_drive_successor_review as m2_successor
import permanent_cap_offset_spoke_retained_review as retained
import retained_flyer_peek_guide_successor as flyer_successor

OUT = Path(__file__).parent.parent / "out"

def _rear_retainer_builder(index: int):
    def build():
        solution = release_candidate.integrated_balance_solution()
        pocket = retained.POCKETS[index]
        length = float(solution["rear_slug_lengths_mm"][index])
        part = retained.retainer_cap(pocket, length)
        part.label = f"balance_retainer_{pocket.id}"
        return part
    return build


# part builder, print orientation (Rot deg to bring seat face down onto XY),
# support note
# (name, builder, print rotation, supports?, note)
PARTS = [
    ("flyer_block", printed.flyer_block, Rot(180, 0, 0), False,
     "plate face down; bearing tube up, bores vertical"),
    ("flyer_arm", flyer_successor.revised_retained_arm, Rot(180, 0, 0), True,
     "frozen torus-free successor arm; spoke face down; 100% infill; support "
     "the collar, guide-seat and front-trim pilot overhangs"),
    ("m2_motor_mount", m2_successor.successor_motor_mount, Rot(0, 0, 0), True,
     "motor plate down; support under riser web ceilings (2.7 cm2); "
     "v1.1: 45 deg chamfer gussets to print support-free"),
    ("carriage_endstop_flag", carriage_endstop_flag.endstop_flag,
     Rot(-90, 0, 0), False,
     "flat on its broad face; separate flag shares rear tower screws"),
    ("spindle_tower", active_sector.revised_spindle_tower, Rot(90, 0, 0), False,
     "revised keyed tower; flange down; bearing tube vertical; four M4 "
     "insert pilots and two yoke datum pockets require coupon inspection"),
    ("nut_bracket", printed.nut_bracket, Rot(90, 0, 0), True,
     "foot down; horizontal nut bore needs teardrop/support"),
    ("m0_motor_mount", printed.m0_motor_mount, Rot(90, 0, 0), False,
     "foot down; motor plate vertical"),
    ("m0_fixed_end_mount", printed.m0_fixed_end_mount, Rot(90, 0, 0), True,
     "stringer foot down; support the horizontal 688 bearing pocket, then "
     "ream/check Ø16.1 seat before pressing bearing"),
    ("endstop_mount", printed.endstop_mount, Rot(90, 0, 0), True,
     "foot down; switch pocket roof needs thin support"),
    ("spool_bracket", printed.spool_bracket, Rot(0, 0, 0), False,
     "post-face down; ears vertical"),
    ("spool_drum", printed.spool_drum, Rot(0, 90, 0), True,
     "axis vertical; support under top flange (or print two halves)"),
    ("felt_tensioner", printed.felt_tensioner, Rot(0, 90, 0), False,
     "side-mount base down; stud vertical"),
    ("dancer_base", printed.dancer_base, Rot(0, 0, 0), False,
     "post-face down; M5 shoulder-pivot hole vertical"),
    ("dancer_arm", printed.dancer_arm, Rot(0, 0, 0), False,
     "flat arm; pivot and pulley shoulder-bolt holes vertical"),
    ("dancer_pulley", printed.dancer_pulley, Rot(0, 0, 0), False,
     "pulley axis vertical; press 623ZZ into bearing pocket"),
    ("entry_bracket", printed.entry_bracket, Rot(0, 0, 0), True,
     "post-face down; small support under eyelet ring annulus"),
    ("rear_post_left_shoe", printed.rear_post_left_shoe,
     Rot(0, 0, 0), False,
     "broad floor down; OD9.2 scallop and both M5 bores print open"),
    ("balance_retainer_rear_left", _rear_retainer_builder(0),
     Rot(0, 0, 0), False,
     "broad retainer face down; preserve three weighed spacer posts and blind insert boss"),
    ("balance_retainer_rear_right", _rear_retainer_builder(1),
     Rot(0, 0, 0), False,
     "broad retainer face down; preserve three weighed spacer posts and blind insert boss"),
    ("balance_retainer_front_left", _rear_retainer_builder(2),
     Rot(0, 0, 0), False,
     "broad retainer face down; preserve three weighed spacer posts and blind insert boss"),
    ("balance_retainer_front_right", _rear_retainer_builder(3),
     Rot(0, 0, 0), False,
     "broad retainer face down; preserve three weighed spacer posts and blind insert boss"),
]

PETG = 1.27e-3  # g/mm^3


def place_on_bed(part):
    """Return ``part`` centered on the configured bed with its low face at Z0.

    Printed solids are authored in machine coordinates.  Rotation alone leaves
    many exported meshes floating above, or buried below, a slicer's build
    plane.  Centering in XY and translating the oriented minimum Z to zero
    makes every release mesh deterministic and directly sliceable.
    """
    bb = part.bounding_box()
    dx = P.bed[0] / 2.0 - (bb.min.X + bb.max.X) / 2.0
    dy = P.bed[1] / 2.0 - (bb.min.Y + bb.max.Y) / 2.0
    placed = Pos(dx, dy, -bb.min.Z) * part
    placed_bb = placed.bounding_box()
    if abs(placed_bb.min.Z) > 1e-7:
        raise RuntimeError(
            f"bed placement failed: min Z is {placed_bb.min.Z:.9f} mm")
    return placed


def mesh_facts(path):
    """Normalize zero-area tessellation debris, then validate the actual STL.

    OCC's sphere tessellator can emit isolated zero-area pole triangles even
    when the source BREP is a valid single solid.  Removing only those
    degenerate faces is geometry-preserving.  Positive-area non-manifold seams,
    open boundaries, multiple bodies, or inconsistent winding are never
    repaired here; they remain hard failures that must be fixed in source CAD.
    """
    mesh = trimesh.load_mesh(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"expected one Trimesh in {path}, got {type(mesh).__name__}")
    keep = mesh.nondegenerate_faces()
    removed_degenerate = int(len(keep) - np.count_nonzero(keep))
    if removed_degenerate:
        mesh.update_faces(keep)
        mesh.remove_unreferenced_vertices()
        mesh.merge_vertices()
        mesh.export(path, file_type="stl")
        # Validate the serialized artifact, not the in-memory repaired object.
        mesh = trimesh.load_mesh(path, process=True)
        if not isinstance(mesh, trimesh.Trimesh):
            raise TypeError(
                f"expected one Trimesh after normalization in {path}")
    incidence = np.bincount(mesh.edges_unique_inverse)
    components = len(mesh.split(only_watertight=False))
    bounds = np.asarray(mesh.bounds, dtype=float)
    remaining_keep = mesh.nondegenerate_faces()
    facts = {
        "removed_degenerate_faces": removed_degenerate,
        "remaining_degenerate_faces": int(
            len(remaining_keep) - np.count_nonzero(remaining_keep)),
        "watertight": bool(mesh.is_watertight),
        "winding_consistent": bool(mesh.is_winding_consistent),
        "connected_components": int(components),
        "boundary_edges": int(np.count_nonzero(incidence == 1)),
        "nonmanifold_edges": int(np.count_nonzero(incidence > 2)),
        "positive_volume": bool(mesh.volume > 0.0),
        "min_xyz_mm": [round(float(v), 6) for v in bounds[0]],
        "max_xyz_mm": [round(float(v), 6) for v in bounds[1]],
    }
    facts["ok"] = bool(
        facts["watertight"]
        and facts["remaining_degenerate_faces"] == 0
        and facts["winding_consistent"]
        and facts["connected_components"] == 1
        and facts["boundary_edges"] == 0
        and facts["nonmanifold_edges"] == 0
        and facts["positive_volume"]
        and abs(bounds[0, 2]) <= 1e-3
        and bounds[0, 0] >= -1e-3
        and bounds[0, 1] >= -1e-3
        and bounds[1, 0] <= P.bed[0] + 1e-3
        and bounds[1, 1] <= P.bed[1] + 1e-3
    )
    return facts


def overhang_fraction(part, max_deg):
    """Meshed down-facing area fraction steeper than max_deg from vertical
    walls (0 deg = vertical wall, 90 deg = flat ceiling), excluding faces
    on the build plate."""
    verts, faces = part.tessellate(tolerance=0.3, angular_tolerance=0.3)
    v = np.array([(p.X, p.Y, p.Z) for p in verts])
    f = np.array(faces)
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = 0.5 * np.linalg.norm(n, axis=1)
    nz = n[:, 2] / (np.linalg.norm(n, axis=1) + 1e-12)
    zmin = v[:, 2].min()
    tri_zmax = tri[:, :, 2].max(axis=1)
    on_plate = tri_zmax < zmin + 0.5
    down = nz < -1e-3
    # overhang angle from vertical: sin(theta) = |nz| for downward normals
    ang = np.degrees(np.arcsin(np.clip(-nz, 0, 1)))
    bad = down & ~on_plate & (ang > max_deg)
    return float(area[bad].sum())


def main(argv: list[str] | None = None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--defer-active-sector",
        action="store_true",
        help="export all unaffected prints but leave the rev-2 spindle tower missing",
    )
    args = parser.parse_args(argv)
    selected_parts = [
        row for row in PARTS
        if not (args.defer_active_sector and row[0] == "spindle_tower")
    ]
    stl_dir = OUT / "stl"
    step_dir = OUT / "step"
    stl_dir.mkdir(parents=True, exist_ok=True)
    step_dir.mkdir(parents=True, exist_ok=True)
    # These directories are generator-owned release artifacts.  Remove only
    # old files of the exact generated types so stale plastic carriage plates,
    # obsolete felt studs, or retired cosmetic braces cannot be printed by
    # mistake.
    for directory, suffix in ((stl_dir, "*.stl"), (step_dir, "*.step")):
        if directory.resolve().parent != OUT.resolve():
            raise RuntimeError(f"refusing to clean unexpected path {directory}")
        for old in directory.glob(suffix):
            old.unlink()
    # Retired viewer sidecars are not matched by ``*.step``.  Delete only the
    # two exact legacy names so an obsolete wire elbow or flyer pulley cannot
    # survive beside the current torus-free release print set.
    for legacy in (
        step_dir / ".wire_elbow.step.glb",
        step_dir / ".flyer_pulley.step.glb",
    ):
        if legacy.is_file():
            legacy.unlink()
    rows = []
    disconnected = []
    mesh_failures = []
    for name, fn, orient, supports, note in selected_parts:
        p = place_on_bed(orient * fn())
        ns = len(p.solids())
        if ns != 1:
            disconnected.append((name, ns))
        bb = p.bounding_box()
        size = [bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z]
        fits = (sorted(size[:2])[0] <= P.bed[0] and
                sorted(size[:2])[1] <= P.bed[1] and size[2] <= P.bed[2])
        of = overhang_fraction(p, P.max_overhang_deg)  # mm^2 absolute
        mass = p.volume * PETG
        stl_path = stl_dir / f"{name}.stl"
        step_path = step_dir / f"{name}.step"
        export_stl(p, str(stl_path), tolerance=0.05,
                   angular_tolerance=0.15)
        export_step(p, str(step_path))
        mesh = mesh_facts(stl_path)
        if not mesh["ok"]:
            mesh_failures.append((name, mesh))
        rows.append({
            "part": name,
            "size_mm": [round(s, 1) for s in size],
            "bed_fit": bool(fits),
            "mass_g_solid": round(mass, 0),
            "overhang_area_mm2": round(of, 0),
            "supports": "yes" if supports else "no",
            "note": note,
            "mesh": mesh,
        })
        print(f"{name:16s} {str([round(s) for s in size]):>15s} "
              f"fit={'Y' if fits else 'N'} overhang>{P.max_overhang_deg:.0f}"
              f"deg={of:7.0f}mm2 sup={'Y' if supports else 'n'} "
              f"mesh={'Y' if mesh['ok'] else 'FAIL'}  "
              f"{note[:48]}")

    # Explicit source-level wall dimensions for the deliberately thin features.
    # These are not mesh guesses: each value is the residual solid thickness
    # from the primitives in printed.py.  In particular, the M1 motor pocket is
    # 2 mm deep in a 6 mm flange, so its roof is 4 mm (the old report
    # incorrectly called the *pocket depth* a 2 mm roof).
    wall_checks = [
        {"feature": "successor flyer-arm spoke web rails", "thickness_mm": 3.5},
        {"feature": "successor flyer-arm shaft-passage ligament",
         "thickness_mm": P.min_wall},
        {"feature": "spindle_tower M1 motor-pocket roof",
         "thickness_mm": 4.0},
        {"feature": "spindle_tower bearing housing radial wall",
         "thickness_mm": P.spindle_housing_r - 11.0},
        {"feature": "carriage_plate nominal plate", "thickness_mm": P.plate_t},
        {"feature": "M2 motor-mount plate", "thickness_mm": 6.0},
        {"feature": "M0 688 housing at retaining-ring groove",
         "thickness_mm": 14.0 - 8.4},
        {"feature": "rear-post shoe scallop back wall",
         "thickness_mm": 3.4},
        {"feature": "rear-post shoe scallop-to-upright-bore ligament",
         "thickness_mm": 2.5},
    ]
    for check in wall_checks:
        check["minimum_mm"] = P.min_wall
        check["ok"] = check["thickness_mm"] >= P.min_wall
    thinnest = min(wall_checks, key=lambda item: item["thickness_mm"])

    # GOAL DoD #6 asks that every required machining operation be called out.
    # Printed parts are exported ready to slice; these are the non-printed cut
    # operations that remain part of normal assembly preparation.
    machining = [
        {"part": "2020 extrusion frame members",
         "operation": "square-cut to BOM lengths and deburr",
         "supplier_option": "order factory-cut lengths to avoid shop machining"},
        {"part": "M2-001 Rev D L79.00 stock-D10 flyer shaft",
         "operation": "machine the full rear 18.50 mm to OD9.991-10.000 h6 / "
                      "ID6.000-6.030, the remaining main span to "
                      "OD11.980-12.000 / ID9.000-9.050, and a polished 3.00 mm "
                      "ID6-to-ID9 internal transition; deburr and polish both "
                      "wire-path mouths to R0.5 minimum; add only the two 0.30 "
                      "mm-deep orthogonal -Y/+X flyer-arm flats, 5.00 mm axial "
                      "at 64.75 mm from rear datum A; pulley-side flats are prohibited",
         "supplier_option": "RFQ the released Rev D L79.00 STEP and inspection "
                            "drawing; do not substitute 1/4 inch McMaster "
                            "89965K431 tube or the retired predecessor shaft"},
        {"part": "T8 lead screw",
         "operation": "none on fully threaded stock: order the release "
                      "188 mm custom screw manufactured with a native 30 mm "
                      "unthreaded Ø8 h9 end journal (z=125..155). Do NOT try "
                      "to turn an Ø8 thread into an Ø8 journal.",
         "supplier_option": "RFQ the dimensioned custom lead-screw drawing; "
                            "thread only the lower 158 mm of an Ø8 blank"},
        {"part": "shaft-wrap sleeve",
         "operation": "bond the configured seamless alumina sleeve to the "
                      "clean upper work shaft; verify full cure, R0.75 edges, "
                      "and Ra <=0.2 um before threading wire",
         "supplier_option": "KEIR Series-45-class custom sleeve from the "
                            "per-job changeover drawing"},
        {"part": "one-piece PEEK flyer guide and exit bell",
         "operation": "machine the frozen one-solid STEP; keep every wire "
                      "channel open and polish the full ID0.60 contact bore, "
                      "R3.25 root elbow and bell surface",
         "supplier_option": "RFQ only after the supplier declares an exact "
                            "natural-unfilled PEEK grade and lot certificate; "
                            "abrasion, dielectric, varnish and hot-load coupons remain open"},
        {"part": "short-leadin PEEK cap pair and active-sector PEEK guide pair",
         "operation": "machine four separate frozen STEP parts; preserve open "
                      "R3.50 lead-ins, guide bowls, datum keys and insert pilots; "
                      "polish all wire-contact surfaces",
         "supplier_option": "same fail-closed natural-unfilled PEEK RFQ packet; "
                            "no filled or adhesive-backed substitute"},
        {"part": "active-sector aluminum yoke",
         "operation": "CNC mill the one-solid keyed yoke and inspect all M3/M4 "
                      "hole axes and guide/tower datums",
         "supplier_option": "6061-T6 or drawing-approved equivalent; quote and "
                            "inspection plan remain TBD"},
        {"part": "stock NBK P30-3GT-BLP-6C-10 flyer pulley",
         "operation": "no pulley machining: receive-inspect the official D10 "
                      "through bore and supplied SCM435 M2 clamp bolt; install "
                      "hub-rear on the complete OD10 h6 seat and torque to 0.5 N m",
         "supplier_option": "order the standard A2017 stock part; delivered-bore "
                            "inspection, belt-pretension check, reversing slip/crush "
                            "cycle and hot endurance remain mandatory"},
        {"part": "six ASTM-B777 balance trims",
         "operation": "machine each serialized annular STEP to its independently "
                      "solved dimension, deburr, weigh and record actual mass",
         "supplier_option": "grade, certified density, lot and price TBD; final "
                            "installed G2.5 balance is mandatory"},
        {"part": "MGN12 rails",
         "operation": "none when ordered at the BOM length",
         "supplier_option": "order finished rails; do not abrasive-cut assembled rails"},
    ]

    audit = {"single_solid_check": "pass" if not disconnected else
             [list(d) for d in disconnected],
             "mesh_check": "pass" if not mesh_failures else
             [{"part": name, **facts} for name, facts in mesh_failures],
             "bed": P.bed, "min_wall_rule_mm": P.min_wall,
             "max_overhang_deg": P.max_overhang_deg, "parts": rows,
             "wall_checks": wall_checks,
             "wall_note": ("source-level thin-feature audit complete; minimum "
                           f"{thinnest['thickness_mm']:.1f} mm at "
                           f"{thinnest['feature']} versus {P.min_wall:.1f} mm rule"),
             "machining": machining,
             "pending_parts": (
                 ["spindle_tower"] if args.defer_active_sector else []
             )}
    (OUT / "reports").mkdir(exist_ok=True)
    (OUT / "reports" / "buildability.json").write_text(
        json.dumps(audit, indent=2))
    bad = [r for r in rows if not r["bed_fit"]]
    print(f"\n{len(rows)} parts, bed-fit fails: {len(bad)}; "
          f"print-oriented STLs in out/stl/ and editable STEP in out/step/")
    if disconnected:
        for n, k in disconnected:
            print(f"  DISCONNECTED: {n} has {k} solids (must be 1)")
        raise SystemExit(1)
    if mesh_failures:
        for name, facts in mesh_failures:
            print(f"  INVALID STL: {name}: {facts}")
        raise SystemExit(1)
    print("connectivity: every part is a single solid")
    print("mesh topology/placement: every STL is watertight, manifold, and on bed")


if __name__ == "__main__":
    main()
