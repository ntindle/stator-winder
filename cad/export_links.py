"""Export per-link collision/animation meshes + kinematics manifest.

Writes out/links/<link>.stl (reference pose, machine coordinates) and
out/links/manifest.json describing each link's pose transform so the twin
(sim/) can animate meshes numerically without touching CAD.

Usage: python export_links.py [--od 46 --stack 15 --slots 24
                              --spindle er11|shaft8]
"""

import json
import hashlib
import sys
import argparse
from pathlib import Path

from build123d import Compound, export_stl

from params import (
    DEFAULT_STATOR,
    DEFAULT_SPINDLE_ID,
    PARAMS as P,
    SPINDLE_OPTIONS,
    StatorSpec,
    spindle_option,
)
import assembly
import coil_growth
import settings_gen
import wire_geometry

OUT = Path(__file__).parent.parent / "out" / "links"

# These parts need their own render materials in the browser player.  They are
# removed from the aggregate link mesh before their group mesh is exported, so
# the player never draws coincident copies (the old all-grey aggregate made the
# felt pads indistinguishable and would z-fight with a coloured overlay).
VISUAL_EXPORT_GROUPS = {
    "felt_pads": {
        "link": "static",
        "file": "static_felt_pads.stl",
        "material": "felt_dark_brown",
        "labels": ("felt_pad_fixed", "felt_pad_moving"),
        "description": "two dimensioned wool-felt drag pads",
    },
}

# The six production balance stacks are serialized occurrences owned by
# integrated_release_candidate.py and are exported by
# integrated_export_player_adapter.py as 6 tungsten, 4 printed-retainer and
# 14 hardware occurrences.  hardware_placements.py intentionally does not
# duplicate those parts in this legacy link exporter, so requiring the former
# generic five-part ``counterweight_*`` group here made every fresh export
# fail after the redesign.


def _split_visual_parts(link_name, parts):
    """Return an aggregate-link part list and exact special render groups."""
    group_specs = {
        name: spec for name, spec in VISUAL_EXPORT_GROUPS.items()
        if spec["link"] == link_name
    }
    labels_to_group = {}
    for group_name, group in group_specs.items():
        for label in group["labels"]:
            if label in labels_to_group:
                raise RuntimeError(
                    f"visual label {label!r} belongs to multiple groups"
                )
            labels_to_group[label] = group_name

    base = []
    grouped = {name: [] for name in group_specs}
    seen = set()
    for part in parts:
        label = getattr(part, "label", "") or ""
        group_name = labels_to_group.get(label)
        if group_name is None:
            base.append(part)
            continue
        grouped[group_name].append(part)
        seen.add(label)

    missing = set(labels_to_group) - seen
    if missing:
        raise RuntimeError(
            f"{link_name} visual export is missing grouped parts: "
            + ", ".join(sorted(missing))
        )
    return base, grouped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--od", type=float, default=46.0)
    ap.add_argument("--stack", type=float, default=15.0)
    ap.add_argument("--slots", type=int, default=24)
    ap.add_argument("--shaft", type=float, default=4.0)
    ap.add_argument("--wire", type=float, default=DEFAULT_STATOR.wire_d)
    ap.add_argument("--turns", type=int, default=DEFAULT_STATOR.turns)
    ap.add_argument("--spindle", choices=sorted(SPINDLE_OPTIONS),
                    default=DEFAULT_SPINDLE_ID,
                    help="physical M1 workholding option")
    args = ap.parse_args()

    spec = StatorSpec(od=args.od, stack=args.stack, slots=args.slots,
                      shaft_d=args.shaft, wire_d=args.wire,
                      turns=args.turns)
    option = spindle_option(args.spindle)
    # Use the same fail-closed finite-coil reach gate as settings generation;
    # an exported geometry set must never disagree with its settings file.
    settings_gen.derive(spec, option.id)
    coil = coil_growth.require_feasible(spec)
    OUT.mkdir(parents=True, exist_ok=True)
    parts_root = OUT / "parts"
    parts_root.mkdir(exist_ok=True)
    if parts_root.resolve().parent != OUT.resolve():
        raise RuntimeError(f"refusing to clean unexpected path {parts_root}")
    for old in parts_root.rglob("*.stl"):
        old.unlink()

    visual_links = assembly.build_links(spec, spindle=option)
    collision_links = assembly.build_links(
        spec, final_wound_collision=True, spindle=option)
    files = {}
    part_files = {}
    part_bounds = {}
    visual_group_records = {}
    for name, parts in collision_links.items():
        visual_parts = visual_links[name]
        base_visual_parts, grouped_visual_parts = _split_visual_parts(
            name, visual_parts
        )
        if not base_visual_parts:
            raise RuntimeError(f"{name} aggregate visual mesh is empty")
        comp = Compound(children=list(base_visual_parts))
        f = OUT / f"{name}.stl"
        export_stl(comp, str(f), tolerance=0.2, angular_tolerance=0.25)
        files[name] = f.name
        for group_name, group_parts in grouped_visual_parts.items():
            group_spec = VISUAL_EXPORT_GROUPS[group_name]
            group_file = OUT / group_spec["file"]
            export_stl(
                Compound(children=list(group_parts)), str(group_file),
                tolerance=0.1, angular_tolerance=0.2,
            )
            visual_group_records[group_name] = {
                "file": group_file.name,
                "link": name,
                "material": group_spec["material"],
                "labels": list(group_spec["labels"]),
                "description": group_spec["description"],
                "excluded_from_base_link_mesh": True,
            }
        # per-part meshes for pair-specific fit exemptions in collide.py
        pdir = OUT / "parts" / name
        pdir.mkdir(parents=True, exist_ok=True)
        plist = []
        pbounds = {}
        for i, p in enumerate(parts):
            label = getattr(p, "label", f"part{i}") or f"part{i}"
            pf = pdir / f"{label}.stl"
            export_stl(p, str(pf), tolerance=0.2, angular_tolerance=0.25)
            plist.append(label)
            bb = p.bounding_box()
            pbounds[label] = [
                [bb.min.X, bb.min.Y, bb.min.Z],
                [bb.max.X, bb.max.Y, bb.max.Z],
            ]
        part_files[name] = plist
        part_bounds[name] = pbounds
        grouped_count = sum(map(len, grouped_visual_parts.values()))
        print(f"{name}: {len(parts)} collision parts; "
              f"{len(base_visual_parts)} base + {grouped_count} grouped "
              f"visual parts -> {f.name}")

    if set(visual_group_records) != set(VISUAL_EXPORT_GROUPS):
        raise RuntimeError("not every declared special visual group was exported")

    # wire visualization tubes (excluded from collision/audit manifests)
    import wire_vis
    # ``wire_vis`` defaults to the project job, while this exporter also
    # supports an explicit --wire variant.  Keep its rendered tube true-scale
    # for the actual export spec in either case.
    wire_vis.R_VIS = spec.wire_d / 2.0
    export_stl(wire_vis.wire_static(), str(OUT / "wire_static.stl"),
               tolerance=0.1, angular_tolerance=0.2)
    export_stl(wire_vis.wire_flyer(), str(OUT / "wire_flyer.stl"),
               tolerance=0.1, angular_tolerance=0.2)

    # separate stator-only mesh (finer, for animation visuals) — same
    # sequential rotation as assembly.spindle_link (NOT combined Rot!)
    import stator_model
    from build123d import Pos, Rot
    st = Pos(0, 0, P.m0_home_standoff) * (Rot(0, 90, 0) * (
        Rot(-90, 0, 0) * stator_model.stator(spec)))
    export_stl(st, str(OUT / "stator_only.stl"), tolerance=0.1,
               angular_tolerance=0.2)

    static_wire = wire_geometry.static_path_spec()
    flyer_wire = wire_geometry.flyer_path_spec()
    tooth_contact = wire_geometry.tooth_contact_spec(spec, coil)
    shaft_contact = wire_geometry.shaft_contact_spec(spec)
    tip_guide = wire_geometry.tip_guide_spec()
    spindle_record = option.manifest_record()
    spindle_source = (
        Path(__file__).parent.parent / option.model_path
        if option.model_path is not None
        else Path(__file__).parent / "assembly.py"
    )
    spindle_record["geometry_source"] = str(
        spindle_source.relative_to(Path(__file__).parent.parent)
    ).replace("\\", "/")
    spindle_record["geometry_source_sha256"] = hashlib.sha256(
        spindle_source.read_bytes()).hexdigest()
    manifest = {
        "units": "mm",
        "frame": "machine (Z=flyer axis, Z0=flyer plane, +Z toward home; Y up)",
        "reference_pose": {"m0": 0.0, "m1": 0.0, "m2": 0.0},
        "mm_per_rad_m0": P.mm_per_rad,
        "m0_home_standoff": P.m0_home_standoff,
        "flyer_tip_r": P.flyer_tip_r,
        "m2_belt_center_distance_mm": abs(P.m2_motor_axis_y),
        "dyn_clearance": P.dyn_clearance,
        "stator": {"od": spec.od, "stack": spec.stack, "slots": spec.slots,
                   "shaft_d": spec.shaft_d, "wire_d": spec.wire_d,
                   "shaft_below": spec.shaft_below,
                   "shaft_above": spec.shaft_above,
                   "hub_od_ratio": spec.hub_od_ratio,
                   "turns": spec.turns, "tooth_len": spec.tooth_len,
                   "slot_fill": coil["packing"]["gross_slot_fill"],
                   "slot_fill_status": coil["status"],
                   "coil_collision_growth": coil["bundle"]["collision_growth_mm"]},
        "spindle": spindle_record,
        "links": {
            "static": {"file": files["static"], "transform": "identity"},
            "carriage": {"file": files["carriage"],
                         "transform": "translate_z",
                         "rule": "dz = m0 * mm_per_rad_m0"},
            "spindle": {"file": files["spindle"],
                        "transform": "translate_z_then_yaw",
                        "rule": "dz = m0*mm_per_rad_m0; rotate about "
                                "vertical axis at (x=0, z=m0_home_standoff"
                                "+dz) by m1"},
            "flyer": {"file": files["flyer"], "transform": "rot_z",
                      "rule": "rotate about machine Z axis by m2"},
        },
        "visual_groups": visual_group_records,
        "wire": {
            "diameter_max": wire_geometry.WIRE_DIAMETER_MAX,
            "radius_max": wire_geometry.WIRE_RADIUS_MAX,
            "diameter_job": spec.wire_d,
            "radius_job": spec.wire_d / 2.0,
            "render_radius": wire_vis.R_VIS,
            "render_diameter_scale": wire_vis.R_VIS / (spec.wire_d / 2.0),
            "slot_edge_clearance":
                coil_growth.DEFAULT_POLICY.opening_edge_clearance_mm,
            "static": static_wire,
            "flyer": flyer_wire,
            "tooth_contact": tooth_contact,
            "shaft_contact": shaft_contact,
            "tip_guide": tip_guide,
            "note": "flyer points are local to M2=0 and rotate about Z; "
                    "static points remain in the machine frame",
        },
        "max_insertion_mm": P.max_insertion(spec, option),
        "parts": part_files,
        "part_bounds": part_bounds,
    }
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"manifest.json written; spindle={option.id}; max insertion for "
          f"this stator: {P.max_insertion(spec, option):.2f} mm")


if __name__ == "__main__":
    main()
