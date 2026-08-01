"""Refined tolerance sweep for the fixed-park shaft-wrap candidate."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

import wirepath


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "out" / "links" / "manifest.json"
REPORT = ROOT / "out" / "reports" / "shaft_wrap_refine.json"

AXIS_Z_MM = (94.75, 95.0)
SLEEVE_Y_MM = (11.85, 12.0, 12.15)
M1_DEG = tuple(float(v) for v in np.arange(0.0, 15.0001, 0.25))
M2_DEG = tuple(float(v) for v in np.arange(44.75, 45.2501, 0.05))
WIRE_RADIUS_MM = 0.25


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cases(manifest: dict) -> list[dict]:
    wire = manifest["wire"]
    guide = wire["tip_guide"]
    contact_base = dict(wire["shaft_contact"])
    feed_local = np.asarray(guide["feed_local_mm"], dtype=float)
    center_local = np.asarray(guide["center_local_mm"], dtype=float)
    standoff = float(manifest["m0_home_standoff"])
    reference = np.array((0.0, 0.0, standoff))
    result = []
    for axis_z in AXIS_Z_MM:
        axis = np.array((0.0, 0.0, axis_z))
        dz = axis_z - standoff
        for sleeve_y in SLEEVE_Y_MM:
            contact = dict(contact_base)
            contact["axial_y_mm"] = sleeve_y
            for m1_deg in M1_DEG:
                spindle_rotation = wirepath.rot_y(math.radians(m1_deg))
                spindle_translation = axis - spindle_rotation @ reference
                for m2_deg in M2_DEG:
                    flyer_rotation = wirepath.rot_z(math.radians(m2_deg))
                    feed = flyer_rotation @ feed_local
                    center = flyer_rotation @ center_local
                    for side in (-1, 1):
                        target = wirepath.shaft_tangent_point(
                            center, axis_z, contact, side,
                        )
                        path, guide_meta = wirepath.tip_guide_path(
                            feed, target, guide, WIRE_RADIUS_MM,
                            flyer_rotation,
                        )
                        samples = wirepath._trim_sampled_polyline(
                            path, start_mm=0.5, end_mm=0.75,
                            spacing=0.25,
                        )
                        result.append({
                            "points": samples,
                            "flyer_rotation": flyer_rotation,
                            "spindle_rotation": spindle_rotation,
                            "spindle_translation": spindle_translation,
                            "carriage_translation": np.array((0.0, 0.0, dz)),
                            "meta": {
                                "axis_z_mm": axis_z,
                                "sleeve_y_mm": sleeve_y,
                                "m1_deg": m1_deg,
                                "m2_deg": m2_deg,
                                "tangent_side": side,
                                "guide_turn_deg": float(
                                    guide_meta["arc_turn_deg"]),
                            },
                        })
    return result


def analyze() -> dict:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    all_cases = cases(manifest)
    exclusions = {
        "flyer": ("tip_toroid_guide", "wire_elbow"),
        "spindle": ("shaft_wrap_sleeve",),
        "carriage": (),
        "static": ("entry_eyelet",),
    }
    parts = {}
    broadphase = {}
    for link in ("flyer", "spindle", "carriage", "static"):
        labels, phase = wirepath._broadphase_manifest_labels(
            manifest, link, all_cases, WIRE_RADIUS_MM,
            exclude=exclusions[link],
        )
        broadphase[link] = phase
        parts[link] = wirepath._load_parts(
            manifest, link, exclude=exclusions[link], include=labels,
        )
    ranked = wirepath._case_clearances(
        parts, all_cases, WIRE_RADIUS_MM,
        progress_label="shaft wrap refined park",
    )
    worst = ranked[0]
    nearest = wirepath._rank_part_clearances(
        parts[worst[1]], wirepath._case_points_in_link(worst[2], worst[1]),
        WIRE_RADIUS_MM,
    )[0]
    # Bounds not directly sampled in the grid.  Wire radius, M0 arrival, M2
    # tracking, and sleeve placement are already swept at their limits.
    m1_chord = 2.0 * 26.0 * math.sin(math.radians(0.25) / 2.0)
    m2_chord = 2.0 * 45.0 * math.sin(math.radians(0.05) / 2.0)
    path_sampling_bound = 0.125
    runout_and_guide_budget = 0.15
    unsampled_budget = (
        m1_chord + m2_chord + path_sampling_bound
        + runout_and_guide_budget
    )
    residual = float(worst[0]) - unsampled_budget
    return {
        "schema": "shaft-wrap-refine/v1",
        "status": "PASS" if residual > 0.0 else "FAIL",
        "release_authorized": False,
        "candidate": {
            "m0_axis_z_nominal_mm": 95.0,
            "m0_axis_z_swept_mm": list(AXIS_Z_MM),
            "sleeve_y_nominal_mm": 12.0,
            "sleeve_y_swept_mm": list(SLEEVE_Y_MM),
            "m2_fixed_park_nominal_deg": 45.0,
            "m2_fixed_park_swept_deg": [min(M2_DEG), max(M2_DEG)],
            "both_tangent_sides": True,
            "complete_m1_symmetry_pitch_deg": [min(M1_DEG), max(M1_DEG)],
        },
        "sampling": {
            "case_count": len(all_cases),
            "m1_step_deg": 0.25,
            "m2_step_deg": 0.05,
            "wire_center_spacing_max_mm": 0.25,
        },
        "broadphase": broadphase,
        "worst_mesh_clearance_mm": float(worst[0]),
        "nearest_link": worst[1],
        "nearest_part": nearest[1],
        "worst_case": worst[2]["meta"],
        "unsampled_and_runout_budget_mm": {
            "m1_half_step_chord_at_r26": m1_chord,
            "m2_half_step_chord_at_r45": m2_chord,
            "wire_polyline_half_spacing": path_sampling_bound,
            "combined_shaft_runout_and_guide_placement": runout_and_guide_budget,
            "total": unsampled_budget,
        },
        "residual_clearance_after_budget_mm": residual,
        "lowest": [
            {"clearance_mm": float(row[0]), "nearest_link": row[1],
             **row[2]["meta"]}
            for row in ranked[:20]
        ],
        "required_followup": [
            "controller must reach M0 home and M2 park before M1 moves",
            "capture markers must distinguish pre-wrap parking from sleeve contact",
            "rerun the complete captured wire and rigid-body gates",
            "physically qualify enamel contact and shaft/sleeve runout",
        ],
        "source_hashes": {
            "out/links/manifest.json": _sha256(MANIFEST),
            "sim/wirepath.py": _sha256(Path(wirepath.__file__)),
            "sim/shaft_wrap_refine.py": _sha256(Path(__file__)),
        },
    }


def main() -> int:
    report = analyze()
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        f"{report['status']}: mesh={report['worst_mesh_clearance_mm']:.6f} "
        f"budget={report['unsampled_and_runout_budget_mm']['total']:.6f} "
        f"residual={report['residual_clearance_after_budget_mm']:.6f}"
    )
    print(REPORT)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
