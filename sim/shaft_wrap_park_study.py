"""Find a fixed M2 park angle for one shaft-wrap axis/sleeve candidate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import shaft_wrap_route_study as base
import wirepath


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "out" / "links" / "manifest.json"
REPORT = ROOT / "out" / "reports" / "shaft_wrap_park_study.json"


def analyze(axis_z: float, sleeve_y: float) -> dict:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    cases = base._cases(manifest, axis_z, sleeve_y)
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
            manifest, link, cases, base.WIRE_RADIUS_MM,
            exclude=exclusions[link],
        )
        broadphase[link] = phase
        parts[link] = wirepath._load_parts(
            manifest, link, exclude=exclusions[link], include=labels,
        )
    ranked = wirepath._case_clearances(
        parts, cases, base.WIRE_RADIUS_MM,
        progress_label="shaft fixed-park study",
    )
    by_angle: dict[float, dict] = {}
    for clearance, link, case, _values in ranked:
        angle = float(case["meta"]["m2_deg"])
        current = by_angle.get(angle)
        if current is None or clearance < current["worst_clearance_mm"]:
            nearest = wirepath._rank_part_clearances(
                parts[link], wirepath._case_points_in_link(case, link),
                base.WIRE_RADIUS_MM,
            )[0]
            by_angle[angle] = {
                "m2_deg": angle,
                "worst_clearance_mm": float(clearance),
                "nearest_link": link,
                "nearest_part": nearest[1],
                "worst_case": case["meta"],
            }
    angles = sorted(by_angle.values(),
                    key=lambda row: row["worst_clearance_mm"], reverse=True)
    best = angles[0]
    return {
        "schema": "shaft-wrap-park-study/v1",
        "status": "GEOMETRIC_CANDIDATE" if best["worst_clearance_mm"] > 0 else "NO_CANDIDATE",
        "release_authorized": False,
        "axis_z_mm": axis_z,
        "sleeve_y_mm": sleeve_y,
        "m1_phase_deg": list(base.M1_PHASE_DEG),
        "m2_step_deg": base.M2_STEP_DEG,
        "both_tangent_sides": True,
        "broadphase": broadphase,
        "best_fixed_park": best,
        "angles": angles,
        "required_followup": [
            "refine around the best M2 angle at <=0.25 degree",
            "prove the selected controller sequence completes M2 park before M1 rotation",
            "rerun captured continuous-wire and rigid-body collision gates",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--axis-z", type=float, default=95.0)
    parser.add_argument("--sleeve-y", type=float, default=24.0)
    parser.add_argument("--out", type=Path, default=REPORT)
    args = parser.parse_args()
    report = analyze(args.axis_z, args.sleeve_y)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(report["status"], report["best_fixed_park"])
    print(args.out)
    return 0 if report["status"] == "GEOMETRIC_CANDIDATE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
