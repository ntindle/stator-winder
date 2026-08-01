"""Fail-closed search for a collision-free phase-lead shaft-wrap pose.

The captured second wrap currently sends the torus-to-sleeve free span
through the finished winding.  This study varies only two physically explicit
design inputs: M0's stator/shaft axis position and the axial station of the
permanent polished sleeve.  It reuses the authoritative wirepath collision
kernel, includes both tangent sides, and sweeps one complete 24-slot M1
symmetry pitch plus every M2 orientation.  It never changes the controller or
release evidence by itself.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

import wirepath


ROOT = Path(__file__).resolve().parents[1]
LINKS = ROOT / "out" / "links"
REPORT = ROOT / "out" / "reports" / "shaft_wrap_route_study.json"


AXIS_Z_MM = tuple(float(value) for value in range(35, 96, 10))
SLEEVE_Y_MM = (12.0, 15.0, 18.0, 21.0, 24.0)
M2_STEP_DEG = 5.0
M1_PHASE_DEG = (0.0, 3.75, 7.5, 11.25, 15.0)
WIRE_RADIUS_MM = 0.25


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _cases(manifest: dict, axis_z: float, sleeve_y: float) -> list[dict]:
    wire = manifest["wire"]
    guide = wire["tip_guide"]
    contact = dict(wire["shaft_contact"])
    contact["axial_y_mm"] = sleeve_y
    feed_local = np.asarray(guide["feed_local_mm"], dtype=float)
    center_local = np.asarray(guide["center_local_mm"], dtype=float)
    standoff = float(manifest["m0_home_standoff"])
    dz = axis_z - standoff
    reference = np.array((0.0, 0.0, standoff))
    axis = np.array((0.0, 0.0, axis_z))
    result: list[dict] = []
    for m1_deg in M1_PHASE_DEG:
        spindle_rotation = wirepath.rot_y(math.radians(m1_deg))
        spindle_translation = axis - spindle_rotation @ reference
        for m2_deg in np.arange(0.0, 360.0, M2_STEP_DEG):
            flyer_rotation = wirepath.rot_z(math.radians(float(m2_deg)))
            feed = flyer_rotation @ feed_local
            center = flyer_rotation @ center_local
            for side in (-1, 1):
                target = wirepath.shaft_tangent_point(
                    center, axis_z, contact, side,
                )
                path, guide_meta = wirepath.tip_guide_path(
                    feed, target, guide, WIRE_RADIUS_MM, flyer_rotation,
                )
                samples = wirepath._trim_sampled_polyline(
                    path, start_mm=0.5, end_mm=0.75, spacing=0.25,
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
                        "m2_deg": float(m2_deg),
                        "tangent_side": side,
                        "guide_turn_deg": float(guide_meta["arc_turn_deg"]),
                    },
                })
    return result


def analyze() -> dict:
    manifest_path = LINKS / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    grids = {
        (axis_z, sleeve_y): _cases(manifest, axis_z, sleeve_y)
        for axis_z in AXIS_Z_MM for sleeve_y in SLEEVE_Y_MM
    }
    all_cases = [case for cases in grids.values() for case in cases]
    exclusions = {
        "flyer": ("tip_toroid_guide", "wire_elbow"),
        "spindle": ("shaft_wrap_sleeve",),
        "carriage": (),
        "static": ("entry_eyelet",),
    }
    parts_by_link = {}
    broadphase = {}
    for link in ("flyer", "spindle", "carriage", "static"):
        labels, phase = wirepath._broadphase_manifest_labels(
            manifest, link, all_cases, WIRE_RADIUS_MM,
            exclude=exclusions[link],
        )
        broadphase[link] = phase
        parts_by_link[link] = wirepath._load_parts(
            manifest, link, exclude=exclusions[link], include=labels,
        )

    results = []
    for index, ((axis_z, sleeve_y), cases) in enumerate(grids.items(), 1):
        ranked = wirepath._case_clearances(
            parts_by_link, cases, WIRE_RADIUS_MM,
            progress_label=f"shaft study {index}/{len(grids)}",
        )
        worst = ranked[0]
        nearest = wirepath._rank_part_clearances(
            parts_by_link[worst[1]],
            wirepath._case_points_in_link(worst[2], worst[1]),
            WIRE_RADIUS_MM,
        )[0]
        results.append({
            "axis_z_mm": axis_z,
            "sleeve_y_mm": sleeve_y,
            "worst_clearance_mm": float(worst[0]),
            "nearest_link": worst[1],
            "nearest_part": nearest[1],
            "worst_case": worst[2]["meta"],
            "passes_mesh_clearance": bool(worst[0] > 0.0),
        })
        print(
            f"axis {axis_z:5.1f} sleeve-y {sleeve_y:4.1f}: "
            f"{worst[0]:8.4f} mm to {worst[1]}/{nearest[1]}",
            flush=True,
        )
    results.sort(key=lambda row: row["worst_clearance_mm"], reverse=True)
    best = results[0]
    return {
        "schema": "shaft-wrap-route-study/v1",
        "status": "GEOMETRIC_CANDIDATE" if best["passes_mesh_clearance"] else "NO_CANDIDATE",
        "release_authorized": False,
        "method": "authoritative wirepath mesh distance, both tangent sides, one 24-slot M1 pitch, full M2 revolution",
        "grid": {
            "axis_z_mm": list(AXIS_Z_MM),
            "sleeve_y_mm": list(SLEEVE_Y_MM),
            "m1_phase_deg": list(M1_PHASE_DEG),
            "m2_step_deg": M2_STEP_DEG,
            "wire_radius_mm": WIRE_RADIUS_MM,
            "case_count_per_candidate": len(next(iter(grids.values()))),
        },
        "broadphase": broadphase,
        "best": best,
        "results": results,
        "required_followup": [
            "refine the best neighborhood to <=0.5 degree M1/M2 and <=0.25 mm M0",
            "prove sleeve remains within exposed shaft and outside stator/coil axial envelope",
            "update controller capture so M0 reaches the selected axis before M1 wrap motion",
            "rerun full continuous-wire and rigid-body collision gates",
        ],
        "source_hashes": {
            "out/links/manifest.json": _sha256(manifest_path),
            "sim/wirepath.py": _sha256(Path(wirepath.__file__)),
            "sim/shaft_wrap_route_study.py": _sha256(Path(__file__)),
        },
    }


def main() -> int:
    report = analyze()
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"{report['status']}: best={report['best']}")
    print(REPORT)
    return 0 if report["status"] == "GEOMETRIC_CANDIDATE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
