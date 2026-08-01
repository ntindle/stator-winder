"""Fail-closed same-link source-BREP overlap audit.

The cycle sweep checks only *between* kinematic links.  This audit first uses
the exported per-part meshes as a fast broad phase, then measures every mesh
contact with the original build123d solids.  Mesh tessellation contacts are
therefore never mistaken for physical interpenetration.

Positive source volume is accepted only for a named mechanical interface:
threaded/T-slot capture, heat-set interference, press/rolling fits, belt tooth
engagement, or the extension-spring loops.  Every accepted record carries its
classification in ``out/reports/static_audit.json``.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import fcl
import numpy as np
import trimesh
from build123d import Compound


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "out"
LINKS = OUT / "links"
sys.path.insert(0, str(ROOT / "cad"))

import assembly  # noqa: E402
import frame_hardware_audit  # noqa: E402
import hardware_placements  # noqa: E402
from params import PARAMS as P, StatorSpec, spindle_option  # noqa: E402


SOURCE_VOLUME_TOL_MM3 = 1.0e-5


# Named press fits, clamped fits, and catalog bodies whose simplified source
# solids intentionally share positive volume.  Thread/T-slot and heat-set
# relationships are derived below rather than enumerated here.
DESIGNED_FITS = {
    # static
    ("m0_fixed_end_mount", "m0_688_2rs"): "bearing press seat",
    ("m0_fixed_end_mount", "m0_din472_16"): "retaining-ring groove",
    ("m0_688_2rs", "m0_din472_16"): "retaining-ring envelope",
    ("m0_688_2rs", "m0_inner_shim"): "inner-race shim seat",
    ("m0_coupling", "m0_inner_shim"): "coupling clamp seat",
    ("m0_coupling", "t8_screw"): "coupling clamp engagement",
    ("m0_coupling", "m0_motor"): "coupling motor-shaft engagement",
    ("flyer_block", "flyer_6001_front"): "bearing press seat",
    ("flyer_block", "flyer_6001_rear"): "bearing press seat",
    ("flyer_block", "m2_din472_28"): "retaining-ring groove",
    ("flyer_6001_rear", "m2_din472_28"): "retaining-ring envelope",
    ("m2_motor_pulley", "m2_motor"): "pulley shaft clamp",
    ("m2_motor_pulley", "gt2_belt"): "GT2 pitch-line engagement",
    ("spool_drum", "spool_bracket"): "axle/bore assembly envelope",
    ("felt_tensioner", "felt_guide_in"): "ceramic guide press seat",
    ("entry_bracket", "entry_eyelet"): "ceramic guide press seat",
    # carriage
    ("spindle_tower", "spindle_608_top"): "bearing press seat",
    ("spindle_tower", "spindle_608_bot"): "bearing press seat",
    ("spindle_tower", "m1_din472_22_lower"): "retaining-ring groove",
    ("spindle_tower", "m1_din472_22_upper"): "retaining-ring groove",
    ("spindle_608_top", "m1_din472_22_upper"): "retaining-ring envelope",
    ("spindle_608_bot", "m1_din472_22_lower"): "retaining-ring envelope",
    ("t8_nut", "nut_bracket"): "T8 nut flange seat",
    ("t8_nut_main", "t8_nut_secondary"): (
        "Zyltech anti-backlash complementary interlock envelope"),
    # spindle
    ("spindle_holder", "stator_final_wound_envelope"): "holder grips stator shaft",
    ("m1_coupling", "spindle_holder"): "coupling clamps holder shank",
    ("m1_coupling", "m1_lower_inner_spacer"): "inner-race stack clamp",
    # flyer
    ("alu_tube", "flyer_arm"): "flyer clamp fit",
    ("alu_tube", "flyer_pulley"): "pulley clamp fit",
    ("alu_tube", "wire_elbow"): "wire-guide press fit",
    ("tip_eyelet", "flyer_arm"): "ceramic eyelet press fit",
    ("flyer_arm", "m2_inner_front_spacer"): "inner-race stack seat",
    ("flyer_pulley", "m2_inner_rear_shim"): "inner-race stack seat",
}
DESIGNED_FITS = {frozenset(pair): meaning
                 for pair, meaning in DESIGNED_FITS.items()}


def _bvh(mesh: trimesh.Trimesh) -> fcl.BVHModel:
    model = fcl.BVHModel()
    model.beginModel(len(mesh.vertices), len(mesh.faces))
    model.addSubModel(mesh.vertices.astype(np.float64),
                      mesh.faces.astype(np.int64))
    model.endModel()
    return model


def _source_volume(a, b) -> float:
    try:
        # A build123d Compound can retain a parent placement while its child
        # solids retain their own placements. OCC booleans on two such
        # parents may compare children in parent-local space; that previously
        # turned the M2 shaft/pulley fit into a fictitious full-pulley overlap.
        # Rebuild identity-parent compounds from already-absolute children so
        # this source audit uses the same world placement as exported meshes.
        def absolute(shape):
            solids = list(shape.solids())
            if not solids:
                return shape
            result = solids[0] if len(solids) == 1 else Compound(children=solids)
            result.label = getattr(shape, "label", "part")
            return result

        common = absolute(a) & absolute(b)
        return 0.0 if common is None else float(common.volume)
    except Exception as exc:
        raise RuntimeError(
            f"OpenCascade common-volume failed for {a.label}/{b.label}: {exc}"
        ) from exc


def _occurrences():
    by_label = {}
    for values in hardware_placements.hardware_occurrences_by_link(P).values():
        for occurrence in values:
            by_label[occurrence.label] = occurrence
    return by_label


def _tslot_host(label: str, frame_hosts: dict[str, tuple[str, str]]):
    if label.startswith("frame_bracket_"):
        for bracket, (floor, upright) in frame_hosts.items():
            if label.startswith(bracket + "_floor_"):
                return floor
            if label.startswith(bracket + "_upright_"):
                return upright
    if label.startswith("rear_post_left_shoe_floor_"):
        return "cross_rear"
    if label.startswith("rear_post_left_shoe_upright_"):
        return "rear_post"
    if label.startswith("rail_L_"):
        return "stringer_L"
    if label.startswith("rail_R_"):
        return "stringer_R"
    if label.startswith(("m0_mount_", "m0_support_")):
        return "stringer_L"
    if label.startswith("endstop_pedestal_"):
        return "cross_front"
    if label.startswith("flyer_block_L_") or label.startswith("m2_mount_L_"):
        return "post_L"
    if label.startswith("flyer_block_R_") or label.startswith("m2_mount_R_"):
        return "post_R"
    if "_base_" in label and label.startswith(("spool_", "felt_", "dancer_", "entry_")):
        return "rear_post"
    if label.startswith("dancer_pivot_tnut"):
        return "rear_post"
    if label.startswith("foot_"):
        try:
            index = int(label.split("_")[1])
        except (ValueError, IndexError):
            return None
        return "base_rail_L" if index in (0, 2) else "base_rail_R"
    return None


def _motor_thread_pair(a: str, b: str) -> bool:
    pair = {a, b}
    return any(
        motor in pair and any(label.startswith(prefix) for label in pair)
        for motor, prefix in (
            ("m0_motor", "m0_motor_m3x10_"),
            ("m1_motor", "m1_motor_m3x10_"),
            ("m2_motor", "m2_motor_m3x10_"),
        )
    )


def _classify_positive(
    link: str,
    a: str,
    b: str,
    occurrences,
    frame_hosts,
):
    pair = frozenset((a, b))
    if pair in DESIGNED_FITS:
        return DESIGNED_FITS[pair]

    oa, ob = occurrences.get(a), occurrences.get(b)
    if oa is not None and ob is not None and oa.mate_id == ob.mate_id:
        return f"same selected hardware stack {oa.mate_id}"

    for hardware_label, host_label in ((a, b), (b, a)):
        expected = _tslot_host(hardware_label, frame_hosts)
        if expected == host_label:
            kind = "T-slot capture" if "tnut" in hardware_label else "T-slot screw passage"
            return kind

    if _motor_thread_pair(a, b):
        return "motor tapped-hole thread engagement"

    # Heat-set inserts intentionally displace printed plastic.  Set screws
    # intentionally enter the shaft/tube they clamp.
    if any("insert" in label for label in pair) and any(
            body in pair for body in (
                "flyer_arm", "flyer_pulley", "dancer_base",
            )):
        return "heat-set insert interference"
    if "alu_tube" in pair and any("m3x8" in label for label in pair):
        return "radial set-screw shaft engagement"

    if "dancer_extension_spring" in pair and any(
            label.startswith(("dancer_spring_fixed_", "dancer_spring_moving_"))
            for label in pair):
        return "extension-spring loop/anchor engagement"

    return None


def main() -> int:
    manifest = json.loads((LINKS / "manifest.json").read_text())
    try:
        job = manifest["stator"]
        option = spindle_option(manifest["spindle"]["id"])
        spec = StatorSpec(
            slots=int(job["slots"]),
            od=float(job["od"]),
            stack=float(job["stack"]),
            shaft_d=float(job["shaft_d"]),
            shaft_below=float(job["shaft_below"]),
            shaft_above=float(job["shaft_above"]),
            hub_od_ratio=float(job["hub_od_ratio"]),
            wire_d=float(job["wire_d"]),
            turns=int(job["turns"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(
            "manifest lacks the complete stator/spindle identity required "
            f"for a source-BREP audit: {exc}"
        ) from exc
    exact_links = assembly.build_links(
        spec, final_wound_collision=True, spindle=option)
    exact = {
        link: {part.label: part for part in parts}
        for link, parts in exact_links.items()
    }
    occurrences = _occurrences()
    frame_hosts = frame_hardware_audit.bracket_host_pairs(
        hardware_placements.current_geometry(P))

    identity = fcl.Transform(np.eye(3), np.zeros(3))
    accepted = []
    unexpected = []
    broadphase_contacts = 0
    for link, labels in manifest["parts"].items():
        meshes = {
            label: trimesh.load(
                LINKS / "parts" / link / f"{label}.stl", force="mesh")
            for label in labels
        }
        bvhs = {label: _bvh(mesh) for label, mesh in meshes.items()}
        missing = sorted(set(labels) - set(exact.get(link, {})))
        if missing:
            unexpected.extend({
                "link": link, "a": label, "b": None,
                "source_volume_mm3": None,
                "reason": "mesh label has no source BREP",
            } for label in missing)
            continue

        for index, a in enumerate(labels):
            for b in labels[index + 1:]:
                result = fcl.CollisionResult()
                fcl.collide(
                    fcl.CollisionObject(bvhs[a], identity),
                    fcl.CollisionObject(bvhs[b], identity),
                    fcl.CollisionRequest(), result,
                )
                if not result.is_collision:
                    continue
                broadphase_contacts += 1
                volume = _source_volume(exact[link][a], exact[link][b])
                if volume <= SOURCE_VOLUME_TOL_MM3:
                    accepted.append({
                        "link": link, "a": a, "b": b,
                        "source_volume_mm3": round(volume, 9),
                        "classification": "zero-volume contact or mesh artifact",
                    })
                    continue
                classification = _classify_positive(
                    link, a, b, occurrences, frame_hosts)
                record = {
                    "link": link, "a": a, "b": b,
                    "source_volume_mm3": round(volume, 6),
                    "classification": classification,
                }
                if classification is None:
                    record["reason"] = "positive source volume has no named interface"
                    unexpected.append(record)
                else:
                    accepted.append(record)

    report = {
        "schema": 2,
        "method": "FCL mesh broad phase plus OpenCascade source-BREP common volume",
        "source_volume_tolerance_mm3": SOURCE_VOLUME_TOL_MM3,
        "job": {
            "stator_od_mm": spec.od,
            "shaft_d_mm": spec.shaft_d,
            "spindle_id": option.id,
            "spindle_artifact_id": option.artifact_id,
        },
        "broadphase_contacts": broadphase_contacts,
        "accepted": accepted,
        "unexpected": unexpected,
        "passed": not unexpected,
    }
    (OUT / "reports").mkdir(exist_ok=True)
    (OUT / "reports" / "static_audit.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"{broadphase_contacts} mesh contacts; {len(accepted)} classified")
    if unexpected:
        print(f"{len(unexpected)} UNEXPECTED positive source overlaps:")
        for row in unexpected:
            print(f"  [{row['link']}] {row['a']} <-> {row['b']}: "
                  f"{row.get('source_volume_mm3')} mm3")
    else:
        print("no unexpected same-link source overlap")
    return 1 if unexpected else 0


if __name__ == "__main__":
    raise SystemExit(main())
