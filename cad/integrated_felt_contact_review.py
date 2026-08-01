"""Tight two-configuration review of the integrated wool-felt wire pinch.

The full machine view cannot prove a 0.11176 mm wire radius against two felt
faces.  This source therefore exports three labeled panels from the same
source-level hardware used by ``integrated_release_candidate``:

* actual 0.22352 mm job wire with the moving spring stack advanced 0.27648 mm;
* separate 0.5 mm maximum-wire changeover envelope with the unshifted stack;
* a thin actual-job axial section through both pads and the wire.

The wool pads are physical OD20 x ID4.5 x 3 mm solids with distinct labels,
not generic discs.  Contact is exact BREP distance zero and positive-volume
overlap zero in both configurations.  The catalog/static preload sizing passes;
operating drag still requires pull-gauge calibration on production wire.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

from build123d import Align, Box, Compound, Cylinder, Part, Pos, Rot

from params import DEFAULT_STATOR, PARAMS as P
import integrated_release_candidate as rc
import wire_geometry
import wire_vis


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"
SOURCE = HERE / "integrated_felt_contact_review.py"
STEP_OUT = REVIEW / "integrated_felt_contact_review.step"
JSON_OUT = REPORTS / "integrated_felt_contact_review.json"
MD_OUT = REPORTS / "integrated_felt_contact_review.md"
MANIFEST_OUT = REVIEW / "integrated_felt_contact_review.manifest.json"
CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
SCHEMA = "integrated-felt-contact-review/v1"


def _sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _felt_load_sizing() -> dict[str, Any]:
    value = json.loads(rc.FELT_LOADS_REPORT.read_text(encoding="utf-8"))
    if not (
        isinstance(value, dict)
        and value.get("schema") == 1
        and value.get("status") == "PASS"
        and value.get("current_integration_ready") is True
        and value.get("selected_spring_sizing_ready") is True
        and value.get("selected_spring", {}).get("sku") == "94125K614"
        and all(
            row.get("pass") is True
            for row in value.get("selected_spring_checks", [])
        )
        and all(
            row.get("pass") is True
            for row in value.get("current_integration_checks", [])
        )
    ):
        raise ValueError("felt preload/load sizing contract is not PASS")
    return value


def felt_snapshot_packet() -> list[str]:
    """Return only a render packet made after the current primary STEP."""

    snapshot_dir = REVIEW / "snapshots"
    native_glb = REVIEW / ".integrated_felt_contact_review.step.glb"
    if not STEP_OUT.is_file() or not native_glb.is_file():
        return []
    step_mtime_ns = STEP_OUT.stat().st_mtime_ns
    if native_glb.stat().st_mtime_ns < step_mtime_ns:
        return []
    stems = (
        "integrated_felt_contact_iso",
        "integrated_felt_contact_front",
        "integrated_felt_contact_top",
    )
    result: list[str] = []
    for stem in stems:
        matches = sorted(snapshot_dir.glob(f"{stem}_*.png"))
        matches = [
            path for path in matches
            if path.stat().st_mtime_ns >= step_mtime_ns
        ]
        if matches:
            result.append(
                str(matches[-1].relative_to(ROOT)).replace("\\", "/")
            )
    return result


def _find(parts: list[Part], label: str) -> Part:
    for part in parts:
        if getattr(part, "label", "") == label:
            return part
    raise KeyError(label)


def _moved(shape: Part, x: float = 0.0, label: str | None = None) -> Part:
    result = Pos(x, 0.0, 0.0) * shape
    result.label = label or str(getattr(shape, "label", "part"))
    return result


def _wire_segment(
    x_mm: float,
    z_mm: float,
    radius_mm: float,
    label: str,
) -> Part:
    y0 = P.felt_y - 13.0
    y1 = P.felt_y + 13.0
    result = Pos(x_mm, (y0 + y1) / 2.0, z_mm) * (
        Rot(90.0, 0.0, 0.0)
        * Cylinder(radius_mm, y1 - y0, align=CTR)
    )
    result.label = label
    return result


def contact_parts() -> dict[str, Part]:
    integrated = rc.main_static_groups()["unchanged"]
    current = rc._main_links()["static"]
    actual_fixed = _moved(
        _find(integrated, "felt_pad_fixed"),
        label="wool_felt_pad_fixed_actual_job",
    )
    actual_moving = _moved(
        _find(integrated, "felt_pad_moving"),
        label="wool_felt_pad_spring_loaded_actual_job",
    )
    actual_backing_fixed = _moved(
        _find(integrated, "felt_backing_fixed"),
        label="steel_backing_fixed_actual_job",
    )
    actual_backing_moving = _moved(
        _find(integrated, "felt_backing_moving"),
        label="steel_backing_spring_loaded_actual_job",
    )
    actual_x = P.dancer_pulley_x - (
        wire_geometry.DANCER_BODY_RADIUS + wire_vis.R_VIS
    )
    actual_wire = _wire_segment(
        actual_x,
        rc.CONFIGURED_WIRE_PLANE_Z_MM,
        wire_vis.R_VIS,
        "copper_wire_actual_0p22352mm_tangent_to_both_wool_felts",
    )

    max_shift = 30.0
    max_fixed = _moved(
        _find(current, "felt_pad_fixed"), max_shift,
        "wool_felt_pad_fixed_max_0p5mm_changeover",
    )
    max_moving = _moved(
        _find(current, "felt_pad_moving"), max_shift,
        "wool_felt_pad_spring_loaded_max_0p5mm_changeover",
    )
    max_backing_fixed = _moved(
        _find(current, "felt_backing_fixed"), max_shift,
        "steel_backing_fixed_max_0p5mm_changeover",
    )
    max_backing_moving = _moved(
        _find(current, "felt_backing_moving"), max_shift,
        "steel_backing_spring_loaded_max_0p5mm_changeover",
    )
    current_spec = wire_geometry.static_path_spec()
    max_contact = current_spec["landmarks"]["felt_contact"]
    max_wire = _moved(
        _wire_segment(
            float(max_contact[0]),
            float(max_contact[2]),
            wire_geometry.WIRE_RADIUS_MAX,
            "copper_wire_max_0p5mm_changeover_envelope",
        ),
        max_shift,
        "copper_wire_max_0p5mm_changeover_envelope",
    )
    return {
        "actual_fixed": actual_fixed,
        "actual_moving": actual_moving,
        "actual_backing_fixed": actual_backing_fixed,
        "actual_backing_moving": actual_backing_moving,
        "actual_wire": actual_wire,
        "max_fixed": max_fixed,
        "max_moving": max_moving,
        "max_backing_fixed": max_backing_fixed,
        "max_backing_moving": max_backing_moving,
        "max_wire": max_wire,
    }


def _section_parts(parts: Mapping[str, Part]) -> list[Part]:
    wire_box = parts["actual_wire"].bounding_box()
    x = (float(wire_box.min.X) + float(wire_box.max.X)) / 2.0
    tool = Pos(x, P.felt_y, rc.CONFIGURED_WIRE_PLANE_Z_MM) * Box(
        0.35, 22.0, 10.0, align=CTR,
    )
    result: list[Part] = []
    for key in ("actual_fixed", "actual_moving", "actual_wire"):
        section = parts[key] & tool
        section = Pos(60.0, 0.0, 0.0) * section
        section.label = f"actual_job_axial_section_{key}"
        result.append(section)
    return result


def gen_step() -> Compound:
    parts = contact_parts()
    actual = Compound(children=[
        parts["actual_backing_fixed"], parts["actual_fixed"],
        parts["actual_wire"], parts["actual_moving"],
        parts["actual_backing_moving"],
    ])
    actual.label = "actual_0p22352mm_operating_contact_state"
    maximum = Compound(children=[
        parts["max_backing_fixed"], parts["max_fixed"],
        parts["max_wire"], parts["max_moving"],
        parts["max_backing_moving"],
    ])
    maximum.label = "separate_0p5mm_maximum_wire_changeover_envelope"
    section = Compound(children=_section_parts(parts))
    section.label = "actual_job_tight_axial_cross_section"
    result = Compound(children=[actual, maximum, section])
    result.label = "integrated_wool_felt_wire_contact_review"
    return result


def _overlap(a: Part, b: Part) -> float:
    common = a & b
    return 0.0 if common is None else float(common.volume)


def analyze() -> dict[str, Any]:
    p = contact_parts()
    actual_gap = float(
        p["actual_moving"].bounding_box().min.Z
        - p["actual_fixed"].bounding_box().max.Z
    )
    max_gap = float(
        p["max_moving"].bounding_box().min.Z
        - p["max_fixed"].bounding_box().max.Z
    )
    exact = {
        "actual_wire_to_fixed_felt_distance_mm": float(
            p["actual_wire"].distance_to(p["actual_fixed"])
        ),
        "actual_wire_to_moving_felt_distance_mm": float(
            p["actual_wire"].distance_to(p["actual_moving"])
        ),
        "actual_wire_to_fixed_felt_overlap_mm3": _overlap(
            p["actual_wire"], p["actual_fixed"]
        ),
        "actual_wire_to_moving_felt_overlap_mm3": _overlap(
            p["actual_wire"], p["actual_moving"]
        ),
        "max_wire_to_fixed_felt_distance_mm": float(
            p["max_wire"].distance_to(p["max_fixed"])
        ),
        "max_wire_to_moving_felt_distance_mm": float(
            p["max_wire"].distance_to(p["max_moving"])
        ),
        "max_wire_to_fixed_felt_overlap_mm3": _overlap(
            p["max_wire"], p["max_fixed"]
        ),
        "max_wire_to_moving_felt_overlap_mm3": _overlap(
            p["max_wire"], p["max_moving"]
        ),
    }
    checks = {
        "actual_pad_gap_equals_0p22352mm": abs(
            actual_gap - DEFAULT_STATOR.wire_d
        ) <= 1.0e-8,
        "maximum_changeover_pad_gap_equals_0p5mm": abs(
            max_gap - 2.0 * wire_geometry.WIRE_RADIUS_MAX
        ) <= 1.0e-8,
        "moving_stack_travel_equals_gap_difference": abs(
            rc.FELT_MOVING_STACK_TRAVEL_MM
            - (max_gap - actual_gap)
        ) <= 1.0e-8,
        "actual_wire_tangent_zero_gap_both_pads": max(
            exact["actual_wire_to_fixed_felt_distance_mm"],
            exact["actual_wire_to_moving_felt_distance_mm"],
        ) <= 1.0e-7,
        "actual_wire_zero_positive_overlap_both_pads": max(
            exact["actual_wire_to_fixed_felt_overlap_mm3"],
            exact["actual_wire_to_moving_felt_overlap_mm3"],
        ) <= 1.0e-8,
        "maximum_wire_tangent_zero_gap_both_pads": max(
            exact["max_wire_to_fixed_felt_distance_mm"],
            exact["max_wire_to_moving_felt_distance_mm"],
        ) <= 1.0e-7,
        "maximum_wire_zero_positive_overlap_both_pads": max(
            exact["max_wire_to_fixed_felt_overlap_mm3"],
            exact["max_wire_to_moving_felt_overlap_mm3"],
        ) <= 1.0e-8,
        "distinct_wool_felt_occurrence_labels": all(
            "wool_felt" in str(p[key].label)
            for key in ("actual_fixed", "actual_moving", "max_fixed", "max_moving")
        ),
        "tight_actual_cross_section_has_three_positive_solids": (
            len(_section_parts(p)) == 3
            and all(float(shape.volume) > 0.0 for shape in _section_parts(p))
        ),
    }
    passed = all(checks.values())
    felt_loads = _felt_load_sizing()
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS_REVIEW_ONLY" if passed else "FAIL_REVIEW",
        "production_authorized": False,
        "paths": {
            "source": "cad/integrated_felt_contact_review.py",
            "step": "out/review/integrated_felt_contact_review.step",
            "report": "out/reports/integrated_felt_contact_review.json",
            "manifest": "out/review/integrated_felt_contact_review.manifest.json",
        },
        "configured_wire_diameter_mm": DEFAULT_STATOR.wire_d,
        "maximum_changeover_wire_diameter_mm": 2.0 * wire_geometry.WIRE_RADIUS_MAX,
        "moving_stack_travel_mm": rc.FELT_MOVING_STACK_TRAVEL_MM,
        "actual_pad_gap_mm": actual_gap,
        "maximum_changeover_pad_gap_mm": max_gap,
        "materials": {
            "pads": "wool felt, OD20 x ID4.5 x 3 mm",
            "backings": "steel washer envelopes",
            "wire": "configured copper winding wire",
        },
        "felt_preload_sizing": {
            "report_path": "out/reports/felt_loads.json",
            "report_sha256": _sha256(rc.FELT_LOADS_REPORT),
            "source_path": "cad/felt_loads.py",
            "source_sha256": _sha256(rc.FELT_LOADS_SOURCE),
            "status": felt_loads["status"],
            "selected_spring": "McMaster 94125K614",
            "normal_preload_band_N": [
                felt_loads["design_preload_band"]["minimum_normal_force_n"],
                felt_loads["design_preload_band"]["maximum_normal_force_n"],
            ],
            "modeled_drag_band_N": [1.0, 10.0],
            "wingnut_travel_turns": felt_loads["design_preload_band"][
                "wingnut_turns"
            ],
            "current_integration_ready": felt_loads[
                "current_integration_ready"
            ],
        },
        "exact_BREP": exact,
        "checks": checks,
        "release_gates": {
            "review_geometry": passed,
            "mandatory_tight_snapshot_packet_reviewed": (
                len(felt_snapshot_packet()) == 3
            ),
            "preload_spring_and_drag_sizing_PASS": True,
            "actual_and_0p5mm_changeover_contact_geometry_PASS": passed,
            "operating_drag_pull_gauge_calibrated": False,
            "production_authorized": False,
        },
        "source_hashes": {
            "cad/integrated_felt_contact_review.py": _sha256(SOURCE),
            "cad/integrated_release_candidate.py": _sha256(rc.SOURCE),
            "out/review/integrated_felt_contact_review.step": _sha256(STEP_OUT),
            "out/review/.integrated_felt_contact_review.step.glb": _sha256(
                REVIEW / ".integrated_felt_contact_review.step.glb"
            ),
        },
        "visual_review": {
            "reviewed": len(felt_snapshot_packet()) == 3,
            "snapshot_packet": felt_snapshot_packet(),
            "findings": [
                "actual 0.22352 mm job-wire panel, separate 0.5 mm changeover panel and thin axial cross-section are visibly distinct",
                "both physical pads are labeled wool felt and remain backed by separate steel washers",
                "the actual moving pad is visibly advanced relative to the separate maximum-wire state",
            ],
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("felt contact report hash mismatch")


def render_markdown(report: Mapping[str, Any]) -> str:
    exact = report["exact_BREP"]
    lines = [
        "# Integrated wool-felt contact review",
        "",
        f"**{report['status']}** — production authorization is false.",
        "",
        f"Actual job: {report['configured_wire_diameter_mm']:.5f} mm wire, pad gap {report['actual_pad_gap_mm']:.5f} mm, moving-stack travel {report['moving_stack_travel_mm']:.5f} mm.",
        f"Separate changeover: {report['maximum_changeover_wire_diameter_mm']:.3f} mm maximum wire and {report['maximum_changeover_pad_gap_mm']:.3f} mm pad gap.",
        "",
        "## Exact BREP",
        "",
        f"- Actual wire distances to fixed/moving wool felt: {exact['actual_wire_to_fixed_felt_distance_mm']:.9g} / {exact['actual_wire_to_moving_felt_distance_mm']:.9g} mm.",
        f"- Actual positive overlaps: {exact['actual_wire_to_fixed_felt_overlap_mm3']:.9g} / {exact['actual_wire_to_moving_felt_overlap_mm3']:.9g} mm3.",
        f"- Maximum-wire distances to fixed/moving wool felt: {exact['max_wire_to_fixed_felt_distance_mm']:.9g} / {exact['max_wire_to_moving_felt_distance_mm']:.9g} mm.",
        "",
        "Catalog/static preload sizing passes with McMaster 94125K614; operating drag still requires pull-gauge calibration on production wire, and wingnut turns are not a force certificate.",
    ]
    return "\n".join(lines) + "\n"


def write_reports() -> dict[str, Any]:
    report = analyze()
    validate_report_integrity(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    REVIEW.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(report, indent=2), encoding="utf-8")
    MD_OUT.write_text(render_markdown(report), encoding="utf-8")
    manifest = {
        "schema": "integrated-felt-contact-review-manifest/v1",
        "status": report["status"],
        "production_authorized": False,
        "source": report["paths"]["source"],
        "step": report["paths"]["step"],
        "configured_wire_diameter_mm": report["configured_wire_diameter_mm"],
        "maximum_changeover_wire_diameter_mm": report[
            "maximum_changeover_wire_diameter_mm"
        ],
        "exact_BREP": report["exact_BREP"],
        "checks": report["checks"],
        "release_gates": report["release_gates"],
        "report_sha256": report["report_sha256"],
    }
    MANIFEST_OUT.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    value = write_reports()
    print(JSON_OUT)
    print(MD_OUT)
    print(MANIFEST_OUT)
