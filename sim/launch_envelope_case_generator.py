"""Generate exact, fail-closed evidence bundles for the 24 launch corners.

This is an evidence generator, not an authority shortcut.  It derives a
deterministic representative winding plan at the highest integer turn count
not exceeding the analytical design-fill target, exports the exact stator /
workholding / conservative wound-envelope STEP, and runs the untouched
upstream controller under the repository's virtual-time capture harness when
the job is geometrically feasible.  The independent cycle verifier is then
run on that raw command stream.

No bundle produced here claims corner authority.  Full raw rigid collision,
continuous flexible-conductor behavior, exact two-turn shaft wraps, and
physical coupons remain separate gates.  ``launch_envelope_authority.py``
recognizes these bundles as current evidence while keeping production false.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Mapping, Sequence


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from build123d import Compound, export_step, import_step  # noqa: E402

import assembly  # noqa: E402
import coil_growth  # noqa: E402
import launch_envelope_authority as authority  # noqa: E402
import loads  # noqa: E402
from params import PARAMS as P, StatorSpec, spindle_option  # noqa: E402
import settings_gen  # noqa: E402
import wire_geometry  # noqa: E402


SCHEMA = "launch-envelope-evidence-generation/v1"
PACKING_SCHEMA = "launch-case-packing-preflight/v1"
COLLISION_SCHEMA = "launch-case-collision-preflight/v1"
WIRE_SCHEMA = "launch-case-wire-preflight/v1"
LOAD_SCHEMA = "launch-case-load-preflight/v1"
BUILDABILITY_SCHEMA = "launch-case-buildability-preflight/v1"
CAPTURE_UNAVAILABLE_SCHEMA = "launch-case-capture-unavailable/v1"
CAPTURE_PROVENANCE_SCHEMA = "launch-case-capture-provenance/v1"

DEFAULT_ROOT = ROOT
DEFAULT_CERTIFICATE_ROOT = ROOT / "out" / "launch_certificates"
DEFAULT_JSON = ROOT / "out" / "reports" / "launch_envelope_generation.json"
DEFAULT_MD = ROOT / "out" / "reports" / "launch_envelope_generation.md"

FIXED_BUILDABILITY = "out/reports/buildability.json"
SHAFT8_STEP = "out/custom/step/shaft8_socket_holder.step"
SHAFT8_DRAWING = "output/pdf/shaft8_socket_holder.pdf"
ER11_STEP = "cad/models/upgrades/er11_c8_hifi.step"

STEP_TIMESTAMP_RE = re.compile(
    r"(FILE_NAME\('[^']*',')[^']*(')", re.MULTILINE,
)
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*m")
WALL_CLOCK_LOG_RE = re.compile(
    r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3} - "
)


@dataclass(frozen=True)
class CasePlan:
    case: authority.LaunchCase
    turns_per_tooth: int
    feasible: bool
    one_turn_analysis: dict[str, Any]
    selected_analysis: dict[str, Any]
    rejection_reasons: tuple[str, ...]

    @property
    def spec(self) -> StatorSpec:
        return StatorSpec(
            slots=self.case.slots,
            od=self.case.od_mm,
            stack=self.case.stack_mm,
            shaft_d=self.case.shaft_d_mm,
            wire_d=self.case.wire_finished_d_mm,
            turns=self.turns_per_tooth,
            hub_od_ratio=self.case.hub_od_ratio,
            winding_config=self.case.winding_config,
        )

    @property
    def job(self) -> dict[str, Any]:
        return {
            **self.case.expected_job(),
            "turns_per_tooth": self.turns_per_tooth,
        }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_hash(value: Mapping[str, Any], *, omit: Sequence[str] = ()) -> str:
    payload = {key: val for key, val in value.items() if key not in omit}
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return "Infinity" if value > 0 else "-Infinity"
    if isinstance(value, Mapping):
        return {str(key): _json_safe(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _placement_band(
    spec: StatorSpec,
    analysis: Mapping[str, Any],
    spindle_id: str,
) -> dict[str, Any]:
    """Return the occupied radial band for this finite turn count.

    ``coil_growth`` reports the complete wire-accessible slot span because it
    owns capacity.  A one-turn job does not occupy that entire span.  The raw
    controller distributes a finite number of turns across M0, so cap the
    commanded band to one finished-wire pitch per turn and anchor it at the
    shoe-side end.  High-capacity stress cases naturally retain the complete
    accessible span.  The conservative STEP still includes the full final
    wound envelope; this narrower motion band cannot hide a collision.
    """

    bundle = analysis["bundle"]
    full_start = float(bundle["radial_winding_start_mm"])
    full_end = float(bundle["radial_winding_end_mm"])
    full_span = full_end - full_start
    if full_span <= 0.0:
        occupied_start = full_start
        occupied_end = full_end
        occupied_span = full_span
    else:
        occupied_span = min(full_span, spec.turns * spec.wire_d)
        occupied_end = full_end
        occupied_start = occupied_end - occupied_span
    d_start = spec.od / 2.0 - occupied_start - wire_geometry.TOOTH_CONTACT_Z
    d_end = spec.od / 2.0 - occupied_end - wire_geometry.TOOTH_CONTACT_Z
    maximum = P.max_insertion(spec, spindle_id)
    return {
        "rule": (
            "shoe-anchored finite placement band, at most one finished-wire "
            "radial pitch per configured turn"
        ),
        "full_accessible_radial_span_mm": [full_start, full_end],
        "occupied_radial_span_mm": [occupied_start, occupied_end],
        "occupied_span_mm": occupied_span,
        "turn_pitch_budget_mm": spec.turns * spec.wire_d,
        "winding_insertion_depth_mm": [d_end, d_start],
        "maximum_workholder_insertion_mm": maximum,
        "workholder_reach_margin_mm": maximum - d_start,
    }


def _superseded_od28_24n22p_throat_proof() -> dict[str, Any]:
    """Exact section proof for the rejected fixed-24-slot assumption."""

    wire_d = 0.5
    liner = coil_growth.DEFAULT_POLICY.opening_edge_clearance_mm
    spec = StatorSpec(
        slots=24, od=28.0, stack=5.0, shaft_d=3.0,
        wire_d=wire_d, turns=1,
        hub_od_ratio=0.52,
        winding_config=authority.UPSTREAM_24N22P_WINDING_CONFIG,
    )
    geometry = coil_growth.slot_geometry(spec)
    shoe_inner = float(geometry["shoe_inner_radius_mm"])
    half_neck = float(geometry["tooth_neck_width_mm"]) / 2.0
    sine_half_pitch = math.sin(math.pi / spec.slots)
    # Along the slot bisector, clearance to either neck is r*sin(pi/N)-w/2
    # and clearance to the shoe underface is shoe_inner-r.  Their equality is
    # the maximum inscribed center clearance behind the shoe.
    optimum_radius = (
        shoe_inner + half_neck
    ) / (1.0 + sine_half_pitch)
    maximum_center_clearance = shoe_inner - optimum_radius
    required_center_clearance = wire_d / 2.0 + liner
    maximum_finished_wire = 2.0 * (maximum_center_clearance - liner)
    maximum_half_neck = (
        shoe_inner * sine_half_pitch
        - required_center_clearance * (1.0 + sine_half_pitch)
    )
    legacy = coil_growth.analyze_job(spec)
    return {
        "classification": (
            "physically contradictory synthetic lamination/liner combination; "
            "not a universal OD28 machine-envelope failure"
        ),
        "superseded_assumption": (
            "OD28 forced to 24n22p, hub ratio 0.52, 2.5 mm tooth neck, "
            "1.6 mm shoe, and 0.127 mm liner"
        ),
        "slot_half_pitch_deg": 180.0 / spec.slots,
        "shoe_inner_radius_mm": shoe_inner,
        "tooth_neck_width_mm": 2.0 * half_neck,
        "liner_each_side_mm": liner,
        "wire_finished_diameter_mm": wire_d,
        "required_wire_center_core_clearance_mm": required_center_clearance,
        "maximum_inscribed_center_clearance_mm": maximum_center_clearance,
        "center_clearance_deficit_mm": (
            required_center_clearance - maximum_center_clearance
        ),
        "maximum_compatible_finished_wire_mm": maximum_finished_wire,
        "finished_wire_diameter_deficit_mm": wire_d - maximum_finished_wire,
        "optimum_slot_bisector_center_radius_mm": optimum_radius,
        "maximum_neck_width_for_0p5_wire_and_0p127_liner_mm": (
            2.0 * maximum_half_neck
        ),
        "minimum_neck_width_reduction_mm": (
            2.0 * half_neck - 2.0 * maximum_half_neck
        ),
        "coil_growth_access_span_mm": legacy["slot_access"][
            "accessible_radial_span_mm"
        ],
        "coil_growth_max_turns_at_design_fill": legacy["packing"][
            "max_turns_at_design_fill"
        ],
        "resolution": (
            "bind OD28 launch evidence to upstream's 12n14p representative "
            "topology and retain per-job measured-lamination authority"
        ),
    }


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_json_safe(value), indent=2, sort_keys=True,
                   allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _relative(root: Path, path: Path) -> str:
    return Path(path).resolve().relative_to(Path(root).resolve()).as_posix()


def _artifact_row(root: Path, path: Path, job: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "path": _relative(root, path),
        "sha256": _sha256(path),
        "job_identity": dict(job),
    }


def _report_hash(report: dict[str, Any]) -> dict[str, Any]:
    report["report_sha256"] = _canonical_hash(
        report, omit=("report_sha256",),
    )
    return report


def derive_case_plan(case: authority.LaunchCase) -> CasePlan:
    """Select a deterministic capacity-stress plan for one geometry corner."""

    one_turn_spec = StatorSpec(
        slots=case.slots,
        od=case.od_mm,
        stack=case.stack_mm,
        shaft_d=case.shaft_d_mm,
        wire_d=case.wire_finished_d_mm,
        turns=1,
        hub_od_ratio=case.hub_od_ratio,
        winding_config=case.winding_config,
    )
    one_turn = coil_growth.analyze_job(one_turn_spec)
    one_turn = {
        **one_turn,
        "representative_placement_band": _placement_band(
            one_turn_spec, one_turn, case.spindle_id,
        ),
    }
    maximum_design_turns = int(
        one_turn["packing"]["max_turns_at_design_fill"]
    )
    turns = max(1, maximum_design_turns)
    selected_spec = StatorSpec(
        slots=case.slots,
        od=case.od_mm,
        stack=case.stack_mm,
        shaft_d=case.shaft_d_mm,
        wire_d=case.wire_finished_d_mm,
        turns=turns,
        hub_od_ratio=case.hub_od_ratio,
        winding_config=case.winding_config,
    )
    selected = coil_growth.analyze_job(selected_spec)
    selected = {
        **selected,
        "representative_placement_band": _placement_band(
            selected_spec, selected, case.spindle_id,
        ),
    }
    reach_margin = float(selected["representative_placement_band"][
        "workholder_reach_margin_mm"
    ])
    feasible = bool(
        maximum_design_turns >= 1 and selected.get("status") == "PASS"
        and reach_margin + 1.0e-9 >= case.reach_reserve_mm
    )
    reasons_list = list(selected.get("reasons", []) if not feasible else [])
    if (selected.get("status") == "PASS"
            and reach_margin + 1.0e-9 < case.reach_reserve_mm):
        reasons_list.append(
            f"representative placement leaves {reach_margin:.3f} mm "
            f"workholder reach reserve, below {case.reach_reserve_mm:.3f} mm"
        )
    reasons = tuple(str(reason) for reason in reasons_list)
    return CasePlan(
        case=case,
        turns_per_tooth=turns,
        feasible=feasible,
        one_turn_analysis=_json_safe(one_turn),
        selected_analysis=_json_safe(selected),
        rejection_reasons=reasons,
    )


def derive_all_case_plans() -> tuple[CasePlan, ...]:
    return tuple(derive_case_plan(case) for case in authority.required_launch_cases())


def _normalize_step_header(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    normalized, count = STEP_TIMESTAMP_RE.subn(
        r"\g<1>1970-01-01T00:00:00\g<2>", text, count=1,
    )
    if count != 1:
        raise RuntimeError(f"could not normalize STEP timestamp in {path}")
    path.write_text(normalized, encoding="utf-8", newline="\n")


def _shape_facts(shape: Any) -> dict[str, Any]:
    solids = list(shape.solids())
    volumes = [float(solid.volume) for solid in solids]
    bbox = shape.bounding_box()
    return {
        "solid_count": len(solids),
        "all_solids_positive_volume": bool(
            solids and all(volume > 0.0 for volume in volumes)
        ),
        "total_volume_mm3": sum(volumes),
        "bounds_mm": [
            [float(bbox.min.X), float(bbox.min.Y), float(bbox.min.Z)],
            [float(bbox.max.X), float(bbox.max.Y), float(bbox.max.Z)],
        ],
    }


def _generate_job_step(plan: CasePlan, path: Path) -> tuple[dict[str, Any], list[Any]]:
    parts = assembly.spindle_link(
        plan.spec,
        final_wound_collision=plan.feasible,
        spindle=plan.case.spindle_id,
    )
    shape = Compound(children=list(parts))
    source_facts = _shape_facts(shape)
    path.parent.mkdir(parents=True, exist_ok=True)
    export_step(shape, str(path))
    _normalize_step_header(path)
    imported = import_step(str(path))
    imported_facts = _shape_facts(imported)
    valid = bool(
        source_facts["all_solids_positive_volume"]
        and imported_facts["all_solids_positive_volume"]
        and imported_facts["solid_count"] == source_facts["solid_count"]
        and math.isclose(
            imported_facts["total_volume_mm3"],
            source_facts["total_volume_mm3"],
            rel_tol=1.0e-8,
            abs_tol=1.0e-5,
        )
    )
    return {
        "valid_closed_geometry": valid,
        "exact_core_and_selected_workholding": valid,
        "conservative_final_wound_envelope_included": plan.feasible,
        "source_brep": source_facts,
        "reimported_step": imported_facts,
        "sha256": _sha256(path),
    }, parts


def _packing_report(plan: CasePlan) -> dict[str, Any]:
    analysis = plan.selected_analysis
    packing = analysis["packing"]
    opening = analysis["slot_opening"]
    return _report_hash({
        "schema": PACKING_SCHEMA,
        "status": "PASS" if plan.feasible else "FAIL",
        "job": plan.job,
        "selection_rule": (
            "highest positive integer turns at or below analytical design "
            "slot-fill target for the case-bound representative lamination"
        ),
        "representative_lamination": {
            "slots": plan.case.slots,
            "hub_od_mm": plan.case.od_mm * plan.case.hub_od_ratio,
            "hub_od_ratio": plan.case.hub_od_ratio,
            "winding_config": plan.case.winding_config,
            "upstream_config_id": plan.case.upstream_config_id,
            "topology_basis": plan.case.topology_basis,
            "workholder_reach_reserve_mm": plan.case.reach_reserve_mm,
        },
        "superseded_OD28_24n22p_throat_proof": (
            _superseded_od28_24n22p_throat_proof()
            if plan.case.od_mm == 28.0 else None
        ),
        "one_turn_feasible": plan.one_turn_analysis.get("status") == "PASS",
        "capacity_and_opening_pass": plan.feasible,
        "maximum_turns_at_design_fill": int(
            plan.one_turn_analysis["packing"]["max_turns_at_design_fill"]
        ),
        "maximum_turns_at_hard_fill": int(
            plan.one_turn_analysis["packing"]["max_turns_at_maximum_fill"]
        ),
        "selected_gross_slot_fill": packing["gross_slot_fill"],
        "design_slot_fill_limit": packing["design_slot_fill_limit"],
        "hard_slot_fill_limit": packing["maximum_slot_fill_limit"],
        "slot_opening_margin_mm": opening["margin_mm"],
        "representative_placement_band": analysis[
            "representative_placement_band"
        ],
        "rejection_reasons": list(plan.rejection_reasons),
        "analysis": analysis,
        "limits": [
            "This analytical topology is not the OD46 measured-wire Hamiltonian packing certificate.",
            "The launch matrix proves representative feasible laminations, not every internal lamination geometry at the same OD and shaft endpoints.",
            "A per-job measured lamination section must regenerate packing and workholder reach before hardware motion is authorized.",
            "A real stator lamination drawing and receiving measurements remain required for an order-specific plan.",
        ],
    })


def _settings_evidence(plan: CasePlan, bundle: Path) -> tuple[Path, dict[str, Any] | None, dict[str, Any]]:
    if not plan.feasible:
        path = bundle / "settings_rejected.json"
        report = _report_hash({
            "schema": "launch-case-settings-rejection/v1",
            "status": "FAIL",
            "job": plan.job,
            "physical_travel_within_limits": False,
            "reason": "packing/access feasibility failed before motion settings generation",
            "rejection_reasons": list(plan.rejection_reasons),
        })
        _write_json(path, report)
        return path, None, report
    cfg = settings_gen.derive(plan.spec, plan.case.spindle_id)
    if int(cfg["winding"]["turns"]) != plan.turns_per_tooth:
        raise RuntimeError("derived settings turns drifted from case plan")
    placement = plan.selected_analysis["representative_placement_band"]
    d_end, d_start = [
        float(value) for value in placement["winding_insertion_depth_mm"]
    ]
    radius = plan.spec.od / 2.0

    def m0(axis_z: float) -> float:
        return round(P.m0_rad_for_axis_z(axis_z), 3)

    wind_range_start = m0(radius - d_start)
    wind_range_end = m0(radius - d_end)
    if not wind_range_start < wind_range_end:
        raise RuntimeError("representative placement band has no M0 travel")
    m0_rotating = m0(radius + 12.0)
    m0_zero = m0(radius + 20.0)
    cfg["motor"]["M0"]["wind_range_start"] = wind_range_start
    cfg["motor"]["M0"]["wind_range_end"] = wind_range_end
    cfg["motor"]["M0"]["end_to_zero"] = round(
        m0_zero - wind_range_end, 3,
    )
    cfg["motor"]["M1"]["end_to_rotating_position"] = round(
        m0_rotating - wind_range_end, 3,
    )
    cfg["job"]["winding_insertion_depth_mm"] = [d_end, d_start]
    cfg["job"]["radial_winding_span_mm"] = list(
        placement["occupied_radial_span_mm"]
    )
    cfg["job"].update({
        "hub_od_mm": plan.case.od_mm * plan.case.hub_od_ratio,
        "hub_od_ratio": plan.case.hub_od_ratio,
        "winding_config": plan.case.winding_config,
        "upstream_config_id": plan.case.upstream_config_id,
        "topology_basis": plan.case.topology_basis,
        "reach_reserve_mm": plan.case.reach_reserve_mm,
        "workholder_reach_margin_mm": placement[
            "workholder_reach_margin_mm"
        ],
        "placement_band_rule": placement["rule"],
        "full_accessible_radial_span_mm": placement[
            "full_accessible_radial_span_mm"
        ],
    })
    path = bundle / "settings.yml"
    topology_yaml = "".join((
        f"  hub_od_mm: {cfg['job']['hub_od_mm']}\n",
        f"  hub_od_ratio: {cfg['job']['hub_od_ratio']}\n",
        f"  winding_config: {json.dumps(cfg['job']['winding_config'])}\n",
        f"  upstream_config_id: {json.dumps(cfg['job']['upstream_config_id'])}\n",
        f"  topology_basis: {json.dumps(cfg['job']['topology_basis'])}\n",
        f"  reach_reserve_mm: {cfg['job']['reach_reserve_mm']}\n",
        "  workholder_reach_margin_mm: "
        f"{cfg['job']['workholder_reach_margin_mm']}\n",
        f"  placement_band_rule: {json.dumps(cfg['job']['placement_band_rule'])}\n",
        "  full_accessible_radial_span_mm: "
        f"{cfg['job']['full_accessible_radial_span_mm']}\n",
    ))
    path.write_text(
        settings_gen.to_yaml(cfg) + topology_yaml,
        encoding="utf-8", newline="\n",
    )
    verdict = {
        "status": "PASS",
        "job_identity_bound": True,
        "physical_travel_within_limits": True,
        "wind_range_rad": [
            cfg["motor"]["M0"]["wind_range_start"],
            cfg["motor"]["M0"]["wind_range_end"],
        ],
        "representative_placement_band": placement,
        "required_workholder_reach_reserve_mm": plan.case.reach_reserve_mm,
        "hardware_motion_authorized": False,
    }
    return path, cfg, verdict


def _run_command(command: Sequence[str], *, cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(command), cwd=str(cwd), text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
    )


def _stable_subprocess_output(value: str) -> str:
    """Remove logger wall-clock rows while retaining deterministic summaries."""

    lines = []
    for raw in value.splitlines():
        line = ANSI_ESCAPE_RE.sub("", raw).rstrip()
        if WALL_CLOCK_LOG_RE.match(line):
            continue
        if line:
            lines.append(line)
    return "\n".join(lines)


def _git_provenance(path: Path) -> dict[str, Any]:
    commit = _run_command(
        ("git", "-C", str(path), "rev-parse", "HEAD"), cwd=ROOT,
    )
    status = _run_command(
        ("git", "-C", str(path), "status", "--porcelain"), cwd=ROOT,
    )
    return {
        "repository": str(path.resolve()),
        "commit": commit.stdout.strip() if commit.returncode == 0 else None,
        "commit_command_returncode": commit.returncode,
        "dirty": bool(status.stdout.strip()) if status.returncode == 0 else None,
        "status_command_returncode": status.returncode,
        "status_output": status.stdout.strip(),
    }


def _capture_evidence(
    plan: CasePlan,
    bundle: Path,
    settings_path: Path,
    *,
    run_capture: bool,
) -> tuple[Path, Path, Path, dict[str, Any]]:
    provenance_path = bundle / "capture_provenance.json"
    audit_path = bundle / "capture_audit.json"
    winder = ROOT.parent / "winder"
    provenance = {
        "schema": CAPTURE_PROVENANCE_SCHEMA,
        "status": "BLOCKED",
        "job": plan.job,
        "controller_mode": "upstream",
        "winder": _git_provenance(winder) if winder.is_dir() else {
            "repository": str(winder), "commit": None, "dirty": None,
            "error": "winder checkout is missing",
        },
    }

    if not plan.feasible or not run_capture:
        capture_path = bundle / "capture_unavailable.json"
        reason = (
            "case packing/access is infeasible"
            if not plan.feasible else
            "capture execution disabled by caller"
        )
        unavailable = _report_hash({
            "schema": CAPTURE_UNAVAILABLE_SCHEMA,
            "status": "BLOCKED",
            "job": plan.job,
            "reason": reason,
            "cycle_complete": False,
            "unmodified_upstream": False,
            "both_shaft_wraps_exactly_two_turns": False,
        })
        _write_json(capture_path, unavailable)
        _write_json(audit_path, unavailable)
        provenance["status"] = "BLOCKED"
        provenance["reason"] = reason
        _write_json(provenance_path, _report_hash(provenance))
        verdict = {
            "status": "BLOCKED",
            "unmodified_upstream": False,
            "cycle_complete": False,
            "all_required_motion_classes_present": False,
            "three_phase_configured_slot_cycle_complete": False,
            "both_shaft_wraps_exactly_two_turns": False,
            "observed_shaft_wrap_turns": [],
        }
        return capture_path, audit_path, provenance_path, verdict

    capture_path = bundle / "capture.jsonl"
    capture_command = (
        sys.executable, str(HERE / "capture.py"),
        "--settings", str(settings_path),
        "--winder", str(winder),
        "--controller", "upstream",
        "--output", str(capture_path),
    )
    capture_run = _run_command(capture_command, cwd=HERE)
    provenance["capture_command"] = list(capture_command)
    provenance["capture_returncode"] = capture_run.returncode
    provenance["capture_output"] = _stable_subprocess_output(
        capture_run.stdout
    )
    if capture_run.returncode != 0 or not capture_path.is_file():
        failed_path = bundle / "capture_unavailable.json"
        failed = _report_hash({
            "schema": CAPTURE_UNAVAILABLE_SCHEMA,
            "status": "BLOCKED",
            "job": plan.job,
            "reason": "untouched-upstream capture command failed",
            "returncode": capture_run.returncode,
            "output": _stable_subprocess_output(capture_run.stdout),
            "cycle_complete": False,
            "unmodified_upstream": False,
            "both_shaft_wraps_exactly_two_turns": False,
        })
        _write_json(failed_path, failed)
        _write_json(audit_path, failed)
        provenance["status"] = "BLOCKED"
        _write_json(provenance_path, _report_hash(provenance))
        verdict = {
            "status": "BLOCKED",
            "unmodified_upstream": False,
            "cycle_complete": False,
            "all_required_motion_classes_present": False,
            "three_phase_configured_slot_cycle_complete": False,
            "both_shaft_wraps_exactly_two_turns": False,
            "observed_shaft_wrap_turns": [],
        }
        return failed_path, audit_path, provenance_path, verdict

    # ``verify_cycle`` intentionally binds to ``OUT/settings.yml``.  Point
    # that module-level root at this exact bundle in a fresh interpreter;
    # this runs the unchanged verifier logic without overwriting the canonical
    # OD46 settings artifact or weakening its hash check.
    verify_driver = (
        "import sys; from pathlib import Path; import verify_cycle as v; "
        "v.OUT=Path(sys.argv[1]).resolve(); "
        "sys.argv=['verify_cycle.py','--capture',sys.argv[2],"
        "'--report',sys.argv[3],'--expect-controller','upstream']; "
        "raise SystemExit(v.main())"
    )
    verify_command = (
        sys.executable, "-c", verify_driver,
        str(bundle), str(capture_path), str(audit_path),
    )
    verify_run = _run_command(verify_command, cwd=HERE)
    provenance["verify_command"] = list(verify_command)
    provenance["verify_returncode"] = verify_run.returncode
    provenance["verify_output"] = _stable_subprocess_output(
        verify_run.stdout
    )
    if not audit_path.is_file():
        raise RuntimeError(
            f"cycle verifier did not write {audit_path}: {verify_run.stdout}"
        )
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    checks = audit.get("checks", {})
    check = lambda name: bool(checks.get(name, {}).get("ok"))
    wrap_turns = [
        float(row["turns"]) for row in audit.get("shaft_wraps", [])
        if isinstance(row.get("turns"), (int, float))
    ]
    winder_clean = provenance["winder"].get("dirty") is False
    unmodified = bool(
        winder_clean
        and check("capture is untouched upstream rather than adapter output")
        and check("controller mode")
    )
    cycle_complete = check("cycle completion marker")
    motion_complete = bool(
        check("configured tooth passes")
        and check("three phases")
        and check("all teeth visited once")
        and check("axis M0 commanded")
        and check("axis M1 commanded")
        and check("axis M2 commanded")
        and check("both raw shaft-wrap target intervals physically complete")
    )
    exact_wraps = check(
        "both raw shaft-wrap intervals execute exactly two M1 turns"
    )
    verdict = {
        "status": (
            "PASS" if unmodified and cycle_complete and motion_complete
            and exact_wraps else "FAIL"
        ),
        "unmodified_upstream": unmodified,
        "cycle_complete": cycle_complete,
        "all_required_motion_classes_present": motion_complete,
        "three_phase_configured_slot_cycle_complete": bool(
            cycle_complete and check("configured tooth passes")
            and check("three phases") and check("all teeth visited once")
        ),
        "configured_slots": plan.case.slots,
        "both_shaft_wraps_exactly_two_turns": exact_wraps,
        "observed_shaft_wrap_turns": wrap_turns,
        "independent_cycle_audit_status": audit.get("status"),
        "capture_sha256": _sha256(capture_path),
        "capture_audit_sha256": _sha256(audit_path),
    }
    provenance["status"] = "PASS" if unmodified else "FAIL"
    provenance["capture_sha256"] = _sha256(capture_path)
    provenance["audit_sha256"] = _sha256(audit_path)
    _write_json(provenance_path, _report_hash(provenance))
    return capture_path, audit_path, provenance_path, verdict


def _collision_report(
    plan: CasePlan,
    settings_cfg: Mapping[str, Any] | None,
    capture_verdict: Mapping[str, Any],
    step_facts: Mapping[str, Any],
) -> dict[str, Any]:
    preflight_pass = bool(
        plan.feasible and settings_cfg is not None
        and step_facts.get("valid_closed_geometry") is True
    )
    return _report_hash({
        "schema": COLLISION_SCHEMA,
        "status": "BLOCKED" if preflight_pass else "FAIL",
        "job": plan.job,
        "job_identity_bound": True,
        "parameter_envelope_pass": preflight_pass,
        "raw_capture_cycle_complete": capture_verdict.get("cycle_complete") is True,
        "collision_count": None,
        "minimum_dynamic_clearance_mm": None,
        "required_dynamic_clearance_mm": authority.MINIMUM_DYNAMIC_CLEARANCE_MM,
        "full_raw_motion_covered": False,
        "blocker": (
            "bundle-local exact link meshes and full raw FCL sweep are not yet generated"
        ),
        "limits": [
            "Settings reach and valid BREP are preflight only, not a collision certificate.",
            "No OD46 clearance result is reused for a launch corner.",
        ],
    })


def _wire_report(
    plan: CasePlan,
    capture_verdict: Mapping[str, Any],
) -> dict[str, Any]:
    fixed_minimum = min(
        float(wire_geometry.ENTRY_BEND_RADIUS),
        float(wire_geometry.FLYER_ELBOW_RADIUS),
        float(wire_geometry.tip_guide_spec()[
            "minimum_wire_center_bend_radius_mm"
        ]),
    )
    deposited = None
    if plan.feasible:
        deposited = wire_geometry.tooth_contact_spec(
            plan.spec, plan.selected_analysis,
        )["minimum_max_wire_deposited_curvature_radius_mm"]
    return _report_hash({
        "schema": WIRE_SCHEMA,
        "status": "BLOCKED" if plan.feasible else "FAIL",
        "job": plan.job,
        "job_identity_bound": True,
        "fixed_machine_guide_minimum_bend_radius_mm": fixed_minimum,
        "fixed_machine_guide_bend_radius_pass": (
            fixed_minimum >= authority.MINIMUM_WIRE_BEND_RADIUS_MM
        ),
        "reported_workpiece_deposited_curvature_mm": deposited,
        "workpiece_deposited_curvature_is_machine_guide_gate": False,
        "minimum_bend_radius_mm": None,
        "unintended_contact_count": None,
        "all_deposition_loci_and_intervals_proven": False,
        "continuous_conductor_proven": False,
        "observed_raw_shaft_wrap_turns": list(
            capture_verdict.get("observed_shaft_wrap_turns", [])
        ),
        "both_shaft_wraps_exactly_two_turns": (
            capture_verdict.get("both_shaft_wraps_exactly_two_turns") is True
        ),
        "blocker": (
            "case-specific flexible-conductor/contact authority and exact deposition loci are absent"
        ),
    })


def _part_mass_and_spindle_inertia(parts: Sequence[Any]) -> tuple[float, float]:
    density_g_mm3 = loads.G_CM3["steel"] / 1000.0
    mass_g = 0.0
    inertia_kg_m2 = 0.0
    for part in parts:
        volume = float(part.volume)
        center = part.center()
        geometric_iyy = float(part.matrix_of_inertia[1][1])
        geometric_about_axis = geometric_iyy + volume * (
            float(center.X) ** 2
            + (float(center.Z) - P.m0_home_standoff) ** 2
        )
        mass_g += volume * density_g_mm3
        inertia_kg_m2 += geometric_about_axis * density_g_mm3 * 1.0e-9
    return mass_g, inertia_kg_m2


_CARRIAGE_MASS_G: float | None = None
_M2_COMMON: dict[str, float] | None = None


def _common_load_context() -> tuple[float, dict[str, float]]:
    global _CARRIAGE_MASS_G, _M2_COMMON
    if _CARRIAGE_MASS_G is None:
        _CARRIAGE_MASS_G = sum(
            part.volume * (
                loads.G_CM3["petg"]
                if part.label in ("carriage_plate", "spindle_tower", "nut_bracket")
                else loads.G_CM3["steel"]
            ) / 1000.0
            for part in assembly.carriage_link()
        ) + 350.0
    if _M2_COMMON is None:
        flyer = loads.current_flyer_mass_model()
        izz = float(flyer["total"]["izz_about_M2_axis_g_mm2"]) * 1.0e-9
        required = izz * 200.0 + 2.0 * (
            1.5 * 10.0 * 0.060 / (2.0 * math.pi)
        )
        motor = "NEMA17 McMaster 6627T421 encoder motor @24V (M2)"
        _M2_COMMON = {
            "required_torque_nm": required,
            "motor_margin": loads.torque_at_rpm(motor, 191.0) / required,
            "pulley_margin": P.m2_motor_pulley_capacity_nm / required,
        }
    return _CARRIAGE_MASS_G, dict(_M2_COMMON)


def _load_report(
    plan: CasePlan,
    spindle_parts: Sequence[Any],
    capture_verdict: Mapping[str, Any],
) -> dict[str, Any]:
    if not plan.feasible:
        return _report_hash({
            "schema": LOAD_SCHEMA,
            "status": "FAIL",
            "job": plan.job,
            "job_identity_bound": True,
            "minimum_axis_margin_multiple": None,
            "M0_M1_M2_all_pass": False,
            "reason": "no feasible winding job exists for load evaluation",
            "physical_coupon_complete": False,
        })
    carriage_mass_g, m2 = _common_load_context()
    spindle_mass_g, spindle_inertia = _part_mass_and_spindle_inertia(spindle_parts)
    moving_mass_g = carriage_mass_g + spindle_mass_g
    axial_force = 10.0 + 0.02 * moving_mass_g * 9.81e-3
    m0_torque = axial_force * (P.m0_lead / 1000.0) / (2.0 * math.pi * 0.5)
    motor17 = "17HS19-2004D-E1K + CL42T-V41 @24V (M0/M1)"
    rpm_m0 = P.m0_velocity_max_rad * 60.0 / (2.0 * math.pi)
    m0_margin = loads.torque_at_rpm(motor17, rpm_m0) / m0_torque
    sleeve_radius_m = (
        wire_geometry.shaft_wrap_sleeve_spec(plan.spec)["outer_diameter_mm"]
        / 2000.0
    )
    m1_torque = 10.0 * sleeve_radius_m + spindle_inertia * 50.0 + 0.02
    rpm_m1 = P.m1_velocity_max_rad * 60.0 / (2.0 * math.pi)
    m1_margin = loads.torque_at_rpm(motor17, rpm_m1) / m1_torque
    coupling_m0 = P.coupling_5x8_dynamic_reversing_nm / m0_torque
    coupling_m1 = P.coupling_5x8_dynamic_reversing_nm / m1_torque
    margins = {
        "M0": m0_margin,
        "M1": m1_margin,
        "M2": m2["motor_margin"],
        "M2_pulley": m2["pulley_margin"],
        "M0_coupling": coupling_m0,
        "M1_coupling": coupling_m1,
    }
    minimum = min(margins.values())
    passed = bool(
        minimum >= authority.MINIMUM_AXIS_MARGIN_MULTIPLE
        and capture_verdict.get("cycle_complete") is True
    )
    return _report_hash({
        "schema": LOAD_SCHEMA,
        "status": "PASS" if passed else "BLOCKED",
        "job": plan.job,
        "job_identity_bound": True,
        "mass_model": (
            "all spindle-link solids conservatively assigned steel density; "
            "exact current flyer mass/inertia model"
        ),
        "moving_mass_g": moving_mass_g,
        "spindle_inertia_about_M1_kg_m2": spindle_inertia,
        "required_torque_nm": {
            "M0": m0_torque,
            "M1": m1_torque,
            "M2": m2["required_torque_nm"],
        },
        "axis_margin_multiple": margins,
        "minimum_axis_margin_multiple": minimum,
        "required_minimum_margin_multiple": authority.MINIMUM_AXIS_MARGIN_MULTIPLE,
        "M0_M1_M2_all_pass": passed,
        "raw_motion_cycle_bound": capture_verdict.get("cycle_complete") is True,
        "physical_coupon_complete": False,
        "limits": [
            "Analytical motor margin is not a received-hardware torque or endurance coupon.",
            "Shaft-wrap turn-count failure remains a capture gate, not hidden inside load margin.",
        ],
    })


def _dependency(root: Path, relative: str) -> dict[str, Any]:
    path = root / relative
    return {
        "path": relative,
        "exists": path.is_file(),
        "sha256": _sha256(path) if path.is_file() else None,
    }


def _buildability_report(
    root: Path,
    plan: CasePlan,
    step_facts: Mapping[str, Any],
) -> dict[str, Any]:
    fixed_path = root / FIXED_BUILDABILITY
    fixed = (
        json.loads(fixed_path.read_text(encoding="utf-8"))
        if fixed_path.is_file() else {}
    )
    fixed_pass = bool(
        fixed.get("single_solid_check") == "pass"
        and fixed.get("mesh_check") == "pass"
        and not fixed.get("pending_parts")
        and all(row.get("bed_fit") is True for row in fixed.get("parts", []))
        and all(row.get("mesh", {}).get("ok") is True
                for row in fixed.get("parts", []))
    )
    dependency_paths = (
        (ER11_STEP,) if plan.case.spindle_id == "er11"
        else (SHAFT8_STEP, SHAFT8_DRAWING)
    )
    dependencies = [_dependency(root, relative) for relative in dependency_paths]
    workholding_pass = all(row["exists"] for row in dependencies)
    passed = bool(
        fixed_pass
        and workholding_pass
        and step_facts.get("valid_closed_geometry") is True
    )
    return _report_hash({
        "schema": BUILDABILITY_SCHEMA,
        "status": "PASS" if passed else "FAIL",
        "job": plan.job,
        "job_identity_bound": True,
        "all_parts_buildable": passed,
        "all_parts_fit_220x220x250_mm": fixed_pass,
        "fixed_machine_buildability": {
            "path": FIXED_BUILDABILITY,
            "sha256": _sha256(fixed_path) if fixed_path.is_file() else None,
            "pass": fixed_pass,
        },
        "selected_workholding_dependencies": dependencies,
        "exact_job_step_valid": step_facts.get("valid_closed_geometry") is True,
        "physical_receiving_and_coupon_gates_complete": False,
    })


def _source_hashes(root: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for relative in authority.REQUIRED_SOURCE_PATHS:
        path = root / relative
        if not path.is_file():
            raise FileNotFoundError(f"required launch source is missing: {relative}")
        result[relative] = _sha256(path)
    return result


def generate_case(
    plan: CasePlan,
    *,
    root: Path = ROOT,
    certificate_root: Path = DEFAULT_CERTIFICATE_ROOT,
    run_capture: bool = True,
) -> dict[str, Any]:
    root = Path(root).resolve()
    bundle = Path(certificate_root).resolve() / plan.case.case_id
    bundle.mkdir(parents=True, exist_ok=True)

    packing_path = bundle / "packing.json"
    packing = _packing_report(plan)
    _write_json(packing_path, packing)

    settings_path, settings_cfg, settings_verdict = _settings_evidence(plan, bundle)

    step_path = bundle / "job_geometry.step"
    step_facts, spindle_parts = _generate_job_step(plan, step_path)
    step_verdict = {
        "status": "PASS" if plan.feasible and step_facts[
            "valid_closed_geometry"] else "FAIL",
        "exact_job_geometry": bool(
            plan.feasible and step_facts["valid_closed_geometry"]
        ),
        "valid_closed_geometry": step_facts["valid_closed_geometry"],
        "geometry_facts": step_facts,
    }

    capture_path, capture_audit_path, provenance_path, capture_verdict = (
        _capture_evidence(
            plan, bundle, settings_path, run_capture=run_capture,
        )
    )

    collision_path = bundle / "collision.json"
    collision = _collision_report(
        plan, settings_cfg, capture_verdict, step_facts,
    )
    _write_json(collision_path, collision)
    collision_verdict = {
        "status": collision["status"],
        "job_identity_bound": True,
        "collision_count": collision["collision_count"],
        "minimum_dynamic_clearance_mm": collision[
            "minimum_dynamic_clearance_mm"
        ],
        "full_raw_motion_covered": False,
    }

    wire_path = bundle / "wire.json"
    wire = _wire_report(plan, capture_verdict)
    _write_json(wire_path, wire)
    wire_verdict = {
        "status": wire["status"],
        "job_identity_bound": True,
        "minimum_bend_radius_mm": wire["minimum_bend_radius_mm"],
        "unintended_contact_count": wire["unintended_contact_count"],
        "all_deposition_loci_and_intervals_proven": False,
        "continuous_conductor_proven": False,
        "fixed_machine_guide_minimum_bend_radius_mm": wire[
            "fixed_machine_guide_minimum_bend_radius_mm"
        ],
    }

    load_path = bundle / "load.json"
    load = _load_report(plan, spindle_parts, capture_verdict)
    _write_json(load_path, load)
    load_verdict = {
        "status": load["status"],
        "job_identity_bound": True,
        "minimum_axis_margin_multiple": load.get(
            "minimum_axis_margin_multiple"
        ),
        "M0_M1_M2_all_pass": load.get("M0_M1_M2_all_pass", False),
        "physical_coupon_complete": False,
    }

    buildability_path = bundle / "buildability.json"
    buildability = _buildability_report(root, plan, step_facts)
    _write_json(buildability_path, buildability)
    buildability_verdict = {
        "status": buildability["status"],
        "job_identity_bound": True,
        "all_parts_buildable": buildability["all_parts_buildable"],
        "all_parts_fit_220x220x250_mm": buildability[
            "all_parts_fit_220x220x250_mm"
        ],
        "physical_receiving_and_coupon_gates_complete": False,
    }

    packing_verdict = {
        "status": packing["status"],
        "job_identity_bound": True,
        "capacity_and_opening_pass": packing["capacity_and_opening_pass"],
    }
    if isinstance(settings_verdict, Mapping) and "job_identity_bound" not in settings_verdict:
        settings_verdict = {
            **settings_verdict,
            "job_identity_bound": True,
        }

    artifacts = {
        "capture": _artifact_row(root, capture_path, plan.job),
        "step": _artifact_row(root, step_path, plan.job),
        "settings": _artifact_row(root, settings_path, plan.job),
        "packing": _artifact_row(root, packing_path, plan.job),
        "collision": _artifact_row(root, collision_path, plan.job),
        "wire": _artifact_row(root, wire_path, plan.job),
        "load": _artifact_row(root, load_path, plan.job),
        "buildability": _artifact_row(root, buildability_path, plan.job),
        "capture_audit": _artifact_row(root, capture_audit_path, plan.job),
        "capture_provenance": _artifact_row(root, provenance_path, plan.job),
    }
    verdicts = {
        "capture": capture_verdict,
        "step": step_verdict,
        "settings": dict(settings_verdict),
        "packing": packing_verdict,
        "collision": collision_verdict,
        "wire": wire_verdict,
        "load": load_verdict,
        "buildability": buildability_verdict,
    }
    blockers = sorted(
        key for key in authority.REQUIRED_ARTIFACT_KEYS
        if verdicts[key].get("status") != "PASS"
    )
    certificate = {
        "schema": authority.CORNER_EVIDENCE_SCHEMA,
        "case_id": plan.case.case_id,
        "status": "FAIL_CLOSED",
        "corner_authorized": False,
        "production_authorized": False,
        "source_dependency_closure_complete": True,
        "job": plan.job,
        "representative_plan": {
            "selection": "maximum integer turns at analytical design fill",
            "feasible": plan.feasible,
            "rejection_reasons": list(plan.rejection_reasons),
        },
        "sources": _source_hashes(root),
        "artifacts": artifacts,
        "verdicts": verdicts,
        "blocking_gates": blockers,
        "limits": [
            "This bundle is current partial evidence and cannot authorize its corner.",
            "Raw upstream wrap turns are reported exactly; they are never normalized to two.",
            "Flexible-conductor dynamics, sag, snagging, and physical coupons are not simulated authority.",
        ],
    }
    certificate["certificate_payload_sha256"] = _canonical_hash(certificate)
    certificate_path = bundle / "certificate.json"
    _write_json(certificate_path, certificate)
    return {
        "case_id": plan.case.case_id,
        "job": plan.job,
        "representative_plan_feasible": plan.feasible,
        "certificate_path": _relative(root, certificate_path),
        "certificate_sha256": _sha256(certificate_path),
        "blocking_gates": blockers,
        "verdict_statuses": {
            key: verdict["status"] for key, verdict in verdicts.items()
        },
        "observed_shaft_wrap_turns": capture_verdict.get(
            "observed_shaft_wrap_turns", []
        ),
    }


def _generation_report(
    rows: Sequence[Mapping[str, Any]],
    authority_report: Mapping[str, Any],
) -> dict[str, Any]:
    gate_counts: dict[str, dict[str, int]] = {}
    for key in authority.REQUIRED_ARTIFACT_KEYS:
        counts: dict[str, int] = {}
        for row in rows:
            status = str(row["verdict_statuses"][key])
            counts[status] = counts.get(status, 0) + 1
        gate_counts[key] = counts
    report = {
        "schema": SCHEMA,
        "status": "FAIL_CLOSED",
        "production_authorized": False,
        "case_count": len(rows),
        "representative_plan_feasible_count": sum(
            row["representative_plan_feasible"] is True for row in rows
        ),
        "representative_plan_infeasible_count": sum(
            row["representative_plan_feasible"] is False for row in rows
        ),
        "authority_summary": authority_report["summary"],
        "evidence_progress": authority_report["evidence_progress"],
        "gate_counts": gate_counts,
        "OD28_topology_correction": _superseded_od28_24n22p_throat_proof(),
        "cases": list(rows),
        "limits": [
            "Zero PASS corner certificates is expected until every required gate is PASS.",
            "OD28 rows use the upstream-supported 12n14p pattern and an explicit reach-bounded representative hub; OD65 rows use 24n22p.",
            "These rows prove representative feasible laminations, not universal internal stator geometry at a given OD.",
            "Every real job still requires its measured lamination section to pass packing, slot access, and workholder reach regeneration.",
            "Current untouched-upstream captures remain negative regression evidence while shaft wraps are not exactly two physical turns.",
        ],
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: Mapping[str, Any]) -> str:
    progress = report["evidence_progress"]
    lines = [
        "# Launch-envelope evidence generation",
        "",
        "Status: **FAIL_CLOSED**",
        "",
        f"Generated cases: {report['case_count']}.",
        "",
        (
            "Representative winding plans: "
            f"{report['representative_plan_feasible_count']} feasible; "
            f"{report['representative_plan_infeasible_count']} infeasible."
        ),
        "",
        (
            "Authority: "
            f"{progress['current_exact_authority_certificates']} PASS corner "
            "certificates; "
            f"{progress['current_fail_closed_evidence_bundles']} current "
            "fail-closed evidence bundles."
        ),
        "",
        "| gate | PASS | FAIL | BLOCKED |",
        "|---|---:|---:|---:|",
    ]
    for gate, counts in report["gate_counts"].items():
        lines.append(
            f"| `{gate}` | {counts.get('PASS', 0)} | "
            f"{counts.get('FAIL', 0)} | {counts.get('BLOCKED', 0)} |"
        )
    lines.extend(["", "## Case results", ""])
    for row in report["cases"]:
        turns = row["job"]["turns_per_tooth"]
        wraps = row["observed_shaft_wrap_turns"]
        lines.append(
            f"- `{row['case_id']}`: turns {turns}; feasible "
            f"{str(row['representative_plan_feasible']).lower()}; "
            f"blockers {', '.join(row['blocking_gates'])}; wraps {wraps}."
        )
    proof = report["OD28_topology_correction"]
    lines.extend([
        "",
        "## OD28 topology correction",
        "",
        (
            "The superseded OD28/24n22p/5-mil-liner throat admits at most "
            f"{proof['maximum_compatible_finished_wire_mm']:.6f} mm "
            "finished wire, so 0.500000 mm is short by "
            f"{proof['finished_wire_diameter_deficit_mm']:.6f} mm."
        ),
        "",
        (
            "OD28 launch rows now bind upstream `dev-12n14p-settings.yml`, "
            "their exact hub geometry, and 0.25 mm workholder-reach reserve."
        ),
        "",
        (
            "This is representative-lamination evidence only; every real "
            "lamination section retains its own packing and reach gate."
        ),
    ])
    lines.extend(["", "## Limits", ""])
    lines.extend(f"- {item}" for item in report["limits"])
    lines.extend(["", f"Report SHA-256: `{report['report_sha256']}`", ""])
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate fail-closed exact launch-corner evidence bundles",
    )
    parser.add_argument("--machine-root", type=Path, default=ROOT)
    parser.add_argument("--certificate-root", type=Path)
    parser.add_argument("--case-id", action="append", default=[])
    parser.add_argument(
        "--no-capture", action="store_true",
        help="write explicit capture-unavailable evidence instead of running upstream",
    )
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--md-out", type=Path)
    args = parser.parse_args(argv)

    root = args.machine_root.resolve()
    if root != ROOT.resolve():
        parser.error(
            "generation currently requires the active machine source root; "
            "use authority.analyze(root=...) for isolated audit fixtures"
        )
    certificate_root = (
        args.certificate_root.resolve()
        if args.certificate_root else DEFAULT_CERTIFICATE_ROOT.resolve()
    )
    try:
        certificate_root.relative_to(root)
    except ValueError:
        parser.error("certificate root must be inside machine root")

    plans = derive_all_case_plans()
    if args.case_id:
        requested = set(args.case_id)
        known = {plan.case.case_id for plan in plans}
        unknown = requested - known
        if unknown:
            parser.error("unknown case id(s): " + ", ".join(sorted(unknown)))
        plans = tuple(plan for plan in plans if plan.case.case_id in requested)

    rows = []
    for index, plan in enumerate(plans, 1):
        print(
            f"[{index}/{len(plans)}] {plan.case.case_id}: "
            f"turns={plan.turns_per_tooth} feasible={plan.feasible}",
            flush=True,
        )
        rows.append(generate_case(
            plan, root=root, certificate_root=certificate_root,
            run_capture=not args.no_capture,
        ))

    authority_report = authority.analyze(
        root=root, certificate_root=certificate_root,
    )
    authority.write_reports(authority_report)
    report = _generation_report(rows, authority_report)
    json_out = (args.json_out.resolve() if args.json_out else DEFAULT_JSON)
    md_out = (args.md_out.resolve() if args.md_out else DEFAULT_MD)
    _write_json(json_out, report)
    md_out.parent.mkdir(parents=True, exist_ok=True)
    md_out.write_text(render_markdown(report), encoding="utf-8", newline="\n")
    print(
        "launch evidence: "
        f"authority PASS={authority_report['summary']['passing']}; "
        f"current blocked={authority_report['evidence_progress']['current_fail_closed_evidence_bundles']}; "
        f"missing={authority_report['summary']['missing']}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
