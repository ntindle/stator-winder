"""Fail-closed physical placement and packed-route tolerance budget.

The packing and route solvers operate on exact measured-input geometry.  This
gate asks the separate release question: is the smallest *non-contact* route
margin still positive after controller acceptance, encoder quantization and
measured hardware uncertainty are applied?

Intentional wire-parent and wire-liner support contacts are deliberately not
treated as clearance.  They have zero nominal gap by construction and remain
coupon/contact-mechanics evidence, not a source of fictitious route margin.

The default invocation is diagnostic and therefore leaves all receiving and
hardware evidence unknown.  A PASS is possible only with measured-input
packing, a PASS hash-bound route report containing parent/contact-excluded
core and copper margins, and provenance-bearing physical evidence for every
uncertainty term.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

try:  # Package import in tests, direct import when run from sim/.
    from .controller_adapter import PACKING_M0_SETTLE_TOLERANCE_RAD
except ImportError:  # pragma: no cover - direct sim-path import
    from controller_adapter import PACKING_M0_SETTLE_TOLERANCE_RAD


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"

import sys

if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import PARAMS  # noqa: E402


SCHEMA = "placement-route-tolerance/v1"
PACKING_PATH = REPORTS / "slot_packing.json"
PLAN_PATH = REPORTS / "slot_winding_plan.json"
ROUTES_PATH = REPORTS / "slot_wire_routes.json"
OUTPUT_PATH = REPORTS / "placement_tolerance.json"
MARKDOWN_PATH = REPORTS / "placement_tolerance.md"

# The selected M0 kit is specified as a 1000 PPR differential quadrature
# encoder in bom.csv.  Four decoded edges per pulse yield 4000 counts/rev.
DEFAULT_ENCODER_PPR = 1000
DEFAULT_QUADRATURE_MULTIPLIER = 4
ENCODER_SOURCE = (
    "bom.csv: StepperOnline 17HS19-2004D-E1K M0 kit, 1000 PPR encoder"
)


@dataclass(frozen=True)
class PhysicalEvidence:
    """Worst-case physical evidence values and their traceable provenance.

    TIR values are charged at their full reported value.  This is
    intentionally conservative and avoids assuming that a measured indicator
    swing is perfectly centered.  Instrument uncertainty values are the
    absolute +/- uncertainty of the reported diameter/thickness measurement.
    """

    m0_observed_max_error_rad: float | None = None
    m0_observation_source: str | None = None
    m0_carriage_physical_error_mm: float | None = None
    m0_carriage_source: str | None = None
    spindle_tir_mm: float | None = None
    spindle_tir_source: str | None = None
    mounted_stator_tir_mm: float | None = None
    mounted_stator_tir_source: str | None = None
    wire_diameter_instrument_uncertainty_mm: float | None = None
    wire_measurement_source: str | None = None
    liner_thickness_instrument_uncertainty_mm: float | None = None
    liner_measurement_source: str | None = None
    contact_position_uncertainty_mm: float | None = None
    contact_uncertainty_source: str | None = None


@dataclass(frozen=True)
class RouteMargins:
    """Nominal contact-excluded surface-clearance margins in millimetres."""

    copper_nonparent_margin_mm: float | None
    core_noncontact_margin_mm: float | None
    copper_source: str | None
    core_source: str | None


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _finite_nonnegative(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) and number >= 0.0 else None


def _finite_number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _proven(value: Any, source: Any) -> bool:
    return (
        _finite_nonnegative(value) is not None
        and isinstance(source, str)
        and bool(source.strip())
    )


def _verify_embedded_hash(report: Mapping[str, Any], field: str) -> str | None:
    expected = report.get(field)
    if not isinstance(expected, str) or len(expected) != 64:
        return f"missing or malformed {field}"
    payload = dict(report)
    payload.pop(field, None)
    if _canonical_hash(payload) != expected:
        return f"{field} mismatch"
    return None


def _source_path(name: str) -> Path | None:
    relative = Path(name)
    candidates = (
        ROOT / relative,
        CAD / relative,
        HERE / relative,
    )
    return next((path for path in candidates if path.is_file()), None)


def _source_hash_mismatches(report: Mapping[str, Any]) -> list[str]:
    hashes = report.get("source_hashes")
    if not isinstance(hashes, Mapping):
        return ["input report has no source_hashes mapping"]
    mismatches: list[str] = []
    for name, expected in hashes.items():
        path = _source_path(str(name))
        if path is None:
            mismatches.append(f"source missing: {name}")
        elif not isinstance(expected, str) or _sha256(path) != expected:
            mismatches.append(f"source hash stale: {name}")
    return mismatches


def _nested_number(mapping: Mapping[str, Any], *keys: str) -> float | None:
    current: Any = mapping
    for key in keys:
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    return _finite_nonnegative(current)


def _first_margin(
    mapping: Mapping[str, Any],
    keys: tuple[str, ...],
) -> tuple[float | None, str | None]:
    for key in keys:
        value = _finite_number(mapping.get(key))
        if value is not None:
            return value, f"validation.{key}"
    return None, None


def extract_noncontact_route_margins(route_report: Mapping[str, Any]
                                     ) -> RouteMargins:
    """Read only explicitly contact-excluded route margins.

    Older route tables expose ``nonparent_raw_chordal_distance_mm`` per
    crossing rather than a top-level margin.  That field is explicitly
    parent-excluded, so subtracting the report's required copper center
    distance is valid.  Generic ``minimum_*_margin`` fields are intentionally
    ignored because they include the target's designed parent/liner contact.
    """

    validation = route_report.get("validation", {})
    if not isinstance(validation, Mapping):
        validation = {}

    copper, copper_source = _first_margin(validation, (
        "minimum_nonparent_copper_margin_mm",
        "minimum_nonparent_clearance_margin_mm",
        "minimum_nonparent_clearance_mm",
        "minimum_noncontact_copper_margin_mm",
    ))
    core, core_source = _first_margin(validation, (
        "minimum_noncontact_core_margin_mm",
        "minimum_nonparent_core_margin_mm",
        "minimum_core_noncontact_margin_mm",
    ))

    routes = route_report.get("routes")
    if not isinstance(routes, list):
        routes = []
    required_copper = _nested_number(
        route_report, "input_contract", "required_copper_center_distance_mm")

    per_route_copper: list[tuple[float, str]] = []
    per_route_core: list[tuple[float, str]] = []
    for index, row in enumerate(routes):
        if not isinstance(row, Mapping):
            continue
        metadata = row.get("planner_metadata", {})
        post = (
            metadata.get("exact_release_postcheck", {})
            if isinstance(metadata, Mapping) else {}
        )
        if not isinstance(post, Mapping):
            continue
        for key in (
            "nonparent_copper_margin_mm",
            "minimum_nonparent_copper_margin_mm",
            "nonparent_clearance_mm",
        ):
            value = _finite_number(post.get(key))
            if value is not None:
                per_route_copper.append((
                    value,
                    f"routes[{index}].planner_metadata."
                    f"exact_release_postcheck.{key}",
                ))
        raw = _finite_nonnegative(
            post.get("nonparent_raw_chordal_distance_mm"))
        if raw is not None and required_copper is not None:
            margin = raw - required_copper
            per_route_copper.append((
                margin,
                f"routes[{index}].planner_metadata."
                "exact_release_postcheck."
                "nonparent_raw_chordal_distance_mm minus "
                "input_contract.required_copper_center_distance_mm",
            ))
        for key in (
            "noncontact_core_margin_mm",
            "minimum_noncontact_core_margin_mm",
            "nonparent_core_margin_mm",
        ):
            value = _finite_number(post.get(key))
            if value is not None:
                per_route_core.append((
                    value,
                    f"routes[{index}].planner_metadata."
                    f"exact_release_postcheck.{key}",
                ))

    if copper is None and per_route_copper:
        copper, copper_source = min(per_route_copper, key=lambda item: item[0])
    if core is None and per_route_core:
        core, core_source = min(per_route_core, key=lambda item: item[0])
    return RouteMargins(copper, core, copper_source, core_source)


def evaluate_budget(
    margins: RouteMargins,
    evidence: PhysicalEvidence,
    *,
    packing_is_measured: bool,
    upstream_statuses: Mapping[str, str],
    m0_settle_tolerance_rad: float = PACKING_M0_SETTLE_TOLERANCE_RAD,
    m0_mm_per_rad: float = PARAMS.mm_per_rad,
    encoder_ppr: int = DEFAULT_ENCODER_PPR,
    quadrature_multiplier: int = DEFAULT_QUADRATURE_MULTIPLIER,
) -> dict[str, Any]:
    """Evaluate one physical tolerance budget without reading the filesystem."""

    hard_failures: list[str] = []
    unknowns: list[str] = []
    if (not math.isfinite(m0_settle_tolerance_rad)
            or m0_settle_tolerance_rad < 0.0):
        hard_failures.append("controller M0 settle tolerance is invalid")
    if not math.isfinite(m0_mm_per_rad) or m0_mm_per_rad <= 0.0:
        hard_failures.append("T8 M0 mm/rad conversion is invalid")
    if (isinstance(encoder_ppr, bool) or encoder_ppr <= 0
            or isinstance(quadrature_multiplier, bool)
            or quadrature_multiplier <= 0):
        hard_failures.append("encoder resolution is invalid")

    counts_per_rev = encoder_ppr * quadrature_multiplier
    radians_per_count = 2.0 * math.pi / max(counts_per_rev, 1)
    # The actual shaft may be anywhere within half of the reported count.
    encoder_quantization_mm = 0.5 * radians_per_count * m0_mm_per_rad
    controller_acceptance_mm = m0_settle_tolerance_rad * m0_mm_per_rad

    if not packing_is_measured:
        unknowns.append(
            "packing uses supplier nominal/default inputs rather than "
            "receipt measurements")

    upstream_required = ("packing", "plan", "routes")
    for name in upstream_required:
        status = upstream_statuses.get(name)
        if status == "FAIL":
            hard_failures.append(f"upstream {name} status is FAIL")
        elif status != "PASS":
            unknowns.append(f"upstream {name} status is not PASS")

    evidence_fields = (
        ("m0_observed_max_error_rad", "m0_observation_source"),
        ("m0_carriage_physical_error_mm", "m0_carriage_source"),
        ("spindle_tir_mm", "spindle_tir_source"),
        ("mounted_stator_tir_mm", "mounted_stator_tir_source"),
        ("wire_diameter_instrument_uncertainty_mm",
         "wire_measurement_source"),
        ("liner_thickness_instrument_uncertainty_mm",
         "liner_measurement_source"),
        ("contact_position_uncertainty_mm",
         "contact_uncertainty_source"),
    )
    evidence_state: dict[str, dict[str, Any]] = {}
    for value_field, source_field in evidence_fields:
        raw_value = getattr(evidence, value_field)
        source = getattr(evidence, source_field)
        valid_number = _finite_nonnegative(raw_value)
        proven = _proven(raw_value, source)
        if raw_value is not None and valid_number is None:
            hard_failures.append(f"{value_field} is invalid")
        if not proven:
            unknowns.append(
                f"{value_field} lacks a finite nonnegative measurement "
                "and provenance")
        evidence_state[value_field] = {
            "value": valid_number,
            "source": source,
            "status": "PROVEN" if proven else "UNKNOWN",
        }

    observed_rad = _finite_nonnegative(evidence.m0_observed_max_error_rad)
    observed_proven = _proven(
        evidence.m0_observed_max_error_rad,
        evidence.m0_observation_source)
    if observed_proven and observed_rad is not None:
        observed_mm = observed_rad * m0_mm_per_rad
        acceptance_headroom_mm = max(
            0.0, controller_acceptance_mm - observed_mm)
        if observed_rad > m0_settle_tolerance_rad + 1.0e-12:
            hard_failures.append(
                "observed M0 error exceeds controller settle tolerance")
    else:
        observed_mm = None
        acceptance_headroom_mm = None

    def proven_mm(value_field: str, source_field: str) -> float | None:
        value = getattr(evidence, value_field)
        source = getattr(evidence, source_field)
        return _finite_nonnegative(value) if _proven(value, source) else None

    carriage = proven_mm(
        "m0_carriage_physical_error_mm", "m0_carriage_source")
    spindle = proven_mm("spindle_tir_mm", "spindle_tir_source")
    stator = proven_mm(
        "mounted_stator_tir_mm", "mounted_stator_tir_source")
    wire_u = proven_mm(
        "wire_diameter_instrument_uncertainty_mm",
        "wire_measurement_source")
    liner_u = proven_mm(
        "liner_thickness_instrument_uncertainty_mm",
        "liner_measurement_source")
    contact_u = proven_mm(
        "contact_position_uncertainty_mm",
        "contact_uncertainty_source")

    common_known = controller_acceptance_mm + encoder_quantization_mm
    for value in (carriage, spindle, stator, contact_u):
        if value is not None:
            common_known += value
    copper_known_lower_bound = common_known + (wire_u or 0.0)
    core_known_lower_bound = (
        common_known + (wire_u or 0.0) / 2.0 + (liner_u or 0.0)
    )
    all_evidence_proven = all(
        row["status"] == "PROVEN" for row in evidence_state.values())
    copper_complete = all_evidence_proven
    core_complete = all_evidence_proven

    if margins.copper_nonparent_margin_mm is None:
        unknowns.append(
            "route report has no explicit parent-excluded copper margin")
    elif margins.copper_nonparent_margin_mm <= (
            copper_known_lower_bound + 1.0e-12):
        hard_failures.append(
            "nonparent copper margin does not exceed the known physical "
            "uncertainty lower bound")
    if margins.core_noncontact_margin_mm is None:
        unknowns.append(
            "route report has no explicit liner-contact-excluded core margin")
    elif margins.core_noncontact_margin_mm <= (
            core_known_lower_bound + 1.0e-12):
        hard_failures.append(
            "noncontact core margin does not exceed the known physical "
            "uncertainty lower bound")

    if hard_failures:
        status = "FAIL"
    elif unknowns:
        status = "NOT_PROVEN"
    else:
        status = "PASS"

    return {
        "status": status,
        "axis_contract": {
            "t8_lead_mm_per_rev": m0_mm_per_rad * 2.0 * math.pi,
            "m0_mm_per_rad": m0_mm_per_rad,
            "controller_m0_settle_tolerance_rad": m0_settle_tolerance_rad,
            "controller_acceptance_mm": controller_acceptance_mm,
            "encoder_ppr": encoder_ppr,
            "quadrature_multiplier": quadrature_multiplier,
            "encoder_counts_per_rev": counts_per_rev,
            "encoder_radians_per_count": radians_per_count,
            "encoder_half_count_quantization_mm": encoder_quantization_mm,
            "encoder_source": ENCODER_SOURCE,
        },
        "m0_observation": {
            "maximum_error_rad": observed_rad,
            "maximum_error_mm": observed_mm,
            "acceptance_headroom_mm": acceptance_headroom_mm,
            "note": (
                "observed error plus remaining controller headroom equals "
                "the full acceptance term; it is not double-counted"),
        },
        "physical_evidence": evidence_state,
        "contact_policy": {
            "intended_parent_wire_contacts": (
                "zero nominal gap; excluded from nonparent clearance"),
            "intended_liner_contacts": (
                "zero nominal gap; excluded from noncontact core clearance"),
            "contact_mechanics_release": (
                "requires measured contact uncertainty and winding coupon"),
        },
        "route_margins": {
            "copper_nonparent_margin_mm": (
                margins.copper_nonparent_margin_mm),
            "copper_source": margins.copper_source,
            "core_noncontact_margin_mm": margins.core_noncontact_margin_mm,
            "core_source": margins.core_source,
        },
        "budget": {
            "controller_acceptance_decomposition": {
                "observed_error_mm": observed_mm,
                "remaining_allowed_headroom_mm": acceptance_headroom_mm,
                "budgeted_total_mm": controller_acceptance_mm,
                "unknown_observation_fallback": (
                    "budget the complete controller acceptance envelope"),
            },
            "common_known_lower_bound_mm": common_known,
            "copper_known_lower_bound_mm": copper_known_lower_bound,
            "core_known_lower_bound_mm": core_known_lower_bound,
            "copper_complete": copper_complete,
            "core_complete": core_complete,
            "copper_residual_mm": (
                None if margins.copper_nonparent_margin_mm is None
                else margins.copper_nonparent_margin_mm
                - copper_known_lower_bound),
            "core_residual_mm": (
                None if margins.core_noncontact_margin_mm is None
                else margins.core_noncontact_margin_mm
                - core_known_lower_bound),
            "wire_uncertainty_rule": (
                "full diameter uncertainty against copper; half diameter "
                "uncertainty against core"),
            "liner_uncertainty_rule": (
                "full thickness uncertainty against core; none against "
                "copper-copper clearance"),
            "tir_rule": (
                "full reported spindle and mounted-stator TIR charged "
                "conservatively"),
        },
        "hard_failures": sorted(set(hard_failures)),
        "unknowns": sorted(set(unknowns)),
    }


def _artifact_record(path: Path, report: Mapping[str, Any], hash_field: str
                     ) -> dict[str, Any]:
    try:
        display_path = str(path.relative_to(ROOT)).replace("\\", "/")
    except ValueError:
        display_path = str(path)
    return {
        "path": display_path,
        "file_sha256": _sha256(path),
        hash_field: report.get(hash_field),
    }


def build_report(
    *,
    packing_path: Path = PACKING_PATH,
    plan_path: Path = PLAN_PATH,
    routes_path: Path = ROUTES_PATH,
    evidence: PhysicalEvidence = PhysicalEvidence(),
    encoder_ppr: int = DEFAULT_ENCODER_PPR,
    quadrature_multiplier: int = DEFAULT_QUADRATURE_MULTIPLIER,
) -> dict[str, Any]:
    """Load, integrity-check and budget the current release artifacts."""

    packing_path = packing_path.resolve()
    plan_path = plan_path.resolve()
    routes_path = routes_path.resolve()
    packing = json.loads(packing_path.read_text(encoding="utf-8"))
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    routes = json.loads(routes_path.read_text(encoding="utf-8"))

    integrity_failures: list[str] = []
    for name, report, field in (
        ("packing", packing, "report_sha256"),
        ("plan", plan, "proof_sha256"),
        ("routes", routes, "report_sha256"),
    ):
        problem = _verify_embedded_hash(report, field)
        if problem:
            integrity_failures.append(f"{name}: {problem}")
        integrity_failures.extend(
            f"{name}: {problem}"
            for problem in _source_hash_mismatches(report)
        )

    packing_hash = packing.get("report_sha256")
    if _nested_number(packing, "config", "m0_mm_per_rad") is None:
        integrity_failures.append("packing: missing M0 mm/rad conversion")
    if plan.get("packing_report", {}).get(
            "report_sha256") != packing_hash:
        integrity_failures.append("plan is stale for packing report")
    if routes.get("input_contract", {}).get(
            "packing_report_sha256") != packing_hash:
        integrity_failures.append("route report is stale for packing report")

    pack_wire = _nested_number(
        packing, "config", "wire_finished_diameter_mm")
    pack_liner = _nested_number(packing, "config", "liner_thickness_mm")
    plan_wire = _nested_number(plan, "job", "wire_finished_d_mm")
    plan_liner = _nested_number(plan, "job", "liner_max_thickness_mm")
    route_wire = _nested_number(
        routes, "input_contract", "wire_finished_diameter_mm")
    route_liner = _nested_number(
        routes, "input_contract", "liner_thickness_mm")
    if pack_wire is None or plan_wire is None or route_wire is None or not (
            math.isclose(pack_wire, plan_wire, abs_tol=1.0e-12)
            and math.isclose(pack_wire, route_wire, abs_tol=1.0e-12)):
        integrity_failures.append(
            "wire measurement differs across packing, plan and routes")
    if pack_liner is None or plan_liner is None or route_liner is None or not (
            math.isclose(pack_liner, plan_liner, abs_tol=1.0e-12)
            and math.isclose(pack_liner, route_liner, abs_tol=1.0e-12)):
        integrity_failures.append(
            "liner measurement differs across packing, plan and routes")

    packing_is_measured = (
        packing.get("role") == "authoritative_measured_release_job"
        and packing.get("config", {}).get("input_provenance")
        == "measured_receiving_input"
    )
    plan_status = plan.get("selected_case", {}).get("status")
    margins = extract_noncontact_route_margins(routes)
    result = evaluate_budget(
        margins,
        evidence,
        packing_is_measured=packing_is_measured,
        upstream_statuses={
            "packing": str(packing.get("status", "NOT_PROVEN")),
            "plan": str(plan_status or "NOT_PROVEN"),
            "routes": str(routes.get("status", "NOT_PROVEN")),
        },
        m0_mm_per_rad=float(PARAMS.mm_per_rad),
        encoder_ppr=encoder_ppr,
        quadrature_multiplier=quadrature_multiplier,
    )
    if integrity_failures:
        result["hard_failures"] = sorted(set(
            [*result["hard_failures"], *integrity_failures]))
        result["status"] = "FAIL"

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": result.pop("status"),
        "role": "hardware_motion_release_gate",
        "input_artifacts": {
            "packing": _artifact_record(
                packing_path, packing, "report_sha256"),
            "plan": _artifact_record(plan_path, plan, "proof_sha256"),
            "routes": _artifact_record(
                routes_path, routes, "report_sha256"),
        },
        "job_identity": {
            "packing_role": packing.get("role"),
            "packing_input_provenance": packing.get("config", {}).get(
                "input_provenance"),
            "wire_finished_diameter_mm": pack_wire,
            "liner_thickness_mm": pack_liner,
            "packing_report_sha256": packing_hash,
        },
        **result,
        "source_hashes": {
            "sim/placement_tolerance.py": _sha256(Path(__file__)),
            "sim/controller_adapter.py": _sha256(
                HERE / "controller_adapter.py"),
            "cad/params.py": _sha256(CAD / "params.py"),
            "bom.csv": _sha256(ROOT / "bom.csv"),
        },
        "release_rule": (
            "hardware motion may be authorized only when this status is "
            "PASS and its report hash is bound into the generated job"),
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError(f"report schema must be {SCHEMA}")
    problem = _verify_embedded_hash(report, "report_sha256")
    if problem:
        raise ValueError(problem)
    if report.get("status") not in {"PASS", "NOT_PROVEN", "FAIL"}:
        raise ValueError("invalid tolerance report status")


def render_markdown(report: Mapping[str, Any]) -> str:
    axis = report["axis_contract"]
    margins = report["route_margins"]
    budget = report["budget"]
    failures = report["hard_failures"]
    unknowns = report["unknowns"]
    lines = [
        "# Physical placement and route tolerance budget",
        "",
        f"Status: **{report['status']}**",
        "",
        "## Known machine terms",
        "",
        f"- T8 conversion: {axis['m0_mm_per_rad']:.9f} mm/rad "
        f"({axis['t8_lead_mm_per_rev']:.6f} mm/rev)",
        f"- Controller M0 acceptance: "
        f"{axis['controller_m0_settle_tolerance_rad']:.7f} rad = "
        f"{axis['controller_acceptance_mm']:.6f} mm",
        f"- Encoder: {axis['encoder_counts_per_rev']} quadrature counts/rev; "
        f"half-count uncertainty {axis['encoder_half_count_quantization_mm']:.6f} mm",
        "",
        "## Contact-excluded route margins",
        "",
        f"- Nonparent copper margin: {margins['copper_nonparent_margin_mm']} mm",
        f"- Noncontact core margin: {margins['core_noncontact_margin_mm']} mm",
        f"- Known copper budget lower bound: "
        f"{budget['copper_known_lower_bound_mm']:.6f} mm",
        f"- Known core budget lower bound: "
        f"{budget['core_known_lower_bound_mm']:.6f} mm",
        "",
        "Designed parent-wire and liner contacts are zero-gap supports and are "
        "excluded from these clearance margins.",
        "",
        "## Hard failures",
        "",
        *([f"- {item}" for item in failures] or ["- None"]),
        "",
        "## Evidence still unknown",
        "",
        *([f"- {item}" for item in unknowns] or ["- None"]),
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ]
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any], json_path: Path,
                  markdown_path: Path) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    markdown_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")


def _evidence_from_args(args: argparse.Namespace) -> PhysicalEvidence:
    return PhysicalEvidence(
        m0_observed_max_error_rad=args.m0_observed_error_rad,
        m0_observation_source=args.m0_observation_source,
        m0_carriage_physical_error_mm=args.m0_carriage_error_mm,
        m0_carriage_source=args.m0_carriage_source,
        spindle_tir_mm=args.spindle_tir_mm,
        spindle_tir_source=args.spindle_tir_source,
        mounted_stator_tir_mm=args.stator_tir_mm,
        mounted_stator_tir_source=args.stator_tir_source,
        wire_diameter_instrument_uncertainty_mm=args.wire_uncertainty_mm,
        wire_measurement_source=args.wire_measurement_source,
        liner_thickness_instrument_uncertainty_mm=args.liner_uncertainty_mm,
        liner_measurement_source=args.liner_measurement_source,
        contact_position_uncertainty_mm=args.contact_uncertainty_mm,
        contact_uncertainty_source=args.contact_uncertainty_source,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--packing", type=Path, default=PACKING_PATH)
    parser.add_argument("--plan", type=Path, default=PLAN_PATH)
    parser.add_argument("--routes", type=Path, default=ROUTES_PATH)
    parser.add_argument("--json", type=Path, default=OUTPUT_PATH)
    parser.add_argument("--markdown", type=Path, default=MARKDOWN_PATH)
    parser.add_argument("--encoder-ppr", type=int,
                        default=DEFAULT_ENCODER_PPR)
    parser.add_argument("--quadrature-multiplier", type=int,
                        default=DEFAULT_QUADRATURE_MULTIPLIER)
    parser.add_argument("--m0-observed-error-rad", type=float)
    parser.add_argument("--m0-observation-source")
    parser.add_argument("--m0-carriage-error-mm", type=float)
    parser.add_argument("--m0-carriage-source")
    parser.add_argument("--spindle-tir-mm", type=float)
    parser.add_argument("--spindle-tir-source")
    parser.add_argument("--stator-tir-mm", type=float)
    parser.add_argument("--stator-tir-source")
    parser.add_argument("--wire-uncertainty-mm", type=float)
    parser.add_argument("--wire-measurement-source")
    parser.add_argument("--liner-uncertainty-mm", type=float)
    parser.add_argument("--liner-measurement-source")
    parser.add_argument("--contact-uncertainty-mm", type=float)
    parser.add_argument("--contact-uncertainty-source")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = build_report(
        packing_path=args.packing,
        plan_path=args.plan,
        routes_path=args.routes,
        evidence=_evidence_from_args(args),
        encoder_ppr=args.encoder_ppr,
        quadrature_multiplier=args.quadrature_multiplier,
    )
    validate_report(report)
    write_outputs(report, args.json, args.markdown)
    print(
        f"physical placement tolerance {report['status']}: "
        f"copper margin={report['route_margins']['copper_nonparent_margin_mm']}, "
        f"known budget={report['budget']['copper_known_lower_bound_mm']:.6f} mm"
    )
    if args.check and report["status"] != "PASS":
        raise SystemExit(
            "physical placement tolerance is not PASS; hardware motion refused")


if __name__ == "__main__":
    main()
