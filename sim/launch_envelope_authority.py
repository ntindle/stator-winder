"""Fail-closed authority matrix for the declared OD28..65 launch envelope.

This audit is intentionally isolated from the current CAD, job generators,
collision audits, and purchasing catalog.  It does not infer launch support
from parameter limits or from the existing OD46 default-job artifacts.  A
launch row passes only when a self-hashed certificate bundle exists for the
exact geometry corner, finished-wire extreme, and workholding endpoint, and
every referenced artifact/source hash is current.

Certificate bundles live under::

    out/launch_certificates/<case_id>/certificate.json

The normalized certificate contract is documented by
``certificate_contract()`` below.  The bundle owns its capture, STEP,
settings, packing, collision, wire, load, and buildability evidence.  This
prevents a launch case from silently reusing the canonical OD46 files.

OD90 is a separate, non-gating generation advisory.  Even a complete OD90
generation bundle never contributes to ``production_authorized``.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
DEFAULT_CERTIFICATE_ROOT = ROOT / "out" / "launch_certificates"
JSON_OUT = REPORTS / "launch_envelope_authority.json"
MD_OUT = REPORTS / "launch_envelope_authority.md"

SCHEMA = "launch-envelope-authority/v1"
CORNER_CERTIFICATE_SCHEMA = "launch-corner-certificate/v1"
CORNER_EVIDENCE_SCHEMA = "launch-corner-evidence/v1"
OD90_CERTIFICATE_SCHEMA = "od90-advisory-generation-certificate/v1"

LAUNCH_ODS_MM = (28.0, 65.0)
LAUNCH_STACKS_MM = (5.0, 20.0)
LAUNCH_WIRE_EXTREMES_MM = (0.2, 0.5)
MINIMUM_DYNAMIC_CLEARANCE_MM = 2.0
MINIMUM_WIRE_BEND_RADIUS_MM = 3.0
MINIMUM_AXIS_MARGIN_MULTIPLE = 2.0

# A machine handling envelope is not a promise that one arbitrary internal
# lamination topology exists at every OD.  The earlier matrix silently forced
# the OD28 endpoint into the default 24n22p/0.52-hub topology; with the
# selected 5 mil slot liner that synthetic throat cannot admit 0.5 mm wire.
# Upstream itself ships both of the winding configurations below.  Bind the
# representative topology into every launch job so the endpoint evidence is
# explicit and cannot be mistaken for universal packing authority.
UPSTREAM_12N14P_CONFIG_ID = "dev-12n14p-settings.yml"
UPSTREAM_12N14P_WINDING_CONFIG = "AabBCcaABbcC"
UPSTREAM_24N22P_CONFIG_ID = "dev-24n22p-settings.yml"
UPSTREAM_24N22P_WINDING_CONFIG = "AaAabBbBCcCcaAaABbBbcCcC"
OD28_ER11_REPRESENTATIVE_HUB_OD_MM = 19.5
OD28_SHAFT8_REPRESENTATIVE_HUB_OD_MM = 16.0
REPRESENTATIVE_REACH_RESERVE_MM = 0.25


@dataclass(frozen=True)
class WorkholdingCoverage:
    id: str
    spindle_id: str
    shaft_d_mm: float
    reason: str


WORKHOLDING_COVERAGE = (
    WorkholdingCoverage(
        "er11_shaft3", "er11", 3.0,
        "ER11 lower declared shaft endpoint",
    ),
    WorkholdingCoverage(
        "er11_shaft7", "er11", 7.0,
        "ER11 upper physical shaft endpoint",
    ),
    WorkholdingCoverage(
        "shaft8", "shaft8", 8.0,
        "dedicated shaft8 holder and GOAL upper endpoint",
    ),
)

REQUIRED_ARTIFACT_KEYS = (
    "capture",
    "step",
    "settings",
    "packing",
    "collision",
    "wire",
    "load",
    "buildability",
)

# These are the minimum source owners whose current bytes must be bound into
# every exact launch-corner certificate.  Future sources may be added to a
# certificate, but omitting any owner below fails closed.
REQUIRED_SOURCE_PATHS = (
    "cad/params.py",
    "cad/stator_model.py",
    "cad/assembly.py",
    "cad/settings_gen.py",
    "cad/integrated_release_candidate.py",
    "cad/permanent_cap_production_review.py",
    "cad/carriage_active_sector_terminal_guide.py",
    "cad/slot_packing_audit.py",
    "cad/coil_growth.py",
    "cad/wire_geometry.py",
    "cad/job_artifacts.py",
    "cad/loads.py",
    "cad/buildability.py",
    "sim/capture.py",
    "sim/verify_cycle.py",
    "sim/traj.py",
    "sim/collide.py",
    "sim/slot_packing.py",
    "sim/slot_wire_routes.py",
    "sim/wirepath.py",
    "sim/carriage_active_sector_terminal_guide_audit.py",
    "sim/full_cycle_continuous_conductor_authority_audit.py",
    "sim/launch_envelope_authority.py",
    "sim/launch_envelope_case_generator.py",
)

REQUIRED_VERDICT_CONTRACT: Mapping[str, Mapping[str, Any]] = {
    "capture": {
        "status": "PASS",
        "unmodified_upstream": True,
        "cycle_complete": True,
        "all_required_motion_classes_present": True,
        "three_phase_configured_slot_cycle_complete": True,
        "both_shaft_wraps_exactly_two_turns": True,
    },
    "step": {
        "status": "PASS",
        "exact_job_geometry": True,
        "valid_closed_geometry": True,
    },
    "settings": {
        "status": "PASS",
        "job_identity_bound": True,
        "physical_travel_within_limits": True,
    },
    "packing": {
        "status": "PASS",
        "job_identity_bound": True,
        "capacity_and_opening_pass": True,
    },
    "collision": {
        "status": "PASS",
        "job_identity_bound": True,
        "collision_count": 0,
        "minimum_dynamic_clearance_mm": MINIMUM_DYNAMIC_CLEARANCE_MM,
        "full_raw_motion_covered": True,
    },
    "wire": {
        "status": "PASS",
        "job_identity_bound": True,
        "minimum_bend_radius_mm": MINIMUM_WIRE_BEND_RADIUS_MM,
        "unintended_contact_count": 0,
        "all_deposition_loci_and_intervals_proven": True,
        "continuous_conductor_proven": True,
    },
    "load": {
        "status": "PASS",
        "job_identity_bound": True,
        "minimum_axis_margin_multiple": MINIMUM_AXIS_MARGIN_MULTIPLE,
        "M0_M1_M2_all_pass": True,
    },
    "buildability": {
        "status": "PASS",
        "job_identity_bound": True,
        "all_parts_buildable": True,
        "all_parts_fit_220x220x250_mm": True,
    },
}

EXISTING_OD46_PATHS = {
    "capture": "out/capture/upstream_current_raw.jsonl",
    "settings": "out/settings.yml",
    "assembly_manifest": "out/links/manifest.json",
    "active_sector_audit": (
        "out/reports/carriage_active_sector_terminal_guide_audit.json"
    ),
    "cycle_audit": "out/reports/upstream_current_raw_cycle.json",
    "packing": "out/reports/slot_packing.json",
    "slot_wire_routes": "out/reports/slot_wire_routes.json",
    "continuous_wire": "out/reports/continuous_wire_audit.json",
}

_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class LaunchCase:
    case_id: str
    od_mm: float
    stack_mm: float
    wire_finished_d_mm: float
    spindle_id: str
    shaft_d_mm: float
    slots: int = 24
    hub_od_ratio: float = 0.52
    winding_config: str = UPSTREAM_24N22P_WINDING_CONFIG
    upstream_config_id: str = UPSTREAM_24N22P_CONFIG_ID
    topology_basis: str = "representative_24n22p_parametric_lamination"
    reach_reserve_mm: float = REPRESENTATIVE_REACH_RESERVE_MM

    def expected_job(self) -> dict[str, Any]:
        return {
            "od_mm": self.od_mm,
            "stack_mm": self.stack_mm,
            "wire_finished_d_mm": self.wire_finished_d_mm,
            "spindle_id": self.spindle_id,
            "shaft_d_mm": self.shaft_d_mm,
            "slots": self.slots,
            "hub_od_mm": self.od_mm * self.hub_od_ratio,
            "hub_od_ratio": self.hub_od_ratio,
            "winding_config": self.winding_config,
            "upstream_config_id": self.upstream_config_id,
            "topology_basis": self.topology_basis,
            "reach_reserve_mm": self.reach_reserve_mm,
        }


def _number_token(value: float) -> str:
    text = f"{float(value):g}".replace("-", "m").replace(".", "p")
    return text


def required_launch_cases() -> tuple[LaunchCase, ...]:
    rows: list[LaunchCase] = []
    for od in LAUNCH_ODS_MM:
        for stack in LAUNCH_STACKS_MM:
            for wire in LAUNCH_WIRE_EXTREMES_MM:
                for workholding in WORKHOLDING_COVERAGE:
                    if od == 28.0:
                        slots = 12
                        winding_config = UPSTREAM_12N14P_WINDING_CONFIG
                        upstream_config_id = UPSTREAM_12N14P_CONFIG_ID
                        hub_od = (
                            OD28_SHAFT8_REPRESENTATIVE_HUB_OD_MM
                            if workholding.spindle_id == "shaft8" else
                            OD28_ER11_REPRESENTATIVE_HUB_OD_MM
                        )
                        hub_od_ratio = hub_od / od
                        topology_basis = (
                            "representative_OD28_12n14p_lamination_sized_"
                            "to_current_workholder_reach"
                        )
                    else:
                        slots = 24
                        winding_config = UPSTREAM_24N22P_WINDING_CONFIG
                        upstream_config_id = UPSTREAM_24N22P_CONFIG_ID
                        hub_od_ratio = 0.52
                        topology_basis = (
                            "representative_OD65_24n22p_parametric_lamination"
                        )
                    case_id = (
                        f"od{_number_token(od)}_stack{_number_token(stack)}_"
                        f"wire{_number_token(wire)}_{workholding.id}"
                    )
                    rows.append(LaunchCase(
                        case_id=case_id,
                        od_mm=od,
                        stack_mm=stack,
                        wire_finished_d_mm=wire,
                        spindle_id=workholding.spindle_id,
                        shaft_d_mm=workholding.shaft_d_mm,
                        slots=slots,
                        hub_od_ratio=hub_od_ratio,
                        winding_config=winding_config,
                        upstream_config_id=upstream_config_id,
                        topology_basis=topology_basis,
                    ))
    return tuple(rows)


OD90_ADVISORY_CASE = LaunchCase(
    case_id="advisory_od90_stack20_wire0p2_er11_shaft4",
    od_mm=90.0,
    stack_mm=20.0,
    wire_finished_d_mm=0.2,
    spindle_id="er11",
    shaft_d_mm=4.0,
)


def _canonical_hash(value: Any, *, omit: Sequence[str] = ()) -> str:
    if not isinstance(value, Mapping):
        payload = value
    else:
        payload = {key: val for key, val in value.items() if key not in omit}
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _resolve_under(root: Path, relative: str) -> Path:
    candidate = Path(relative)
    if candidate.is_absolute():
        raise ValueError("path must be machine-root-relative")
    resolved_root = root.resolve()
    resolved = (resolved_root / candidate).resolve()
    try:
        resolved.relative_to(resolved_root)
    except ValueError as exc:
        raise ValueError("path escapes machine root") from exc
    return resolved


def _relative(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("JSON root must be an object")
    return value


def _load_settings(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore

        value = yaml.safe_load(text)
        if isinstance(value, dict):
            return value
    except (ImportError, ValueError, TypeError):
        pass
    # JSON is valid YAML and is sufficient for small isolated certificate
    # fixtures if PyYAML is unavailable.
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError("settings root must be an object")
    return value


def _capture_rows(path: Path) -> tuple[dict[str, Any], bool]:
    meta: dict[str, Any] | None = None
    cycle_complete = False
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw in enumerate(handle, 1):
            if not raw.strip():
                continue
            value = json.loads(raw)
            if not isinstance(value, dict):
                raise ValueError(f"capture row {line_number} is not an object")
            if value.get("e") == "meta":
                if meta is not None:
                    raise ValueError("capture has more than one meta row")
                meta = value
            if value.get("e") == "cycle_complete":
                cycle_complete = True
    if meta is None:
        raise ValueError("capture has no meta row")
    return meta, cycle_complete


def _float_equal(actual: Any, expected: float) -> bool:
    try:
        return math.isclose(
            float(actual), float(expected), rel_tol=0.0, abs_tol=1.0e-9,
        )
    except (TypeError, ValueError):
        return False


def _job_errors(
    actual: Mapping[str, Any], expected: Mapping[str, Any], *, prefix: str,
    require_turns: bool = False,
) -> list[str]:
    errors: list[str] = []
    aliases = {
        "od_mm": ("od_mm", "od"),
        "stack_mm": ("stack_mm", "stack"),
        "wire_finished_d_mm": (
            "wire_finished_d_mm", "wire_finished_diameter_mm", "wire_d",
        ),
        "shaft_d_mm": ("shaft_d_mm", "shaft_d"),
        "spindle_id": ("spindle_id",),
        "slots": ("slots",),
    }
    for key, expected_value in expected.items():
        names = aliases.get(key, (key,))
        value = next((actual[name] for name in names if name in actual), None)
        if isinstance(expected_value, (int, float)) and not isinstance(
                expected_value, bool):
            if not _float_equal(value, float(expected_value)):
                errors.append(
                    f"{prefix}.{key}={value!r}, expected {expected_value!r}"
                )
        elif value != expected_value:
            errors.append(
                f"{prefix}.{key}={value!r}, expected {expected_value!r}"
            )
    if require_turns:
        value = actual.get("turns_per_tooth", actual.get("turns"))
        try:
            if int(value) <= 0:
                raise ValueError
        except (TypeError, ValueError):
            errors.append(f"{prefix}.turns_per_tooth must be a positive integer")
    return errors


def _artifact_row_errors(
    root: Path,
    certificate_dir: Path,
    key: str,
    row: Any,
    expected_job: Mapping[str, Any],
) -> tuple[list[str], Path | None, str | None]:
    errors: list[str] = []
    if not isinstance(row, Mapping):
        return [f"artifacts.{key} must be an object"], None, None
    relative = row.get("path")
    expected_sha = row.get("sha256")
    if not isinstance(relative, str) or not relative:
        errors.append(f"artifacts.{key}.path is missing")
        return errors, None, None
    if (not isinstance(expected_sha, str)
            or expected_sha != expected_sha.lower()
            or not _SHA256_RE.fullmatch(expected_sha)):
        errors.append(f"artifacts.{key}.sha256 is not lowercase SHA-256")
        return errors, None, None
    try:
        path = _resolve_under(root, relative)
    except ValueError as exc:
        errors.append(f"artifacts.{key}.path: {exc}")
        return errors, None, None
    try:
        path.relative_to(certificate_dir.resolve())
    except ValueError:
        errors.append(
            f"artifacts.{key}.path must be inside its case certificate bundle"
        )
    if not path.is_file():
        errors.append(f"artifacts.{key} file is missing: {relative}")
        return errors, path, None
    actual_sha = _sha256(path)
    if actual_sha != expected_sha.lower():
        errors.append(
            f"artifacts.{key} hash mismatch: certificate={expected_sha.lower()} "
            f"actual={actual_sha}"
        )
    declared_job = row.get("job_identity")
    if not isinstance(declared_job, Mapping):
        errors.append(f"artifacts.{key}.job_identity is missing")
    else:
        errors.extend(_job_errors(
            declared_job, expected_job, prefix=f"artifacts.{key}.job_identity",
        ))
    if path.stat().st_size <= 0:
        errors.append(f"artifacts.{key} file is empty")
    return errors, path, actual_sha


def _source_errors(root: Path, sources: Any) -> list[str]:
    if not isinstance(sources, Mapping):
        return ["sources must be a path-to-SHA256 object"]
    errors: list[str] = []
    for relative in REQUIRED_SOURCE_PATHS:
        if relative not in sources:
            errors.append(f"sources.{relative} is missing")
    for raw_relative, expected in sources.items():
        if not isinstance(raw_relative, str) or not raw_relative:
            errors.append("source path keys must be nonempty strings")
            continue
        relative = raw_relative.replace("\\", "/")
        if relative != raw_relative:
            errors.append(
                f"source path must use canonical forward slashes: {raw_relative}"
            )
            continue
        if (not isinstance(expected, str)
                or expected != expected.lower()
                or not _SHA256_RE.fullmatch(expected)):
            errors.append(f"sources.{relative} is invalid")
            continue
        try:
            path = _resolve_under(root, relative)
        except ValueError as exc:
            errors.append(f"sources.{relative}: {exc}")
            continue
        if not path.is_file():
            errors.append(f"source file is missing: {relative}")
            continue
        actual = _sha256(path)
        if actual != expected.lower():
            errors.append(
                f"source hash mismatch for {relative}: "
                f"certificate={expected.lower()} actual={actual}"
            )
    return errors


def _verdict_errors(verdicts: Any) -> list[str]:
    if not isinstance(verdicts, Mapping):
        return ["verdicts must be an object"]
    errors: list[str] = []
    minimum_fields = {
        ("collision", "minimum_dynamic_clearance_mm"),
        ("wire", "minimum_bend_radius_mm"),
        ("load", "minimum_axis_margin_multiple"),
    }
    for key, contract in REQUIRED_VERDICT_CONTRACT.items():
        row = verdicts.get(key)
        if not isinstance(row, Mapping):
            errors.append(f"verdicts.{key} is missing")
            continue
        for field, expected in contract.items():
            actual = row.get(field)
            if (key, field) in minimum_fields:
                try:
                    if float(actual) + 1.0e-12 < float(expected):
                        errors.append(
                            f"verdicts.{key}.{field}={actual!r}, "
                            f"minimum {expected!r}"
                        )
                except (TypeError, ValueError):
                    errors.append(
                        f"verdicts.{key}.{field} must be numeric and >= "
                        f"{expected!r}"
                    )
            elif actual != expected:
                errors.append(
                    f"verdicts.{key}.{field}={actual!r}, expected {expected!r}"
                )
    return errors


def _semantic_artifact_errors(
    case: LaunchCase,
    paths: Mapping[str, Path | None],
    actual_hashes: Mapping[str, str | None],
) -> list[str]:
    errors: list[str] = []
    expected = case.expected_job()

    step = paths.get("step")
    if step is not None and step.is_file() and step.suffix.lower() not in {
            ".step", ".stp"}:
        errors.append("STEP evidence must use .step or .stp")
    capture = paths.get("capture")
    capture_meta: dict[str, Any] | None = None
    if capture is not None and capture.is_file():
        if capture.suffix.lower() != ".jsonl":
            errors.append("capture evidence must use .jsonl")
        try:
            capture_meta, cycle_complete = _capture_rows(capture)
            job = capture_meta.get("job")
            if not isinstance(job, Mapping):
                errors.append("capture meta.job is missing")
            else:
                errors.extend(_job_errors(
                    job, expected, prefix="capture.meta.job",
                    require_turns=True,
                ))
            if capture_meta.get("controller_mode") != "upstream":
                errors.append("capture controller_mode must be upstream")
            if capture_meta.get("upstream_source_modified_by_harness") is True:
                errors.append("capture says upstream source was modified")
            if capture_meta.get("winder_dirty") is True:
                errors.append("capture says the upstream winder checkout was dirty")
            if not cycle_complete:
                errors.append("capture has no cycle_complete row")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            errors.append(f"capture semantic validation failed: {exc}")

    settings = paths.get("settings")
    if settings is not None and settings.is_file():
        if settings.suffix.lower() not in {".yml", ".yaml", ".json"}:
            errors.append("settings evidence must use .yml, .yaml, or .json")
        try:
            document = _load_settings(settings)
            job = document.get("job")
            if not isinstance(job, Mapping):
                errors.append("settings job is missing")
            else:
                errors.extend(_job_errors(
                    job, expected, prefix="settings.job",
                ))
            winding = document.get("winding")
            if not isinstance(winding, Mapping):
                errors.append("settings winding block is missing")
            else:
                try:
                    if int(winding.get("turns")) <= 0:
                        raise ValueError
                except (TypeError, ValueError):
                    errors.append("settings winding.turns must be positive")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            errors.append(f"settings semantic validation failed: {exc}")

    if capture_meta is not None:
        bound = capture_meta.get("settings_sha256")
        settings_sha = actual_hashes.get("settings")
        if not isinstance(bound, str) or bound.lower() != settings_sha:
            errors.append(
                "capture settings_sha256 does not match the bundled settings"
            )

    packing = paths.get("packing")
    if packing is not None and packing.is_file():
        try:
            document = _load_json(packing)
            if document.get("status") != "PASS":
                errors.append("packing report status is not PASS")
            identity = document.get("job", document.get("config"))
            if not isinstance(identity, Mapping):
                errors.append("packing report has no job/config identity")
            else:
                packing_expected = {
                    key: expected[key]
                    for key in ("od_mm", "stack_mm", "wire_finished_d_mm", "slots")
                }
                errors.extend(_job_errors(
                    identity, packing_expected, prefix="packing.job",
                ))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            errors.append(f"packing semantic validation failed: {exc}")

    for key in ("collision", "wire", "load", "buildability"):
        path = paths.get(key)
        if path is None or not path.is_file():
            continue
        if path.suffix.lower() != ".json":
            errors.append(f"{key} evidence must use .json")
            continue
        try:
            document = _load_json(path)
            if document.get("status") != "PASS":
                errors.append(f"{key} report status is not PASS")
            identity = document.get("job")
            if not isinstance(identity, Mapping):
                errors.append(f"{key} report job identity is missing")
            else:
                errors.extend(_job_errors(
                    identity, expected, prefix=f"{key}.job",
                ))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            errors.append(f"{key} semantic validation failed: {exc}")
    return errors


def _blocked_evidence_semantic_errors(
    case: LaunchCase,
    certificate: Mapping[str, Any],
    paths: Mapping[str, Path | None],
    actual_hashes: Mapping[str, str | None],
) -> list[str]:
    """Validate a current, explicitly non-authorizing evidence bundle.

    The generator can produce useful exact geometry, settings, analytical
    reports, and untouched-upstream captures before every authority gate is
    closed.  Those artifacts must not be forced through the PASS-certificate
    semantics above: doing so would either discard useful progress or tempt a
    caller to relabel a failed raw capture as PASS.  This path therefore checks
    identity, hashes, capture/settings binding, and fail-closed verdict
    consistency while requiring the corner and production claims to remain
    false.
    """

    errors: list[str] = []
    expected = case.expected_job()
    verdicts = certificate.get("verdicts")

    def validate_json_report(
        key: str, document: Mapping[str, Any], *, allow_missing_hash: bool = False,
    ) -> None:
        verdict = verdicts.get(key) if isinstance(verdicts, Mapping) else None
        if isinstance(verdict, Mapping):
            if document.get("status") != verdict.get("status"):
                errors.append(
                    f"{key} report status does not match certificate verdict"
                )
        claimed = document.get("report_sha256")
        if claimed is None and allow_missing_hash:
            return
        calculated = _canonical_hash(document, omit=("report_sha256",))
        if claimed != calculated:
            errors.append(f"{key} report_sha256 is invalid")

    certificate_job = certificate.get("job")
    turns = None
    if isinstance(certificate_job, Mapping):
        turns = certificate_job.get(
            "turns_per_tooth", certificate_job.get("turns")
        )

    step = paths.get("step")
    if step is not None and step.is_file() and step.suffix.lower() not in {
            ".step", ".stp"}:
        errors.append("STEP evidence must use .step or .stp")
    if step is not None and step.is_file() and isinstance(verdicts, Mapping):
        step_verdict = verdicts.get("step")
        geometry_facts = (
            step_verdict.get("geometry_facts")
            if isinstance(step_verdict, Mapping) else None
        )
        if (isinstance(geometry_facts, Mapping)
                and geometry_facts.get("sha256") is not None
                and geometry_facts.get("sha256") != actual_hashes.get("step")):
            errors.append("STEP geometry facts hash does not match artifact")

    settings = paths.get("settings")
    settings_document: dict[str, Any] | None = None
    if settings is not None and settings.is_file():
        if settings.suffix.lower() not in {".yml", ".yaml", ".json"}:
            errors.append(
                "blocked settings evidence must use .yml, .yaml, or .json"
            )
        else:
            try:
                settings_document = _load_settings(settings)
                identity = settings_document.get("job")
                if not isinstance(identity, Mapping):
                    errors.append("blocked settings evidence has no job identity")
                else:
                    errors.extend(_job_errors(
                        identity, expected, prefix="settings.job",
                    ))
                winding = settings_document.get("winding")
                if isinstance(winding, Mapping) and turns is not None:
                    if not _float_equal(winding.get("turns"), float(turns)):
                        errors.append(
                            "settings winding.turns does not match certificate job"
                        )
                if settings.suffix.lower() == ".json":
                    validate_json_report("settings", settings_document)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                errors.append(f"blocked settings evidence cannot be read: {exc}")

    capture = paths.get("capture")
    if capture is not None and capture.is_file():
        if capture.suffix.lower() == ".jsonl":
            try:
                meta, cycle_complete = _capture_rows(capture)
                identity = meta.get("job")
                if not isinstance(identity, Mapping):
                    errors.append("blocked capture meta.job is missing")
                else:
                    errors.extend(_job_errors(
                        identity, expected, prefix="capture.meta.job",
                    ))
                if meta.get("controller_mode") != "upstream":
                    errors.append("raw launch capture controller_mode is not upstream")
                if not cycle_complete:
                    errors.append("raw launch capture has no cycle_complete row")
                if meta.get("settings_sha256") != actual_hashes.get("settings"):
                    errors.append(
                        "raw launch capture settings_sha256 does not match settings"
                    )
                if turns is not None and not _float_equal(
                    meta.get("turns", meta.get("settings_turns")), float(turns)
                ):
                    errors.append("raw launch capture turns do not match certificate")
                verdict = (
                    verdicts.get("capture")
                    if isinstance(verdicts, Mapping) else None
                )
                if (isinstance(verdict, Mapping)
                        and verdict.get("capture_sha256") is not None
                        and verdict.get("capture_sha256")
                        != actual_hashes.get("capture")):
                    errors.append(
                        "capture verdict hash does not match raw capture artifact"
                    )
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                errors.append(f"blocked capture evidence cannot be read: {exc}")
        elif capture.suffix.lower() == ".json":
            try:
                document = _load_json(capture)
                identity = document.get("job")
                if not isinstance(identity, Mapping):
                    errors.append("capture-unavailable report has no job identity")
                else:
                    errors.extend(_job_errors(
                        identity, expected, prefix="capture_unavailable.job",
                    ))
                if document.get("status") == "PASS":
                    errors.append(
                        "JSON capture substitute may not claim PASS; use raw JSONL"
                    )
                validate_json_report("capture", document)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                errors.append(f"capture-unavailable report cannot be read: {exc}")
        else:
            errors.append("blocked capture evidence must use .jsonl or .json")

    for key in ("packing", "collision", "wire", "load", "buildability"):
        path = paths.get(key)
        if path is None or not path.is_file():
            continue
        if path.suffix.lower() != ".json":
            errors.append(f"blocked {key} evidence must use .json")
            continue
        try:
            document = _load_json(path)
            identity = document.get("job")
            if not isinstance(identity, Mapping):
                errors.append(f"blocked {key} report job identity is missing")
            else:
                errors.extend(_job_errors(
                    identity, expected, prefix=f"{key}.job",
                ))
            validate_json_report(key, document)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            errors.append(f"blocked {key} evidence cannot be read: {exc}")

    if not isinstance(verdicts, Mapping):
        errors.append("blocked evidence verdicts must be an object")
    else:
        nonpassing: list[str] = []
        for key in REQUIRED_ARTIFACT_KEYS:
            row = verdicts.get(key)
            if not isinstance(row, Mapping):
                errors.append(f"blocked evidence verdicts.{key} is missing")
                continue
            status = row.get("status")
            if status not in {"PASS", "FAIL", "BLOCKED"}:
                errors.append(
                    f"blocked evidence verdicts.{key}.status is invalid"
                )
            if status != "PASS":
                nonpassing.append(key)
        blockers = certificate.get("blocking_gates")
        if not isinstance(blockers, list) or any(
                not isinstance(item, str) for item in blockers):
            errors.append("blocking_gates must be a list of gate names")
        elif sorted(blockers) != sorted(nonpassing):
            errors.append(
                "blocking_gates must equal the non-PASS required verdict gates"
            )
        if not nonpassing:
            errors.append(
                "blocked evidence must retain at least one non-PASS verdict"
            )
    return errors


def _validate_blocked_corner_evidence(
    root: Path,
    certificate_path: Path,
    case: LaunchCase,
    certificate: Mapping[str, Any],
    base: Mapping[str, Any],
) -> dict[str, Any]:
    """Return a current-but-blocked row without promoting it to authority."""

    errors: list[str] = []
    if certificate.get("case_id") != case.case_id:
        errors.append("evidence case_id does not match its directory")
    if certificate.get("status") != "FAIL_CLOSED":
        errors.append("evidence status must be FAIL_CLOSED")
    if certificate.get("corner_authorized") is not False:
        errors.append("blocked evidence corner_authorized must be false")
    if certificate.get("production_authorized") is not False:
        errors.append("blocked evidence production_authorized must be false")
    if certificate.get("source_dependency_closure_complete") is not True:
        errors.append("source_dependency_closure_complete is not true")
    job = certificate.get("job")
    if not isinstance(job, Mapping):
        errors.append("evidence job is missing")
    else:
        errors.extend(_job_errors(
            job, case.expected_job(), prefix="evidence.job",
            require_turns=True,
        ))
    calculated_payload = _canonical_hash(
        certificate, omit=("certificate_payload_sha256",),
    )
    if certificate.get("certificate_payload_sha256") != calculated_payload:
        errors.append(
            "certificate_payload_sha256 does not match canonical evidence payload"
        )
    errors.extend(_source_errors(root, certificate.get("sources")))

    artifacts = certificate.get("artifacts")
    paths: dict[str, Path | None] = {}
    actual_hashes: dict[str, str | None] = {}
    missing_artifact_keys: list[str] = []
    certificate_dir = certificate_path.parent
    if not isinstance(artifacts, Mapping):
        errors.append("artifacts must be an object")
        missing_artifact_keys.extend(REQUIRED_ARTIFACT_KEYS)
    else:
        for key in REQUIRED_ARTIFACT_KEYS:
            if key not in artifacts:
                missing_artifact_keys.append(key)
                errors.append(f"artifacts.{key} is missing")
                paths[key] = None
                actual_hashes[key] = None
                continue
            row_errors, path, actual_sha = _artifact_row_errors(
                root, certificate_dir, key, artifacts[key],
                case.expected_job(),
            )
            errors.extend(row_errors)
            paths[key] = path
            actual_hashes[key] = actual_sha
        for key, row in artifacts.items():
            if key in REQUIRED_ARTIFACT_KEYS:
                continue
            row_errors, _path, _actual_sha = _artifact_row_errors(
                root, certificate_dir, str(key), row, case.expected_job(),
            )
            errors.extend(row_errors)
    errors.extend(_blocked_evidence_semantic_errors(
        case, certificate, paths, actual_hashes,
    ))

    missing_sources = [
        path for path in REQUIRED_SOURCE_PATHS
        if not isinstance(certificate.get("sources"), Mapping)
        or path not in certificate["sources"]
    ]
    blockers = list(certificate.get("blocking_gates", []))
    if errors:
        return {
            **base,
            "status": "INVALID_CERTIFICATE",
            "certificate_current": False,
            "corner_authorized": False,
            "errors": errors,
            "blocking_gates": blockers,
            "missing_artifact_keys": missing_artifact_keys,
            "missing_source_paths": missing_sources,
            "certificate_sha256": _sha256(certificate_path),
            "certificate_payload_sha256": calculated_payload,
        }
    return {
        **base,
        "status": "EVIDENCE_BUNDLE_BLOCKED",
        "certificate_current": True,
        "corner_authorized": False,
        "errors": [f"{gate} gate is not PASS" for gate in blockers],
        "blocking_gates": blockers,
        "missing_artifact_keys": [],
        "missing_source_paths": [],
        "certificate_sha256": _sha256(certificate_path),
        "certificate_payload_sha256": calculated_payload,
    }


def _validate_corner_certificate(
    root: Path, certificate_root: Path, case: LaunchCase,
) -> dict[str, Any]:
    certificate_path = certificate_root / case.case_id / "certificate.json"
    relative_certificate = _relative(root, certificate_path)
    base = {
        "case_id": case.case_id,
        "expected_job": case.expected_job(),
        "certificate_path": relative_certificate,
        "counts_toward_launch_matrix": True,
    }
    if not certificate_path.is_file():
        return {
            **base,
            "status": "MISSING_CERTIFICATE",
            "certificate_current": False,
            "errors": ["exact launch-corner certificate bundle is missing"],
            "missing_artifact_keys": list(REQUIRED_ARTIFACT_KEYS),
            "missing_source_paths": list(REQUIRED_SOURCE_PATHS),
        }
    errors: list[str] = []
    try:
        certificate = _load_json(certificate_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return {
            **base,
            "status": "INVALID_CERTIFICATE",
            "certificate_current": False,
            "errors": [f"certificate JSON cannot be read: {exc}"],
        }

    if certificate.get("schema") == CORNER_EVIDENCE_SCHEMA:
        return _validate_blocked_corner_evidence(
            root, certificate_path, case, certificate, base,
        )

    if certificate.get("schema") != CORNER_CERTIFICATE_SCHEMA:
        errors.append(
            f"schema must be {CORNER_CERTIFICATE_SCHEMA}"
        )
    if certificate.get("case_id") != case.case_id:
        errors.append("certificate case_id does not match its directory")
    if certificate.get("status") != "PASS":
        errors.append("certificate status is not PASS")
    if certificate.get("corner_authorized") is not True:
        errors.append("certificate corner_authorized is not true")
    if certificate.get("source_dependency_closure_complete") is not True:
        errors.append("source_dependency_closure_complete is not true")
    if certificate.get("production_authorized") is True:
        errors.append(
            "individual corner certificate may not authorize whole-machine production"
        )
    job = certificate.get("job")
    if not isinstance(job, Mapping):
        errors.append("certificate job is missing")
    else:
        errors.extend(_job_errors(
            job, case.expected_job(), prefix="certificate.job",
            require_turns=True,
        ))
    claimed_payload = certificate.get("certificate_payload_sha256")
    calculated_payload = _canonical_hash(
        certificate, omit=("certificate_payload_sha256",),
    )
    if claimed_payload != calculated_payload:
        errors.append(
            "certificate_payload_sha256 does not match canonical certificate payload"
        )

    errors.extend(_source_errors(root, certificate.get("sources")))
    errors.extend(_verdict_errors(certificate.get("verdicts")))

    artifacts = certificate.get("artifacts")
    paths: dict[str, Path | None] = {}
    actual_hashes: dict[str, str | None] = {}
    missing_artifact_keys: list[str] = []
    certificate_dir = certificate_path.parent
    if not isinstance(artifacts, Mapping):
        errors.append("artifacts must be an object")
        missing_artifact_keys.extend(REQUIRED_ARTIFACT_KEYS)
    else:
        for key in REQUIRED_ARTIFACT_KEYS:
            if key not in artifacts:
                missing_artifact_keys.append(key)
                errors.append(f"artifacts.{key} is missing")
                paths[key] = None
                actual_hashes[key] = None
                continue
            row_errors, path, actual_sha = _artifact_row_errors(
                root, certificate_dir, key, artifacts[key], case.expected_job(),
            )
            errors.extend(row_errors)
            paths[key] = path
            actual_hashes[key] = actual_sha
    errors.extend(_semantic_artifact_errors(case, paths, actual_hashes))

    status = "PASS" if not errors else "INVALID_CERTIFICATE"
    return {
        **base,
        "status": status,
        "certificate_current": not errors,
        "certificate_sha256": _sha256(certificate_path),
        "certificate_payload_sha256": calculated_payload,
        "errors": errors,
        "missing_artifact_keys": missing_artifact_keys,
        "missing_source_paths": [
            path for path in REQUIRED_SOURCE_PATHS
            if not isinstance(certificate.get("sources"), Mapping)
            or path not in certificate["sources"]
        ],
    }


def _existing_file_row(root: Path, relative: str) -> dict[str, Any]:
    try:
        path = _resolve_under(root, relative)
    except ValueError as exc:
        return {"path": relative, "exists": False, "error": str(exc)}
    if not path.is_file():
        return {"path": relative, "exists": False, "sha256": None}
    return {
        "path": relative,
        "exists": True,
        "sha256": _sha256(path),
        "size_bytes": path.stat().st_size,
    }


def _existing_od46_evidence(root: Path) -> dict[str, Any]:
    artifacts = {
        key: _existing_file_row(root, relative)
        for key, relative in EXISTING_OD46_PATHS.items()
    }
    observed_jobs: dict[str, Any] = {}
    report_statuses: dict[str, Any] = {}
    source_bindings: list[dict[str, Any]] = []

    capture_path = root / EXISTING_OD46_PATHS["capture"]
    if capture_path.is_file():
        try:
            meta, cycle_complete = _capture_rows(capture_path)
            observed_jobs["capture"] = meta.get("job")
            report_statuses["capture_cycle_complete_row_present"] = cycle_complete
            report_statuses["capture_controller_mode"] = meta.get("controller_mode")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            report_statuses["capture_error"] = str(exc)

    settings_path = root / EXISTING_OD46_PATHS["settings"]
    if settings_path.is_file():
        try:
            settings = _load_settings(settings_path)
            observed_jobs["settings"] = settings.get("job")
            report_statuses["settings_hardware_motion_authorized"] = (
                settings.get("job", {}).get("hardware_motion_authorized")
                if isinstance(settings.get("job"), Mapping) else None
            )
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            report_statuses["settings_error"] = str(exc)

    manifest_path = root / EXISTING_OD46_PATHS["assembly_manifest"]
    if manifest_path.is_file():
        try:
            manifest = _load_json(manifest_path)
            observed_jobs["assembly_manifest"] = manifest.get("stator")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            report_statuses["assembly_manifest_error"] = str(exc)

    for key in (
        "active_sector_audit", "cycle_audit", "packing",
        "slot_wire_routes", "continuous_wire",
    ):
        path = root / EXISTING_OD46_PATHS[key]
        if not path.is_file():
            report_statuses[key] = "MISSING"
            continue
        try:
            document = _load_json(path)
            report_statuses[key] = document.get("status")
            if key == "active_sector_audit":
                hashes = document.get("source_hashes")
                if isinstance(hashes, Mapping):
                    for relative, recorded in hashes.items():
                        try:
                            source = _resolve_under(root, str(relative))
                        except ValueError:
                            continue
                        if not source.is_file() or not isinstance(recorded, str):
                            continue
                        actual = _sha256(source)
                        source_bindings.append({
                            "path": str(relative),
                            "recorded_sha256": recorded,
                            "current_sha256": actual,
                            "current": recorded == actual,
                        })
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            report_statuses[key] = f"INVALID: {exc}"

    capture_inventory: list[dict[str, Any]] = []
    out = root / "out"
    if out.is_dir():
        for path in sorted(out.rglob("*.jsonl")):
            try:
                with path.open("r", encoding="utf-8") as handle:
                    first = next((line for line in handle if line.strip()), "")
                value = json.loads(first)
                job = value.get("job") if isinstance(value, Mapping) else None
                if isinstance(job, Mapping) and "od_mm" in job:
                    capture_inventory.append({
                        "path": _relative(root, path),
                        "od_mm": job.get("od_mm"),
                        "stack_mm": job.get("stack_mm"),
                        "wire_finished_d_mm": job.get("wire_finished_d_mm"),
                    })
            except (OSError, StopIteration, json.JSONDecodeError):
                continue
    by_od = Counter(str(row["od_mm"]) for row in capture_inventory)
    exact_default = {
        "od_mm": 46.0,
        "stack_mm": 15.0,
        "wire_finished_d_mm": 0.22352,
        "spindle_id": "er11",
        "shaft_d_mm": 4.0,
        "slots": 24,
    }
    identities_match_default = []
    for owner, job in observed_jobs.items():
        if not isinstance(job, Mapping):
            identities_match_default.append(False)
            continue
        subset = {
            key: value for key, value in exact_default.items()
            if key in job or key in {"od_mm", "stack_mm", "slots"}
        }
        identities_match_default.append(
            not _job_errors(job, subset, prefix=owner)
        )
    return {
        "classification": "EXISTING_DEFAULT_JOB_EVIDENCE_ONLY",
        "counts_toward_launch_matrix": False,
        "reason": (
            "OD46 is not one of the required OD28/OD65 geometry corners and "
            "cannot substitute for any launch certificate"
        ),
        "expected_default_job": exact_default,
        "observed_jobs": observed_jobs,
        "observed_identities_match_default_where_available": (
            bool(identities_match_default) and all(identities_match_default)
        ),
        "artifacts": artifacts,
        "report_statuses": report_statuses,
        "active_sector_source_bindings": source_bindings,
        "active_sector_all_recorded_sources_current": (
            bool(source_bindings)
            and all(row["current"] for row in source_bindings)
        ),
        "capture_inventory": {
            "count": len(capture_inventory),
            "by_od_mm": dict(sorted(by_od.items())),
            "non_OD46_capture_paths": [
                row["path"] for row in capture_inventory
                if not _float_equal(row["od_mm"], 46.0)
            ],
        },
    }


def _validate_od90_advisory(
    root: Path, certificate_root: Path,
) -> dict[str, Any]:
    case = OD90_ADVISORY_CASE
    path = certificate_root / case.case_id / "generation_certificate.json"
    base = {
        "case_id": case.case_id,
        "job": case.expected_job(),
        "certificate_path": _relative(root, path),
        "counts_toward_launch_matrix": False,
        "production_authority": "NONE_ADVISORY_ONLY",
    }
    if not path.is_file():
        return {
            **base,
            "status": "MISSING_ADVISORY_GENERATION",
            "errors": [
                "OD90 STEP/settings generation smoke certificate is missing"
            ],
        }
    try:
        document = _load_json(path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return {
            **base,
            "status": "INVALID_ADVISORY_GENERATION",
            "errors": [str(exc)],
        }
    errors: list[str] = []
    if document.get("schema") != OD90_CERTIFICATE_SCHEMA:
        errors.append(f"schema must be {OD90_CERTIFICATE_SCHEMA}")
    if document.get("case_id") != case.case_id:
        errors.append("advisory case_id mismatch")
    if document.get("status") != "PASS":
        errors.append("advisory generation status is not PASS")
    if document.get("advisory_only") is not True:
        errors.append("advisory_only must be true")
    if document.get("source_dependency_closure_complete") is not True:
        errors.append("source_dependency_closure_complete is not true")
    if document.get("production_authorized") is not False:
        errors.append("OD90 advisory may not authorize production")
    job = document.get("job")
    if not isinstance(job, Mapping):
        errors.append("advisory job is missing")
    else:
        errors.extend(_job_errors(
            job, case.expected_job(), prefix="advisory.job",
            require_turns=True,
        ))
    claimed = document.get("certificate_payload_sha256")
    calculated = _canonical_hash(
        document, omit=("certificate_payload_sha256",),
    )
    if claimed != calculated:
        errors.append("advisory certificate payload hash mismatch")
    errors.extend(_source_errors(root, document.get("sources")))
    artifacts = document.get("artifacts")
    paths: dict[str, Path | None] = {}
    actual_hashes: dict[str, str | None] = {}
    if not isinstance(artifacts, Mapping):
        errors.append("advisory artifacts must be an object")
    else:
        for key in ("step", "settings"):
            if key not in artifacts:
                errors.append(f"advisory artifacts.{key} is missing")
                continue
            row_errors, artifact_path, actual = _artifact_row_errors(
                root, path.parent, key, artifacts[key], case.expected_job(),
            )
            errors.extend(row_errors)
            paths[key] = artifact_path
            actual_hashes[key] = actual
    errors.extend(_semantic_artifact_errors(case, paths, actual_hashes))
    return {
        **base,
        "status": "ADVISORY_GENERATION_PASS" if not errors else
                  "INVALID_ADVISORY_GENERATION",
        "certificate_sha256": _sha256(path),
        "errors": errors,
    }


def certificate_contract() -> dict[str, Any]:
    return {
        "schema": CORNER_CERTIFICATE_SCHEMA,
        "blocked_evidence_schema": CORNER_EVIDENCE_SCHEMA,
        "bundle_location": (
            "out/launch_certificates/<case_id>/certificate.json"
        ),
        "self_hash_field": "certificate_payload_sha256",
        "self_hash_rule": (
            "SHA-256 of canonical sorted compact JSON excluding only "
            "certificate_payload_sha256"
        ),
        "required_top_level": {
            "status": "PASS",
            "corner_authorized": True,
            "production_authorized": False,
            "source_dependency_closure_complete": True,
        },
        "required_sources": list(REQUIRED_SOURCE_PATHS),
        "required_artifacts": list(REQUIRED_ARTIFACT_KEYS),
        "artifact_rule": (
            "each path is machine-root-relative, inside the case bundle, "
            "nonempty, exact-SHA-bound, and declares the exact job identity"
        ),
        "required_verdicts": REQUIRED_VERDICT_CONTRACT,
        "capture_binding": (
            "unmodified upstream capture, one meta row, cycle_complete row, "
            "exact job identity, and settings_sha256 equal to bundled settings"
        ),
        "blocked_evidence_rule": (
            "FAIL_CLOSED bundles may record current partial evidence, but must "
            "set both authority booleans false and list every non-PASS gate"
        ),
    }


def analyze(
    *,
    root: Path = ROOT,
    certificate_root: Path | None = None,
) -> dict[str, Any]:
    root = Path(root).resolve()
    certificate_root = (
        Path(certificate_root).resolve()
        if certificate_root is not None
        else (root / "out" / "launch_certificates").resolve()
    )
    try:
        certificate_root.relative_to(root)
    except ValueError as exc:
        raise ValueError("certificate_root must be inside machine root") from exc

    cases = required_launch_cases()
    rows = [
        _validate_corner_certificate(root, certificate_root, case)
        for case in cases
    ]
    passing = sum(row["status"] == "PASS" for row in rows)
    missing = sum(row["status"] == "MISSING_CERTIFICATE" for row in rows)
    blocked = sum(
        row["status"] == "EVIDENCE_BUNDLE_BLOCKED" for row in rows
    )
    invalid = len(rows) - passing - missing - blocked
    current_evidence = sum(
        row.get("certificate_current") is True for row in rows
    )
    all_required = passing == len(rows)
    existing_od46 = _existing_od46_evidence(root)
    od90 = _validate_od90_advisory(root, certificate_root)

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if all_required else "FAIL",
        "decision": (
            "LAUNCH_OD28_TO_OD65_EXACT_CORNER_AUTHORITY_COMPLETE"
            if all_required else
            "DO_NOT_CLAIM_OD28_TO_OD65_LAUNCH_SUPPORT"
        ),
        "production_authorized": bool(all_required),
        "scope": {
            "launch_od_mm": [28.0, 65.0],
            "launch_stack_mm": [5.0, 20.0],
            "wire_extremes_mm": [0.2, 0.5],
            "slots": 24,
            "workholding_coverage": [
                asdict(row) for row in WORKHOLDING_COVERAGE
            ],
            "authority_interpretation": (
                "exact launch-corner regression plus fail-closed exact per-job "
                "certification for interior jobs; endpoint declarations alone "
                "are not geometry authority"
            ),
        },
        "matrix_contract": {
            "geometry_corner_count": 4,
            "wire_extreme_count": 2,
            "workholding_case_count": len(WORKHOLDING_COVERAGE),
            "required_certificate_count": len(rows),
            "dimensions": (
                "OD(28,65) x stack(5,20) x wire(0.2,0.5) x "
                "workholding(er11-shaft3, er11-shaft7, shaft8-shaft8)"
            ),
        },
        "certificate_contract": certificate_contract(),
        "summary": {
            "required": len(rows),
            "passing": passing,
            "missing": missing,
            "invalid_or_stale": invalid,
        },
        "evidence_progress": {
            "current_exact_authority_certificates": passing,
            "current_fail_closed_evidence_bundles": blocked,
            "current_bundles_total": current_evidence,
            "missing_bundles": missing,
            "invalid_or_stale_bundles": invalid,
        },
        "release_gates": {
            "all_required_launch_corner_certificates_pass": all_required,
            "every_corner_binds_current_required_sources": (
                all_required and all(row["certificate_current"] for row in rows)
            ),
            "existing_OD46_not_substituted_for_launch_corner": True,
            "OD90_excluded_from_launch_authority": True,
            "all_launch_rows_have_current_evidence_bundles": (
                current_evidence == len(rows)
            ),
            "production_authorized": bool(all_required),
        },
        "required_certificates": rows,
        "actionable_missing_matrix": [
            {
                "case_id": row["case_id"],
                "certificate_path": row["certificate_path"],
                "status": row["status"],
                "errors": row["errors"],
            }
            for row in rows if row["status"] != "PASS"
        ],
        "existing_OD46_evidence": existing_od46,
        "OD90_advisory_generation": od90,
        "limits": [
            "OD46 evidence is reported for context and never counts toward the 24 launch rows.",
            "OD90 STEP/settings generation is advisory and never production authority.",
            "An interior OD/stack/wire job remains hardware-blocked until its own exact certificate passes.",
            "This isolated audit does not mutate the current CAD, job generators, release catalog, or controller source.",
            "A current FAIL_CLOSED evidence bundle is progress, not a corner-authority certificate.",
            "Launch rows prove case-bound representative feasible laminations, not every internal lamination geometry sharing an OD and shaft endpoint.",
            "Every real stator job must regenerate packing, slot access, and workholder reach from its measured lamination section.",
        ],
        "audit_source": {
            "path": "sim/launch_envelope_authority.py",
            "sha256": _sha256(Path(__file__)),
        },
    }
    report["report_payload_sha256"] = _canonical_hash(
        report, omit=("report_payload_sha256",),
    )
    return report


def render_markdown(report: Mapping[str, Any]) -> str:
    summary = report["summary"]
    progress = report["evidence_progress"]
    lines = [
        "# Launch-envelope authority",
        "",
        f"Status: **{report['status']}**",
        "",
        f"Decision: `{report['decision']}`",
        "",
        f"Production authorized: **{str(report['production_authorized']).lower()}**",
        "",
        (
            f"Required exact certificates: {summary['required']}; "
            f"PASS {summary['passing']}; missing {summary['missing']}; "
            f"invalid/stale {summary['invalid_or_stale']}."
        ),
        (
            "Current fail-closed evidence bundles: "
            f"{progress['current_fail_closed_evidence_bundles']}; "
            f"current bundles total {progress['current_bundles_total']}."
        ),
        "",
        "## Required launch matrix",
        "",
        "| case | OD | stack | wire | topology | spindle / shaft | status | action |",
        "|---|---:|---:|---:|---|---|---|---|",
    ]
    for row in report["required_certificates"]:
        job = row["expected_job"]
        action = (
            "current exact certificate"
            if row["status"] == "PASS" else
            "current evidence; close listed gates"
            if row["status"] == "EVIDENCE_BUNDLE_BLOCKED" else
            f"create/fix `{row['certificate_path']}`"
        )
        lines.append(
            f"| `{row['case_id']}` | {job['od_mm']:g} | "
            f"{job['stack_mm']:g} | {job['wire_finished_d_mm']:g} | "
            f"{job['slots']} slots; hub OD {job['hub_od_mm']:g} mm; "
            f"`{job['upstream_config_id']}`; "
            f"{job['reach_reserve_mm']:g} mm reserve | "
            f"{job['spindle_id']} / {job['shaft_d_mm']:g} mm | "
            f"{row['status']} | {action} |"
        )

    od46 = report["existing_OD46_evidence"]
    lines.extend([
        "",
        "## Existing OD46 evidence (non-gating)",
        "",
        f"Classification: `{od46['classification']}`.",
        "",
        od46["reason"] + ".",
        "",
        (
            "Capture inventory by OD: `"
            + json.dumps(od46["capture_inventory"]["by_od_mm"], sort_keys=True)
            + "`."
        ),
        "",
        "Current report statuses:",
        "",
    ])
    for name, status in od46["report_statuses"].items():
        lines.append(f"- `{name}`: `{status}`")

    od90 = report["OD90_advisory_generation"]
    lines.extend([
        "",
        "## OD90 advisory generation",
        "",
        f"Status: `{od90['status']}`.",
        "",
        (
            f"Expected non-gating generation certificate: "
            f"`{od90['certificate_path']}`."
        ),
        "",
        "OD90 never contributes to the OD28..65 launch-production gate.",
        "",
        "## Limits",
        "",
    ])
    lines.extend(f"- {item}" for item in report["limits"])
    lines.extend([
        "",
        f"Payload SHA-256: `{report['report_payload_sha256']}`",
        "",
    ])
    return "\n".join(lines)


def write_reports(
    report: Mapping[str, Any], json_out: Path = JSON_OUT,
    md_out: Path = MD_OUT,
) -> None:
    json_out = Path(json_out)
    md_out = Path(md_out)
    json_out.parent.mkdir(parents=True, exist_ok=True)
    md_out.parent.mkdir(parents=True, exist_ok=True)
    json_out.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    md_out.write_text(render_markdown(report), encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--machine-root", type=Path, default=ROOT)
    parser.add_argument("--certificate-root", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--md-out", type=Path)
    parser.add_argument(
        "--allow-fail", action="store_true",
        help="write the fail-closed report but return process status 0",
    )
    args = parser.parse_args(argv)
    machine_root = args.machine_root.resolve()
    report = analyze(
        root=machine_root,
        certificate_root=args.certificate_root,
    )
    json_out = args.json_out or (
        machine_root / "out" / "reports" / "launch_envelope_authority.json"
    )
    md_out = args.md_out or (
        machine_root / "out" / "reports" / "launch_envelope_authority.md"
    )
    write_reports(report, json_out, md_out)
    print(
        f"launch envelope: {report['status']}; "
        f"{report['summary']['passing']}/{report['summary']['required']} "
        "exact certificates PASS; "
        f"production_authorized={report['production_authorized']}"
    )
    return 0 if report["status"] == "PASS" or args.allow_fail else 1


if __name__ == "__main__":
    raise SystemExit(main())
