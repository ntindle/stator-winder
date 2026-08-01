"""Fail-closed acceptance gate for the aggregate-boundary R3 follower.

This gate consumes source-owned endpoint and route studies; it does not
create CAD, a strand schedule, or an interpolated conductor shape.  Current
artifacts prove 2,400 endpoint classifications and supporting aggregate
tangents, but they do not provide the physical R3 follower, the continuous
intra-half-turn law, swept clearance, or downstream-length coupling required
to promote those endpoint facts to moving-wire authority.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"

FOLLOWER_STUDY_PATH = REPORTS / "aggregate_boundary_follower_locus_study.json"
TERMINAL_LOCI_PATH = REPORTS / "carriage_active_sector_terminal_guide_loci.json"
GUIDE_AUDIT_PATH = REPORTS / "carriage_active_sector_terminal_guide_audit.json"
AGGREGATE_PATH = REPORTS / "permanent_cap_aggregate_authorization.json"
SUPPORT_TRADE_PATH = REPORTS / "cap_live_tail_manufactured_support_trade.json"
CONTINUITY_PATH = REPORTS / "moving_half_turn_segment_continuity_audit.json"
DANCER_PATH = REPORTS / "dancer_loads.json"
CAD_AUDIT_PATH = REPORTS / "aggregate_boundary_follower_cad_audit.json"
MOUNT_SCREEN_PATH = REPORTS / "aggregate_boundary_follower_mount_screen.json"
INTEGRATION_AUDIT_PATH = (
    REPORTS / "aggregate_boundary_follower_integration_audit.json"
)
RETRACTION_TOPOLOGY_PATH = (
    REPORTS / "aggregate_boundary_follower_retraction_topology.json"
)
REPLACEMENT_ARCHITECTURE_PATH = (
    REPORTS / "aggregate_boundary_follower_replacement_architecture.json"
)
REPLACEMENT_CAD_AUDIT_PATH = (
    REPORTS / "aggregate_boundary_follower_replacement_cad_audit.json"
)
RETRACTION_PROCUREMENT_PATH = (
    REPORTS / "aggregate_boundary_follower_retraction_procurement.json"
)
G0_NORMAL_AUDIT_PATH = (
    REPORTS / "aggregate_boundary_follower_g0_normal_audit.json"
)
G0_LANDING_TRADE_PATH = (
    REPORTS / "aggregate_boundary_follower_g0_landing_trade.json"
)
CUSTOM_RETURN_SCREEN_PATH = (
    REPORTS / "aggregate_boundary_follower_custom_return_screen.json"
)
PROTOTYPE_CAD_AUDIT_PATH = (
    REPORTS / "aggregate_boundary_follower_prototype_cad_audit.json"
)
ROUTE_SWEEP_PATH = (
    REPORTS / "aggregate_boundary_follower_route_sweep.json"
)
REPLACEMENT_TRANSITION_SWEEP_PATH = (
    REPORTS / "aggregate_boundary_follower_replacement_transition_sweep.json"
)
REPLACEMENT_LOAD_WEAR_PATH = (
    REPORTS / "aggregate_boundary_follower_replacement_load_wear.json"
)
C1_REBOUND_SWEEP_PATH = (
    REPORTS / "aggregate_boundary_follower_c1_rebound_sweep.json"
)
PLACEMENT_TRADE_PATH = (
    REPORTS / "aggregate_boundary_follower_placement_trade.json"
)
SUCCESSOR_PROTOTYPE_SOURCE_PATH = (
    ROOT / "cad" / "aggregate_boundary_follower_successor_prototype.py"
)
SUCCESSOR_PROTOTYPE_STEP_PATH = (
    ROOT / "out" / "review"
    / "aggregate_boundary_follower_successor_prototype.step"
)
SUCCESSOR_PROTOTYPE_MANIFEST_PATH = (
    ROOT / "out" / "review"
    / "aggregate_boundary_follower_successor_prototype_manifest.json"
)
SUCCESSOR_PROTOTYPE_AUDIT_PATH = (
    REPORTS / "aggregate_boundary_follower_successor_prototype_audit.json"
)
SUCCESSOR_PLACEMENT_COLLISION_AUDIT_SOURCE_PATH = (
    HERE
    / "aggregate_boundary_follower_successor_prototype_placement_collision_audit.py"
)
SUCCESSOR_PLACEMENT_COLLISION_AUDIT_PATH = (
    REPORTS
    / "aggregate_boundary_follower_successor_prototype_placement_collision_audit.json"
)
EXPECTED_SUCCESSOR_PLACEMENT_COLLISION_AUDIT_SOURCE_SHA256 = (
    "1484275a2cbaf16a4163714a90bee90dde66c470ebb601facadc75ba2ecf8b2c"
)

OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_acceptance.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_acceptance.md"

SCHEMA = "aggregate-boundary-follower-acceptance/v14"
EXPECTED_LOCI = 2400
EXPECTED_G0_LOCI = 48
EXPECTED_INTERVALS = 2400


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any], field: str = "report_sha256") -> str:
    body = deepcopy(dict(value))
    body.pop(field, None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _canonical_value_hash(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _self_hash_ok(value: Mapping[str, Any]) -> bool:
    expected = value.get("report_sha256")
    return isinstance(expected, str) and expected == _canonical_hash(value)


def _source_path(relative: Any) -> Path:
    """Resolve the two source roots used by the current report family."""

    name = Path(str(relative).replace("\\", "/"))
    local = ROOT / name
    if local.is_file():
        return local
    # The normal project GOAL is one directory above ``machine``; the
    # aggregate authority intentionally hashes it as the short name GOAL.md.
    parent = ROOT.parent / name
    return parent if parent.is_file() else local


def _source_hashes_current(value: Mapping[str, Any]) -> bool:
    rows = value.get("source_hashes")
    if not isinstance(rows, Mapping) or not rows:
        return False
    for relative, expected in rows.items():
        path = _source_path(relative)
        if not path.is_file() or _sha256(path) != expected:
            return False
    return True


def _recorded_path_hashes_current(value: Mapping[str, Any]) -> bool:
    """Validate every embedded ``path``/``sha256`` evidence row.

    The isolated prototype audit intentionally binds generated sources,
    manifests, reports, and STEP artifacts instead of exposing a flat
    ``source_hashes`` table.  Walk those rows generically so acceptance fails
    closed on any later artifact drift.
    """

    checked = 0

    def visit(node: Any) -> bool:
        nonlocal checked
        if isinstance(node, Mapping):
            path_value = node.get("path")
            digest = node.get("sha256")
            if isinstance(path_value, str) and isinstance(digest, str):
                path = _source_path(path_value)
                checked += 1
                if not path.is_file() or _sha256(path) != digest:
                    return False
            return all(visit(child) for child in node.values())
        if isinstance(node, list):
            return all(visit(child) for child in node)
        return True

    return visit(value) and checked > 0


def _successor_prototype_sources_current(
    value: Mapping[str, Any],
    *,
    source_path: Path = SUCCESSOR_PROTOTYPE_SOURCE_PATH,
    step_path: Path = SUCCESSOR_PROTOTYPE_STEP_PATH,
    manifest_path: Path = SUCCESSOR_PROTOTYPE_MANIFEST_PATH,
    placement_path: Path = PLACEMENT_TRADE_PATH,
) -> bool:
    """Independently bind the isolated successor source, STEP, and manifest."""

    files = {
        "source": Path(source_path),
        "step": Path(step_path),
        "manifest": Path(manifest_path),
        "placement": Path(placement_path),
    }
    expected = value.get("input_hashes")
    if not isinstance(expected, Mapping) or set(expected) != set(files):
        return False
    if any(
        not path.is_file() or _sha256(path) != expected.get(name)
        for name, path in files.items()
    ):
        return False

    try:
        manifest = _load(files["manifest"])
    except (OSError, ValueError, json.JSONDecodeError):
        return False
    artifacts = manifest.get("artifacts")
    placement = manifest.get("placement_trade")
    authority = manifest.get("authority")
    if not all(isinstance(row, Mapping) for row in (
        artifacts, placement, authority,
    )):
        return False
    assert isinstance(artifacts, Mapping)
    assert isinstance(placement, Mapping)
    assert isinstance(authority, Mapping)
    return (
        manifest.get("status")
        == "PASS_ISOLATED_POSITIVE_VOLUME_TOPOLOGY_PROTOTYPE"
        and artifacts.get("source_sha256") == expected["source"]
        and artifacts.get("step_sha256") == expected["step"]
        and artifacts.get("step_exists") is True
        and artifacts.get("step_size_bytes") == files["step"].stat().st_size
        and placement.get("file_sha256") == expected["placement"]
        and authority.get("isolated_review_only") is True
        and all(
            authority.get(key) is False
            for key in authority
            if key != "isolated_review_only"
        )
    )


def _successor_placement_collision_sources_current(
    value: Mapping[str, Any],
    *,
    audit_source_path: Path = SUCCESSOR_PLACEMENT_COLLISION_AUDIT_SOURCE_PATH,
    prototype_source_path: Path = SUCCESSOR_PROTOTYPE_SOURCE_PATH,
    prototype_step_path: Path = SUCCESSOR_PROTOTYPE_STEP_PATH,
    prototype_manifest_path: Path = SUCCESSOR_PROTOTYPE_MANIFEST_PATH,
    placement_path: Path = PLACEMENT_TRADE_PATH,
) -> bool:
    """Freeze the audit implementation and every path in its input hash chain."""

    audit_source = Path(audit_source_path)
    if (
        not audit_source.is_file()
        or _sha256(audit_source)
        != EXPECTED_SUCCESSOR_PLACEMENT_COLLISION_AUDIT_SOURCE_SHA256
    ):
        return False
    files = {
        "prototype_source": Path(prototype_source_path),
        "prototype_STEP": Path(prototype_step_path),
        "prototype_manifest": Path(prototype_manifest_path),
        "placement_trade": Path(placement_path),
    }
    expected = value.get("input_hashes")
    return (
        isinstance(expected, Mapping)
        and set(expected) == set(files)
        and all(
            path.is_file() and _sha256(path) == expected.get(name)
            for name, path in files.items()
        )
    )


def _cad_audit_sources_current(value: Mapping[str, Any]) -> bool:
    evidence = value.get("source_evidence")
    if not isinstance(evidence, Mapping):
        return False
    for path_key, hash_key in (
        ("cad_source", "cad_source_sha256"),
        ("cad_brief", "cad_brief_sha256"),
    ):
        relative = evidence.get(path_key)
        expected = evidence.get(hash_key)
        if not isinstance(relative, str) or not isinstance(expected, str):
            return False
        path = ROOT / Path(relative.replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            return False
    step = evidence.get("step")
    if not isinstance(step, Mapping) or step.get("exists") is not True:
        return False
    relative = step.get("path")
    expected = step.get("sha256")
    if not isinstance(relative, str) or not isinstance(expected, str):
        return False
    path = ROOT / Path(relative.replace("\\", "/"))
    return (
        path.is_file()
        and _sha256(path) == expected
        and step.get("matches_inspected_authoritative_sha256") is True
    )


def _mount_screen_sources_current(value: Mapping[str, Any]) -> bool:
    evidence = value.get("source_evidence")
    if not isinstance(evidence, Mapping):
        return False
    for path_key, hash_key in (
        ("cad_source", "cad_source_sha256"),
        ("cad_audit", "cad_audit_sha256"),
    ):
        relative = evidence.get(path_key)
        expected = evidence.get(hash_key)
        if not isinstance(relative, str) or not isinstance(expected, str):
            return False
        path = ROOT / Path(relative.replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            return False
    return True


def _integration_audit_sources_current(value: Mapping[str, Any]) -> bool:
    if not _source_hashes_current(value):
        return False
    artifacts = value.get("artifacts")
    if not isinstance(artifacts, Mapping) or not artifacts:
        return False
    for row in artifacts.values():
        if not isinstance(row, Mapping):
            return False
        relative = row.get("path")
        expected = row.get("sha256")
        if not isinstance(relative, str) or not isinstance(expected, str):
            return False
        path = ROOT / Path(relative.replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            return False
    return True


def _retraction_topology_sources_current(value: Mapping[str, Any]) -> bool:
    rows = value.get("source_bindings")
    if not isinstance(rows, Mapping) or not rows:
        return False
    for relative, expected in rows.items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            return False
    return True


def _endpoint_binding(
    study: Mapping[str, Any], loci_document: Mapping[str, Any],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    study_rows = study.get("loci", [])
    locus_rows = loci_document.get("loci", [])
    bindings: list[dict[str, Any]] = []
    mismatches: list[dict[str, Any]] = []
    if not isinstance(study_rows, list) or not isinstance(locus_rows, list):
        return bindings, [{"code": "endpoint_arrays_missing"}]
    for index in range(max(len(study_rows), len(locus_rows))):
        if index >= len(study_rows) or index >= len(locus_rows):
            mismatches.append({
                "code": "endpoint_array_length_mismatch",
                "locus_index": index,
            })
            continue
        follower = study_rows[index]
        guide = locus_rows[index]
        terminal = guide.get("terminal_binding", {})
        fields = {
            "locus_index": (follower.get("locus_index"), guide.get("locus_index")),
            "pass_index": (follower.get("pass_index"), guide.get("pass_index")),
            "state_index": (follower.get("state_index"), guide.get("state_index")),
            "turn_index": (follower.get("turn_index"), guide.get("turn_index")),
            "half_turn_index": (
                follower.get("half_turn_index"), guide.get("half_turn_index")
            ),
            "tooth_index": (follower.get("tooth_index"), guide.get("tooth_index")),
            "lane_id": (follower.get("lane_id"), terminal.get("lane_id")),
            "time_s": (follower.get("time_s"), guide.get("time_s")),
        }
        bad = {}
        for name, (left, right) in fields.items():
            if name == "time_s":
                try:
                    equal = abs(float(left) - float(right)) <= 1.0e-9
                except (TypeError, ValueError):
                    equal = False
            else:
                equal = left == right
            if not equal:
                bad[name] = {"study": left, "terminal_locus": right}
        row = {
            "locus_index": index,
            "pass_index": guide.get("pass_index"),
            "state_index": guide.get("state_index"),
            "turn_index": guide.get("turn_index"),
            "half_turn_index": guide.get("half_turn_index"),
            "tooth_index": guide.get("tooth_index"),
            "time_s": guide.get("time_s"),
            "lane_id": terminal.get("lane_id"),
            "cap_endpoint_local_mm": terminal.get("cap_endpoint_local_mm"),
            "contact_owner": follower.get("contact_owner"),
            "g_current": follower.get("g_current"),
            "support_candidate_count": follower.get("support_candidate_count"),
            "direct_straight_span_C1": follower.get("direct_straight_span_C1"),
        }
        bindings.append(row)
        if bad:
            mismatches.append({
                "code": "endpoint_identity_mismatch",
                "locus_index": index,
                "fields": bad,
            })
    return bindings, mismatches


def _g0_blockers(
    bindings: list[dict[str, Any]], g0_normal_audit: Mapping[str, Any],
) -> list[dict[str, Any]]:
    unsupported = {
        int(row["locus_index"])
        for row in g0_normal_audit.get("coverage", {}).get("loci", [])
        if row.get("existing_positive_BREP_owner") is not True
    }
    return [{
        "code": "g0_right_seam_positive_BREP_normal_missing",
        "locus_index": row["locus_index"],
        "pass_index": row["pass_index"],
        "state_index": row["state_index"],
        "turn_index": row["turn_index"],
        "half_turn_index": row["half_turn_index"],
        "tooth_index": row["tooth_index"],
        "time_s": row["time_s"],
        "lane_id": row["lane_id"],
        "cap_endpoint_local_mm": row["cap_endpoint_local_mm"],
        "required_owner": "Nomex liner or permanent-cap positive BREP surface",
        "reason": (
            "the post-cut right PEEK seam remains 0.022496245 mm beyond the "
            "current wire radius; the constructive landing is not integrated "
            "or tolerance/force/route/collision qualified"
        ),
    } for row in bindings
      if float(row.get("g_current") or 0.0) == 0.0
      and int(row["locus_index"]) in unsupported]


def analyze(
    *,
    follower_study_path: Path = FOLLOWER_STUDY_PATH,
    terminal_loci_path: Path = TERMINAL_LOCI_PATH,
    guide_audit_path: Path = GUIDE_AUDIT_PATH,
    aggregate_path: Path = AGGREGATE_PATH,
    support_trade_path: Path = SUPPORT_TRADE_PATH,
    continuity_path: Path = CONTINUITY_PATH,
    dancer_path: Path = DANCER_PATH,
    cad_audit_path: Path = CAD_AUDIT_PATH,
    mount_screen_path: Path = MOUNT_SCREEN_PATH,
    integration_audit_path: Path = INTEGRATION_AUDIT_PATH,
    retraction_topology_path: Path = RETRACTION_TOPOLOGY_PATH,
    replacement_architecture_path: Path = REPLACEMENT_ARCHITECTURE_PATH,
    replacement_cad_audit_path: Path = REPLACEMENT_CAD_AUDIT_PATH,
    retraction_procurement_path: Path = RETRACTION_PROCUREMENT_PATH,
    g0_normal_audit_path: Path = G0_NORMAL_AUDIT_PATH,
    g0_landing_trade_path: Path = G0_LANDING_TRADE_PATH,
    custom_return_screen_path: Path = CUSTOM_RETURN_SCREEN_PATH,
    prototype_cad_audit_path: Path = PROTOTYPE_CAD_AUDIT_PATH,
    route_sweep_path: Path = ROUTE_SWEEP_PATH,
    replacement_transition_sweep_path: Path = REPLACEMENT_TRANSITION_SWEEP_PATH,
    replacement_load_wear_path: Path = REPLACEMENT_LOAD_WEAR_PATH,
    c1_rebound_sweep_path: Path = C1_REBOUND_SWEEP_PATH,
    placement_trade_path: Path = PLACEMENT_TRADE_PATH,
    successor_prototype_source_path: Path = SUCCESSOR_PROTOTYPE_SOURCE_PATH,
    successor_prototype_step_path: Path = SUCCESSOR_PROTOTYPE_STEP_PATH,
    successor_prototype_manifest_path: Path = SUCCESSOR_PROTOTYPE_MANIFEST_PATH,
    successor_prototype_audit_path: Path = SUCCESSOR_PROTOTYPE_AUDIT_PATH,
    successor_placement_collision_audit_source_path: Path = (
        SUCCESSOR_PLACEMENT_COLLISION_AUDIT_SOURCE_PATH
    ),
    successor_placement_collision_audit_path: Path = (
        SUCCESSOR_PLACEMENT_COLLISION_AUDIT_PATH
    ),
) -> dict[str, Any]:
    paths = {
        "follower_locus_study": Path(follower_study_path),
        "terminal_loci": Path(terminal_loci_path),
        "guide_audit": Path(guide_audit_path),
        "aggregate_authority": Path(aggregate_path),
        "manufactured_support_trade": Path(support_trade_path),
        "moving_segment_continuity": Path(continuity_path),
        "dancer_static_model": Path(dancer_path),
        "follower_cad_audit": Path(cad_audit_path),
        "follower_mount_screen": Path(mount_screen_path),
        "follower_integration_audit": Path(integration_audit_path),
        "follower_retraction_topology": Path(retraction_topology_path),
        "follower_replacement_architecture": Path(
            replacement_architecture_path
        ),
        "follower_replacement_cad_audit": Path(
            replacement_cad_audit_path
        ),
        "follower_retraction_procurement": Path(retraction_procurement_path),
        "follower_g0_normal_audit": Path(g0_normal_audit_path),
        "follower_g0_landing_trade": Path(g0_landing_trade_path),
        "follower_custom_return_screen": Path(custom_return_screen_path),
        "follower_prototype_cad_audit": Path(prototype_cad_audit_path),
        "follower_route_sweep": Path(route_sweep_path),
        "follower_replacement_transition_sweep": Path(
            replacement_transition_sweep_path
        ),
        "follower_replacement_load_wear": Path(replacement_load_wear_path),
        "follower_C1_rebound_sweep": Path(c1_rebound_sweep_path),
        "follower_placement_trade": Path(placement_trade_path),
        "follower_successor_prototype_audit": Path(
            successor_prototype_audit_path
        ),
        "follower_successor_placement_collision_audit": Path(
            successor_placement_collision_audit_path
        ),
    }
    successor_prototype_artifact_paths = {
        "source": Path(successor_prototype_source_path),
        "step": Path(successor_prototype_step_path),
        "manifest": Path(successor_prototype_manifest_path),
        "audit": Path(successor_prototype_audit_path),
    }
    successor_placement_collision_artifact_paths = {
        "audit_source": Path(successor_placement_collision_audit_source_path),
        "audit": Path(successor_placement_collision_audit_path),
        "prototype_source": Path(successor_prototype_source_path),
        "prototype_STEP": Path(successor_prototype_step_path),
        "prototype_manifest": Path(successor_prototype_manifest_path),
        "placement_trade": Path(placement_trade_path),
    }
    values = {name: _load(path) for name, path in paths.items()}
    study = values["follower_locus_study"]
    loci_document = values["terminal_loci"]
    guide = values["guide_audit"]
    aggregate = values["aggregate_authority"]
    trade = values["manufactured_support_trade"]
    continuity = values["moving_segment_continuity"]
    dancer = values["dancer_static_model"]
    cad_audit = values["follower_cad_audit"]
    mount_screen = values["follower_mount_screen"]
    integration_audit = values["follower_integration_audit"]
    retraction_topology = values["follower_retraction_topology"]
    replacement_architecture = values["follower_replacement_architecture"]
    replacement_cad_audit = values["follower_replacement_cad_audit"]
    retraction_procurement = values["follower_retraction_procurement"]
    g0_normal_audit = values["follower_g0_normal_audit"]
    g0_landing_trade = values["follower_g0_landing_trade"]
    custom_return_screen = values["follower_custom_return_screen"]
    prototype_cad_audit = values["follower_prototype_cad_audit"]
    route_sweep = values["follower_route_sweep"]
    replacement_transition_sweep = values[
        "follower_replacement_transition_sweep"
    ]
    replacement_load_wear = values["follower_replacement_load_wear"]
    c1_rebound_sweep = values["follower_C1_rebound_sweep"]
    placement_trade = values["follower_placement_trade"]
    successor_prototype_audit = values["follower_successor_prototype_audit"]
    successor_placement_collision_audit = values[
        "follower_successor_placement_collision_audit"
    ]

    bindings, endpoint_mismatches = _endpoint_binding(study, loci_document)
    g0_rows = _g0_blockers(bindings, g0_normal_audit)
    nonzero = [row for row in bindings if float(row.get("g_current") or 0.0) > 0.0]
    nonzero_without_tangent = [
        row["locus_index"] for row in nonzero
        if int(row.get("support_candidate_count") or 0) <= 0
    ]
    direct_c1_failures = [
        row["locus_index"] for row in nonzero
        if row.get("direct_straight_span_C1") is not True
    ]

    selected = trade.get("recommended_successor", {})
    selected_gates = selected.get("gates", {}) if isinstance(selected, Mapping) else {}
    prototype = trade.get("prototype", {})
    positive_r3_cad = (
        _self_hash_ok(cad_audit)
        and _cad_audit_sources_current(cad_audit)
        and cad_audit.get("positive_volume_R3_prototype_geometry_proven") is True
        and all(cad_audit.get("geometry_gates", {}).values())
    )
    mechanism_complete = cad_audit.get("mechanism_complete") is True
    integration_evidence_current = (
        _self_hash_ok(integration_audit)
        and _integration_audit_sources_current(integration_audit)
    )
    replacement_carriage_integration_authorized = (
        integration_evidence_current
        and integration_audit.get("status") == "PASS"
        and integration_audit.get("assembly_integration_authorized") is True
        and integration_audit.get("collision_authorized") is True
    )
    retraction_analysis_closed = (
        _self_hash_ok(retraction_topology)
        and _retraction_topology_sources_current(retraction_topology)
        and bool(retraction_topology.get("analysis_gates"))
        and all(retraction_topology.get("analysis_gates", {}).values())
    )
    physical_retraction_integrated = (
        retraction_analysis_closed
        and retraction_topology.get("physical_authority") is True
        and bool(retraction_topology.get("physical_authority_gates"))
        and all(retraction_topology.get("physical_authority_gates", {}).values())
    )
    replacement_architecture_bound = (
        _self_hash_ok(replacement_architecture)
        and _source_hashes_current(replacement_architecture)
        and bool(replacement_architecture.get("analysis_gates"))
        and all(replacement_architecture.get("analysis_gates", {}).values())
        and replacement_architecture.get("exact_install_counts", {}).get(
            "physical_follower_occurrences"
        ) == 4
        and replacement_architecture.get("exact_install_counts", {}).get(
            "shared_U_windowed_replacement_carrier"
        ) == 1
    )
    replacement_cad_static_bound = (
        _self_hash_ok(replacement_cad_audit)
        and replacement_cad_audit.get("static_CAD_geometry_proven") is True
        and bool(replacement_cad_audit.get("proof_gates"))
        and all(replacement_cad_audit.get("proof_gates", {}).values())
        and all(
            row.get("exists") is True
            and row.get("matches_inspected_sha256") is True
            for row in replacement_cad_audit.get(
                "artifact_binding", {}
            ).values()
        )
        and replacement_cad_audit.get("leaf_accounting", {}).get(
            "STEP_review_leaf_count"
        ) == 73
        and replacement_cad_audit.get("leaf_accounting", {}).get(
            "manufactured_leaf_count"
        ) == 69
        and replacement_cad_audit.get("state_pair_audit", {}).get(
            "state_count"
        ) == 36
        and replacement_cad_audit.get("state_pair_audit", {}).get(
            "unique_geometry_signature_count"
        ) == 5
        and replacement_cad_audit.get("state_pair_audit", {}).get(
            "all_scopes_zero_positive"
        ) is True
    )
    retraction_procurement_evidence_current = (
        _self_hash_ok(retraction_procurement)
        and _retraction_topology_sources_current(retraction_procurement)
        and bool(retraction_procurement.get("evidence_gates"))
        and all(retraction_procurement.get("evidence_gates", {}).values())
    )
    retraction_hardware_releasable = (
        retraction_procurement_evidence_current
        and retraction_procurement.get("physical_procurement_authority") is True
        and retraction_procurement.get("order_authorized") is True
        and bool(retraction_procurement.get("fail_closed_gates"))
        and all(retraction_procurement.get("fail_closed_gates", {}).values())
    )
    g0_normal_evidence_current = (
        _self_hash_ok(g0_normal_audit)
        and _integration_audit_sources_current(g0_normal_audit)
        and g0_normal_audit.get("coverage", {}).get(
            "classified_g0_locus_count"
        ) == EXPECTED_G0_LOCI
        and g0_normal_audit.get("coverage", {}).get(
            "existing_positive_BREP_owner_count"
        ) == 24
        and g0_normal_audit.get("coverage", {}).get("unsupported_count") == 24
    )
    g0_landing_trade_current = (
        _self_hash_ok(g0_landing_trade)
        and _integration_audit_sources_current(g0_landing_trade)
        and g0_landing_trade.get("geometry_gates")
        and all(g0_landing_trade.get("geometry_gates", {}).values())
    )
    g0_landing_and_range_route_integrated = (
        g0_landing_trade_current
        and g0_landing_trade.get("status") == "PASS"
        and g0_landing_trade.get("wire_route_authorized") is True
        and g0_landing_trade.get("collision_authorized") is True
        and bool(g0_landing_trade.get("release_gates"))
        and all(g0_landing_trade.get("release_gates", {}).values())
    )
    custom_return_screen_current = (
        _self_hash_ok(custom_return_screen)
        and _retraction_topology_sources_current(custom_return_screen)
        and bool(custom_return_screen.get("evidence_gates"))
        and all(custom_return_screen.get("evidence_gates", {}).values())
    )
    custom_return_qualified = (
        custom_return_screen_current
        and custom_return_screen.get("physical_authority") is True
        and custom_return_screen.get("CAD_authority") is True
        and bool(custom_return_screen.get("fail_closed_gates"))
        and all(custom_return_screen.get("fail_closed_gates", {}).values())
    )
    prototype_cad_evidence_current = (
        _self_hash_ok(prototype_cad_audit)
        and _recorded_path_hashes_current(prototype_cad_audit)
        and prototype_cad_audit.get("status")
        == "PASS_ISOLATED_PROTOTYPE_GEOMETRY_ONLY_NO_AUTHORITY"
        and bool(prototype_cad_audit.get("evidence_gates"))
        and all(prototype_cad_audit.get("evidence_gates", {}).values())
    )
    g0_shelf_isolated_cad_bound = (
        prototype_cad_evidence_current
        and prototype_cad_audit.get(
            "aggregate_boundary_g0_cap_shelf", {}
        ).get("status") == "PASS_ESTABLISHED_LOCAL_GEOMETRY_ONLY"
        and all(prototype_cad_audit.get(
            "aggregate_boundary_g0_cap_shelf", {}
        ).get("evidence_gates", {}).values())
    )
    custom_return_isolated_cad_bound = (
        prototype_cad_evidence_current
        and prototype_cad_audit.get(
            "aggregate_boundary_follower_custom_return_packaging", {}
        ).get("status") == "PASS_ESTABLISHED_LOCAL_GEOMETRY_ONLY"
        and all(prototype_cad_audit.get(
            "aggregate_boundary_follower_custom_return_packaging", {}
        ).get("evidence_gates", {}).values())
    )
    route_sweep_analytic_classification_bound = (
        _self_hash_ok(route_sweep)
        and _source_hashes_current(route_sweep)
        and _recorded_path_hashes_current(route_sweep)
        and route_sweep.get("status") == "FAIL"
        and bool(route_sweep.get("analytic_gates"))
        and all(route_sweep.get("analytic_gates", {}).values())
        and route_sweep.get("coverage", {}).get("evaluated_loci")
        == EXPECTED_LOCI
        and route_sweep.get("coverage", {}).get(
            "diameter_route_case_count"
        ) == 2 * EXPECTED_LOCI
        and route_sweep.get("coverage", {}).get(
            "physically_authorized_route_case_count"
        ) == 0
        and route_sweep.get("wire_route_authorized") is False
        and not any(route_sweep.get("physical_gates", {}).values())
    )
    replacement_transition_evidence_current = (
        _self_hash_ok(replacement_transition_sweep)
        and _source_hashes_current(replacement_transition_sweep)
    )
    replacement_transition_geometry_bound = (
        replacement_transition_evidence_current
        and replacement_transition_sweep.get("status")
        == "PASS_SAMPLED_GEOMETRY_ONLY"
        and replacement_transition_sweep.get("sampling", {}).get(
            "total_pose_count"
        ) == 232
        and bool(replacement_transition_sweep.get("sampling", {}).get("gates"))
        and all(
            replacement_transition_sweep.get("sampling", {})
            .get("gates", {})
            .values()
        )
        and replacement_transition_sweep.get("sampled_geometry_result", {}).get(
            "sampling_contract_passes"
        ) is True
        and replacement_transition_sweep.get("sampled_geometry_result", {}).get(
            "collision_samples_zero"
        ) is True
        and replacement_transition_sweep.get("sampled_geometry_result", {}).get(
            "clearance_gate_passes"
        ) is True
        and replacement_transition_sweep.get("collision_audit", {}).get(
            "positive_failure_count"
        ) == 0
        and replacement_transition_sweep.get("clearance_audit", {}).get(
            "passes_2p00mm_gate"
        ) is True
        and replacement_transition_sweep.get("clearance_audit", {}).get(
            "minimum_sampled_exact_clearance_mm", 0.0
        ) >= 2.0
        and not any(replacement_transition_sweep.get("authority", {}).values())
    )
    replacement_transition_physical_authority = (
        replacement_transition_evidence_current
        and replacement_transition_sweep.get("status") == "PASS"
        and bool(replacement_transition_sweep.get("authority"))
        and all(
            replacement_transition_sweep.get("authority", {}).get(key) is True
            for key in (
                "clearance_authorized",
                "continuous_tolerance_stack_proven",
                "retraction_mechanism_authorized",
                "selection_mechanism_authorized",
                "transition_collision_authorized",
                "assembly_integration_authorized",
            )
        )
    )
    replacement_load_wear_evidence_current = (
        _self_hash_ok(replacement_load_wear)
        and _recorded_path_hashes_current(replacement_load_wear)
    )
    replacement_load_wear_analytic_bound = (
        replacement_load_wear_evidence_current
        and replacement_load_wear.get("status") == "FAIL"
        and bool(replacement_load_wear.get("analytical_gates"))
        and all(replacement_load_wear.get("analytical_gates", {}).values())
        and bool(replacement_load_wear.get("qualification_gates"))
        and not any(replacement_load_wear.get("qualification_gates", {}).values())
        and bool(replacement_load_wear.get("authority"))
        and not any(replacement_load_wear.get("authority", {}).values())
    )
    replacement_load_wear_qualified = (
        replacement_load_wear_evidence_current
        and replacement_load_wear.get("status") == "PASS"
        and bool(replacement_load_wear.get("qualification_gates"))
        and all(replacement_load_wear.get("qualification_gates", {}).values())
        and all(
            replacement_load_wear.get("authority", {}).get(key) is True
            for key in (
                "load_authorized", "fatigue_authorized", "wear_authorized",
                "retention_authorized", "tolerance_authorized",
                "assembly_integration_authorized",
            )
        )
    )
    c1_rebound_evidence_current = (
        _self_hash_ok(c1_rebound_sweep)
        and _source_hashes_current(c1_rebound_sweep)
        and _recorded_path_hashes_current(c1_rebound_sweep)
    )
    c1_rebound_analytic_bound = (
        c1_rebound_evidence_current
        and c1_rebound_sweep.get("status") == "FAIL"
        and bool(c1_rebound_sweep.get("analytic_gates"))
        and all(c1_rebound_sweep.get("analytic_gates", {}).values())
        and c1_rebound_sweep.get("coverage", {}).get(
            "nonzero_attempted_case_count"
        ) == 2 * (EXPECTED_LOCI - EXPECTED_G0_LOCI)
        and c1_rebound_sweep.get("coverage", {}).get(
            "analytic_C1_biarc_pass_case_count"
        ) == 2 * (EXPECTED_LOCI - EXPECTED_G0_LOCI)
        and c1_rebound_sweep.get("coverage", {}).get(
            "positive_volume_placed_case_count"
        ) == 0
        and c1_rebound_sweep.get("coverage", {}).get(
            "physically_authorized_case_count"
        ) == 0
        and bool(c1_rebound_sweep.get("physical_gates"))
        and not any(c1_rebound_sweep.get("physical_gates", {}).values())
    )
    c1_rebound_physical_authority = (
        c1_rebound_evidence_current
        and c1_rebound_sweep.get("status") == "PASS"
        and c1_rebound_sweep.get("wire_route_authorized") is True
        and c1_rebound_sweep.get("collision_authorized") is True
        and c1_rebound_sweep.get("assembly_integration_authorized") is True
        and bool(c1_rebound_sweep.get("physical_gates"))
        and all(c1_rebound_sweep.get("physical_gates", {}).values())
    )
    placement_trade_evidence_current = (
        _self_hash_ok(placement_trade)
        and _source_hashes_current(placement_trade)
        and _recorded_path_hashes_current(placement_trade)
    )
    placement_trade_analytic_bound = (
        placement_trade_evidence_current
        and placement_trade.get("status") == "FAIL"
        and bool(placement_trade.get("analytic_gates"))
        and all(placement_trade.get("analytic_gates", {}).values())
        and placement_trade.get("coverage", {}).get(
            "compared_nonzero_cases"
        ) == 2 * (EXPECTED_LOCI - EXPECTED_G0_LOCI)
        and placement_trade.get("coverage", {}).get(
            "current_CAD_full_center_covered_case_count"
        ) == 0
        and placement_trade.get("coverage", {}).get(
            "successor_analytic_center_covered_case_count"
        ) == 2 * (EXPECTED_LOCI - EXPECTED_G0_LOCI)
        and bool(placement_trade.get("physical_gates"))
        and not any(placement_trade.get("physical_gates", {}).values())
    )
    placement_successor_physical_authority = (
        placement_trade_evidence_current
        and placement_trade.get("status") == "PASS"
        and placement_trade.get("wire_route_authorized") is True
        and placement_trade.get("collision_authorized") is True
        and placement_trade.get("assembly_integration_authorized") is True
        and bool(placement_trade.get("physical_gates"))
        and all(placement_trade.get("physical_gates", {}).values())
    )
    successor_prototype_evidence_current = (
        _self_hash_ok(successor_prototype_audit)
        and _successor_prototype_sources_current(
            successor_prototype_audit,
            source_path=successor_prototype_artifact_paths["source"],
            step_path=successor_prototype_artifact_paths["step"],
            manifest_path=successor_prototype_artifact_paths["manifest"],
            placement_path=paths["follower_placement_trade"],
        )
    )
    successor_prototype_geometry_bound = (
        successor_prototype_evidence_current
        and successor_prototype_audit.get("status") == "PASS"
        and successor_prototype_audit.get("decision")
        == "ISOLATED_TOPOLOGY_PROTOTYPE_ONLY__NO_ASSEMBLY_OR_PHYSICAL_AUTHORITY"
        and bool(successor_prototype_audit.get("checks"))
        and all(successor_prototype_audit.get("checks", {}).values())
        and successor_prototype_audit.get("evidence", {}).get(
            "STEP_size_bytes"
        ) == successor_prototype_artifact_paths["step"].stat().st_size
        and successor_prototype_audit.get("evidence", {}).get(
            "stage", {}
        ).get("count") == 4
        and successor_prototype_audit.get("evidence", {}).get(
            "guide", {}
        ).get("count") == 4
        and successor_prototype_audit.get("evidence", {}).get(
            "preload", {}
        ).get("leaf_count") == 4
        and successor_prototype_audit.get("evidence", {}).get(
            "preload", {}
        ).get("shoe_count") == 4
        and successor_prototype_audit.get("evidence", {}).get(
            "guide", {}
        ).get("all_4704_case_surface_proved") is False
        and bool(successor_prototype_audit.get("authority"))
        and not any(successor_prototype_audit.get("authority", {}).values())
    )
    successor_prototype_route_and_motion_bound = (
        successor_prototype_geometry_bound
        and successor_prototype_audit.get("evidence", {}).get(
            "guide", {}
        ).get("all_4704_case_surface_proved") is True
        and successor_prototype_audit.get("authority", {}).get(
            "wire_route_authorized"
        ) is True
        and successor_prototype_audit.get("authority", {}).get(
            "dynamics_authorized"
        ) is True
    )
    successor_prototype_collision_authorized = (
        successor_prototype_geometry_bound
        and successor_prototype_audit.get("authority", {}).get(
            "collision_authorized"
        ) is True
    )
    successor_prototype_load_buildability_qualified = (
        successor_prototype_geometry_bound
        and successor_prototype_audit.get("authority", {}).get(
            "load_authorized"
        ) is True
        and successor_prototype_audit.get("authority", {}).get(
            "buildability_authorized"
        ) is True
    )
    successor_placement_collision_evidence_current = (
        _self_hash_ok(successor_placement_collision_audit)
        and _successor_placement_collision_sources_current(
            successor_placement_collision_audit,
            audit_source_path=(
                successor_placement_collision_artifact_paths["audit_source"]
            ),
            prototype_source_path=(
                successor_placement_collision_artifact_paths[
                    "prototype_source"
                ]
            ),
            prototype_step_path=(
                successor_placement_collision_artifact_paths["prototype_STEP"]
            ),
            prototype_manifest_path=(
                successor_placement_collision_artifact_paths[
                    "prototype_manifest"
                ]
            ),
            placement_path=(
                successor_placement_collision_artifact_paths["placement_trade"]
            ),
        )
        and bool(successor_placement_collision_audit.get("authority"))
        and not any(
            successor_placement_collision_audit.get("authority", {}).values()
        )
    )
    successor_placement_coverage = successor_placement_collision_audit.get(
        "analytic_all_4704_case_coverage", {}
    )
    successor_direct_floor = successor_placement_collision_audit.get(
        "direct_all_4704_guide_to_floor_BREP", {}
    )
    successor_sampled_BREP = successor_placement_collision_audit.get(
        "sampled_endpoint_BREP", {}
    )
    successor_analytic_center_range_bound = (
        successor_placement_collision_evidence_current
        and successor_placement_collision_audit.get("status")
        == "PASS_AUDIT__PROTOTYPE_NOT_PLACEMENT_OR_COLLISION_READY"
        and successor_placement_coverage.get("case_count") == 4704
        and successor_placement_coverage.get(
            "exact_identity_center_bounds_covered_case_count"
        ) == 4704
        and successor_placement_coverage.get(
            "modeled_1p50x2p40x1p10_center_travel_covered_case_count"
        ) == 4704
        and successor_placement_coverage.get(
            "numeric_yaw_elevation_range_covered_case_count"
        ) == 4704
    )
    successor_realized_tangent_all_cases = (
        successor_placement_collision_evidence_current
        and successor_placement_coverage.get(
            "prototype_Rot_realized_tangent_match_case_count"
        ) == 4704
    )
    successor_full_2mm_relief_all_cases = (
        successor_placement_collision_evidence_current
        and successor_placement_coverage.get(
            "full_2mm_R3_to_fixed_R5_relief_margin_case_count"
        ) == 4704
    )
    successor_guide_floor_collision_zero = (
        successor_placement_collision_evidence_current
        and successor_direct_floor.get("case_count") == 4704
        and successor_direct_floor.get("zero_positive_common_volume_case_count")
        == 4704
        and successor_direct_floor.get("positive_common_volume_case_count") == 0
        and successor_direct_floor.get("kernel_exception_count") == 0
    )
    successor_sampled_self_collision_zero = (
        successor_placement_collision_evidence_current
        and successor_sampled_BREP.get("self_collision", {}).get(
            "unique_positive_pair_count"
        ) == 0
        and successor_sampled_BREP.get("self_collision", {}).get(
            "positive_collision_evaluation_count"
        ) == 0
        and successor_sampled_BREP.get("self_collision", {}).get(
            "kernel_exception_count"
        ) == 0
    )
    successor_sampled_floor_collision_zero = (
        successor_placement_collision_evidence_current
        and successor_sampled_BREP.get("own_floor_leaf_collision", {}).get(
            "unique_positive_pair_count"
        ) == 0
        and successor_sampled_BREP.get("own_floor_leaf_collision", {}).get(
            "positive_collision_evaluation_count"
        ) == 0
        and successor_sampled_BREP.get("own_floor_leaf_collision", {}).get(
            "kernel_exception_count"
        ) == 0
    )
    successor_exact_local_sibling_collision_zero = (
        successor_placement_collision_evidence_current
        and successor_sampled_BREP.get(
            "exact_active_local_rebased_sibling_collision", {}
        ).get("unique_positive_pair_count") == 0
        and successor_sampled_BREP.get(
            "exact_active_local_rebased_sibling_collision", {}
        ).get("positive_collision_evaluation_count") == 0
        and successor_sampled_BREP.get(
            "exact_active_local_rebased_sibling_collision", {}
        ).get("kernel_exception_count") == 0
    )
    successor_prototype_physical_authority = (
        placement_successor_physical_authority
        and successor_prototype_route_and_motion_bound
        and successor_realized_tangent_all_cases
        and successor_full_2mm_relief_all_cases
        and successor_guide_floor_collision_zero
        and successor_sampled_self_collision_zero
        and successor_sampled_floor_collision_zero
        and successor_exact_local_sibling_collision_zero
        and successor_prototype_collision_authorized
        and successor_prototype_load_buildability_qualified
        and successor_prototype_audit.get("authority", {}).get(
            "assembly_integration_authorized"
        ) is True
        and successor_prototype_audit.get("authority", {}).get(
            "production_authorized"
        ) is True
    )
    r3_route_closure = (
        positive_r3_cad
        and mechanism_complete
        and cad_audit.get("wire_route_authorized") is True
        and isinstance(selected_gates, Mapping)
        and selected_gates.get("all_2400_raw_pose_route_and_rigid_clearance") is True
    )

    continuity_gates = continuity.get("gates", {})
    continuity_coverage = continuity.get("coverage", {})
    exact_continuous_law = (
        continuity.get("physical_authority_status") == "PROVEN"
        and continuity_gates.get("physical_quasistatic_moving_interval_authorized") is True
        and continuity_coverage.get("required_half_turn_intervals") == EXPECTED_INTERVALS
        and continuity_coverage.get("proved_adjacent_C0_interval_count") == EXPECTED_INTERVALS
    )
    adaptive_swept_clearance = (
        exact_continuous_law
        and continuity_gates.get("moving_rigid_and_prior_copper_clearance_proven") is True
        and continuity_gates.get("moving_contact_tail_ownership_proven") is True
    )

    dancer_result = dancer.get("recommended", {}).get("result", {})
    dancer_static_model_available = (
        dancer_result.get("ok") is True
        and isinstance(dancer_result.get("angle_sweep"), list)
        and bool(dancer_result.get("angle_sweep"))
    )
    downstream_length_history_bound = False
    dancer_static_coupling = (
        dancer_static_model_available and downstream_length_history_bound
    )
    dancer_limitations = dancer.get("method", {}).get("limitations", [])
    dancer_dynamic_authority = False

    guide_route_bound = (
        guide.get("player_route_api", {}).get("locus_count") == EXPECTED_LOCI
        and guide.get("player_route_api", {}).get("compact_file_sha256")
        == _sha256(paths["terminal_loci"])
        and guide.get("release_gates", {}).get(
            "all_2400_physical_bell_terminal_routes_pass"
        ) is True
        and guide.get("release_gates", {}).get(
            "all_2400_short_leadin_endpoints_join_actual_cap_lane_BREP"
        ) is True
    )

    gates = {
        "predecessor_self_hashes_valid": all(
            _self_hash_ok(values[name]) for name in (
                "follower_locus_study", "guide_audit", "aggregate_authority",
                "manufactured_support_trade", "moving_segment_continuity",
                "follower_cad_audit",
                "follower_mount_screen",
                "follower_integration_audit",
                "follower_retraction_topology",
                "follower_replacement_architecture",
                "follower_replacement_cad_audit",
                "follower_retraction_procurement",
                "follower_g0_normal_audit",
                "follower_g0_landing_trade",
                "follower_custom_return_screen",
                "follower_prototype_cad_audit",
                "follower_route_sweep",
                "follower_replacement_transition_sweep",
                "follower_replacement_load_wear",
                "follower_C1_rebound_sweep",
                "follower_placement_trade",
                "follower_successor_prototype_audit",
                "follower_successor_placement_collision_audit",
            )
        ),
        "predecessor_source_hashes_current": all(
            _source_hashes_current(values[name]) for name in (
                "follower_locus_study", "guide_audit", "aggregate_authority",
                "manufactured_support_trade", "moving_segment_continuity",
            )
        ) and _cad_audit_sources_current(cad_audit)
        and _mount_screen_sources_current(mount_screen)
        and _integration_audit_sources_current(integration_audit)
        and _retraction_topology_sources_current(retraction_topology)
        and _source_hashes_current(replacement_architecture)
        and all(
            row.get("exists") is True
            and row.get("matches_inspected_sha256") is True
            for row in replacement_cad_audit.get(
                "artifact_binding", {}
            ).values()
        )
        and _retraction_topology_sources_current(retraction_procurement)
        and _integration_audit_sources_current(g0_normal_audit)
        and _integration_audit_sources_current(g0_landing_trade)
        and _retraction_topology_sources_current(custom_return_screen)
        and _recorded_path_hashes_current(prototype_cad_audit)
        and _source_hashes_current(replacement_transition_sweep)
        and _recorded_path_hashes_current(replacement_load_wear)
        and _source_hashes_current(c1_rebound_sweep)
        and _recorded_path_hashes_current(c1_rebound_sweep)
        and _source_hashes_current(placement_trade)
        and _recorded_path_hashes_current(placement_trade)
        and _successor_prototype_sources_current(
            successor_prototype_audit,
            source_path=successor_prototype_artifact_paths["source"],
            step_path=successor_prototype_artifact_paths["step"],
            manifest_path=successor_prototype_artifact_paths["manifest"],
            placement_path=paths["follower_placement_trade"],
        )
        and _successor_placement_collision_sources_current(
            successor_placement_collision_audit,
            audit_source_path=(
                successor_placement_collision_artifact_paths["audit_source"]
            ),
            prototype_source_path=(
                successor_placement_collision_artifact_paths[
                    "prototype_source"
                ]
            ),
            prototype_step_path=(
                successor_placement_collision_artifact_paths["prototype_STEP"]
            ),
            prototype_manifest_path=(
                successor_placement_collision_artifact_paths[
                    "prototype_manifest"
                ]
            ),
            placement_path=(
                successor_placement_collision_artifact_paths["placement_trade"]
            ),
        ),
        "route_sweep_sources_and_artifacts_current": (
            _source_hashes_current(route_sweep)
            and _recorded_path_hashes_current(route_sweep)
        ),
        "g0_normal_audit_current_and_exactly_classified": (
            g0_normal_evidence_current
        ),
        "g0_robust_landing_trade_current": g0_landing_trade_current,
        "g0_PEEK_shelf_isolated_CAD_geometry_bound": (
            g0_shelf_isolated_cad_bound
        ),
        "g0_PEEK_shelf_and_0p65mm_cap_lane_integrated": (
            g0_landing_and_range_route_integrated
        ),
        "custom_return_concepts_screened": custom_return_screen_current,
        "custom_return_isolated_CAD_geometry_bound": (
            custom_return_isolated_cad_bound
        ),
        "all_2400_loci_x_two_diameters_analytic_route_classified": (
            route_sweep_analytic_classification_bound
        ),
        "all_4704_nonzero_routes_have_exact_analytic_C1_biarcs": (
            c1_rebound_analytic_bound
        ),
        "positive_volume_C1_route_and_normal_preload_authorized": (
            c1_rebound_physical_authority
        ),
        "successor_placement_trade_exactly_bound": (
            placement_trade_analytic_bound
        ),
        "successor_prototype_source_STEP_manifest_audit_current": (
            successor_prototype_evidence_current
        ),
        "successor_isolated_positive_volume_prototype_geometry_bound": (
            successor_prototype_geometry_bound
        ),
        "successor_placement_collision_audit_source_and_paths_current": (
            successor_placement_collision_evidence_current
        ),
        "successor_analytic_center_and_numeric_range_coverage_bound": (
            successor_analytic_center_range_bound
        ),
        "successor_realized_tangent_matches_all_4704_cases": (
            successor_realized_tangent_all_cases
        ),
        "successor_full_2mm_R3_relief_margin_all_4704_cases": (
            successor_full_2mm_relief_all_cases
        ),
        "successor_modeled_guides_zero_positive_to_floor_all_4704": (
            successor_guide_floor_collision_zero
        ),
        "successor_sampled_endpoint_self_collision_zero": (
            successor_sampled_self_collision_zero
        ),
        "successor_sampled_endpoint_floor_collision_zero": (
            successor_sampled_floor_collision_zero
        ),
        "successor_exact_active_local_sibling_collision_zero": (
            successor_exact_local_sibling_collision_zero
        ),
        "successor_prototype_all_4704_routes_and_full_motion_bound": (
            successor_prototype_route_and_motion_bound
        ),
        "successor_prototype_collision_sweep_authorized": (
            successor_prototype_collision_authorized
        ),
        "successor_prototype_load_tolerance_buildability_qualified": (
            successor_prototype_load_buildability_qualified
        ),
        "successor_redatumed_stage_positive_volume_and_integrated": (
            successor_prototype_physical_authority
        ),
        "custom_return_hardware_CAD_and_endurance_qualified": (
            custom_return_qualified
        ),
        "replacement_architecture_identity_and_counts_bound": (
            replacement_architecture_bound
        ),
        "replacement_carriage_static_CAD_and_zero_overlap_bound": (
            replacement_cad_static_bound
        ),
        "replacement_carriage_sampled_transition_geometry_bound": (
            replacement_transition_geometry_bound
        ),
        "replacement_carriage_transition_physical_authority": (
            replacement_transition_physical_authority
        ),
        "replacement_load_wear_analytical_screens_bound": (
            replacement_load_wear_analytic_bound
        ),
        "replacement_load_wear_physical_qualification_complete": (
            replacement_load_wear_qualified
        ),
        "retraction_procurement_evidence_current": (
            retraction_procurement_evidence_current
        ),
        "retraction_hardware_stack_selected_and_releasable": (
            retraction_hardware_releasable
        ),
        "exact_2400_endpoint_identity_and_terminal_binding": (
            len(bindings) == EXPECTED_LOCI and not endpoint_mismatches
        ),
        "source_owned_physical_terminal_route_bound": guide_route_bound,
        "aggregate_authority_PASS_without_strand_order": (
            aggregate.get("status") == "PASS"
            and aggregate.get("aggregate_geometry_authorized") is True
            and aggregate.get("aggregate_loft", {}).get(
                "exact_strand_packing_predicted"
            ) is False
        ),
        "all_nonzero_growth_endpoints_have_supporting_tangent": (
            len(nonzero) == EXPECTED_LOCI - EXPECTED_G0_LOCI
            and not nonzero_without_tangent
        ),
        "all_48_g0_cap_or_liner_normals_source_owned": len(g0_rows) == 0,
        "positive_volume_R3_follower_CAD_provenance": positive_r3_cad,
        "R3_follower_mechanism_complete": mechanism_complete,
        "retraction_topology_analysis_closed": retraction_analysis_closed,
        "positive_retraction_and_actual_position_interlock_integrated": (
            physical_retraction_integrated
        ),
        "replacement_carriage_integration_and_collision_authorized": (
            replacement_carriage_integration_authorized
        ),
        "eccentric_40N_mount_load_path_qualified": (
            mount_screen.get("status") == "PASS"
            and all(mount_screen.get("release_gates", {}).values())
        ),
        "R3_route_closes_every_direct_C1_mismatch": r3_route_closure,
        "exact_continuous_intra_half_turn_follower_law": exact_continuous_law,
        "adaptive_transition_swept_rigid_core_cap_prior_self_clearance": (
            adaptive_swept_clearance
        ),
        "downstream_length_history_bound_to_raw_intervals": (
            downstream_length_history_bound
        ),
        "dancer_static_equilibrium_coupled_to_downstream_length": (
            dancer_static_coupling
        ),
        "dynamic_dancer_and_flyer_acceleration_authority": (
            dancer_dynamic_authority
        ),
    }

    interval_indices = list(range(EXPECTED_INTERVALS))
    blockers: list[dict[str, Any]] = [*g0_rows]
    if not positive_r3_cad:
        blockers.append({
            "code": "positive_volume_R3_follower_CAD_provenance_missing",
            "affected_locus_count": len(direct_c1_failures),
            "affected_locus_indices": direct_c1_failures,
            "trade_candidate_id": selected.get("id") if isinstance(selected, Mapping) else None,
            "trade_status": selected.get("status") if isinstance(selected, Mapping) else None,
            "prototype_created": prototype.get("created") if isinstance(prototype, Mapping) else None,
            "required_evidence": (
                "positive-volume R3 follower CAD, source hash, attachment datum, "
                "surface owner, and exact route/clearance report"
            ),
        })
    if not mechanism_complete:
        blockers.append({
            "code": "positive_volume_R3_follower_mechanism_incomplete",
            "affected_locus_count": len(direct_c1_failures),
            "affected_locus_indices": direct_c1_failures,
            "CAD_audit_decision": cad_audit.get("decision"),
            "mechanism_blockers": cad_audit.get("mechanism_blockers", []),
            "reason": (
                "positive-volume R3 geometry exists, but incomplete attachment, "
                "preload, retraction, and bearing owners forbid route authority"
            ),
        })
    if gates["eccentric_40N_mount_load_path_qualified"] is not True:
        blockers.append({
            "code": "eccentric_40N_mount_load_path_unqualified",
            "affected_locus_count": EXPECTED_LOCI,
            "mount_screen_decision": mount_screen.get("decision"),
            "radial_case": mount_screen.get("load_cases", {}).get(
                "radial_X_40N"
            ),
            "mount_blockers": mount_screen.get("blockers", []),
        })
    if not physical_retraction_integrated:
        blockers.append({
            "code": "positive_retraction_and_actual_position_interlock_unintegrated",
            "affected_locus_count": EXPECTED_LOCI,
            "topology_status": retraction_topology.get("status"),
            "analysis_gates": retraction_topology.get("analysis_gates", {}),
            "physical_authority_gates": retraction_topology.get(
                "physical_authority_gates", {}
            ),
            "reason": (
                "the radial/tangential/M0 equations close, but their CAD, "
                "selected hardware, positive gimbal dock, dual-NC circuit, "
                "collision sweep, and fault injection remain absent"
            ),
        })
    if not replacement_carriage_integration_authorized:
        blockers.append({
            "code": "replacement_carriage_integration_missing",
            "affected_locus_count": EXPECTED_LOCI,
            "integration_decision": integration_audit.get("decision"),
            "reference_positive_pair_count": integration_audit.get(
                "reference_pose_OCC_scan", {}
            ).get("positive_pair_count"),
            "additive_integration_feasible": integration_audit.get(
                "additive_integration_feasible"
            ),
            "minimum_implementation_plan": integration_audit.get(
                "minimum_implementation_plan", []
            ),
            "replacement_architecture_report_sha256": (
                replacement_architecture.get("report_sha256")
            ),
            "replacement_static_CAD_bound": replacement_cad_static_bound,
            "replacement_CAD_audit_report_sha256": (
                replacement_cad_audit.get("report_sha256")
            ),
            "replacement_STEP_sha256": replacement_cad_audit.get(
                "artifact_binding", {}
            ).get("step", {}).get("sha256"),
            "static_state_pair_audit": replacement_cad_audit.get(
                "state_pair_audit"
            ),
            "remaining_replacement_CAD_blockers": (
                replacement_cad_audit.get("open_blockers", [])
            ),
            "coarse_selection_stroke_mm": replacement_architecture.get(
                "travel", {}
            ).get("coarse_selection_stroke_mm"),
            "sampled_transition_geometry_bound": (
                replacement_transition_geometry_bound
            ),
            "sampled_transition_report_sha256": (
                replacement_transition_sweep.get("report_sha256")
            ),
        })
    if not replacement_transition_physical_authority:
        blockers.append({
            "code": "replacement_carriage_transition_physical_authority_open",
            "sampled_geometry_bound": replacement_transition_geometry_bound,
            "report_sha256": replacement_transition_sweep.get(
                "report_sha256"
            ),
            "sampled_pose_count": replacement_transition_sweep.get(
                "sampling", {}
            ).get("total_pose_count"),
            "minimum_sampled_exact_clearance_mm": (
                replacement_transition_sweep.get("clearance_audit", {}).get(
                    "minimum_sampled_exact_clearance_mm"
                )
            ),
            "authority": replacement_transition_sweep.get("authority"),
            "reason": (
                "232 nominal BREP poses are clear, but the selection and "
                "retraction mechanisms, continuous tolerance stack, and "
                "assembly integration are not physically authorized"
            ),
        })
    if not replacement_load_wear_qualified:
        blockers.append({
            "code": "replacement_load_wear_physical_qualification_incomplete",
            "analytical_screens_bound": replacement_load_wear_analytic_bound,
            "report_sha256": replacement_load_wear.get("report_sha256"),
            "load_envelope": replacement_load_wear.get("load_envelope"),
            "qualification_gates": replacement_load_wear.get(
                "qualification_gates"
            ),
            "authority": replacement_load_wear.get("authority"),
        })
    if not c1_rebound_physical_authority:
        blockers.append({
            "code": "analytic_C1_biarcs_not_positive_volume_or_compression_compatible",
            "analytic_C1_biarcs_bound": c1_rebound_analytic_bound,
            "report_sha256": c1_rebound_sweep.get("report_sha256"),
            "coverage": c1_rebound_sweep.get("coverage"),
            "bounds": c1_rebound_sweep.get("bounds"),
            "physical_gates": c1_rebound_sweep.get("physical_gates"),
            "reason": (
                "all 4,704 nonzero diameter cases close analytically, but no "
                "positive-volume guide is placed and the circular arc center "
                "does not provide aggregate-normal compression"
            ),
        })
    if not successor_prototype_physical_authority:
        blockers.append({
            "code": "successor_prototype_realized_placement_and_collision_failures",
            "analytic_trade_bound": placement_trade_analytic_bound,
            "placement_collision_audit_bound": (
                successor_placement_collision_evidence_current
            ),
            "placement_collision_audit_report_sha256": (
                successor_placement_collision_audit.get("report_sha256")
            ),
            "isolated_positive_volume_prototype_bound": (
                successor_prototype_geometry_bound
            ),
            "successor_prototype_audit_report_sha256": (
                successor_prototype_audit.get("report_sha256")
            ),
            "analytic_case_count": successor_placement_coverage.get(
                "case_count"
            ),
            "center_covered_case_count": successor_placement_coverage.get(
                "modeled_1p50x2p40x1p10_center_travel_covered_case_count"
            ),
            "numeric_range_covered_case_count": successor_placement_coverage.get(
                "numeric_yaw_elevation_range_covered_case_count"
            ),
            "realized_tangent_match_case_count": successor_placement_coverage.get(
                "prototype_Rot_realized_tangent_match_case_count"
            ),
            "realized_tangent_error_deg": successor_placement_coverage.get(
                "prototype_Rot_tangent_error_deg"
            ),
            "full_2mm_relief_margin_case_count": successor_placement_coverage.get(
                "full_2mm_R3_to_fixed_R5_relief_margin_case_count"
            ),
            "minimum_R3_to_R5_remaining_radial_margin_mm": (
                successor_placement_coverage.get(
                    "minimum_R3_to_R5_remaining_radial_margin_mm"
                )
            ),
            "guide_floor_all_case_counts": {
                key: successor_direct_floor.get(key)
                for key in (
                    "case_count", "zero_positive_common_volume_case_count",
                    "positive_common_volume_case_count",
                    "kernel_exception_count",
                )
            },
            "sampled_pose_count": successor_sampled_BREP.get(
                "sampling_contract", {}
            ).get("total_identity_pose_count"),
            "sampled_self_collision_counts": {
                key: successor_sampled_BREP.get("self_collision", {}).get(key)
                for key in (
                    "positive_collision_evaluation_count",
                    "unique_positive_pair_count", "kernel_exception_count",
                )
            },
            "sampled_floor_collision_counts": {
                key: successor_sampled_BREP.get(
                    "own_floor_leaf_collision", {}
                ).get(key)
                for key in (
                    "positive_collision_evaluation_count",
                    "unique_positive_pair_count", "kernel_exception_count",
                )
            },
            "exact_local_sibling_collision_counts": {
                key: successor_sampled_BREP.get(
                    "exact_active_local_rebased_sibling_collision", {}
                ).get(key)
                for key in (
                    "positive_collision_evaluation_count",
                    "unique_positive_pair_count", "kernel_exception_count",
                )
            },
            "selected_successor_topology": placement_trade.get(
                "successor_trade", {}
            ).get("selected_topology"),
            "minimum_successor_center_strokes_XYZ_mm": placement_trade.get(
                "successor_trade", {}
            ).get("common_exact_minimum_center_strokes_XYZ_mm"),
            "carrier_host_screen": placement_trade.get("carrier_host_screen"),
            "audit_evidence_checks": successor_placement_collision_audit.get(
                "evidence_checks"
            ),
            "audit_blocking_findings": successor_placement_collision_audit.get(
                "blocking_findings"
            ),
            "prototype_authority": successor_prototype_audit.get("authority"),
            "reason": (
                "all 4,704 center and numeric command ranges fit, but the "
                "prototype Euler transform realizes 0 requested tangents, "
                "0 cases retain the full 2 mm relief margin, 12 guide/floor "
                "cases intersect, and sampled self, floor, and exact-local "
                "sibling collisions remain positive"
            ),
        })
    if not retraction_hardware_releasable:
        blockers.append({
            "code": "retraction_hardware_procurement_incomplete",
            "affected_locus_count": EXPECTED_LOCI,
            "procurement_status": retraction_procurement.get("status"),
            "shaft_candidate": retraction_procurement.get("shaft", {}).get(
                "selected_purchase_candidate", {}
            ).get("catalog_number"),
            "bushing_candidate": retraction_procurement.get("bushing", {}).get(
                "selected_candidate", {}
            ).get("catalog_number"),
            "fail_closed_gates": retraction_procurement.get(
                "fail_closed_gates", {}
            ),
            "reason": (
                "the shaft needs cutting, the bushing needs a new pocket, "
                "both stock centering springs and the stock constant-force "
                "spring are rejected, and the direct-opening switches require "
                "a remote positive transfer"
            ),
        })
    if not g0_landing_and_range_route_integrated:
        selected_landing = next((
            row for row in g0_landing_trade.get("candidates", [])
            if row.get("selected") is True
        ), {})
        blockers.append({
            "code": "g0_robust_PEEK_shelf_and_wire_range_route_unintegrated",
            "affected_locus_count": len(g0_rows),
            "trade_decision": g0_landing_trade.get("decision"),
            "selected_candidate": selected_landing.get("id"),
            "shelf_dimensions_mm": selected_landing.get(
                "shelf_dimensions_mm"
            ),
            "mouth_dimensions_mm": selected_landing.get(
                "mouth_dimensions_mm"
            ),
            "current_cap_lane_clear_width_mm": g0_landing_trade.get(
                "inputs", {}
            ).get("current_cap_lane_clear_width_mm"),
            "required_cap_lane_clear_width_mm": 0.65,
            "isolated_cap_shelf_CAD_bound": g0_shelf_isolated_cad_bound,
            "prototype_CAD_report_sha256": prototype_cad_audit.get(
                "report_sha256"
            ),
            "isolated_cap_shelf_STEP_sha256": prototype_cad_audit.get(
                "aggregate_boundary_g0_cap_shelf", {}
            ).get("artifact", {}).get("sha256"),
            "release_gates": g0_landing_trade.get("release_gates"),
        })
    if not custom_return_qualified:
        blockers.append({
            "code": "custom_return_hardware_unintegrated_and_unqualified",
            "affected_locus_count": EXPECTED_LOCI,
            "screen_status": custom_return_screen.get("status"),
            "preferred_tangential_concept": custom_return_screen.get(
                "recommendation", {}
            ).get("tangential_primary"),
            "bounded_radial_concept": custom_return_screen.get(
                "recommendation", {}
            ).get("independent_radial_return"),
            "isolated_return_package_CAD_bound": (
                custom_return_isolated_cad_bound
            ),
            "prototype_CAD_report_sha256": prototype_cad_audit.get(
                "report_sha256"
            ),
            "isolated_return_STEP_sha256": prototype_cad_audit.get(
                "aggregate_boundary_follower_custom_return_packaging", {}
            ).get("artifact", {}).get("sha256"),
            "fail_closed_gates": custom_return_screen.get(
                "fail_closed_gates", {}
            ),
        })
    blockers.extend([
        {
            "code": "exact_continuous_intra_half_turn_follower_law_missing",
            "affected_interval_count": EXPECTED_INTERVALS,
            "affected_interval_indices": interval_indices,
            "available_C0_paired_interval_count": continuity_coverage.get(
                "proved_adjacent_C0_interval_count"
            ),
            "available_evidence_authority": continuity.get("physical_authority_status"),
            "route_sweep_analytic_classification_bound": (
                route_sweep_analytic_classification_bound
            ),
            "route_sweep_report_sha256": route_sweep.get("report_sha256"),
            "diameter_route_case_count": route_sweep.get(
                "coverage", {}
            ).get("diameter_route_case_count"),
            "direct_C1_case_count": route_sweep.get(
                "coverage", {}
            ).get("nonzero_direct_C1_case_count"),
            "reason": (
                "affine endpoint homotopy is mathematical C0 evidence only; "
                "it is not a source-owned follower law and may not pass through "
                "the aggregate interior"
            ),
        },
        {
            "code": "adaptive_transition_swept_clearance_missing",
            "affected_interval_count": EXPECTED_INTERVALS,
            "affected_interval_indices": interval_indices,
            "required_classes": [
                "final rigid parts", "core", "cap", "prior aggregate", "self",
            ],
            "exact_FCL_or_equivalent_narrow_phase_run": False,
            "route_sweep_physical_gates": route_sweep.get(
                "physical_gates"
            ),
        },
        {
            "code": "downstream_length_and_dancer_static_coupling_missing",
            "affected_interval_count": EXPECTED_INTERVALS,
            "affected_interval_indices": interval_indices,
            "dancer_static_model_available": dancer_static_model_available,
            "downstream_length_history_bound": downstream_length_history_bound,
            "required_conservation_law": (
                "delta dancer stored length = delta feed - delta downstream route length"
            ),
            "available_chord_only_length_proxy": route_sweep.get(
                "turn_and_length_bounds", {}
            ).get("length_proxy"),
        },
        {
            "code": "dynamic_authority_forbidden_by_current_evidence",
            "affected_interval_count": EXPECTED_INTERVALS,
            "dancer_limitations": dancer_limitations,
            "reason": (
                "the current dancer study is quasi-static and explicitly excludes "
                "transient damping and flyer acceleration"
            ),
        },
    ])
    for mismatch in endpoint_mismatches:
        blockers.append({"code": "endpoint_binding_mismatch", **mismatch})
    if nonzero_without_tangent:
        blockers.append({
            "code": "nonzero_growth_support_tangent_missing",
            "affected_locus_indices": nonzero_without_tangent,
        })

    passed = all(gates.values())
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if passed else "FAIL",
        "decision": (
            "AGGREGATE_BOUNDARY_FOLLOWER_ACCEPTED"
            if passed else "ENDPOINT_FAMILY_BOUND__MOVING_PHYSICAL_FOLLOWER_NOT_ACCEPTED"
        ),
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "dynamic_authority": "FAIL",
        "scope": {
            "isolated_simulation_gate_only": True,
            "CAD_prototype_consumed": True,
            "release_modified": False,
            "deterministic_strand_order_used": False,
            "linear_interpolation_through_aggregate_interior_allowed": False,
            "straight_taut_free_spans_allowed_when_endpoint_reacted_C0_C1_clear_and_tensioned": True,
        },
        "coverage": {
            "required_endpoint_loci": EXPECTED_LOCI,
            "bound_endpoint_loci": len(bindings),
            "endpoint_binding_mismatch_count": len(endpoint_mismatches),
            "g0_blocker_count": len(g0_rows),
            "nonzero_growth_endpoint_count": len(nonzero),
            "nonzero_growth_support_tangent_count": (
                len(nonzero) - len(nonzero_without_tangent)
            ),
            "direct_C1_mismatch_locus_count": len(direct_c1_failures),
            "required_moving_intervals": EXPECTED_INTERVALS,
            "physically_authorized_moving_intervals": 0,
            "replacement_sampled_transition_pose_count": (
                replacement_transition_sweep.get("sampling", {}).get(
                    "total_pose_count"
                )
            ),
            "analytic_C1_biarc_case_count": c1_rebound_sweep.get(
                "coverage", {}
            ).get("analytic_C1_biarc_pass_case_count"),
            "current_CAD_C1_center_covered_case_count": placement_trade.get(
                "coverage", {}
            ).get("current_CAD_full_center_covered_case_count"),
            "successor_analytic_C1_center_covered_case_count": (
                placement_trade.get("coverage", {}).get(
                    "successor_analytic_center_covered_case_count"
                )
            ),
            "successor_isolated_prototype_stage_count": (
                successor_prototype_audit.get("evidence", {}).get(
                    "stage", {}
                ).get("count")
            ),
            "successor_prototype_4704_case_surface_proved": (
                successor_prototype_audit.get("evidence", {}).get(
                    "guide", {}
                ).get("all_4704_case_surface_proved")
            ),
            "successor_realized_tangent_match_case_count": (
                successor_placement_coverage.get(
                    "prototype_Rot_realized_tangent_match_case_count"
                )
            ),
            "successor_full_2mm_relief_margin_case_count": (
                successor_placement_coverage.get(
                    "full_2mm_R3_to_fixed_R5_relief_margin_case_count"
                )
            ),
            "successor_guide_floor_positive_case_count": (
                successor_direct_floor.get("positive_common_volume_case_count")
            ),
            "successor_sampled_self_positive_pair_count": (
                successor_sampled_BREP.get("self_collision", {}).get(
                    "unique_positive_pair_count"
                )
            ),
            "successor_sampled_floor_positive_pair_count": (
                successor_sampled_BREP.get("own_floor_leaf_collision", {}).get(
                    "unique_positive_pair_count"
                )
            ),
            "successor_exact_local_sibling_positive_pair_count": (
                successor_sampled_BREP.get(
                    "exact_active_local_rebased_sibling_collision", {}
                ).get("unique_positive_pair_count")
            ),
        },
        "endpoint_binding_sha256": _canonical_value_hash(bindings),
        "g0_blocker_locus_indices": [row["locus_index"] for row in g0_rows],
        "g0_blockers": g0_rows,
        "g0_normal_evidence": {
            "schema": g0_normal_audit.get("schema"),
            "status": g0_normal_audit.get("status"),
            "decision": g0_normal_audit.get("decision"),
            "report_sha256": g0_normal_audit.get("report_sha256"),
            "existing_positive_BREP_owner_count": g0_normal_audit.get(
                "coverage", {}
            ).get("existing_positive_BREP_owner_count"),
            "unsupported_count": g0_normal_audit.get("coverage", {}).get(
                "unsupported_count"
            ),
            "right_unsupported_gap_mm": g0_normal_audit.get(
                "coverage", {}
            ).get("right_unsupported_gap_mm"),
            "release_gates": g0_normal_audit.get("release_gates"),
        },
        "g0_landing_trade_evidence": {
            "schema": g0_landing_trade.get("schema"),
            "status": g0_landing_trade.get("status"),
            "decision": g0_landing_trade.get("decision"),
            "report_sha256": g0_landing_trade.get("report_sha256"),
            "trade_current": g0_landing_trade_current,
            "integrated_and_range_route_authorized": (
                g0_landing_and_range_route_integrated
            ),
            "inputs": g0_landing_trade.get("inputs"),
            "release_gates": g0_landing_trade.get("release_gates"),
        },
        "custom_return_evidence": {
            "schema": custom_return_screen.get("schema"),
            "status": custom_return_screen.get("status"),
            "report_sha256": custom_return_screen.get("report_sha256"),
            "screen_current": custom_return_screen_current,
            "hardware_qualified": custom_return_qualified,
            "recommendation": custom_return_screen.get("recommendation"),
            "requirements": custom_return_screen.get("requirements"),
            "fail_closed_gates": custom_return_screen.get(
                "fail_closed_gates"
            ),
        },
        "isolated_prototype_CAD_evidence": {
            "schema": prototype_cad_audit.get("schema"),
            "status": prototype_cad_audit.get("status"),
            "decision": prototype_cad_audit.get("decision"),
            "report_sha256": prototype_cad_audit.get("report_sha256"),
            "bindings_current": prototype_cad_evidence_current,
            "g0_cap_shelf_geometry_bound": g0_shelf_isolated_cad_bound,
            "custom_return_geometry_bound": custom_return_isolated_cad_bound,
            "g0_cap_shelf": prototype_cad_audit.get(
                "aggregate_boundary_g0_cap_shelf"
            ),
            "custom_return_package": prototype_cad_audit.get(
                "aggregate_boundary_follower_custom_return_packaging"
            ),
            "fail_closed_gates": prototype_cad_audit.get(
                "fail_closed_gates"
            ),
        },
        "R3_CAD_provenance": {
            "candidate_id": selected.get("id") if isinstance(selected, Mapping) else None,
            "candidate_status": selected.get("status") if isinstance(selected, Mapping) else None,
            "positive_volume_CAD_complete": positive_r3_cad,
            "mechanism_complete": mechanism_complete,
            "CAD_audit_schema": cad_audit.get("schema"),
            "CAD_audit_report_sha256": cad_audit.get("report_sha256"),
            "CAD_audit_step": cad_audit.get("source_evidence", {}).get("step"),
            "trade_prototype_record": prototype,
        },
        "continuous_motion_evidence": {
            "available_schema": continuity.get("schema"),
            "available_decision": continuity.get("decision"),
            "physical_authority_status": continuity.get("physical_authority_status"),
            "paired_C0_interval_count": continuity_coverage.get(
                "proved_adjacent_C0_interval_count"
            ),
            "unpaired_closing_interval_count": continuity_coverage.get(
                "unpaired_final_half_turn_interval_count"
            ),
            "exact_follower_law_proven": exact_continuous_law,
            "adaptive_swept_clearance_proven": adaptive_swept_clearance,
        },
        "follower_route_sweep_evidence": {
            "schema": route_sweep.get("schema"),
            "status": route_sweep.get("status"),
            "decision": route_sweep.get("decision"),
            "report_sha256": route_sweep.get("report_sha256"),
            "analytic_classification_bound": (
                route_sweep_analytic_classification_bound
            ),
            "coverage": route_sweep.get("coverage"),
            "diameter_and_contact_contract": route_sweep.get(
                "diameter_and_contact_contract"
            ),
            "turn_and_length_bounds": route_sweep.get(
                "turn_and_length_bounds"
            ),
            "analytic_gates": route_sweep.get("analytic_gates"),
            "physical_gates": route_sweep.get("physical_gates"),
            "blockers": route_sweep.get("blockers"),
        },
        "C1_rebound_evidence": {
            "schema": c1_rebound_sweep.get("schema"),
            "status": c1_rebound_sweep.get("status"),
            "decision": c1_rebound_sweep.get("decision"),
            "report_sha256": c1_rebound_sweep.get("report_sha256"),
            "evidence_current": c1_rebound_evidence_current,
            "exact_analytic_C1_biarcs_bound": c1_rebound_analytic_bound,
            "positive_volume_route_authorized": c1_rebound_physical_authority,
            "coverage": c1_rebound_sweep.get("coverage"),
            "bounds": c1_rebound_sweep.get("bounds"),
            "follower_center_travel": c1_rebound_sweep.get(
                "follower_center_travel"
            ),
            "analytic_gates": c1_rebound_sweep.get("analytic_gates"),
            "physical_gates": c1_rebound_sweep.get("physical_gates"),
        },
        "placement_trade_evidence": {
            "schema": placement_trade.get("schema"),
            "status": placement_trade.get("status"),
            "decision": placement_trade.get("decision"),
            "report_sha256": placement_trade.get("report_sha256"),
            "evidence_current": placement_trade_evidence_current,
            "analytic_trade_bound": placement_trade_analytic_bound,
            "successor_physical_authority": (
                placement_successor_physical_authority
            ),
            "coverage": placement_trade.get("coverage"),
            "current_replacement_CAD": placement_trade.get(
                "current_replacement_CAD"
            ),
            "successor_trade": placement_trade.get("successor_trade"),
            "carrier_host_screen": placement_trade.get("carrier_host_screen"),
            "analytic_gates": placement_trade.get("analytic_gates"),
            "physical_gates": placement_trade.get("physical_gates"),
        },
        "successor_prototype_evidence": {
            "schema": successor_prototype_audit.get("schema"),
            "status": successor_prototype_audit.get("status"),
            "decision": successor_prototype_audit.get("decision"),
            "report_sha256": successor_prototype_audit.get("report_sha256"),
            "evidence_current": successor_prototype_evidence_current,
            "isolated_positive_volume_geometry_bound": (
                successor_prototype_geometry_bound
            ),
            "all_4704_routes_and_full_motion_bound": (
                successor_prototype_route_and_motion_bound
            ),
            "collision_authorized": successor_prototype_collision_authorized,
            "load_tolerance_buildability_qualified": (
                successor_prototype_load_buildability_qualified
            ),
            "physical_authority": successor_prototype_physical_authority,
            "artifact_binding": {
                name: {
                    "path": str(path.relative_to(ROOT)).replace("\\", "/"),
                    "sha256": _sha256(path),
                }
                for name, path in successor_prototype_artifact_paths.items()
            },
            "checks": successor_prototype_audit.get("checks"),
            "input_hashes": successor_prototype_audit.get("input_hashes"),
            "evidence": successor_prototype_audit.get("evidence"),
            "authority": successor_prototype_audit.get("authority"),
        },
        "successor_placement_collision_evidence": {
            "schema": successor_placement_collision_audit.get("schema"),
            "status": successor_placement_collision_audit.get("status"),
            "decision": successor_placement_collision_audit.get("decision"),
            "report_sha256": successor_placement_collision_audit.get(
                "report_sha256"
            ),
            "evidence_current": successor_placement_collision_evidence_current,
            "analytic_center_and_range_bound": (
                successor_analytic_center_range_bound
            ),
            "realized_tangent_all_cases": successor_realized_tangent_all_cases,
            "full_2mm_relief_all_cases": successor_full_2mm_relief_all_cases,
            "guide_floor_collision_zero": successor_guide_floor_collision_zero,
            "sampled_self_collision_zero": (
                successor_sampled_self_collision_zero
            ),
            "sampled_floor_collision_zero": (
                successor_sampled_floor_collision_zero
            ),
            "exact_local_sibling_collision_zero": (
                successor_exact_local_sibling_collision_zero
            ),
            "artifact_binding": {
                name: {
                    "path": str(path.relative_to(ROOT)).replace("\\", "/"),
                    "sha256": _sha256(path),
                }
                for name, path in (
                    successor_placement_collision_artifact_paths.items()
                )
            },
            "analytic_all_4704_case_coverage": successor_placement_coverage,
            "direct_all_4704_guide_to_floor_counts": {
                key: successor_direct_floor.get(key)
                for key in (
                    "case_count", "exact_distance_query_count",
                    "zero_positive_common_volume_case_count",
                    "positive_common_volume_case_count",
                    "kernel_exception_count", "minimum_exact_distance_mm",
                    "maximum_common_volume_mm3",
                )
            },
            "sampled_endpoint_collision_counts": {
                name: {
                    key: successor_sampled_BREP.get(name, {}).get(key)
                    for key in (
                        "pair_evaluation_count",
                        "positive_collision_evaluation_count",
                        "unique_positive_pair_count",
                        "kernel_exception_count",
                        "maximum_common_volume_mm3",
                    )
                }
                for name in (
                    "self_collision", "own_floor_leaf_collision",
                    "conservative_R3_to_own_floor_collision",
                    "review_rack_sibling_collision",
                    "exact_active_local_rebased_sibling_collision",
                )
            },
            "sampled_pose_count": successor_sampled_BREP.get(
                "sampling_contract", {}
            ).get("total_identity_pose_count"),
            "evidence_checks": successor_placement_collision_audit.get(
                "evidence_checks"
            ),
            "blocking_findings": successor_placement_collision_audit.get(
                "blocking_findings"
            ),
            "blockers": successor_placement_collision_audit.get("blockers"),
            "authority": successor_placement_collision_audit.get("authority"),
        },
        "dancer_coupling": {
            "static_model_available": dancer_static_model_available,
            "downstream_length_history_bound": downstream_length_history_bound,
            "static_coupling_proven": dancer_static_coupling,
            "dynamic_authority": dancer_dynamic_authority,
            "limitations": dancer_limitations,
        },
        "mount_load_evidence": {
            "schema": mount_screen.get("schema"),
            "status": mount_screen.get("status"),
            "decision": mount_screen.get("decision"),
            "report_sha256": mount_screen.get("report_sha256"),
            "mount_geometry": mount_screen.get("mount_geometry"),
            "load_cases": mount_screen.get("load_cases"),
        },
        "integration_evidence": {
            "schema": integration_audit.get("schema"),
            "status": integration_audit.get("status"),
            "decision": integration_audit.get("decision"),
            "report_sha256": integration_audit.get("report_sha256"),
            "additive_integration_feasible": integration_audit.get(
                "additive_integration_feasible"
            ),
            "reference_positive_pair_count": integration_audit.get(
                "reference_pose_OCC_scan", {}
            ).get("positive_pair_count"),
            "replacement_carriage_integration_authorized": (
                replacement_carriage_integration_authorized
            ),
        },
        "retraction_evidence": {
            "schema": retraction_topology.get("schema"),
            "status": retraction_topology.get("status"),
            "report_sha256": retraction_topology.get("report_sha256"),
            "analysis_gates": retraction_topology.get("analysis_gates", {}),
            "physical_authority_gates": retraction_topology.get(
                "physical_authority_gates", {}
            ),
            "physical_retraction_integrated": physical_retraction_integrated,
        },
        "replacement_architecture_evidence": {
            "schema": replacement_architecture.get("schema"),
            "status": replacement_architecture.get("status"),
            "decision": replacement_architecture.get("decision"),
            "report_sha256": replacement_architecture.get("report_sha256"),
            "identity_and_counts_bound": replacement_architecture_bound,
            "travel": replacement_architecture.get("travel"),
            "exact_install_counts": replacement_architecture.get(
                "exact_install_counts"
            ),
            "physical_gates": replacement_architecture.get("physical_gates"),
        },
        "replacement_CAD_evidence": {
            "schema": replacement_cad_audit.get("schema"),
            "status": replacement_cad_audit.get("status"),
            "decision": replacement_cad_audit.get("decision"),
            "report_sha256": replacement_cad_audit.get("report_sha256"),
            "static_CAD_geometry_bound": replacement_cad_static_bound,
            "STEP_sha256": replacement_cad_audit.get(
                "artifact_binding", {}
            ).get("step", {}).get("sha256"),
            "leaf_accounting": replacement_cad_audit.get(
                "leaf_accounting"
            ),
            "hardware_witness": replacement_cad_audit.get(
                "hardware_witness"
            ),
            "state_pair_audit": replacement_cad_audit.get(
                "state_pair_audit"
            ),
            "authority": replacement_cad_audit.get("authority"),
            "open_blockers": replacement_cad_audit.get("open_blockers"),
        },
        "replacement_transition_evidence": {
            "schema": replacement_transition_sweep.get("schema"),
            "status": replacement_transition_sweep.get("status"),
            "decision": replacement_transition_sweep.get("decision"),
            "report_sha256": replacement_transition_sweep.get(
                "report_sha256"
            ),
            "evidence_current": replacement_transition_evidence_current,
            "sampled_geometry_bound": replacement_transition_geometry_bound,
            "physical_authority": replacement_transition_physical_authority,
            "sampling": replacement_transition_sweep.get("sampling"),
            "sampled_geometry_result": replacement_transition_sweep.get(
                "sampled_geometry_result"
            ),
            "collision_audit": replacement_transition_sweep.get(
                "collision_audit"
            ),
            "clearance_audit": replacement_transition_sweep.get(
                "clearance_audit"
            ),
            "authority": replacement_transition_sweep.get("authority"),
            "blockers": replacement_transition_sweep.get("blockers"),
        },
        "replacement_load_wear_evidence": {
            "schema": replacement_load_wear.get("schema"),
            "status": replacement_load_wear.get("status"),
            "decision": replacement_load_wear.get("decision"),
            "report_sha256": replacement_load_wear.get("report_sha256"),
            "evidence_current": replacement_load_wear_evidence_current,
            "analytical_screens_bound": replacement_load_wear_analytic_bound,
            "physical_qualification_complete": (
                replacement_load_wear_qualified
            ),
            "load_envelope": replacement_load_wear.get("load_envelope"),
            "analytical_gates": replacement_load_wear.get(
                "analytical_gates"
            ),
            "qualification_gates": replacement_load_wear.get(
                "qualification_gates"
            ),
            "authority": replacement_load_wear.get("authority"),
            "open_blockers": replacement_load_wear.get("open_blockers"),
        },
        "retraction_procurement_evidence": {
            "schema": retraction_procurement.get("schema"),
            "status": retraction_procurement.get("status"),
            "catalog_snapshot_date": retraction_procurement.get(
                "catalog_snapshot_date"
            ),
            "report_sha256": retraction_procurement.get("report_sha256"),
            "evidence_current": retraction_procurement_evidence_current,
            "hardware_stack_releasable": retraction_hardware_releasable,
            "fail_closed_gates": retraction_procurement.get(
                "fail_closed_gates"
            ),
        },
        "gates": gates,
        "blocker_count": len(blockers),
        "blockers": blockers,
        "input_files": {
            name: str(path.relative_to(ROOT)).replace("\\", "/")
            for name, path in paths.items()
        },
        "input_sha256": {name: _sha256(path) for name, path in paths.items()},
        "source_hashes": {
            "sim/aggregate_boundary_follower_acceptance.py": _sha256(Path(__file__)),
            "cad/dancer_loads.py": _sha256(ROOT / "cad" / "dancer_loads.py"),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported aggregate-follower acceptance schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("aggregate-follower acceptance report hash mismatch")
    for relative, expected in report.get("source_hashes", {}).items():
        path = _source_path(relative)
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale acceptance source {relative}")
    for name, relative in report.get("input_files", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != report.get("input_sha256", {}).get(name):
            raise ValueError(f"stale acceptance input {name}")
    successor = report.get("successor_prototype_evidence")
    if not isinstance(successor, Mapping) or not _recorded_path_hashes_current(
        successor.get("artifact_binding", {})
    ):
        raise ValueError("stale successor prototype source, STEP, manifest, or audit")
    if successor.get("physical_authority") is not False:
        raise ValueError("isolated successor prototype cannot have physical authority")
    if any(successor.get("authority", {}).values()):
        raise ValueError("successor prototype audit authority must remain false")
    placement_collision = report.get("successor_placement_collision_evidence")
    if (
        not isinstance(placement_collision, Mapping)
        or not _recorded_path_hashes_current(
            placement_collision.get("artifact_binding", {})
        )
    ):
        raise ValueError("stale successor placement/collision audit evidence")
    if any(placement_collision.get("authority", {}).values()):
        raise ValueError("successor placement/collision authority must remain false")
    expected = "PASS" if all(report.get("gates", {}).values()) else "FAIL"
    if report.get("status") != expected:
        raise ValueError("acceptance status/gate mismatch")
    if report.get("production_authorized") is not False:
        raise ValueError("isolated follower gate cannot authorize production")
    if report.get("assembly_integration_authorized") is not False:
        raise ValueError("isolated follower gate cannot authorize integration")
    if report.get("dynamic_authority") != "FAIL":
        raise ValueError("current follower gate must fail dynamic authority")


def render_markdown(report: Mapping[str, Any]) -> str:
    coverage = report["coverage"]
    lines = [
        "# Aggregate-boundary follower acceptance gate", "",
        f"**{report['status']} — {report['decision']}**", "",
        "The exact endpoint family, 4,704 analytic C1 biarcs, and 232-pose "
        "nominal replacement-carriage transition sweep are bound. An isolated "
        "positive-volume four-stage successor topology prototype is also "
        "hash-bound. Its numeric center and command ranges cover all 4,704 "
        "cases, but the realized tangent and collision audit fails, so no "
        "moving physical follower is accepted.", "",
        "## Coverage", "",
        f"- Endpoint loci bound: {coverage['bound_endpoint_loci']} / {coverage['required_endpoint_loci']}",
        f"- g=0 right-seam normal blockers: {coverage['g0_blocker_count']} (24 left PEEK-floor owners already exact)",
        f"- Nonzero-growth supporting tangents: {coverage['nonzero_growth_support_tangent_count']} / {coverage['nonzero_growth_endpoint_count']}",
        f"- Direct C1 mismatches requiring physical R3 closure: {coverage['direct_C1_mismatch_locus_count']}",
        f"- Exact analytic C1 biarcs: {coverage['analytic_C1_biarc_case_count']} / 4704",
        f"- Current CAD C1 centers covered: {coverage['current_CAD_C1_center_covered_case_count']} / 4704",
        f"- Successor analytic C1 centers covered: {coverage['successor_analytic_C1_center_covered_case_count']} / 4704",
        f"- Isolated positive-volume successor stages: {coverage['successor_isolated_prototype_stage_count']} (4,704-case surface bound: {coverage['successor_prototype_4704_case_surface_proved']})",
        f"- Realized prototype tangent matches: {coverage['successor_realized_tangent_match_case_count']} / 4704",
        f"- Cases retaining full 2 mm R3/R5 relief margin: {coverage['successor_full_2mm_relief_margin_case_count']} / 4704",
        f"- Direct guide/floor positive cases: {coverage['successor_guide_floor_positive_case_count']} / 4704",
        f"- Sampled positive pair counts — self: {coverage['successor_sampled_self_positive_pair_count']}; floor: {coverage['successor_sampled_floor_positive_pair_count']}; exact-local siblings: {coverage['successor_exact_local_sibling_positive_pair_count']}",
        f"- Nominal replacement transition poses checked: {coverage['replacement_sampled_transition_pose_count']}",
        f"- Physically authorized moving intervals: {coverage['physically_authorized_moving_intervals']} / {coverage['required_moving_intervals']}",
        "", "## Required evidence", "",
        "- Complete the R3 follower attachments, spring anchors, tangential "
        "bearing/return, and positive M0 retraction linkage.",
        "- Correct the prototype Euler frame: the current transform realizes "
        "none of the 4,704 requested guide tangents.",
        "- Redesign the moving guide/floor, self, and exact-active-local sibling "
        "interfaces, then run a continuous tolerance-aware motion sweep.",
        "- Transfer and validate the prototype relief coupons in the selected "
        "carrier; the reviewed carrier itself has not been modified.",
        "- Qualify retention, preload, fatigue, PEEK/wire wear, bushing PV, "
        "tolerances, and the full multidirectional 40 N proof case.",
        "- Replace the existing active-sector yoke/guides with a U-windowed "
        "four-occurrence module; the current additive placement has 21 "
        "positive-volume reference intersections.",
        "- Integrate the closed M0 cam/dwell, gimbal dock, and dual-NC actual-"
        "position interlock; the topology equations alone are not authority.",
        "- Select or custom-manufacture the centering and independent-return "
        "springs, redesign the igus bushing pocket, and provide remote positive "
        "transfer for the dual direct-opening switches.",
        "- Integrate the robust 1.50 x 0.75 x >=0.30 mm right-seam PEEK "
        "shelf, rebind diameter-dependent endpoints, and widen the entire "
        "0.47752 mm cap lane to at least 0.65 mm before 0.5 mm wire can pass.",
        "- Prototype and rate-sort the screened 17-7PH tangential spring or "
        "etched flexure, then CAD-sweep and calibrate the reduced constant-force "
        "radial cartridge before any return hardware is selected.",
        "- Redesign or prove the eccentric tower joint for the 5.52 N m radial "
        "40 N load case; direct 10 N-per-M4 sharing is insufficient.",
        "- Exact continuous intra-half-turn follower law; affine passage through the aggregate interior is forbidden.",
        "- Adaptive swept clearance against final rigid parts, core, cap, prior aggregate, and self.",
        "- Downstream route-length history coupled to the existing dancer static equilibrium.",
        "- Separate dynamic dancer/flyer-acceleration evidence.",
        "", "## Gates", "",
    ]
    lines.extend(
        f"- {'PASS' if value else 'OPEN'} — `{name}`"
        for name, value in report["gates"].items()
    )
    lines.extend([
        "", "## Authority boundary", "",
        "No strand order is invented. Straight taut free spans remain eligible "
        "only with owned endpoint reactions, C0/C1 continuity, swept clearance, "
        "and positive tension. Production, assembly integration, and dynamic "
        "authority remain false.", "",
    ])
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result = dict(report or analyze())
    validate_report_integrity(result)
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(render_markdown(result), encoding="utf-8")
    return result


def main() -> int:
    report = write_outputs()
    print(
        f"aggregate follower acceptance {report['status']}: "
        f"endpoints={report['coverage']['bound_endpoint_loci']}; "
        f"g0={report['coverage']['g0_blocker_count']}; "
        f"moving={report['coverage']['physically_authorized_moving_intervals']}"
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
