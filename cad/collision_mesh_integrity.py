"""Fail-closed topology gate for adapter collision meshes.

CAD / validation brief
----------------------
Input
    An isolated integrated-adapter ``links/manifest.json``, its per-occurrence
    STL assets, and the reference-pose GLB.  The adapter's exact vendor visual
    meshes are read-only inputs.
Output
    A hash-bound JSON audit plus optional, separately stored conservative
    convex-hull collision assets for explicitly approved imported COTS whose
    vendor tessellations are open.  No source or adapter artifact is repaired,
    replaced, shrunk, or overwritten.
Policy
    Custom printed, PEEK, and fabricated occurrences must serialize as one
    watertight shell with no boundary/nonmanifold edges.  Every effective
    collision asset must be nonempty, finite, consistently wound, watertight,
    and free of degenerate/boundary/nonmanifold topology after vertex merge.
    Imported vendor visuals are reported separately.  Only the explicitly
    named Leadshine/MGN family is eligible for a whole-occurrence convex-hull
    collision overbound; exact vendor geometry remains the visual authority.
Final integration
    A final adapter supplies explicit per-part ``provenance_class`` metadata,
    calls :func:`audit_adapter`, consumes :func:`collision_override_plan`
    before hashing its collision manifest, and calls
    :func:`require_release_ready`.  Legacy label inference exists only to
    diagnose older frozen adapters and deliberately cannot pass the explicit
    provenance gate.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping

import numpy as np
from pygltflib import GLTF2
from scipy.spatial import ConvexHull
import trimesh


ROOT = Path(__file__).resolve().parent.parent
SCHEMA = "collision-mesh-integrity/v1"
POLICY_VERSION = "collision-mesh-policy/1"
LINK_NAMES = ("static", "carriage", "spindle", "flyer")
HULL_METHOD = "whole_occurrence_convex_hull"

PROVENANCE_CUSTOM = "custom_manufactured"
PROVENANCE_IMPORTED = "imported_cots_exact_visual"
PROVENANCE_MODELED = "modeled_cots_or_hardware"
PROVENANCE_UNCLASSIFIED = "unclassified"
VALID_PROVENANCE = {
    PROVENANCE_CUSTOM,
    PROVENANCE_IMPORTED,
    PROVENANCE_MODELED,
}

# Exact imported COTS occurrences known in the legacy adapter.  Final adapters
# must serialize this provenance explicitly instead of relying on this set.
LEGACY_IMPORTED_COTS = {
    "mgn12_rail_L",
    "mgn12_rail_R",
    "mgn12h_L",
    "mgn12h_R",
    "m2_Leadshine_CS-M21708_exact_cableless",
    "flyer_6001_front",
    "flyer_6001_rear",
    "spindle_608_top",
    "spindle_608_bot",
    "m1_coupling",
    "t8_screw",
    "t8_nut_main",
    "t8_nut_spring",
    "t8_nut_secondary",
}

# Only these exact-vendor families may be automatically overbounded.  Other
# open imports remain failures until an explicit engineering decision is added.
LEGACY_HULL_ELIGIBLE = {
    "mgn12_rail_L",
    "mgn12_rail_R",
    "mgn12h_L",
    "mgn12h_R",
    "m2_Leadshine_CS-M21708_exact_cableless",
}

# The detailed winged model is useful visually but its frozen tessellation has
# a two-edge nonmanifold seam.  Its collision substitute may be a conservative
# whole hull; custom-manufactured parts remain ineligible for auto-hulling.
LEGACY_MODELED_HULL_ELIGIBLE = {"felt_m4_wingnut"}

LEGACY_CUSTOM_EXACT = {
    "rear_post_left_shoe",
    "m0_fixed_end_mount",
    "m0_motor_mount",
    "endstop_mount",
    "flyer_block",
    "m2_successor_nema17_mount",
    "spool_bracket",
    "spool_drum",
    "felt_tensioner",
    "dancer_base",
    "dancer_arm",
    "dancer_pulley",
    "entry_bracket",
    "fabricated_carriage_0p250in_mic6",
    "printable_carriage_endstop_flag",
    "spindle_tower",
    "nut_bracket",
    "front_cap_natural_unfilled_PEEK_production_review",
    "rear_cap_natural_unfilled_PEEK_production_review",
    "retained_offset_spoke_flyer_arm_one_printed_solid",
}
CUSTOM_LABEL_RE = re.compile(r"(?i)(peek|petg|fabricated|printed|printable)")
HARDWARE_LABEL_RE = re.compile(
    r"(?i)(m[23458]x\d|tnut|washer|nyloc|set_screw|shoulder|"
    r"insert|bolt|shim|spring|collar|circlip|din47)"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_hash(payload: Mapping[str, Any]) -> str:
    value = dict(payload)
    value.pop("contract_sha256", None)
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _safe_name(label: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", label).strip("._")
    return value[:140] or "unlabeled"


def _load_mesh(path: Path) -> tuple[trimesh.Trimesh, dict[str, int]]:
    """Load without topology repair and merge coincident STL vertices."""

    loaded = trimesh.load(path, force="mesh", process=False)
    if isinstance(loaded, trimesh.Scene):
        meshes = [geometry for geometry in loaded.geometry.values()]
        loaded = (
            trimesh.util.concatenate(meshes)
            if meshes
            else trimesh.Trimesh(process=False)
        )
    if not isinstance(loaded, trimesh.Trimesh):
        raise TypeError(f"expected a Trimesh from {path}, got {type(loaded).__name__}")
    mesh = loaded.copy()
    before = {"vertices": int(len(mesh.vertices)), "faces": int(len(mesh.faces))}
    if len(mesh.vertices):
        # Required normalization for binary STL, which repeats triangle
        # vertices.  This does not fill holes, delete faces, or alter bounds.
        mesh.merge_vertices()
        mesh.remove_unreferenced_vertices()
    after = {"vertices": int(len(mesh.vertices)), "faces": int(len(mesh.faces))}
    return mesh, {
        "vertices_before_merge": before["vertices"],
        "vertices_after_merge": after["vertices"],
        "faces_before_merge": before["faces"],
        "faces_after_merge": after["faces"],
    }


def topology_facts(path: Path | str) -> dict[str, Any]:
    """Classify the serialized mesh after vertex merge, without repairing it."""

    source = Path(path).resolve()
    mesh, counts = _load_mesh(source)
    empty = bool(len(mesh.vertices) == 0 or len(mesh.faces) == 0)
    finite = bool(
        not empty
        and np.isfinite(np.asarray(mesh.vertices)).all()
        and np.isfinite(np.asarray(mesh.faces, dtype=float)).all()
    )

    boundary_edges = 0
    nonmanifold_edges = 0
    zero_length_edges = 0
    shell_count = 0
    degenerate_faces = 0
    if not empty:
        edges = np.asarray(mesh.edges_sorted, dtype=np.int64)
        valid = edges[:, 0] != edges[:, 1]
        zero_length_edges = int(np.count_nonzero(~valid))
        if np.any(valid):
            _unique, incidence = np.unique(
                edges[valid], axis=0, return_counts=True
            )
            boundary_edges = int(np.count_nonzero(incidence == 1))
            nonmanifold_edges = int(np.count_nonzero(incidence > 2))
        keep = np.asarray(mesh.nondegenerate_faces(), dtype=bool)
        degenerate_faces = int(len(keep) - np.count_nonzero(keep))
        components = trimesh.graph.connected_components(
            mesh.face_adjacency,
            nodes=np.arange(len(mesh.faces), dtype=np.int64),
            min_len=1,
        )
        shell_count = int(len(components))

    if empty:
        topology_class = "empty"
    elif boundary_edges and nonmanifold_edges:
        topology_class = "boundary_and_nonmanifold"
    elif boundary_edges:
        topology_class = "boundary_edge"
    elif nonmanifold_edges:
        topology_class = "nonmanifold_only"
    elif shell_count > 1:
        topology_class = "closed_multi_shell"
    else:
        topology_class = "closed_single_shell"

    watertight = bool(
        not empty
        and boundary_edges == 0
        and nonmanifold_edges == 0
        and mesh.is_watertight
    )
    winding_consistent = bool(not empty and mesh.is_winding_consistent)
    clean_collision_mesh = bool(
        finite
        and watertight
        and winding_consistent
        and degenerate_faces == 0
        and zero_length_edges == 0
    )
    bounds = None
    if not empty and finite:
        bounds = np.asarray(mesh.bounds, dtype=float).tolist()
    volume = float(mesh.volume) if not empty and finite else 0.0
    return {
        "path": str(source),
        "sha256": sha256(source),
        **counts,
        "empty": empty,
        "finite": finite,
        "topology_class": topology_class,
        "boundary_edges": boundary_edges,
        "nonmanifold_edges": nonmanifold_edges,
        "nonmanifold_only": bool(
            nonmanifold_edges > 0 and boundary_edges == 0
        ),
        "multi_shell": bool(shell_count > 1),
        "shell_count": shell_count,
        "degenerate_faces": degenerate_faces,
        "zero_length_edges": zero_length_edges,
        "watertight": watertight,
        "winding_consistent": winding_consistent,
        "signed_volume_mm3": volume,
        "positive_signed_volume": bool(math.isfinite(volume) and volume > 0.0),
        "bounds_mm": bounds,
        "clean_collision_mesh": clean_collision_mesh,
    }


def _manifest_part_path(
    links_dir: Path, link: str, label: str, record: Mapping[str, Any]
) -> Path:
    file_name = record.get("file")
    if not isinstance(file_name, str) or Path(file_name).name != file_name:
        raise ValueError(f"unsafe collision filename for {link}/{label}")
    path = (links_dir / "parts" / link / file_name).resolve()
    parent = (links_dir / "parts" / link).resolve()
    if parent not in path.parents:
        raise ValueError(f"collision asset escapes part directory: {link}/{label}")
    return path


def _legacy_policy(label: str) -> str:
    if label in LEGACY_IMPORTED_COTS:
        return PROVENANCE_IMPORTED
    if label in LEGACY_CUSTOM_EXACT:
        return PROVENANCE_CUSTOM
    if HARDWARE_LABEL_RE.search(label):
        return PROVENANCE_MODELED
    if CUSTOM_LABEL_RE.search(label):
        return PROVENANCE_CUSTOM
    return PROVENANCE_MODELED


def _provenance(
    link: str,
    label: str,
    record: Mapping[str, Any],
    overrides: Mapping[str, str],
    allow_legacy: bool,
) -> tuple[str, str]:
    explicit = record.get("provenance_class")
    if explicit is not None:
        if explicit not in VALID_PROVENANCE:
            return PROVENANCE_UNCLASSIFIED, "invalid_manifest_value"
        return str(explicit), "manifest"
    key = f"{link}/{label}"
    if key in overrides:
        selected = overrides[key]
        if selected not in VALID_PROVENANCE:
            return PROVENANCE_UNCLASSIFIED, "invalid_override_value"
        return selected, "explicit_override"
    if allow_legacy:
        return _legacy_policy(label), "legacy_label_fallback"
    return PROVENANCE_UNCLASSIFIED, "missing"


def _hull_halfspace_violation(
    hull: trimesh.Trimesh, points: np.ndarray
) -> float:
    equations = ConvexHull(np.asarray(hull.vertices, dtype=float)).equations
    normals = equations[:, :3]
    offsets = equations[:, 3]
    maximum = -math.inf
    for start in range(0, len(points), 5000):
        block = points[start : start + 5000]
        values = block @ normals.T + offsets
        maximum = max(maximum, float(np.max(values)))
    return maximum


def generate_conservative_hull(
    source_path: Path | str,
    output_path: Path | str,
    *,
    source_visual_role: str = PROVENANCE_IMPORTED,
    containment_tolerance_mm: float = 1.0e-6,
) -> dict[str, Any]:
    """Generate and verify a whole-occurrence convex collision overbound."""

    source = Path(source_path).resolve()
    target = Path(output_path).resolve()
    source_hash_before = sha256(source)
    mesh, _counts = _load_mesh(source)
    if len(mesh.vertices) < 4 or len(mesh.faces) == 0:
        raise ValueError(f"cannot hull empty/underspecified source {source}")
    if not np.isfinite(np.asarray(mesh.vertices)).all():
        raise ValueError(f"cannot hull non-finite source {source}")
    hull = mesh.convex_hull
    hull.metadata.clear()
    target.parent.mkdir(parents=True, exist_ok=True)
    hull.export(target, file_type="stl")
    if sha256(source) != source_hash_before:
        raise RuntimeError("vendor visual changed while generating collision hull")

    serialized, _ = _load_mesh(target)
    envelope_facts = topology_facts(target)
    points = np.asarray(mesh.vertices, dtype=float)
    violation = _hull_halfspace_violation(serialized, points)
    source_bounds = np.asarray(mesh.bounds, dtype=float)
    hull_bounds = np.asarray(serialized.bounds, dtype=float)
    lower_shortfall = np.maximum(hull_bounds[0] - source_bounds[0], 0.0)
    upper_shortfall = np.maximum(source_bounds[1] - hull_bounds[1], 0.0)
    aabb_shortfall = float(max(np.max(lower_shortfall), np.max(upper_shortfall)))
    contained = bool(
        violation <= containment_tolerance_mm
        and aabb_shortfall <= containment_tolerance_mm
    )
    overbound_pass = bool(envelope_facts["clean_collision_mesh"] and contained)
    source_extents = source_bounds[1] - source_bounds[0]
    source_aabb_volume = float(np.prod(source_extents))
    envelope_volume = float(abs(serialized.volume))
    return {
        "method": HULL_METHOD,
        "not_a_repair": True,
        "source_visual_retained": True,
        "source_visual_role": source_visual_role,
        "exact_vendor_visual_retained": bool(
            source_visual_role == PROVENANCE_IMPORTED
        ),
        "detailed_modeled_visual_retained": bool(
            source_visual_role == PROVENANCE_MODELED
        ),
        "fills_concavities_holes_and_voids": True,
        "source_visual": {
            "path": str(source),
            "sha256": source_hash_before,
            "vertex_count_after_merge": int(len(mesh.vertices)),
            "bounds_mm": source_bounds.tolist(),
        },
        "collision_envelope": envelope_facts,
        "overbound_proof": {
            "basis": (
                "every source triangle lies in the convex hull of its vertices"
            ),
            "all_source_vertices_contained": contained,
            "maximum_halfspace_violation_mm": violation,
            "maximum_aabb_shortfall_mm": aabb_shortfall,
            "containment_tolerance_mm": containment_tolerance_mm,
            "source_aabb_preserved_or_expanded": bool(
                aabb_shortfall <= containment_tolerance_mm
            ),
            "source_aabb_volume_mm3": source_aabb_volume,
            "envelope_volume_mm3": envelope_volume,
            "envelope_volume_to_source_aabb_ratio": (
                envelope_volume / source_aabb_volume
                if source_aabb_volume > 0.0
                else None
            ),
            "source_signed_volume_mm3_non_authoritative_for_open_mesh": float(
                mesh.volume
            ),
            "status": "PASS" if overbound_pass else "FAIL",
        },
        "status": "PASS" if overbound_pass else "FAIL",
    }


def _read_float_vec3_accessor(gltf: GLTF2, index: int) -> np.ndarray:
    accessor = gltf.accessors[index]
    if accessor.componentType != 5126 or accessor.type != "VEC3":
        raise ValueError("NORMAL accessor must be FLOAT VEC3")
    if accessor.bufferView is None or accessor.sparse is not None:
        raise ValueError("NORMAL accessor must be a dense buffer view")
    view = gltf.bufferViews[accessor.bufferView]
    blob = gltf.binary_blob()
    if blob is None:
        raise ValueError("GLB has no binary blob")
    stride = int(view.byteStride or 12)
    start = int(view.byteOffset or 0) + int(accessor.byteOffset or 0)
    rows = []
    for row in range(int(accessor.count)):
        rows.append(
            np.frombuffer(blob, dtype="<f4", count=3, offset=start + row * stride)
        )
    return np.asarray(rows, dtype=float)


def gltf_normal_audit(path: Path | str) -> dict[str, Any]:
    source = Path(path).resolve()
    if not source.is_file():
        return {
            "path": str(source),
            "status": "FAIL",
            "reason": "missing GLB",
            "primitive_count": 0,
        }
    gltf = GLTF2().load(str(source))
    rows = []
    for mesh_index, mesh in enumerate(gltf.meshes or []):
        for primitive_index, primitive in enumerate(mesh.primitives or []):
            position_index = getattr(primitive.attributes, "POSITION", None)
            normal_index = getattr(primitive.attributes, "NORMAL", None)
            row: dict[str, Any] = {
                "mesh_index": mesh_index,
                "mesh_name": mesh.name,
                "primitive_index": primitive_index,
                "position_accessor": position_index,
                "normal_accessor": normal_index,
                "normal_present": normal_index is not None,
            }
            try:
                if position_index is None or normal_index is None:
                    raise ValueError("POSITION or NORMAL accessor missing")
                positions = gltf.accessors[position_index]
                normals = _read_float_vec3_accessor(gltf, int(normal_index))
                lengths = np.linalg.norm(normals, axis=1)
                row.update(
                    {
                        "count_matches_position": bool(
                            int(positions.count) == len(normals)
                        ),
                        "finite": bool(np.isfinite(normals).all()),
                        "minimum_length": float(np.min(lengths)),
                        "maximum_length": float(np.max(lengths)),
                        "unit_length": bool(
                            np.isfinite(lengths).all()
                            and np.all(np.abs(lengths - 1.0) <= 1.0e-3)
                        ),
                    }
                )
                row["status"] = (
                    "PASS"
                    if row["count_matches_position"]
                    and row["finite"]
                    and row["unit_length"]
                    else "FAIL"
                )
            except Exception as exc:  # report malformed glTF fail-closed
                row.update({"status": "FAIL", "reason": str(exc)})
            rows.append(row)
    passed = bool(rows and all(row["status"] == "PASS" for row in rows))
    return {
        "path": str(source),
        "sha256": sha256(source),
        "primitive_count": len(rows),
        "all_primitives_have_finite_unit_NORMAL": passed,
        "status": "PASS" if passed else "FAIL",
        "primitives": rows,
    }


def _aggregate_visual_audit(
    manifest: Mapping[str, Any], links_dir: Path
) -> list[dict[str, Any]]:
    rows = []
    for link, record in manifest.get("links", {}).items():
        file_name = record.get("file")
        if not isinstance(file_name, str) or Path(file_name).name != file_name:
            rows.append({"link": link, "status": "FAIL", "reason": "unsafe file"})
            continue
        path = links_dir / file_name
        facts = topology_facts(path)
        rows.append(
            {
                "link": link,
                "role": "exact_visual_aggregate_not_collision_authority",
                "manifest_sha256": record.get("sha256"),
                "manifest_hash_valid": bool(
                    facts["sha256"] == record.get("sha256")
                ),
                **facts,
            }
        )
    return rows


def _visual_asset_hash_audit(
    manifest: Mapping[str, Any], links_dir: Path, glb_path: Path
) -> dict[str, Any]:
    rows = []
    for section in ("links", "visual_groups", "wire_assets"):
        for name, record in manifest.get(section, {}).items():
            file_name = record.get("file")
            safe = bool(
                isinstance(file_name, str) and Path(file_name).name == file_name
            )
            path = links_dir / file_name if safe else links_dir / "__unsafe__"
            actual = sha256(path) if safe and path.is_file() else None
            rows.append(
                {
                    "section": section,
                    "name": name,
                    "path": str(path),
                    "manifest_sha256": record.get("sha256"),
                    "actual_sha256": actual,
                    "hash_valid": bool(actual and actual == record.get("sha256")),
                }
            )
    glb_record = manifest.get("reference_pose_glb")
    if isinstance(glb_record, Mapping):
        file_name = glb_record.get("file")
        safe = bool(
            isinstance(file_name, str) and Path(file_name).name == file_name
        )
        expected_location = links_dir.parent / file_name if safe else None
        location_matches = bool(
            expected_location is not None
            and expected_location.resolve() == glb_path.resolve()
        )
        actual = sha256(glb_path) if glb_path.is_file() else None
        rows.append(
            {
                "section": "reference_pose_glb",
                "name": "reference_pose_glb",
                "path": str(glb_path),
                "manifest_sha256": glb_record.get("sha256"),
                "actual_sha256": actual,
                "manifest_location_matches": location_matches,
                "hash_valid": bool(
                    location_matches and actual == glb_record.get("sha256")
                ),
            }
        )
    else:
        rows.append(
            {
                "section": "reference_pose_glb",
                "name": "reference_pose_glb",
                "path": str(glb_path),
                "manifest_sha256": None,
                "actual_sha256": sha256(glb_path) if glb_path.is_file() else None,
                "manifest_location_matches": False,
                "hash_valid": False,
                "reason": "manifest has no reference_pose_glb record",
            }
        )
    failures = [
        f"{row['section']}/{row['name']}"
        for row in rows
        if not row["hash_valid"]
    ]
    return {
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "assets": rows,
    }


def audit_adapter(
    manifest_path: Path | str,
    envelope_root: Path | str,
    *,
    glb_path: Path | str | None = None,
    provenance_overrides: Mapping[str, str] | None = None,
    allow_legacy_label_fallback: bool = False,
    generate_imported_envelopes: bool = True,
) -> dict[str, Any]:
    """Audit one isolated adapter and return a deterministic report payload."""

    manifest_file = Path(manifest_path).resolve()
    links_dir = manifest_file.parent
    envelopes = Path(envelope_root).resolve()
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    overrides = dict(provenance_overrides or {})

    manifest_contract_valid = bool(
        manifest.get("contract_sha256") == canonical_hash(manifest)
    )
    records = manifest.get("parts")
    if not isinstance(records, Mapping):
        raise ValueError("adapter manifest has no per-occurrence collision mapping")

    rows = []
    asset_hash_mismatches = []
    provenance_sources = Counter()
    topology_classes = Counter()
    provenance_counts = Counter()
    custom_failures = []
    effective_failures = []
    envelope_count = 0

    for link, by_label in records.items():
        if not isinstance(by_label, Mapping):
            raise ValueError(f"adapter manifest collision link {link} is malformed")
        for label, raw_record in by_label.items():
            if not isinstance(raw_record, Mapping):
                raise ValueError(f"adapter part record is malformed: {link}/{label}")
            part_path = _manifest_part_path(links_dir, link, label, raw_record)
            expected_hash = raw_record.get("sha256")
            actual_hash = sha256(part_path)
            hash_valid = bool(expected_hash == actual_hash)
            if not hash_valid:
                asset_hash_mismatches.append(f"{link}/{label}")
            provenance, provenance_source = _provenance(
                link,
                label,
                raw_record,
                overrides,
                allow_legacy_label_fallback,
            )
            provenance_sources[provenance_source] += 1
            provenance_counts[provenance] += 1
            facts = topology_facts(part_path)
            topology_classes[facts["topology_class"]] += 1
            single_shell_required = provenance == PROVENANCE_CUSTOM
            custom_pass = bool(
                facts["clean_collision_mesh"]
                and (not single_shell_required or facts["shell_count"] == 1)
            )
            envelope = None
            effective = facts
            explicit_hull_method = raw_record.get("collision_overbound_method")
            explicit_hull_eligible = explicit_hull_method == HULL_METHOD
            invalid_hull_method = bool(
                explicit_hull_method is not None and not explicit_hull_eligible
            )
            legacy_imported_hull = bool(
                provenance == PROVENANCE_IMPORTED
                and label in LEGACY_HULL_ELIGIBLE
            )
            legacy_modeled_hull = bool(
                provenance == PROVENANCE_MODELED
                and label in LEGACY_MODELED_HULL_ELIGIBLE
            )
            hull_eligible = bool(
                provenance in {PROVENANCE_IMPORTED, PROVENANCE_MODELED}
                and (
                    legacy_imported_hull
                    or legacy_modeled_hull
                    or explicit_hull_eligible
                )
                and not invalid_hull_method
            )
            if not facts["clean_collision_mesh"] and hull_eligible:
                if generate_imported_envelopes:
                    target = (
                        envelopes
                        / link
                        / f"{_safe_name(label)}__whole_convex_hull.stl"
                    )
                    envelope = generate_conservative_hull(
                        part_path, target, source_visual_role=provenance
                    )
                    envelope_count += 1
                    if envelope["status"] == "PASS":
                        effective = envelope["collision_envelope"]
                else:
                    envelope = {
                        "status": "FAIL",
                        "reason": "required imported-COTS envelope generation disabled",
                    }
            effective_pass = bool(
                hash_valid
                and provenance != PROVENANCE_UNCLASSIFIED
                and not invalid_hull_method
                and effective["clean_collision_mesh"]
                and (
                    provenance != PROVENANCE_CUSTOM
                    or effective["shell_count"] == 1
                )
                and (envelope is None or envelope.get("status") == "PASS")
            )
            if provenance == PROVENANCE_CUSTOM and not custom_pass:
                custom_failures.append(f"{link}/{label}")
            if not effective_pass:
                effective_failures.append(f"{link}/{label}")
            rows.append(
                {
                    "link": link,
                    "label": label,
                    "manifest_record": dict(raw_record),
                    "manifest_asset_hash_valid": hash_valid,
                    "provenance_class": provenance,
                    "provenance_source": provenance_source,
                    "single_shell_required": single_shell_required,
                    "original_visual_and_collision_mesh": facts,
                    "imported_hull_eligible": hull_eligible,
                    "hull_eligibility_source": (
                        "manifest"
                        if explicit_hull_eligible
                        else "legacy_imported_allowlist"
                        if legacy_imported_hull
                        else "legacy_modeled_hardware_allowlist"
                        if legacy_modeled_hull
                        else None
                    ),
                    "invalid_collision_overbound_method": invalid_hull_method,
                    "generated_collision_overbound": envelope,
                    "effective_collision_mesh": effective,
                    "custom_topology_pass": custom_pass,
                    "effective_collision_topology_pass": effective_pass,
                }
            )

    explicit_provenance = bool(
        rows
        and all(
            row["provenance_source"] in {"manifest", "explicit_override"}
            for row in rows
        )
    )
    if glb_path is None:
        glb_record = manifest.get("reference_pose_glb", {})
        glb_name = glb_record.get(
            "file", "integrated_candidate_reference_pose.glb"
        )
        glb_path = links_dir.parent / glb_name
    glb_file = Path(glb_path).resolve()
    normal_report = gltf_normal_audit(glb_file)
    aggregates = _aggregate_visual_audit(manifest, links_dir)
    visual_hashes = _visual_asset_hash_audit(manifest, links_dir, glb_file)
    passed = bool(
        manifest_contract_valid
        and not asset_hash_mismatches
        and visual_hashes["status"] == "PASS"
        and explicit_provenance
        and not custom_failures
        and not effective_failures
        and normal_report["status"] == "PASS"
    )
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "policy_version": POLICY_VERSION,
        "status": "PASS" if passed else "FAIL",
        "passed": passed,
        "production_authorized": False,
        "inputs": {
            "manifest_path": str(manifest_file),
            "manifest_sha256": sha256(manifest_file),
            "manifest_contract_sha256": manifest.get("contract_sha256"),
            "manifest_contract_valid": manifest_contract_valid,
            "adapter_status": manifest.get("status"),
            "adapter_production_authorized": manifest.get(
                "production_authorized"
            ),
        },
        "policy": {
            "custom_requirement": (
                "one watertight shell; zero boundary/nonmanifold/degenerate/"
                "zero-length edges after vertex merge; finite and winding-consistent"
            ),
            "effective_collision_requirement": (
                "nonempty watertight finite winding-consistent mesh with zero "
                "boundary/nonmanifold/degenerate/zero-length edges after merge"
            ),
            "imported_visual_policy": (
                "open exact vendor visuals may remain visible only when the "
                "collision mapping uses a verified conservative overbound"
            ),
            "modeled_hardware_visual_policy": (
                "an explicitly reviewed detailed modeled-hardware visual may "
                "remain visible while collision uses a verified overbound; "
                "custom manufactured geometry is never auto-hulled"
            ),
            "automatic_imported_overbound_allowlist": sorted(
                LEGACY_HULL_ELIGIBLE
            ),
            "automatic_modeled_hardware_overbound_allowlist": sorted(
                LEGACY_MODELED_HULL_ELIGIBLE
            ),
            "whole_hull_risk": (
                "convex hull fills all holes, grooves, concavities, and component "
                "voids; false-positive collisions are expected and must not be "
                "waived without a tighter independently proven envelope"
            ),
        },
        "gates": {
            "manifest_contract_hash_valid": manifest_contract_valid,
            "all_manifest_collision_asset_hashes_valid": not asset_hash_mismatches,
            "all_manifest_visual_and_glb_hashes_valid": (
                visual_hashes["status"] == "PASS"
            ),
            "explicit_per_part_provenance": explicit_provenance,
            "all_custom_parts_strict_topology": not custom_failures,
            "all_effective_collision_meshes_strict_topology": not effective_failures,
            "reference_glb_finite_unit_NORMAL": normal_report["status"] == "PASS",
        },
        "summary": {
            "part_count": len(rows),
            "topology_class_counts": dict(sorted(topology_classes.items())),
            "provenance_counts": dict(sorted(provenance_counts.items())),
            "provenance_source_counts": dict(sorted(provenance_sources.items())),
            "generated_overbound_count": envelope_count,
            "asset_hash_mismatches": asset_hash_mismatches,
            "visual_or_glb_hash_mismatches": visual_hashes["failures"],
            "custom_topology_failures": custom_failures,
            "effective_collision_failures": effective_failures,
        },
        "parts": rows,
        "aggregate_visual_meshes": aggregates,
        "visual_and_glb_hash_audit": visual_hashes,
        "reference_glb_NORMAL": normal_report,
        "integration_contract": {
            "adapter_call": (
                "audit_adapter(manifest_path, envelope_root, "
                "provenance_overrides=explicit_mapping)"
            ),
            "consume": "collision_override_plan(report)",
            "two_pass_export_required": True,
            "draft_manifest_must_be_contract_hashed": True,
            "required_before_final_manifest_hash": True,
            "final_manifest_must_bind": [
                "draft mesh-integrity report hash",
                "exact source-visual hash",
                "effective collision-envelope hash",
                "overbound method and PASS containment proof",
            ],
            "final_effective_assets_must_be_reaudited": True,
            "retain_exact_vendor_visuals": True,
            "collision_mapping_must_reference_overbound_file": True,
            "final_gate": "require_release_ready(report)",
            "legacy_provenance_template_api": "provenance_template(manifest_path)",
        },
        "source_hashes": {
            "cad/collision_mesh_integrity.py": sha256(Path(__file__)),
        },
    }
    report["contract_sha256"] = canonical_hash(report)
    return report


def collision_override_plan(report: Mapping[str, Any]) -> dict[str, Any]:
    """Return verified imported-COTS collision substitutions for an adapter."""

    plan: dict[str, dict[str, Any]] = {}
    for row in report.get("parts", []):
        envelope = row.get("generated_collision_overbound")
        if not envelope:
            continue
        if envelope.get("status") != "PASS":
            raise ValueError(f"failed collision overbound for {row['link']}/{row['label']}")
        asset = envelope["collision_envelope"]
        path = Path(asset["path"])
        if not path.is_file() or sha256(path) != asset.get("sha256"):
            raise ValueError(f"collision overbound hash drift: {path}")
        if envelope["overbound_proof"].get("status") != "PASS":
            raise ValueError(f"collision overbound containment failed: {path}")
        source_visual = Path(envelope["source_visual"]["path"])
        if (
            not source_visual.is_file()
            or sha256(source_visual) != envelope["source_visual"].get("sha256")
        ):
            raise ValueError(f"retained source visual hash drift: {source_visual}")
        plan.setdefault(row["link"], {})[row["label"]] = {
            "collision_mesh_path": str(path),
            "collision_mesh_sha256": asset["sha256"],
            "source_visual_path": envelope["source_visual"]["path"],
            "source_visual_sha256": envelope["source_visual"]["sha256"],
            "method": envelope["method"],
            "overbound_status": envelope["overbound_proof"]["status"],
            "source_visual_retained": True,
            "source_visual_role": envelope.get("source_visual_role"),
            "exact_vendor_visual_retained": envelope.get(
                "exact_vendor_visual_retained", False
            ),
            "detailed_modeled_visual_retained": envelope.get(
                "detailed_modeled_visual_retained", False
            ),
        }
    return plan


def provenance_template(manifest_path: Path | str) -> dict[str, Any]:
    """Return an explicit legacy mapping template requiring source review.

    The template exposes every fallback decision and calls out all inferred
    custom/printed/PEEK/fabricated keys.  Generation does not approve it: final
    adapter source must serialize reviewed values into its part records or pass
    an explicit mapping to :func:`audit_adapter`.
    """

    source = Path(manifest_path).resolve()
    manifest = json.loads(source.read_text(encoding="utf-8"))
    records = manifest.get("parts")
    if not isinstance(records, Mapping):
        raise ValueError("adapter manifest has no collision part mapping")
    mapping: dict[str, dict[str, Any]] = {}
    custom = []
    for link, by_label in records.items():
        if not isinstance(by_label, Mapping):
            raise ValueError(f"malformed collision link {link}")
        for label, record in by_label.items():
            explicit = (
                record.get("provenance_class")
                if isinstance(record, Mapping)
                else None
            )
            selected = (
                explicit if explicit in VALID_PROVENANCE else _legacy_policy(label)
            )
            key = f"{link}/{label}"
            row: dict[str, Any] = {
                "provenance_class": selected,
                "inference_source": (
                    "manifest"
                    if explicit in VALID_PROVENANCE
                    else "legacy_label_review_template"
                ),
                "review_required": explicit not in VALID_PROVENANCE,
            }
            if (
                label in LEGACY_HULL_ELIGIBLE
                or label in LEGACY_MODELED_HULL_ELIGIBLE
            ):
                row["collision_overbound_method"] = HULL_METHOD
            mapping[key] = row
            if selected == PROVENANCE_CUSTOM:
                custom.append(key)
    payload: dict[str, Any] = {
        "schema": "collision-mesh-provenance-template/v1",
        "status": "REVIEW_REQUIRED",
        "production_authorized": False,
        "source_manifest": {
            "path": str(source),
            "sha256": sha256(source),
            "contract_sha256": manifest.get("contract_sha256"),
            "contract_valid": bool(
                manifest.get("contract_sha256") == canonical_hash(manifest)
            ),
        },
        "instructions": (
            "Review every entry against build_adapter_links source; serialize "
            "provenance_class and any collision_overbound_method into final "
            "part records. This template is not release authority."
        ),
        "mapping": dict(sorted(mapping.items())),
        "custom_printed_PEEK_fabricated_keys": sorted(custom),
        "counts": {
            "all_parts": len(mapping),
            "custom_printed_PEEK_fabricated": len(custom),
            "review_required": sum(
                1 for row in mapping.values() if row["review_required"]
            ),
        },
        "source_hashes": {
            "cad/collision_mesh_integrity.py": sha256(Path(__file__)),
        },
    }
    payload["contract_sha256"] = canonical_hash(payload)
    return payload


def require_release_ready(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("collision mesh audit schema drift")
    if report.get("contract_sha256") != canonical_hash(report):
        raise ValueError("collision mesh audit contract hash mismatch")
    if report.get("status") != "PASS" or report.get("passed") is not True:
        raise ValueError("collision mesh integrity gate is not PASS")


def write_report(path: Path | str, report: Mapping[str, Any]) -> Path:
    target = Path(path).resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return target


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fail-closed adapter collision-mesh integrity audit"
    )
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--envelopes", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--glb", type=Path)
    parser.add_argument(
        "--provenance-template",
        type=Path,
        help="write a separate REVIEW_REQUIRED explicit mapping template",
    )
    parser.add_argument(
        "--legacy-frozen-policy",
        action="store_true",
        help=(
            "classify old manifests by audited label fallback; this never "
            "satisfies the explicit-provenance release gate"
        ),
    )
    parser.add_argument("--no-envelopes", action="store_true")
    args = parser.parse_args()
    report = audit_adapter(
        args.manifest,
        args.envelopes,
        glb_path=args.glb,
        allow_legacy_label_fallback=args.legacy_frozen_policy,
        generate_imported_envelopes=not args.no_envelopes,
    )
    target = write_report(args.output, report)
    if args.provenance_template is not None:
        write_report(
            args.provenance_template,
            provenance_template(args.manifest),
        )
    print(target)
    print(
        f"{report['status']}: {report['summary']['part_count']} parts; "
        f"{len(report['summary']['custom_topology_failures'])} custom failures; "
        f"{len(report['summary']['effective_collision_failures'])} effective failures"
    )
    if report["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
