"""Isolated link/visual export contract for the integrated release candidate.

CAD brief
---------
Model
    Secondary STL/GLB player assets derived from
    :mod:`integrated_release_candidate`; the candidate STEP remains the primary
    CAD artifact.
Task type
    Non-destructive export/player adapter.  ``cad/assembly.py`` and the
    canonical ``out/links`` directory are never written.
Coordinate convention
    Existing machine millimetre frame.  M0/M1/M2 are model-space radians;
    M0 translates +Z by ``m0 * PARAMS.mm_per_rad``, M1 rotates about machine Y
    through the translated stator axis, and M2 rotates about machine Z.
Positioning
    The final flyer link already contains the one-piece PEEK hollow-shaft-to-tip
    guide and all six positive balance stacks.  The carriage owns the fixed
    active-sector guide pair/yoke; the spindle owns the two short-leadin caps.
Outputs
    ``out/review/integrated_adapter/links/*.stl``, a player-compatible manifest,
    a PBR reference-pose GLB, and draft/final collision-integrity reports.  All
    outputs are review-only.
Validation
    Exact link labels, non-overlapping material groups, source/asset hashes,
    sampled candidate/upstream/player-hierarchy transform matrices, explicit
    per-part provenance, verified conservative overbounds for the exact
    allowlist, and a second audit of the final effective collision mapping.

This module deliberately has no authority to promote the candidate or to mark
the still-open conductor and hardware gates complete.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
from build123d import Compound, Part, export_stl

import assembly
import coil_growth
import collision_mesh_integrity as mesh_integrity
import integrated_release_candidate as candidate
from params import DEFAULT_SPINDLE_ID, DEFAULT_STATOR, PARAMS as P, spindle_option
import retained_flyer_peek_guide_successor as flyer_successor
import wire_geometry
import wire_vis


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DEFAULT_OUTPUT = ROOT / "out" / "review" / "integrated_adapter"
RELEASE_ROOT = ROOT / "out" / "review" / "integrated_adapter_releases"
RELEASE_IDENTITY_NAME = "release_identity.json"
LINK_NAMES = ("static", "carriage", "spindle", "flyer")
SCHEMA = "integrated-candidate-player-adapter/v1"
MATRIX_TOLERANCE = 1.0e-10
WIRE_CHAIN_TOLERANCE_MM = 2.0e-6
DRAFT_MANIFEST_NAME = "manifest.draft.json"
DRAFT_COLLISION_AUDIT_NAME = "collision_mesh_integrity.draft.json"
FINAL_COLLISION_AUDIT_NAME = "collision_mesh_integrity.final.json"
ACTIVE_TERMINAL_LOCI_NAME = "carriage_active_sector_terminal_guide_loci.json"
ACTIVE_TERMINAL_LOCI_SOURCE = ROOT / "out" / "reports" / ACTIVE_TERMINAL_LOCI_NAME
ACTIVE_TERMINAL_LOCI_SCHEMA = "carriage-active-sector-terminal-guide-loci/v1"
EXPECTED_ACTIVE_TERMINAL_LOCI = 24 * 100
FLYER_REFERENCE_SEGMENT = "flyer_geometric_bore"
FLYER_REFERENCE_SAMPLES_FIELD = (
    "geometric_bore_to_tensioned_handoff_local_samples_mm"
)
M2_BELT_LABEL = "m2_successor_210_3gt_6_belt"
M2_MOTOR_PULLEY_LABEL = (
    "NBK_P30_3GT_BLP_6C_5_stock_split_clamp_vendor_occurrence"
)
M2_FLYER_PULLEY_LABEL = (
    "NBK_P30_3GT_BLP_6C_10_stock_hub_rear_vendor_occurrence"
)
LEGACY_RELEASE_CLOSURE_INPUTS_V1 = (
    "cad/integrated_export_player_adapter.py",
    "cad/integrated_release_candidate.py",
    "cad/assembly.py",
    "cad/collision_mesh_integrity.py",
    "out/reports/integrated_release_candidate.json",
    "out/reports/carriage_active_sector_terminal_guide_loci.json",
)
RELEASE_CLOSURE_INPUTS = (
    *LEGACY_RELEASE_CLOSURE_INPUTS_V1,
    "sim/animate.py",
    "sim/player_template.html",
)
RELEASE_ID_PREFIX = "iar1-"
RELEASE_ID_HASH_LENGTH = 20

# Source-reviewed collision substitutions.  These are the current four open or
# nonmanifold detailed meshes.  Eligibility is exact (link, label), never a
# broad provenance rule: all custom/printed/PEEK/fabricated/guide geometry is
# required to serialize cleanly and can never enter this set implicitly.
COLLISION_OVERBOUND_ALLOWLIST = frozenset({
    ("static", "m2_Leadshine_CS-M21708_exact_cableless"),
    ("static", "felt_m4_wingnut"),
    ("carriage", "mgn12h_L"),
    ("carriage", "mgn12h_R"),
})

CUSTOM_TOPOLOGY_LABEL_RE = re.compile(
    r"(?i)(peek|petg|fabricated|printed|printable|custom|guide|crown|"
    r"active[_-]?sector)"
)


MATERIALS: dict[str, dict[str, Any]] = {
    "static_link": {
        "color_rgba": [0.78, 0.80, 0.83, 1.0],
        "metallic": 0.05,
        "roughness": 0.75,
        "double_sided": False,
        "description": "aggregate fixed machine structure",
    },
    "carriage_link": {
        "color_rgba": [0.45, 0.60, 0.90, 1.0],
        "metallic": 0.05,
        "roughness": 0.70,
        "double_sided": False,
        "description": "aggregate M0 carriage",
    },
    "spindle_link": {
        "color_rgba": [0.95, 0.65, 0.25, 1.0],
        "metallic": 0.05,
        "roughness": 0.72,
        "double_sided": False,
        "description": "aggregate M1 spindle and stator",
    },
    "flyer_link": {
        "color_rgba": [0.90, 0.32, 0.35, 1.0],
        "metallic": 0.05,
        "roughness": 0.72,
        "double_sided": False,
        "description": "aggregate M2 flyer",
    },
    "felt_dark_brown": {
        "color_rgba": [0.22, 0.075, 0.025, 1.0],
        "metallic": 0.0,
        "roughness": 1.0,
        "double_sided": True,
        "description": "wool felt",
    },
    "belt_dark_rubber": {
        "color_rgba": [0.045, 0.052, 0.060, 1.0],
        "metallic": 0.0,
        "roughness": 0.92,
        "double_sided": True,
        "description": "dark reinforced 210-3GT drive belt",
    },
    "pulley_aluminum": {
        "color_rgba": [0.66, 0.70, 0.76, 1.0],
        "metallic": 0.82,
        "roughness": 0.27,
        "double_sided": False,
        "description": "stock metallic NBK P30 pulley",
    },
    "peek_natural": {
        "color_rgba": [0.86, 0.78, 0.56, 1.0],
        "metallic": 0.0,
        "roughness": 0.58,
        "double_sided": False,
        "description": "natural unfilled PEEK",
    },
    "machined_aluminum": {
        "color_rgba": [0.68, 0.72, 0.78, 1.0],
        "metallic": 0.72,
        "roughness": 0.34,
        "double_sided": False,
        "description": "machined aluminum guide support",
    },
    "stainless_hardware": {
        "color_rgba": [0.72, 0.75, 0.80, 1.0],
        "metallic": 0.78,
        "roughness": 0.26,
        "double_sided": False,
        "description": "steel and stainless retention hardware",
    },
    "tungsten_dark": {
        "color_rgba": [0.23, 0.25, 0.27, 1.0],
        "metallic": 0.88,
        "roughness": 0.30,
        "double_sided": False,
        "description": "ASTM B777 tungsten balance slugs",
    },
    "petg_retainer": {
        "color_rgba": [0.20, 0.23, 0.28, 1.0],
        "metallic": 0.0,
        "roughness": 0.82,
        "double_sided": False,
        "description": "printed PETG counterweight retainers",
    },
    "enameled_copper": {
        "color_rgba": [0.82, 0.30, 0.055, 1.0],
        "metallic": 0.12,
        "roughness": 0.48,
        "double_sided": True,
        "description": "true-diameter enameled copper conductor witness",
    },
}

LINK_MATERIALS = {name: f"{name}_link" for name in LINK_NAMES}


@dataclass(frozen=True)
class AdapterOverrides:
    """Legacy negative-test hooks plus isolated explicit additions.

    ``flyer_tip_guide`` is intentionally rejected by the final candidate
    because replacing the integrated one-piece PEEK guide invalidates balance.
    ``terminal_crown_parts`` are physical spindle-link test occurrences.
    ``link_additions`` supports other explicit physical review occurrences.
    A wire map replaces only the listed owners, leaving other candidate wire
    witnesses intact.  Explicit material labels take precedence over the
    built-in semantic label rules.
    """

    flyer_tip_guide: Part | Compound | None = None
    terminal_crown_parts: tuple[Part | Compound, ...] = ()
    link_additions: Mapping[str, tuple[Part | Compound, ...]] = field(
        default_factory=dict
    )
    wire_visuals_by_link: Mapping[str, tuple[Part | Compound, ...]] = field(
        default_factory=dict
    )
    material_by_label: Mapping[str, str] = field(default_factory=dict)
    # Keys are exact ``<link>/<final occurrence label>`` strings.  Every
    # physical override must be classified here; visual wire witnesses are not
    # collision occurrences and are intentionally excluded.
    provenance_by_occurrence: Mapping[str, str] = field(default_factory=dict)

    @classmethod
    def terminal_guides(
        cls,
        *,
        flyer_tip_guide: Part | Compound,
        terminal_crown_parts: Sequence[Part | Compound],
        wire_visuals_by_link: Mapping[
            str, Sequence[Part | Compound]
        ] | None = None,
    ) -> "AdapterOverrides":
        """Make the intended flyer-guide + spindle-crown ownership explicit."""

        crowns = tuple(terminal_crown_parts)
        material_by_label = {"terminal_tip_guide_override": "peek_natural"}
        provenance_by_occurrence = {
            "flyer/terminal_tip_guide_override": mesh_integrity.PROVENANCE_CUSTOM
        }
        for index, shape in enumerate(crowns):
            label = _label(shape)
            if not label:
                raise ValueError(f"terminal crown occurrence {index} is unlabeled")
            material_by_label[label] = "peek_natural"
            provenance_by_occurrence[
                f"spindle/{label}"
            ] = mesh_integrity.PROVENANCE_CUSTOM
        wires = {
            owner: tuple(parts)
            for owner, parts in (wire_visuals_by_link or {}).items()
        }
        return cls(
            flyer_tip_guide=flyer_tip_guide,
            terminal_crown_parts=crowns,
            wire_visuals_by_link=wires,
            material_by_label=material_by_label,
            provenance_by_occurrence=provenance_by_occurrence,
        )


@dataclass(frozen=True)
class CollisionPipelineResult:
    manifest: dict[str, Any]
    draft_report: dict[str, Any]
    final_report: dict[str, Any]
    draft_manifest_path: Path
    draft_report_path: Path
    final_manifest_path: Path
    final_report_path: Path


@dataclass(frozen=True)
class _VisualRule:
    name: str
    material: str
    description: str

    def matches(self, link: str, label: str) -> bool:
        if self.name == "felt_pads":
            return link == "static" and label in {
                "felt_pad_fixed", "felt_pad_moving"
            }
        if self.name == "m2_drive_belt":
            return link == "static" and label == M2_BELT_LABEL
        if self.name == "m2_motor_pulley":
            return link == "static" and label == M2_MOTOR_PULLEY_LABEL
        if self.name == "m2_flyer_pulley":
            return link == "flyer" and label == M2_FLYER_PULLEY_LABEL
        if self.name == "peek_caps":
            return link == "spindle" and label in {
                "front_one_solid_PEEK_cap_with_short_open_leadins",
                "rear_one_solid_PEEK_cap_with_short_open_leadins",
            }
        if self.name == "cap_retention_hardware":
            return link == "spindle" and label.startswith("cap_retention_")
        if self.name == "counterweight_tungsten":
            return link == "flyer" and (
                "ASTM_B777_tungsten_slug" in label
                or label.startswith("front_balance_B777_annular_slug_")
            )
        if self.name == "counterweight_retainers":
            return link == "flyer" and (
                "printed_retainer_face_boss_three_point_spacer" in label
            )
        if self.name == "counterweight_retention_hardware":
            return link == "flyer" and (
                "McMaster_94459A130" in label
                or "McMaster_92125A126" in label
                or label.startswith("front_balance_M2_washer_")
                or label.startswith("front_balance_ISO4762_M2x8_")
                or label.startswith("front_balance_M2_heat_insert_")
            )
        if self.name == "flyer_peek_guide":
            return (
                link == "flyer"
                and label == "one_piece_polished_unfilled_PEEK_shaft_to_tip_guide"
            )
        if self.name == "flyer_peek_guide_retention_hardware":
            return link == "flyer" and label.startswith("PEEK_guide_")
        if self.name == "active_sector_peek_guides":
            return (
                link == "carriage"
                and label.endswith(
                    "_M0_following_M1_static_PEEK_active_sector"
                )
            )
        if self.name == "active_sector_yoke":
            return (
                link == "carriage"
                and label == "M0_carriage_owned_aluminum_active_sector_split_yoke"
            )
        if self.name == "active_sector_retention_hardware":
            return link == "carriage" and label.startswith("active_sector_")
        return False


VISUAL_RULES = (
    _VisualRule("felt_pads", "felt_dark_brown", "fixed and moving felt pads"),
    _VisualRule(
        "m2_drive_belt",
        "belt_dark_rubber",
        "dark 210-3GT belt; pulley overlap is intended tooth engagement",
    ),
    _VisualRule(
        "m2_motor_pulley",
        "pulley_aluminum",
        "metallic stock motor-side NBK P30 pulley",
    ),
    _VisualRule(
        "m2_flyer_pulley",
        "pulley_aluminum",
        "metallic stock flyer-side NBK P30 D10 pulley",
    ),
    _VisualRule("peek_caps", "peek_natural", "front and rear production PEEK caps"),
    _VisualRule(
        "cap_retention_hardware",
        "stainless_hardware",
        "six complete short-leadin cap-retention occurrences",
    ),
    _VisualRule(
        "counterweight_tungsten",
        "tungsten_dark",
        "four retained rear slugs plus two annular front trim slugs",
    ),
    _VisualRule(
        "counterweight_retainers",
        "petg_retainer",
        "four printed positive-retention caps with spacer posts",
    ),
    _VisualRule(
        "counterweight_retention_hardware",
        "stainless_hardware",
        "four rear insert/screw pairs plus two front washer/screw/insert stacks",
    ),
    _VisualRule(
        "flyer_peek_guide",
        "peek_natural",
        "one-piece polished PEEK hollow-shaft-to-tip guide and exit bell",
    ),
    _VisualRule(
        "flyer_peek_guide_retention_hardware",
        "stainless_hardware",
        "three positive PEEK-guide screw/insert stacks",
    ),
    _VisualRule(
        "active_sector_peek_guides",
        "peek_natural",
        "front and rear M0-following active-sector PEEK guides",
    ),
    _VisualRule(
        "active_sector_yoke",
        "machined_aluminum",
        "machined split yoke supporting the active-sector guides",
    ),
    _VisualRule(
        "active_sector_retention_hardware",
        "stainless_hardware",
        "active-sector guide/yoke/tower retention hardware",
    ),
)


def _label(shape: Part | Compound) -> str:
    return str(getattr(shape, "label", "") or "")


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def release_source_hashes(
    inputs: tuple[str, ...] = RELEASE_CLOSURE_INPUTS,
) -> dict[str, str]:
    """Return the exact ordered source closure used by immutable releases."""

    hashes: dict[str, str] = {}
    for relative in inputs:
        digest = _sha256(ROOT / relative)
        if digest is None:
            raise FileNotFoundError(f"release closure input is missing: {relative}")
        hashes[relative] = digest
    return hashes


def _release_closure_identity_for_inputs(
    source_hashes: Mapping[str, str], inputs: tuple[str, ...],
) -> dict[str, Any]:
    """Hash one explicitly ordered closure (also used by history tests)."""

    hashes = dict(source_hashes)
    if set(hashes) != set(inputs):
        raise ValueError("release closure source set drift")
    normalized: dict[str, str] = {}
    for relative in inputs:
        digest = str(hashes[relative]).lower()
        if re.fullmatch(r"[0-9a-f]{64}", digest) is None:
            raise ValueError(f"invalid release closure hash for {relative}")
        normalized[relative] = digest
    payload = "".join(
        f"{relative}={normalized[relative]}\n" for relative in inputs
    )
    closure_sha256 = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    return {
        "release_id": RELEASE_ID_PREFIX + closure_sha256[:RELEASE_ID_HASH_LENGTH],
        "closure_sha256": closure_sha256,
        "source_hashes": normalized,
        "payload_format": "ordered relative=lowercase_sha256 lines with final LF",
    }


def release_closure_identity(
    source_hashes: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Compute the stable ``iar1-<20 hex>`` identity for a source closure.

    The full digest is SHA-256 of ordered UTF-8 lines
    ``relative=lowercase_sha256\n``.  It is intentionally distinct from the
    manifest contract hash, which binds the exported asset manifest.
    """

    hashes = (
        release_source_hashes() if source_hashes is None else dict(source_hashes)
    )
    return _release_closure_identity_for_inputs(
        hashes, RELEASE_CLOSURE_INPUTS
    )


def _validated_release_output(
    output_dir: Path | str | None,
    identity: Mapping[str, Any],
) -> Path:
    """Return a safe export path without following the canonical selector.

    ``integrated_adapter`` is a review selector implemented as a directory
    junction on Windows.  Resolving it before validation silently converts an
    export into an overwrite of the previously selected immutable release.
    Exports therefore default to the current identity-named release directory
    and refuse the selector (or any of its descendants) as a write target.
    """

    expected_id = str(identity.get("release_id", ""))
    if re.fullmatch(r"iar1-[0-9a-f]{20}", expected_id) is None:
        raise ValueError("invalid integrated adapter release identity")
    requested = (
        RELEASE_ROOT / expected_id if output_dir is None else Path(output_dir)
    )
    lexical = requested if requested.is_absolute() else Path.cwd() / requested
    lexical = lexical.absolute()
    selector = DEFAULT_OUTPUT.absolute()
    if lexical == selector or selector in lexical.parents:
        raise ValueError(
            "integrated adapter refuses to export through canonical review "
            "selector; export to the identity-named release directory"
        )
    release_root = RELEASE_ROOT.absolute()
    try:
        relative = lexical.relative_to(release_root)
    except ValueError:
        relative = None
    if relative is not None and len(relative.parts) == 1:
        if relative.name != expected_id:
            raise ValueError(
                "integrated adapter immutable release directory does not "
                f"match current identity {expected_id}"
            )
    return lexical.resolve()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    value = dict(payload)
    value.pop("contract_sha256", None)
    body = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(body).hexdigest()


def _active_terminal_payload_hash(payload: Mapping[str, Any]) -> str:
    value = dict(payload)
    value.pop("locus_payload_sha256", None)
    body = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(body).hexdigest()


def stage_active_terminal_loci(
    output_dir: Path | str,
    source_path: Path | str = ACTIVE_TERMINAL_LOCI_SOURCE,
) -> dict[str, Any]:
    """Copy the exact sampled terminal-route API into an isolated adapter.

    The staged file is a byte-for-byte copy so the browser consumes the same
    hash-bound artifact as the active-sector audit.  Its authority is limited
    to the 2,400 winding loci; flexible-wire dynamics and non-winding
    transitions stay explicitly unproved.
    """

    source = Path(source_path).resolve()
    if not source.is_file():
        raise ValueError(f"active terminal locus artifact is missing: {source}")
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("active terminal locus artifact is not one JSON object")
    if payload.get("schema") != ACTIVE_TERMINAL_LOCI_SCHEMA:
        raise ValueError("active terminal locus schema drift")
    loci = payload.get("loci")
    run = payload.get("run")
    if (not isinstance(loci, list)
            or len(loci) != EXPECTED_ACTIVE_TERMINAL_LOCI
            or not isinstance(run, Mapping)
            or run.get("locus_count") != EXPECTED_ACTIVE_TERMINAL_LOCI):
        raise ValueError("active terminal locus artifact is not exactly 2,400 loci")
    payload_hash = payload.get("locus_payload_sha256")
    if (not isinstance(payload_hash, str)
            or payload_hash != _active_terminal_payload_hash(payload)):
        raise ValueError("active terminal locus payload hash drift")
    if "torus" in json.dumps(payload, sort_keys=True).lower():
        raise ValueError("obsolete torus metadata leaked into terminal loci")
    segment_contract = payload.get("segment_contract")
    flyer_reference = payload.get("flyer_reference")
    if (
        not isinstance(segment_contract, Mapping)
        or FLYER_REFERENCE_SEGMENT not in segment_contract
        or not isinstance(flyer_reference, Mapping)
        or flyer_reference.get("frame") != "flyer_reference_M2_axis_plus_Z"
    ):
        raise ValueError("active terminal flyer reference contract is missing")
    if any(
        not isinstance(contract, Mapping)
        or any(not isinstance(contract.get(field), str)
               for field in ("surface_owner", "local_frame", "authority"))
        for contract in segment_contract.values()
    ):
        raise ValueError("active terminal global segment metadata is malformed")
    flyer_samples = flyer_reference.get(FLYER_REFERENCE_SAMPLES_FIELD)
    if not isinstance(flyer_samples, list) or len(flyer_samples) < 2:
        raise ValueError("active terminal flyer reference polyline is missing")
    full_flyer_samples = flyer_reference.get(
        "full_geometric_bore_local_samples_mm"
    )
    if (
        not isinstance(full_flyer_samples, list)
        or flyer_reference.get("full_geometric_bore_point_count")
        != len(full_flyer_samples)
        or flyer_reference.get("conductor_prefix_point_count")
        != len(flyer_samples)
        or full_flyer_samples[:len(flyer_samples)] != flyer_samples
    ):
        raise ValueError("active terminal flyer reference prefix drift")
    if any(
        segment.get("name") == FLYER_REFERENCE_SEGMENT
        for locus in loci
        for segment in locus.get("segments", [])
    ):
        raise ValueError("active terminal flyer reference is redundantly repeated")
    maximum_edges = len(flyer_samples) - 1 + max(
        sum(
            len(segment["machine_world_samples_mm"]) - 1
            for segment in locus["segments"]
        )
        for locus in loci
    )
    if maximum_edges <= 0:
        raise ValueError("active terminal locus polylines are empty")

    output = Path(output_dir).resolve()
    target = output / "reports" / ACTIVE_TERMINAL_LOCI_NAME
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    try:
        source_display = str(source.relative_to(ROOT)).replace("\\", "/")
    except ValueError:
        source_display = str(source)
    return {
        "schema": ACTIVE_TERMINAL_LOCI_SCHEMA,
        "file": str(target.relative_to(output)).replace("\\", "/"),
        "source": source_display,
        "artifact_sha256": _sha256(target),
        "locus_payload_sha256": payload_hash,
        "locus_count": len(loci),
        "maximum_polyline_edges_per_locus": maximum_edges,
        "authority": "exact sampled active terminal route at raw half-turn starts",
        "held_between_loci_for_review_only": True,
        "park_index_load_unload_proven": False,
        "sag_tension_settling_neatness_proven": False,
    }


def _safe_name(label: str) -> str:
    name = re.sub(r"[^A-Za-z0-9_.-]+", "_", label).strip("._")
    return name[:140] or "unlabeled"


def _override_occurrence_keys(selected: AdapterOverrides) -> set[str]:
    keys: set[str] = set()
    if selected.flyer_tip_guide is not None:
        keys.add("flyer/terminal_tip_guide_override")
    for shape in selected.terminal_crown_parts:
        keys.add(f"spindle/{_label(shape)}")
    for link, additions in selected.link_additions.items():
        keys.update(f"{link}/{_label(shape)}" for shape in additions)
    return keys


def validate_override_provenance(selected: AdapterOverrides) -> set[str]:
    """Require source-reviewed provenance for every physical override."""

    keys = _override_occurrence_keys(selected)
    declared = set(selected.provenance_by_occurrence)
    missing = sorted(keys - declared)
    extra = sorted(declared - keys)
    if missing:
        raise ValueError(
            "physical adapter overrides lack explicit provenance: "
            + ", ".join(missing)
        )
    if extra:
        raise ValueError(
            "override provenance names absent physical occurrences: "
            + ", ".join(extra)
        )
    for key, provenance in selected.provenance_by_occurrence.items():
        if provenance not in mesh_integrity.VALID_PROVENANCE:
            raise ValueError(f"invalid override provenance {provenance!r}: {key}")
        label = key.split("/", 1)[1]
        if (
            CUSTOM_TOPOLOGY_LABEL_RE.search(label)
            and provenance != mesh_integrity.PROVENANCE_CUSTOM
        ):
            raise ValueError(
                f"custom/printed/PEEK/fabricated/guide override must use "
                f"strict custom provenance: {key}"
            )
    return keys


def part_provenance(
    link: str,
    label: str,
    selected: AdapterOverrides | None = None,
    *,
    override_keys: set[str] | None = None,
) -> tuple[str, str]:
    """Return the explicit manifest class and its source-level policy origin."""

    config = selected or AdapterOverrides()
    key = f"{link}/{label}"
    keys = override_keys if override_keys is not None else validate_override_provenance(config)
    if key in keys:
        provenance = config.provenance_by_occurrence[key]
        source = "explicit_adapter_override"
    elif label in mesh_integrity.LEGACY_IMPORTED_COTS:
        provenance = mesh_integrity.PROVENANCE_IMPORTED
        source = "reviewed_base_imported_COTS"
    elif (
        label in mesh_integrity.LEGACY_CUSTOM_EXACT
        or mesh_integrity.CUSTOM_LABEL_RE.search(label)
        or CUSTOM_TOPOLOGY_LABEL_RE.search(label)
    ):
        provenance = mesh_integrity.PROVENANCE_CUSTOM
        source = "strict_custom_printed_PEEK_fabricated_policy"
    else:
        provenance = mesh_integrity.PROVENANCE_MODELED
        source = "reviewed_base_modeled_COTS_or_hardware"
    if provenance not in mesh_integrity.VALID_PROVENANCE:
        raise RuntimeError(f"unclassified adapter occurrence: {key}")
    if (
        CUSTOM_TOPOLOGY_LABEL_RE.search(label)
        and provenance != mesh_integrity.PROVENANCE_CUSTOM
    ):
        raise RuntimeError(f"custom topology policy bypass: {key}")
    return provenance, source


def collision_overbound_method(
    link: str, label: str, provenance: str
) -> str | None:
    """Return the exact reviewed method; custom geometry is never hullable."""

    if provenance == mesh_integrity.PROVENANCE_CUSTOM:
        if (link, label) in COLLISION_OVERBOUND_ALLOWLIST:
            raise RuntimeError(f"custom occurrence entered overbound allowlist: {link}/{label}")
        return None
    if (link, label) in COLLISION_OVERBOUND_ALLOWLIST:
        return mesh_integrity.HULL_METHOD
    return None


def _validate_link_mapping(
    mapping: Mapping[str, Iterable[Part | Compound]], field_name: str
) -> None:
    unknown = set(mapping) - set(LINK_NAMES)
    if unknown:
        raise ValueError(f"{field_name} has unknown links: {sorted(unknown)}")


def _require_unique_labels(
    links: Mapping[str, Sequence[Part | Compound]],
) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    for link in LINK_NAMES:
        labels = [_label(shape) for shape in links[link]]
        missing = [index for index, label in enumerate(labels) if not label]
        if missing:
            raise ValueError(f"{link} contains unlabeled occurrences {missing}")
        duplicates = sorted({label for label in labels if labels.count(label) > 1})
        if duplicates:
            raise ValueError(f"{link} contains duplicate labels: {duplicates}")
        result[link] = labels
    return result


def build_adapter_links(
    overrides: AdapterOverrides | None = None,
) -> dict[str, list[Part | Compound]]:
    """Build candidate physical links with isolated, explicit additions."""

    selected = overrides or AdapterOverrides()
    _validate_link_mapping(selected.link_additions, "link_additions")
    validate_override_provenance(selected)
    links = {
        name: list(parts)
        for name, parts in candidate.build_links(
            tip_guide_override=selected.flyer_tip_guide
        ).items()
    }
    for shape in selected.terminal_crown_parts:
        links["spindle"].append(shape)
    for link, additions in selected.link_additions.items():
        links[link].extend(additions)
    if set(links) != set(LINK_NAMES):
        raise RuntimeError("candidate four-link contract drift")
    _require_unique_labels(links)
    return links


def build_adapter_wire_visuals(
    overrides: AdapterOverrides | None = None,
) -> dict[str, list[Part | Compound]]:
    selected = overrides or AdapterOverrides()
    _validate_link_mapping(
        selected.wire_visuals_by_link, "wire_visuals_by_link"
    )
    wires = {name: list(parts) for name, parts in candidate.wire_visuals().items()}
    for link, replacement in selected.wire_visuals_by_link.items():
        wires[link] = list(replacement)
    for required in ("static", "flyer"):
        if not wires.get(required):
            raise ValueError(f"player adapter requires a {required} wire visual")
    for link, parts in wires.items():
        labels = [_label(shape) for shape in parts]
        if not all(labels):
            raise ValueError(f"{link} wire visuals contain an unlabeled occurrence")
    return wires


def _group_for_label(
    link: str,
    label: str,
    explicit_materials: Mapping[str, str],
) -> tuple[str, str, str] | None:
    material = explicit_materials.get(label)
    if material is not None:
        if material not in MATERIALS:
            raise ValueError(f"unknown material {material!r} for {label!r}")
        return (
            f"override_{link}_{_safe_name(material)}",
            material,
            "explicit guide/crown override material group",
        )
    matches = [rule for rule in VISUAL_RULES if rule.matches(link, label)]
    if len(matches) > 1:
        raise RuntimeError(f"visual label {label!r} matches multiple groups")
    if not matches:
        return None
    rule = matches[0]
    return rule.name, rule.material, rule.description


def split_visual_parts(
    link: str,
    parts: Sequence[Part | Compound],
    explicit_materials: Mapping[str, str] | None = None,
) -> tuple[list[Part | Compound], dict[str, dict[str, Any]]]:
    """Separate material overlays from the aggregate mesh without duplicates."""

    base: list[Part | Compound] = []
    groups: dict[str, dict[str, Any]] = {}
    for shape in parts:
        label = _label(shape)
        selection = _group_for_label(link, label, explicit_materials or {})
        if selection is None:
            base.append(shape)
            continue
        name, material, description = selection
        record = groups.setdefault(
            name,
            {"material": material, "description": description, "parts": []},
        )
        if record["material"] != material:
            raise RuntimeError(f"visual group {name!r} material drift")
        record["parts"].append(shape)
    if not base:
        raise RuntimeError(f"{link} aggregate visual mesh would be empty")
    return base, groups


def _location_matrix(location: Any) -> np.ndarray:
    transform = location.wrapped.Transformation()
    result = np.eye(4, dtype=float)
    for row in range(3):
        for column in range(4):
            result[row, column] = float(transform.Value(row + 1, column + 1))
    return result


def _translate(x: float, y: float, z: float) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, 3] = [x, y, z]
    return result


def _rotate_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray(
        [[c, 0.0, s, 0.0], [0.0, 1.0, 0.0, 0.0],
         [-s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0]],
        dtype=float,
    )


def _rotate_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray(
        [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0],
         [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        dtype=float,
    )


def player_hierarchy_matrix(
    link: str, m0: float, m1: float, m2: float
) -> np.ndarray:
    """World matrix produced by the existing browser player's node hierarchy."""

    dz = float(m0) * P.mm_per_rad
    if link == "static":
        return np.eye(4, dtype=float)
    if link == "carriage":
        return _translate(0.0, 0.0, dz)
    if link == "spindle":
        return (
            _translate(0.0, 0.0, dz)
            @ _translate(0.0, 0.0, P.m0_home_standoff)
            @ _rotate_y(float(m1))
            @ _translate(0.0, 0.0, -P.m0_home_standoff)
        )
    if link == "flyer":
        return _rotate_z(float(m2))
    raise ValueError(link)


DEFAULT_KINEMATIC_SAMPLES = (
    (0.0, 0.0, 0.0),
    (-0.125, math.radians(7.0), math.radians(-13.0)),
    (-1.2, 0.7, -0.3),
    (-5.75, -math.pi / 2.0, math.pi),
    (-11.0, 2.0 * math.pi + 0.17, -4.0 * math.pi - 0.41),
)


def kinematic_equivalence_report(
    samples: Sequence[tuple[float, float, float]] = DEFAULT_KINEMATIC_SAMPLES,
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    max_candidate_upstream = 0.0
    max_candidate_player = 0.0
    for sample_index, (m0, m1, m2) in enumerate(samples):
        for link in LINK_NAMES:
            integrated = _location_matrix(
                candidate.link_location(link, m0=m0, m1=m1, m2=m2)
            )
            upstream = _location_matrix(
                assembly.link_location(link, m0=m0, m1=m1, m2=m2)
            )
            player = player_hierarchy_matrix(link, m0, m1, m2)
            upstream_delta = float(np.max(np.abs(integrated - upstream)))
            player_delta = float(np.max(np.abs(integrated - player)))
            max_candidate_upstream = max(max_candidate_upstream, upstream_delta)
            max_candidate_player = max(max_candidate_player, player_delta)
            rows.append({
                "sample_index": sample_index,
                "link": link,
                "m0_rad": float(m0),
                "m1_rad": float(m1),
                "m2_rad": float(m2),
                "candidate_vs_upstream_max_abs": upstream_delta,
                "candidate_vs_player_max_abs": player_delta,
            })
    sample_payload = [[float(v) for v in sample] for sample in samples]
    sample_sha = hashlib.sha256(
        json.dumps(sample_payload, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return {
        "status": (
            "PASS"
            if max(max_candidate_upstream, max_candidate_player)
            <= MATRIX_TOLERANCE
            else "FAIL"
        ),
        "matrix_tolerance": MATRIX_TOLERANCE,
        "sample_count": len(samples),
        "link_sample_count": len(rows),
        "sample_sha256": sample_sha,
        "candidate_vs_upstream_max_abs": max_candidate_upstream,
        "candidate_vs_player_hierarchy_max_abs": max_candidate_player,
        "mm_per_rad_m0": P.mm_per_rad,
        "m0_home_standoff_mm": P.m0_home_standoff,
        "rows": rows,
    }


def _bounds(shape: Part | Compound) -> list[list[float]]:
    box = shape.bounding_box()
    return [
        [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        [float(box.max.X), float(box.max.Y), float(box.max.Z)],
    ]


def _wire_point(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) \
            or len(value) != 3:
        raise ValueError(f"{label} must be one XYZ point")
    point = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in point):
        raise ValueError(f"{label} must be finite")
    return point


def _wire_point_gap(left: Any, right: Any) -> float:
    a = _wire_point(left, "wire point")
    b = _wire_point(right, "wire point")
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def _configured_static_wire_manifest() -> dict[str, Any]:
    """Describe the exact centerline used by the candidate static wire mesh.

    The legacy :func:`wire_geometry.static_path_spec` stops at the old shaft
    rear mouth and uses the maximum-wire dancer radius.  The integrated
    candidate instead renders the actual 0.22352 mm job wire in contact with
    the felt and dancer, then continues its on-axis run through the complete
    hollow shaft to the one-piece PEEK guide root.  Reproduce those same
    source equations here so the exported manifest cannot advertise the old
    rear-mouth endpoint while its STL visibly reaches the guide.
    """

    centerline_wire_radius = float(wire_vis.R_VIS)
    path_radius = wire_geometry.DANCER_BODY_RADIUS + centerline_wire_radius
    z_plane = float(candidate.CONFIGURED_WIRE_PLANE_Z_MM)
    dancer_center = (
        float(P.dancer_pulley_x), float(P.dancer_pulley_y), z_plane,
    )
    theta_in = math.pi
    theta_out = theta_in - math.radians(wire_geometry.DANCER_WRAP_DEG)

    def on_pulley(theta: float) -> tuple[float, float, float]:
        return (
            dancer_center[0] + path_radius * math.cos(theta),
            dancer_center[1] + path_radius * math.sin(theta),
            z_plane,
        )

    tangent_in = on_pulley(theta_in)
    tangent_out = on_pulley(theta_out)
    spool = (tangent_in[0], float(P.spool_y), z_plane)
    felt = (tangent_in[0], float(P.felt_y), z_plane)
    felt_guide_in = (tangent_in[0], float(P.felt_y) - 15.0, z_plane)
    entry_corner = (0.0, 0.0, z_plane)
    entry_incoming = wire_geometry._unit(  # noqa: SLF001 - shared equation
        wire_geometry._sub(entry_corner, tangent_out)  # noqa: SLF001
    )
    entry_arc, entry_meta = wire_geometry._circular_fillet(  # noqa: SLF001
        entry_corner,
        entry_incoming,
        (0.0, 0.0, 1.0),
        wire_geometry.ENTRY_BEND_RADIUS,
    )
    arc_count = max(
        2,
        math.ceil(wire_geometry.DANCER_WRAP_DEG / wire_geometry.ARC_STEP_DEG),
    )
    dancer_arc = [
        on_pulley(theta_in + (theta_out - theta_in) * index / arc_count)
        for index in range(arc_count + 1)
    ]

    entry_eyelet = (
        0.0,
        0.0,
        float(P.wire_entry_z + 3.0 - candidate.ENTRY_REAR_SHIFT_MM),
    )
    shaft_bore_rear = (
        0.0, 0.0, float(candidate.flyer_shaft_d10.WORLD_REAR_Z_MM),
    )
    guide_root = (
        0.0, 0.0, float(flyer_successor.GUIDE_ROOT_AXIAL_START_Z_MM),
    )
    axial_points = [entry_arc[-1], entry_eyelet, shaft_bore_rear, guide_root]
    if any(
        abs(point[0]) > WIRE_CHAIN_TOLERANCE_MM
        or abs(point[1]) > WIRE_CHAIN_TOLERANCE_MM
        for point in axial_points
    ) or any(
        left[2] >= right[2] - WIRE_CHAIN_TOLERANCE_MM
        for left, right in zip(axial_points, axial_points[1:])
    ):
        raise RuntimeError("configured static wire shaft-axis run drifted")

    points = wire_geometry._dedupe([  # noqa: SLF001 - shared sampler
        spool,
        felt_guide_in,
        felt,
        tangent_in,
        *dancer_arc[1:],
        *entry_arc,
        entry_eyelet,
        shaft_bore_rear,
        guide_root,
    ])
    channel_open = wire_geometry._sub(  # noqa: SLF001
        entry_arc[0], wire_geometry._mul(entry_incoming, 7.0)  # noqa: SLF001
    )
    channel_axial = wire_geometry._add(  # noqa: SLF001
        entry_arc[-1], (0.0, 0.0, 7.0)
    )
    return {
        "model": (
            "actual-job contact-state centerline used by "
            "integrated_release_candidate.configured_static_supply_wire"
        ),
        "points": [list(point) for point in points],
        "landmarks": {
            "spool_payoff": list(spool),
            "felt_contact": list(felt),
            "felt_guide_in": list(felt_guide_in),
            "dancer_center": list(dancer_center),
            "dancer_tangent_in": list(tangent_in),
            "dancer_tangent_out": list(tangent_out),
            "entry_corner": list(entry_corner),
            "entry_eyelet": list(entry_eyelet),
            "shaft_bore_rear": list(shaft_bore_rear),
            "bore_rear": list(shaft_bore_rear),
            "guide_root": list(guide_root),
        },
        "dancer": {
            "body_radius": wire_geometry.DANCER_BODY_RADIUS,
            "path_radius": path_radius,
            "theta_in_deg": math.degrees(theta_in),
            "theta_out_deg": math.degrees(theta_out),
            "wrap_deg": wire_geometry.DANCER_WRAP_DEG,
            "direction": "clockwise",
            "tangent_in_direction": [
                math.sin(theta_in), -math.cos(theta_in), 0.0,
            ],
            "tangent_out_direction": [
                math.sin(theta_out), -math.cos(theta_out), 0.0,
            ],
        },
        "entry_bend": entry_meta,
        "entry_channel_points": [list(point) for point in wire_geometry._dedupe(  # noqa: SLF001
            [channel_open, entry_arc[0], *entry_arc[1:], channel_axial]
        )],
        "shaft_axis_run": {
            "start_mm": list(entry_arc[-1]),
            "entry_eyelet_mm": list(entry_eyelet),
            "shaft_bore_rear_mm": list(shaft_bore_rear),
            "end_mm": list(guide_root),
            "owner": "static",
            "M2_axis_invariant": True,
        },
        "felt_offset_from_stud": math.hypot(
            felt[0] - float(P.rear_post_x), felt[1] - float(P.felt_y)
        ),
        "spool_pack_radius": math.hypot(
            spool[1] - float(P.spool_y),
            spool[2] - float(P.rear_post_z + 60.0),
        ),
        "configured_wire_radius_mm": centerline_wire_radius,
    }


def _wire_handoff_contract(wire: Mapping[str, Any]) -> dict[str, Any]:
    """Fail closed unless static, flyer, and guide share both handoffs."""

    static = wire.get("static")
    flyer = wire.get("flyer")
    guide = wire.get("active_terminal_guide")
    if not all(isinstance(value, Mapping) for value in (static, flyer, guide)):
        raise ValueError("continuous wire manifest sections are missing")
    static_points = static.get("points")
    flyer_points = flyer.get("points")
    guide_points = guide.get("bore_centerline_local_mm")
    if not all(
        isinstance(points, list) and len(points) >= 2
        for points in (static_points, flyer_points, guide_points)
    ):
        raise ValueError("continuous wire manifest polylines are missing")

    landmarks = static.get("landmarks")
    if not isinstance(landmarks, Mapping):
        raise ValueError("static wire landmarks are missing")
    seam = _wire_point(static_points[-1], "static wire endpoint")
    gaps = {
        "static_endpoint_to_guide_root_mm": _wire_point_gap(
            seam, landmarks.get("guide_root")
        ),
        "static_to_flyer_bore_start_mm": _wire_point_gap(
            seam, flyer_points[0]
        ),
        "flyer_to_guide_bore_start_mm": _wire_point_gap(
            flyer_points[0], guide_points[0]
        ),
        "flyer_to_guide_bore_end_mm": _wire_point_gap(
            flyer_points[-1], guide_points[-1]
        ),
        "flyer_to_dynamic_transition_origin_mm": _wire_point_gap(
            flyer_points[-1], guide.get("unproved_transition_origin_local_mm")
        ),
    }
    if max(gaps.values()) > WIRE_CHAIN_TOLERANCE_MM:
        raise ValueError(f"continuous wire manifest has a handoff gap: {gaps}")
    if abs(seam[0]) > WIRE_CHAIN_TOLERANCE_MM or abs(
        seam[1]
    ) > WIRE_CHAIN_TOLERANCE_MM:
        raise ValueError("static/flyer handoff is not on the M2 axis")
    if len(flyer_points) != len(guide_points) or any(
        _wire_point_gap(left, right) > WIRE_CHAIN_TOLERANCE_MM
        for left, right in zip(flyer_points, guide_points)
    ):
        raise ValueError("flyer wire and PEEK bore centerline differ")
    return {
        "status": "PASS",
        "static_to_flyer_seam_local_mm": list(seam),
        "flyer_to_dynamic_seam_local_mm": list(
            _wire_point(flyer_points[-1], "flyer wire endpoint")
        ),
        "maximum_gap_mm": max(gaps.values()),
        "tolerance_mm": WIRE_CHAIN_TOLERANCE_MM,
        "gap_measurements_mm": gaps,
        "static_owner_continues_through_shaft_to_guide_root": True,
        "static_to_flyer_handoff_is_M2_axis_invariant": True,
        "unsupported_flexible_intervals_authorized": False,
    }


def _validate_wire_locus_binding(
    wire: Mapping[str, Any], payload: Mapping[str, Any]
) -> None:
    """Bind the exact-locus flyer prefix to the same complete PEEK bore."""

    contract = _wire_handoff_contract(wire)
    flyer_reference = payload.get("flyer_reference")
    if not isinstance(flyer_reference, Mapping):
        raise ValueError("active terminal flyer reference is missing")
    full = flyer_reference.get("full_geometric_bore_local_samples_mm")
    prefix = flyer_reference.get(FLYER_REFERENCE_SAMPLES_FIELD)
    expected = wire["flyer"]["points"]
    if not isinstance(full, list) or not isinstance(prefix, list):
        raise ValueError("active terminal flyer reference polylines are missing")
    if len(full) != len(expected) or any(
        _wire_point_gap(left, right) > WIRE_CHAIN_TOLERANCE_MM
        for left, right in zip(full, expected)
    ):
        raise ValueError("active terminal full bore differs from wire manifest")
    if not prefix or _wire_point_gap(
        prefix[0], contract["static_to_flyer_seam_local_mm"]
    ) > WIRE_CHAIN_TOLERANCE_MM:
        raise ValueError("active terminal prefix misses static guide-root seam")


def _wire_manifest() -> dict[str, Any]:
    coil = coil_growth.require_feasible(DEFAULT_STATOR)
    bore_samples = flyer_successor.guide_bore_centerline_samples(0.50)
    transition_origin = list(map(float, bore_samples[-1]))
    wire = {
        "diameter_max": wire_geometry.WIRE_DIAMETER_MAX,
        "radius_max": wire_geometry.WIRE_RADIUS_MAX,
        "diameter_job": DEFAULT_STATOR.wire_d,
        "radius_job": DEFAULT_STATOR.wire_d / 2.0,
        "render_radius": DEFAULT_STATOR.wire_d / 2.0,
        "render_diameter_scale": 1.0,
        "static": _configured_static_wire_manifest(),
        "flyer": {
            "model": "exact one-piece PEEK guide geometric bore centerline",
            "points": bore_samples,
            "landmarks": {
                "bore_rear": bore_samples[0],
                "geometric_bore_end": bore_samples[-1],
            },
            "dynamic_winding_continuation": (
                "active_terminal_locus_route.flyer_reference prefix plus "
                "per-locus world segments"
            ),
        },
        "tooth_contact": wire_geometry.tooth_contact_spec(DEFAULT_STATOR, coil),
        "shaft_contact": wire_geometry.shaft_contact_spec(DEFAULT_STATOR),
        "active_terminal_guide": {
            "model": (
                "one-piece polished natural-unfilled-PEEK hollow-shaft-to-tip "
                "bore with target-selectable axisymmetric exit bell"
            ),
            "bore_centerline_local_mm": bore_samples,
            "unproved_transition_origin_local_mm": transition_origin,
            "review_focus_center_local_mm": transition_origin,
            "bore_diameter_mm": 2.0 * flyer_successor.GUIDE_BORE_RADIUS_MM,
            "minimum_centerline_bend_radius_mm": (
                flyer_successor.GUIDE_CENTERLINE_RADIUS_MM
            ),
            "exit_bell_contact_surface_radius_mm": (
                flyer_successor.BELL_CONTACT_SURFACE_RADIUS_MM
            ),
            "material": "natural unfilled PEEK",
            "winding_geometry_source": (
                "out/reports/carriage_active_sector_terminal_guide_loci.json"
            ),
        },
        "visual_authority": (
            "candidate rigid-link witnesses plus exact sampled active terminal "
            "loci; continuous flexible-wire behavior and non-winding "
            "transitions remain release-unproven"
        ),
        "continuous_conductor_release_authorized": False,
    }
    wire["continuous_handoff"] = _wire_handoff_contract(wire)
    return wire


def _stator_manifest() -> dict[str, Any]:
    coil = coil_growth.require_feasible(DEFAULT_STATOR)
    return {
        "od": DEFAULT_STATOR.od,
        "stack": DEFAULT_STATOR.stack,
        "slots": DEFAULT_STATOR.slots,
        "shaft_d": DEFAULT_STATOR.shaft_d,
        "wire_d": DEFAULT_STATOR.wire_d,
        "shaft_below": DEFAULT_STATOR.shaft_below,
        "shaft_above": DEFAULT_STATOR.shaft_above,
        "hub_od_ratio": DEFAULT_STATOR.hub_od_ratio,
        "turns": DEFAULT_STATOR.turns,
        "tooth_len": DEFAULT_STATOR.tooth_len,
        "slot_fill": coil["packing"]["gross_slot_fill"],
        "slot_fill_status": coil["status"],
        "coil_collision_growth": coil["bundle"]["collision_growth_mm"],
    }


def _write_reference_glb(
    output: Path, manifest: Mapping[str, Any], links_dir: Path
) -> None:
    import trimesh

    scene = trimesh.Scene()

    def add(name: str, file_name: str, material_key: str) -> None:
        mesh = trimesh.load(links_dir / file_name, force="mesh")
        # A merged vertex shared by coincident or hard-edged components can
        # average opposing face normals to a zero vector.  Split vertices per
        # triangle for the review GLB so every exported PBR normal is the
        # finite unit face normal; positions and triangles are unchanged.
        mesh.unmerge_vertices()
        spec = MATERIALS[material_key]
        # Vertex/face colors exported by trimesh are legal glTF but some CAD
        # viewers apply their lit fallback material and render those meshes
        # nearly black.  Use an explicit glTF PBR material so the reference
        # GLB and the animated player share the same review colors.
        material = trimesh.visual.material.PBRMaterial(
            name=name,
            baseColorFactor=spec["color_rgba"],
            metallicFactor=float(spec["metallic"]),
            roughnessFactor=float(spec["roughness"]),
            doubleSided=bool(spec["double_sided"]),
        )
        mesh.visual = trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(mesh, node_name=name, geom_name=name)

    for link in LINK_NAMES:
        add(link, manifest["links"][link]["file"], LINK_MATERIALS[link])
    for owner, record in manifest["wire_assets"].items():
        add(f"wire_{owner}", record["file"], record["material"])
    for name, record in manifest["visual_groups"].items():
        add(name, record["file"], record["material"])
    output.parent.mkdir(parents=True, exist_ok=True)
    # Explicit normals are required by the viewer's lit PBR shader.  Trimesh
    # otherwise omits them for STL-derived meshes and the result renders as a
    # black silhouette despite correct base-color factors.
    output.write_bytes(scene.export(file_type="glb", include_normals=True))


def _write_json(path: Path, payload: Mapping[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return path


def _export_clean_stl(
    shape: Part | Compound,
    path: Path,
    *,
    tolerance: float,
    angular_tolerance: float,
) -> None:
    """Export an STL and remove tessellator-only zero-area triangles.

    OCC can emit coincident vertices on analytic seam endpoints.  The source
    BREP remains a valid single solid, but those triangles become zero-length
    edges after the collision audit merges STL vertices.  Removing only
    degenerate and duplicate faces preserves the exact tessellated surface and
    lets the normal custom-geometry topology gate remain authoritative.
    """

    export_stl(
        shape,
        str(path),
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
    )
    import trimesh

    mesh = trimesh.load(path, force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh) or mesh.is_empty:
        raise ValueError(f"STL export produced no mesh: {path}")
    mesh.merge_vertices()
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    path.write_bytes(mesh.export(file_type="stl"))


def _generated_envelopes_by_key(
    report: Mapping[str, Any],
) -> dict[tuple[str, str], Mapping[str, Any]]:
    result: dict[tuple[str, str], Mapping[str, Any]] = {}
    for row in report.get("parts", []):
        envelope = row.get("generated_collision_overbound")
        if not envelope:
            continue
        key = (str(row["link"]), str(row["label"]))
        if key in result:
            raise ValueError(f"duplicate collision overbound report row: {key}")
        result[key] = envelope
    return result


def run_collision_integrity_two_pass(
    manifest: Mapping[str, Any], output_dir: Path | str
) -> CollisionPipelineResult:
    """Draft-audit, bind verified substitutions, then re-audit final assets."""

    output = Path(output_dir).resolve()
    links_dir = output / "links"
    draft_manifest_path = links_dir / DRAFT_MANIFEST_NAME
    draft_report_path = output / DRAFT_COLLISION_AUDIT_NAME
    final_manifest_path = links_dir / "manifest.json"
    final_report_path = output / FINAL_COLLISION_AUDIT_NAME

    draft = deepcopy(dict(manifest))
    draft["collision_pipeline"] = {
        "schema": "integrated-adapter-collision-two-pass/v1",
        "phase": "draft_source_visual_mapping",
        "policy_version": mesh_integrity.POLICY_VERSION,
        "overbound_allowlist": sorted(
            f"{link}/{label}" for link, label in COLLISION_OVERBOUND_ALLOWLIST
        ),
        "custom_geometry_never_hulled": True,
        "final_effective_assets_reaudit_required": True,
    }
    draft["contract_sha256"] = _canonical_hash(draft)
    _write_json(draft_manifest_path, draft)

    draft_report = mesh_integrity.audit_adapter(
        draft_manifest_path,
        links_dir / "parts",
        generate_imported_envelopes=True,
        allow_legacy_label_fallback=False,
    )
    mesh_integrity.require_release_ready(draft_report)
    mesh_integrity.write_report(draft_report_path, draft_report)
    plan = mesh_integrity.collision_override_plan(draft_report)
    envelopes = _generated_envelopes_by_key(draft_report)
    plan_keys = {
        (str(link), str(label))
        for link, by_label in plan.items()
        for label in by_label
    }
    if plan_keys != set(envelopes):
        raise ValueError("collision override plan/report envelope set drift")
    unexpected = sorted(plan_keys - set(COLLISION_OVERBOUND_ALLOWLIST))
    if unexpected:
        raise ValueError(f"unreviewed collision substitutions: {unexpected}")

    draft_report_hash = mesh_integrity.sha256(draft_report_path)
    final = deepcopy(draft)
    substitutions: list[dict[str, Any]] = []
    for link, label in sorted(plan_keys):
        record = final["parts"][link][label]
        if record.get("provenance_class") == mesh_integrity.PROVENANCE_CUSTOM:
            raise ValueError(f"custom geometry cannot be hulled: {link}/{label}")
        planned = plan[link][label]
        envelope = envelopes[(link, label)]
        source_visual = Path(planned["source_visual_path"]).resolve()
        expected_source = (
            links_dir / "parts" / link / str(record["file"])
        ).resolve()
        if source_visual != expected_source:
            raise ValueError(f"source visual path drift: {link}/{label}")
        if (
            mesh_integrity.sha256(source_visual) != record.get("sha256")
            or planned["source_visual_sha256"] != record.get("sha256")
        ):
            raise ValueError(f"source visual hash drift: {link}/{label}")
        collision_mesh = Path(planned["collision_mesh_path"]).resolve()
        collision_parent = (links_dir / "parts" / link).resolve()
        if collision_parent not in collision_mesh.parents:
            raise ValueError(f"collision overbound escaped part directory: {link}/{label}")
        if mesh_integrity.sha256(collision_mesh) != planned["collision_mesh_sha256"]:
            raise ValueError(f"collision overbound hash drift: {link}/{label}")
        proof = deepcopy(envelope["overbound_proof"])
        proof_sha = _canonical_hash(proof)
        source_binding = {
            "file": source_visual.name,
            "sha256": planned["source_visual_sha256"],
            "retained_for_exact_visuals": True,
            "role": planned.get("source_visual_role"),
        }
        overbound_binding = {
            "method": planned["method"],
            "status": planned["overbound_status"],
            "collision_mesh_file": collision_mesh.name,
            "collision_mesh_sha256": planned["collision_mesh_sha256"],
            "source_visual_sha256": planned["source_visual_sha256"],
            "overbound_proof": proof,
            "overbound_proof_sha256": proof_sha,
            "draft_audit_file": draft_report_path.name,
            "draft_audit_sha256": draft_report_hash,
            "draft_audit_contract_sha256": draft_report["contract_sha256"],
            "exact_vendor_visual_retained": planned.get(
                "exact_vendor_visual_retained", False
            ),
            "detailed_modeled_visual_retained": planned.get(
                "detailed_modeled_visual_retained", False
            ),
        }
        record.update({
            "file": collision_mesh.name,
            "sha256": planned["collision_mesh_sha256"],
            "collision_role": "verified_conservative_overbound",
            "source_visual": source_binding,
            "collision_overbound": overbound_binding,
        })
        substitutions.append({
            "link": link,
            "label": label,
            "source_visual_sha256": planned["source_visual_sha256"],
            "collision_mesh_sha256": planned["collision_mesh_sha256"],
            "overbound_proof_sha256": proof_sha,
        })

    final["collision_pipeline"] = {
        "schema": "integrated-adapter-collision-two-pass/v1",
        "phase": "final_effective_mapping",
        "policy_version": mesh_integrity.POLICY_VERSION,
        "policy_source": "cad/collision_mesh_integrity.py",
        "policy_source_sha256": mesh_integrity.sha256(
            Path(mesh_integrity.__file__)
        ),
        "draft_manifest": {
            "file": f"links/{draft_manifest_path.name}",
            "sha256": mesh_integrity.sha256(draft_manifest_path),
            "contract_sha256": draft["contract_sha256"],
        },
        "draft_audit": {
            "file": draft_report_path.name,
            "sha256": draft_report_hash,
            "contract_sha256": draft_report["contract_sha256"],
            "status": draft_report["status"],
        },
        "overbound_allowlist": sorted(
            f"{link}/{label}" for link, label in COLLISION_OVERBOUND_ALLOWLIST
        ),
        "applicable_substitution_count": len(substitutions),
        "substitutions": substitutions,
        "exact_source_visuals_retained": True,
        "custom_geometry_never_hulled": True,
        "final_effective_assets_reaudited": True,
    }
    final["collision_manifest_contract"] = {
        **final.get("collision_manifest_contract", {}),
        "status": "PASS_TWO_PASS_EFFECTIVE_MAPPING",
        "explicit_per_part_provenance": True,
        "source_visuals_preserved": True,
        "verified_overbound_substitution_count": len(substitutions),
        "effective_assets_reaudited": True,
    }
    final["contract_sha256"] = _canonical_hash(final)
    _write_json(final_manifest_path, final)

    final_report = mesh_integrity.audit_adapter(
        final_manifest_path,
        links_dir / "parts",
        generate_imported_envelopes=False,
        allow_legacy_label_fallback=False,
    )
    mesh_integrity.require_release_ready(final_report)
    if final_report["summary"]["generated_overbound_count"] != 0:
        raise ValueError("final effective collision re-audit generated new envelopes")
    mesh_integrity.write_report(final_report_path, final_report)
    validate_collision_pipeline(
        final, output, final_report_path=final_report_path
    )
    return CollisionPipelineResult(
        manifest=final,
        draft_report=draft_report,
        final_report=final_report,
        draft_manifest_path=draft_manifest_path,
        draft_report_path=draft_report_path,
        final_manifest_path=final_manifest_path,
        final_report_path=final_report_path,
    )


def export_adapter(
    output_dir: Path | str | None = None,
    *,
    overrides: AdapterOverrides | None = None,
    export_reference_glb: bool = True,
    mesh_tolerance: float = 0.2,
    angular_tolerance: float = 0.25,
) -> dict[str, Any]:
    """Export exact visuals, run the required two-pass collision gate, return final."""

    selected = overrides or AdapterOverrides()
    release_identity = release_closure_identity()
    output = _validated_release_output(output_dir, release_identity)
    canonical_links = (ROOT / "out" / "links").resolve()
    if output == canonical_links or canonical_links in output.parents:
        raise ValueError("integrated adapter refuses to write canonical out/links")
    active_terminal_loci_record = stage_active_terminal_loci(output)
    links_dir = output / "links"
    links_dir.mkdir(parents=True, exist_ok=True)

    links = build_adapter_links(selected)
    override_keys = validate_override_provenance(selected)
    labels = _require_unique_labels(links)
    wires = build_adapter_wire_visuals(selected)
    link_records: dict[str, Any] = {}
    group_records: dict[str, Any] = {}
    collision_part_records: dict[str, dict[str, dict[str, Any]]] = {}
    part_bounds: dict[str, Any] = {}

    transform_contract = {
        "static": {"transform": "identity", "rule": "identity"},
        "carriage": {
            "transform": "translate_z",
            "rule": "dz = m0 * mm_per_rad_m0",
        },
        "spindle": {
            "transform": "translate_z_then_rotate_y_about_stator_axis",
            "rule": (
                "Tz(m0*mm_per_rad_m0) * Tz(m0_home_standoff) * "
                "Ry(m1) * Tz(-m0_home_standoff)"
            ),
        },
        "flyer": {"transform": "rot_z", "rule": "Rz(m2)"},
    }

    for link in LINK_NAMES:
        base, groups = split_visual_parts(
            link, links[link], selected.material_by_label
        )
        file_path = links_dir / f"{link}.stl"
        _export_clean_stl(
            Compound(children=list(base)),
            file_path,
            tolerance=mesh_tolerance,
            angular_tolerance=angular_tolerance,
        )
        link_records[link] = {
            "file": file_path.name,
            "material": LINK_MATERIALS[link],
            "labels": labels[link],
            "base_mesh_excluded_labels": sorted(
                _label(shape)
                for record in groups.values()
                for shape in record["parts"]
            ),
            **transform_contract[link],
        }
        part_bounds[link] = {
            _label(shape): _bounds(shape) for shape in links[link]
        }
        # Per-occurrence meshes are mandatory: collide.py consumes this exact
        # mapping for pair-specific intended-contact exemptions.  Display
        # labels remain authoritative while filenames are confined/sanitized.
        part_dir = links_dir / "parts" / link
        part_dir.mkdir(parents=True, exist_ok=True)
        filenames = [_safe_name(label) + ".stl" for label in labels[link]]
        if len(set(filenames)) != len(filenames):
            raise ValueError(
                f"{link} labels collide after safe filename normalization"
            )
        collision_part_records[link] = {}
        for shape, label, filename in zip(links[link], labels[link], filenames):
            part_path = part_dir / filename
            _export_clean_stl(
                shape,
                part_path,
                tolerance=mesh_tolerance,
                angular_tolerance=angular_tolerance,
            )
            source_hash = _sha256(part_path)
            provenance, provenance_source = part_provenance(
                link,
                label,
                selected,
                override_keys=override_keys,
            )
            method = collision_overbound_method(link, label, provenance)
            part_record: dict[str, Any] = {
                "file": filename,
                "sha256": source_hash,
                "source_visual_file": filename,
                "source_visual_sha256": source_hash,
                "collision_role": "exact_source_visual_and_draft_collision_mesh",
                "provenance_class": provenance,
                "provenance_source": provenance_source,
            }
            if method is not None:
                part_record["collision_overbound_method"] = method
            collision_part_records[link][label] = part_record
        for name, record in groups.items():
            if name in group_records:
                raise RuntimeError(f"visual group {name!r} spans multiple links")
            group_path = links_dir / f"visual_{_safe_name(name)}.stl"
            _export_clean_stl(
                Compound(children=list(record["parts"])),
                group_path,
                tolerance=min(mesh_tolerance, 0.1),
                angular_tolerance=min(angular_tolerance, 0.2),
            )
            group_records[name] = {
                "file": group_path.name,
                "link": link,
                "material": record["material"],
                "labels": sorted(_label(shape) for shape in record["parts"]),
                "description": record["description"],
                "excluded_from_base_link_mesh": True,
            }

    wire_records: dict[str, Any] = {}
    for owner, parts in wires.items():
        file_path = links_dir / f"wire_{owner}.stl"
        _export_clean_stl(
            Compound(children=list(parts)),
            file_path,
            tolerance=min(mesh_tolerance, 0.1),
            angular_tolerance=min(angular_tolerance, 0.2),
        )
        wire_records[owner] = {
            "file": file_path.name,
            "link": owner,
            "material": "enameled_copper",
            "labels": [_label(shape) for shape in parts],
            "excluded_from_collision_links": True,
        }
        if owner not in {"static", "flyer"}:
            group_name = f"wire_{owner}"
            if group_name in group_records:
                raise RuntimeError(f"duplicate visual group {group_name!r}")
            group_records[group_name] = {
                **wire_records[owner],
                "description": f"{owner}-owned conductor witnesses",
                "excluded_from_base_link_mesh": True,
            }

    kinematics = kinematic_equivalence_report()
    if kinematics["status"] != "PASS":
        raise RuntimeError("integrated candidate transform contract drift")
    option = spindle_option(DEFAULT_SPINDLE_ID)
    manifest: dict[str, Any] = {
        "schema": SCHEMA,
        "release_identity": release_identity,
        "status": "REVIEW_ASSETS_READY_RELEASE_GATES_OPEN",
        "production_authorized": False,
        "canonical_promotion_authorized": False,
        "units": "mm",
        "frame": (
            "machine frame; Z is flyer axis, Y is M1 rotation axis, "
            "M0=M1=M2=0 reference pose"
        ),
        "reference_pose": {"m0": 0.0, "m1": 0.0, "m2": 0.0},
        "mm_per_rad_m0": P.mm_per_rad,
        "m0_home_standoff": P.m0_home_standoff,
        "flyer_tip_r": P.flyer_tip_r,
        "m2_belt_center_distance_mm": abs(P.m2_motor_axis_y),
        "dyn_clearance": P.dyn_clearance,
        "stator": _stator_manifest(),
        "spindle": option.manifest_record(),
        "links": link_records,
        "visual_groups": group_records,
        "wire_assets": wire_records,
        "wire": _wire_manifest(),
        "active_terminal_locus_route": active_terminal_loci_record,
        "materials": MATERIALS,
        "kinematic_equivalence": kinematics,
        "override_contract": {
            "final_one_piece_PEEK_flyer_guide_owner": "flyer",
            "active_sector_guide_pair_owner": "carriage",
            "short_leadin_cap_pair_owner": "spindle",
            "legacy_flyer_tip_guide_override_authorized": False,
            "override_active": bool(
                selected.flyer_tip_guide
                or selected.terminal_crown_parts
                or selected.link_additions
                or selected.wire_visuals_by_link
            ),
        },
        "parts": collision_part_records,
        "part_bounds": part_bounds,
        "collision_manifest_contract": {
            "status": "DRAFT_PENDING_TWO_PASS_INTEGRITY_AUDIT",
            "consumer": "sim/collide.py --links <adapter>/links",
            "physical_occurrences_only": True,
            "visual_overlays_excluded": True,
            "logical_labels_preserved": True,
            "safe_filename_mapping_explicit": True,
            "explicit_per_part_provenance": True,
            "custom_geometry_never_hulled": True,
            "all_four_links_have_parts": all(
                bool(collision_part_records.get(link)) for link in LINK_NAMES
            ),
        },
        "source_hashes": release_source_hashes(),
        "release_boundaries": {
            "primary_STEP_unchanged": True,
            "full_raw_cycle_collision_regenerated": False,
            "active_terminal_route_2400_sampled_loci_bound": True,
            "park_index_load_unload_transition_proven": False,
            "continuous_phase_aware_conductor_proven": False,
            "dynamic_wire_tension_sag_snag_wear_proven": False,
            "production_authorized": False,
        },
    }
    for record in link_records.values():
        record["sha256"] = _sha256(links_dir / record["file"])
    for record in group_records.values():
        record["sha256"] = _sha256(links_dir / record["file"])
    for record in wire_records.values():
        record["sha256"] = _sha256(links_dir / record["file"])

    if not export_reference_glb:
        raise ValueError(
            "two-pass collision integrity requires the normal-audited reference GLB"
        )
    glb_path = output / "integrated_candidate_reference_pose.glb"
    _write_reference_glb(glb_path, manifest, links_dir)
    manifest["reference_pose_glb"] = {
        "file": glb_path.name,
        "sha256": _sha256(glb_path),
        "coordinate_units": "mm_for_existing_browser_player",
        "animation": False,
    }
    result = run_collision_integrity_two_pass(manifest, output)
    _write_json(
        output / RELEASE_IDENTITY_NAME,
        {
            "schema": "integrated-adapter-release-identity/v1",
            **release_identity,
        },
    )
    validate_manifest(
        result.manifest,
        output,
        final_collision_report=result.final_report_path,
    )
    return result.manifest


def _resolved_pipeline_file(output: Path, relative: str) -> Path:
    if not isinstance(relative, str) or Path(relative).is_absolute():
        raise ValueError(f"unsafe collision pipeline path {relative!r}")
    path = (output / relative).resolve()
    if path != output and output not in path.parents:
        raise ValueError(f"collision pipeline asset escapes output: {relative}")
    return path


def validate_collision_pipeline(
    manifest: Mapping[str, Any],
    output_dir: Path | str,
    *,
    final_report_path: Path | str | None = None,
) -> None:
    """Validate proof/hash bindings and the final effective-asset re-audit."""

    output = Path(output_dir).resolve()
    links_dir = output / "links"
    pipeline = manifest.get("collision_pipeline")
    if not isinstance(pipeline, Mapping):
        raise ValueError("final adapter manifest has no collision two-pass contract")
    if pipeline.get("schema") != "integrated-adapter-collision-two-pass/v1":
        raise ValueError("collision two-pass schema drift")
    if pipeline.get("phase") != "final_effective_mapping":
        raise ValueError("collision two-pass pipeline is not final")
    expected_allowlist = sorted(
        f"{link}/{label}" for link, label in COLLISION_OVERBOUND_ALLOWLIST
    )
    if pipeline.get("overbound_allowlist") != expected_allowlist:
        raise ValueError("collision overbound allowlist drift")
    if pipeline.get("custom_geometry_never_hulled") is not True:
        raise ValueError("custom geometry hull prohibition is not bound")

    draft_manifest_record = pipeline.get("draft_manifest")
    draft_audit_record = pipeline.get("draft_audit")
    if not isinstance(draft_manifest_record, Mapping) or not isinstance(
        draft_audit_record, Mapping
    ):
        raise ValueError("collision pipeline draft bindings are missing")
    draft_manifest_path = _resolved_pipeline_file(
        output, str(draft_manifest_record.get("file"))
    )
    draft_audit_path = _resolved_pipeline_file(
        output, str(draft_audit_record.get("file"))
    )
    if _sha256(draft_manifest_path) != draft_manifest_record.get("sha256"):
        raise ValueError("draft collision manifest hash drift")
    if _sha256(draft_audit_path) != draft_audit_record.get("sha256"):
        raise ValueError("draft collision audit hash drift")
    draft_audit = json.loads(draft_audit_path.read_text(encoding="utf-8"))
    mesh_integrity.require_release_ready(draft_audit)
    if draft_audit.get("contract_sha256") != draft_audit_record.get(
        "contract_sha256"
    ):
        raise ValueError("draft collision audit contract drift")

    parts = manifest.get("parts")
    if not isinstance(parts, Mapping):
        raise ValueError("final adapter collision parts are missing")
    bound_substitutions: dict[tuple[str, str], Mapping[str, Any]] = {}
    part_count = 0
    for link, by_label in parts.items():
        if not isinstance(by_label, Mapping):
            raise ValueError(f"collision link {link} is malformed")
        for label, record in by_label.items():
            part_count += 1
            if not isinstance(record, Mapping):
                raise ValueError(f"collision part {link}/{label} is malformed")
            provenance = record.get("provenance_class")
            if provenance not in mesh_integrity.VALID_PROVENANCE:
                raise ValueError(f"collision part lacks explicit provenance: {link}/{label}")
            if (
                CUSTOM_TOPOLOGY_LABEL_RE.search(str(label))
                and provenance != mesh_integrity.PROVENANCE_CUSTOM
            ):
                raise ValueError(f"custom topology provenance bypass: {link}/{label}")
            method = record.get("collision_overbound_method")
            binding = record.get("collision_overbound")
            if provenance == mesh_integrity.PROVENANCE_CUSTOM and (
                method is not None or binding is not None
            ):
                raise ValueError(f"custom geometry was hulled: {link}/{label}")
            if method is not None and (link, label) not in COLLISION_OVERBOUND_ALLOWLIST:
                raise ValueError(f"unreviewed collision overbound method: {link}/{label}")

            source_name = record.get("source_visual_file")
            source_hash = record.get("source_visual_sha256")
            if (
                not isinstance(source_name, str)
                or Path(source_name).name != source_name
                or not isinstance(source_hash, str)
            ):
                raise ValueError(f"source visual binding missing: {link}/{label}")
            source_path = links_dir / "parts" / str(link) / source_name
            if _sha256(source_path) != source_hash:
                raise ValueError(f"source visual hash drift: {link}/{label}")

            if binding is None:
                if record.get("file") != source_name or record.get("sha256") != source_hash:
                    raise ValueError(f"unbound collision substitution: {link}/{label}")
                continue
            if not isinstance(binding, Mapping):
                raise ValueError(f"collision overbound binding malformed: {link}/{label}")
            if (link, label) not in COLLISION_OVERBOUND_ALLOWLIST:
                raise ValueError(f"unreviewed collision substitution: {link}/{label}")
            if provenance == mesh_integrity.PROVENANCE_CUSTOM:
                raise ValueError(f"custom collision substitution: {link}/{label}")
            if (
                binding.get("method") != mesh_integrity.HULL_METHOD
                or binding.get("status") != "PASS"
                or method != mesh_integrity.HULL_METHOD
            ):
                raise ValueError(f"collision overbound method/status drift: {link}/{label}")
            if binding.get("source_visual_sha256") != source_hash:
                raise ValueError(f"collision source visual binding drift: {link}/{label}")
            effective_name = binding.get("collision_mesh_file")
            effective_hash = binding.get("collision_mesh_sha256")
            if (
                record.get("file") != effective_name
                or record.get("sha256") != effective_hash
                or effective_name == source_name
            ):
                raise ValueError(f"effective collision mapping drift: {link}/{label}")
            effective_path = links_dir / "parts" / str(link) / str(effective_name)
            if _sha256(effective_path) != effective_hash:
                raise ValueError(f"effective collision hash drift: {link}/{label}")
            proof = binding.get("overbound_proof")
            if not isinstance(proof, Mapping) or proof.get("status") != "PASS":
                raise ValueError(f"collision overbound proof failed: {link}/{label}")
            if _canonical_hash(proof) != binding.get("overbound_proof_sha256"):
                raise ValueError(f"collision overbound proof hash drift: {link}/{label}")
            if (
                binding.get("draft_audit_sha256") != draft_audit_record.get("sha256")
                or binding.get("draft_audit_contract_sha256")
                != draft_audit_record.get("contract_sha256")
            ):
                raise ValueError(f"collision draft-audit binding drift: {link}/{label}")
            bound_substitutions[(str(link), str(label))] = binding

    listed = pipeline.get("substitutions")
    if not isinstance(listed, list):
        raise ValueError("collision substitution list is missing")
    listed_keys = {(str(row.get("link")), str(row.get("label"))) for row in listed}
    if listed_keys != set(bound_substitutions):
        raise ValueError("collision substitution list/mapping drift")
    if pipeline.get("applicable_substitution_count") != len(bound_substitutions):
        raise ValueError("collision substitution count drift")

    report_path = (
        Path(final_report_path).resolve()
        if final_report_path is not None
        else output / FINAL_COLLISION_AUDIT_NAME
    )
    if not report_path.is_file():
        raise ValueError("final effective collision re-audit is missing")
    final_report = json.loads(report_path.read_text(encoding="utf-8"))
    mesh_integrity.require_release_ready(final_report)
    final_manifest_path = links_dir / "manifest.json"
    if final_report.get("inputs", {}).get("manifest_contract_sha256") != manifest.get(
        "contract_sha256"
    ):
        raise ValueError("final collision re-audit manifest contract drift")
    if final_manifest_path.is_file() and final_report.get("inputs", {}).get(
        "manifest_sha256"
    ) != _sha256(final_manifest_path):
        raise ValueError("final collision re-audit manifest file hash drift")
    summary = final_report.get("summary", {})
    if summary.get("part_count") != part_count:
        raise ValueError("final collision re-audit part count drift")
    if summary.get("generated_overbound_count") != 0:
        raise ValueError("final collision re-audit generated an envelope")
    if summary.get("effective_collision_failures"):
        raise ValueError("final collision re-audit has effective failures")


def validate_manifest(
    manifest: Mapping[str, Any],
    output_dir: Path | str,
    *,
    final_collision_report: Path | str | None = None,
) -> None:
    if manifest.get("schema") != SCHEMA:
        raise ValueError("integrated adapter manifest schema drift")
    if manifest.get("production_authorized") is not False:
        raise ValueError("integrated adapter must stay review-only")
    if manifest.get("canonical_promotion_authorized") is not False:
        raise ValueError("integrated adapter cannot authorize promotion")
    if manifest.get("contract_sha256") != _canonical_hash(manifest):
        raise ValueError("integrated adapter manifest hash mismatch")
    if manifest.get("kinematic_equivalence", {}).get("status") != "PASS":
        raise ValueError("integrated adapter kinematic contract is not PASS")
    if set(manifest.get("links", {})) != set(LINK_NAMES):
        raise ValueError("integrated adapter link set drift")
    wire = manifest.get("wire")
    if not isinstance(wire, Mapping):
        raise ValueError("integrated adapter wire manifest is missing")
    expected_handoff = _wire_handoff_contract(wire)
    if wire.get("continuous_handoff") != expected_handoff:
        raise ValueError("integrated adapter wire handoff contract drift")
    links_dir = Path(output_dir).resolve() / "links"
    records = [
        *manifest["links"].values(),
        *manifest["visual_groups"].values(),
        *manifest["wire_assets"].values(),
    ]
    for record in records:
        file_name = record.get("file")
        if not isinstance(file_name, str) or Path(file_name).name != file_name:
            raise ValueError(f"unsafe adapter asset name {file_name!r}")
        path = links_dir / file_name
        if _sha256(path) != record.get("sha256"):
            raise ValueError(f"adapter asset hash mismatch: {path}")
    terminal_route = manifest.get("active_terminal_locus_route")
    if terminal_route is not None:
        if not isinstance(terminal_route, Mapping):
            raise ValueError("active terminal locus manifest record is malformed")
        if (
            terminal_route.get("schema") != ACTIVE_TERMINAL_LOCI_SCHEMA
            or terminal_route.get("locus_count") != EXPECTED_ACTIVE_TERMINAL_LOCI
            or terminal_route.get("held_between_loci_for_review_only") is not True
            or terminal_route.get("park_index_load_unload_proven") is not False
            or terminal_route.get("sag_tension_settling_neatness_proven") is not False
        ):
            raise ValueError("active terminal locus manifest scope drift")
        route_path = _resolved_pipeline_file(
            Path(output_dir).resolve(), str(terminal_route.get("file"))
        )
        if _sha256(route_path) != terminal_route.get("artifact_sha256"):
            raise ValueError("active terminal locus staged artifact hash drift")
        payload = json.loads(route_path.read_text(encoding="utf-8"))
        if (
            payload.get("locus_payload_sha256")
            != terminal_route.get("locus_payload_sha256")
            or _active_terminal_payload_hash(payload)
            != terminal_route.get("locus_payload_sha256")
        ):
            raise ValueError("active terminal locus staged payload hash drift")
        _validate_wire_locus_binding(wire, payload)
    parts = manifest.get("parts")
    if not isinstance(parts, Mapping) or set(parts) != set(LINK_NAMES):
        raise ValueError("integrated adapter collision part mapping drift")
    for link, by_label in parts.items():
        if not isinstance(by_label, Mapping) or not by_label:
            raise ValueError(f"integrated adapter collision link {link} is empty")
        filenames: set[str] = set()
        for label, record in by_label.items():
            if not isinstance(label, str) or not label:
                raise ValueError(f"integrated adapter {link} has unsafe label")
            if not isinstance(record, Mapping):
                raise ValueError(f"integrated adapter {link}/{label} is malformed")
            file_name = record.get("file")
            if (
                not isinstance(file_name, str)
                or Path(file_name).name != file_name
                or not file_name.lower().endswith(".stl")
            ):
                raise ValueError(f"unsafe collision asset {link}/{label}")
            if file_name in filenames:
                raise ValueError(f"duplicate collision filename in {link}: {file_name}")
            filenames.add(file_name)
            path = links_dir / "parts" / link / file_name
            if _sha256(path) != record.get("sha256"):
                raise ValueError(f"collision asset hash mismatch: {path}")
            if record.get("provenance_class") not in mesh_integrity.VALID_PROVENANCE:
                raise ValueError(f"collision asset lacks provenance: {link}/{label}")
    if "collision_pipeline" in manifest:
        validate_collision_pipeline(
            manifest,
            output_dir,
            final_report_path=final_collision_report,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        type=Path,
        help=(
            "immutable release directory; defaults to "
            "out/review/integrated_adapter_releases/<current release id>"
        ),
    )
    parser.add_argument(
        "--print-release-identity",
        action="store_true",
        help="print the immutable CAD-and-player source closure identity without exporting",
    )
    args = parser.parse_args()
    if args.print_release_identity:
        print(json.dumps(release_closure_identity(), indent=2))
        return
    output = _validated_release_output(args.out, release_closure_identity())
    manifest = export_adapter(output)
    validate_manifest(manifest, output)
    print(output / "links" / "manifest.json")


if __name__ == "__main__":
    main()
