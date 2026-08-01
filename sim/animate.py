"""Export the full captured winding cycle as an animated GLB and player.

The GLB keeps the machine's four rigid links and the two machine-guide wire runs
as separate named nodes.  The legacy raw-baseline outputs
``out/winding_cycle_upstream_raw.glb`` and
``out/play_animation_upstream_raw.html`` embed the authoritative capture
timeline so it can be opened directly and watched with pause, scrubbing, slow
motion, command stepping, and camera presets.  That legacy CAD manifest has no
continuous-conductor or active-terminal-locus contract, so its player remains
explicitly non-governing and makes no continuity claim.  The separate selected
integrated player consumes the hash-bound 2,400-locus active-sector route; even
that exact sampled winding route does not authorize the still-unproved
park/index/load/unload transitions, conductor sag/tension dynamics, or strand
settling/neatness.

Node tree (GLB is Y-up, right-handed -- identical to the machine frame):
  root
   |-- static
   |-- felt_pads          [dark wool-felt material; excluded from static mesh]
   |-- wire_static
   |-- carriage            [translation: (0,0,m0*mm_per_rad)]
   |    `-- spindle_pivot  [at (0,0,standoff); rotation about Y]
   |         `-- spindle   [offset (0,0,-standoff)]
   `-- flyer               [rotation about Z]
        |-- wire_flyer
        `-- manifest-defined one-piece PEEK guide and six-stack balance groups

Keyframes are motion-adaptive so interpolation never aliases a large move.
``--speed`` controls GLB time compression and the player's default virtual
playback rate; the player always reports and scrubs in uncompressed virtual
seconds.

Usage:
  python animate.py --capture ../out/capture/commands.jsonl
  python animate.py --capture ../out/capture/upstream_current_raw.jsonl \
      -o ../out/winding_cycle_upstream_raw.glb \
      --html ../out/play_animation_upstream_raw.html
"""

import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import pygltflib as gl
import trimesh

import continuous_conductor_route as conductor_route
from traj import Timeline, load_events, winding_windows
from winding_plan import load_slot_winding_plan

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out"

COLORS = {
    "static": [0.78, 0.80, 0.83, 1.0],
    "carriage": [0.45, 0.60, 0.90, 1.0],
    "spindle": [0.95, 0.65, 0.25, 1.0],
    "flyer": [0.90, 0.32, 0.35, 1.0],
    "wire_static": [0.72, 0.30, 0.08, 1.0],
    "wire_flyer": [0.94, 0.46, 0.10, 1.0],
    "felt_pads": [0.22, 0.075, 0.025, 1.0],
}

MATERIAL_PROPERTIES = {
    "felt_pads": {"metallic": 0.0, "roughness": 1.0, "double_sided": True},
}

REQUIRED_VISUAL_GROUPS = {
    "felt_pads": {
        "link": "static",
        "labels": {"felt_pad_fixed", "felt_pad_moving"},
    },
}

SLOT_WINDING_PLAN = OUT / "reports" / "slot_winding_plan.json"
CONTINUOUS_CONDUCTOR_ROUTE = (
    OUT / "reports" / "continuous_conductor_route.json"
)
ACTIVE_TERMINAL_LOCI_NAME = "carriage_active_sector_terminal_guide_loci.json"
ACTIVE_TERMINAL_LOCI = OUT / "reports" / ACTIVE_TERMINAL_LOCI_NAME
ACTIVE_TERMINAL_LOCI_SCHEMA = "carriage-active-sector-terminal-guide-loci/v1"
ACTIVE_TERMINAL_LOCI_PER_PASS = 100
EXPECTED_ACTIVE_TERMINAL_LOCI = 24 * ACTIVE_TERMINAL_LOCI_PER_PASS
FLYER_REFERENCE_SEGMENT = "flyer_geometric_bore"
FLYER_REFERENCE_SAMPLES_FIELD = (
    "geometric_bore_to_tensioned_handoff_local_samples_mm"
)
# Normal GOAL.md requires the unmodified upstream command stream.  ContractWind
# remains an explicit diagnostic option through --capture; it is no longer the
# accidental default used by the published player.
DEFAULT_CAPTURE = OUT / "capture" / "upstream_current_raw.jsonl"
LEGACY_RAW_GLB = OUT / "winding_cycle_upstream_raw.glb"
LEGACY_RAW_HTML = OUT / "play_animation_upstream_raw.html"
INTEGRATED_ADAPTER_MANIFEST_SCHEMA = "integrated-candidate-player-adapter/v1"
INTEGRATED_CONDUCTOR_MODE = "integrated_fail_closed_review"
LEGACY_CONDUCTOR_MODE = "legacy_baseline_unavailable_unproved"
MAX_DYNAMIC_WIRE_SEGMENTS = 128
# Reserve a separate pool for the red/dashed cap-to-live-tail witness.  The
# exact active-terminal prefix can itself contain hundreds of sampled edges.
MAX_UNPROVEN_LIVE_CONNECTOR_SEGMENTS = 128
PLAYER_SCHEMA = "winder-player/v3"


def _finite(value: Any, field: str) -> float:
    """Return a finite float or reject the artifact field."""
    if isinstance(value, bool):
        raise RuntimeError(f"{field} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{field} must be a finite number") from exc
    if not math.isfinite(result):
        raise RuntimeError(f"{field} must be finite")
    return result


def _same_number(left: Any, right: Any, *, tolerance: float = 1e-9) -> bool:
    return math.isclose(
        _finite(left, "left identity value"),
        _finite(right, "right identity value"),
        rel_tol=0.0,
        abs_tol=tolerance,
    )


def _canonical_plan_proof_sha256(raw: Mapping[str, Any]) -> str:
    payload = dict(raw)
    payload.pop("proof_sha256", None)
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_player_slot_plan(path=SLOT_WINDING_PLAN):
    """Load the constructive plan and retain exact presentation geometry.

    ``winding_plan.load_slot_winding_plan`` validates the controller-facing
    placement and half-turn contract.  This additional layer verifies the
    proof hash and the exact two-sided coordinates used by the browser.
    """
    loaded = load_slot_winding_plan(path)
    raw = loaded.raw
    proof_sha = raw.get("proof_sha256")
    actual_proof_sha = _canonical_plan_proof_sha256(raw)
    if not isinstance(proof_sha, str) or proof_sha != actual_proof_sha:
        raise RuntimeError(
            "slot winding plan proof_sha256 does not cover its current payload"
        )

    frame = raw["coordinate_frame"]
    active_angle = math.radians(_finite(
        frame.get("active_tooth_center_angle_deg"),
        "plan.coordinate_frame.active_tooth_center_angle_deg",
    ))
    transition = raw["selected_case"]["transition_proof"]
    first_side = transition.get("first_side_insertion", [])
    mouth = transition.get("sequential_mouth_access", {})
    later_side = transition.get(
        "later_neighbor_side_mouth_connected",
        mouth.get("prefilled_neighbor_side_mouth_connected", []),
    )
    if len(first_side) != loaded.turns_per_tooth:
        raise RuntimeError("plan first-side transition proof is incomplete")
    if len(later_side) != loaded.turns_per_tooth:
        raise RuntimeError("plan later-neighbor transition proof is incomplete")

    placements = []
    for index, (placement, item) in enumerate(zip(
            loaded.placements, raw["placements"])):
        left = item.get("left_slot_half_turn_center_mm")
        right = item.get("right_slot_half_turn_center_mm")
        if not isinstance(left, list) or len(left) != 2:
            raise RuntimeError(f"plan placement {index} left center is malformed")
        if not isinstance(right, list) or len(right) != 2:
            raise RuntimeError(f"plan placement {index} right center is malformed")
        left_r, left_t = map(float, left)
        right_r, right_t = map(float, right)
        if (not _same_number(left_r, placement.radial_mm)
                or not _same_number(right_r, placement.radial_mm)
                or not _same_number(left_t, placement.tangential_mm)
                or not _same_number(right_t, -placement.tangential_mm)):
            raise RuntimeError(
                f"plan placement {index} two-sided centers do not mirror exactly"
            )

        # The left center is expressed in the left slot-bisector frame.  Rotate
        # it into the active-tooth frame; the opposite coil side is its mirror.
        active_radial = (
            left_r * math.cos(active_angle) + left_t * math.sin(active_angle)
        )
        active_tangential = (
            -left_r * math.sin(active_angle) + left_t * math.cos(active_angle)
        )
        if not _same_number(
                active_radial, placement.active_tooth_radial_mm):
            raise RuntimeError(
                f"plan placement {index} active-tooth radial projection drifted"
            )
        raw_active_t = _finite(
            item.get("active_tooth_tangential_mm"),
            f"plan.placements[{index}].active_tooth_tangential_mm",
        )
        if not _same_number(active_tangential, raw_active_t):
            raise RuntimeError(
                f"plan placement {index} active-tooth tangential projection drifted"
            )
        if active_tangential >= 0.0:
            raise RuntimeError(
                f"plan placement {index} left side is not tangential-negative"
            )

        first = first_side[index]
        later_connected = later_side[index]
        insertion_supported = (
            first.get("mouth_connected") is True
            or first.get("progressively_supported") is True
        )
        if (first.get("placement_index") != index
                or not insertion_supported
                or later_connected is not True):
            raise RuntimeError(
                f"plan placement {index} transition record is not complete/PASS"
            )
        predecessors = first.get("support_predecessor_indices", [])
        if not isinstance(predecessors, list) or any(
                not isinstance(value, int) or value >= index or value < 0
                for value in predecessors):
            raise RuntimeError(
                f"plan placement {index} support predecessor list is malformed"
            )

        placements.append({
            "turn": index,
            "radial": placement.radial_mm,
            "slotTangential": placement.tangential_mm,
            "activeRadial": placement.active_tooth_radial_mm,
            "m0Target": placement.m0_target_rad,
            "leftActiveTangential": active_tangential,
            "rightActiveTangential": -active_tangential,
            "leftSlotCenter": [left_r, left_t],
            "rightSlotCenter": [right_r, right_t],
            "layer": placement.layer,
            "row": placement.row,
            "contact": placement.contact_id,
            "support": first.get("support"),
            "supportPredecessors": predecessors,
            "laterNeighborMouthConnected": True,
        })

    return {
        "schema": loaded.raw["schema"],
        "algorithm": loaded.raw.get("algorithm"),
        "path": str(loaded.path),
        "artifactSha256": loaded.sha256,
        "proofSha256": proof_sha,
        "sourceHashes": loaded.raw.get("source_hashes", {}),
        "job": {
            "slots": loaded.slots,
            "od": loaded.od_mm,
            "stack": loaded.stack_mm,
            "wireFinishedDiameter": loaded.wire_finished_d_mm,
            "modelWireEnvelope": loaded.model_wire_envelope_mm,
            "linerMaximumThickness": loaded.liner_max_thickness_mm,
            "turnsPerTooth": loaded.turns_per_tooth,
        },
        "receivingSensitivity": {
            "wireEnvelope": loaded.receiving_sensitivity_wire_envelope_mm,
            "status": loaded.receiving_sensitivity_status,
        },
        "transitionStatus": loaded.transition_status,
        "coordinateFrame": frame,
        "placements": placements,
        "placementCount": len(placements),
        "halfTurnCenterCount": len(loaded.half_turn_centers),
        "geometryAuthority": {
            "slotLegCenters": "exact constructive plan centers",
            "endTurnConnectors": "illustrative semicircles between exact leg centers",
            "liveFeedSpan": "visualization only; not a sequential route proof",
        },
    }


def _load_continuous_conductor_route(
        path=CONTINUOUS_CONDUCTOR_ROUTE, *, capture_path,
        plan_path=SLOT_WINDING_PLAN,
        manifest_path=None):
    """Load the exact route shared by validation and the canonical player."""

    route_path = Path(path).resolve()
    if not route_path.is_file():
        raise RuntimeError(
            "canonical raw player requires generated continuous conductor "
            f"route: {route_path}"
        )
    route = json.loads(route_path.read_text(encoding="utf-8"))
    active_manifest_path = Path(
        manifest_path if manifest_path is not None
        else OUT / "links" / "manifest.json"
    ).resolve()
    conductor_route.validate_route_artifact(
        route,
        capture_path=Path(capture_path).resolve(),
        plan_path=Path(plan_path).resolve(),
        manifest_path=active_manifest_path,
    )
    if route.get("structural_status") != "PASS":
        raise RuntimeError("continuous conductor route is structurally invalid")
    # Release FAIL is expected until the production caps/flyer transitions are
    # geometrically authorized.  The player must show those edges, not hide or
    # relabel them as passing geometry.
    if (route.get("status") != "FAIL"
            or route.get("production_authorized") is not False):
        raise RuntimeError("continuous conductor route did not fail closed")
    return route


def _raw_player_manifest_role(manifest: Mapping[str, Any]) -> str:
    """Classify raw-player assets without silently downgrading an adapter.

    The historical ``cad/export_links.py`` manifest predates schema tagging and
    intentionally has only the two rigid guide-wire meshes.  The selected
    integrated adapter is explicitly schema-tagged and must retain every one of
    its conductor bindings; a malformed or unknown schema is never treated as
    the permissive legacy case.
    """

    schema = manifest.get("schema")
    if schema is None:
        return LEGACY_CONDUCTOR_MODE
    if schema == INTEGRATED_ADAPTER_MANIFEST_SCHEMA:
        return INTEGRATED_CONDUCTOR_MODE
    raise RuntimeError(f"unsupported raw-player CAD manifest schema: {schema!r}")


def _wire_point_gap(left: Any, right: Any) -> float:
    if not all(
        isinstance(point, (list, tuple)) and len(point) == 3
        for point in (left, right)
    ):
        raise RuntimeError("wire handoff point is malformed")
    return math.sqrt(sum(
        (_finite(left[index], "wire handoff coordinate")
         - _finite(right[index], "wire handoff coordinate")) ** 2
        for index in range(3)
    ))


def _validate_active_terminal_wire_chain(
        wire: Mapping[str, Any], payload: Mapping[str, Any]) -> dict[str, Any]:
    """Bind the exact-locus bore prefix to the visible static/flyer meshes."""

    handoff = conductor_route._wire_handoff_contract({"wire": wire})
    flyer_reference = payload.get("flyer_reference")
    if not isinstance(flyer_reference, Mapping):
        raise RuntimeError("active terminal flyer reference is missing")
    full = flyer_reference.get("full_geometric_bore_local_samples_mm")
    prefix = flyer_reference.get(FLYER_REFERENCE_SAMPLES_FIELD)
    flyer_points = wire.get("flyer", {}).get("points")
    if not all(isinstance(points, list) and len(points) >= 2
               for points in (full, prefix, flyer_points)):
        raise RuntimeError("wire/locus flyer polylines are missing")
    tolerance = float(handoff["tolerance_mm"])
    if len(full) != len(flyer_points) or any(
        _wire_point_gap(left, right) > tolerance
        for left, right in zip(full, flyer_points)
    ):
        raise RuntimeError("active terminal full bore differs from CAD wire")
    if _wire_point_gap(
        prefix[0], handoff["static_to_flyer_seam_local_mm"]
    ) > tolerance:
        raise RuntimeError("active terminal prefix misses the guide-root seam")
    return handoff


def _active_terminal_payload_hash(payload: Mapping[str, Any]) -> str:
    """Hash a locus payload exactly as its audit generator does."""

    body = dict(payload)
    body.pop("locus_payload_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_active_terminal_loci(path, *, capture_path) -> dict[str, Any]:
    """Load and structurally validate the exact 2,400 winding-locus API.

    This artifact authorizes only the sampled active winding terminal route.
    It deliberately carries no authority for flexible-conductor behavior or
    the park/index/load/unload portions of the continuous route.
    """

    locus_path = Path(path).resolve()
    if not locus_path.is_file():
        raise RuntimeError(
            "canonical raw player requires generated active-sector terminal "
            f"loci: {locus_path}"
        )
    payload = json.loads(locus_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError("active-sector terminal loci must be one JSON object")
    if payload.get("schema") != ACTIVE_TERMINAL_LOCI_SCHEMA:
        raise RuntimeError("active-sector terminal locus schema drifted")
    expected_hash = payload.get("locus_payload_sha256")
    if (not isinstance(expected_hash, str)
            or expected_hash != _active_terminal_payload_hash(payload)):
        raise RuntimeError("active-sector terminal locus payload hash drifted")
    if "torus" in json.dumps(payload, sort_keys=True).lower():
        raise RuntimeError("obsolete torus metadata leaked into terminal loci")

    run = payload.get("run")
    loci = payload.get("loci")
    segment_contract = payload.get("segment_contract")
    if not isinstance(run, Mapping) or not isinstance(loci, list):
        raise RuntimeError("active-sector terminal locus run/loci are malformed")
    if (run.get("locus_count") != EXPECTED_ACTIVE_TERMINAL_LOCI
            or len(loci) != EXPECTED_ACTIVE_TERMINAL_LOCI):
        raise RuntimeError("active-sector terminal route is not exactly 2,400 loci")
    capture = Path(capture_path).resolve()
    if run.get("capture_sha256") != _file_sha256(capture):
        raise RuntimeError("active-sector terminal loci bind another raw capture")
    if not isinstance(segment_contract, Mapping) or not segment_contract:
        raise RuntimeError("active-sector terminal segment contract is missing")
    for name, contract in segment_contract.items():
        if (not isinstance(name, str) or not isinstance(contract, Mapping)
                or any(not isinstance(contract.get(field), str)
                       for field in ("surface_owner", "local_frame", "authority"))):
            raise RuntimeError("active-sector terminal segment metadata is malformed")
    if FLYER_REFERENCE_SEGMENT not in segment_contract:
        raise RuntimeError("active-sector terminal flyer reference contract is missing")
    flyer_reference = payload.get("flyer_reference")
    if (not isinstance(flyer_reference, Mapping)
            or flyer_reference.get("frame")
            != "flyer_reference_M2_axis_plus_Z"):
        raise RuntimeError("active-sector terminal flyer reference is malformed")
    flyer_samples = flyer_reference.get(FLYER_REFERENCE_SAMPLES_FIELD)
    if not isinstance(flyer_samples, list) or len(flyer_samples) < 2:
        raise RuntimeError("active-sector terminal flyer reference has no polyline")
    full_flyer_samples = flyer_reference.get(
        "full_geometric_bore_local_samples_mm"
    )
    if (not isinstance(full_flyer_samples, list)
            or flyer_reference.get("full_geometric_bore_point_count")
            != len(full_flyer_samples)
            or flyer_reference.get("conductor_prefix_point_count")
            != len(flyer_samples)
            or full_flyer_samples[:len(flyer_samples)] != flyer_samples):
        raise RuntimeError("active-sector terminal flyer reference prefix drifted")
    for point_index, point in enumerate(flyer_samples):
        if not isinstance(point, list) or len(point) != 3:
            raise RuntimeError("active-sector terminal flyer reference point is malformed")
        for component in point:
            _finite(component, f"flyer reference point {point_index}")
    expected_names = set(segment_contract) - {FLYER_REFERENCE_SEGMENT}
    prior_time = -math.inf
    for index, locus in enumerate(loci):
        if not isinstance(locus, Mapping):
            raise RuntimeError(f"active terminal locus {index} is malformed")
        pass_index, state_index = divmod(index, 100)
        expected_identity = {
            "locus_index": index,
            "pass_index": pass_index,
            "state_index": state_index,
            "turn_index": state_index // 2,
            "half_turn_index": state_index & 1,
        }
        for field, expected in expected_identity.items():
            if locus.get(field) != expected:
                raise RuntimeError(
                    f"active terminal locus {index} {field} drifted"
                )
        time_s = _finite(locus.get("time_s"), f"locus {index} time_s")
        if time_s <= prior_time:
            raise RuntimeError("active terminal locus times are not strictly ordered")
        prior_time = time_s
        axes = locus.get("axes")
        if not isinstance(axes, Mapping):
            raise RuntimeError(f"active terminal locus {index} axes are malformed")
        for name in ("M0_raw_rad", "M1_spindle_rad", "M2_flyer_rad"):
            _finite(axes.get(name), f"locus {index} axes.{name}")
        flyer_angle = _finite(
            axes.get("M2_flyer_rad"), f"locus {index} axes.M2_flyer_rad"
        )
        reference_end = flyer_samples[-1]
        c, s = math.cos(flyer_angle), math.sin(flyer_angle)
        prior_endpoint = [
            c * float(reference_end[0]) - s * float(reference_end[1]),
            s * float(reference_end[0]) + c * float(reference_end[1]),
            float(reference_end[2]),
        ]
        segments = locus.get("segments")
        if not isinstance(segments, list) or not segments:
            raise RuntimeError(f"active terminal locus {index} has no segments")
        observed_names: set[str] = set()
        for segment_index, segment in enumerate(segments):
            if not isinstance(segment, Mapping):
                raise RuntimeError(
                    f"active terminal locus {index} segment is malformed"
                )
            name = segment.get("name")
            if not isinstance(name, str) or name not in expected_names:
                raise RuntimeError(
                    f"active terminal locus {index} has an unknown segment"
                )
            if name in observed_names:
                raise RuntimeError(
                    f"active terminal locus {index} duplicates segment {name}"
                )
            observed_names.add(name)
            points = segment.get("machine_world_samples_mm")
            if not isinstance(points, list) or len(points) < 2:
                raise RuntimeError(
                    f"active terminal locus {index} segment {name} has no polyline"
                )
            for point_index, point in enumerate(points):
                if not isinstance(point, list) or len(point) != 3:
                    raise RuntimeError(
                        f"active terminal locus {index} segment {name} point is malformed"
                    )
                for component in point:
                    _finite(
                        component,
                        f"locus {index} segment {segment_index} point {point_index}",
                    )
            if _wire_point_gap(prior_endpoint, points[0]) > 2.0e-6:
                raise RuntimeError(
                    f"active terminal locus {index} disconnects before {name}"
                )
            prior_endpoint = points[-1]
        if observed_names != expected_names:
            raise RuntimeError(
                f"active terminal locus {index} segment set is incomplete"
            )
    return payload


def _active_terminal_max_edges(payload: Mapping[str, Any]) -> int:
    """Return the exact largest per-locus polyline edge requirement."""

    reference_edges = len(
        payload["flyer_reference"][FLYER_REFERENCE_SAMPLES_FIELD]
    ) - 1
    return reference_edges + max(
        sum(
            len(segment["machine_world_samples_mm"]) - 1
            for segment in locus["segments"]
        )
        for locus in payload["loci"]
    )


def _validate_active_terminal_timeline(
        payload: Mapping[str, Any], half_turns: list[dict[str, Any]],
        timeline: Timeline) -> None:
    """Bind every exact locus to its raw half-turn crossing and axis pose."""

    loci = payload["loci"]
    if len(half_turns) != len(loci):
        raise RuntimeError("active terminal loci and raw half turns differ")
    axis_names = ("M0_raw_rad", "M1_spindle_rad", "M2_flyer_rad")
    for index, (locus, half_turn) in enumerate(zip(loci, half_turns)):
        if (locus["pass_index"] != half_turn["passIndex"]
                or locus["state_index"] != half_turn["halfTurnIndex"]):
            raise RuntimeError(
                f"active terminal locus {index} raw half-turn identity drifted"
            )
        time_s = float(locus["time_s"])
        if not _same_number(time_s, half_turn["start"], tolerance=1.0e-6):
            raise RuntimeError(
                f"active terminal locus {index} is not at its half-turn start"
            )
        pose = timeline.pose_at(time_s)
        for axis_index, name in enumerate(axis_names):
            if not _same_number(
                    locus["axes"][name], pose[axis_index], tolerance=2.0e-6):
                raise RuntimeError(
                    f"active terminal locus {index} {name} differs from Timeline"
                )


def quat_y(a):
    return [0.0, math.sin(a / 2), 0.0, math.cos(a / 2)]


def quat_z(a):
    return [0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]


def _round_knots(track):
    """Compact a trajectory for the player's numeric axis readout."""
    return [[round(t, 4), round(p, 9)] for t, p in track.knots]


def _rotation_intervals(track, start, end, count, interval_angle,
                        direction=None):
    """Physical intervals at directed or absolute angular crossings.

    ``direction`` is ``+1``/``-1`` for a controller-authoritative monotonic
    move.  In that mode every crossing is solved against the reconstructed
    piecewise-linear position, and a pre-crossing reversal fails closed.
    ``None`` retains the older absolute-travel diagnostic behavior.
    """
    if count < 0 or interval_angle <= 0.0:
        raise ValueError("rotation interval count/angle must be positive")
    points = [(start, track.pos_at(start))]
    points.extend((t, p) for t, p in track.knots if start < t < end)
    points.append((end, track.pos_at(end)))
    points.sort()

    result = []
    travelled = 0.0
    threshold = interval_angle
    interval_start = None
    start_position = track.pos_at(start)
    for (t0, p0), (t1, p1) in zip(points, points[1:]):
        signed_distance = p1 - p0
        if direction is not None and direction not in (-1, 1):
            raise ValueError("rotation direction must be -1, +1, or None")
        if (direction is not None and len(result) < count
                and direction * signed_distance < -1.0e-9):
            raise RuntimeError(
                "axis reversed before all directed physical crossings"
            )
        distance = (abs(signed_distance) if direction is None
                    else direction * signed_distance)
        if distance <= 1e-12 or t1 <= t0:
            continue
        if interval_start is None:
            interval_start = t0
        while (len(result) < count and
               travelled + distance + 1e-9 >= threshold):
            fraction = (threshold - travelled) / distance
            interval_end = t0 + fraction * (t1 - t0)
            expected = (start_position
                        + (1 if direction is None else direction) * threshold)
            if (direction is not None
                    and abs(track.pos_at(interval_end) - expected) > 2.0e-6):
                raise RuntimeError(
                    "directed physical crossing does not match axis track"
                )
            result.append([round(interval_start, 6), round(interval_end, 6)])
            interval_start = interval_end
            threshold += interval_angle
        travelled += distance
        if len(result) == count:
            break
    return result


def _turn_intervals(track, start, end, count):
    return _rotation_intervals(
        track, start, end, count, 2.0 * math.pi, direction=None,
    )


def _turn_crossings(track, start, end, count):
    """Compatibility view containing only completed-turn timestamps."""
    return [interval[1] for interval in
            _turn_intervals(track, start, end, count)]


def _require_equal(label: str, expected: Any, actual: Any) -> None:
    if isinstance(expected, (int, float)) and not isinstance(expected, bool):
        equal = _same_number(expected, actual)
    else:
        equal = expected == actual
    if not equal:
        raise RuntimeError(
            f"player artifact identity mismatch for {label}: "
            f"plan={expected!r}, capture/model={actual!r}"
        )


def _validate_player_artifact_identity(meta, manifest, slot_plan):
    """Bind the browser state to one exact plan, capture, and CAD manifest."""
    capture_plan = meta.get("winding_plan")
    if not isinstance(capture_plan, Mapping):
        raise RuntimeError(
            "capture has no winding_plan identity; recapture with the "
            "plan-scheduled controller"
        )
    required_capture_fields = {
        "schema": slot_plan["schema"],
        "sha256": slot_plan["artifactSha256"],
        "proof_sha256": slot_plan["proofSha256"],
        "transition_status": "PASS",
        "nominal_wire_mm": slot_plan["job"]["wireFinishedDiameter"],
        "model_wire_envelope_mm": slot_plan["job"]["modelWireEnvelope"],
        "receiving_sensitivity_wire_envelope_mm": (
            slot_plan["receivingSensitivity"]["wireEnvelope"]),
        "receiving_sensitivity_status": (
            slot_plan["receivingSensitivity"]["status"]),
        "turns_per_tooth": slot_plan["job"]["turnsPerTooth"],
        "placement_count": slot_plan["placementCount"],
        "half_turn_center_count": slot_plan["halfTurnCenterCount"],
    }
    for field, expected in required_capture_fields.items():
        if field not in capture_plan:
            raise RuntimeError(f"capture winding_plan.{field} is missing")
        _require_equal(
            f"capture.winding_plan.{field}", expected, capture_plan[field])

    capture_path = capture_plan.get("path")
    if capture_path is not None and Path(capture_path).resolve() != Path(
            slot_plan["path"]).resolve():
        raise RuntimeError(
            "capture winding_plan.path does not identify the loaded artifact"
        )

    _require_equal("capture turns", slot_plan["job"]["turnsPerTooth"],
                   meta.get("turns"))
    _require_equal("capture teeth", slot_plan["job"]["slots"],
                   meta.get("teeth_count"))
    capture_job = meta.get("job")
    if not isinstance(capture_job, Mapping):
        raise RuntimeError("capture job identity is missing")
    job_checks = {
        "slots": slot_plan["job"]["slots"],
        "od_mm": slot_plan["job"]["od"],
        "stack_mm": slot_plan["job"]["stack"],
        "wire_finished_d_mm": slot_plan["job"]["wireFinishedDiameter"],
        "liner_max_thickness_mm": slot_plan["job"]["linerMaximumThickness"],
    }
    for field, expected in job_checks.items():
        if field not in capture_job:
            raise RuntimeError(f"capture job.{field} is missing")
        _require_equal(f"capture job.{field}", expected, capture_job[field])

    stator = manifest.get("stator")
    if not isinstance(stator, Mapping):
        raise RuntimeError("CAD manifest stator identity is missing")
    manifest_checks = {
        "slots": slot_plan["job"]["slots"],
        "od": slot_plan["job"]["od"],
        "stack": slot_plan["job"]["stack"],
        "wire_d": slot_plan["job"]["wireFinishedDiameter"],
        "turns": slot_plan["job"]["turnsPerTooth"],
    }
    for field, expected in manifest_checks.items():
        if field not in stator:
            raise RuntimeError(f"CAD manifest stator.{field} is missing")
        _require_equal(f"CAD manifest stator.{field}", expected, stator[field])


def _validate_raw_player_artifact_identity(meta, manifest, slot_plan):
    """Bind a raw player to unmodified upstream commands without promoting
    the diagnostic constructive plan to controller authority.
    """
    if meta.get("controller_mode") != "upstream":
        raise RuntimeError("raw player requires controller_mode=upstream")
    if meta.get("controller_adapter_sha256") is not None:
        raise RuntimeError("raw player refuses a project-adapter capture")
    if meta.get("winding_plan") is not None:
        raise RuntimeError(
            "raw player requires no controller-bound winding_plan identity"
        )
    if meta.get("capture_schema") != 4:
        raise RuntimeError("raw player requires capture schema 4")
    _require_equal("raw capture turns", slot_plan["job"]["turnsPerTooth"],
                   meta.get("turns"))
    _require_equal("raw capture teeth", slot_plan["job"]["slots"],
                   meta.get("teeth_count"))
    capture_job = meta.get("job")
    if not isinstance(capture_job, Mapping):
        raise RuntimeError("raw capture job identity is missing")
    job_checks = {
        "slots": slot_plan["job"]["slots"],
        "od_mm": slot_plan["job"]["od"],
        "stack_mm": slot_plan["job"]["stack"],
        "wire_finished_d_mm": slot_plan["job"]["wireFinishedDiameter"],
        "liner_max_thickness_mm": slot_plan["job"]["linerMaximumThickness"],
    }
    for field, expected in job_checks.items():
        if field not in capture_job:
            raise RuntimeError(f"raw capture job.{field} is missing")
        _require_equal(f"raw capture job.{field}", expected,
                       capture_job[field])
    stator = manifest.get("stator")
    if not isinstance(stator, Mapping):
        raise RuntimeError("CAD manifest stator identity is missing")
    manifest_checks = {
        "slots": slot_plan["job"]["slots"],
        "od": slot_plan["job"]["od"],
        "stack": slot_plan["job"]["stack"],
        "wire_d": slot_plan["job"]["wireFinishedDiameter"],
        "turns": slot_plan["job"]["turnsPerTooth"],
    }
    for field, expected in manifest_checks.items():
        if field not in stator:
            raise RuntimeError(f"CAD manifest stator.{field} is missing")
        _require_equal(f"CAD manifest stator.{field}", expected, stator[field])
    velocities = meta.get("velocities")
    if (not isinstance(velocities, list) or len(velocities) != 4
            or any(_finite(value, "raw capture velocity") <= 0.0
                   for value in velocities)):
        raise RuntimeError("raw capture velocities are malformed")


def _m0_target_for_radial(meta, radial_mm):
    job = meta["job"]
    radial_start, radial_end = map(float, job["radial_winding_span_mm"])
    m0_start, m0_end = map(float, meta["m0_wind_range"])
    if not radial_start < radial_end or not m0_start < m0_end:
        raise RuntimeError("capture radial/M0 winding ranges must increase")
    fraction = (float(radial_mm) - radial_start) / (
        radial_end - radial_start)
    return m0_start + fraction * (m0_end - m0_start)


def _packing_progress(events, windings, meta, slot_plan):
    """Build exact turn clocks from explicit pass origins and crossings.

    Collision-offset passes can begin between half-turn crossings.  Their first
    plan center completes the remaining partial half-turn, whereas a zero-
    offset pass begins at placement center zero and completes its last half-turn
    at the first final-hold crossing.  The capture origin record selects the
    correct interpretation; playback never guesses a phase-zero origin.
    """
    grouped = {}
    origins = {}
    for event in events:
        if event.get("e") == "packing_pass_origin":
            pass_index = event.get("pass_index")
            if not isinstance(pass_index, int) or pass_index in origins:
                raise RuntimeError(
                    "packing pass origin index must be unique integers")
            origins[pass_index] = event
            continue
        if event.get("e") != "packing_waypoint":
            continue
        pass_index = event.get("pass_index")
        waypoint_index = event.get("waypoint_index")
        if not isinstance(pass_index, int) or not isinstance(waypoint_index, int):
            raise RuntimeError("packing waypoint pass/index must be integers")
        grouped.setdefault(pass_index, []).append(event)

    expected_passes = set(range(len(windings)))
    if set(grouped) != expected_passes:
        raise RuntimeError(
            "capture packing waypoints do not cover every winding pass exactly"
        )
    if set(origins) != expected_passes:
        raise RuntimeError(
            "capture packing origins do not cover every winding pass exactly"
        )
    # Upstream's third wind_wire argument is phase-local (0..7) and therefore
    # repeats three times.  Preserve it as windIndex, while all capture packing
    # evidence uses the controller's monotonic global pass index (0..23).
    for global_pass_index, winding in enumerate(windings):
        winding["windIndex"] = winding.get("windIndex", winding["passIndex"])
        winding["passIndex"] = global_pass_index
    turns = slot_plan["job"]["turnsPerTooth"]
    center_count = 2 * turns
    all_half_turns = []
    depositions = []
    for winding in windings:
        pass_index = winding["passIndex"]
        origin = origins[pass_index]
        rows = grouped.get(pass_index, [])
        rows.sort(key=lambda row: row["waypoint_index"])
        if len(rows) not in (center_count, center_count + 1,
                             center_count + 2):
            raise RuntimeError(
                f"packing pass {pass_index} has {len(rows)} waypoint events; "
                f"expected {center_count} placement centers plus 0..2 holds"
            )
        if [row["waypoint_index"] for row in rows] != list(range(len(rows))):
            raise RuntimeError(
                f"packing pass {pass_index} waypoint index sequence is not exact"
            )
        if any(float(right["t"]) + 1e-9 < float(left["t"])
               for left, right in zip(rows, rows[1:])):
            raise RuntimeError(
                f"packing pass {pass_index} waypoint time sequence regressed"
            )

        start_phase = _finite(
            origin.get("start_phase_rad"),
            f"packing pass {pass_index} start phase")
        phase_origin = _finite(
            origin.get("phase_origin_rad"),
            f"packing pass {pass_index} phase origin")
        first_crossing = _finite(
            origin.get("first_crossing_phase_rad"),
            f"packing pass {pass_index} first crossing")
        actual_travel = _finite(
            origin.get("actual_travel_rad"),
            f"packing pass {pass_index} actual travel")
        final_hold_phase = _finite(
            origin.get("final_hold_phase_rad"),
            f"packing pass {pass_index} final hold phase")
        if (not _same_number(first_crossing, phase_origin)
                or not _same_number(actual_travel, final_hold_phase)
                or start_phase < -1e-9 or start_phase > math.pi + 1e-6
                or phase_origin + 1e-9 < start_phase
                or phase_origin > math.pi + 1e-6):
            raise RuntimeError(
                f"packing pass {pass_index} phase-origin contract is malformed"
            )
        if origin.get("expected_deposition_center_count") != center_count:
            raise RuntimeError(
                f"packing pass {pass_index} origin center count is wrong")
        if (origin.get("placement_zero_settled_before_first_crossing") is not True
                or origin.get("pre_crossing_deposition_count") != 0):
            raise RuntimeError(
                f"packing pass {pass_index} lacks fail-closed pre-crossing evidence"
            )

        centers = [row for row in rows
                   if row.get("kind") == "placement_center"]
        holds = [row for row in rows if row.get("kind") == "final_hold"]
        if len(centers) != center_count or len(centers) + len(holds) != len(rows):
            raise RuntimeError(
                f"packing pass {pass_index} must contain exactly "
                f"{center_count} placement_center events and only final holds"
            )
        for center_index, row in enumerate(centers):
            expected_phase = phase_origin + center_index * math.pi
            if not _same_number(
                    row.get("m2_phase_rad"), expected_phase,
                    tolerance=1e-7):
                raise RuntimeError(
                    f"packing pass {pass_index} center {center_index} "
                    "does not match its explicit phase origin"
                )
            placement_index = center_index // 2
            if row.get("placement_index") != placement_index:
                raise RuntimeError(
                    f"packing pass {pass_index} center {center_index} "
                    "does not identify the plan placement"
                )
            placement = slot_plan["placements"][placement_index]
            expected_m0 = placement["m0Target"]
            if not _same_number(
                    row.get("m0_target_rad"), expected_m0,
                    tolerance=2e-6):
                raise RuntimeError(
                    f"packing pass {pass_index} center {center_index} "
                    "M0 target does not map to the plan placement"
                )
        for hold_index, row in enumerate(holds):
            if row.get("placement_index") != turns - 1:
                raise RuntimeError(
                    f"packing pass {pass_index} final hold {hold_index} "
                    "does not retain placement 49"
                )
            expected_m0 = slot_plan["placements"][-1]["m0Target"]
            if not _same_number(
                    row.get("m0_target_rad"), expected_m0,
                    tolerance=2e-6):
                raise RuntimeError(
                    f"packing pass {pass_index} final hold M0 target drifted"
                )

        for waypoint_index, row in enumerate(rows):
            error = abs(_finite(
                row.get("m0_error_rad"),
                f"packing pass {pass_index} waypoint M0 error",
            ))
            if error > 0.0200001:
                raise RuntimeError(
                    f"packing pass {pass_index} waypoint {waypoint_index} "
                    f"M0 error {error:.6f} rad exceeds controller contract"
                )
            if row.get("m0_settled_before_crossing") is not True:
                raise RuntimeError(
                    f"packing pass {pass_index} waypoint {waypoint_index} "
                    "was not settled before its crossing"
                )
            ready_phase = _finite(
                row.get("m0_ready_phase_rad"),
                f"packing pass {pass_index} waypoint ready phase")
            crossing_phase = _finite(
                row.get("m2_phase_rad"),
                f"packing pass {pass_index} waypoint crossing phase")
            if ready_phase > crossing_phase + 1e-9:
                raise RuntimeError(
                    f"packing pass {pass_index} waypoint {waypoint_index} "
                    "M0 became ready after its crossing"
                )
        if not _same_number(
                rows[-1].get("m2_phase_rad"), actual_travel,
                tolerance=1e-7):
            raise RuntimeError(
                f"packing pass {pass_index} final event does not reach target")
        if rows[-1].get("placement_index") != turns - 1:
            raise RuntimeError(
                f"packing pass {pass_index} final event does not hold placement 49")

        winding["turnTimes"] = []
        winding["halfTurnTimes"] = []
        zero_offset = phase_origin <= start_phase + 1e-6
        if zero_offset:
            # Center zero is the initial boundary.  The first hold after center
            # 99 closes the hundredth half turn.
            if not holds:
                raise RuntimeError(
                    f"zero-offset packing pass {pass_index} lacks its closing hold")
            starts = centers
            ends = centers[1:] + [holds[0]]
        else:
            # The first center closes the remainder of a half turn already in
            # progress at the captured origin; later centers close full halves.
            starts = [origin] + centers[:-1]
            ends = centers

        for half_index, (start_row, end_row) in enumerate(zip(starts, ends)):
            start = float(start_row["t"])
            end = float(end_row["t"])
            if end + 1e-9 < start:
                raise RuntimeError(
                    f"packing pass {pass_index} half turn {half_index} time regressed")
            placement_index = half_index // 2
            placement = slot_plan["placements"][placement_index]
            first_side_is_right = bool(winding["clockwise"])
            right_side = first_side_is_right == (half_index % 2 == 0)
            half = {
                "tooth": winding["tooth"],
                "phase": winding["phase"],
                "passIndex": pass_index,
                "turn": placement_index,
                "half": half_index % 2,
                "halfTurnIndex": half_index,
                "side": "right" if right_side else "left",
                "start": start,
                "end": end,
                "placementIndex": placement_index,
                "contact": placement["contact"],
                "support": placement["support"],
                "supportPredecessors": placement["supportPredecessors"],
                "phaseOrigin": phase_origin,
                "partialFromOffsetOrigin": bool(not zero_offset
                                                  and half_index == 0),
            }
            all_half_turns.append(half)
            winding["halfTurnTimes"].append(end)
        for turn_index in range(turns):
            left = all_half_turns[-2 * turns + 2 * turn_index]
            right = all_half_turns[-2 * turns + 2 * turn_index + 1]
            deposition = {
                "tooth": winding["tooth"],
                "phase": winding["phase"],
                "passIndex": pass_index,
                "turn": turn_index,
                "placementIndex": turn_index,
                "start": left["start"],
                "midpoint": left["end"],
                "end": right["end"],
                "clockwise": winding["clockwise"],
                "contact": left["contact"],
                "support": left["support"],
                "supportPredecessors": left["supportPredecessors"],
            }
            depositions.append(deposition)
            winding["turnTimes"].append(deposition["end"])

        if winding["turnTimes"][-1] > winding["end"] + 1e-6:
            raise RuntimeError(
                f"packing pass {pass_index} completed after wind_wire_done"
            )
    return all_half_turns, depositions


def _raw_active_radial(meta, manifest, timeline, time):
    """Presented-tooth radius selected by reconstructed raw M0 motion."""
    contact_z = _finite(
        meta["job"].get("wire_contact_z_mm"),
        "raw capture job.wire_contact_z_mm",
    )
    axis_z = (_finite(manifest["m0_home_standoff"], "M0 standoff")
              + timeline.axes[0].pos_at(time)
              * _finite(manifest["mm_per_rad_m0"], "M0 mm/rad"))
    return axis_z - contact_z


def _raw_upstream_progress(windings, timeline, meta, manifest, slot_plan):
    """Derive 50 physical turns/pass from actual Timeline M2 crossings.

    Raw M0 selects the radial locus.  The browser may use the diagnostic plan's
    tangential/layer offsets to make the elastic coil buildup visible, but that
    geometry is explicitly approximate and never relabeled as raw authority.
    """
    turns = int(meta["turns"])
    if turns != slot_plan["job"]["turnsPerTooth"]:
        raise RuntimeError("raw turns and diagnostic presentation plan differ")
    all_half_turns = []
    depositions = []
    for global_pass_index, winding in enumerate(windings):
        winding["windIndex"] = winding.get("passIndex", global_pass_index)
        winding["passIndex"] = global_pass_index
        direction = 1 if winding["clockwise"] else -1
        intervals = _rotation_intervals(
            timeline.axes[2],
            _finite(winding["motionStart"], "raw winding motionStart"),
            _finite(winding["end"], "raw winding end"),
            2 * turns,
            math.pi,
            direction=direction,
        )
        if len(intervals) != 2 * turns:
            raise RuntimeError(
                f"raw pass {global_pass_index} contains {len(intervals)} of "
                f"{2 * turns} physical half-turn crossings"
            )
        winding["turnTimes"] = []
        winding["halfTurnTimes"] = []
        winding_half_turns = []
        for half_index, (start, end) in enumerate(intervals):
            placement_index = half_index // 2
            first_side_is_right = bool(winding["clockwise"])
            right_side = first_side_is_right == (half_index % 2 == 0)
            raw_m0_start = timeline.axes[0].pos_at(start)
            raw_m0_end = timeline.axes[0].pos_at(end)
            raw_radial_start = _raw_active_radial(
                meta, manifest, timeline, start,
            )
            raw_radial_end = _raw_active_radial(
                meta, manifest, timeline, end,
            )
            half = {
                "tooth": int(winding["tooth"]),
                "phase": int(winding["phase"]),
                "passIndex": global_pass_index,
                "turn": placement_index,
                "half": half_index % 2,
                "halfTurnIndex": half_index,
                "side": "right" if right_side else "left",
                "start": float(start),
                "end": float(end),
                "placementIndex": placement_index,
                "visualPlacementIndex": placement_index,
                "rawM0StartRad": round(float(raw_m0_start), 9),
                "rawM0EndRad": round(float(raw_m0_end), 9),
                "rawActiveRadialStartMm": round(float(raw_radial_start), 9),
                "rawActiveRadialEndMm": round(float(raw_radial_end), 9),
                "contact": "observed_raw_M0_radial_locus",
                "support": "approximate_elastic_slot_placement",
                "supportPredecessors": [],
                "clockAuthority": (
                    "physical M2 pi crossing from reconstructed upstream "
                    "constant-velocity Timeline"
                ),
                "radialAuthority": (
                    "reconstructed raw M0 motion mapped through CAD mm/rad"
                ),
                "tangentialAuthority": (
                    "diagnostic constructive-plan offset for watchability; "
                    "not raw controller authority"
                ),
            }
            all_half_turns.append(half)
            winding_half_turns.append(half)
            winding["halfTurnTimes"].append(float(end))

        for turn_index in range(turns):
            first = winding_half_turns[2 * turn_index]
            second = winding_half_turns[2 * turn_index + 1]
            raw_radial = (
                first["rawActiveRadialEndMm"]
                + second["rawActiveRadialEndMm"]
            ) / 2.0
            deposition = {
                "tooth": int(winding["tooth"]),
                "phase": int(winding["phase"]),
                "passIndex": global_pass_index,
                "turn": turn_index,
                "placementIndex": turn_index,
                "visualPlacementIndex": turn_index,
                "start": first["start"],
                "midpoint": first["end"],
                "end": second["end"],
                "clockwise": bool(winding["clockwise"]),
                "rawM0StartRad": first["rawM0StartRad"],
                "rawM0MidpointRad": first["rawM0EndRad"],
                "rawM0EndRad": second["rawM0EndRad"],
                "rawActiveRadialFirstSideMm": first[
                    "rawActiveRadialEndMm"
                ],
                "rawActiveRadialSecondSideMm": second[
                    "rawActiveRadialEndMm"
                ],
                "rawActiveRadialMeanMm": round(float(raw_radial), 9),
                "visualActiveRadialMm": round(float(raw_radial), 9),
                "contact": "observed_raw_M0_radial_locus",
                "support": "approximate_elastic_slot_placement",
                "supportPredecessors": [],
                "geometryAuthority": (
                    "raw M0 radial progression; diagnostic plan tangential/"
                    "layer offset; approximate elastic final placement"
                ),
            }
            depositions.append(deposition)
            winding["turnTimes"].append(float(second["end"]))

        radial_values = [
            value
            for half in winding_half_turns
            for value in (
                half["rawActiveRadialStartMm"],
                half["rawActiveRadialEndMm"],
            )
        ]
        winding["physicalTurnCount"] = turns
        winding["physicalHalfTurnCount"] = 2 * turns
        winding["rawActiveRadialRangeMm"] = [
            min(radial_values), max(radial_values),
        ]
        winding["turnClockAuthority"] = (
            "Timeline M2 physical crossings; no packing-waypoint events"
        )
    return all_half_turns, depositions


def _contract_shaft_wraps(events):
    wraps = []
    active_wrap = None
    wrap_sequences = {}
    for event in events:
        if event["e"] != "shaft_wrap_phase":
            continue
        number = int(event["next_wire_idx"])
        phase = event.get("phase")
        wrap_sequences.setdefault(number, []).append(phase)
        if phase == "contact_start":
            if active_wrap is not None:
                raise RuntimeError("nested shaft-wrap contact markers")
            active_wrap = {
                "start": event["t"],
                "number": number,
                "startM1": float(event["m1_rad"]),
                "intervalAuthority": "explicit ContractWind contact markers",
            }
        elif phase == "contact_done":
            if active_wrap is None or active_wrap["number"] != number:
                raise RuntimeError("unpaired shaft-wrap contact_done marker")
            active_wrap["end"] = event["t"]
            end_m1 = float(event["m1_rad"])
            delta_m1 = end_m1 - active_wrap["startM1"]
            active_wrap["deltaM1"] = round(delta_m1, 9)
            active_wrap["direction"] = 1 if delta_m1 >= 0.0 else -1
            active_wrap["turns"] = round(
                abs(delta_m1) / (2.0 * math.pi), 6,
            )
            wraps.append(active_wrap)
            active_wrap = None
    expected_wrap_phases = [
        "prepark_start", "m0_parked", "contact_start", "contact_done",
    ]
    if (active_wrap is not None or len(wraps) != 2
            or sorted(wrap_sequences) != [1, 2]
            or any(wrap_sequences[number] != expected_wrap_phases
                   for number in (1, 2))):
        raise RuntimeError(
            f"incomplete shaft-wrap contact marker contract: {wrap_sequences}"
        )
    return wraps


def _raw_shaft_wraps(events, timeline):
    """Infer raw shaft contact only from high-level boundaries and M1 cmds."""
    wraps = []
    start_indices = [
        index for index, event in enumerate(events)
        if event.get("e") == "wind_wire_around_shaft"
    ]
    if len(start_indices) != 2:
        raise RuntimeError("raw capture does not contain two shaft-wrap calls")
    velocity = _finite(timeline.meta["velocities"][1], "raw M1 velocity")
    for number, start_index in enumerate(start_indices, start=1):
        done_index = next((
            index for index in range(start_index + 1, len(events))
            if events[index].get("e") == "wind_wire_around_shaft_done"
        ), None)
        if done_index is None:
            raise RuntimeError(f"raw shaft wrap {number} has no done marker")
        marker_start = events[start_index]
        marker_done = events[done_index]
        m1_commands = [
            (index, event)
            for index, event in enumerate(
                events[start_index + 1:done_index], start=start_index + 1,
            )
            if event.get("e") == "cmd" and event.get("m") == 1
        ]
        if len(m1_commands) != 1:
            raise RuntimeError(
                f"raw shaft wrap {number} has {len(m1_commands)} M1 commands"
            )
        command_index, command = m1_commands[0]
        command_t = _finite(command["t"], "raw shaft M1 command time")
        start_m1 = timeline.axes[1].pos_at(command_t)
        target_m1 = _finite(
            command.get("model_target", command.get("a")),
            "raw shaft M1 target",
        )
        delta_m1 = target_m1 - start_m1
        arrival_t = command_t + abs(delta_m1) / velocity
        done_t = _finite(marker_done["t"], "raw shaft done time")
        if arrival_t > done_t + 1.0e-9:
            raise RuntimeError(
                f"raw shaft wrap {number} M1 target arrives after done marker"
            )
        actual_end = timeline.axes[1].pos_at(arrival_t)
        if abs(actual_end - target_m1) > 2.0e-6:
            raise RuntimeError(
                f"raw shaft wrap {number} did not physically reach M1 target"
            )
        turns = abs(delta_m1) / (2.0 * math.pi)
        if turns <= 0.0:
            raise RuntimeError(
                f"raw shaft wrap {number} has no physical M1 travel"
            )
        wraps.append({
            "start": command_t,
            "end": round(arrival_t, 9),
            "number": number,
            "startM1": round(float(start_m1), 9),
            "endM1": round(float(target_m1), 9),
            "deltaM1": round(float(delta_m1), 9),
            "direction": 1 if delta_m1 >= 0.0 else -1,
            "turns": round(turns, 6),
            "sourceCommandIndex": command_index,
            "sourceCommand": command.get("command", "").strip(),
            "markerStart": float(marker_start["t"]),
            "markerDone": done_t,
            "intervalAuthority": (
                "inferred from exact raw M1 command + reconstructed Timeline "
                "arrival; high-level markers bound the inference"
            ),
        })
    return wraps


def _coil_start_events(windings, depositions):
    """Describe each tooth-pass start without inventing a lead conductor.

    A coil start is a playback annotation spanning the captured positioning
    window and the first exact plan turn.  This helper records the intended
    no-cut topology used by ContractWind and the integrated route.  The legacy
    raw-baseline caller explicitly clears that continuity claim because its CAD
    manifest has no route contract.
    """
    by_pass = {}
    for deposition in depositions:
        by_pass.setdefault(deposition["passIndex"], []).append(deposition)

    starts = []
    for expected_pass, winding in enumerate(windings):
        pass_index = winding.get("passIndex")
        if pass_index != expected_pass:
            raise RuntimeError("winding pass indices are not monotonic")
        rows = sorted(by_pass.get(pass_index, []), key=lambda row: row["turn"])
        if not rows or rows[0].get("turn") != 0:
            raise RuntimeError(
                f"winding pass {pass_index} has no exact first-turn deposition"
            )
        first = rows[0]
        positioning_start = _finite(
            winding.get("start"), f"winding pass {pass_index} start")
        tooth_presented_at = _finite(
            winding.get("positionedAt"),
            f"winding pass {pass_index} positionedAt",
        )
        motion_start = _finite(
            winding.get("motionStart"),
            f"winding pass {pass_index} motionStart",
        )
        lay_start = _finite(
            first.get("start"), f"winding pass {pass_index} first lay start")
        first_half_end = _finite(
            first.get("midpoint"),
            f"winding pass {pass_index} first half end",
        )
        first_turn_end = _finite(
            first.get("end"), f"winding pass {pass_index} first turn end")
        if not (positioning_start <= tooth_presented_at + 1e-9
                and tooth_presented_at <= motion_start + 1e-9
                and motion_start <= lay_start + 1e-9
                and lay_start <= first_half_end + 1e-9
                and first_half_end <= first_turn_end + 1e-9):
            raise RuntimeError(
                f"winding pass {pass_index} coil-start clock is not ordered"
            )
        clockwise = bool(winding["clockwise"])
        first_side = "right" if clockwise else "left"
        raw_clock = first.get("geometryAuthority") is not None
        starts.append({
            "event": "coil_start",
            "positioningStart": positioning_start,
            "toothPresentedAt": tooth_presented_at,
            "motionStart": motion_start,
            "layStart": lay_start,
            "firstHalfEnd": first_half_end,
            "firstTurnEnd": first_turn_end,
            "phase": int(winding["phase"]),
            "passIndex": pass_index,
            "tooth": int(winding["tooth"]),
            "direction": "clockwise" if clockwise else "counter-clockwise",
            "firstSide": first_side,
            "secondSide": "left" if clockwise else "right",
            "placementIndex": 0,
            "continuousConductor": True,
            "cutOrJoin": False,
            "continuityFromPassIndex": (
                None if pass_index == 0 else pass_index - 1
            ),
            "markerAuthority": (
                (
                    "annotation at the first physical raw M2 turn; marker "
                    "geometry uses approximate elastic slot placement and is "
                    "not raw controller authority"
                ) if raw_clock else (
                    "annotation at the exact first plan center; no synthetic "
                    "lead conductor geometry"
                )
            ),
            "turnClockAuthority": (
                "Timeline physical M2 crossing" if raw_clock
                else "observed ContractWind packing waypoint"
            ),
        })
    return starts


def _bind_coil_starts_to_terminal_loci(
        coil_starts: list[dict[str, Any]],
        payload: Mapping[str, Any]) -> None:
    """Attach each pass annotation to its first exact winding locus."""

    loci = payload.get("loci")
    if not isinstance(loci, list):
        raise RuntimeError("active terminal loci are missing for coil starts")
    for expected_pass, start in enumerate(coil_starts):
        if start.get("passIndex") != expected_pass:
            raise RuntimeError("coil-start pass order drifted")
        locus_index = expected_pass * ACTIVE_TERMINAL_LOCI_PER_PASS
        if locus_index >= len(loci):
            raise RuntimeError(f"coil start {expected_pass} has no first locus")
        locus = loci[locus_index]
        if (locus.get("locus_index") != locus_index
                or locus.get("pass_index") != expected_pass
                or locus.get("state_index") != 0
                or locus.get("turn_index") != 0
                or locus.get("half_turn_index") != 0):
            raise RuntimeError(
                f"coil start {expected_pass} first terminal locus drifted"
            )
        locus_time = _finite(
            locus.get("time_s"), f"coil start {expected_pass} locus time"
        )
        if not _same_number(
            locus_time, start.get("layStart"), tolerance=1.0e-6
        ):
            raise RuntimeError(
                f"coil start {expected_pass} lay clock misses first locus"
            )
        start["firstTerminalLocusIndex"] = locus_index
        start["firstTerminalLocusTime"] = locus_time
        start["firstTerminalLocusExact"] = True


def _player_data(events, timeline, manifest, export_speed, default_rate,
                 slot_plan=None, capture_path=None,
                 conductor_route_path=CONTINUOUS_CONDUCTOR_ROUTE,
                 active_terminal_loci_path=None):
    """Build the compact, self-contained state stream used by the player."""
    meta = next(event for event in events if event["e"] == "meta")
    commands = [
        [event["t"], event["m"], event.get("model_target", event["a"]),
         event.get("controller_target"), event.get("command", "").strip()]
        for event in events if event["e"] == "cmd"
    ]
    markers = [
        [event["t"], event["e"], event.get(
            "args",
            ([event.get("phase"), event.get("next_wire_idx")]
             if event.get("e") == "shaft_wrap_phase" else []),
        ),
         event.get("m2state")]
        for event in events if event["e"] not in ("meta", "cmd")
    ]

    raw_mode = (
        meta.get("controller_mode") == "upstream"
        and meta.get("controller_adapter_sha256") is None
        and meta.get("winding_plan") is None
    )
    contract_mode = (
        meta.get("controller_mode") == "contract"
        and isinstance(meta.get("winding_plan"), Mapping)
    )
    if not raw_mode and not contract_mode:
        raise RuntimeError(
            "capture is neither canonical raw-upstream nor plan-bound contract"
        )
    raw_conductor_mode = (
        _raw_player_manifest_role(manifest) if raw_mode else None
    )
    integrated_raw_route = raw_conductor_mode == INTEGRATED_CONDUCTOR_MODE

    windings = winding_windows(events)
    wraps = (_raw_shaft_wraps(events, timeline) if raw_mode
             else _contract_shaft_wraps(events))
    if raw_mode:
        for wrap in wraps:
            markers.extend([
                [wrap["start"], "raw_shaft_contact_start", [
                    wrap["number"], wrap["sourceCommand"],
                ], None],
                [wrap["end"], "raw_shaft_contact_done", [
                    wrap["number"], wrap["turns"],
                ], None],
            ])

    event_end = max(float(event["t"]) for event in events)
    virtual_duration = max(event_end, timeline.t_end)
    wire = manifest["wire"]
    wire_handoff = (
        conductor_route._wire_handoff_contract(manifest)
        if integrated_raw_route else None
    )
    active_terminal_guide = wire.get("active_terminal_guide")
    if not isinstance(active_terminal_guide, Mapping):
        if integrated_raw_route:
            raise RuntimeError(
                "integrated raw CAD manifest lacks the final one-piece PEEK "
                "active terminal guide"
            )
        legacy_guide = wire.get("tip_guide")
        if not isinstance(legacy_guide, Mapping):
            raise RuntimeError("diagnostic CAD manifest has no guide origin")
        active_terminal_guide = {
            "unproved_transition_origin_local_mm": legacy_guide[
                "feed_local_mm"
            ],
            "review_focus_center_local_mm": legacy_guide["center_local_mm"],
        }
    stator = manifest["stator"]
    slot_plan = (_load_player_slot_plan() if slot_plan is None else slot_plan)
    if raw_mode:
        _validate_raw_player_artifact_identity(meta, manifest, slot_plan)
        slot_plan = {
            **slot_plan,
            "presentationRole": (
                "diagnostic approximate elastic slot placement; not raw "
                "controller authority"
            ),
        }
    else:
        _validate_player_artifact_identity(meta, manifest, slot_plan)
        slot_plan = {
            **slot_plan,
            "presentationRole": (
                "exact ContractWind constructive placement authority"
            ),
        }
    continuous_route = None
    continuous_route_path_value = None
    continuous_route_artifact_sha256 = None
    active_terminal_loci = None
    active_terminal_loci_path_value = None
    active_terminal_loci_artifact_sha256 = None
    conductor_evidence = {
        "mode": "contract_plan_visualization",
        "playerGoverning": False,
        "continuousRouteAvailable": False,
        "activeTerminalLociAvailable": False,
        "continuousConductorAuthorized": False,
        "productionAuthorized": False,
        "reason": (
            "ContractWind diagnostic rendering has no continuous-conductor "
            "authority."
        ),
    }
    if raw_mode and capture_path is None:
        raise RuntimeError("raw player must bind an explicit capture path")
    if integrated_raw_route:
        continuous_route_path_obj = Path(conductor_route_path).resolve()
        continuous_route = _load_continuous_conductor_route(
            continuous_route_path_obj,
            capture_path=capture_path,
            manifest_path=OUT / "links" / "manifest.json",
        )
        continuous_route_path_value = str(continuous_route_path_obj)
        continuous_route_artifact_sha256 = _file_sha256(
            continuous_route_path_obj
        )
        observed_order = [{
            "pass_index": index,
            "phase": int(winding["phase"]),
            "tooth": int(winding["tooth"]),
            "clockwise": bool(winding["clockwise"]),
        } for index, winding in enumerate(windings)]
        if continuous_route.get("pass_order") != observed_order:
            raise RuntimeError(
                "continuous conductor route pass order differs from raw capture"
            )
        if continuous_route.get("wire_handoff_contract") != wire_handoff:
            raise RuntimeError(
                "continuous conductor route wire handoff differs from CAD manifest"
            )
        active_terminal_loci_path_obj = Path(
            active_terminal_loci_path
            if active_terminal_loci_path is not None else
            OUT / "reports" / ACTIVE_TERMINAL_LOCI_NAME
        ).resolve()
        active_terminal_loci = _load_active_terminal_loci(
            active_terminal_loci_path_obj,
            capture_path=capture_path,
        )
        locus_binding = manifest.get("active_terminal_locus_route")
        if not isinstance(locus_binding, Mapping):
            raise RuntimeError(
                "canonical raw CAD manifest does not bind the active terminal loci"
            )
        bound_relative = locus_binding.get("file")
        if (not isinstance(bound_relative, str)
                or Path(bound_relative).is_absolute()):
            raise RuntimeError("active terminal locus manifest path is unsafe")
        bound_path = (OUT / bound_relative).resolve()
        out_root = OUT.resolve()
        if (bound_path != out_root and out_root not in bound_path.parents):
            raise RuntimeError("active terminal locus manifest path escapes asset root")
        if bound_path != active_terminal_loci_path_obj:
            raise RuntimeError("active terminal locus path differs from CAD manifest")
        if (locus_binding.get("schema") != ACTIVE_TERMINAL_LOCI_SCHEMA
                or locus_binding.get("locus_count")
                != EXPECTED_ACTIVE_TERMINAL_LOCI
                or locus_binding.get("artifact_sha256")
                != _file_sha256(active_terminal_loci_path_obj)
                or locus_binding.get("locus_payload_sha256")
                != active_terminal_loci["locus_payload_sha256"]
                or locus_binding.get("park_index_load_unload_proven") is not False
                or locus_binding.get("sag_tension_settling_neatness_proven")
                is not False):
            raise RuntimeError("active terminal locus CAD manifest binding drifted")
        active_terminal_loci_path_value = str(active_terminal_loci_path_obj)
        active_terminal_loci_artifact_sha256 = _file_sha256(
            active_terminal_loci_path_obj
        )
        conductor_evidence = {
            "mode": INTEGRATED_CONDUCTOR_MODE,
            "playerGoverning": True,
            "continuousRouteAvailable": True,
            "activeTerminalLociAvailable": True,
            "continuousConductorAuthorized": False,
            "productionAuthorized": False,
            "reason": (
                "Selected integrated review binds the ordered presentation "
                "route and 2,400 exact terminal loci, while unsupported "
                "flexible intervals remain release-failed."
            ),
        }
    elif raw_mode:
        conductor_evidence = {
            "mode": LEGACY_CONDUCTOR_MODE,
            "playerGoverning": False,
            "continuousRouteAvailable": False,
            "activeTerminalLociAvailable": False,
            "continuousConductorAuthorized": False,
            "productionAuthorized": False,
            "reason": (
                "Legacy out/links assets contain rigid guide-wire meshes but "
                "no continuous route or active-terminal-locus contract."
            ),
        }
    wire_render_radius = _finite(
        wire.get(
            "render_radius",
            slot_plan["job"]["wireFinishedDiameter"] / 2.0,
        ),
        "CAD manifest wire.render_radius",
    )
    expected_render_radius = slot_plan["job"]["wireFinishedDiameter"] / 2.0
    if not _same_number(wire_render_radius, expected_render_radius):
        raise RuntimeError(
            "wire presentation radius must equal the job wire radius; "
            "oversized display tubes create false felt-pad intersections"
        )
    if raw_mode:
        half_turns, depositions = _raw_upstream_progress(
            windings, timeline, meta, manifest, slot_plan,
        )
        if integrated_raw_route:
            _validate_active_terminal_timeline(
                active_terminal_loci, half_turns, timeline,
            )
            wire_handoff = _validate_active_terminal_wire_chain(
                wire, active_terminal_loci
            )
    else:
        half_turns, depositions = _packing_progress(
            events, windings, meta, slot_plan,
        )
    coil_starts = _coil_start_events(windings, depositions)
    if integrated_raw_route:
        _bind_coil_starts_to_terminal_loci(
            coil_starts, active_terminal_loci
        )
    elif raw_mode:
        for start in coil_starts:
            start.update({
                "continuousConductor": False,
                "continuityFromPassIndex": None,
                "continuityStatus": "UNAVAILABLE_UNPROVED",
                "markerAuthority": (
                    f"{start['markerAuthority']}; legacy baseline has no "
                    "continuous-route or active-terminal-locus authority"
                ),
            })
    markers.extend([
        [start["positioningStart"], "coil_start", [
            start["phase"], start["passIndex"], start["tooth"],
            start["direction"],
        ], None]
        for start in coil_starts
    ])
    markers.sort(key=lambda marker: marker[0])
    expected_depositions = len(windings) * int(meta["turns"])
    if len(depositions) != expected_depositions:
        raise RuntimeError(
            f"captured {len(depositions)} complete turns; "
            f"expected {expected_depositions}"
        )
    capture_path_value = None
    capture_sha256 = None
    if capture_path is not None:
        capture_path_obj = Path(capture_path).resolve()
        if not capture_path_obj.is_file():
            raise RuntimeError("player capture path is not a file")
        capture_path_value = str(capture_path_obj)
        capture_sha256 = _file_sha256(capture_path_obj)
    command_counts = {
        str(axis): sum(command[1] == axis for command in commands)
        for axis in range(4)
    }
    radial_values = [
        deposition.get("rawActiveRadialMeanMm")
        for deposition in depositions
        if deposition.get("rawActiveRadialMeanMm") is not None
    ]
    capture_mode = "upstream_raw" if raw_mode else "contract_plan"
    return {
        "schema": PLAYER_SCHEMA,
        "captureMode": capture_mode,
        "capturePath": capture_path_value,
        "captureSha256": capture_sha256,
        "captureAuthority": (
            "unmodified upstream serial command stream reconstructed with "
            "the upstream constant-velocity Timeline" if raw_mode else
            "plan-bound ContractWind diagnostic capture"
        ),
        "virtualDuration": round(virtual_duration, 4),
        "exportSpeed": export_speed,
        "defaultRate": default_rate,
        "mmPerRadM0": manifest["mm_per_rad_m0"],
        "standoff": manifest["m0_home_standoff"],
        "flyerFeed": active_terminal_guide[
            "unproved_transition_origin_local_mm"
        ],
        "flyerGuideCenter": active_terminal_guide[
            "review_focus_center_local_mm"
        ],
        "wireRenderRadius": wire_render_radius,
        "toothContact": wire["tooth_contact"],
        "shaftContact": wire["shaft_contact"],
        "wireHandoff": wire_handoff,
        "dynamicWireSegmentCapacity": max(
            MAX_DYNAMIC_WIRE_SEGMENTS,
            ((_active_terminal_max_edges(active_terminal_loci)
              + MAX_UNPROVEN_LIVE_CONNECTOR_SEGMENTS)
             if active_terminal_loci is not None else 0),
        ),
        "stator": stator,
        "slotPlan": slot_plan,
        "turnsPerTooth": int(meta["turns"]),
        "teethCount": int(meta["teeth_count"]),
        "phaseCount": sum(1 for event in events if event["e"] == "wind"),
        "captureCommit": meta.get("winder_commit", "unknown"),
        "commands": commands,
        "commandCountsByAxis": command_counts,
        "commandCount": len(commands),
        "markers": markers,
        "axisKnots": [_round_knots(timeline.axes[i]) for i in range(4)],
        "windings": windings,
        "wraps": wraps,
        "halfTurns": half_turns,
        "depositions": depositions,
        "coilStarts": coil_starts,
        "conductorEvidence": conductor_evidence,
        "conductorRoute": continuous_route,
        "conductorRoutePath": continuous_route_path_value,
        "conductorRouteArtifactSha256": continuous_route_artifact_sha256,
        "conductorRouteSha256": (
            continuous_route["report_sha256"]
            if continuous_route is not None else None
        ),
        "activeTerminalLoci": active_terminal_loci,
        "activeTerminalLociPath": active_terminal_loci_path_value,
        "activeTerminalLociArtifactSha256": (
            active_terminal_loci_artifact_sha256
        ),
        "activeTerminalLociSha256": (
            active_terminal_loci["locus_payload_sha256"]
            if active_terminal_loci is not None else None
        ),
        "activeTerminalLociMaxEdges": (
            _active_terminal_max_edges(active_terminal_loci)
            if active_terminal_loci is not None else 0
        ),
        "depositionSummary": {
            "count": len(depositions),
            "expected": expected_depositions,
            "halfTurnCount": len(half_turns),
            "expectedHalfTurns": 2 * expected_depositions,
            "straightSamples": 12,
            "endTurnSamples": 16,
            "source": (
                "Timeline physical M2 crossings + reconstructed raw M0 radial "
                "progression" if raw_mode else
                "observed plan-waypoint crossings + exact constructive "
                "slot-leg centers"
            ),
            "slotLegAuthority": (
                "approximate elastic slot placement: raw M0 radial progression "
                "+ diagnostic plan tangential/layer offset; not raw controller "
                "authority" if raw_mode else
                "exact constructive plan centers"
            ),
            "endTurnAuthority": (
                (
                    "2,400 exact sampled active terminal loci during winding; "
                    "shared park/index/load/unload route remains red/dashed and "
                    "fail-closed"
                ) if integrated_raw_route else (
                    "legacy baseline has no active-terminal-locus or continuous-"
                    "route artifact; displayed turns are disconnected approximate "
                    "elastic review geometry"
                ) if raw_mode else
                "illustrative connector; not route validation"
            ),
            "rawRadialRangeMm": (
                [min(radial_values), max(radial_values)]
                if radial_values else None
            ),
            "turnClockAuthority": (
                "50 physical M2 revolutions/pass from directed Timeline pi "
                "crossings" if raw_mode else
                "explicit ContractWind packing waypoint crossings"
            ),
        },
        "limitations": (([
            "Raw M0/M1/M2/M3 command and reconstructed axis motion are authoritative.",
            "The 50 turn clocks per pass are physical M2 crossings, not constructive packing centers.",
            "Final coil tangential/layer offsets are an approximate elastic visualization borrowed from the diagnostic plan.",
            "The exact active terminal route is sampled only at 2,400 raw half-turn-start loci; it is held between crossings for review and is not a continuous flexible-wire solution.",
            "Wire sag, tension dynamics, elasticity, slot insertion, enamel contact, strand settling, and neatness are not simulated.",
            "Inferred shaft-wrap intervals use the exact raw M1 command and Timeline arrival inside the upstream high-level markers.",
            "One shared ordered conductor route keeps the live endpoint visible through indexing; park/index/load/unload transitions remain red/dashed and release-FAIL.",
        ] if integrated_raw_route else [
            "Raw M0/M1/M2/M3 command and reconstructed axis motion are authoritative.",
            "The 50 turn clocks per pass are physical M2 crossings, not constructive packing centers.",
            "Legacy baseline assets have no continuous-conductor route, active-terminal loci, or flexible live-wire solution.",
            "Displayed turns are disconnected approximate elastic review geometry; they do not establish continuity between turns or passes.",
            "Wire sag, tension dynamics, elasticity, slot insertion, enamel contact, strand settling, and neatness are not simulated.",
            "Inferred shaft-wrap intervals use the exact raw M1 command and Timeline arrival inside the upstream high-level markers.",
            "Use the selected integrated player and full-cycle conductor audit for governing conductor review.",
        ]) if raw_mode else [
            "ContractWind diagnostic packing centers are exact for its constructive plan.",
            "End-turn connectors and live feed span remain visualization-only.",
        ]),
    }


def _write_player(glb_path, html_path, data):
    template = (HERE / "player_template.html").read_text(encoding="utf-8")
    glb_b64 = base64.b64encode(Path(glb_path).read_bytes()).decode("ascii")
    state_json = json.dumps(data, separators=(",", ":"), ensure_ascii=True)
    state_b64 = base64.b64encode(state_json.encode("utf-8")).decode("ascii")
    html = template.replace("__GLB_B64__", glb_b64)
    html = html.replace("__STATE_B64__", state_b64)
    if "__GLB_B64__" in html or "__STATE_B64__" in html:
        raise RuntimeError("player template replacement failed")
    html_path = Path(html_path)
    html_path.parent.mkdir(parents=True, exist_ok=True)
    html_path.write_text(html, encoding="utf-8")
    return html_path


def _visual_group_asset_records(manifest):
    """Validate separately materialled CAD render groups fail-closed."""
    groups = manifest.get("visual_groups")
    if not isinstance(groups, Mapping):
        raise RuntimeError("CAD manifest has no visual_groups contract")
    result = []
    for name, expected in REQUIRED_VISUAL_GROUPS.items():
        record = groups.get(name)
        if not isinstance(record, Mapping):
            raise RuntimeError(f"CAD visual group {name!r} is missing")
        if record.get("link") != expected["link"]:
            raise RuntimeError(f"CAD visual group {name!r} has the wrong link")
        if set(record.get("labels", [])) != expected["labels"]:
            raise RuntimeError(f"CAD visual group {name!r} labels drifted")
        if record.get("excluded_from_base_link_mesh") is not True:
            raise RuntimeError(
                f"CAD visual group {name!r} is not excluded from its base mesh"
            )
        file_name = record.get("file")
        if (not isinstance(file_name, str)
                or Path(file_name).name != file_name
                or not file_name.lower().endswith(".stl")):
            raise RuntimeError(f"CAD visual group {name!r} file is unsafe")
        result.append({"name": name, "link": expected["link"],
                       "path": OUT / "links" / file_name})
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--capture", default=str(DEFAULT_CAPTURE),
        help=("captured JSONL command stream; canonical raw upstream and "
              "plan-bound ContractWind captures are both supported"),
    )
    ap.add_argument(
        "--conductor-route", type=Path,
        default=CONTINUOUS_CONDUCTOR_ROUTE,
        help=("continuous-conductor JSON artifact for the schema-tagged "
              "integrated raw player; the legacy baseline intentionally "
              "does not consume or imply this contract"),
    )
    ap.add_argument(
        "--active-terminal-loci", type=Path,
        help=("exact 2,400-locus active-sector terminal-route JSON for a raw "
              "player; defaults at render time to <asset-root>/reports/"
              f"{ACTIVE_TERMINAL_LOCI_NAME}"),
    )
    ap.add_argument("--speed", type=float, default=10.0,
                    help="GLB time compression and default virtual playback rate")
    ap.add_argument("-o", "--output", default=None)
    ap.add_argument("--html", default=None,
                    help="self-contained watchable player output")
    ap.add_argument("--no-html", action="store_true",
                    help="write only the GLB")
    args = ap.parse_args()
    if args.speed <= 0:
        ap.error("--speed must be greater than zero")

    capture_path = Path(args.capture).resolve()
    if not capture_path.is_file():
        ap.error(f"--capture does not exist: {capture_path}")
    canonical_raw = capture_path == DEFAULT_CAPTURE.resolve()
    output = (
        Path(args.output) if args.output else
        LEGACY_RAW_GLB if canonical_raw else OUT / "winding_cycle.glb"
    )
    html_output = (
        Path(args.html) if args.html else
        LEGACY_RAW_HTML if canonical_raw else OUT / "play_animation.html"
    )
    if not canonical_raw and args.output is None and args.html is None:
        raise RuntimeError(
            "the default published player is reserved for the canonical raw "
            "capture; provide explicit --output and --html for diagnostics"
        )

    manifest = json.loads((OUT / "links" / "manifest.json").read_text())
    mm_per_rad = manifest["mm_per_rad_m0"]
    standoff = manifest["m0_home_standoff"]

    events = load_events(capture_path)
    tl = Timeline(events)

    # ---- adaptive keyframes ------------------------------------------------
    keys = []                                # (t, m0, m1, m2)
    last = None
    for t, m0, m1, m2 in tl.samples(max_dm2=math.radians(20),
                                    max_dm0=0.4,
                                    max_dm1=math.radians(3)):
        if last is None or (abs(m0 - last[1]) * mm_per_rad >= 0.5
                            or abs(m1 - last[2]) >= math.radians(3)
                            or abs(m2 - last[3]) >= math.radians(20)
                            or t - last[0] > 2.0):
            keys.append((t, m0, m1, m2))
            last = (t, m0, m1, m2)
    print(f"{len(keys)} keyframes over {tl.t_end/60:.1f} virtual min "
          f"(GLB duration {tl.t_end/args.speed/60:.1f} min at {args.speed}x)")

    times = np.array([k[0] / args.speed for k in keys], np.float32)
    tr_car = np.array([[0, 0, k[1] * mm_per_rad] for k in keys], np.float32)
    rq_spin = np.array([quat_y(k[2]) for k in keys], np.float32)
    rq_fly = np.array([quat_z(k[3]) for k in keys], np.float32)

    # ---- geometry ----------------------------------------------------------
    blobs = []                               # (bytes, target)
    accessors = []
    buffer_views = []
    meshes_gl = []
    materials = []
    nodes = []

    def add_blob(data: bytes, target=None):
        off = sum(len(blob) for blob, _ in blobs)
        pad = (4 - off % 4) % 4
        if pad:
            blobs[-1] = (blobs[-1][0] + b"\x00" * pad, blobs[-1][1])
            off += pad
        blobs.append((data, target))
        buffer_views.append(gl.BufferView(
            buffer=0, byteOffset=off, byteLength=len(data), target=target))
        return len(buffer_views) - 1

    def add_accessor(arr, ctype, atype, target=None, minmax=False):
        bv = add_blob(arr.tobytes(), target)
        acc = gl.Accessor(bufferView=bv, componentType=ctype,
                          count=len(arr), type=atype)
        if minmax:
            acc.max = arr.max(axis=0).tolist()
            acc.min = arr.min(axis=0).tolist()
        accessors.append(acc)
        return len(accessors) - 1

    visual_group_assets = _visual_group_asset_records(manifest)
    assets = [
        ("static", OUT / "links" / manifest["links"]["static"]["file"]),
        ("carriage", OUT / "links" / manifest["links"]["carriage"]["file"]),
        ("spindle", OUT / "links" / manifest["links"]["spindle"]["file"]),
        ("flyer", OUT / "links" / manifest["links"]["flyer"]["file"]),
        ("wire_static", OUT / "links" / "wire_static.stl"),
        ("wire_flyer", OUT / "links" / "wire_flyer.stl"),
    ]
    assets.extend((record["name"], record["path"])
                  for record in visual_group_assets)
    mesh_indices = {}
    for name, path in assets:
        mesh = trimesh.load(path, force="mesh")
        vertices = mesh.vertices.astype(np.float32)
        normals = mesh.vertex_normals.astype(np.float32)
        faces = mesh.faces.astype(np.uint32).ravel()
        pos_acc = add_accessor(vertices, gl.FLOAT, gl.VEC3,
                               gl.ARRAY_BUFFER, minmax=True)
        normal_acc = add_accessor(normals, gl.FLOAT, gl.VEC3,
                                  gl.ARRAY_BUFFER)
        idx_acc = add_accessor(faces, gl.UNSIGNED_INT, gl.SCALAR,
                               gl.ELEMENT_ARRAY_BUFFER)
        if name.startswith("wire_"):
            material_properties = {
                "metallic": 0.12, "roughness": 0.48,
                "double_sided": True,
            }
        else:
            material_properties = MATERIAL_PROPERTIES.get(name, {
                "metallic": 0.05, "roughness": 0.75,
                "double_sided": False,
            })
        materials.append(gl.Material(
            pbrMetallicRoughness=gl.PbrMetallicRoughness(
                baseColorFactor=COLORS[name],
                metallicFactor=material_properties["metallic"],
                roughnessFactor=material_properties["roughness"]),
            doubleSided=material_properties["double_sided"],
            name=name))
        meshes_gl.append(gl.Mesh(primitives=[gl.Primitive(
            attributes=gl.Attributes(POSITION=pos_acc, NORMAL=normal_acc),
            indices=idx_acc,
            material=len(materials) - 1)], name=name))
        mesh_indices[name] = len(meshes_gl) - 1

    # nodes: 0 static, 1 carriage, 2 spindle_pivot, 3 spindle, 4 flyer,
    #        5 static wire, 6 flyer-frame wire, then material groups.
    nodes.append(gl.Node(mesh=0, name="static"))
    nodes.append(gl.Node(name="carriage", mesh=1, children=[2]))
    nodes.append(gl.Node(name="spindle_pivot", translation=[0, 0, standoff],
                         children=[3]))
    nodes.append(gl.Node(name="spindle", mesh=2,
                         translation=[0, 0, -standoff]))
    nodes.append(gl.Node(name="flyer", mesh=3, children=[6]))
    nodes.append(gl.Node(mesh=4, name="wire_static"))
    nodes.append(gl.Node(mesh=5, name="wire_flyer"))
    scene_nodes = [0, 1, 4, 5]
    link_parent_nodes = {"carriage": 1, "spindle": 3, "flyer": 4}
    for record in visual_group_assets:
        node_index = len(nodes)
        nodes.append(gl.Node(
            mesh=mesh_indices[record["name"]], name=record["name"]
        ))
        if record["link"] == "static":
            scene_nodes.append(node_index)
        else:
            parent = nodes[link_parent_nodes[record["link"]]]
            parent.children = list(parent.children or []) + [node_index]

    # ---- animation ---------------------------------------------------------
    t_acc = add_accessor(times.reshape(-1, 1), gl.FLOAT, gl.SCALAR)
    accessors[t_acc].min = [float(times.min())]
    accessors[t_acc].max = [float(times.max())]
    car_acc = add_accessor(tr_car, gl.FLOAT, gl.VEC3)
    spin_acc = add_accessor(rq_spin, gl.FLOAT, gl.VEC4)
    fly_acc = add_accessor(rq_fly, gl.FLOAT, gl.VEC4)

    anim = gl.Animation(
        name="winding_cycle",
        samplers=[
            gl.AnimationSampler(input=t_acc, output=car_acc,
                                interpolation="LINEAR"),
            gl.AnimationSampler(input=t_acc, output=spin_acc,
                                interpolation="LINEAR"),
            gl.AnimationSampler(input=t_acc, output=fly_acc,
                                interpolation="LINEAR"),
        ],
        channels=[
            gl.AnimationChannel(sampler=0, target=gl.AnimationChannelTarget(
                node=1, path="translation")),
            gl.AnimationChannel(sampler=1, target=gl.AnimationChannelTarget(
                node=2, path="rotation")),
            gl.AnimationChannel(sampler=2, target=gl.AnimationChannelTarget(
                node=4, path="rotation")),
        ])

    blob = b"".join(blob for blob, _ in blobs)
    g = gl.GLTF2(
        scene=0,
        scenes=[gl.Scene(nodes=scene_nodes)],
        nodes=nodes,
        meshes=meshes_gl,
        materials=materials,
        accessors=accessors,
        bufferViews=buffer_views,
        buffers=[gl.Buffer(byteLength=len(blob))],
        animations=[anim],
    )
    g.set_binary_blob(blob)
    output.parent.mkdir(parents=True, exist_ok=True)
    g.save(output)
    print(f"wrote {output} ({output.stat().st_size/1e6:.1f} MB)")

    if not args.no_html:
        data = _player_data(
            events, tl, manifest, args.speed, args.speed,
            capture_path=capture_path,
            conductor_route_path=Path(args.conductor_route).resolve(),
            active_terminal_loci_path=(
                Path(args.active_terminal_loci).resolve()
                if args.active_terminal_loci is not None else None
            ),
        )
        data["glbSha256"] = _file_sha256(output)
        html_path = _write_player(output, html_output, data)
        turn_counts = [len(winding["turnTimes"])
                       for winding in data["windings"]]
        print(f"wrote {html_path} ({html_path.stat().st_size/1e6:.1f} MB); "
              f"{len(data['windings'])} tooth passes, turn crossings "
              f"{min(turn_counts)}..{max(turn_counts)}")
        print(
            f"capture sha256 {data['captureSha256']}; "
            f"GLB sha256 {data['glbSha256']}; "
            f"HTML sha256 {_file_sha256(html_path)}"
        )
        if data["activeTerminalLociSha256"] is not None:
            print(
                "active terminal loci "
                f"{data['activeTerminalLociSha256']}; "
                f"{len(data['activeTerminalLoci']['loci'])} winding loci"
            )


if __name__ == "__main__":
    main()
