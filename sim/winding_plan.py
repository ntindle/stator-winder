"""Strict consumer for the constructive slot winding plan.

The geometry audit owns ``out/reports/slot_winding_plan.json``.  This module
does not invent a traverse from turn count or available span: it validates the
explicit fifty-placement construction and converts its 100 alternating slot-
side crossings into the M0/M2 radians used by the upstream serial controller.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


SCHEMA = "slot-winding-plan/v1"


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field} must be an integer")
    return value


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be an object")
    return value


def _sequence(value: Any, field: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{field} must be an array")
    return value


@dataclass(frozen=True)
class SlotPlacement:
    turn_index: int
    radial_mm: float
    tangential_mm: float
    active_tooth_radial_mm: float
    m0_target_rad: float
    layer: int
    row: int
    contact_id: str


@dataclass(frozen=True)
class HalfTurnCenter:
    half_turn_index: int
    phase_turns: float
    placement_index: int
    radial_mm: float
    m0_target_rad: float


@dataclass(frozen=True)
class SlotWindingPlan:
    path: Path
    sha256: str
    raw: Mapping[str, Any]
    slots: int
    od_mm: float
    stack_mm: float
    wire_finished_d_mm: float
    model_wire_envelope_mm: float
    receiving_sensitivity_wire_envelope_mm: float
    receiving_sensitivity_status: str
    turns_per_tooth: int
    liner_max_thickness_mm: float
    placements: tuple[SlotPlacement, ...]
    half_turn_centers: tuple[HalfTurnCenter, ...]
    final_hold_placement_index: int
    hold_through_lead_out: bool
    transition_status: str

    @property
    def controller_ready(self) -> bool:
        return self.transition_status == "PASS"

    def validate_settings(self, config: Mapping[str, Any]) -> None:
        """Fail if the plan does not describe the loaded physical job."""
        job = _mapping(config.get("job"), "settings.job")
        winding = _mapping(config.get("winding"), "settings.winding")
        checks = (
            ("slots", self.slots, _integer(
                job.get("slots", len(str(winding.get("winding_config", "")))),
                "settings.job.slots")),
            ("od_mm", self.od_mm,
             _number(job.get("od_mm"), "settings.job.od_mm")),
            ("stack_mm", self.stack_mm,
             _number(job.get("stack_mm"), "settings.job.stack_mm")),
            ("wire_finished_d_mm", self.wire_finished_d_mm, _number(
                job.get("wire_finished_d_mm"),
                "settings.job.wire_finished_d_mm")),
            ("turns_per_tooth", self.turns_per_tooth, _integer(
                winding.get("turns"), "settings.winding.turns")),
            ("liner_max_thickness_mm", self.liner_max_thickness_mm,
             _number(job.get("liner_max_thickness_mm"),
                     "settings.job.liner_max_thickness_mm")),
        )
        for name, expected, actual in checks:
            if isinstance(expected, int):
                ok = expected == actual
            else:
                ok = math.isclose(
                    float(expected), float(actual), rel_tol=0.0,
                    abs_tol=1e-9)
            if not ok:
                raise ValueError(
                    f"winding plan {name} mismatch: plan={expected}, "
                    f"settings={actual}"
                )

        span = _sequence(
            job.get("radial_winding_span_mm"),
            "settings.job.radial_winding_span_mm")
        if len(span) != 2:
            raise ValueError(
                "settings.job.radial_winding_span_mm must have two values")
        low, high = map(float, span)
        if not low < high:
            raise ValueError("settings radial winding span must increase")
        for placement in self.placements:
            if not (low - 1e-9 <= placement.active_tooth_radial_mm
                    <= high + 1e-9):
                raise ValueError(
                    f"winding plan placement {placement.turn_index} active "
                    f"radial {placement.active_tooth_radial_mm:.6f} mm lies "
                    "outside settings "
                    f"span [{low:.6f}, {high:.6f}] mm"
                )

    def controller_waypoints(
        self,
        radial_span_mm: Sequence[float],
        m0_wind_range_rad: Sequence[float],
        actual_m2_travel_rad: float,
        first_crossing_phase_rad: float = 0.0,
    ) -> list[dict[str, float]]:
        """Map slot-local centers to complete M2-phase/M0-radian points.

        The plan supplies both slot-side crossings for every physical turn.
        A boundary point at exactly ``turns_per_tooth`` is held at the final
        placement, followed by an optional upstream half-turn lead-out hold.
        """
        if len(radial_span_mm) != 2 or len(m0_wind_range_rad) != 2:
            raise ValueError("radial span and M0 range must each have two values")
        radial_start, radial_end = map(float, radial_span_mm)
        m0_start, m0_end = map(float, m0_wind_range_rad)
        if not radial_start < radial_end or not m0_start < m0_end:
            raise ValueError("radial span and M0 range must increase")
        nominal_travel = self.turns_per_tooth * 2.0 * math.pi
        maximum_travel = nominal_travel + math.pi
        actual = _number(actual_m2_travel_rad, "actual_m2_travel_rad")
        if not (nominal_travel - 1e-6 <= actual
                <= maximum_travel + 1e-6):
            raise ValueError(
                f"actual M2 travel {actual:.9f} rad is outside the plan's "
                f"{nominal_travel:.9f}..{maximum_travel:.9f} rad "
                "winding-plus-lead-out contract"
            )
        if actual > nominal_travel + 1e-6 and not self.hold_through_lead_out:
            raise ValueError(
                "upstream selected a lead-out half-turn but the winding plan "
                "does not authorize a final-position hold"
            )

        def checked_explicit_m0(center: HalfTurnCenter) -> float:
            fraction = (
                center.radial_mm - radial_start
            ) / (radial_end - radial_start)
            mapped = m0_start + fraction * (m0_end - m0_start)
            if not math.isclose(
                    mapped, center.m0_target_rad,
                    rel_tol=0.0, abs_tol=1.0e-3):
                raise ValueError(
                    f"half-turn {center.half_turn_index} explicit M0 target "
                    f"{center.m0_target_rad:.9f} does not match the generated "
                    f"active-radial mapping {mapped:.9f}"
                )
            return center.m0_target_rad

        first_phase = _number(
            first_crossing_phase_rad, "first_crossing_phase_rad")
        if first_phase < -1e-9 or first_phase > math.pi + 1e-6:
            raise ValueError(
                "first slot crossing must be between logical phase 0 and pi")
        first_phase = max(0.0, first_phase)
        closure_phase = first_phase + nominal_travel
        if actual < closure_phase - 1e-6:
            raise ValueError(
                f"actual M2 target {actual:.9f} rad occurs before the "
                f"required post-deposition closure crossing "
                f"{closure_phase:.9f} rad"
            )
        points = [{
            "m2_phase_rad": (
                first_phase + center.phase_turns * 2.0 * math.pi),
            "m0_target_rad": checked_explicit_m0(center),
            "placement_index": center.placement_index,
            "kind": "placement_center",
        } for center in self.half_turn_centers]
        final_m0 = self.placements[
            self.final_hold_placement_index
        ].m0_target_rad
        if points[-1]["m2_phase_rad"] > actual + 1e-6:
            raise ValueError(
                "actual M2 target occurs before the final planned crossing")
        # Every remaining half-turn crossing is an explicit final-placement
        # hold.  Do not jump directly to the endpoint and hide a lead-out
        # crossing from capture.
        next_phase = points[-1]["m2_phase_rad"] + math.pi
        while next_phase <= actual + 1e-9:
            points.append({
                "m2_phase_rad": min(next_phase, actual),
                "m0_target_rad": final_m0,
                "placement_index": self.final_hold_placement_index,
                "kind": "final_hold",
            })
            next_phase += math.pi
        if points[-1]["m2_phase_rad"] < actual - 1e-9:
            points.append({
                "m2_phase_rad": actual,
                "m0_target_rad": final_m0,
                "placement_index": self.final_hold_placement_index,
                "kind": "final_hold",
            })
        return points


def load_slot_winding_plan(path: str | Path) -> SlotWindingPlan:
    path = Path(path).resolve()
    payload = path.read_bytes()
    try:
        raw = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid winding plan JSON: {path}: {exc}") from exc
    root = _mapping(raw, "plan")
    if root.get("schema") != SCHEMA:
        raise ValueError(
            f"winding plan schema must be {SCHEMA!r}, got "
            f"{root.get('schema')!r}")
    job = _mapping(root.get("job"), "plan.job")
    slots = _integer(job.get("slots"), "plan.job.slots")
    turns = _integer(
        job.get("turns_per_tooth"), "plan.job.turns_per_tooth")
    if slots < 3 or turns <= 0:
        raise ValueError("plan slots must be >=3 and turns_per_tooth positive")
    wire_d = _number(
        job.get("wire_finished_d_mm"), "plan.job.wire_finished_d_mm")
    model_wire = _number(
        job.get("model_wire_envelope_mm", wire_d),
        "plan.job.model_wire_envelope_mm")
    receiving_case = _mapping(
        root.get("receiving_wire_envelope_case", {}),
        "plan.receiving_wire_envelope_case")
    receiving_wire = _number(
        receiving_case.get(
            "wire_finished_d_mm",
            job.get("receiving_sensitivity_wire_envelope_mm", model_wire)),
        "plan.receiving_wire_envelope_case.wire_finished_d_mm")
    receiving_status = receiving_case.get("status", "NOT_PROVEN")
    if receiving_status not in {"PASS", "FAIL", "NOT_PROVEN"}:
        raise ValueError(
            "receiving wire envelope status must be PASS, FAIL, or NOT_PROVEN")
    if model_wire + 1e-12 < wire_d:
        raise ValueError(
            "plan model_wire_envelope_mm cannot be smaller than nominal wire")

    frame = _mapping(root.get("coordinate_frame"), "plan.coordinate_frame")
    if frame.get("name") != "slot_bisector_local":
        raise ValueError("plan coordinate frame must be slot_bisector_local")
    if not str(frame.get("radial_axis", "")).startswith("+x outward"):
        raise ValueError("plan radial axis must begin '+x outward'")

    raw_placements = _sequence(root.get("placements"), "plan.placements")
    if len(raw_placements) != turns:
        raise ValueError(
            f"plan must have {turns} placements, got {len(raw_placements)}")
    placements: list[SlotPlacement] = []
    for index, value in enumerate(raw_placements):
        item = _mapping(value, f"plan.placements[{index}]")
        turn_index = _integer(
            item.get("turn_index"), f"plan.placements[{index}].turn_index")
        if turn_index != index:
            raise ValueError("plan placement turn_index sequence is not exact")
        contact_id = item.get("contact_id")
        if not isinstance(contact_id, str) or not contact_id:
            raise ValueError(f"plan.placements[{index}].contact_id required")
        placements.append(SlotPlacement(
            turn_index=index,
            radial_mm=_number(
                item.get("radial_mm"), f"plan.placements[{index}].radial_mm"),
            tangential_mm=_number(
                item.get("tangential_mm"),
                f"plan.placements[{index}].tangential_mm"),
            active_tooth_radial_mm=_number(
                item.get("active_tooth_radial_mm"),
                f"plan.placements[{index}].active_tooth_radial_mm"),
            m0_target_rad=_number(
                item.get("m0_target_rad"),
                f"plan.placements[{index}].m0_target_rad"),
            layer=_integer(
                item.get("layer"), f"plan.placements[{index}].layer"),
            row=_integer(item.get("row"), f"plan.placements[{index}].row"),
            contact_id=contact_id,
        ))

    raw_centers = _sequence(
        root.get("half_turn_centers"), "plan.half_turn_centers")
    expected_centers = 2 * turns
    if len(raw_centers) != expected_centers:
        raise ValueError(
            f"plan must have {expected_centers} half-turn centers, got "
            f"{len(raw_centers)}")
    centers: list[HalfTurnCenter] = []
    for index, value in enumerate(raw_centers):
        item = _mapping(value, f"plan.half_turn_centers[{index}]")
        half_index = _integer(
            item.get("half_turn_index"),
            f"plan.half_turn_centers[{index}].half_turn_index")
        phase = _number(
            item.get("phase_turns"),
            f"plan.half_turn_centers[{index}].phase_turns")
        placement_index = _integer(
            item.get("placement_index"),
            f"plan.half_turn_centers[{index}].placement_index")
        radial = _number(
            item.get("radial_mm"),
            f"plan.half_turn_centers[{index}].radial_mm")
        m0_target = _number(
            item.get("m0_target_rad"),
            f"plan.half_turn_centers[{index}].m0_target_rad")
        expected_placement = index // 2
        if half_index != index or not math.isclose(
                phase, index / 2.0, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("plan half-turn index/phase sequence is not exact")
        if placement_index != expected_placement:
            raise ValueError(
                "each adjacent half-turn pair must reference one placement")
        if not math.isclose(
                radial, placements[placement_index].active_tooth_radial_mm,
                rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(
                f"half-turn center {index} active radial does not match "
                "placement")
        if not math.isclose(
                m0_target, placements[placement_index].m0_target_rad,
                rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(
                f"half-turn center {index} M0 target does not match placement")
        centers.append(HalfTurnCenter(
            half_turn_index=index,
            phase_turns=phase,
            placement_index=placement_index,
            radial_mm=radial,
            m0_target_rad=m0_target,
        ))

    hold = _mapping(
        root.get("final_hold_policy"), "plan.final_hold_policy")
    hold_index = _integer(
        hold.get("placement_index"),
        "plan.final_hold_policy.placement_index")
    if hold_index != turns - 1:
        raise ValueError("final hold must use the last placement")
    if not isinstance(hold.get("hold_through_lead_out"), bool):
        raise ValueError("final hold lead-out flag must be bool")
    selected = _mapping(root.get("selected_case"), "plan.selected_case")
    if selected.get("status") != "PASS":
        raise ValueError("plan selected_case.status must be PASS")
    transition = _mapping(
        selected.get("transition_proof"),
        "plan.selected_case.transition_proof")
    status = transition.get("status")
    if status not in {"PASS", "FAIL", "NOT_PROVEN"}:
        raise ValueError(
            "plan transition_proof.status must be PASS, FAIL, or NOT_PROVEN")

    proof = _mapping(
        selected.get("final_slot_proof"),
        "plan.selected_case.final_slot_proof")
    if proof.get("status") != "PASS":
        raise ValueError("plan selected final_slot_proof.status must be PASS")
    pairwise = _number(
        proof.get("minimum_pairwise_center_distance_mm"),
        "plan.final_slot_proof.minimum_pairwise_center_distance_mm")
    if pairwise + 1e-9 < wire_d:
        raise ValueError(
            f"plan pairwise center distance {pairwise:.6f} mm is below "
            f"nominal wire diameter {wire_d:.6f} mm")

    return SlotWindingPlan(
        path=path,
        sha256=hashlib.sha256(payload).hexdigest(),
        raw=root,
        slots=slots,
        od_mm=_number(job.get("od_mm"), "plan.job.od_mm"),
        stack_mm=_number(job.get("stack_mm"), "plan.job.stack_mm"),
        wire_finished_d_mm=wire_d,
        model_wire_envelope_mm=model_wire,
        receiving_sensitivity_wire_envelope_mm=receiving_wire,
        receiving_sensitivity_status=receiving_status,
        turns_per_tooth=turns,
        liner_max_thickness_mm=_number(
            job.get("liner_max_thickness_mm"),
            "plan.job.liner_max_thickness_mm"),
        placements=tuple(placements),
        half_turn_centers=tuple(centers),
        final_hold_placement_index=hold_index,
        hold_through_lead_out=hold["hold_through_lead_out"],
        transition_status=status,
    )
