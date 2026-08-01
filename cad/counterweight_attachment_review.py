"""Focused STEP/manifest generator for the flyer balance attachments.

The review consumes the exact serialized occurrences from the integrated
release candidate.  Four rear-facing M3 stacks clamp annular tungsten slugs
between a positive 1 mm arm floor and a separate printed retainer whose boss
contains the heat-set insert and 1.8 mm blind cap.  Two front M2 stacks retain
the final trim slugs in blind pilots in the main printed spoke.

The review stays in the machine reference frame (M2=0).  Its focused section
cuts one current rear M3 stack through the fastener axis, so the screw, insert,
printed retainer boss/spacers, tungsten slug, and one-piece arm floor remain
visible as real sectioned solids rather than a schematic over open air.
"""

from __future__ import annotations

from copy import copy
import json
from pathlib import Path

from build123d import Align, Box, Compound, Part, Pos, export_step

import integrated_release_candidate as candidate


OUT = Path(__file__).resolve().parent.parent / "out"
REVIEW = OUT / "review"

REAR_M3_STACK_IDS = (
    "rear_left",
    "rear_right",
    "front_left",
    "front_right",
)
REAR_M3_STACK_SUFFIXES = (
    "tungsten_slug",
    "printed_retainer_with_three_spacers",
    "McMaster_94459A130_insert",
    "McMaster_92125A126_M3x6_screw",
)
REAR_M3_OCCURRENCE_LABELS = tuple(
    f"{stack_id}_{suffix}"
    for stack_id in REAR_M3_STACK_IDS
    for suffix in REAR_M3_STACK_SUFFIXES
)
FRONT_M2_OCCURRENCE_LABELS = (
    "front_trim_B777_-3.6",
    "front_trim_B777_+3.6",
    *(f"front_trim_hardware_{index}" for index in range(1, 7)),
)
COUNTERWEIGHT_OCCURRENCE_LABELS = (
    *REAR_M3_OCCURRENCE_LABELS,
    *FRONT_M2_OCCURRENCE_LABELS,
)
COLLAR_CONTEXT_LABELS = (
    "shaft_clamp_neg_y_radial_M3x8_set_screw_not_counterweight",
    "shaft_clamp_neg_y_radial_M3_insert_not_counterweight",
    "shaft_clamp_pos_x_radial_M3x8_set_screw_not_counterweight",
    "shaft_clamp_pos_x_radial_M3_insert_not_counterweight",
)

# The positive-X half of this axial stack gives an uncluttered section while
# preserving both sides of the real material boundary: the arm floor behind
# the slug and the printed retainer boss/blind cap ahead of it.
SECTION_STACK_ID = "rear_right"
SECTION_AXIS_X_MM = 9.0
SECTION_OCCURRENCE_LABELS = tuple(
    f"{SECTION_STACK_ID}_{suffix}" for suffix in REAR_M3_STACK_SUFFIXES
)
ARM_CONTEXT_LABEL = "retained_arm_one_piece_positive_material_context"
SHAFT_CONTEXT_LABEL = "released_L79_D10_hollow_shaft_context"


def _selected_occurrences(
    labels: tuple[str, ...],
    occurrences: dict[str, Part] | None = None,
) -> list[Part]:
    """Copy and relabel current candidate occurrences for this review."""

    current = (
        candidate.retained_rotating_parts()
        if occurrences is None
        else occurrences
    )
    missing = [label for label in labels if label not in current]
    if missing:
        raise KeyError(
            "counterweight review occurrence contract drift: "
            + ", ".join(missing)
        )
    selected: list[Part] = []
    for label in labels:
        shape = copy(current[label])
        shape.label = label
        selected.append(shape)
    return selected


def _context_part(occurrences: dict[str, Part], key: str, label: str) -> Part:
    shape = copy(occurrences[key])
    shape.label = label
    return shape


def gen_step() -> Compound:
    """Return all six current balance stacks with real printed context."""

    occurrences = candidate.retained_rotating_parts()
    arm = _context_part(occurrences, "retained_arm", ARM_CONTEXT_LABEL)
    shaft = _context_part(occurrences, "shaft", SHAFT_CONTEXT_LABEL)
    assembly = Compound(children=[
        arm,
        shaft,
        *_selected_occurrences(
            COUNTERWEIGHT_OCCURRENCE_LABELS, occurrences
        ),
        *_selected_occurrences(COLLAR_CONTEXT_LABELS, occurrences),
    ])
    assembly.label = "counterweight_attachment_review"
    return assembly


def gen_section_step() -> Compound:
    """Section one real rear M3 stack through its axial fastener centerline."""

    occurrences = candidate.retained_rotating_parts()
    arm = _context_part(occurrences, "retained_arm", ARM_CONTEXT_LABEL)
    source_parts = [
        arm,
        *_selected_occurrences(SECTION_OCCURRENCE_LABELS, occurrences),
    ]
    keeper = Pos(SECTION_AXIS_X_MM, -100.0, -100.0) * Box(
        100.0,
        200.0,
        200.0,
        align=(Align.MIN, Align.MIN, Align.MIN),
    )
    sectioned: list[Part] = []
    for part in source_parts:
        cut = part & keeper
        if float(cut.volume) <= 0.0:
            raise RuntimeError(
                f"section removed current load-path part {part.label}"
            )
        cut.label = f"{part.label}_positive_x_axis_half_section"
        sectioned.append(cut)
    assembly = Compound(children=sectioned)
    assembly.label = (
        "rear_right_M3_counterweight_closed_load_path_axis_half_section"
    )
    return assembly


def attachment_audit() -> dict:
    """Return the integrated six-stack positive-material geometry audit."""

    return candidate.integrated_six_stack_attachment_audit()


def write_manifest() -> Path:
    """Write current six-stack attachment and exact balance evidence."""

    audit = attachment_audit()
    balance = candidate.integrated_balance_solution()
    selected_row = next(
        row
        for row in audit["rear_M3_retained_stacks"]["stacks"]
        if row["id"] == SECTION_STACK_ID
    )
    manifest = {
        "schema": "counterweight-attachment-review/v2",
        "status": audit["status"],
        "step": "out/review/counterweight_attachment.step",
        "section_step": "out/review/counterweight_attachment_section.step",
        "source": "cad/counterweight_attachment_review.py",
        "coordinate_frame": "machine reference pose, M2=0, millimetres",
        "serialized_occurrences": {
            "rear_M3_stacks": list(REAR_M3_OCCURRENCE_LABELS),
            "front_M2_stacks": list(FRONT_M2_OCCURRENCE_LABELS),
            "shaft_clamp_context_not_counterweights": list(
                COLLAR_CONTEXT_LABELS
            ),
        },
        "section": {
            "stack_id": SECTION_STACK_ID,
            "axis_x_mm": SECTION_AXIS_X_MM,
            "occurrences": list(SECTION_OCCURRENCE_LABELS),
            "positive_material_context": ARM_CONTEXT_LABEL,
            "closed_structural_load_path": selected_row[
                "closed_structural_load_path"
            ],
            "blind_positive_material_ahead_of_tip_mm": selected_row[
                "blind_positive_material_ahead_of_tip_mm"
            ],
        },
        "attachment_geometry": audit,
        "balance": {
            "authority": balance["authority"],
            "rear_slug_lengths_mm": balance["rear_slug_lengths_mm"],
            "front_trim_common_thickness_mm": balance[
                "front_trim_common_thickness_mm"
            ],
            "scaled_balance_residual_norm": balance[
                "scaled_balance_residual_norm"
            ],
            "mass_g": balance["mass_properties"]["mass_g"],
            "izz_about_M2_axis_kg_m2": balance["mass_properties"][
                "izz_about_M2_axis_kg_m2"
            ],
        },
        "physical_pull_proof_complete": audit[
            "physical_pull_proof_complete"
        ],
    }
    REVIEW.mkdir(parents=True, exist_ok=True)
    target = REVIEW / "counterweight_attachment.json"
    target.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return target


def main() -> None:
    REVIEW.mkdir(parents=True, exist_ok=True)
    step_target = REVIEW / "counterweight_attachment.step"
    section_target = REVIEW / "counterweight_attachment_section.step"
    export_step(gen_step(), step_target)
    export_step(gen_section_step(), section_target)
    print(step_target)
    print(section_target)
    print(write_manifest())


if __name__ == "__main__":
    main()
