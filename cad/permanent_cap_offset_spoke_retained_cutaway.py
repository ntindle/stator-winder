"""Focused half-section proof for the retained flyer counterweight stacks.

This secondary review artifact deliberately cuts through the rear-left and
front-left pocket axes.  It exists so a reviewer can see the screw, 1 mm
floor, annular slug, three-point spacer/cap, full insert, and blind printed
boss without the rest of the machine obscuring them.
"""

from __future__ import annotations

from build123d import Compound

import permanent_cap_offset_spoke_retained_review as retained


def _half_stack_detail(
    pocket: retained.Pocket,
    length_mm: float,
) -> Compound:
    margin = 2.0
    cutter = retained._box(
        pocket.x_mm - pocket.housing_r_mm - margin,
        pocket.x_mm + pocket.housing_r_mm + margin,
        pocket.y_mm,
        pocket.y_mm + pocket.housing_r_mm + margin,
        pocket.rear_z_mm - 0.5,
        pocket.front_z_mm + 0.5,
    )
    arm_half = retained.retained_arm() & cutter
    arm_half.label = f"{pocket.id}_continuous_arm_housing_floor_half_section"
    children = [arm_half]
    for name, shape, _material in retained.stack_parts(pocket, length_mm):
        half = shape & cutter
        half.label = f"{name}_half_section"
        children.append(half)
    result = Compound(children=children)
    result.label = (
        f"{pocket.id}_closed_load_path_cutaway__"
        "floor_slug_three_spacers_face_boss_insert_screw_blind_cap"
    )
    return result


def gen_step() -> Compound:
    lengths = retained.solve_slug_lengths()
    rear = _half_stack_detail(retained.POCKETS[0], lengths[0])
    front = _half_stack_detail(retained.POCKETS[2], lengths[2])
    result = Compound(children=[rear, front])
    result.label = "retained_flyer_counterweight_supported_screw_cutaway"
    return result

