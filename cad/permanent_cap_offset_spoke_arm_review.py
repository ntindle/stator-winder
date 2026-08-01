"""Focused visual handoff for the isolated offset-spoke rotating arm."""

from build123d import Compound

import permanent_cap_offset_spoke_review as design


def gen_step() -> Compound:
    result = Compound(children=[
        design.offset_spoke_arm(),
        design.tip_toroid(),
        *design.provisional_balance_slug_envelopes(),
        design.flyer_wire_transition_witness(),
    ])
    result.label = "offset_spoke_arm_focused_review"
    return result
