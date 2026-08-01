"""Focused cap/block/arm clearance context for visual review only."""

from build123d import Compound

import permanent_cap_offset_spoke_review as design


def gen_step() -> Compound:
    static = design.shifted_static_module_parts()
    result = Compound(children=[
        design.cap_collision_support_envelope(1),
        design.cap_collision_support_envelope(-1),
        static["block"],
        design.extended_hollow_shaft(),
        design.offset_spoke_arm(),
        design.tip_toroid(),
        *design.provisional_balance_slug_envelopes(),
    ])
    result.label = "cap_envelope_offset_spoke_clearance_context"
    return result
