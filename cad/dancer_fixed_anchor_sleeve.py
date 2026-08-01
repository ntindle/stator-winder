"""OD4 x ID2.2 x 1.5 fixed spring-loop stand-off sleeve."""

from shop_artifacts import SLEEVE_SPECS, sleeve_part


def gen_step():
    return sleeve_part(
        SLEEVE_SPECS["dancer_fixed_anchor_sleeve_od4_id2p2_l1p5"]
    )
