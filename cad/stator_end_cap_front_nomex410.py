"""DXF-skill entry point for the front Nomex 410 end-face star cap."""

from stator_insulation_nomex410 import end_cap_dxf


def gen_dxf():
    return end_cap_dxf("front")
