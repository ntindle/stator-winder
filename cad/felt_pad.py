"""1:1 local-cut DXF for the wool felt wire-drag pad."""

import ezdxf
from ezdxf import units


OUTER_DIAMETER_MM = 20.0
BORE_DIAMETER_MM = 4.5
NOMINAL_FINISHED_THICKNESS_MM = 3.0
STOCK_THICKNESS_MM = 3.175
STOCK_SKU = "McMaster 8341K31"


def gen_dxf():
    document = ezdxf.new("R2013")
    document.units = units.MM
    document.layers.add("CUT", color=1)
    modelspace = document.modelspace()
    modelspace.add_circle(
        (0.0, 0.0), OUTER_DIAMETER_MM / 2.0,
        dxfattribs={"layer": "CUT"},
    )
    modelspace.add_circle(
        (0.0, 0.0), BORE_DIAMETER_MM / 2.0,
        dxfattribs={"layer": "CUT"},
    )
    return document
