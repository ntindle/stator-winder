"""Export the two controlled COTS solids and their installed stack review."""

from pathlib import Path
import sys

from build123d import Compound, Pos, Rot, export_step


CAD = Path(__file__).resolve().parents[2]
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import hardware  # noqa: E402
from params import PARAMS as P  # noqa: E402


HERE = Path(__file__).resolve().parent


def installed_parts():
    """Return one stack with rail mounting face at local Z=0."""
    standoff = Rot(180, 0, 0) * hardware.foot_standoff_m5_ff_18(
        "foot_standoff"
    )
    foot = Pos(0, 0, -P.foot_standoff_h) * hardware.machine_foot_m5_17(
        "rubber_foot"
    )
    screw = Pos(0, 0, P.foot_set_screw_projection) * hardware.set_screw(
        "M5", 12.0, label="bonded_set_screw"
    )
    return [standoff, foot, screw]


def gen_step():
    result = Compound(children=installed_parts())
    result.label = "elesa_wurth_35mm_foot_stack"
    return result


def main() -> int:
    exports = {
        "elesa_432001.step": hardware.machine_foot_m5_17(),
        "wurth_970180581.step": hardware.foot_standoff_m5_ff_18(),
        "foot_stack_review.step": gen_step(),
    }
    for name, part in exports.items():
        path = HERE / name
        export_step(part, str(path))
        bbox = part.bounding_box()
        print(
            f"{path}: solids={len(part.solids())}, "
            f"bbox=({bbox.size.X:.3f}, {bbox.size.Y:.3f}, {bbox.size.Z:.3f})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
