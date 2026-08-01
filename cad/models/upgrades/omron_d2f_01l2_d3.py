"""Export the controlled-drawing Omron D2F-01L2-D3 reference solid.

The source geometry lives in :func:`cad.cots.endstop` so the assembly and
standalone release artifact cannot drift.  Run from any working directory.
"""

from pathlib import Path
import sys

from build123d import export_step


CAD = Path(__file__).resolve().parents[2]
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import cots  # noqa: E402


OUT = Path(__file__).with_suffix(".step")


def gen_step():
    return cots.endstop(label="omron_d2f_01l2_d3")


def main() -> int:
    part = gen_step()
    export_step(part, str(OUT))
    bbox = part.bounding_box()
    print(
        f"{OUT}: solids={len(part.solids())}, "
        f"bbox=({bbox.size.X:.3f}, {bbox.size.Y:.3f}, {bbox.size.Z:.3f}) mm"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
