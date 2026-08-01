"""Assembly at a representative mid-winding pose (for review snapshots).

Pose via env vars POSE_M0/POSE_M1/POSE_M2 (model-space radians); defaults
to deep insertion on tooth 2 with the flyer at 4 o'clock.
"""

import os
import assembly


def gen_step():
    m0 = float(os.environ.get("POSE_M0", "-65.581"))
    m1 = float(os.environ.get("POSE_M1", "-0.5236"))
    m2 = float(os.environ.get("POSE_M2", "-2.0"))
    return assembly.machine(m0=m0, m1=m1, m2=m2)
