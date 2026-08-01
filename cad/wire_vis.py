"""Wire visualization solids generated from the validated centerlines.

Two polyline tubes at the exact default-job finished-wire diameter:
  wire_static — spool payoff -> felt pinch -> dancer pulley -> entry elbow
                -> up to the flyer bore rear mouth (fixed in the frame)
  wire_flyer  — bore rear -> tangent elbow -> torus feed (fixed in the
                FLYER frame; the fairlead/work span is a live overlay)

The centerline is authoritative and shared with ``sim/wirepath.py`` through
the exported manifest.  The former Ø1.2 presentation tube falsely penetrated
both felt pads by 4.387 mm3 each; visibility is now handled by player color,
camera and highlighting rather than dishonest geometric thickness.
"""

from build123d import (Part, AngularDirection, BuildSketch, Circle, Edge,
                       Plane, Transition, Wire, sweep)

import wire_geometry
from params import DEFAULT_STATOR

R_VIS = DEFAULT_STATOR.wire_d / 2.0


def _fillet_edge(meta):
    center, start = meta["center"], meta["start"]
    radial = tuple(start[i] - center[i] for i in range(3))
    plane = Plane(origin=center, x_dir=radial, z_dir=meta["axis"])
    return Edge.make_circle(meta["radius"], plane, 0.0, meta["turn_deg"],
                            AngularDirection.COUNTER_CLOCKWISE)


def _swept_tube(edges, start, initial_direction, label):
    paths = Wire.combine(edges)
    if len(paths) != 1:
        raise ValueError(f"{label} centerline has {len(paths)} disconnected wires")
    with BuildSketch(Plane(origin=start, z_dir=initial_direction)) as profile:
        Circle(R_VIS)
    p = sweep(profile.sketch, paths[0], transition=Transition.TRANSFORMED)
    p.label = label
    return p


def wire_static() -> Part:
    spec = wire_geometry.static_path_spec()
    lm, dancer, entry = spec["landmarks"], spec["dancer"], spec["entry_bend"]
    dancer_plane = Plane(origin=lm["dancer_center"], x_dir=(1, 0, 0),
                         z_dir=(0, 0, 1))
    edges = [
        Edge.make_line(lm["spool_payoff"], lm["dancer_tangent_in"]),
        Edge.make_circle(dancer["path_radius"], dancer_plane,
                         dancer["theta_in_deg"], dancer["theta_out_deg"],
                         AngularDirection.CLOCKWISE),
        Edge.make_line(lm["dancer_tangent_out"], entry["start"]),
        _fillet_edge(entry),
        Edge.make_line(entry["end"], lm["bore_rear"]),
    ]
    return _swept_tube(edges, lm["spool_payoff"], (0, 1, 0),
                       "wire_static")


def wire_flyer() -> Part:
    spec = wire_geometry.flyer_path_spec()
    lm, elbow = spec["landmarks"], spec["elbow_bend"]
    edges = [
        Edge.make_line(lm["bore_rear"], elbow["start"]),
        _fillet_edge(elbow),
        Edge.make_line(elbow["end"], lm["guide_feed"]),
    ]
    return _swept_tube(edges, lm["bore_rear"], (0, 0, 1), "wire_flyer")
