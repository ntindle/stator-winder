"""Deterministic wire-path verification for GOAL.md DoD #3.

The centerlines come from the CAD export manifest; visualization and
validation therefore cannot silently drift apart.  This checker covers:

* source geometry (felt straightness, dancer tangency/wrap, bend radii);
* connected exported wire meshes;
* every static free segment against every non-guide machine part,
  explicitly including the M2 motor, both pulleys, and belt;
* guide-channel fit for the entry and flyer elbow passages; and
* moving elbow-to-tip and tip-to-tooth segments over flyer angle and lay
  depth, with the lay point following the final wound-tooth perimeter; and
* both captured shaft wraps, using a tangent free span to the exposed shaft.

Clearances include the launch-envelope maximum wire radius (0.25 mm).  The
STEP/GLB wire is rendered at the selected job's true finished diameter; its
smaller presentation mesh is not substituted for the conservative clearance
radius used here.
"""

from __future__ import annotations

import argparse
import json
import hashlib
import math
from functools import lru_cache
from pathlib import Path

import numpy as np
import trimesh
from trimesh.collision import CollisionManager

from traj import Timeline, load_events

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out"
LINKS = OUT / "links"
MIN_BEND = 3.0
PROGRESS = OUT / "reports" / "wirepath.progress.json"
STATIC_WIRE_INTENTIONAL_CONTACTS = (
    "spool_drum", "felt_pad_fixed", "felt_pad_moving", "felt_guide_in",
    "dancer_pulley", "entry_bracket", "entry_eyelet",
)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _progress(stage, **detail):
    """Publish bounded, inspectable progress for the long dense sweep."""
    PROGRESS.parent.mkdir(parents=True, exist_ok=True)
    PROGRESS.write_text(json.dumps({"stage": stage, **detail}, indent=2))


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _circle_tangent_points_2d(point, center, radius):
    point = np.asarray(point, dtype=float)
    center = np.asarray(center, dtype=float)
    delta = point - center
    distance2 = float(np.dot(delta, delta))
    if distance2 <= radius * radius:
        raise ValueError("tangent source lies inside contact circle")
    base = center + delta * (radius * radius / distance2)
    perpendicular = np.array([-delta[1], delta[0]])
    offset = radius * math.sqrt(distance2 - radius * radius) / distance2
    return (base + perpendicular * offset,
            base - perpendicular * offset)


def tooth_support_tangents(tip, contact):
    """Return both tangencies to the wire-offset elliptical end former."""
    point = np.asarray(tip, dtype=float)[:2]
    a = float(contact["physical_tangential_radius_mm"])
    b = float(contact["physical_axial_radius_mm"])
    offset = float(contact["wire_offset_radius_mm"])

    def value(theta):
        c, s = math.cos(theta), math.sin(theta)
        body = np.array([a * c, b * s])
        normal = _unit(np.array([c / a, s / b]))
        return float(np.dot(normal, point - body) - offset)

    def contact_point(theta):
        c, s = math.cos(theta), math.sin(theta)
        body = np.array([a * c, b * s])
        normal = _unit(np.array([c / a, s / b]))
        return body + offset * normal

    grid = np.linspace(0.0, 2.0 * math.pi, 721)
    values = [value(theta) for theta in grid]
    roots = []
    for index in range(len(grid) - 1):
        lo, hi = float(grid[index]), float(grid[index + 1])
        flo, fhi = values[index], values[index + 1]
        if abs(flo) < 1e-10:
            roots.append(lo)
            continue
        if flo * fhi > 0.0:
            continue
        for _ in range(45):
            mid = (lo + hi) / 2.0
            fmid = value(mid)
            if flo * fmid <= 0.0:
                hi, fhi = mid, fmid
            else:
                lo, flo = mid, fmid
        roots.append((lo + hi) / 2.0)

    candidates = []
    for root in roots:
        tangent = contact_point(root)
        line = tangent - point
        # The centered convex former is entirely on the same side as its
        # origin.  Preserve the historical +/- support-side convention.
        center_cross = line[0] * (-point[1]) - line[1] * (-point[0])
        side = 1 if center_cross >= 0.0 else -1
        target = np.array([
            tangent[0], tangent[1], float(contact["z_mm"])
        ])
        if not any(old_side == side
                   and np.linalg.norm(old - target) < 1e-7
                   for old_side, old in candidates):
            candidates.append((side, target))
    by_side = {side: target for side, target in candidates}
    if set(by_side) != {-1, 1}:
        raise RuntimeError(
            f"expected two rectangle support tangents, got {candidates}"
        )
    return by_side


def tooth_contact_point(tip, contact, motion_sign):
    """Select the tension-trailing support tangent for flyer motion sign."""
    desired_support_side = -1 if motion_sign >= 0 else 1
    return tooth_support_tangents(tip, contact)[desired_support_side]


def shaft_tangent_point(tip, axis_z, contact, side):
    """Tangent contact from ``tip`` to the finite shaft in the X/Z plane."""
    point = np.array([float(tip[0]), float(tip[2])])
    center = np.array([0.0, float(axis_z)])
    delta = point - center
    distance2 = float(np.dot(delta, delta))
    radius = float(contact["radius_to_wire_center_mm"])
    if distance2 <= radius * radius:
        raise ValueError("flyer tip projection lies inside shaft contact radius")
    base = center + delta * (radius * radius / distance2)
    perpendicular = np.array([-delta[1], delta[0]])
    offset = (radius * math.sqrt(distance2 - radius * radius) / distance2)
    tangent = base + (1.0 if side >= 0 else -1.0) * perpendicular * offset
    return np.array([tangent[0], float(contact["axial_y_mm"]), tangent[1]])


def tip_guide_path(feed, target, guide, wire_radius, rotation=None,
                   arc_step_deg=2.0):
    """Tangent/torus-arc/tangent wire path over the flyer fairlead.

    The fixed feed lies on the torus axis.  Rotational symmetry lets the wire
    select the meridian containing the instantaneous target, so this remains a
    physical smooth path for both winding directions and shaft wraps.
    """
    rotation = np.eye(3) if rotation is None else np.asarray(rotation)
    center = rotation @ np.asarray(guide["center_local_mm"], dtype=float)
    axis = _unit(rotation @ np.asarray(guide["axis_local"], dtype=float))
    feed = np.asarray(feed, dtype=float)
    target = np.asarray(target, dtype=float)
    feed_rel = feed - center
    feed_axial = float(np.dot(feed_rel, axis))
    feed_transverse = feed_rel - feed_axial * axis
    if np.linalg.norm(feed_transverse) > 1e-6:
        raise ValueError("tip-guide feed is not on the torus axis")

    target_rel = target - center
    target_axial = float(np.dot(target_rel, axis))
    transverse = target_rel - target_axial * axis
    target_rho = float(np.linalg.norm(transverse))
    if target_rho < 1e-8:
        # Deterministic transverse basis for the measure-zero axial case.
        seed = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(seed, axis))) > 0.9:
            seed = np.array([0.0, 0.0, 1.0])
        meridian = _unit(seed - np.dot(seed, axis) * axis)
    else:
        meridian = transverse / target_rho

    major = float(guide["major_radius_mm"])
    path_radius = float(guide["tube_radius_mm"]) + float(wire_radius)
    circle = np.array([0.0, major])
    entry_candidates = _circle_tangent_points_2d(
        np.array([feed_axial, 0.0]), circle, path_radius,
    )
    entry_inner = [point for point in entry_candidates
                   if point[1] < major]
    if not entry_inner:
        raise RuntimeError("tip torus has no inner entry tangent")
    entry = max(entry_inner, key=lambda point: point[0])

    exit_candidates = _circle_tangent_points_2d(
        np.array([target_axial, target_rho]), circle, path_radius,
    )
    exit_outer = [point for point in exit_candidates
                  if point[1] > major]
    if not exit_outer:
        raise RuntimeError("tip torus has no outer exit tangent")
    # The outer candidate is normally unique.  If a limiting pose yields two,
    # choose the one whose front-side arc is shorter.
    theta_entry = math.atan2(entry[1] - major, entry[0])

    def front_delta(point):
        theta = math.atan2(point[1] - major, point[0])
        ccw = (theta - theta_entry) % (2.0 * math.pi)
        contains_front = ((-theta_entry) % (2.0 * math.pi)) <= ccw + 1e-12
        return ccw if contains_front else ccw - 2.0 * math.pi

    exit_point = min(exit_outer, key=lambda point: abs(front_delta(point)))
    delta = front_delta(exit_point)
    count = max(2, math.ceil(abs(math.degrees(delta)) / arc_step_deg))
    arc_2d = [
        circle + path_radius * np.array([
            math.cos(theta_entry + delta * index / count),
            math.sin(theta_entry + delta * index / count),
        ])
        for index in range(count + 1)
    ]

    def world(point):
        return center + point[0] * axis + point[1] * meridian

    points = [feed, world(entry)]
    points.extend(world(point) for point in arc_2d[1:])
    points.append(target)
    return np.asarray(points), {
        "arc_turn_deg": abs(math.degrees(delta)),
        "wire_center_bend_radius_mm": path_radius,
        "inside_wire_path_radius_mm": major - path_radius,
        "entry_tangent_error": _angle_deg(
            world(entry) - feed, world(arc_2d[1]) - world(arc_2d[0])
        ),
        "exit_tangent_error": _angle_deg(
            target - world(exit_point),
            world(arc_2d[-1]) - world(arc_2d[-2]),
        ),
    }


def _unit(v):
    v = np.asarray(v, dtype=float)
    length = np.linalg.norm(v)
    if length < 1e-10:
        raise ValueError("zero-length wire vector")
    return v / length


def _angle_deg(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(_unit(a), _unit(b)),
                                                 -1.0, 1.0))))


def _sample_polyline(points, spacing=0.25):
    points = np.asarray(points, dtype=float)
    sampled = []
    for index, (a, b) in enumerate(zip(points, points[1:])):
        count = max(2, math.ceil(float(np.linalg.norm(b - a)) / spacing) + 1)
        segment = np.linspace(a, b, count)
        if index:
            segment = segment[1:]
        sampled.append(segment)
    return np.vstack(sampled)


def _trim_sampled_polyline(points, start_mm=0.0, end_mm=0.0,
                           spacing=0.25):
    sampled = _sample_polyline(points, spacing=spacing)
    lengths = np.linalg.norm(np.diff(sampled, axis=0), axis=1)
    distance = np.concatenate(([0.0], np.cumsum(lengths)))
    keep = ((distance >= start_mm - 1e-9)
            & (distance <= distance[-1] - end_mm + 1e-9))
    result = sampled[keep]
    if len(result) < 2:
        raise ValueError("wire-path trimming removed the complete segment")
    return result


def _surface_clearance(query, points, wire_radius):
    """Exact point-to-triangle clearance after accounting for wire radius.

    ``signed_distance`` asks Trimesh to ray-classify every point as inside or
    outside.  That is unnecessary for a continuously sampled wire centreline
    and made the release sweep take hours.  ``on_surface`` returns the same
    exact closest triangle distance without the ray classifier.  Every free
    segment is sampled at no more than 0.25 mm while the minimum checked wire
    radius is 0.25 mm: a path which crosses a closed solid must therefore have
    a sample no farther than 0.125 mm from the crossed surface and necessarily
    fails this clearance test.  Intentional guide interiors are explicitly
    excluded, so no checked segment begins wholly inside a candidate solid.
    """
    distances = np.asarray(query.on_surface(points)[1], dtype=float)
    if not np.all(np.isfinite(distances)):
        raise RuntimeError("non-finite point-to-surface wire clearance")
    return float(distances.min() - wire_radius)


@lru_cache(maxsize=None)
def _load_mesh(path):
    return trimesh.load(path, force="mesh")


def _load_parts(manifest, link, exclude=(), include=None):
    excluded = set(exclude)
    included = None if include is None else set(include)
    return {
        label: _load_mesh(str(LINKS / "parts" / link / f"{label}.stl"))
        for label in manifest["parts"][link]
        if label not in excluded and (included is None or label in included)
    }


def _rank_part_clearances(parts, points, wire_radius):
    rows = []
    for label, mesh in parts.items():
        value = _surface_clearance(trimesh.proximity.ProximityQuery(mesh),
                                   points, wire_radius)
        rows.append((value, label))
    return sorted(rows)


def _case_points_in_link(case, link):
    world = case["points"]
    if link == "flyer":
        return world @ case["flyer_rotation"]
    if link == "spindle":
        return ((world - case["spindle_translation"])
                @ case.get("spindle_rotation", np.eye(3)))
    if link == "carriage":
        return world - case["carriage_translation"]
    return world


def _aabb_distance(left, right):
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    gap = np.maximum(np.maximum(left[0] - right[1],
                               right[0] - left[1]), 0.0)
    return float(np.linalg.norm(gap))


def _broadphase_parts(parts_by_link, cases, wire_radius,
                      clearance_band=25.0):
    """Cull parts with a proved AABB gap outside the reporting band."""
    selected = {}
    report = {}
    for link, parts in parts_by_link.items():
        sweep_min = np.full(3, np.inf)
        sweep_max = np.full(3, -np.inf)
        for case in cases:
            points = _case_points_in_link(case, link)
            sweep_min = np.minimum(sweep_min, points.min(axis=0))
            sweep_max = np.maximum(sweep_max, points.max(axis=0))
        sweep = np.vstack((sweep_min, sweep_max))
        candidates = {}
        excluded = []
        for label, mesh in parts.items():
            lower = _aabb_distance(sweep, mesh.bounds) - wire_radius
            if lower <= clearance_band:
                candidates[label] = mesh
            else:
                excluded.append((lower, label))
        selected[link] = candidates
        report[link] = {
            "input_parts": len(parts),
            "candidate_parts": len(candidates),
            "excluded_parts": len(excluded),
            "clearance_band_mm": clearance_band,
            "minimum_excluded_aabb_clearance_mm": (
                round(min(row[0] for row in excluded), 4)
                if excluded else None
            ),
            "swept_aabb_mm": [sweep_min.round(4).tolist(),
                              sweep_max.round(4).tolist()],
        }
    return selected, report


def _broadphase_manifest_labels(manifest, link, cases, wire_radius,
                                exclude=(), clearance_band=25.0,
                                force_include=()):
    """Cull by exported source AABBs before opening any per-part STL."""
    bounds_by_link = manifest.get("part_bounds")
    if not isinstance(bounds_by_link, dict) or link not in bounds_by_link:
        raise RuntimeError(
            "manifest lacks per-part AABBs; rerun cad/export_links.py"
        )
    excluded_names = set(exclude)
    forced = set(force_include)
    labels = [label for label in manifest["parts"][link]
              if label not in excluded_names]
    sweep_min = np.full(3, np.inf)
    sweep_max = np.full(3, -np.inf)
    for case in cases:
        points = _case_points_in_link(case, link)
        sweep_min = np.minimum(sweep_min, points.min(axis=0))
        sweep_max = np.maximum(sweep_max, points.max(axis=0))
    sweep = np.vstack((sweep_min, sweep_max))
    candidates = []
    removed = []
    for label in labels:
        try:
            bounds = np.asarray(bounds_by_link[link][label], dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"missing/invalid AABB for {link}/{label}") from exc
        lower = _aabb_distance(sweep, bounds) - wire_radius
        if lower <= clearance_band or label in forced:
            candidates.append(label)
        else:
            removed.append((lower, label))
    return candidates, {
        "input_parts": len(labels),
        "candidate_parts": len(candidates),
        "excluded_parts": len(removed),
        "clearance_band_mm": clearance_band,
        "minimum_excluded_aabb_clearance_mm": (
            round(min(row[0] for row in removed), 4) if removed else None
        ),
        "swept_aabb_mm": [sweep_min.round(4).tolist(),
                          sweep_max.round(4).tolist()],
        "source": "manifest source-solid AABBs before STL load",
    }


def _case_clearances(parts_by_link, cases, wire_radius,
                     initial_band=4.0, rank_count=20,
                     progress_label="dynamic"):
    """Return exact worst-case clearances with AABB branch-and-bound.

    A single concatenated link mesh defeats the R-tree broad phase: every wire
    point is compared with a large, unrelated triangle soup.  Here each source
    part keeps its own exact mesh and AABB.  Only case/part pairs whose proved
    AABB lower bound is inside ``band`` reach the exact triangle-distance
    query.  The band expands until at least ``rank_count`` cases have an exact
    result no greater than the band.  Consequently the reported worst and
    lowest rows are exact; all omitted pairs have a mathematical lower bound
    greater than those rows.
    """
    if not cases:
        return []

    target_count = min(rank_count, len(cases))
    case_points = {}
    case_bounds = {}
    unique_cases = {}
    for link in parts_by_link:
        points_for_link = []
        cases_for_unique = []
        seen = {}
        for case_index, case in enumerate(cases):
            points = np.asarray(_case_points_in_link(case, link), dtype=float)
            # Depth does not change the world/static or flyer-local wire path,
            # and shaft-wrap M1 sampling does not change the flyer-local path.
            # Hashing at nanometre precision collapses those exact kinematic
            # repeats before any triangle query (6480 -> 2 for the flyer).
            rounded = np.round(points, decimals=9)
            key = (rounded.shape, hashlib.sha256(rounded.tobytes()).digest())
            unique_index = seen.get(key)
            if unique_index is None:
                unique_index = len(points_for_link)
                seen[key] = unique_index
                points_for_link.append(points)
                cases_for_unique.append([])
            cases_for_unique[unique_index].append(case_index)
        case_points[link] = points_for_link
        unique_cases[link] = cases_for_unique
        case_bounds[link] = [
            np.vstack((points.min(axis=0), points.max(axis=0)))
            for points in points_for_link
        ]

    band = float(initial_band)
    while True:
        _progress(
            f"{progress_label}: exact clearance",
            band_mm=band,
            case_count=len(cases),
            links=list(parts_by_link),
            unique_paths={link: len(case_points[link])
                          for link in parts_by_link},
        )
        per_link = {
            link: np.full(len(cases), np.inf, dtype=float)
            for link in parts_by_link
        }
        for link, parts in parts_by_link.items():
            for part_number, (label, mesh) in enumerate(parts.items(), 1):
                indices = [
                    index for index, bounds in enumerate(case_bounds[link])
                    if _aabb_distance(bounds, mesh.bounds) - wire_radius
                    <= band
                ]
                if not indices:
                    continue
                # A path AABB can overlap a part even when only its short end
                # is nearby.  Apply the same rigorous lower bound per sampled
                # centre point so exact triangle queries never spend time on
                # the long, obviously distant remainder of the wire.
                filtered_points = {}
                for index in indices:
                    points = case_points[link][index]
                    gap = np.maximum(
                        np.maximum(mesh.bounds[0] - points,
                                   points - mesh.bounds[1]),
                        0.0,
                    )
                    lower = np.linalg.norm(gap, axis=1) - wire_radius
                    selected = points[lower <= band]
                    if len(selected):
                        filtered_points[index] = selected
                indices = list(filtered_points)
                if not indices:
                    continue
                _progress(
                    f"{progress_label}: exact clearance",
                    band_mm=band,
                    link=link,
                    part=label,
                    part_number=part_number,
                    part_count=len(parts),
                    candidate_unique_paths=len(indices),
                    candidate_cases=sum(
                        len(unique_cases[link][index]) for index in indices
                    ),
                    candidate_wire_samples=sum(
                        len(filtered_points[index]) for index in indices
                    ),
                    mesh_faces=int(len(mesh.faces)),
                )
                query = trimesh.proximity.ProximityQuery(mesh)
                # Keep exact queries bounded while amortizing their setup.
                for start in range(0, len(indices), 128):
                    batch_indices = indices[start:start + 128]
                    lengths = [len(filtered_points[i])
                               for i in batch_indices]
                    points = np.vstack(
                        [filtered_points[i] for i in batch_indices]
                    )
                    distances = []
                    # Trimesh materializes candidate-triangle work arrays for
                    # each query.  Simple parts safely amortize setup over a
                    # larger point batch; dense imported COTS meshes stay
                    # deliberately small.  This changes only scheduling, not
                    # the exact point-to-triangle distance calculation.
                    face_count = len(mesh.faces)
                    if face_count < 3_000:
                        exact_chunk_size = 2_048
                    elif face_count < 12_000:
                        exact_chunk_size = 1_024
                    else:
                        exact_chunk_size = 128
                    for point_start in range(0, len(points), exact_chunk_size):
                        distances.append(np.asarray(
                            query.on_surface(
                                points[point_start:
                                       point_start + exact_chunk_size]
                            )[1],
                            dtype=float,
                        ))
                    clearances = np.concatenate(distances) - wire_radius
                    if not np.all(np.isfinite(clearances)):
                        raise RuntimeError(
                            f"non-finite {link} wire clearance"
                        )
                    cursor = 0
                    for unique_index, length in zip(batch_indices, lengths):
                        value = float(clearances[cursor:cursor + length].min())
                        for case_index in unique_cases[link][unique_index]:
                            per_link[link][case_index] = min(
                                per_link[link][case_index], value
                            )
                        cursor += length

        finite_exact = []
        for index in range(len(cases)):
            value = min(per_link[link][index] for link in per_link)
            if math.isfinite(value) and value <= band:
                finite_exact.append(value)
        if len(finite_exact) >= target_count:
            break
        band *= 2.0
        if band > 256.0:
            raise RuntimeError(
                "wire clearance branch-and-bound could not establish "
                f"{target_count} exact ranked cases"
            )

    ranked = []
    for index, case in enumerate(cases):
        values = {link: float(per_link[link][index])
                  for link in per_link}
        nearest = min(values, key=values.get)
        ranked.append((values[nearest], nearest, case, values))
    ranked.sort(key=lambda row: row[0])
    return ranked


def _mesh_collision(a, b):
    manager = CollisionManager()
    manager.add_object("guide", b)
    hit, contacts = manager.in_collision_single(a, return_data=True)
    return bool(hit), len(contacts)


def _record_check(report, name, ok, value, requirement=None):
    report["geometry"][name] = {
        "ok": bool(ok), "value": value, "requirement": requirement,
    }
    if not ok:
        report["fail"].append(name)


def _validate_geometry(report, wire):
    static = wire["static"]
    flyer = wire["flyer"]
    lm = static["landmarks"]

    spool = np.array(lm["spool_payoff"])
    felt = np.array(lm["felt_contact"])
    tangent_in = np.array(lm["dancer_tangent_in"])
    tangent_out = np.array(lm["dancer_tangent_out"])
    center = np.array(lm["dancer_center"])
    entry = np.array(lm["entry_corner"])

    felt_deflection = _angle_deg(felt - spool, tangent_in - felt)
    _record_check(report, "felt straight-pass deflection deg",
                  felt_deflection <= 0.1, round(felt_deflection, 6), "<= 0.1")

    felt_offset = float(static["felt_offset_from_stud"])
    felt_ok = 2.25 < felt_offset < 9.75
    _record_check(report, "felt contact radial offset mm", felt_ok,
                  round(felt_offset, 4), "2.25 < offset < 9.75")

    rin = tangent_in[:2] - center[:2]
    rout = tangent_out[:2] - center[:2]
    incoming = tangent_in[:2] - felt[:2]
    outgoing = entry[:2] - tangent_out[:2]
    tangent_error_in = abs(float(np.dot(_unit(rin), _unit(incoming))))
    tangent_error_out = abs(float(np.dot(_unit(rout), _unit(outgoing))))
    _record_check(report, "dancer incoming tangency dot",
                  tangent_error_in < 1e-6, tangent_error_in, "< 1e-6")
    _record_check(report, "dancer outgoing tangency dot",
                  tangent_error_out < 1e-6, tangent_error_out, "< 1e-6")

    expected_in = np.array(static["dancer"]["tangent_in_direction"][:2])
    expected_out = np.array(static["dancer"]["tangent_out_direction"][:2])
    direction_in = float(np.dot(_unit(incoming), _unit(expected_in)))
    direction_out = float(np.dot(_unit(outgoing), _unit(expected_out)))
    _record_check(report, "dancer incoming C1 direction dot",
                  direction_in > 0.999999, direction_in, "> 0.999999")
    _record_check(report, "dancer outgoing C1 direction dot",
                  direction_out > 0.999999, direction_out, "> 0.999999")

    wrap = float(static["dancer"]["wrap_deg"])
    _record_check(report, "dancer wrap deg", abs(wrap - 80.0) < 1e-6,
                  wrap, "80")

    guide_radii = {
        "spool pack": float(static["spool_pack_radius"]),
        "felt inlet fairlead": 3.5,
        "dancer pulley": float(static["dancer"]["path_radius"]),
        "entry elbow": float(static["entry_bend"]["radius"]),
        "flyer elbow": float(flyer["elbow_bend"]["radius"]),
        "tip torus meridian": float(
            wire["tip_guide"]["wire_path_radius_mm"]
        ),
        "tip torus inside path": float(
            wire["tip_guide"]["inside_wire_path_radius_mm"]
        ),
        "shaft-wrap sleeve": float(
            wire["shaft_contact"]["radius_to_wire_center_mm"]
        ),
    }
    for name, radius in guide_radii.items():
        _record_check(report, f"guide radius: {name}", radius >= MIN_BEND,
                      round(radius, 4), f">= {MIN_BEND}")

    report["guides"] = {
        name: {"radius": radius, "ok": radius >= MIN_BEND}
        for name, radius in guide_radii.items()
    }

    sleeve = wire["shaft_contact"]["sleeve"]
    sleeve_span = list(map(float, sleeve["axial_span_mm"]))
    shaft_top = float(sleeve["shaft_top_y_mm"])
    finite_sleeve = (
        sleeve_span[0] < float(wire["shaft_contact"]["axial_y_mm"])
        < sleeve_span[1] <= shaft_top + 1e-9
    )
    _record_check(
        report, "shaft-wrap contact inside finite sleeve", finite_sleeve,
        {"contact_y_mm": wire["shaft_contact"]["axial_y_mm"],
         "span_mm": sleeve_span, "shaft_top_y_mm": shaft_top},
        "span_min < contact < span_max <= shaft top",
    )

    # Prove both trailing rounded-envelope branches and the complete physical
    # torus path for every flyer degree.  Collision clearance is checked below;
    # this is the independent source-geometry continuity/bend gate.
    contact = wire["tooth_contact"]
    guide = wire["tip_guide"]
    feed_local = np.asarray(guide["feed_local_mm"], dtype=float)
    center_local = np.asarray(guide["center_local_mm"], dtype=float)
    guide_errors = []
    guide_turns = []
    for angle in np.radians(np.arange(0.0, 360.0, 1.0)):
        rotation = rot_z(angle)
        center_world = rotation @ center_local
        feed_world = rotation @ feed_local
        for motion_sign in (-1, 1):
            target = tooth_contact_point(center_world, contact, motion_sign)
            _, meta = tip_guide_path(
                feed_world, target, guide, wire["radius_max"], rotation,
            )
            guide_errors.extend((meta["entry_tangent_error"],
                                 meta["exit_tangent_error"]))
            guide_turns.append(meta["arc_turn_deg"])
    max_error = max(guide_errors)
    _record_check(report, "tip torus sampled C1 tangent error deg",
                  max_error <= 1.01, round(max_error, 6), "<= 1.01")
    report["tip_guide_sweep"] = {
        "angles": 360,
        "motion_signs": 2,
        "arc_turn_deg_range": [round(min(guide_turns), 4),
                               round(max(guide_turns), 4)],
        "max_discretized_tangent_error_deg": round(max_error, 6),
        "note": "error is the 2-degree display arc chord; analytic tangency is exact",
    }


def _validate_meshes(report):
    report["mesh_integrity"] = {}
    for name in ("wire_static", "wire_flyer"):
        mesh = _load_mesh(str(LINKS / f"{name}.stl"))
        components = int(mesh.body_count)
        ok = components == 1 and mesh.is_watertight
        report["mesh_integrity"][name] = {
            "connected_components": components,
            "watertight": bool(mesh.is_watertight),
            "ok": ok,
        }
        if not ok:
            report["fail"].append(f"{name} disconnected or open")

    # Presentation tube must fit the deliberately modeled guide channels.
    static_wire = _load_mesh(str(LINKS / "wire_static.stl"))
    flyer_wire = _load_mesh(str(LINKS / "wire_flyer.stl"))
    fits = [
        ("entry bracket channel", static_wire,
         _load_mesh(str(LINKS / "parts/static/entry_bracket.stl"))),
        ("felt inlet fairlead", static_wire,
         _load_mesh(str(LINKS / "parts/static/felt_guide_in.stl"))),
        ("entry eyelet bore", static_wire,
         _load_mesh(str(LINKS / "parts/static/entry_eyelet.stl"))),
        ("flyer shaft bore", flyer_wire,
         _load_mesh(str(LINKS / "parts/flyer/alu_tube.stl"))),
        ("flyer elbow channel", flyer_wire,
         _load_mesh(str(LINKS / "parts/flyer/wire_elbow.stl"))),
        ("flyer pulley bore", flyer_wire,
         _load_mesh(str(LINKS / "parts/flyer/flyer_pulley.stl"))),
    ]
    report["guide_channel_fit"] = {}
    for name, wire_mesh, guide_mesh in fits:
        hit, contacts = _mesh_collision(wire_mesh, guide_mesh)
        report["guide_channel_fit"][name] = {
            "collision": hit, "contacts": contacts, "ok": not hit,
        }
        if hit:
            report["fail"].append(f"visual wire intersects {name}")


def _validate_static(report, manifest, wire, wire_radius):
    points = _sample_polyline(wire["static"]["points"])
    # Only the listed parts are intentional wire contacts/guides.  Everything
    # else, including M2 belt/motor/pulley, remains authoritative collision
    # geometry for this check.
    static_exclusions = STATIC_WIRE_INTENTIONAL_CONTACTS
    required_report_parts = {
        "gt2_belt", "m2_motor_pulley", "m2_motor", "felt_tensioner",
    }
    candidate_labels, static_broadphase = _broadphase_manifest_labels(
        manifest, "static", [{"points": points}], wire_radius,
        exclude=static_exclusions,
        force_include=required_report_parts,
    )
    candidates = _load_parts(
        manifest, "static", exclude=static_exclusions,
        include=candidate_labels,
    )
    print(f"static wire broad phase: {len(candidates)}/"
          f"{static_broadphase['input_parts']} parts",
          flush=True)
    rows = _rank_part_clearances(candidates, points, wire_radius)
    worst, nearest = rows[0]
    report["static"] = {
        "sample_count": len(points),
        "broadphase": static_broadphase,
        "worst_clearance": round(worst, 4),
        "nearest_part": nearest,
        "lowest": [{"clearance": round(v, 4), "part": label}
                   for v, label in rows[:12]],
        "reported_parts": {
            label: round(next(v for v, name in rows if name == label), 4)
            for label in (
                "gt2_belt", "m2_motor_pulley", "m2_motor",
                "felt_tensioner",
            )
        },
        "intentional_contacts": list(STATIC_WIRE_INTENTIONAL_CONTACTS),
        "ok": worst > 0.0,
    }
    if worst <= 0.0:
        report["fail"].append(f"static wire contact: {nearest}")


def _captured_wrap_intervals(events, timeline):
    """Return physical shaft-wrap intervals from either capture lane.

    ContractWind emits explicit phase markers.  Untouched upstream emits only
    the method call and the actual M1/M2 commands, so infer the same physical
    interval from the M1 target through the following M2 return command.  The
    inferred raw lane is the release authority; adapter markers remain useful
    for diagnostic captures but are never required for raw validation.
    """

    marker_events = [
        event for event in events if event.get("e") == "shaft_wrap_phase"]
    if marker_events:
        wraps = []
        active = None
        phase_sequences = {}
        for event in marker_events:
            number = int(event["next_wire_idx"])
            phase = event.get("phase")
            phase_sequences.setdefault(number, []).append(phase)
            if phase == "contact_start":
                if active is not None:
                    raise RuntimeError("nested shaft-wrap contact_start markers")
                active = {
                    "start": float(event["t"]),
                    "number": number,
                    "start_m0": float(event["m0_rad"]),
                    "start_m1": float(event["m1_rad"]),
                    "start_m2": float(event["m2_rad"]),
                }
            elif phase == "contact_done":
                if active is None or active["number"] != number:
                    raise RuntimeError("unpaired shaft-wrap contact_done marker")
                active["end"] = float(event["t"])
                active["end_m0"] = float(event["m0_rad"])
                active["end_m1"] = float(event["m1_rad"])
                active["end_m2"] = float(event["m2_rad"])
                active["delta_m1"] = active["end_m1"] - active["start_m1"]
                active["tangent_side"] = (
                    -1 if active["delta_m1"] > 0.0 else 1)
                wraps.append(active)
                active = None
        if active is not None:
            raise RuntimeError("shaft-wrap contact interval is not closed")
        required = [
            "prepark_start", "m0_parked", "contact_start", "contact_done"]
        ok = (
            len(wraps) == 2
            and sorted(phase_sequences) == [1, 2]
            and all(phase_sequences[number] == required for number in (1, 2)))
        return wraps, {
            "source": "contract_adapter_phase_markers",
            "ok": ok,
            "required_sequence": required,
            "observed": phase_sequences,
        }

    calls = [
        event for event in events
        if event.get("e") == "wind_wire_around_shaft"]
    wraps = []
    observed = {}
    for call in calls:
        number = int(call["args"][0])
        start = float(call["t"])
        m1_command = next(
            event for event in events
            if (event.get("e") == "cmd" and event.get("m") == 1
                and float(event["t"]) >= start - 1.0e-12))
        m2_return = next(
            event for event in events
            if (event.get("e") == "cmd" and event.get("m") == 2
                and float(event["t"]) > start + 1.0e-12))
        end = float(m2_return["t"])
        start_pose = timeline.pose_at(start)
        end_pose = timeline.pose_at(end)
        target_m1 = float(m1_command["a"])
        delta_m1 = target_m1 - float(start_pose[1])
        wraps.append({
            "start": start,
            "end": end,
            "number": number,
            "start_m0": float(start_pose[0]),
            "start_m1": float(start_pose[1]),
            "start_m2": float(start_pose[2]),
            "end_m0": float(end_pose[0]),
            "end_m1": float(end_pose[1]),
            "end_m2": float(end_pose[2]),
            "target_m1": target_m1,
            "delta_m1": delta_m1,
            "tangent_side": -1 if delta_m1 > 0.0 else 1,
        })
        observed[number] = [
            "wind_wire_around_shaft", "m1_absolute_target", "m2_return"]
    fixed = all(
        abs(row["end_m0"] - row["start_m0"]) <= 1.0e-9
        and abs(row["end_m2"] - row["start_m2"]) <= 1.0e-9
        and abs(row["end_m1"] - row["target_m1"]) <= 0.01
        for row in wraps)
    required = [
        "wind_wire_around_shaft", "m1_absolute_target", "m2_return"]
    return wraps, {
        "source": "raw_upstream_commands",
        "ok": len(wraps) == 2 and sorted(observed) == [1, 2] and fixed,
        "required_sequence": required,
        "observed": observed,
        "m0_m2_fixed_and_m1_arrived": fixed,
    }


def _validate_moving(report, manifest, wire, wire_radius, capture_path):
    # Keep the final-wound stator envelope in this check. The free span is
    # allowed to meet it only at the analytically constructed contact point;
    # sampled interior points must remain outside.
    moving_exclusions = {
        "flyer": ("tip_toroid_guide", "wire_elbow"),
        "spindle": (),
        "carriage": (),
        "static": ("entry_eyelet",),
    }

    guide = wire["tip_guide"]
    feed_local = np.asarray(guide["feed_local_mm"], dtype=float)
    guide_center_local = np.asarray(guide["center_local_mm"], dtype=float)
    standoff = float(manifest["m0_home_standoff"])
    mm_per_rad = float(manifest["mm_per_rad_m0"])
    stator = manifest["stator"]
    contact = wire["tooth_contact"]
    shallow_depth, deep_depth = map(
        float, contact["insertion_depth_range_mm"]
    )
    if deep_depth > float(manifest["max_insertion_mm"]) + 1e-9:
        raise RuntimeError("wire contact depth exceeds mechanical insertion limit")
    lay_depths = np.linspace(shallow_depth, deep_depth, 9)
    angle_step_deg = 1.0
    angles = np.radians(np.arange(0.0, 360.0, angle_step_deg))

    # Path shape is independent of M0 depth; cache the 720 angle/direction
    # templates once and reuse them at all nine spindle translations.
    path_templates = []
    for angle in angles:
        rotation = rot_z(angle)
        feed = rotation @ feed_local
        guide_center = rotation @ guide_center_local
        for motion_sign in (-1, 1):
            lay = tooth_contact_point(guide_center, contact, motion_sign)
            path, guide_meta = tip_guide_path(
                feed, lay, guide, wire_radius, rotation,
            )
            samples = _trim_sampled_polyline(
                path, start_mm=0.5, end_mm=0.75, spacing=0.25,
            )
            path_templates.append((
                angle, rotation, motion_sign, samples, guide_meta,
            ))

    cases = []
    for depth in lay_depths:
        m0 = (stator["od"] / 2.0 - depth - standoff) / mm_per_rad
        dz = m0 * mm_per_rad
        translation = np.array([0.0, 0.0, dz])
        for angle, rotation, motion_sign, samples, guide_meta in path_templates:
            cases.append({
                    "points": samples,
                    "flyer_rotation": rotation,
                    "spindle_rotation": np.eye(3),
                    "spindle_translation": translation,
                    "carriage_translation": translation,
                    "meta": {
                        "segment": "feed->torus->tooth",
                        "depth": round(float(depth), 4),
                        "angle_deg": round(math.degrees(angle), 3),
                        "motion_sign": motion_sign,
                        "guide_turn_deg": round(
                            guide_meta["arc_turn_deg"], 4
                        ),
                    },
            })

    parts_by_link = {}
    moving_broadphase = {}
    for link in ("flyer", "spindle", "carriage", "static"):
        labels, phase_report = _broadphase_manifest_labels(
            manifest, link, cases, wire_radius,
            exclude=moving_exclusions[link],
        )
        parts_by_link[link] = _load_parts(
            manifest, link, exclude=moving_exclusions[link], include=labels,
        )
        moving_broadphase[link] = phase_report
    print("moving wire broad phase: " + ", ".join(
        f"{name} {row['candidate_parts']}/{row['input_parts']} parts"
        for name, row in moving_broadphase.items()
    ), flush=True)
    ranked = _case_clearances(
        parts_by_link, cases, wire_radius, progress_label="moving wire",
    )
    worst = ranked[0]

    nearest_parts = _rank_part_clearances(
        parts_by_link[worst[1]],
        _case_points_in_link(worst[2], worst[1]),
        wire_radius,
    )
    report["moving"] = {
        "pose_count": len(lay_depths) * len(angles),
        "segment_cases": len(ranked),
        "worst_clearance": round(worst[0], 4),
        "ok": worst[0] > 0.0,
        "contact_model": contact,
        "tip_guide_model": guide,
        "sampling": {
            "lay_depth_count": len(lay_depths),
            "flyer_angle_step_deg": angle_step_deg,
            "wire_center_spacing_max_mm": 0.25,
            "surface_distance_method": (
                "exact point-to-triangle distance with per-part AABB "
                "branch-and-bound"
            ),
            "crossing_detection_bound_mm": 0.125,
            "maximum_unsampled_tip_chord_mm": round(
                2.0 * float(manifest["flyer_tip_r"])
                * math.sin(math.radians(angle_step_deg) / 2.0), 6
            ),
        },
        "broadphase": moving_broadphase,
        "nearest_part_at_worst": nearest_parts[0][1],
        "lowest": [
            {"clearance": round(row[0], 4), **row[2]["meta"],
             "nearest": row[1]}
            for row in ranked[:20]
        ],
    }
    if worst[0] <= 0.0:
        report["fail"].append(
            f"moving wire contact: {worst[1]}/{nearest_parts[0][1]}"
        )

    # The two project-controller shaft wraps occur with the stator retracted
    # to the rotation plane. Validate the physical sleeve tangent and complete
    # torus guide path with <=0.5-degree M1 steps plus every trajectory knot.
    events = load_events(capture_path)
    timeline = Timeline(events)
    wraps, wrap_capture_contract = _captured_wrap_intervals(events, timeline)
    marker_contract_ok = bool(wrap_capture_contract["ok"])
    if not marker_contract_ok:
        report["shaft_wrap"] = {
            "ok": False,
            "reason": "captured shaft-wrap interval contract failed",
            "capture_contract": wrap_capture_contract,
        }
        report["fail"].append(
            "shaft wrap capture lacks two complete physical intervals"
        )
        return

    shaft_cases = []
    shaft_alternate_cases = []
    shaft_contact = wire["shaft_contact"]
    ref = np.array([0.0, 0.0, standoff])
    max_dm1_deg = 0.5
    max_dm2_deg = 1.0
    max_dm0_mm = 0.25
    observed_steps = {"m0_mm": 0.0, "m1_deg": 0.0, "m2_deg": 0.0}
    for wrap in wraps:
        # Only the explicit physical sleeve-contact interval is modeled here.
        # The preceding M0/M2 parking transition remains part of the continuous
        # tail audit; it must not be mislabeled as wire already on the shaft.
        # Build a dense time set per trajectory-knot interval through both M1
        # turns. Direct cumulative-radian deltas are intentional.
        knots = sorted({wrap["start"], wrap["end"], *(
            t for t in timeline.knot_times()
            if wrap["start"] <= t <= wrap["end"]
        )})
        times = set()
        for left, right in zip(knots, knots[1:]):
            a = timeline.pose_at(left)
            b = timeline.pose_at(right)
            step_count = max(
                1,
                math.ceil(abs(b[0] - a[0]) * mm_per_rad / max_dm0_mm),
                math.ceil(abs(b[1] - a[1]) /
                          math.radians(max_dm1_deg)),
                math.ceil(abs(b[2] - a[2]) /
                          math.radians(max_dm2_deg)),
            )
            segment_times = np.linspace(left, right, step_count + 1)
            times.update(float(value) for value in segment_times)
        for time in sorted(times):
            m0, m1, m2 = timeline.pose_at(time)
            dz = m0 * mm_per_rad
            flyer_rotation = rot_z(m2)
            spindle_rotation = rot_y(m1)
            axis = np.array([0.0, 0.0, standoff + dz])
            spindle_translation = axis - spindle_rotation @ ref
            feed = flyer_rotation @ feed_local
            guide_center = flyer_rotation @ guide_center_local
            for side, destination in (
                (wrap["tangent_side"], shaft_cases),
                (-wrap["tangent_side"], shaft_alternate_cases),
            ):
                target = shaft_tangent_point(
                    guide_center, axis[2], shaft_contact, side,
                )
                path, guide_meta = tip_guide_path(
                    feed, target, guide, wire_radius, flyer_rotation,
                )
                samples = _trim_sampled_polyline(
                    path, start_mm=0.5, end_mm=0.75, spacing=0.25,
                )
                destination.append({
                    "points": samples,
                    "flyer_rotation": flyer_rotation,
                    "spindle_rotation": spindle_rotation,
                    "spindle_translation": spindle_translation,
                    "carriage_translation": np.array([0.0, 0.0, dz]),
                    "meta": {
                        "wrap": wrap["number"], "time": round(time, 4),
                        "tangent_side": side,
                        "physical": destination is shaft_cases,
                        "guide_turn_deg": round(
                            guide_meta["arc_turn_deg"], 4
                        ),
                    },
                })

        ordered_times = sorted(times)
        for left, right in zip(ordered_times, ordered_times[1:]):
            a = timeline.pose_at(left)
            b = timeline.pose_at(right)
            observed_steps["m0_mm"] = max(
                observed_steps["m0_mm"], abs(b[0] - a[0]) * mm_per_rad,
            )
            observed_steps["m1_deg"] = max(
                observed_steps["m1_deg"],
                math.degrees(abs(b[1] - a[1])),
            )
            observed_steps["m2_deg"] = max(
                observed_steps["m2_deg"],
                math.degrees(abs(b[2] - a[2])),
            )

    if not shaft_cases:
        report["shaft_wrap"] = {"ok": False, "reason": "no captured wraps"}
        report["fail"].append("shaft wrap wire validation has no intervals")
        return


    shaft_exclusions = dict(moving_exclusions)
    shaft_exclusions["spindle"] = ("shaft_wrap_sleeve",)
    shaft_parts_by_link = {}
    shaft_broadphase = {}
    combined_shaft_cases = shaft_cases + shaft_alternate_cases
    for link in ("flyer", "spindle", "carriage", "static"):
        labels, phase_report = _broadphase_manifest_labels(
            manifest, link, combined_shaft_cases, wire_radius,
            exclude=shaft_exclusions[link],
        )
        shaft_parts_by_link[link] = _load_parts(
            manifest, link, exclude=shaft_exclusions[link], include=labels,
        )
        shaft_broadphase[link] = phase_report
    print("shaft-wrap broad phase: " + ", ".join(
        f"{name} {row['candidate_parts']}/{row['input_parts']} parts"
        for name, row in shaft_broadphase.items()
    ), flush=True)

    shaft_ranked = _case_clearances(
        shaft_parts_by_link, shaft_cases, wire_radius,
        progress_label="shaft-wrap physical tangent",
    )
    shaft_alternate_ranked = _case_clearances(
        shaft_parts_by_link, shaft_alternate_cases, wire_radius,
        progress_label="shaft-wrap alternate tangent",
    )
    shaft_worst = shaft_ranked[0]
    shaft_nearest = _rank_part_clearances(
        shaft_parts_by_link[shaft_worst[1]],
        _case_points_in_link(shaft_worst[2], shaft_worst[1]), wire_radius,
    )
    report["shaft_wrap"] = {
        "wrap_count": len(wraps),
        "phase_marker_contract": {
            "ok": marker_contract_ok,
            **wrap_capture_contract,
        },
        "contact_intervals": [
            {
                "number": wrap["number"],
                "start_s": wrap["start"],
                "end_s": wrap["end"],
                "m0_start_rad": wrap["start_m0"],
                "m0_end_rad": wrap["end_m0"],
                "m2_start_rad": wrap["start_m2"],
                "m2_end_rad": wrap["end_m2"],
                "m1_delta_rad": wrap["delta_m1"],
            }
            for wrap in wraps
        ],
        "case_count": len(shaft_ranked),
        "contact_model": shaft_contact,
        "sampling": {
            "maximum_m1_step_deg": max_dm1_deg,
            "maximum_m2_step_deg": max_dm2_deg,
            "maximum_m0_step_mm": max_dm0_mm,
            "observed_maximum_steps": {
                key: round(value, 6) for key, value in observed_steps.items()
            },
            "wire_center_spacing_max_mm": 0.25,
            "surface_distance_method": (
                "exact point-to-triangle distance with per-part AABB "
                "branch-and-bound"
            ),
            "crossing_detection_bound_mm": 0.125,
            "trajectory_knots_included": True,
            "maximum_unsampled_sleeve_surface_chord_mm": round(
                2.0 * float(shaft_contact["radius_to_wire_center_mm"])
                * math.sin(math.radians(max_dm1_deg) / 2.0), 6
            ),
        },
        "broadphase": shaft_broadphase,
        "worst_clearance": round(shaft_worst[0], 4),
        "nearest_part_at_worst": shaft_nearest[0][1],
        "ok": shaft_worst[0] > 0.0,
        "lowest": [
            {"clearance": round(row[0], 4), **row[2]["meta"],
             "nearest": row[1]}
            for row in shaft_ranked[:20]
        ],
        "alternate_tangent_diagnostic": {
            "worst_clearance": round(shaft_alternate_ranked[0][0], 4),
            "note": "opposite non-deposition tangent; reported but not gated",
        },
    }
    if shaft_worst[0] <= 0.0:
        report["fail"].append(
            f"shaft-wrap wire contact: {shaft_worst[1]}/{shaft_nearest[0][1]}"
        )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Deterministic machine and captured shaft-wire audit")
    parser.add_argument(
        "--capture", type=Path,
        default=OUT / "capture" / "commands.jsonl",
        help="captured command stream whose shaft-wrap motion is validated",
    )
    parser.add_argument(
        "--output", type=Path,
        default=OUT / "reports" / "wirepath.json",
        help="JSON report path",
    )
    args = parser.parse_args(argv)
    capture_path = args.capture
    if not capture_path.is_absolute():
        capture_path = (HERE.parent / capture_path).resolve()
    output_path = args.output
    if not output_path.is_absolute():
        output_path = (HERE.parent / output_path).resolve()
    if not capture_path.is_file():
        parser.error(f"capture does not exist: {capture_path}")

    manifest_path = LINKS / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    capture_events = load_events(capture_path)
    capture_meta = next((
        event for event in capture_events if event.get("e") == "meta"), {})
    wire = manifest.get("wire")
    if not wire:
        raise RuntimeError("manifest lacks shared wire geometry; rerun export_links.py")
    wire_radius = float(wire["radius_max"])
    report = {
        "schema": "wirepath-validation/v2",
        "geometry": {}, "guides": {}, "fail": [],
        "evidence": {
            "manifest_sha256": _sha256(manifest_path),
            "capture_path": str(capture_path),
            "capture_sha256": _sha256(capture_path),
            "capture_schema": capture_meta.get("capture_schema"),
            "controller_mode": capture_meta.get("controller_mode"),
            "winder_commit": capture_meta.get("winder_commit"),
            "wirepath_source_sha256": _sha256(__file__),
            "distance_method": (
                "exact unsigned point-to-triangle distance; continuous "
                "crossing detection bounded by 0.25 mm centerline sampling"
            ),
        },
    }

    print("wirepath stage 1/4: analytical geometry", flush=True)
    _progress("stage 1/4: analytical geometry")
    _validate_geometry(report, wire)
    print("wirepath stage 2/4: exported guide meshes", flush=True)
    _progress("stage 2/4: exported guide meshes")
    _validate_meshes(report)
    print("wirepath stage 3/4: static free path", flush=True)
    _progress("stage 3/4: static free path")
    _validate_static(report, manifest, wire, wire_radius)
    print("wirepath stage 4/4: moving and shaft-wrap paths", flush=True)
    _progress("stage 4/4: moving and shaft-wrap paths")
    _validate_moving(
        report, manifest, wire, wire_radius, capture_path)

    report["status"] = "PASS" if not report["fail"] else "FAIL"
    report["passed"] = not report["fail"]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2) + "\n")
    _progress("complete", result="PASS" if not report["fail"] else "FAIL")

    print("=== wire geometry ===")
    for name, result in report["geometry"].items():
        print(f"  [{'OK' if result['ok'] else 'FAIL'}] {name}: "
              f"{result['value']}")
    print("\n=== exported wire meshes ===")
    for name, result in report["mesh_integrity"].items():
        print(f"  [{'OK' if result['ok'] else 'FAIL'}] {name}: "
              f"{result['connected_components']} component(s), "
              f"watertight={result['watertight']}")
    print("\n=== static free path ===")
    print(f"  min {report['static']['worst_clearance']:.3f} mm to "
          f"{report['static']['nearest_part']}")
    print("  belt/M2 clearances: " + ", ".join(
        f"{name}={value:.3f} mm"
        for name, value in report["static"]["reported_parts"].items()
    ))
    print("\n=== moving free path ===")
    first = report["moving"]["lowest"][0]
    print(f"  min {first['clearance']:.3f} mm {first['segment']} "
          f"depth={first['depth']:.2f} angle={first['angle_deg']:.0f} "
          f"nearest={first['nearest']}/"
          f"{report['moving']['nearest_part_at_worst']}")
    shaft = report["shaft_wrap"]
    print("\n=== captured shaft-wrap free path ===")
    print(f"  min {shaft['worst_clearance']:.3f} mm over "
          f"{shaft['wrap_count']} wraps; nearest="
          f"{shaft['lowest'][0]['nearest']}/"
          f"{shaft['nearest_part_at_worst']}")
    print(f"\nRESULT: {'PASS' if not report['fail'] else 'FAIL'}")
    if report["fail"]:
        for failure in report["fail"]:
            print("  -", failure)
    return 0 if not report["fail"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
