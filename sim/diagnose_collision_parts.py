"""Resolve link-level clearance failures to individual CAD occurrences.

``collide.py`` intentionally uses one BVH per moving link so it can sweep the
entire winding cycle quickly.  This companion tool rechecks the reported worst
pose with one BVH per labeled part.  It is diagnostic evidence, not a second
clearance gate: the authoritative pass/fail remains ``collide.py``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import fcl
import numpy as np
import trimesh

from collide import (EXEMPT, Kinematics, LINKS, make_bvh,
                     resolve_part_assets)


ROOT = Path(__file__).resolve().parent.parent
REPORT = ROOT / "out" / "reports" / "clearance.json"
OUTPUT = ROOT / "out" / "reports" / "clearance_part_diagnostics.json"


def _object(mesh: trimesh.Trimesh, transform: fcl.Transform):
    return fcl.CollisionObject(make_bvh(mesh), transform)


def _part_results(manifest: dict, pair: tuple[str, str], pose: dict, *,
                  links: Path = LINKS,
                  assets_by_link: dict | None = None) -> dict:
    a, b = pair
    kin = Kinematics(manifest)
    if assets_by_link is None:
        assets_by_link = resolve_part_assets(manifest, links)
    transforms = {}
    for link in pair:
        rotation, translation = kin.link_tf(
            link, pose["m0"], pose["m1"], pose["m2"])
        transforms[link] = fcl.Transform(rotation, translation)

    objects: dict[str, dict[str, fcl.CollisionObject]] = {}
    exemptions = EXEMPT.get(pair, {})
    for link in pair:
        objects[link] = {}
        for label, path in assets_by_link[link].items():
            if label in exemptions.get(link, set()):
                continue
            mesh = trimesh.load(path, force="mesh")
            objects[link][label] = _object(mesh, transforms[link])

    collisions = []
    nearest = []
    for label_a, object_a in objects[a].items():
        for label_b, object_b in objects[b].items():
            collision = fcl.CollisionResult()
            fcl.collide(object_a, object_b, fcl.CollisionRequest(), collision)
            if collision.is_collision:
                collisions.append({"a": label_a, "b": label_b})
                continue
            distance = fcl.distance(
                object_a, object_b, fcl.DistanceRequest(), fcl.DistanceResult())
            if math.isfinite(distance):
                nearest.append((float(distance), label_a, label_b))

    nearest.sort()
    return {
        "pair": [a, b],
        "pose": pose,
        "collision_count": len(collisions),
        "collisions": collisions,
        "nearest_noncolliding": [
            {"distance_mm": round(distance, 4), "a": label_a, "b": label_b}
            for distance, label_a, label_b in nearest[:20]
        ],
    }


def _workspace_path(value: Path) -> Path:
    return value if value.is_absolute() else (ROOT / value).resolve()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Resolve link-level clearance minima to CAD occurrences")
    parser.add_argument("--links", type=Path, default=LINKS)
    parser.add_argument("--report", type=Path, default=REPORT)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args(argv)
    links = _workspace_path(args.links)
    report_path = _workspace_path(args.report)
    output_path = _workspace_path(args.output)

    manifest_path = links / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    clearance = json.loads(report_path.read_text())
    assets_by_link = resolve_part_assets(manifest, links)
    results = []
    for pair_name, entry in clearance["min_clearance_mm"].items():
        pair = tuple(pair_name.split("-", 1))
        result = _part_results(
            manifest, pair, entry["pose"], links=links,
            assets_by_link=assets_by_link)
        results.append(result)
        print(f"{pair_name}: {result['collision_count']} colliding part pairs")
        for item in result["collisions"][:20]:
            print(f"  HIT  {item['a']} <-> {item['b']}")
        for item in result["nearest_noncolliding"][:5]:
            print(f"  {item['distance_mm']:7.3f}  {item['a']} <-> {item['b']}")

    stale_exemptions = []
    for pair, by_link in EXEMPT.items():
        for link, labels in by_link.items():
            available = set(assets_by_link.get(link, {}))
            stale_exemptions.extend({
                "pair": list(pair), "link": link, "label": label,
            } for label in sorted(set(labels) - available))

    diagnostic = {
        "schema": "collision-part-diagnostics/v2",
        "status": "DIAGNOSTIC_ONLY",
        "inputs": {
            "clearance_report": str(report_path),
            "clearance_report_sha256": hashlib.sha256(
                report_path.read_bytes()).hexdigest(),
            "links": str(links),
            "manifest_sha256": hashlib.sha256(
                manifest_path.read_bytes()).hexdigest(),
        },
        "stale_fit_exemptions": stale_exemptions,
        "pairs": results,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(diagnostic, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
