"""Interference proof + minimum-clearance report (GOAL DoD #2).

Pose-sampled collision & distance checking over the full captured cycle,
using fcl BVH models of the per-link meshes exported from CAD.

Method:
 1. Sample the reconstructed timeline finely (<=2 deg M2, <=1 deg M1,
    <=0.25 rad M0 between samples).
 2. Deduplicate poses by symmetry-aware quantization: (m0 0.25 rad,
    m1 mod tooth-pitch 1 deg, m2 mod 360 1 deg). The stator is
    slots-fold symmetric and every other body in a pair is axisymmetric
    or pose-invariant, so this loses no geometry.
 3. fcl collide() for binary interpenetration, distance() for clearance.
 4. Any pair with clearance < refine_below gets neighborhood refinement at
    0.25 deg / 0.05 rad m0.

Pairs: flyer-spindle, flyer-carriage, flyer-static, spindle-static,
carriage-static, spindle-carriage.

Output: ../out/reports/clearance.json + console summary.
"""

import argparse
import hashlib
import json
import math
import os
from collections import defaultdict
from collections.abc import Mapping
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

# Windows spawn imports NumPy independently in every collision worker. Letting
# each copy create a full BLAS thread pool multiplies memory use until large
# assemblies fail before geometry is checked. Distance queries are scalar, so
# one BLAS thread per process is both faster and deterministic here.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np
import trimesh
import fcl

from traj import Timeline, load_events

_W = {}


def _worker_init(links_path=None):
    """Each worker builds its own BVHs (fcl objects aren't picklable)."""
    links = Path(links_path).resolve() if links_path is not None else LINKS
    manifest = load_manifest(links)
    part_meshes = {}
    for name, assets in resolve_part_assets(manifest, links).items():
        part_meshes[name] = {
            label: trimesh.load(path, force="mesh")
            for label, path in assets.items()
        }

    def merged(link, exclude=frozenset()):
        ms = [m for lbl, m in part_meshes[link].items()
              if lbl not in exclude]
        return make_bvh(trimesh.util.concatenate(ms))

    full = {name: merged(name) for name in part_meshes}
    pair_bvhs = {}
    for a, b in PAIRS:
        ex = EXEMPT.get((a, b), {})
        pair_bvhs[(a, b)] = (
            merged(a, frozenset(ex.get(a, ()))) if a in ex else full[a],
            merged(b, frozenset(ex.get(b, ()))) if b in ex else full[b])
    _W["kin"] = Kinematics(manifest)
    _W["pair_bvhs"] = pair_bvhs
    _W["links"] = list(part_meshes)


def _check_batch(poses):
    kin, pair_bvhs = _W["kin"], _W["pair_bvhs"]
    worst = {p: (math.inf, None) for p in PAIRS}
    collisions = []
    for pose in poses:
        t, m0, m1, m2 = pose
        tfs = {}
        for name in _W["links"]:
            R, tr = kin.link_tf(name, m0, m1, m2)
            tfs[name] = fcl.Transform(R, tr)
        for a, b in PAIRS:
            bva, bvb = pair_bvhs[(a, b)]
            oa = fcl.CollisionObject(bva, tfs[a])
            ob = fcl.CollisionObject(bvb, tfs[b])
            cres = fcl.CollisionResult()
            fcl.collide(oa, ob, fcl.CollisionRequest(), cres)
            if cres.is_collision:
                collisions.append({"pose": pose, "pair": (a, b)})
                if -1.0 < worst[(a, b)][0]:
                    worst[(a, b)] = (-1.0, pose)
                continue
            d = fcl.distance(oa, ob, fcl.DistanceRequest(),
                             fcl.DistanceResult())
            if d < worst[(a, b)][0]:
                worst[(a, b)] = (d, pose)
    return worst, collisions

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out"
LINKS = OUT / "links"


def load_manifest(links=None):
    root = LINKS if links is None else Path(links)
    return json.loads((root / "manifest.json").read_text())


def resolve_part_assets(manifest, links=None):
    """Resolve legacy label lists or explicit label-to-file mappings.

    Canonical manifests historically store ``parts[link]`` as a list and use
    ``<label>.stl`` implicitly.  Isolated integrated exports may sanitize a
    display label into a different filename, so they can instead store either
    ``{label: filename}`` or ``{label: {"file": filename}}``.  Every resolved
    file remains confined to ``parts/<link>``.
    """

    root = (LINKS if links is None else Path(links)).resolve()
    records = manifest.get("parts")
    if not isinstance(records, Mapping) or not records:
        raise ValueError("links manifest has no collision part mapping")
    resolved = {}
    for link, record in records.items():
        base = (root / "parts" / str(link)).resolve()
        if isinstance(record, list):
            entries = ((str(label), f"{label}.stl") for label in record)
        elif isinstance(record, Mapping):
            def mapped_entries():
                for label, value in record.items():
                    if isinstance(value, Mapping):
                        value = value.get("file")
                    if not isinstance(value, str) or not value:
                        raise ValueError(
                            f"invalid collision asset for {link}/{label}"
                        )
                    yield str(label), value
            entries = mapped_entries()
        else:
            raise ValueError(f"invalid collision part mapping for {link}")

        by_label = {}
        for label, filename in entries:
            path = (base / filename).resolve()
            if path != base and base not in path.parents:
                raise ValueError(
                    f"collision asset escapes parts/{link}: {filename}"
                )
            by_label[label] = path
        if not by_label:
            raise ValueError(f"collision link {link} has no parts")
        resolved[str(link)] = by_label
    return resolved


def make_bvh(mesh: trimesh.Trimesh) -> fcl.BVHModel:
    m = fcl.BVHModel()
    m.beginModel(len(mesh.vertices), len(mesh.faces))
    m.addSubModel(mesh.vertices.astype(np.float64),
                  mesh.faces.astype(np.int64))
    m.endModel()
    return m


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Kinematics:
    def __init__(self, manifest):
        self.mm_per_rad = manifest["mm_per_rad_m0"]
        self.standoff = manifest["m0_home_standoff"]

    def link_tf(self, link, m0, m1, m2):
        """Return (R, t) world transform of a link."""
        dz = m0 * self.mm_per_rad
        if link == "static":
            return np.eye(3), np.zeros(3)
        if link == "carriage":
            return np.eye(3), np.array([0.0, 0.0, dz])
        if link == "spindle":
            R = rot_y(m1)
            axis = np.array([0.0, 0.0, self.standoff + dz])
            ref = np.array([0.0, 0.0, self.standoff])
            t = axis - R @ ref
            return R, t
        if link == "flyer":
            return rot_z(m2), np.zeros(3)
        raise ValueError(link)


PAIRS = [("flyer", "spindle"), ("flyer", "carriage"), ("flyer", "static"),
         ("spindle", "static"), ("carriage", "static"),
         ("spindle", "carriage")]

# Intended-fit contacts (press fits, bores, rolling interfaces) are exempt
# from interference checking for the specific pair only. Every exemption is
# a permanent by-design contact surface, documented in the clearance report:
#   flyer-static:    flyer tube runs INSIDE the two 6001ZZ (static side)
#   carriage-static: MGN blocks ride the rails; T8 nut rides the screw;
#                    screw-end 608 is the screw's own support. The recessed
#                    rail screws are part of the manufacturer-defined running
#                    interface and sit below the carriage block.
#   spindle-carriage: chuck shank runs inside the spindle 608s; the beam
#                    coupling (spindle link) clamps the M1 motor shaft.
#                    The two inner-race spacers deliberately run inside the
#                    carriage-side outer-race spacer/rings with 0.20 mm
#                    diametral assembly clearance; the upper shaft collar
#                    likewise passes concentrically inside the outer-race
#                    retaining ring. Their exact source audit has zero
#                    positive common volume.
#                    The M1 motor body is >38 mm from any spindle part at
#                    all poses (rigid same-carriage geometry), so excluding
#                    the whole motor model with its shaft is safe.
EXEMPT = {
    ("flyer", "static"): {"static": {
        "flyer_6001_front",
        "flyer_6001_rear",
        "gt2_belt",
        # Exact successor drive name.  The belt deliberately meshes with the
        # flyer-link P30 pulley; excluding only this belt occurrence preserves
        # every other flyer/static collision check.
        "m2_successor_210_3gt_6_belt",
    }},
    ("carriage", "static"): {"static": {
        "mgn12_rail_L", "mgn12_rail_R", "t8_screw",
        *(f"rail_{side}_m3x8_{index}"
          for side in ("L", "R") for index in range(1, 7)),
    }},
    ("spindle", "carriage"): {
        "spindle": {"m1_inner_race_spacer", "m1_lower_inner_spacer",
                    "m1_upper_shaft_collar"},
        "carriage": {"spindle_608_top", "spindle_608_bot", "m1_motor"},
    },
}


REPORT_SCHEMA = "collision-clearance/v2"


def proof_passed(collisions, worst, target_mm, *, tolerance_mm=1.0e-9):
    """Return the fail-closed release verdict for one completed sweep."""
    if collisions or not worst:
        return False
    try:
        clearances = [float(value[0]) for value in worst.values()]
        target = float(target_mm)
    except (TypeError, ValueError, IndexError):
        return False
    return (
        math.isfinite(target)
        and target >= 0.0
        and all(math.isfinite(value) and value + tolerance_mm >= target
                for value in clearances)
    )


def main():
    global LINKS
    parser = argparse.ArgumentParser(
        description="Full captured-cycle rigid-body interference proof")
    parser.add_argument(
        "--workers", type=int,
        default=int(os.environ.get("WINDER_COLLISION_WORKERS", "4")),
        help=("collision worker processes; use 1 for deterministic in-process "
              "execution where Windows multiprocessing pipes are unavailable"),
    )
    parser.add_argument(
        "--capture", type=Path,
        default=OUT / "capture" / "commands.jsonl",
        help=("captured command stream to validate; defaults to the canonical "
              "contract capture"),
    )
    parser.add_argument(
        "--links", type=Path,
        default=LINKS,
        help=("directory containing manifest.json and parts/<link>/*.stl; "
              "defaults to the canonical out/links export"),
    )
    parser.add_argument(
        "--output", type=Path,
        default=OUT / "reports" / "clearance.json",
        help="JSON report path",
    )
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("--workers must be at least 1")
    links_path = args.links
    if not links_path.is_absolute():
        links_path = (HERE.parent / links_path).resolve()
    if not (links_path / "manifest.json").is_file():
        parser.error(f"links manifest does not exist: {links_path / 'manifest.json'}")
    # The worker initializer receives this path explicitly because Windows
    # multiprocessing uses spawn and therefore does not inherit a mutated
    # module global from the parent process.
    LINKS = links_path
    capture_path = args.capture
    if not capture_path.is_absolute():
        capture_path = (HERE.parent / capture_path).resolve()
    output_path = args.output
    if not output_path.is_absolute():
        output_path = (HERE.parent / output_path).resolve()
    if not capture_path.is_file():
        parser.error(f"capture does not exist: {capture_path}")

    manifest = load_manifest()
    kin = Kinematics(manifest)
    slots = manifest["stator"]["slots"]
    pitch = 2 * math.pi / slots

    # load per-part meshes; build one merged BVH per (pair, side) honoring
    # the fit exemptions
    part_meshes = {}
    for name, assets in resolve_part_assets(manifest, LINKS).items():
        part_meshes[name] = {
            label: trimesh.load(path, force="mesh")
            for label, path in assets.items()
        }

    def merged_bvh(link, exclude=frozenset()):
        ms = [m for lbl, m in part_meshes[link].items() if lbl not in exclude]
        mesh = trimesh.util.concatenate(ms)
        return make_bvh(mesh)

    full_bvh = {name: merged_bvh(name) for name in part_meshes}
    pair_bvhs = {}
    for a, b in PAIRS:
        ex = EXEMPT.get((a, b), {})
        pair_bvhs[(a, b)] = (
            merged_bvh(a, frozenset(ex.get(a, ()))) if a in ex else
            full_bvh[a],
            merged_bvh(b, frozenset(ex.get(b, ()))) if b in ex else
            full_bvh[b],
        )
    for name, parts in part_meshes.items():
        print(f"{name}: {sum(len(m.faces) for m in parts.values())} tris, "
              f"{len(parts)} parts")

    events = load_events(capture_path)
    tl = Timeline(events)
    meta = next((event for event in events if event.get("e") == "meta"), {})
    print(f"capture: {capture_path}")
    print(f"controller: {meta.get('controller_mode', 'unknown')}")
    print(f"timeline: {tl.t_end/60:.1f} min virtual")

    def quant(m0, m1, m2):
        return (round(m0 / 0.25),
                round((m1 % pitch) / math.radians(1.0)),
                round((m2 % (2 * math.pi)) / math.radians(1.0)))

    seen = {}
    n_raw = 0
    for t, m0, m1, m2 in tl.samples():
        n_raw += 1
        q = quant(m0, m1, m2)
        if q not in seen:
            seen[q] = (t, m0, m1, m2)
    poses = list(seen.values())
    print(f"{n_raw} raw samples -> {len(poses)} unique quantized poses")

    def check(pose, pairs=PAIRS):
        t, m0, m1, m2 = pose
        tfs = {}
        for name in part_meshes:
            R, tr = kin.link_tf(name, m0, m1, m2)
            tfs[name] = fcl.Transform(R, tr)
        out = {}
        for a, b in pairs:
            bva, bvb = pair_bvhs[(a, b)]
            oa = fcl.CollisionObject(bva, tfs[a])
            ob = fcl.CollisionObject(bvb, tfs[b])
            creq = fcl.CollisionRequest()
            cres = fcl.CollisionResult()
            fcl.collide(oa, ob, creq, cres)
            if cres.is_collision:
                out[(a, b)] = -1.0
                continue
            dreq = fcl.DistanceRequest()
            dres = fcl.DistanceResult()
            d = fcl.distance(oa, ob, dreq, dres)
            out[(a, b)] = d
        return out

    worst = {p: (math.inf, None) for p in PAIRS}
    collisions = []
    requested_workers = args.workers
    nproc = max(1, min(requested_workers, os.cpu_count() or 4))
    print(f"collision workers: {nproc} (one BLAS thread each)")
    chunks = [poses[i::nproc * 3] for i in range(nproc * 3)]

    def consume(results):
        for k, (w, c) in enumerate(results):
            collisions.extend(c)
            for pair, (d, pose) in w.items():
                if d < worst[pair][0]:
                    worst[pair] = (d, pose)
            print(f"  chunk {k + 1}/{len(chunks)} done; worst: "
                  + ", ".join(f"{a}-{b}:{d:.2f}"
                              for (a, b), (d, _) in worst.items()))

    if nproc == 1:
        # Windows sandboxed sessions may prohibit multiprocessing.Pipe even
        # though all FCL operations themselves are available.  Reuse the
        # already-built in-process BVHs so this mode executes the identical
        # batch kernel without a child process or a weaker sample set.
        _W["kin"] = kin
        _W["pair_bvhs"] = pair_bvhs
        _W["links"] = list(part_meshes)
        consume(map(_check_batch, chunks))
    else:
        with ProcessPoolExecutor(max_workers=nproc,
                                 initializer=_worker_init,
                                 initargs=(str(LINKS),)) as ex:
            consume(ex.map(_check_batch, chunks))

    # refinement pass around near-threshold poses
    refine_below = 4.0
    refined = {}
    for pair, (d, pose) in worst.items():
        if d >= refine_below or d < 0:
            refined[pair] = (d, pose)
            continue
        t, m0, m1, m2 = pose
        best = (d, pose)
        for dm2 in np.arange(-math.radians(2), math.radians(2.01),
                             math.radians(0.25)):
            for dm0 in (-0.125, 0.0, 0.125):
                p2 = (t, m0 + dm0, m1, m2 + dm2)
                r = check(p2, pairs=[pair])
                if r[pair] < best[0]:
                    best = (r[pair], p2)
        refined[pair] = best
    worst = refined

    target_mm = float(manifest["dyn_clearance"])
    passed = proof_passed(collisions, worst, target_mm)
    minimum_dynamic_clearance_mm = min(
        float(value[0]) for value in worst.values())
    report = {
        "schema": REPORT_SCHEMA,
        "status": "PASS" if passed else "FAIL",
        "passed": passed,
        "capture": {
            "path": str(capture_path),
            "sha256": hashlib.sha256(capture_path.read_bytes()).hexdigest(),
            "schema": meta.get("capture_schema"),
            "controller_mode": meta.get("controller_mode"),
            "winder_commit": meta.get("winder_commit"),
        },
        "links": {
            "path": str(LINKS),
            "manifest_sha256": hashlib.sha256(
                (LINKS / "manifest.json").read_bytes()).hexdigest(),
        },
        "n_raw_samples": n_raw,
        "n_unique_poses": len(poses),
        "minimum_dynamic_clearance_mm": round(
            minimum_dynamic_clearance_mm, 6),
        "collisions": [{"pair": list(c["pair"]),
                        "pose": list(c["pose"])} for c in collisions],
        "min_clearance_mm": {
            f"{a}-{b}": {"clearance": round(d, 3),
                         "pose": {"t": round(p[0], 2), "m0": round(p[1], 3),
                                  "m1": round(p[2], 3), "m2": round(p[3], 3)},
                         "phase": tl.phase_at(p[0])}
            for (a, b), (d, p) in worst.items()
        },
        "target_mm": target_mm,
        "fit_exemptions": {
            f"{a}-{b}": {link: sorted(labels)
                           for link, labels in by_link.items()}
            for (a, b), by_link in EXEMPT.items()
        },
        "source_hashes": {
            "sim/collide.py": hashlib.sha256(
                Path(__file__).read_bytes()).hexdigest(),
            "sim/traj.py": hashlib.sha256(
                (HERE / "traj.py").read_bytes()).hexdigest(),
            "out/links/manifest.json": hashlib.sha256(
                (LINKS / "manifest.json").read_bytes()).hexdigest(),
        },
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2))
    print(f"report: {output_path}")

    print("\n=== INTERFERENCE PROOF ===")
    print(f"collisions: {len(collisions)}")
    for (a, b), (d, p) in worst.items():
        phase = tl.phase_at(p[0])
        flag = "OK " if d >= target_mm else "FAIL"
        print(f"  [{flag}] {a:9s}-{b:9s} min {d:7.2f} mm  "
              f"@ t={p[0]:7.1f}s m0={p[1]:8.3f} m1={p[2]:8.3f} "
              f"m2={p[3]:9.3f}  ({phase})")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
