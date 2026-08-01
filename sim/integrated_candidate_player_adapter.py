"""Render the isolated integrated-candidate links with the existing player.

The canonical animator intentionally reads ``out/links``.  This wrapper changes
its process-local asset root and material contract only while rendering, then
restores every global.  It never stages, copies, or replaces canonical assets.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Iterator, Mapping

import animate


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DEFAULT_ADAPTER_ROOT = ROOT / "out" / "review" / "integrated_adapter"
EXPECTED_SCHEMA = "integrated-candidate-player-adapter/v1"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_manifest(adapter_root: Path | str = DEFAULT_ADAPTER_ROOT) -> dict[str, Any]:
    root = Path(adapter_root).resolve()
    path = root / "links" / "manifest.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("schema") != EXPECTED_SCHEMA:
        raise ValueError("integrated candidate player manifest schema drift")
    if value.get("production_authorized") is not False:
        raise ValueError("integrated candidate player assets must stay review-only")
    if value.get("canonical_promotion_authorized") is not False:
        raise ValueError("integrated candidate manifest cannot authorize promotion")
    if value.get("kinematic_equivalence", {}).get("status") != "PASS":
        raise ValueError("integrated candidate transform proof is not PASS")
    links = value.get("links")
    if not isinstance(links, Mapping) or set(links) != {
        "static", "carriage", "spindle", "flyer"
    }:
        raise ValueError("integrated candidate four-link contract drift")
    for section in ("links", "visual_groups", "wire_assets"):
        records = value.get(section)
        if not isinstance(records, Mapping):
            raise ValueError(f"manifest {section} is missing")
        for name, record in records.items():
            if not isinstance(record, Mapping):
                raise ValueError(f"manifest {section}.{name} is malformed")
            file_name = record.get("file")
            if not isinstance(file_name, str) or Path(file_name).name != file_name:
                raise ValueError(f"manifest {section}.{name} has unsafe file")
            asset = root / "links" / file_name
            if not asset.is_file() or _sha256(asset) != record.get("sha256"):
                raise ValueError(f"manifest {section}.{name} asset/hash mismatch")
    wires = value["wire_assets"]
    for owner in ("static", "flyer"):
        expected = f"wire_{owner}.stl"
        if wires.get(owner, {}).get("file") != expected:
            raise ValueError(f"player requires {expected}")
    return value


def _material_contract(
    manifest: Mapping[str, Any],
) -> tuple[dict[str, list[float]], dict[str, dict[str, Any]], dict[str, Any]]:
    materials = manifest.get("materials")
    if not isinstance(materials, Mapping):
        raise ValueError("integrated candidate material catalog is missing")
    colors: dict[str, list[float]] = {}
    properties: dict[str, dict[str, Any]] = {}

    def add(render_name: str, material_key: str) -> None:
        material = materials.get(material_key)
        if not isinstance(material, Mapping):
            raise ValueError(f"material {material_key!r} is missing")
        color = material.get("color_rgba")
        if not isinstance(color, list) or len(color) != 4:
            raise ValueError(f"material {material_key!r} color is malformed")
        colors[render_name] = [float(value) for value in color]
        properties[render_name] = {
            "metallic": float(material["metallic"]),
            "roughness": float(material["roughness"]),
            "double_sided": bool(material["double_sided"]),
        }

    for link, record in manifest["links"].items():
        add(link, record["material"])
    add("wire_static", manifest["wire_assets"]["static"]["material"])
    add("wire_flyer", manifest["wire_assets"]["flyer"]["material"])

    required_groups: dict[str, Any] = {}
    for name, record in manifest["visual_groups"].items():
        link = record.get("link")
        if link not in {"static", "carriage", "spindle", "flyer"}:
            raise ValueError(f"visual group {name!r} has invalid link {link!r}")
        labels = record.get("labels")
        if not isinstance(labels, list) or not labels:
            raise ValueError(f"visual group {name!r} has no exact label set")
        add(name, record["material"])
        required_groups[name] = {"link": link, "labels": set(labels)}
    return colors, properties, required_groups


@contextmanager
def configured_animate(
    adapter_root: Path | str = DEFAULT_ADAPTER_ROOT,
) -> Iterator[dict[str, Any]]:
    """Temporarily point ``animate`` at isolated assets, then fully restore it."""

    root = Path(adapter_root).resolve()
    manifest = load_manifest(root)
    colors, properties, required_groups = _material_contract(manifest)
    previous = {
        "OUT": animate.OUT,
        "COLORS": animate.COLORS,
        "MATERIAL_PROPERTIES": animate.MATERIAL_PROPERTIES,
        "REQUIRED_VISUAL_GROUPS": animate.REQUIRED_VISUAL_GROUPS,
    }
    animate.OUT = root
    animate.COLORS = colors
    animate.MATERIAL_PROPERTIES = properties
    animate.REQUIRED_VISUAL_GROUPS = required_groups
    try:
        yield manifest
    finally:
        animate.OUT = previous["OUT"]
        animate.COLORS = previous["COLORS"]
        animate.MATERIAL_PROPERTIES = previous["MATERIAL_PROPERTIES"]
        animate.REQUIRED_VISUAL_GROUPS = previous["REQUIRED_VISUAL_GROUPS"]


def render_player(
    adapter_root: Path | str = DEFAULT_ADAPTER_ROOT,
    *,
    capture: Path | str = animate.DEFAULT_CAPTURE,
    conductor_route: Path | str | None = None,
    output: Path | str | None = None,
    html: Path | str | None = None,
    speed: float = 10.0,
    no_html: bool = False,
) -> dict[str, Any]:
    """Generate an isolated review GLB/player using unchanged animator code."""

    root = Path(adapter_root).resolve()
    output_path = Path(output).resolve() if output else (
        root / "winding_cycle_integrated_candidate_raw.glb"
    )
    html_path = Path(html).resolve() if html else (
        root / "play_integrated_candidate_raw.html"
    )
    capture_path = Path(capture).resolve()
    if not capture_path.is_file():
        raise ValueError(f"capture does not exist: {capture_path}")
    conductor_route_path = Path(
        conductor_route if conductor_route is not None
        else root / "reports" / "continuous_conductor_route.json"
    ).resolve()
    if not conductor_route_path.is_file():
        raise ValueError(
            f"conductor route does not exist: {conductor_route_path}"
        )
    if speed <= 0.0:
        raise ValueError("speed must be greater than zero")

    argv = [
        "animate.py",
        "--capture", str(capture_path),
        "--speed", str(float(speed)),
        "--output", str(output_path),
        "--html", str(html_path),
    ]
    argv.extend(["--conductor-route", str(conductor_route_path)])
    if no_html:
        argv.append("--no-html")
    previous_argv = sys.argv
    try:
        with configured_animate(root) as manifest:
            sys.argv = argv
            animate.main()
    finally:
        sys.argv = previous_argv

    result = {
        "schema": "integrated-candidate-player-render/v1",
        "review_only": True,
        "adapter_manifest": str(root / "links" / "manifest.json"),
        "adapter_contract_sha256": manifest["contract_sha256"],
        "capture": str(capture_path),
        "capture_sha256": _sha256(capture_path),
        "conductor_route": str(conductor_route_path),
        "conductor_route_artifact_sha256": _sha256(conductor_route_path),
        "glb": str(output_path),
        "glb_sha256": _sha256(output_path),
        "html": None if no_html else str(html_path),
        "html_sha256": None if no_html else _sha256(html_path),
        "canonical_assets_modified": False,
    }
    report_path = root / "player_render.json"
    report_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--adapter-root", type=Path, default=DEFAULT_ADAPTER_ROOT)
    parser.add_argument("--capture", type=Path, default=animate.DEFAULT_CAPTURE)
    parser.add_argument("--conductor-route", type=Path)
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--html", type=Path)
    parser.add_argument("--no-html", action="store_true")
    args = parser.parse_args()
    result = render_player(
        args.adapter_root,
        capture=args.capture,
        conductor_route=args.conductor_route,
        output=args.output,
        html=args.html,
        speed=args.speed,
        no_html=args.no_html,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
