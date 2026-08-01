"""Process-isolation tests for the integrated-candidate player wrapper."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import animate
import continuous_conductor_route as route_contract
import integrated_candidate_player_adapter as player


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class IntegratedCandidatePlayerAdapterTests(unittest.TestCase):
    def _fixture(self, root: Path) -> dict:
        links_dir = root / "links"
        links_dir.mkdir(parents=True)
        materials = {
            "base": {
                "color_rgba": [0.1, 0.2, 0.3, 1.0],
                "metallic": 0.0,
                "roughness": 0.5,
                "double_sided": False,
            },
            "copper": {
                "color_rgba": [0.8, 0.3, 0.1, 1.0],
                "metallic": 0.1,
                "roughness": 0.4,
                "double_sided": True,
            },
            "felt": {
                "color_rgba": [0.2, 0.05, 0.02, 1.0],
                "metallic": 0.0,
                "roughness": 1.0,
                "double_sided": True,
            },
        }

        def asset(name: str) -> dict:
            path = links_dir / name
            path.write_bytes(name.encode("ascii"))
            return {"file": name, "sha256": _sha(path)}

        manifest = {
            "schema": player.EXPECTED_SCHEMA,
            "production_authorized": False,
            "canonical_promotion_authorized": False,
            "contract_sha256": "fixture",
            "kinematic_equivalence": {"status": "PASS"},
            "materials": materials,
            "links": {
                name: {**asset(f"{name}.stl"), "material": "base"}
                for name in ("static", "carriage", "spindle", "flyer")
            },
            "wire_assets": {
                owner: {
                    **asset(f"wire_{owner}.stl"),
                    "material": "copper",
                }
                for owner in ("static", "flyer")
            },
            "visual_groups": {
                "felt_pads": {
                    **asset("visual_felt_pads.stl"),
                    "link": "static",
                    "material": "felt",
                    "labels": ["felt_pad_fixed", "felt_pad_moving"],
                }
            },
        }
        (links_dir / "manifest.json").write_text(
            json.dumps(manifest), encoding="utf-8"
        )
        return manifest

    def test_context_switches_asset_and_material_contract_then_restores(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._fixture(root)
            original_out = animate.OUT
            original_colors = animate.COLORS
            original_properties = animate.MATERIAL_PROPERTIES
            original_groups = animate.REQUIRED_VISUAL_GROUPS
            with player.configured_animate(root) as manifest:
                self.assertEqual(animate.OUT, root.resolve())
                self.assertEqual(manifest["schema"], player.EXPECTED_SCHEMA)
                self.assertEqual(
                    animate.REQUIRED_VISUAL_GROUPS["felt_pads"]["labels"],
                    {"felt_pad_fixed", "felt_pad_moving"},
                )
                self.assertEqual(animate.COLORS["felt_pads"], [0.2, 0.05, 0.02, 1.0])
            self.assertIs(animate.OUT, original_out)
            self.assertIs(animate.COLORS, original_colors)
            self.assertIs(animate.MATERIAL_PROPERTIES, original_properties)
            self.assertIs(animate.REQUIRED_VISUAL_GROUPS, original_groups)

    def test_context_restores_globals_after_exception(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._fixture(root)
            original = (
                animate.OUT,
                animate.COLORS,
                animate.MATERIAL_PROPERTIES,
                animate.REQUIRED_VISUAL_GROUPS,
            )
            with self.assertRaisesRegex(RuntimeError, "intentional"):
                with player.configured_animate(root):
                    raise RuntimeError("intentional")
            self.assertEqual(animate.OUT, original[0])
            self.assertIs(animate.COLORS, original[1])
            self.assertIs(animate.MATERIAL_PROPERTIES, original[2])
            self.assertIs(animate.REQUIRED_VISUAL_GROUPS, original[3])

    def test_manifest_rejects_asset_hash_drift(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._fixture(root)
            (root / "links" / "flyer.stl").write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "asset/hash mismatch"):
                player.load_manifest(root)

    def test_visual_group_must_have_a_kinematic_owner(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = self._fixture(root)
            broken = copy.deepcopy(manifest)
            broken["visual_groups"]["felt_pads"]["link"] = "worldish"
            (root / "links" / "manifest.json").write_text(
                json.dumps(broken), encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "invalid link"):
                with player.configured_animate(root):
                    pass

    def test_explicit_route_reaches_animate_and_all_process_state_restores(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._fixture(root)
            route = root / "proved-successor-route.json"
            route.write_text('{"fixture": true}', encoding="utf-8")
            output = root / "review.glb"
            html = root / "review.html"
            original_argv = sys.argv
            original = (
                animate.OUT,
                animate.COLORS,
                animate.MATERIAL_PROPERTIES,
                animate.REQUIRED_VISUAL_GROUPS,
            )
            observed: dict[str, object] = {}

            def fake_main() -> None:
                observed["argv"] = list(sys.argv)
                observed["out"] = animate.OUT
                observed["colors"] = animate.COLORS
                output.write_bytes(b"fixture glb")
                html.write_text("fixture html", encoding="utf-8")

            with mock.patch.object(animate, "main", side_effect=fake_main):
                result = player.render_player(
                    root,
                    conductor_route=route,
                    output=output,
                    html=html,
                )

            argv = observed["argv"]
            self.assertEqual(
                argv[argv.index("--conductor-route") + 1],
                str(route.resolve()),
            )
            self.assertEqual(observed["out"], root.resolve())
            self.assertIsNot(observed["colors"], original[1])
            self.assertIs(sys.argv, original_argv)
            self.assertIs(animate.OUT, original[0])
            self.assertIs(animate.COLORS, original[1])
            self.assertIs(animate.MATERIAL_PROPERTIES, original[2])
            self.assertIs(animate.REQUIRED_VISUAL_GROUPS, original[3])
            self.assertEqual(result["conductor_route"], str(route.resolve()))
            self.assertEqual(
                result["conductor_route_artifact_sha256"], _sha(route),
            )

    def test_route_render_exception_restores_argv_and_animate_globals(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._fixture(root)
            route = root / "route.json"
            route.write_text("{}", encoding="utf-8")
            original_argv = sys.argv
            original = (
                animate.OUT,
                animate.COLORS,
                animate.MATERIAL_PROPERTIES,
                animate.REQUIRED_VISUAL_GROUPS,
            )
            with mock.patch.object(
                animate, "main", side_effect=RuntimeError("render failed")
            ):
                with self.assertRaisesRegex(RuntimeError, "render failed"):
                    player.render_player(root, conductor_route=route)
            self.assertIs(sys.argv, original_argv)
            self.assertIs(animate.OUT, original[0])
            self.assertIs(animate.COLORS, original[1])
            self.assertIs(animate.MATERIAL_PROPERTIES, original[2])
            self.assertIs(animate.REQUIRED_VISUAL_GROUPS, original[3])

    def test_default_route_is_the_adapter_manifest_bound_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._fixture(root)
            route = root / "reports" / "continuous_conductor_route.json"
            route.parent.mkdir(parents=True)
            route.write_text('{"fixture": true}', encoding="utf-8")
            output = root / "review.glb"
            observed: dict[str, object] = {}

            def fake_main() -> None:
                observed["argv"] = list(sys.argv)
                output.write_bytes(b"fixture glb")

            with mock.patch.object(animate, "main", side_effect=fake_main):
                result = player.render_player(
                    root, output=output, no_html=True,
                )
            self.assertEqual(
                observed["argv"][
                    observed["argv"].index("--conductor-route") + 1
                ],
                str(route.resolve()),
            )
            self.assertEqual(result["conductor_route"], str(route.resolve()))
            self.assertEqual(
                result["conductor_route_artifact_sha256"], _sha(route)
            )

    def test_missing_default_adapter_route_fails_before_render(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._fixture(root)
            with mock.patch.object(animate, "main") as animate_main:
                with self.assertRaisesRegex(
                    ValueError, "continuous_conductor_route.json"
                ):
                    player.render_player(root)
            animate_main.assert_not_called()

    def test_missing_explicit_route_fails_before_mutating_process_state(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._fixture(root)
            original_argv = sys.argv
            original_out = animate.OUT
            missing = root / "missing-route.json"
            with mock.patch.object(animate, "main") as animate_main:
                with self.assertRaisesRegex(
                    ValueError, "conductor route does not exist"
                ):
                    player.render_player(root, conductor_route=missing)
            animate_main.assert_not_called()
            self.assertIs(sys.argv, original_argv)
            self.assertIs(animate.OUT, original_out)

    def test_wrong_capture_bound_route_fails_closed(self) -> None:
        """A different JSON path cannot weaken the raw-capture hash bind."""

        expected = {
            "raw_capture_sha256": "0" * 64,
            "slot_winding_plan_sha256": _sha(route_contract.PLAN),
            "cad_manifest_sha256": _sha(route_contract.MANIFEST),
            "generator_source_sha256": _sha(Path(route_contract.__file__)),
            "traj_source_sha256": _sha(
                Path(route_contract.__file__).parent / "traj.py"
            ),
        }
        report = {
            "schema": route_contract.SCHEMA,
            "source_hashes": expected,
        }
        report["report_sha256"] = route_contract._canonical_hash(report)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "wrong-capture-route.json"
            path.write_text(json.dumps(report), encoding="utf-8")
            with self.assertRaisesRegex(
                ValueError, "stale sources: raw_capture_sha256"
            ):
                animate._load_continuous_conductor_route(
                    path,
                    capture_path=route_contract.CAPTURE,
                    plan_path=route_contract.PLAN,
                    manifest_path=route_contract.MANIFEST,
                )


if __name__ == "__main__":
    unittest.main()
