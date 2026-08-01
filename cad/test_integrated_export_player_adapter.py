"""Regression gates for the isolated integrated export/player adapter."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

from build123d import Align, Cylinder

import integrated_export_player_adapter as adapter


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)


class IntegratedAdapterReleaseIdentityTests(unittest.TestCase):
    def test_release_identity_uses_documented_order_and_final_lf(self) -> None:
        self.assertEqual(
            adapter.RELEASE_CLOSURE_INPUTS,
            (
                "cad/integrated_export_player_adapter.py",
                "cad/integrated_release_candidate.py",
                "cad/assembly.py",
                "cad/collision_mesh_integrity.py",
                "out/reports/integrated_release_candidate.json",
                "out/reports/carriage_active_sector_terminal_guide_loci.json",
                "sim/animate.py",
                "sim/player_template.html",
            ),
        )
        fake = {
            relative: f"{index:064x}"
            for index, relative in enumerate(
                adapter.RELEASE_CLOSURE_INPUTS, start=1
            )
        }
        expected_payload = "".join(
            f"{relative}={fake[relative]}\n"
            for relative in adapter.RELEASE_CLOSURE_INPUTS
        )
        expected = hashlib.sha256(
            expected_payload.encode("utf-8")
        ).hexdigest()
        identity = adapter.release_closure_identity(dict(reversed(fake.items())))
        self.assertEqual(identity["closure_sha256"], expected)
        self.assertEqual(identity["release_id"], f"iar1-{expected[:20]}")
        self.assertEqual(
            tuple(identity["source_hashes"]), adapter.RELEASE_CLOSURE_INPUTS
        )

    def test_release_identity_rejects_source_set_or_hash_drift(self) -> None:
        fake = {
            relative: "a" * 64 for relative in adapter.RELEASE_CLOSURE_INPUTS
        }
        missing = dict(fake)
        missing.pop(adapter.RELEASE_CLOSURE_INPUTS[-1])
        with self.assertRaisesRegex(ValueError, "source set drift"):
            adapter.release_closure_identity(missing)
        with self.assertRaisesRegex(ValueError, "source set drift"):
            adapter.release_closure_identity({})
        invalid = dict(fake)
        invalid[adapter.RELEASE_CLOSURE_INPUTS[0]] = "not-a-sha"
        with self.assertRaisesRegex(ValueError, "invalid release closure hash"):
            adapter.release_closure_identity(invalid)

    def test_historical_release_vector_reproduces_f87eed_identity(self) -> None:
        historical = {
            "cad/integrated_export_player_adapter.py": (
                "99312baca9b139b62085d0abeef237a6432434f491bfcdeffd7e0ab97495c63a"
            ),
            "cad/integrated_release_candidate.py": (
                "0436fb9eeb6b1df11ebbb0a487accf6b58caf6e86f305105eca3ef8578729122"
            ),
            "cad/assembly.py": (
                "6bea37e623361d6970e5e455731e9310c95e2623ca092d9ed69e112b15329fa4"
            ),
            "cad/collision_mesh_integrity.py": (
                "f29686e29457d8b4d036b824b46e6a9f1fa9da450c4d6f704fece8601e276813"
            ),
            "out/reports/integrated_release_candidate.json": (
                "034f7347c5e12214b13ea3619b7919064cfcdfe9a6b54fa4d2797b23dcd16e90"
            ),
            "out/reports/carriage_active_sector_terminal_guide_loci.json": (
                "d4706344f05f81c8eae7f07d399bb007972b075038c0e44f8d42754c292295d1"
            ),
        }
        identity = adapter._release_closure_identity_for_inputs(
            historical, adapter.LEGACY_RELEASE_CLOSURE_INPUTS_V1
        )
        self.assertEqual(
            identity["closure_sha256"],
            "f87eed5259bbba55b3418abcba53fea4a6c2eddf6225a881b33a9ec225373376",
        )
        self.assertEqual(identity["release_id"], "iar1-f87eed5259bbba55b341")

    def test_release_output_refuses_selector_and_wrong_identity(self) -> None:
        identity = adapter.release_closure_identity()
        expected = adapter.RELEASE_ROOT / identity["release_id"]
        self.assertEqual(
            adapter._validated_release_output(None, identity),
            expected.resolve(),
        )
        with self.assertRaisesRegex(ValueError, "canonical review selector"):
            adapter._validated_release_output(adapter.DEFAULT_OUTPUT, identity)
        with self.assertRaisesRegex(ValueError, "does not match current identity"):
            adapter._validated_release_output(
                adapter.RELEASE_ROOT / "iar1-00000000000000000000",
                identity,
            )

    def test_cli_default_reuses_validated_identity_output_for_validation(self) -> None:
        identity = {
            "release_id": "iar1-1234567890abcdef1234",
            "closure_sha256": "1" * 64,
            "source_hashes": {},
            "payload_format": "test",
        }
        output = Path("C:/tmp/iar1-1234567890abcdef1234")
        manifest = {"schema": adapter.SCHEMA}
        with (
            mock.patch.object(sys, "argv", ["adapter.py"]),
            mock.patch.object(
                adapter, "release_closure_identity", return_value=identity
            ),
            mock.patch.object(
                adapter, "_validated_release_output", return_value=output
            ) as validate_output,
            mock.patch.object(
                adapter, "export_adapter", return_value=manifest
            ) as export,
            mock.patch.object(adapter, "validate_manifest") as validate,
            mock.patch("builtins.print"),
        ):
            adapter.main()
        validate_output.assert_called_once_with(None, identity)
        export.assert_called_once_with(output)
        validate.assert_called_once_with(manifest, output)


class IntegratedExportPlayerAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.links = adapter.build_adapter_links()

    def test_four_candidate_links_and_required_material_groups(self) -> None:
        self.assertEqual(set(self.links), set(adapter.LINK_NAMES))
        groups = {}
        group_owners = {}
        for link, parts in self.links.items():
            base, selected = adapter.split_visual_parts(link, parts)
            self.assertTrue(base)
            groups.update(selected)
            group_owners.update({name: link for name in selected})
        expected = {
            "felt_pads",
            "m2_drive_belt",
            "m2_motor_pulley",
            "m2_flyer_pulley",
            "peek_caps",
            "cap_retention_hardware",
            "counterweight_tungsten",
            "counterweight_retainers",
            "counterweight_retention_hardware",
            "flyer_peek_guide",
            "flyer_peek_guide_retention_hardware",
            "active_sector_peek_guides",
            "active_sector_yoke",
            "active_sector_retention_hardware",
        }
        self.assertTrue(expected.issubset(groups))
        self.assertEqual(len(groups["felt_pads"]["parts"]), 2)
        self.assertEqual(len(groups["m2_drive_belt"]["parts"]), 1)
        self.assertEqual(len(groups["m2_motor_pulley"]["parts"]), 1)
        self.assertEqual(len(groups["m2_flyer_pulley"]["parts"]), 1)
        self.assertEqual(groups["m2_drive_belt"]["material"], "belt_dark_rubber")
        self.assertEqual(groups["m2_motor_pulley"]["material"], "pulley_aluminum")
        self.assertEqual(groups["m2_flyer_pulley"]["material"], "pulley_aluminum")
        self.assertEqual(group_owners["m2_drive_belt"], "static")
        self.assertEqual(group_owners["m2_motor_pulley"], "static")
        self.assertEqual(group_owners["m2_flyer_pulley"], "flyer")
        self.assertEqual(len(groups["peek_caps"]["parts"]), 2)
        self.assertEqual(len(groups["cap_retention_hardware"]["parts"]), 12)
        self.assertEqual(len(groups["counterweight_tungsten"]["parts"]), 6)
        self.assertEqual(len(groups["counterweight_retainers"]["parts"]), 4)
        self.assertEqual(
            len(groups["counterweight_retention_hardware"]["parts"]), 14
        )
        self.assertEqual(len(groups["flyer_peek_guide"]["parts"]), 1)
        self.assertEqual(
            len(groups["flyer_peek_guide_retention_hardware"]["parts"]), 6
        )
        self.assertEqual(len(groups["active_sector_peek_guides"]["parts"]), 2)
        self.assertEqual(len(groups["active_sector_yoke"]["parts"]), 1)
        self.assertEqual(
            len(groups["active_sector_retention_hardware"]["parts"]), 24
        )
    def test_exact_terminal_loci_are_staged_byte_for_byte_and_fail_closed(self):
        segment = {
            "name": "fixture",
            "machine_world_samples_mm": [[0, 0, 0], [1, 0, 0], [1, 1, 0]],
        }
        payload = {
            "schema": adapter.ACTIVE_TERMINAL_LOCI_SCHEMA,
            "run": {"locus_count": adapter.EXPECTED_ACTIVE_TERMINAL_LOCI},
            "segment_contract": {
                "flyer_geometric_bore": {
                    "surface_owner": "flyer",
                    "local_frame": "flyer_reference_M2_axis_plus_Z",
                    "authority": "fixture shared flyer path",
                },
                "fixture": {
                    "surface_owner": "none",
                    "local_frame": "fixture",
                    "authority": "fixture per-locus path",
                },
            },
            "flyer_reference": {
                "frame": "flyer_reference_M2_axis_plus_Z",
                "full_geometric_bore_local_samples_mm": [
                    [0, 0, 0], [0, 1, 0], [0, 2, 0]
                ],
                "full_geometric_bore_point_count": 3,
                "conductor_prefix_point_count": 2,
                "geometric_bore_to_tensioned_handoff_local_samples_mm": [
                    [0, 0, 0], [0, 1, 0]
                ],
                "source_api": "fixture",
            },
            "loci": [
                {"locus_index": index, "segments": [segment]}
                for index in range(adapter.EXPECTED_ACTIVE_TERMINAL_LOCI)
            ],
        }
        payload["locus_payload_sha256"] = (
            adapter._active_terminal_payload_hash(payload)
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "source.json"
            source.write_text(json.dumps(payload), encoding="utf-8")
            output = root / "adapter"

            record = adapter.stage_active_terminal_loci(output, source)

            staged = output / record["file"]
            self.assertEqual(staged.read_bytes(), source.read_bytes())
            self.assertEqual(record["locus_count"], 2400)
            self.assertEqual(record["maximum_polyline_edges_per_locus"], 3)
            self.assertTrue(record["held_between_loci_for_review_only"])
            self.assertFalse(record["park_index_load_unload_proven"])
            self.assertFalse(record["sag_tension_settling_neatness_proven"])

    def test_wire_manifest_uses_final_one_piece_peek_guide_not_legacy_metadata(self):
        wire = adapter._wire_manifest()
        self.assertIn("active_terminal_guide", wire)
        self.assertNotIn("tip_guide", wire)
        self.assertNotIn("torus", json.dumps(wire).lower())
        guide = wire["active_terminal_guide"]
        self.assertEqual(guide["material"], "natural unfilled PEEK")
        self.assertEqual(
            guide["minimum_centerline_bend_radius_mm"],
            adapter.flyer_successor.GUIDE_CENTERLINE_RADIUS_MM,
        )
        self.assertIn("one-piece PEEK", wire["flyer"]["model"])
        self.assertNotIn("tip_guide_center", wire["flyer"]["landmarks"])
        seam = [
            0.0,
            0.0,
            adapter.flyer_successor.GUIDE_ROOT_AXIAL_START_Z_MM,
        ]
        self.assertEqual(wire["static"]["points"][-1], seam)
        self.assertEqual(wire["static"]["landmarks"]["guide_root"], seam)
        self.assertEqual(wire["flyer"]["points"][0], seam)
        self.assertEqual(
            wire["static"]["landmarks"]["shaft_bore_rear"],
            [0.0, 0.0, adapter.candidate.flyer_shaft_d10.WORLD_REAR_Z_MM],
        )
        self.assertAlmostEqual(
            wire["static"]["dancer"]["path_radius"],
            adapter.wire_geometry.DANCER_BODY_RADIUS + adapter.wire_vis.R_VIS,
            places=12,
        )
        self.assertNotEqual(
            wire["static"]["dancer"]["path_radius"],
            adapter.wire_geometry.DANCER_PATH_RADIUS,
        )
        handoff = wire["continuous_handoff"]
        self.assertEqual(handoff, adapter._wire_handoff_contract(wire))
        self.assertEqual(handoff["status"], "PASS")
        self.assertEqual(handoff["maximum_gap_mm"], 0.0)
        self.assertTrue(
            handoff["static_owner_continues_through_shaft_to_guide_root"]
        )
        self.assertTrue(
            handoff["static_to_flyer_handoff_is_M2_axis_invariant"]
        )
        self.assertFalse(wire["continuous_conductor_release_authorized"])

    def test_wire_and_active_locus_bore_binding_fails_closed_on_either_seam(self):
        wire = adapter._wire_manifest()
        full = wire["flyer"]["points"]
        payload = {
            "flyer_reference": {
                "full_geometric_bore_local_samples_mm": full,
                adapter.FLYER_REFERENCE_SAMPLES_FIELD: full[:-1],
            }
        }
        adapter._validate_wire_locus_binding(wire, payload)

        stale_static = adapter.deepcopy(wire)
        stale_static["static"]["points"][-1] = [0.0, 0.0, -100.0]
        with self.assertRaisesRegex(ValueError, "handoff gap"):
            adapter._wire_handoff_contract(stale_static)

        stale_locus = adapter.deepcopy(payload)
        stale_locus["flyer_reference"][
            "full_geometric_bore_local_samples_mm"
        ][0] = [0.0, 0.0, -100.0]
        with self.assertRaisesRegex(ValueError, "full bore differs"):
            adapter._validate_wire_locus_binding(wire, stale_locus)

    def test_transform_matrices_match_upstream_and_player_hierarchy(self) -> None:
        report = adapter.kinematic_equivalence_report()
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["sample_count"], 5)
        self.assertEqual(report["link_sample_count"], 20)
        self.assertLessEqual(
            report["candidate_vs_upstream_max_abs"],
            adapter.MATRIX_TOLERANCE,
        )
        self.assertLessEqual(
            report["candidate_vs_player_hierarchy_max_abs"],
            adapter.MATRIX_TOLERANCE,
        )
        self.assertAlmostEqual(report["mm_per_rad_m0"], adapter.P.mm_per_rad)

    def test_transform_proof_is_not_only_a_zero_pose_identity_check(self) -> None:
        samples = [(-3.25, 0.77, -1.13)]
        report = adapter.kinematic_equivalence_report(samples)
        self.assertEqual(report["status"], "PASS")
        spindle = adapter.player_hierarchy_matrix("spindle", *samples[0])
        flyer = adapter.player_hierarchy_matrix("flyer", *samples[0])
        self.assertFalse(math.isclose(spindle[0, 3], 0.0))
        self.assertFalse(math.isclose(flyer[0, 0], 1.0))

    def test_obsolete_tip_guide_override_is_rejected_by_final_candidate(self) -> None:
        tip = Cylinder(1.0, 2.0, align=CTR)
        tip.label = "future_unfilled_PEEK_guide"
        crown = Cylinder(3.0, 1.0, align=CTR)
        crown.label = "future_terminal_crown_unfilled_PEEK"
        overrides = adapter.AdapterOverrides.terminal_guides(
            flyer_tip_guide=tip,
            terminal_crown_parts=(crown,),
        )
        with self.assertRaisesRegex(
            ValueError, "superseded by the retained one-piece PEEK guide"
        ):
            adapter.build_adapter_links(overrides)

    def test_duplicate_or_unlabeled_additions_fail_closed(self) -> None:
        duplicate = Cylinder(1.0, 1.0, align=CTR)
        duplicate.label = adapter._label(self.links["spindle"][0])
        with self.assertRaisesRegex(ValueError, "duplicate labels"):
            adapter.build_adapter_links(
                adapter.AdapterOverrides(
                    link_additions={"spindle": (duplicate,)},
                    provenance_by_occurrence={
                        f"spindle/{duplicate.label}": (
                            adapter.mesh_integrity.PROVENANCE_MODELED
                        )
                    },
                )
            )
        unlabeled = Cylinder(1.0, 1.0, align=CTR)
        unlabeled.label = ""
        with self.assertRaisesRegex(ValueError, "unlabeled"):
            adapter.AdapterOverrides.terminal_guides(
                flyer_tip_guide=Cylinder(1.0, 1.0, align=CTR),
                terminal_crown_parts=(unlabeled,),
            )

    def test_export_refuses_canonical_links_directory_before_geometry_work(self) -> None:
        canonical = adapter.ROOT / "out" / "links"
        with self.assertRaisesRegex(ValueError, "refuses to write canonical"):
            adapter.export_adapter(canonical)

    def test_collision_manifest_contract_uses_explicit_safe_file_mapping(self) -> None:
        """Prove collide.py can resolve the adapter's mapped part filenames."""

        sim_dir = adapter.ROOT / "sim"
        if str(sim_dir) not in sys.path:
            sys.path.insert(0, str(sim_dir))
        import collide

        physical = Cylinder(1.0, 1.0, align=CTR)
        physical.label = "display+label"
        fake_links = {
            link: [physical if link == "static" else self.links[link][0]]
            for link in adapter.LINK_NAMES
        }
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            links_dir = root / "links"
            for link, parts in fake_links.items():
                (links_dir / "parts" / link).mkdir(parents=True)
            # Avoid meshing here; the contract under test is exact logical
            # label -> confined safe filename resolution.
            manifest = {"parts": {}}
            for link, parts in fake_links.items():
                by_label = {}
                for shape in parts:
                    label = adapter._label(shape)
                    filename = adapter._safe_name(label) + ".stl"
                    (links_dir / "parts" / link / filename).write_bytes(b"mesh")
                    by_label[label] = {"file": filename}
                manifest["parts"][link] = by_label
            resolved = collide.resolve_part_assets(manifest, links_dir)
            self.assertEqual(set(resolved), set(adapter.LINK_NAMES))
            self.assertEqual(
                resolved["static"]["display+label"].name,
                "display_label.stl",
            )
            for link, assets in resolved.items():
                parent = (links_dir / "parts" / link).resolve()
                self.assertTrue(all(parent in path.parents for path in assets.values()))

    def test_manifest_validation_checks_collision_asset_hashes(self) -> None:
        """A compact fake export exercises mandatory part/hash validation."""

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            links_dir = root / "links"
            links_dir.mkdir(parents=True)
            asset = links_dir / "static.stl"
            asset.write_bytes(b"aggregate")
            part = links_dir / "parts" / "static" / "part.stl"
            part.parent.mkdir(parents=True)
            part.write_bytes(b"part")
            manifest = {
                "schema": adapter.SCHEMA,
                "production_authorized": False,
                "canonical_promotion_authorized": False,
                "kinematic_equivalence": {"status": "PASS"},
                "links": {},
                "visual_groups": {},
                "wire_assets": {},
                "wire": adapter._wire_manifest(),
                "parts": {},
            }
            for link in adapter.LINK_NAMES:
                link_asset = links_dir / f"{link}.stl"
                link_asset.write_bytes(b"aggregate")
                link_part = links_dir / "parts" / link / "part.stl"
                link_part.parent.mkdir(parents=True, exist_ok=True)
                link_part.write_bytes(b"part")
                manifest["links"][link] = {
                    "file": link_asset.name,
                    "sha256": adapter._sha256(link_asset),
                }
                manifest["parts"][link] = {
                    "part": {
                        "file": "part.stl",
                        "sha256": adapter._sha256(link_part),
                        "source_visual_file": "part.stl",
                        "source_visual_sha256": adapter._sha256(link_part),
                        "provenance_class": (
                            adapter.mesh_integrity.PROVENANCE_MODELED
                        ),
                    }
                }
            manifest["contract_sha256"] = adapter._canonical_hash(manifest)
            adapter.validate_manifest(manifest, root)
            (links_dir / "parts" / "flyer" / "part.stl").write_bytes(b"drift")
            with self.assertRaisesRegex(ValueError, "collision asset hash mismatch"):
                adapter.validate_manifest(manifest, root)

class WireManifestHandoffTests(unittest.TestCase):
    """Fast source-level gates independent of generated collision reports."""

    def test_configured_manifest_runs_through_shaft_to_guide_root(self) -> None:
        wire = adapter._wire_manifest()
        seam = [
            0.0,
            0.0,
            adapter.flyer_successor.GUIDE_ROOT_AXIAL_START_Z_MM,
        ]
        self.assertEqual(wire["static"]["points"][-1], seam)
        self.assertEqual(wire["static"]["landmarks"]["guide_root"], seam)
        self.assertEqual(wire["flyer"]["points"][0], seam)
        self.assertEqual(
            wire["static"]["landmarks"]["shaft_bore_rear"],
            [0.0, 0.0, adapter.candidate.flyer_shaft_d10.WORLD_REAR_Z_MM],
        )
        self.assertAlmostEqual(
            wire["static"]["dancer"]["path_radius"],
            adapter.wire_geometry.DANCER_BODY_RADIUS + adapter.wire_vis.R_VIS,
            places=12,
        )
        handoff = wire["continuous_handoff"]
        self.assertEqual(handoff, adapter._wire_handoff_contract(wire))
        self.assertEqual(handoff["maximum_gap_mm"], 0.0)
        self.assertFalse(handoff["unsupported_flexible_intervals_authorized"])

    def test_old_endpoint_and_locus_bore_drift_both_fail_closed(self) -> None:
        wire = adapter._wire_manifest()
        full = wire["flyer"]["points"]
        payload = {
            "flyer_reference": {
                "full_geometric_bore_local_samples_mm": full,
                adapter.FLYER_REFERENCE_SAMPLES_FIELD: full[:-1],
            }
        }
        adapter._validate_wire_locus_binding(wire, payload)

        stale_static = adapter.deepcopy(wire)
        stale_static["static"]["points"][-1] = [0.0, 0.0, -100.0]
        with self.assertRaisesRegex(ValueError, "handoff gap"):
            adapter._wire_handoff_contract(stale_static)

        stale_locus = adapter.deepcopy(payload)
        stale_locus["flyer_reference"][
            "full_geometric_bore_local_samples_mm"
        ][0] = [0.0, 0.0, -100.0]
        with self.assertRaisesRegex(ValueError, "full bore differs"):
            adapter._validate_wire_locus_binding(wire, stale_locus)


class CollisionProvenancePolicyTests(unittest.TestCase):
    def test_future_active_sector_and_guide_overrides_are_strict_custom(self) -> None:
        shape = Cylinder(1.0, 1.0, align=CTR)
        shape.label = "future_active_sector_PEEK_terminal_guide"
        missing = adapter.AdapterOverrides(
            link_additions={"spindle": (shape,)}
        )
        with self.assertRaisesRegex(ValueError, "lack explicit provenance"):
            adapter.validate_override_provenance(missing)
        wrong = adapter.AdapterOverrides(
            link_additions={"spindle": (shape,)},
            provenance_by_occurrence={
                f"spindle/{shape.label}": adapter.mesh_integrity.PROVENANCE_MODELED
            },
        )
        with self.assertRaisesRegex(ValueError, "strict custom provenance"):
            adapter.validate_override_provenance(wrong)
        strict = adapter.AdapterOverrides(
            link_additions={"spindle": (shape,)},
            provenance_by_occurrence={
                f"spindle/{shape.label}": adapter.mesh_integrity.PROVENANCE_CUSTOM
            },
        )
        keys = adapter.validate_override_provenance(strict)
        provenance, _source = adapter.part_provenance(
            "spindle", shape.label, strict, override_keys=keys
        )
        self.assertEqual(provenance, adapter.mesh_integrity.PROVENANCE_CUSTOM)
        self.assertIsNone(
            adapter.collision_overbound_method("spindle", shape.label, provenance)
        )

    def test_overbound_allowlist_is_exactly_the_reviewed_current_four(self) -> None:
        self.assertEqual(
            adapter.COLLISION_OVERBOUND_ALLOWLIST,
            {
                ("static", "m2_Leadshine_CS-M21708_exact_cableless"),
                ("static", "felt_m4_wingnut"),
                ("carriage", "mgn12h_L"),
                ("carriage", "mgn12h_R"),
            },
        )
        for link, label in adapter.COLLISION_OVERBOUND_ALLOWLIST:
            provenance, _source = adapter.part_provenance(link, label)
            self.assertNotEqual(
                provenance, adapter.mesh_integrity.PROVENANCE_CUSTOM
            )
            self.assertEqual(
                adapter.collision_overbound_method(link, label, provenance),
                adapter.mesh_integrity.HULL_METHOD,
            )



class ReferenceGlbMaterialTests(unittest.TestCase):
    def test_reference_glb_uses_explicit_pbr_materials_and_normals(self) -> None:
        """Prevent the near-black lit-material fallback seen in CAD Viewer."""

        import numpy as np
        import trimesh
        from pygltflib import GLTF2

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            mesh_path = root / "triangle.stl"
            triangle = trimesh.Trimesh(
                vertices=np.asarray([[0, 0, 0], [1, 0, 0], [0, 1, 0]]),
                faces=np.asarray([[0, 1, 2]]),
                process=False,
            )
            triangle.export(mesh_path)
            manifest = {
                "links": {
                    link: {"file": mesh_path.name}
                    for link in adapter.LINK_NAMES
                },
                "wire_assets": {},
                "visual_groups": {},
            }
            output = root / "reference.glb"
            adapter._write_reference_glb(output, manifest, root)
            scene = trimesh.load(output, force="scene")
            material = scene.geometry["static"].visual.material
            self.assertEqual(type(material).__name__, "PBRMaterial")
            self.assertTrue(
                np.array_equal(material.main_color, [199, 204, 212, 255])
            )
            gltf = GLTF2().load(str(output))
            self.assertTrue(
                all(
                    mesh.primitives[0].attributes.NORMAL is not None
                    for mesh in gltf.meshes
                )
            )


class CollisionTwoPassPipelineTests(unittest.TestCase):
    """Compact real meshes exercise draft audit, substitution, and re-audit."""

    def _closed_box(self):
        import trimesh

        return trimesh.creation.box(extents=[2.0, 2.0, 2.0])

    def _open_box(self):
        mesh = self._closed_box()
        keep = list(range(len(mesh.faces) - 2))
        mesh.update_faces(keep)
        mesh.remove_unreferenced_vertices()
        return mesh

    def _fixture(self, output: Path) -> tuple[dict, Path]:
        links_dir = output / "links"
        links_dir.mkdir(parents=True)
        links = {}
        parts = {}
        labels = {
            "static": "m2_Leadshine_CS-M21708_exact_cableless",
            "carriage": "future_active_sector_PEEK_guide",
            "spindle": "modeled_spindle_fixture",
            "flyer": "printed_flyer_fixture",
        }
        provenance = {
            "static": adapter.mesh_integrity.PROVENANCE_IMPORTED,
            "carriage": adapter.mesh_integrity.PROVENANCE_CUSTOM,
            "spindle": adapter.mesh_integrity.PROVENANCE_MODELED,
            "flyer": adapter.mesh_integrity.PROVENANCE_CUSTOM,
        }
        source_visual = None
        for link in adapter.LINK_NAMES:
            aggregate_path = links_dir / f"{link}.stl"
            self._closed_box().export(aggregate_path)
            links[link] = {
                "file": aggregate_path.name,
                "sha256": adapter._sha256(aggregate_path),
                "material": adapter.LINK_MATERIALS[link],
            }
            part_dir = links_dir / "parts" / link
            part_dir.mkdir(parents=True)
            label = labels[link]
            part_path = part_dir / f"{adapter._safe_name(label)}.stl"
            mesh = self._open_box() if link == "static" else self._closed_box()
            mesh.export(part_path)
            part_hash = adapter._sha256(part_path)
            record = {
                "file": part_path.name,
                "sha256": part_hash,
                "source_visual_file": part_path.name,
                "source_visual_sha256": part_hash,
                "collision_role": "exact_source_visual_and_draft_collision_mesh",
                "provenance_class": provenance[link],
                "provenance_source": "compact_test_fixture",
            }
            if link == "static":
                record["collision_overbound_method"] = (
                    adapter.mesh_integrity.HULL_METHOD
                )
                source_visual = part_path
            parts[link] = {label: record}

        manifest = {
            "schema": adapter.SCHEMA,
            "status": "REVIEW_ASSETS_READY_RELEASE_GATES_OPEN",
            "production_authorized": False,
            "canonical_promotion_authorized": False,
            "kinematic_equivalence": {"status": "PASS"},
            "links": links,
            "visual_groups": {},
            "wire_assets": {},
            "wire": adapter._wire_manifest(),
            "parts": parts,
            "collision_manifest_contract": {
                "status": "DRAFT_PENDING_TWO_PASS_INTEGRITY_AUDIT"
            },
        }
        glb = output / "integrated_candidate_reference_pose.glb"
        adapter._write_reference_glb(glb, manifest, links_dir)
        manifest["reference_pose_glb"] = {
            "file": glb.name,
            "sha256": adapter._sha256(glb),
        }
        self.assertIsNotNone(source_visual)
        return manifest, source_visual

    def test_required_two_pass_binds_and_reaudits_effective_collision_assets(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            manifest, source_visual = self._fixture(output)
            source_hash = adapter._sha256(source_visual)
            result = adapter.run_collision_integrity_two_pass(manifest, output)
            final = result.manifest
            pipeline = final["collision_pipeline"]
            self.assertEqual(pipeline["phase"], "final_effective_mapping")
            self.assertEqual(pipeline["applicable_substitution_count"], 1)
            self.assertEqual(
                result.draft_report["summary"]["generated_overbound_count"], 1
            )
            self.assertEqual(
                result.final_report["summary"]["generated_overbound_count"], 0
            )
            self.assertEqual(result.final_report["status"], "PASS")
            record = final["parts"]["static"][
                "m2_Leadshine_CS-M21708_exact_cableless"
            ]
            self.assertNotEqual(record["file"], record["source_visual_file"])
            self.assertEqual(adapter._sha256(source_visual), source_hash)
            self.assertEqual(record["source_visual_sha256"], source_hash)
            binding = record["collision_overbound"]
            self.assertEqual(binding["status"], "PASS")
            self.assertEqual(
                adapter._canonical_hash(binding["overbound_proof"]),
                binding["overbound_proof_sha256"],
            )
            for by_label in final["parts"].values():
                for item in by_label.values():
                    self.assertIn(
                        item["provenance_class"],
                        adapter.mesh_integrity.VALID_PROVENANCE,
                    )
            adapter.validate_manifest(
                final,
                output,
                final_collision_report=result.final_report_path,
            )

            tampered = adapter.deepcopy(final)
            tampered_record = tampered["parts"]["static"][
                "m2_Leadshine_CS-M21708_exact_cableless"
            ]
            tampered_record["collision_overbound"]["overbound_proof"][
                "status"
            ] = "FAIL"
            with self.assertRaisesRegex(ValueError, "overbound proof failed"):
                adapter.validate_collision_pipeline(
                    tampered,
                    output,
                    final_report_path=result.final_report_path,
                )

    def test_final_validation_detects_retained_source_visual_hash_drift(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            manifest, source_visual = self._fixture(output)
            result = adapter.run_collision_integrity_two_pass(manifest, output)
            source_visual.write_bytes(b"tampered exact visual")
            with self.assertRaisesRegex(ValueError, "source visual hash drift"):
                adapter.validate_collision_pipeline(
                    result.manifest,
                    output,
                    final_report_path=result.final_report_path,
                )


class DriveVisualMaterialContractTests(unittest.TestCase):
    @staticmethod
    def _labeled_cylinder(label: str):
        shape = Cylinder(1.0, 1.0, align=CTR)
        shape.label = label
        return shape

    def test_belt_and_pulley_groups_match_only_exact_labels_and_owners(self):
        self.assertEqual(
            adapter.M2_MOTOR_PULLEY_LABEL,
            adapter.candidate.nbk_p30.STOCK_LABEL,
        )
        self.assertEqual(
            adapter.M2_FLYER_PULLEY_LABEL,
            adapter.candidate.nbk_p30_d10.STOCK_LABEL,
        )
        expected = {
            ("static", adapter.M2_BELT_LABEL): (
                "m2_drive_belt",
                "belt_dark_rubber",
            ),
            ("static", adapter.M2_MOTOR_PULLEY_LABEL): (
                "m2_motor_pulley",
                "pulley_aluminum",
            ),
            ("flyer", adapter.M2_FLYER_PULLEY_LABEL): (
                "m2_flyer_pulley",
                "pulley_aluminum",
            ),
        }
        for (link, label), (name, material) in expected.items():
            selected = adapter._group_for_label(link, label, {})
            self.assertIsNotNone(selected)
            self.assertEqual(selected[:2], (name, material))
            wrong_link = "flyer" if link == "static" else "static"
            self.assertIsNone(
                adapter._group_for_label(wrong_link, label, {})
            )
            self.assertIsNone(
                adapter._group_for_label(link, label + "_near_match", {})
            )

    def test_split_groups_one_belt_and_both_pulley_occurrences(self):
        static_base = self._labeled_cylinder("static_fixture_base")
        flyer_base = self._labeled_cylinder("flyer_fixture_base")
        static_parts = [
            static_base,
            self._labeled_cylinder(adapter.M2_BELT_LABEL),
            self._labeled_cylinder(adapter.M2_MOTOR_PULLEY_LABEL),
        ]
        flyer_parts = [
            flyer_base,
            self._labeled_cylinder(adapter.M2_FLYER_PULLEY_LABEL),
        ]

        static_remaining, static_groups = adapter.split_visual_parts(
            "static", static_parts
        )
        flyer_remaining, flyer_groups = adapter.split_visual_parts(
            "flyer", flyer_parts
        )

        self.assertEqual(static_remaining, [static_base])
        self.assertEqual(flyer_remaining, [flyer_base])
        self.assertEqual(len(static_groups["m2_drive_belt"]["parts"]), 1)
        self.assertEqual(len(static_groups["m2_motor_pulley"]["parts"]), 1)
        self.assertEqual(len(flyer_groups["m2_flyer_pulley"]["parts"]), 1)

    def test_drive_pbr_materials_visually_separate_engagement_and_wire(self):
        belt = adapter.MATERIALS["belt_dark_rubber"]
        pulley = adapter.MATERIALS["pulley_aluminum"]
        copper = adapter.MATERIALS["enameled_copper"]
        self.assertEqual(belt["metallic"], 0.0)
        self.assertGreaterEqual(belt["roughness"], 0.9)
        self.assertGreater(pulley["metallic"], 0.8)
        self.assertLess(pulley["roughness"], 0.3)
        self.assertNotEqual(belt["color_rgba"], copper["color_rgba"])
        self.assertNotEqual(pulley["color_rgba"], copper["color_rgba"])


if __name__ == "__main__":
    unittest.main()
