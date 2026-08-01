"""Regression tests for the fail-closed collision mesh integrity gate."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pygltflib import GLTF2
import trimesh

import collision_mesh_integrity as integrity


def _export(mesh: trimesh.Trimesh, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(path, file_type="stl")
    return path


def _open_box() -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=(2.0, 3.0, 4.0))
    mesh.update_faces(np.arange(len(mesh.faces) - 1))
    mesh.remove_unreferenced_vertices()
    return mesh


def _normal_glb(path: Path) -> Path:
    scene = trimesh.Scene()
    scene.add_geometry(trimesh.creation.box(), geom_name="box", node_name="box")
    path.write_bytes(scene.export(file_type="glb", include_normals=True))
    return path


class TopologyClassificationTests(unittest.TestCase):
    def test_empty_boundary_nonmanifold_only_and_multi_shell(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            empty = trimesh.Trimesh(
                vertices=np.empty((0, 3)), faces=np.empty((0, 3), dtype=int),
                process=False,
            )
            _export(empty, root / "empty.stl")
            self.assertEqual(
                integrity.topology_facts(root / "empty.stl")["topology_class"],
                "empty",
            )

            _export(_open_box(), root / "boundary.stl")
            boundary = integrity.topology_facts(root / "boundary.stl")
            self.assertEqual(boundary["topology_class"], "boundary_edge")
            self.assertGreater(boundary["boundary_edges"], 0)

            # Two closed tetrahedra share one duplicated face.  All edges have
            # incidence >=2, while the shared-face edges have incidence 4.
            vertices = np.asarray(
                [
                    [0, 0, 0],
                    [1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1],
                    [0, 0, -1],
                ],
                dtype=float,
            )
            faces = np.asarray(
                [
                    [0, 1, 2],
                    [0, 3, 1],
                    [1, 3, 2],
                    [2, 3, 0],
                    [0, 2, 1],
                    [0, 1, 4],
                    [1, 2, 4],
                    [2, 0, 4],
                ],
                dtype=int,
            )
            _export(
                trimesh.Trimesh(vertices=vertices, faces=faces, process=False),
                root / "nonmanifold.stl",
            )
            nonmanifold = integrity.topology_facts(root / "nonmanifold.stl")
            self.assertEqual(nonmanifold["topology_class"], "nonmanifold_only")
            self.assertEqual(nonmanifold["boundary_edges"], 0)
            self.assertGreater(nonmanifold["nonmanifold_edges"], 0)

            two_boxes = trimesh.util.concatenate(
                [
                    trimesh.creation.box(),
                    trimesh.creation.box(
                        transform=trimesh.transformations.translation_matrix(
                            [3.0, 0.0, 0.0]
                        )
                    ),
                ]
            )
            _export(two_boxes, root / "multi.stl")
            multi = integrity.topology_facts(root / "multi.stl")
            self.assertEqual(multi["topology_class"], "closed_multi_shell")
            self.assertTrue(multi["watertight"])
            self.assertEqual(multi["shell_count"], 2)


class ConservativeEnvelopeTests(unittest.TestCase):
    def test_open_vendor_visual_is_unchanged_and_hull_overbounds(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = _export(_open_box(), root / "vendor.stl")
            before = integrity.sha256(source)
            result = integrity.generate_conservative_hull(
                source, root / "collision_hull.stl"
            )
            self.assertEqual(result["status"], "PASS")
            self.assertEqual(integrity.sha256(source), before)
            self.assertTrue(
                result["overbound_proof"]["all_source_vertices_contained"]
            )
            self.assertTrue(
                result["collision_envelope"]["clean_collision_mesh"]
            )
            self.assertTrue(result["exact_vendor_visual_retained"])


class AdapterAuditTests(unittest.TestCase):
    def _fixture(
        self,
        root: Path,
        *,
        label: str,
        provenance: str,
        mesh: trimesh.Trimesh,
        overbound_method: str | None = None,
    ) -> tuple[Path, Path]:
        links = root / "links"
        part = _export(mesh, links / "parts" / "static" / "part.stl")
        aggregate = _export(mesh.copy(), links / "static.stl")
        glb = _normal_glb(root / "integrated_candidate_reference_pose.glb")
        part_record = {
            "file": part.name,
            "sha256": integrity.sha256(part),
            "provenance_class": provenance,
        }
        if overbound_method is not None:
            part_record["collision_overbound_method"] = overbound_method
        manifest = {
            "schema": "fixture",
            "status": "REVIEW",
            "production_authorized": False,
            "links": {
                "static": {
                    "file": aggregate.name,
                    "sha256": integrity.sha256(aggregate),
                }
            },
            "parts": {
                "static": {
                    label: part_record
                }
            },
            "visual_groups": {},
            "wire_assets": {},
            "reference_pose_glb": {
                "file": glb.name,
                "sha256": integrity.sha256(glb),
            },
        }
        manifest["contract_sha256"] = integrity.canonical_hash(manifest)
        manifest_path = links / "manifest.json"
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        return manifest_path, glb

    def test_imported_open_visual_gets_verified_collision_override(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            label = "m2_Leadshine_CS-M21708_exact_cableless"
            manifest, glb = self._fixture(
                root,
                label=label,
                provenance=integrity.PROVENANCE_IMPORTED,
                mesh=_open_box(),
            )
            report = integrity.audit_adapter(
                manifest, root / "envelopes", glb_path=glb
            )
            self.assertEqual(report["status"], "PASS")
            self.assertEqual(
                report["summary"]["generated_overbound_count"], 1
            )
            plan = integrity.collision_override_plan(report)
            self.assertIn(label, plan["static"])
            integrity.require_release_ready(report)
            Path(plan["static"][label]["source_visual_path"]).write_bytes(
                b"vendor drift"
            )
            with self.assertRaisesRegex(ValueError, "source visual hash drift"):
                integrity.collision_override_plan(report)

    def test_explicit_manifest_hull_policy_supports_renamed_import(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            label = "renamed_exact_vendor_import"
            manifest, glb = self._fixture(
                root,
                label=label,
                provenance=integrity.PROVENANCE_IMPORTED,
                mesh=_open_box(),
                overbound_method=integrity.HULL_METHOD,
            )
            report = integrity.audit_adapter(
                manifest, root / "envelopes", glb_path=glb
            )
            self.assertEqual(report["status"], "PASS")
            row = report["parts"][0]
            self.assertTrue(row["imported_hull_eligible"])
            self.assertEqual(row["hull_eligibility_source"], "manifest")

    def test_modeled_felt_wingnut_gets_closed_collision_substitute(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            label = "felt_m4_wingnut"
            manifest, glb = self._fixture(
                root,
                label=label,
                provenance=integrity.PROVENANCE_MODELED,
                mesh=_open_box(),
            )
            report = integrity.audit_adapter(
                manifest, root / "envelopes", glb_path=glb
            )
            self.assertEqual(report["status"], "PASS")
            plan = integrity.collision_override_plan(report)
            selected = plan["static"][label]
            self.assertTrue(selected["detailed_modeled_visual_retained"])
            self.assertFalse(selected["exact_vendor_visual_retained"])

    def test_provenance_template_explicitly_lists_inferred_custom_keys(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            label = "printed_PEEK_fixture"
            manifest, _glb = self._fixture(
                root,
                label=label,
                provenance=integrity.PROVENANCE_CUSTOM,
                mesh=trimesh.creation.box(),
            )
            payload = json.loads(manifest.read_text())
            del payload["parts"]["static"][label]["provenance_class"]
            payload["contract_sha256"] = integrity.canonical_hash(payload)
            manifest.write_text(json.dumps(payload), encoding="utf-8")
            template = integrity.provenance_template(manifest)
            key = f"static/{label}"
            self.assertEqual(template["status"], "REVIEW_REQUIRED")
            self.assertIn(key, template["custom_printed_PEEK_fabricated_keys"])
            self.assertEqual(
                template["mapping"][key]["provenance_class"],
                integrity.PROVENANCE_CUSTOM,
            )
            self.assertTrue(template["mapping"][key]["review_required"])
            self.assertEqual(
                template["contract_sha256"], integrity.canonical_hash(template)
            )

    def test_custom_open_mesh_fails_without_silent_hull(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            label = "printed_flyer_arm"
            manifest, glb = self._fixture(
                root,
                label=label,
                provenance=integrity.PROVENANCE_CUSTOM,
                mesh=_open_box(),
            )
            report = integrity.audit_adapter(
                manifest, root / "envelopes", glb_path=glb
            )
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["summary"]["generated_overbound_count"], 0)
            self.assertEqual(
                report["summary"]["custom_topology_failures"],
                [f"static/{label}"],
            )
            with self.assertRaisesRegex(ValueError, "not PASS"):
                integrity.require_release_ready(report)

    def test_legacy_fallback_never_satisfies_explicit_provenance_gate(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest, glb = self._fixture(
                root,
                label="printed_fixture",
                provenance=integrity.PROVENANCE_CUSTOM,
                mesh=trimesh.creation.box(),
            )
            payload = json.loads(manifest.read_text())
            del payload["parts"]["static"]["printed_fixture"][
                "provenance_class"
            ]
            payload["contract_sha256"] = integrity.canonical_hash(payload)
            manifest.write_text(json.dumps(payload), encoding="utf-8")
            report = integrity.audit_adapter(
                manifest,
                root / "envelopes",
                glb_path=glb,
                allow_legacy_label_fallback=True,
            )
            self.assertFalse(report["gates"]["explicit_per_part_provenance"])
            self.assertEqual(report["status"], "FAIL")


class GltfNormalTests(unittest.TestCase):
    def test_missing_normal_accessor_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            valid = _normal_glb(root / "valid.glb")
            self.assertEqual(integrity.gltf_normal_audit(valid)["status"], "PASS")
            gltf = GLTF2().load(str(valid))
            gltf.meshes[0].primitives[0].attributes.NORMAL = None
            missing = root / "missing.glb"
            gltf.save_binary(str(missing))
            report = integrity.gltf_normal_audit(missing)
            self.assertEqual(report["status"], "FAIL")
            self.assertFalse(report["primitives"][0]["normal_present"])


if __name__ == "__main__":
    unittest.main()
