"""Focused contract tests for the isolated replacement-carriage prototype."""

from __future__ import annotations

import itertools
from pathlib import Path
import sys
import unittest

from build123d import Vector

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_replacement_carriage as model
import m1_selector_alternating_former as selector


class AggregateBoundaryFollowerReplacementCarriageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.carrier = model.replacement_carrier()

    def test_one_shared_carrier_preserves_U_window_and_four_M4_axes(self):
        carrier = self.carrier
        self.assertEqual(len(list(carrier.solids())), 1)
        self.assertGreater(float(carrier.volume), 0.0)
        self.assertFalse(carrier.is_inside(Vector(27.75, 0.0, -112.0)))
        self.assertTrue(carrier.is_inside(Vector(36.0, 0.0, -112.0)))
        for x, y, _z in model._primary_m4_local_locations():
            with self.subTest(x=x, y=y):
                self.assertFalse(carrier.is_inside(Vector(x, y, -112.0)))
        for x in (29.0, 35.0):
            for y in (-21.0, 21.0):
                with self.subTest(old_x=x, old_y=y):
                    self.assertTrue(carrier.is_inside(Vector(x, y, -112.0)))

    def test_obsolete_M3_holes_are_restored_and_key_fill_is_superseded(self):
        carrier = self.carrier
        for a, t in itertools.product((-1, 1), repeat=2):
            with self.subTest(axial=a, tangential=t):
                self.assertTrue(carrier.is_inside(Vector(
                    37.10, t * 15.10, a * 21.35,
                )))
                self.assertFalse(carrier.is_inside(Vector(
                    34.70, t * 10.525, a * 23.55,
                )))
        contract = model.geometry_contract()["carrier"]
        self.assertFalse(contract["legacy_guide_M3_holes_present"])
        self.assertFalse(
            contract["legacy_guide_key_pockets_functionally_present"]
        )
        self.assertTrue(
            contract["legacy_guide_key_fill_subsumed_by_clearance_relief"]
        )
        self.assertEqual(contract["integral_selection_bay_count"], 4)

    def test_all_36_selector_gate_states_return_exactly_four_occurrences(self):
        for law, track, gate in itertools.product(
            model.M1_LAWS, range(4), model.M0_GATE_STATES,
        ):
            with self.subTest(law=law, track=track, gate=gate):
                states = model.selected_occurrences(law, track, gate)
                self.assertEqual(len(states), 4)
                self.assertEqual([state["index"] for state in states],
                                 [0, 1, 2, 3])
                self.assertTrue(all(state["owner"] == "M0_carriage"
                                    for state in states))
                self.assertTrue(all(not state["M1_spatial_transform"]
                                    for state in states))
                self.assertTrue(all(not state["M2_spatial_transform"]
                                    for state in states))
                selected = [state for state in states if state["selected"]]
                if gate == "ENGAGED_LOCKED":
                    self.assertEqual(len(selected), 1)
                    self.assertEqual(
                        selected[0]["index"],
                        model.physical_occurrence_index(law, track),
                    )
                    self.assertEqual(
                        selected[0]["coarse_base_abs_y_mm"], 2.05,
                    )
                    self.assertEqual(
                        selected[0]["radial_reference_state"], "mid",
                    )
                else:
                    self.assertEqual(selected, [])
                for state in states:
                    if not state["selected"]:
                        self.assertEqual(
                            state["coarse_base_abs_y_mm"], 10.95,
                        )
                        self.assertEqual(
                            state["radial_reference_state"], "retracted",
                        )

    def test_three_law_track_permutations_are_exact(self):
        direct = [
            model.physical_occurrence_index(selector.LAW_DIRECT, track)
            for track in range(4)
        ]
        reverse_zero = [
            model.physical_occurrence_index(selector.LAW_REVERSE_ZERO, track)
            for track in range(4)
        ]
        reverse_180 = [
            model.physical_occurrence_index(selector.LAW_REVERSE_180, track)
            for track in range(4)
        ]
        self.assertEqual(direct, [0, 1, 2, 3])
        self.assertEqual(reverse_zero, [0, 1, 2, 3])
        self.assertEqual(reverse_180, [2, 3, 0, 1])

    def test_reference_nose_centers_and_machine_transform_are_handed(self):
        expected_local = (
            (29.70, -10.95, 21.35),
            (29.70, 10.95, 21.35),
            (29.70, 10.95, -21.35),
            (29.70, -10.95, -21.35),
        )
        expected_machine = (
            (10.95, 21.35, 65.30),
            (-10.95, 21.35, 65.30),
            (-10.95, -21.35, 65.30),
            (10.95, -21.35, 65.30),
        )
        for identity, local, machine in zip(
            model.OCCURRENCE_IDENTITIES,
            expected_local,
            expected_machine,
        ):
            actual_local = model.occurrence_nose_center(
                identity,
                radial_state="retracted",
                coarse_base_mm=model.COARSE_PARKED_BASE_MM,
            )
            for value, expected in zip(actual_local, local):
                self.assertAlmostEqual(value, expected, places=9)
            actual_machine = model.active_local_to_machine_reference(
                actual_local
            )
            for value, expected in zip(actual_machine, machine):
                self.assertAlmostEqual(value, expected, places=9)

    def test_four_handed_occurrences_have_exact_positive_leaf_counts(self):
        labels: list[str] = []
        for identity in model.OCCURRENCE_IDENTITIES:
            occurrence = model.moving_occurrence(identity)
            self.assertEqual(
                len(list(occurrence.solids())),
                model.MOVING_LEAF_COUNT_PER_OCCURRENCE,
            )
            self.assertEqual(
                len(occurrence.children),
                model.MOVING_LEAF_COUNT_PER_OCCURRENCE,
            )
            for child in occurrence.children:
                self.assertEqual(len(list(child.solids())), 1)
                self.assertGreater(float(child.volume), 0.0)
                labels.append(str(child.label))
            common = self.carrier & occurrence
            overlap = 0.0 if common is None else float(common.volume)
            self.assertAlmostEqual(overlap, 0.0, places=7)
            self.assertAlmostEqual(
                float(self.carrier.distance_to(occurrence)), 0.0, places=7,
            )
        self.assertEqual(len(labels), len(set(labels)))
        forbidden = ("M3", "mounting_backer_context", "active_sector_PEEK")
        self.assertFalse(any(token in label
                             for token in forbidden for label in labels))
        self.assertEqual(
            sum("outer_pivot_MISUMI_SCCG5-10" in label for label in labels),
            4,
        )
        self.assertEqual(
            sum("outer_pivot_MISUMI_NETWS4" in label for label in labels), 8,
        )

    def test_8p90mm_coarse_linkage_is_explicitly_a_blocker_envelope(self):
        blockers = [
            model.coarse_selection_linkage_blocker(identity)
            for identity in model.OCCURRENCE_IDENTITIES
        ]
        self.assertEqual(len(blockers), 4)
        for blocker in blockers:
            self.assertEqual(len(list(blocker.solids())), 1)
            self.assertGreater(float(blocker.volume), 0.0)
            self.assertIn("BLOCKER_ONLY_8p90mm", str(blocker.label))
            bbox = blocker.bounding_box()
            self.assertAlmostEqual(
                float(bbox.max.Y - bbox.min.Y), 8.90, places=9,
            )
        selection = model.geometry_contract()["selection"]
        self.assertAlmostEqual(
            selection["coarse_selection_travel_mm"], 8.90, places=9,
        )
        self.assertEqual(
            selection["coarse_linkage_geometry_mode"],
            "BLOCKER_ENVELOPE_ONLY",
        )

    def test_exact_primary_M4_and_total_leaf_counts_without_duplicates(self):
        hardware = model.primary_tower_m4_hardware()
        labels = [str(part.label) for part in hardware]
        self.assertEqual(len(hardware), 8)
        self.assertEqual(sum("NBK_SSHS_M4x10" in label for label in labels), 4)
        self.assertEqual(sum("M4_washer" in label for label in labels), 0)
        self.assertEqual(
            sum("M4_short_heat_insert" in label for label in labels), 4,
        )
        self.assertEqual(len(labels), len(set(labels)))
        assembly = model.gen_step()
        self.assertEqual(
            len(list(assembly.solids())), model.EXPECTED_LEAF_SOLID_COUNT,
        )
        self.assertEqual(model.EXPECTED_LEAF_SOLID_COUNT, 73)
        contract = model.geometry_contract()
        self.assertGreater(
            contract["fasteners"][
                "primary_M4_same_side_diagonal_center_distance_mm"
            ],
            contract["fasteners"]["primary_M4_recessed_head_diameter_mm"],
        )
        self.assertTrue(
            contract["fasteners"]["proof_basis_X_row_span_preserved"]
        )

    def test_authority_is_false_and_source_is_not_integrated(self):
        contract = model.geometry_contract()
        self.assertTrue(contract["authority"]["review_only"])
        for key, value in contract["authority"].items():
            if key != "review_only":
                self.assertFalse(value, key)
        self.assertIn(
            "UNMODELED_positive_volume_8p90mm_coarse_selection_linkage",
            contract["blockers"],
        )
        self.assertIn(
            "OPEN_MISUMI_SCCG5_10_pivot_pin_retention_load_and_wear",
            contract["blockers"],
        )
        rejected = contract["rejected_overlap_witnesses"]
        self.assertEqual(
            rejected["inward_outer_pivot_nyloc_stack"]
            ["engaged_cross_occurrence_positive_pair_count_each_identity"],
            14,
        )
        self.assertEqual(
            rejected["six_mm_pitch_primary_M4_stack"]
            ["positive_pair_count_reference_state"],
            18,
        )
        self.assertAlmostEqual(
            contract["selection"]
            ["engaged_complete_outer_pivot_envelope_clearance_mm"],
            3.0,
        )
        self.assertAlmostEqual(
            contract["selection"]
            ["inward_q_complete_outer_pivot_envelope_clearance_mm"],
            2.5,
        )
        assembly_source = (HERE / "assembly.py").read_text(encoding="utf-8")
        self.assertNotIn(
            "aggregate_boundary_follower_replacement_carriage",
            assembly_source,
        )

    def test_parked_downstream_stack_retains_2p5mm_carrier_clearance(self):
        for identity in model.OCCURRENCE_IDENTITIES:
            occurrence = model.moving_occurrence(
                identity,
                radial_state="retracted",
                coarse_base_mm=model.COARSE_PARKED_BASE_MM,
            )
            distances = [
                float(child.distance_to(self.carrier))
                for child in occurrence.children[1:]
            ]
            with self.subTest(identity=identity.name):
                self.assertGreaterEqual(min(distances), 2.5 - 1.0e-7)
        carrier_contract = model.geometry_contract()["carrier"]
        self.assertEqual(
            carrier_contract["parked_follower_relief_bounds_local_mm"],
            {
                "x": [25.0, 36.2],
                "abs_y": [5.45, 16.5],
                "abs_z": [9.85, 27.85],
            },
        )
        self.assertEqual(
            carrier_contract["selection_wall_abs_z_bounds_mm"],
            [2.85, 12.85],
        )
        self.assertAlmostEqual(
            carrier_contract["outboard_dogleg_web_min_radial_thickness_mm"],
            2.8,
            places=9,
        )

    def test_all_selector_state_manufactured_leaves_have_zero_positive_overlap(self):
        audit = model.all_selector_state_pair_audit()
        self.assertEqual(audit["state_count"], 36)
        self.assertEqual(audit["engaged_state_count"], 12)
        self.assertEqual(audit["all_parked_state_count"], 24)
        self.assertEqual(audit["unique_geometry_signature_count"], 5)
        self.assertTrue(audit["all_follower_carrier_states_zero_positive"])
        self.assertTrue(audit["all_complete_installed_states_zero_positive"])
        self.assertEqual(audit["follower_carrier_failure_state_count"], 0)
        self.assertEqual(audit["complete_installed_failure_state_count"], 0)
        self.assertFalse(audit["clearance_authority"])
        for state in audit["states"]:
            for scope_name in (
                "follower_carrier_scope", "complete_installed_scope",
            ):
                scope = state[scope_name]
                self.assertEqual(scope["positive_overlap_count"], 0)
                self.assertLessEqual(
                    scope["bbox_candidate_count"], scope["pair_count"],
                )
                self.assertEqual(
                    scope["exact_distance_candidate_count"],
                    scope["bbox_candidate_count"],
                )
                self.assertLessEqual(
                    scope["exact_common_boolean_count"],
                    scope["exact_distance_candidate_count"],
                )

    def test_invalid_selector_states_are_rejected(self):
        with self.assertRaises(ValueError):
            model.selected_occurrences("not-a-law", 0, "ENGAGED_LOCKED")
        with self.assertRaises(ValueError):
            model.selected_occurrences(selector.LAW_DIRECT, 4,
                                       "ENGAGED_LOCKED")
        with self.assertRaises(ValueError):
            model.selected_occurrences(selector.LAW_DIRECT, 0,
                                       "not-a-gate")
        with self.assertRaises(ValueError):
            model.physical_occurrence_index(selector.LAW_DIRECT, True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
