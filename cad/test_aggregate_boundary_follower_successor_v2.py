"""Focused acceptance tests for the isolated successor-follower V2 CAD."""

from __future__ import annotations

from collections import Counter
import hashlib
import math
from pathlib import Path
import sys
import unittest

from build123d import Vector


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_successor_v2 as v2


EXPECTED_PLACEMENT_INTERNAL_SHA256 = (
    "1800b5f9500f5b0041758991cc8f42f8dc0b62654bec3ce84e402b59dd79dbc3"
)
EXPECTED_PLACEMENT_FILE_SHA256 = (
    "be599cbfed61afdfdaa7fc9c053ee1e20a3ab20cfe723699be1eb5a81e4dbb4c"
)
EXPECTED_DATUM_CASES = {
    0: {
        "locus_index": 50,
        "pass_index": 0,
        "state_index": 50,
        "turn_index": 25,
        "half_turn_index": 0,
        "tooth_index": 0,
        "lane_id": "tooth_00_left_front",
        "wire_diameter_mm": 0.2,
        "time_s": 13.339822369,
    },
    1: {
        "locus_index": 102,
        "pass_index": 1,
        "state_index": 2,
        "turn_index": 1,
        "half_turn_index": 0,
        "tooth_index": 1,
        "lane_id": "tooth_01_right_front",
        "wire_diameter_mm": 0.2,
        "time_s": 27.426179939,
    },
    2: {
        "locus_index": 3,
        "pass_index": 0,
        "state_index": 3,
        "turn_index": 1,
        "half_turn_index": 1,
        "tooth_index": 0,
        "lane_id": "tooth_00_right_rear",
        "wire_diameter_mm": 0.2,
        "time_s": 5.957079633,
    },
    3: {
        "locus_index": 151,
        "pass_index": 1,
        "state_index": 51,
        "turn_index": 25,
        "half_turn_index": 1,
        "tooth_index": 1,
        "lane_id": "tooth_01_left_rear",
        "wire_diameter_mm": 0.2,
        "time_s": 35.146902001,
    },
}
EXPECTED_HARDWARE_PER_MODULE = {
    "ISO4762_M3x14_pod_mount": 4,
    "ISO7089_M3_pod_mount_washer": 4,
    "McMaster_94459A130_short_M3_insert": 4,
    "McMaster_90265A115_OD3x10_M2_shoulder_screw": 2,
    "NMB_L-630ZZ_3x6x2p5": 4,
    "DIN988_3x6x0p5_shim": 4,
    "precision_3mm_ID_x_4mm_inner_spacer": 2,
    "matched_4mm_outer_race_spacer": 2,
    "ISO10511_M2_pivot_nyloc": 2,
    "bearing_keeper_cap": 2,
    "ISO4762_M2x6_bearing_keeper": 4,
    "ISO7089_M2_bearing_keeper_washer": 4,
    "ISO4762_M2x6_leaf_root": 2,
    "ISO7089_M2_leaf_root_washer": 2,
    "ISO4762_M2x8_preload_adjuster": 1,
    "ISO10511_M2_adjuster_jam_nut": 1,
    "ISO4762_M2x6_shoe": 1,
    "ISO7089_M2_shoe_washer": 1,
}


def _xyz(value) -> tuple[float, float, float]:
    return float(value.X), float(value.Y), float(value.Z)


def _dot(one, two) -> float:
    return sum(float(one[index]) * float(two[index]) for index in range(3))


def _cross(one, two) -> tuple[float, float, float]:
    ax, ay, az = map(float, one)
    bx, by, bz = map(float, two)
    return ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx


def _unit(value) -> tuple[float, float, float]:
    length = math.sqrt(_dot(value, value))
    return tuple(float(component) / length for component in value)


def _distance(one, two) -> float:
    return math.sqrt(sum(
        (float(one[index]) - float(two[index])) ** 2 for index in range(3)
    ))


def _leaf_shapes(shape):
    children = tuple(getattr(shape, "children", ()) or ())
    if children:
        for child in children:
            yield from _leaf_shapes(child)
    else:
        yield shape


def _case_key(case) -> str:
    return (
        f"L{case['locus_index']}/P{case['pass_index']}/"
        f"id{case['identity']['physical_id']}/d{case['wire_diameter_mm']}"
    )


class AggregateBoundaryFollowerSuccessorV2Tests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = v2.placement_report()
        cls.cases = cls.report["case_comparisons"]

    def test_frozen_4704_case_evidence(self):
        self.assertEqual(
            v2.EXPECTED_PLACEMENT_INTERNAL_SHA256,
            EXPECTED_PLACEMENT_INTERNAL_SHA256,
        )
        self.assertEqual(
            self.report["report_sha256"],
            EXPECTED_PLACEMENT_INTERNAL_SHA256,
        )
        self.assertEqual(len(self.cases), 4704)
        self.assertEqual(
            Counter(int(case["identity"]["physical_id"])
                    for case in self.cases),
            Counter({0: 1176, 1: 1176, 2: 1176, 3: 1176}),
        )
        self.assertEqual(
            hashlib.sha256(v2.PLACEMENT_PATH.read_bytes()).hexdigest(),
            EXPECTED_PLACEMENT_FILE_SHA256,
        )

    def test_exact_real_datum_case_keys(self):
        expected_source_keys = {
            identity: {
                name: value for name, value in expected.items()
                if name != "time_s"
            }
            for identity, expected in EXPECTED_DATUM_CASES.items()
        }
        self.assertEqual(v2.DATUM_CASE_KEYS, expected_source_keys)
        for identity, expected in EXPECTED_DATUM_CASES.items():
            with self.subTest(identity=identity):
                case = v2.datum_case(identity)
                self.assertEqual(
                    int(case["identity"]["physical_id"]), identity,
                )
                self.assertEqual(
                    case["identity"]["name"], v2.IDENTITY_NAMES[identity],
                )
                for name, value in expected.items():
                    self.assertEqual(case[name], value)

    def test_full_guide_frame_matches_all_4704_tangents_and_normals(self):
        worst = {
            "origin": (0.0, None),
            "tangent": (0.0, None),
            "normal": (0.0, None),
            "binormal": (0.0, None),
            "right_handed": (0.0, None),
        }
        for case in self.cases:
            center = tuple(map(float, case["required_center_local_mm"]))
            tangent = _unit(case["required_guide_tangent"])
            normal_seed = _unit(
                case["required_curvature_normal_contact_to_center"]
            )
            binormal = _unit(_cross(tangent, normal_seed))
            normal = _unit(_cross(binormal, tangent))
            frame = v2.guide_frame(center, tangent, normal_seed)
            actual = {
                "origin": _xyz(frame.origin),
                "tangent": _xyz(frame.x_dir),
                "normal": _xyz(frame.y_dir),
                "binormal": _xyz(frame.z_dir),
            }
            expected = {
                "origin": center,
                "tangent": tangent,
                "normal": normal,
                "binormal": binormal,
            }
            key = _case_key(case)
            for name in expected:
                residual = _distance(actual[name], expected[name])
                if residual > worst[name][0]:
                    worst[name] = residual, key
            determinant = _dot(
                _cross(actual["tangent"], actual["normal"]),
                actual["binormal"],
            )
            handed_residual = abs(determinant - 1.0)
            if handed_residual > worst["right_handed"][0]:
                worst["right_handed"] = handed_residual, key

        for name, (residual, witness) in worst.items():
            self.assertLessEqual(
                residual, 1.0e-12,
                msg=f"{name} residual {residual} at {witness}",
            )

    def test_peek_guide_is_one_solid_with_positive_Z_loading_opening(self):
        guide = v2.peek_guide_local()
        self.assertEqual(len(list(guide.solids())), 1)
        self.assertGreater(float(guide.volume), 0.0)

        # Three points along the straight/arc/straight route prove the channel
        # remains open from +Z while retaining material on its backed -Z side.
        for x, y in ((-1.5, -3.0), (0.0, -3.0), (3.0, 0.0)):
            with self.subTest(x=x, y=y):
                self.assertFalse(guide.is_inside(Vector(x, y, 0.0)))
                self.assertFalse(guide.is_inside(Vector(x, y, 0.60)))
                self.assertTrue(guide.is_inside(Vector(x, y, -0.60)))

        contract = v2.geometry_contract()["guide"]
        self.assertEqual(contract["count"], 4)
        self.assertEqual(contract["centerline_bend_radius_mm"], 3.0)
        self.assertTrue(contract["C1_by_construction"])
        self.assertTrue(contract["positive_Z_loading_opening"])

    def test_shared_carrier_is_one_solid_and_window_contains_neutral_work(self):
        carrier = v2.shared_carrier()
        window = v2._shared_window_tool()
        self.assertEqual(len(list(carrier.solids())), 1)
        self.assertGreater(float(carrier.volume), 0.0)

        bounds = v2.SHARED_WINDOW_BOUNDS_MM
        box = window.bounding_box()
        self.assertLessEqual(
            _distance(_xyz(box.min), tuple(map(float, bounds["min"]))),
            1.0e-12,
        )
        self.assertLessEqual(
            _distance(_xyz(box.max), tuple(map(float, bounds["max"]))),
            1.0e-12,
        )
        common = carrier & window
        self.assertLessEqual(
            0.0 if common is None else float(common.volume), 1.0e-8,
        )

        # Every exact required center lies in the shared window, while the
        # four reproducible neutral manufactured guides fit completely inside.
        for case in self.cases:
            center = case["required_center_local_mm"]
            for axis in range(3):
                self.assertGreaterEqual(
                    float(center[axis]), float(bounds["min"][axis]) - 1.0e-12,
                )
                self.assertLessEqual(
                    float(center[axis]), float(bounds["max"][axis]) + 1.0e-12,
                )
        for identity in range(4):
            guide = v2.guide_at_case(v2.datum_case(identity))
            outside = guide - window
            self.assertLessEqual(float(outside.volume), 1.0e-8)

    def test_four_module_and_hardware_counts(self):
        contract = v2.geometry_contract()
        self.assertEqual(contract["pod"]["count"], 4)
        self.assertEqual(contract["guide"]["count"], 4)
        self.assertEqual(contract["preload"]["leaf_count"], 4)
        self.assertEqual(contract["preload"]["shoe_count"], 4)

        hardware = v2.hardware_contract()
        self.assertEqual(hardware["per_module"], EXPECTED_HARDWARE_PER_MODULE)
        self.assertEqual(sum(hardware["per_module"].values()), 46)
        total = hardware["four_module_plus_existing_primary_mount"]
        for name, quantity in EXPECTED_HARDWARE_PER_MODULE.items():
            self.assertEqual(total[name], quantity * 4)
        self.assertEqual(total["NBK_SSHS_M4x10_SD_ALK_primary_tower_screw"], 4)
        self.assertEqual(total["short_M4_heat_set_primary_tower_insert"], 4)

        for identity in range(4):
            with self.subTest(identity=identity):
                self.assertEqual(len(v2.pod_attachment_hardware(identity)), 12)
                self.assertEqual(len(v2.gimbal_hardware(identity)), 26)
                self.assertEqual(len(v2.preload_parts(identity)), 13)
                self.assertEqual(len(v2.center_bound_witnesses(identity)), 2)
                self.assertEqual(len(v2.module_parts(identity)), 57)

    def test_every_manufactured_leaf_is_single_solid_positive_volume(self):
        manufactured = [v2.shared_carrier()]
        manufactured.extend(v2.replacement.primary_tower_m4_hardware())
        construction_count = 0
        for identity in range(4):
            for part in v2.module_parts(identity):
                for leaf in _leaf_shapes(part):
                    if str(leaf.label).startswith("CONSTRUCTION_ONLY_"):
                        construction_count += 1
                    else:
                        manufactured.append(leaf)

        self.assertEqual(construction_count, 8)
        self.assertEqual(len(manufactured), 233)
        for leaf in manufactured:
            with self.subTest(label=leaf.label):
                self.assertEqual(len(list(leaf.solids())), 1)
                self.assertGreater(float(leaf.volume), 0.0)

        assembly = v2.gen_step()
        self.assertEqual(len(list(assembly.children)), 5)
        self.assertEqual(len(list(assembly.solids())), 241)
        self.assertTrue(all(float(solid.volume) > 0.0
                            for solid in assembly.solids()))

    def test_selected_assembly_and_integrated_release_do_not_import_v2(self):
        forbidden = "aggregate_boundary_follower_successor_v2"
        for name in ("assembly.py", "integrated_release_candidate.py"):
            with self.subTest(source=name):
                source = (HERE / name).read_text(encoding="utf-8")
                self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
