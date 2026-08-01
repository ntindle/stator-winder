"""Focused topology regression for the exact flyer wire elbow."""

from pathlib import Path
import sys
import unittest

import numpy as np
import trimesh
from build123d import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
import printed
import wire_geometry


class WireElbowTests(unittest.TestCase):
    def test_single_watertight_print_mesh(self):
        part = printed.wire_elbow()
        self.assertEqual(wire_geometry.TIP_GUIDE_CENTER_Z, -17.0)
        self.assertEqual(wire_geometry.FLYER_ELBOW_RADIUS, 5.0)
        self.assertTrue(part.is_valid)
        self.assertEqual(len(part.solids()), 1)

        vertices, faces = part.tessellate(
            tolerance=0.05, angular_tolerance=0.15)
        mesh = trimesh.Trimesh(
            vertices=np.asarray([(v.X, v.Y, v.Z) for v in vertices]),
            faces=np.asarray(faces),
            process=True,
        )
        incidence = np.bincount(mesh.edges_unique_inverse)
        self.assertTrue(mesh.is_watertight)
        self.assertTrue(mesh.is_winding_consistent)
        self.assertEqual(len(mesh.split(only_watertight=False)), 1)
        self.assertEqual(np.count_nonzero(incidence == 1), 0)
        self.assertEqual(np.count_nonzero(incidence > 2), 0)
        self.assertGreater(mesh.volume, 0.0)

        # The overshot channel must remain open through both tangent runs.
        self.assertFalse(part.is_inside(Vector(0.0, 0.0, -30.0)))
        self.assertFalse(part.is_inside(Vector(0.0, 10.0, -17.0)))
        self.assertTrue(part.is_inside(Vector(2.0, 10.0, -17.0)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
