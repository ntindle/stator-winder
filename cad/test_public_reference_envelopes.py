import unittest

import cots


class PublicReferenceEnvelopeTests(unittest.TestCase):
    def setUp(self):
        cots.set_reference_mode("envelope")

    def tearDown(self):
        cots.set_reference_mode("exact")

    def test_rejects_unknown_reference_mode(self):
        with self.assertRaises(ValueError):
            cots.set_reference_mode("automatic")
        cots.set_reference_mode("envelope")

    def test_bearing_envelopes_match_nominal_dimensions(self):
        for part, expected in (
            (cots.bearing_608(), (22.0, 22.0, 7.0)),
            (cots.bearing_6001(), (28.0, 28.0, 8.0)),
        ):
            size = part.bounding_box().size
            self.assertAlmostEqual(size.X, expected[0], places=5)
            self.assertAlmostEqual(size.Y, expected[1], places=5)
            self.assertAlmostEqual(size.Z, expected[2], places=5)

    def test_rail_and_block_envelopes_match_controlled_bounds(self):
        rail = cots.mgn12_rail()
        rail_size = rail.bounding_box().size
        self.assertAlmostEqual(rail_size.X, 12.0, places=5)
        self.assertAlmostEqual(rail_size.Y, 8.0, places=5)
        self.assertAlmostEqual(rail_size.Z, 150.0, places=5)

        block = cots.mgn12h_block_real()
        block_size = block.bounding_box().size
        self.assertAlmostEqual(block_size.X, 27.0, places=5)
        self.assertAlmostEqual(block_size.Y, 10.0, places=5)
        self.assertAlmostEqual(block_size.Z, 45.4, places=5)


if __name__ == "__main__":
    unittest.main()
