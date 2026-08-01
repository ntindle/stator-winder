import unittest

import printed


class FlyerArmWireReliefTests(unittest.TestCase):
    def test_shaft_wrap_relief_is_open_and_side_rails_remain(self):
        arm = printed.flyer_arm()
        self.assertTrue(arm.is_valid)
        self.assertEqual(len(arm.solids()), 1)

        # Stay 0.1 mm inside every nominal relief face so Boolean tolerances
        # cannot turn this into a coincident-face test.
        void_probe = printed._box(-2.9, 2.9, 41.0, 42.0, -2.9, -2.1)
        self.assertLess((arm & void_probe).volume, 1e-6)

        # The 14 mm finger keeps two nominal 4 mm side rails outside x=+/-3.
        for x0, x1 in ((-6.5, -3.5), (3.5, 6.5)):
            rail_probe = printed._box(x0, x1, 41.1, 41.9, -2.9, -2.1)
            self.assertGreater((arm & rail_probe).volume, 1.0)


if __name__ == "__main__":
    unittest.main()
