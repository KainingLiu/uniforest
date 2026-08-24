import unittest

from Strategy.common import minimum_command, slew_command, wrap_angle


class StrategyCommonTests(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertEqual(wrap_angle(180.0), -180.0)
        self.assertAlmostEqual(wrap_angle(540.0), -180.0)
        self.assertAlmostEqual(wrap_angle(-181.0), 179.0)

    def test_minimum_command(self):
        self.assertEqual(minimum_command(0.0, 40.0), 0.0)
        self.assertEqual(minimum_command(10.0, 40.0), 40.0)
        self.assertEqual(minimum_command(-10.0, 40.0), -40.0)

    def test_slew_command(self):
        self.assertEqual(slew_command(100.0, 0.0, 300.0, 0.1), 30.0)
        self.assertEqual(slew_command(-100.0, 30.0, 300.0, 0.1), 0.0)


if __name__ == '__main__':
    unittest.main()
