import unittest

from Strategy.tag_alignment import median_translation, translation_jump


class TagAlignmentTests(unittest.TestCase):
    def test_jump_filter(self):
        self.assertIsNone(translation_jump((100.0, 20.0), (120.0, 30.0),
                                           30.0, 20.0))
        self.assertEqual(translation_jump((100.0, 20.0), (140.0, 30.0),
                                          30.0, 20.0), (40.0, 10.0))

    def test_median_translation(self):
        self.assertEqual(median_translation([(100, 10), (120, 30),
                                              (110, 20)]), (110, 20))
        with self.assertRaises(ValueError):
            median_translation([])


if __name__ == '__main__':
    unittest.main()
