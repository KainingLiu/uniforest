import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vision"))

from camera_devices import resolve_camera_source


class CameraDeviceTests(unittest.TestCase):
    ROLES = {
        "cube": "/dev/v4l/by-id/cube-video-index0",
        "tag": "/dev/v4l/by-id/tag-video-index0",
    }

    def test_roles_resolve_to_stable_paths(self):
        self.assertEqual(
            resolve_camera_source("cube", key="linux", roles=self.ROLES,
                                  require_exists=False),
            self.ROLES["cube"],
        )
        self.assertEqual(
            resolve_camera_source("tag", key="linux", roles=self.ROLES,
                                  require_exists=False),
            self.ROLES["tag"],
        )

    def test_default_linux_role_is_cube(self):
        self.assertEqual(
            resolve_camera_source(None, key="linux", roles=self.ROLES,
                                  require_exists=False),
            self.ROLES["cube"],
        )

    def test_numeric_index_is_diagnostic_compatibility(self):
        self.assertEqual(resolve_camera_source("2", key="linux",
                                               roles=self.ROLES), 2)

    def test_unknown_role_fails_instead_of_guessing_device_order(self):
        with self.assertRaisesRegex(ValueError, "unknown camera role"):
            resolve_camera_source("other", key="linux", roles=self.ROLES,
                                  require_exists=False)


if __name__ == "__main__":
    unittest.main()
