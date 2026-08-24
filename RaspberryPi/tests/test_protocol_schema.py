import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol.schema import command_data_length, load_schema, validate_python_constants


class ProtocolSchemaTests(unittest.TestCase):
    def test_python_constants_match_schema(self):
        validate_python_constants()

    def test_schema_declares_wire_sizes(self):
        self.assertEqual(command_data_length("CMD_CHASSIS_SPEED"), 8)
        self.assertEqual(command_data_length("CMD_STEPPER_MOVE_DUAL3"), 31)
        self.assertEqual(load_schema()["telemetry"]["TELEM_FULL"]["data_length"], 80)

    def test_safety_timeout_is_explicit(self):
        self.assertEqual(load_schema()["safety"]["communication_timeout_ms"], 200)


if __name__ == "__main__":
    unittest.main()
