import os
import sys
import unittest
import re
import struct
from unittest.mock import Mock
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol import commands
from protocol.transport import Transport
from protocol.schema import command_data_length, load_schema, validate_python_constants


class ProtocolSchemaTests(unittest.TestCase):
    def test_emergency_stop_generation_changes_even_if_send_fails(self):
        transport = Transport()
        transport.send = Mock(return_value=False)
        self.assertEqual(transport.emergency_stop_generation, 0)
        self.assertFalse(transport.emergency_stop())
        self.assertEqual(transport.emergency_stop_generation, 1)
        transport.send.assert_called_once_with(commands.CMD_EMERGENCY_STOP)

    def test_python_constants_match_schema(self):
        validate_python_constants()

    def test_schema_declares_wire_sizes(self):
        self.assertEqual(command_data_length("CMD_CHASSIS_SPEED"), 8)
        self.assertEqual(command_data_length("CMD_STEPPER_MOVE_DUAL3"), 31)
        self.assertEqual(load_schema()["telemetry"]["TELEM_FULL"]["data_length"], 80)

    def test_safety_timeout_is_explicit(self):
        self.assertEqual(load_schema()["safety"]["communication_timeout_ms"], 200)

    def test_action_wire_encoding_and_receive_dispatch(self):
        self.assertEqual(commands.encode_action_start(0x12345678, 3, True),
                         bytes.fromhex('123456780301'))
        self.assertEqual(command_data_length('CMD_ACTION_START'), 6)
        self.assertEqual(command_data_length('CMD_ACTION_STATUS'), 0)
        transport = Transport()
        payload = struct.pack('>IBBBI', 0x12345678, 3, 2, 17, 0x23456789)
        transport._dispatch(commands.TELEM_ACTION, 7, payload)
        status, _ = transport.get_action_status()
        self.assertEqual(status, commands.ActionStatus(0x12345678,3,2,17,0x23456789))
        transport._dispatch(commands.TELEM_ACTION, 7, payload[:-1])
        self.assertEqual(transport.get_action_status()[0], status)

    def test_firmware_ids_and_action_sizes_match_schema(self):
        root = Path(__file__).resolve().parents[2] / 'Uniforest_A/Core'
        header = (root / 'Inc/protocol.h').read_text(encoding='utf-8')
        ids = {name: int(value, 16) for name, value in re.findall(
            r'#define\s+((?:CMD|TELEM)_\w+)\s+(0x[0-9A-Fa-f]+)', header)}
        schema = load_schema()
        for group in ('commands', 'telemetry'):
            for name, entry in schema[group].items():
                if name != 'full_fields':
                    self.assertEqual(ids[name], entry['id'], name)
        source = (root / 'Src/protocol.c').read_text(encoding='utf-8')
        start = source.split('case CMD_ACTION_START:', 1)[1].split('case CMD_ACTION_STATUS:', 1)[0]
        self.assertIn('f->data_len == 6', start)
        self.assertIn('uint8_t payload[11]', source)


if __name__ == "__main__":
    unittest.main()
