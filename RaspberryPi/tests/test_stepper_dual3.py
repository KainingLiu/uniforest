import struct
import unittest
import re
from pathlib import Path
from unittest.mock import Mock

from protocol import commands
from protocol.commands import encode_stepper_move_dual3
from protocol.transport import Transport
from control.stepper import Stepper


class StepperDual3ProtocolTests(unittest.TestCase):
    def test_wire_layout_matches_stm32_offsets(self):
        payload = encode_stepper_move_dual3(
            1, 4000, 0,
            4400, 1,
            0, 8800, 1,
            2000, 6800,
            400, 60, 400,
        )

        self.assertEqual(len(payload), 31)
        self.assertEqual(
            struct.unpack('>BIBIBBIBII3H', payload),
            (1, 4000, 0, 4400, 1, 0, 8800, 1,
             2000, 6800, 400, 60, 400),
        )

    def test_defaults_match_current_firmware(self):
        firmware = Path(__file__).resolve().parents[2] / 'Uniforest_A'
        if not firmware.exists():
            self.skipTest('A-board source checks run on the development checkout')
        header = (firmware / 'Core/Inc/stepper.h').read_text(encoding='utf-8')
        values = dict(re.findall(r'#define\s+(STEP_\w+)\s+(\d+)\b', header))
        for suffix, macro in [('START_DELAY', 'STEP_START_DELAY_US'),
                              ('TARGET_DELAY', 'STEP_TARGET_DELAY_US'),
                              ('ACCEL_STEPS', 'STEP_ACCEL_STEPS')]:
            self.assertEqual(getattr(commands, 'STEPPER_DEFAULT_' + suffix),
                             int(values[macro]), macro)

    def test_dual_interfaces_send_firmware_defaults_and_respect_overrides(self):
        transport = Transport()
        transport.send = Mock(return_value=True)
        stepper = Stepper(transport)
        calls = [('dual', (0, 1200, 0, 1, 1600, 1)),
                 ('dual2', (0, 1200, 0, 1, 1600, 1, 800, 0, 200)),
                 ('dual3', (0, 1200, 0, 800, 1, 1, 1600, 1, 200, 300))]
        for name, args in calls:
            for obj, prefix in [(transport, 'stepper_move_'), (stepper, 'move_')]:
                for override in [None, (900, 120, 300)]:
                    with self.subTest(interface=prefix + name, override=override):
                        kwargs = {} if override is None else dict(zip(
                            ('start_delay', 'target_delay', 'accel_steps'), override))
                        self.assertTrue(getattr(obj, prefix + name)(*args, **kwargs))
                        payload = transport.send.call_args.args[1]
                        self.assertEqual(struct.unpack('>3H', payload[-6:]),
                                         override or (400, 60, 400))

    def test_single_motor_zero_parameters_still_delegate_to_firmware(self):
        transport = Transport()
        transport.send = Mock(return_value=True)
        self.assertTrue(Stepper(transport).move(0, 0, 400))
        payload = transport.send.call_args.args[1]
        self.assertEqual(len(payload), 6)  # Short form: A-board supplies the profile.
        self.assertEqual(struct.unpack('>BBI', payload), (0, 0, 400))


if __name__ == '__main__':
    unittest.main()
