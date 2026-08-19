import struct
import unittest

from protocol.commands import encode_stepper_move_dual3


class StepperDual3ProtocolTests(unittest.TestCase):
    def test_wire_layout_matches_stm32_offsets(self):
        payload = encode_stepper_move_dual3(
            1, 4000, 0,
            4400, 1,
            0, 8800, 1,
            2000, 6800,
            1000, 100, 400,
        )

        self.assertEqual(len(payload), 31)
        self.assertEqual(
            struct.unpack('>BIBIBBIBII3H', payload),
            (1, 4000, 0, 4400, 1, 0, 8800, 1,
             2000, 6800, 1000, 100, 400),
        )


if __name__ == '__main__':
    unittest.main()
