import unittest

from control.actions import Actions
from control.stepper import (
    STEPPER_HORIZ,
    STEPPER_VERT,
    STEP_DIR_FORWARD,
    STEP_DIR_REVERSE,
)


class _ServoRecorder:
    def __init__(self, calls):
        self.calls = calls

    def set_angle(self, servo_id, angle):
        self.calls.append(('set_angle', servo_id, angle))

    def gripper_close(self):
        self.calls.append(('gripper_close',))

    def gripper_open(self):
        self.calls.append(('gripper_open',))


class _Grap3Recorder:
    def __init__(self):
        self.calls = []
        self.servo = _ServoRecorder(self.calls)

    def servo_home(self):
        self.calls.append(('servo_home',))

    def hatch_open(self):
        self.calls.append(('hatch_open',))

    def _wait(self, ms):
        self.calls.append(('wait', ms))

    def _stepper_move_and_wait(self, motor, direction, cm):
        self.calls.append(('move', motor, direction, cm))

    def _stepper_dual_and_wait(self, *args, **kwargs):
        self.calls.append(('dual', args, kwargs))

    def _stepper_dual2_and_wait(self, *args, **kwargs):
        self.calls.append(('dual2', args, kwargs))

    def _stepper_dual3_and_wait(self, *args, **kwargs):
        self.calls.append(('dual3', args, kwargs))


class Grap3SequenceTests(unittest.TestCase):
    def test_retract_uses_one_continuous_three_segment_command(self):
        recorder = _Grap3Recorder()

        Actions.grap3(recorder)

        retract_start = recorder.calls.index(
            ('dual3', (
                STEPPER_VERT, 10, STEP_DIR_FORWARD,
                11, STEP_DIR_REVERSE,
                STEPPER_HORIZ, 22, STEP_DIR_REVERSE,
            ), {
                'other_offset_cm': 5,
                'lead2_offset_cm': 14,
            }))
        self.assertEqual(
            sum(1 for call in recorder.calls if call[0] == 'dual3'),
            1,
        )
        self.assertNotIn(
            ('move', STEPPER_VERT, STEP_DIR_FORWARD, 5),
            recorder.calls,
        )
        self.assertGreater(retract_start, 0)


if __name__ == '__main__':
    unittest.main()
