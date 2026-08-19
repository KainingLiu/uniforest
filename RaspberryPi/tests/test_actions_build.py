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

    def __getattr__(self, name):
        def record(*args, **kwargs):
            self.calls.append((name, args, kwargs))
        return record


class _BuildRecorder:
    def __init__(self):
        self.calls = []
        self.servo = _ServoRecorder(self.calls)

    def servo_home(self):
        self.calls.append(('servo_home',))

    def hatch_open(self):
        self.calls.append(('hatch_open',))

    def _wait(self, ms):
        self.calls.append(('wait', ms))

    def _arm_front_smooth_up(self):
        self.calls.append(('arm_front_smooth_up',))

    def _stepper_move_and_wait(self, motor, direction, cm):
        self.calls.append(('move', motor, direction, cm))

    def _stepper_dual_and_wait(self, *args, **kwargs):
        self.calls.append(('dual', args, kwargs))

    def _stepper_dual2_and_wait(self, *args, **kwargs):
        self.calls.append(('dual2', args, kwargs))

    def _stepper_dual3_and_wait(self, *args, **kwargs):
        self.calls.append(('dual3', args, kwargs))


class BuildSequenceTests(unittest.TestCase):
    def test_first_cube_retract_uses_requested_dual3_schedule(self):
        recorder = _BuildRecorder()

        Actions.build(recorder)

        self.assertIn(
            (
                'dual3',
                (
                    STEPPER_VERT, 10, STEP_DIR_FORWARD,
                    2, STEP_DIR_REVERSE,
                    STEPPER_HORIZ, 23, STEP_DIR_REVERSE,
                ),
                {
                    'other_offset_cm': 3,
                    'lead2_offset_cm': 20,
                },
            ),
            recorder.calls,
        )
        self.assertNotIn(
            ('move', STEPPER_VERT, STEP_DIR_REVERSE, 2),
            recorder.calls,
        )


if __name__ == '__main__':
    unittest.main()
