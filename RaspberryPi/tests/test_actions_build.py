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

    def _arm_front_smooth_up(self, **kwargs):
        self.calls.append(('arm_front_smooth_up', kwargs))

    def _stepper_move_and_wait(self, motor, direction, cm):
        self.calls.append(('move', motor, direction, cm))

    def _stepper_dual_and_wait(self, *args, **kwargs):
        self.calls.append(('dual', args, kwargs))

    def _stepper_dual2_and_wait(self, *args, **kwargs):
        self.calls.append(('dual2', args, kwargs))

    def _stepper_dual3_and_wait(self, *args, **kwargs):
        self.calls.append(('dual3', args, kwargs))


class BuildSequenceTests(unittest.TestCase):
    def test_build_uses_updated_pick_place_geometry_and_angles(self):
        recorder = _BuildRecorder()

        Actions.build(recorder)

        self.assertIn(
            ('move', STEPPER_HORIZ, STEP_DIR_FORWARD, 4),
            recorder.calls,
        )
        self.assertIn(
            ('dual', (
                STEPPER_HORIZ, 19, STEP_DIR_FORWARD,
                STEPPER_VERT, 19, STEP_DIR_REVERSE,
            ), {'m2_offset_cm': 3}),
            recorder.calls,
        )
        self.assertIn(
            ('dual3', (
                    STEPPER_VERT, 10, STEP_DIR_FORWARD,
                2, STEP_DIR_REVERSE,
                STEPPER_HORIZ, 23, STEP_DIR_REVERSE,
            ), {'other_offset_cm': 3, 'lead2_offset_cm': 20}),
            recorder.calls,
        )
        self.assertIn(
            ('dual2', (
                STEPPER_HORIZ, 23, STEP_DIR_FORWARD,
                STEPPER_VERT, 2, STEP_DIR_FORWARD,
                5, STEP_DIR_REVERSE,
            ), {'ph2_offset_cm': 21}),
            recorder.calls,
        )
        self.assertIn(
            ('dual', (
                STEPPER_VERT, 4, STEP_DIR_FORWARD,
                STEPPER_HORIZ, 23, STEP_DIR_REVERSE,
            ), {}),
            recorder.calls,
        )
        self.assertIn(
            ('move', STEPPER_VERT, STEP_DIR_REVERSE, 11.5),
            recorder.calls,
        )
        self.assertIn(
            ('move', STEPPER_VERT, STEP_DIR_FORWARD, 20.5),
            recorder.calls,
        )

        self.assertEqual(
            sum(1 for call in recorder.calls
                if call[0] == 'arm_front_smooth_up'),
            3,
        )
        self.assertIn(('set_angle', (0, 102.2), {}), recorder.calls)
        self.assertIn(('set_angle', (0, 97.2), {}), recorder.calls)
        self.assertIn(('set_angle', (1, 100), {}), recorder.calls)

        pickup_return = recorder.calls.index(('set_angle', (0, 95.2), {}))
        first_lift = recorder.calls.index(('arm_front_smooth_up', {'target': 3}))
        self.assertEqual(first_lift, pickup_return + 1)

        second_return = recorder.calls.index(
            ('set_angle', (0, 97.2), {}), first_lift + 1)
        second_lift = recorder.calls.index(
            ('arm_front_smooth_up', {}), first_lift + 1)
        self.assertEqual(second_lift, second_return + 1)

    def test_first_cube_retract_uses_updated_dual3_schedule(self):
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
