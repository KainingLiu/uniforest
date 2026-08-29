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

    def servo_home(self, settle_ms=300):
        self.calls.append(('servo_home', settle_ms))

    def hatch_open(self, settle_ms=500):
        self.calls.append(('hatch_open', settle_ms))

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
    def test_suction_starts_before_positioning_without_servo_waits(self):
        recorder = _Grap3Recorder()

        Actions.grap3(recorder)

        self.assertEqual(recorder.calls[0], ('gripper_close',))
        self.assertIn(('servo_home', 0), recorder.calls)
        self.assertIn(('hatch_open', 0), recorder.calls)
        self.assertNotIn(('wait', 400), recorder.calls)
        self.assertNotIn(('wait', 200), recorder.calls)
        self.assertNotIn(('wait', 100), recorder.calls)

    def test_release_wait_is_test_mode_only(self):
        normal = _Grap3Recorder()
        test = _Grap3Recorder()

        Actions.grap3(normal)
        Actions.grap3(test, test_mode=True)

        self.assertNotIn(('wait', 1000), normal.calls)
        self.assertEqual(test.calls[-1], ('wait', 1000))

    def test_flip_angle_is_precise(self):
        recorder = _Grap3Recorder()

        Actions.grap3(recorder)

        self.assertIn(('set_angle', 0, 52.2), recorder.calls)

    def test_retract_uses_one_continuous_three_segment_command(self):
        recorder = _Grap3Recorder()

        Actions.grap3(recorder)

        retract_start = recorder.calls.index(
            ('dual3', (
                STEPPER_VERT, 9, STEP_DIR_FORWARD,
                9, STEP_DIR_REVERSE,
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

    def test_all_vertical_9cm_segments_are_updated(self):
        recorder = _Grap3Recorder()

        Actions.grap3(recorder)

        self.assertIn(('dual', (
            STEPPER_HORIZ, 27, STEP_DIR_FORWARD,
            STEPPER_VERT, 9, STEP_DIR_REVERSE,
        ), {'m2_offset_cm': 17}), recorder.calls)
        self.assertIn(('dual', (
            STEPPER_VERT, 9, STEP_DIR_FORWARD,
            STEPPER_HORIZ, 5, STEP_DIR_REVERSE,
        ), {}), recorder.calls)

    def test_final_rise_and_retract_run_together(self):
        recorder = _Grap3Recorder()

        Actions.grap3(recorder)

        self.assertIn(
            ('dual', (
                STEPPER_VERT, 9, STEP_DIR_FORWARD,
                STEPPER_HORIZ, 5, STEP_DIR_REVERSE,
            ), {}),
            recorder.calls,
        )


if __name__ == '__main__':
    unittest.main()
