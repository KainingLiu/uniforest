import unittest
from types import SimpleNamespace

from Strategy.cube_tracker import select_tracked_block
from Strategy.results import TaskResult, TaskStatus, status_from_return_code
from Strategy.tag_controller import PID, TagPidSet, profiled_command
from Strategy.wall_controller import StallConfirmation
from Strategy.wall_approach import velocity_for_direction


class StrategyModuleTests(unittest.TestCase):
    def test_wall_velocity(self):
        self.assertEqual(velocity_for_direction('right', 300), (0.0, 30.0))
        with self.assertRaises(ValueError):
            velocity_for_direction('diagonal', 300)

    def test_result_compatibility(self):
        self.assertEqual(status_from_return_code(0), TaskStatus.SUCCESS)
        self.assertEqual(status_from_return_code(7), TaskStatus.STRATEGY_FAULT)
        result = TaskResult.from_code(0, task='Task1')
        self.assertTrue(result.ok)
        self.assertEqual(result.task, 'Task1')

    def test_pid_set_reset(self):
        pids = TagPidSet(PID(1, 0, 0, 10, 100), PID(1, 0, 0, 10, 100),
                         PID(1, 0, 0, 10, 100))
        pids.distance.update(2, 0.1)
        pids.reset()
        self.assertEqual(pids.distance.integral, 0.0)

    def test_profiled_command_slows_near_target(self):
        self.assertEqual(profiled_command(20.0, 150.0,
                                          slowdown_start=100.0,
                                          creep_start=30.0,
                                          fast_speed=420.0,
                                          max_speed=420.0,
                                          min_speed=100.0), 420.0)
        near = profiled_command(20.0, 10.0, slowdown_start=100.0,
                                 creep_start=30.0, fast_speed=420.0,
                                 max_speed=420.0, min_speed=100.0)
        self.assertEqual(near, 20.0)

    def test_stall_confirmation_requires_continuous_interval(self):
        tracker = StallConfirmation()
        self.assertFalse(tracker.update(True, 1.0, 0.2))
        self.assertFalse(tracker.update(False, 1.1, 0.2))
        self.assertFalse(tracker.update(True, 1.2, 0.2))
        self.assertTrue(tracker.update(True, 1.41, 0.2))

    def test_tracker_rejects_ambiguous_candidates(self):
        now = __import__('time').time()
        cfg = SimpleNamespace(vision_stale_s=1.0,
                              align_track_max_x_jump_mm=80.0,
                              align_track_max_z_jump_mm=80.0,
                              align_track_ambiguity_margin_mm=18.0)
        block = SimpleNamespace(color_name='orange', confidence=80,
                                x=10.0, y=0.0, z=200.0)
        other = SimpleNamespace(color_name='orange', confidence=80,
                                x=20.0, y=0.0, z=200.0)
        result = SimpleNamespace(timestamp=now, all_blocks=[block, other])
        self.assertIsNone(select_tracked_block(result, 'orange', 0.0, 25.0,
                                               cfg, reference_z=200.0,
                                               ambiguity_margin_mm=18.0))


if __name__ == '__main__':
    unittest.main()
