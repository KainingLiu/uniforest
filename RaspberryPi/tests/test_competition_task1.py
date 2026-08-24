import os
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Strategy.competition import (
    CompetitionProgram,
    FirstTaskConfig,
    LONG_DISTANCE_FORWARD_ACCEL_MS,
    LONG_DISTANCE_MOVE_SPEED_MM_S,
    TAG_FOV_RETUNE_SCALE,
)
from Strategy.task1 import Task1Round2Config, Task1Round2Program


def motor(rpm, current):
    return SimpleNamespace(speed_rpm=rpm, torque_current=current)


class FirstTaskTests(unittest.TestCase):
    def test_task1_motion_parameters(self):
        cfg = FirstTaskConfig()
        self.assertEqual(LONG_DISTANCE_MOVE_SPEED_MM_S, 750.0)
        self.assertEqual(LONG_DISTANCE_FORWARD_ACCEL_MS, 800)
        self.assertEqual(
            cfg.delivery_forward_speed_mm_s, LONG_DISTANCE_MOVE_SPEED_MM_S)
        self.assertEqual(
            cfg.long_distance_forward_accel_ms,
            LONG_DISTANCE_FORWARD_ACCEL_MS)
        self.assertEqual(cfg.search_speed_mm_s, 300.0)
        self.assertEqual(cfg.far_wall_speed_mm_s, 200.0)
        self.assertEqual(cfg.far_wall_timeout_s, 4.0)
        self.assertEqual(cfg.near_wall_speed_mm_s, 150.0)
        self.assertEqual(cfg.near_wall_timeout_s, 1.0)
        self.assertTrue(cfg.wall_timeout_is_success)
        self.assertEqual((cfg.align_min_x_mm, cfg.align_max_x_mm),
                         (-21.0, 2.0))
        self.assertEqual(cfg.align_target_x_mm, -1.0)
        self.assertEqual(cfg.search_max_distance_mm, 1500.0)
        self.assertEqual(cfg.target_cube_count, 3)
        self.assertEqual(cfg.orange_search_lock_x_jump_mm, 80.0)
        self.assertEqual(cfg.orange_search_confirm_frames, 2)
        self.assertEqual(cfg.delivery_forward_base_mm, 2800.0)
        self.assertEqual(cfg.delivery_turn_heading_hold_ms, 0)
        self.assertEqual(cfg.delivery_tag_id, 6)
        self.assertEqual(cfg.delivery_tag_distance_mm, 425.0)
        self.assertAlmostEqual(
            cfg.delivery_tag_distance_tolerance_mm,
            30.0 * TAG_FOV_RETUNE_SCALE)
        self.assertAlmostEqual(
            cfg.delivery_tag_lateral_tolerance_mm,
            25.0 * TAG_FOV_RETUNE_SCALE)
        self.assertEqual(cfg.delivery_heading_tolerance_deg, 3.0)
        self.assertAlmostEqual(
            cfg.delivery_tag_distance_deadband_mm,
            5.0 * TAG_FOV_RETUNE_SCALE)
        self.assertAlmostEqual(
            cfg.delivery_tag_lateral_deadband_mm,
            5.0 * TAG_FOV_RETUNE_SCALE)
        self.assertEqual(cfg.delivery_heading_deadband_deg, 0.5)
        self.assertEqual(cfg.delivery_tag_fine_align_timeout_s, 1.0)
        self.assertEqual(cfg.delivery_tag_fine_gain_scale, 1.5)
        self.assertEqual(cfg.delivery_tag_translation_median_frames, 5)
        self.assertEqual(cfg.delivery_heading_target_cw_deg, 180.0)
        self.assertEqual(cfg.unload_reverse_mm, 300.0)
        self.assertEqual(cfg.unload_final_turn_cw_deg, 180.0)
        self.assertEqual(cfg.unload_final_heading_hold_ms, 0)

    def test_preflight_captures_startup_heading_zero(self):
        robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=37.5),
            has_vision=True,
            has_field_localization=True,
        )
        program = CompetitionProgram(robot)

        program._preflight()

        self.assertEqual(program._heading_zero_deg, 37.5)

    def test_heading_zero_recalibration_uses_current_gyro_yaw(self):
        robot = SimpleNamespace(telem=SimpleNamespace(yaw_deg=4.25))
        program = CompetitionProgram(robot)
        program._heading_zero_deg = 1.0

        program._recalibrate_heading_zero()

        self.assertEqual(program._heading_zero_deg, 4.25)

    def test_stall_requires_configured_number_of_loaded_slow_motors(self):
        cfg = FirstTaskConfig()
        three_stalled = SimpleNamespace(motors=[
            motor(0, 3000), motor(10, -3000), motor(20, 2600), motor(0, 100),
        ])
        only_two = SimpleNamespace(motors=[
            motor(100, 3000), motor(100, 3000),
            motor(0, 3000), motor(10, -3000),
        ])

        self.assertTrue(CompetitionProgram._stall_sample(
            three_stalled, cfg, direction='left'))
        self.assertFalse(CompetitionProgram._stall_sample(
            only_two, cfg, direction='left'))
        self.assertTrue(CompetitionProgram._stall_sample(
            only_two, cfg, direction='forward'))

    def test_orange_selection_ignores_nearer_other_color(self):
        blocks = [
            SimpleNamespace(color_name='Purple', confidence=90, x=0, y=0, z=100),
            SimpleNamespace(color_name='Orange', confidence=70, x=50, y=0, z=300),
            SimpleNamespace(color_name='Orange', confidence=60, x=20, y=0, z=200),
        ]
        result = SimpleNamespace(
            timestamp=time.time(), all_blocks=blocks)

        selected = CompetitionProgram._orange_from_result(result, 25, 0.5)

        self.assertIs(selected, blocks[2])

    def test_stale_or_low_confidence_orange_is_rejected(self):
        low = SimpleNamespace(color_name='Orange', confidence=20,
                              x=0, y=0, z=100)
        stale = SimpleNamespace(timestamp=time.time() - 2, all_blocks=[low])
        fresh = SimpleNamespace(timestamp=time.time(), all_blocks=[low])

        self.assertIsNone(
            CompetitionProgram._orange_from_result(stale, 25, 0.5))
        self.assertIsNone(
            CompetitionProgram._orange_from_result(fresh, 25, 0.5))

    def test_visual_pid_direction_deadband_and_limit(self):
        cfg = FirstTaskConfig()
        self.assertGreater(
            CompetitionProgram._alignment_speed(50, 0, 0, cfg), 0)
        self.assertLess(
            CompetitionProgram._alignment_speed(-50, 0, 0, cfg), 0)
        self.assertEqual(
            CompetitionProgram._alignment_speed(1000, 0, 0, cfg),
            cfg.align_max_speed_mm_s)
        self.assertEqual(
            CompetitionProgram._alignment_speed(1, 0, 0, cfg),
            cfg.align_min_speed_mm_s)
        self.assertEqual(cfg.align_min_speed_mm_s, 100.0)
        self.assertEqual(cfg.align_start_speed_mm_s, 40.0)
        self.assertEqual(cfg.orange_fine_align_timeout_s, 1.0)
        self.assertEqual(cfg.align_window_hold_s, 0.20)
        self.assertEqual((cfg.orange_fine_min_x_mm,
                          cfg.orange_fine_max_x_mm), (-3.5, 1.5))
        self.assertEqual(cfg.align_max_speed_mm_s, 250.0)

    def test_alignment_requires_three_fresh_frames_in_window(self):
        class FakeChassis:
            def __init__(self):
                self.commands = []

            def set_speeds(self, rpm):
                self.commands.append(tuple(rpm))

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return [vy, vy, -vy, -vy]

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()
                now = time.time()
                self.results = iter([
                    SimpleNamespace(
                        timestamp=now + 0.5,
                        all_blocks=[SimpleNamespace(
                            color_name='Orange', confidence=80,
                            x=-10, y=0, z=200)]),
                    SimpleNamespace(
                        timestamp=now + 1,
                        all_blocks=[SimpleNamespace(
                            color_name='Orange', confidence=80,
                            x=-5, y=0, z=200)]),
                    SimpleNamespace(
                        timestamp=now + 2,
                        all_blocks=[SimpleNamespace(
                            color_name='Orange', confidence=80,
                            x=-15, y=0, z=200)]),
                    SimpleNamespace(
                        timestamp=now + 3,
                        all_blocks=[SimpleNamespace(
                            color_name='Orange', confidence=80,
                            x=-2, y=0, z=200)]),
                    SimpleNamespace(
                        timestamp=now + 4,
                        all_blocks=[SimpleNamespace(
                            color_name='Orange', confidence=80,
                            x=1, y=0, z=200)]),
                    SimpleNamespace(
                        timestamp=now + 5,
                        all_blocks=[SimpleNamespace(
                            color_name='Orange', confidence=80,
                            x=0, y=0, z=200)]),
                ])

            @property
            def vision_result(self):
                return next(self.results)

        cfg = FirstTaskConfig(align_control_period_s=0)
        robot = FakeRobot()
        program = CompetitionProgram(robot, cfg)

        aligned = program._align_orange(
            SimpleNamespace(x=-10, confidence=80))

        self.assertTrue(aligned)
        self.assertEqual(len(robot.chassis.commands), 8)
        self.assertTrue(all(command == (0, 0, 0, 0)
                            for command in robot.chassis.commands))

    def test_alignment_window_is_inclusive_and_asymmetric(self):
        cfg = FirstTaskConfig()
        self.assertTrue(cfg.align_min_x_mm <= -20 <= cfg.align_max_x_mm)
        self.assertTrue(cfg.align_min_x_mm <= -1 <= cfg.align_max_x_mm)
        self.assertTrue(cfg.align_min_x_mm <= 2 <= cfg.align_max_x_mm)
        self.assertFalse(cfg.align_min_x_mm <= -22 <= cfg.align_max_x_mm)
        self.assertFalse(cfg.align_min_x_mm <= 3 <= cfg.align_max_x_mm)

    def test_long_search_covers_full_1500_mm_range(self):
        class FakeClock:
            def __init__(self):
                self.now = 0.0

            def monotonic(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        class FakeChassis:
            def __init__(self):
                self.commands = []

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return [vy, vy, -vy, -vy]

            def set_speeds(self, rpm):
                self.commands.append(tuple(rpm))

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()

            @property
            def vision_result(self):
                return None

        cfg = FirstTaskConfig(
            search_max_distance_mm=3.6,
            search_control_period_s=0.01,
        )
        clock = FakeClock()
        robot = FakeRobot()
        program = CompetitionProgram(robot, cfg)

        with patch('Strategy.competition.time.monotonic', clock.monotonic), \
                patch('Strategy.competition.time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, 'search range'):
                program._find_orange()

        self.assertEqual(program._search_position_mm, 3.6)
        self.assertEqual(robot.chassis.commands[-1], (0, 0, 0, 0))

    def test_delivery_tag_keeps_fine_adjusting_inside_tolerance(self):
        class FakeClock:
            def __init__(self):
                self.now = 0.0

            def monotonic(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        class FakeChassis:
            def __init__(self):
                self.commands = []

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return (vx, vy, wz, 0.0)

            def set_speeds(self, values):
                self.commands.append(tuple(values))

        now = time.time()
        scale = TAG_FOV_RETUNE_SCALE
        measurements = [
            ((425.0 + 20.0 * scale) / 1000.0, 0.015 * scale)
        ] * 3 + [(0.425, 0.0)] * 6
        poses = iter([
            SimpleNamespace(
                timestamp=now + index,
                tag_solutions=(SimpleNamespace(
                    tag_id=6, distance_m=distance_m,
                    lateral_m=lateral_m,
                    relative_yaw_deg=0.0, score=0.1),),
            )
            for index, (distance_m, lateral_m)
            in enumerate(measurements, 1)
        ])

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()
                self.telem = SimpleNamespace(yaw_deg=-178.0)
                self.pose_count = 0

            @property
            def field_pose(self):
                pose = next(poses)
                if self.pose_count >= 3:
                    self.telem.yaw_deg = -180.0
                self.pose_count += 1
                return pose

        robot = FakeRobot()
        program = CompetitionProgram(
            robot, FirstTaskConfig(delivery_tag_control_period_s=0.1))
        program._heading_zero_deg = 0.0
        clock = FakeClock()

        with patch('Strategy.competition.time.monotonic', clock.monotonic), \
                patch('Strategy.competition.time.sleep', clock.sleep):
            program._align_delivery_tag()

        fine_commands = [command for command in robot.chassis.commands
                         if command != (0, 0, 0, 0)]
        self.assertTrue(fine_commands)
        self.assertGreater(fine_commands[0][0], 0.0)
        self.assertGreater(fine_commands[0][1], 0.0)
        self.assertLess(fine_commands[0][2], 0.0)
        self.assertGreater(max(command[0] for command in fine_commands), 2.3)
        self.assertGreater(max(command[1] for command in fine_commands), 2.6)
        self.assertEqual(robot.chassis.commands[-1], (0, 0, 0, 0))
        self.assertTrue(any(command != (0, 0, 0, 0)
                            for command in robot.chassis.commands[:-1]))

    def test_delivery_tag_fine_timer_survives_brief_tolerance_excursions(self):
        class FakeClock:
            def __init__(self):
                self.now = 0.0

            def monotonic(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        class FakeChassis:
            def __init__(self):
                self.commands = []

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return (vx, vy, wz, 0.0)

            def set_speeds(self, values):
                self.commands.append(tuple(values))

        now = time.time()
        scale = TAG_FOV_RETUNE_SCALE
        measurements = [
            (0.425, 0.015 * scale),
            ((425.0 - 20.0 * scale) / 1000.0, 0.06 * scale),
            ((425.0 - 20.0 * scale) / 1000.0, 0.06 * scale),
            (0.425, 0.015 * scale),
            (0.425, 0.015 * scale),
        ]
        poses = iter([
            SimpleNamespace(
                timestamp=now + index,
                tag_solutions=(SimpleNamespace(
                    tag_id=6, distance_m=distance_m,
                    lateral_m=lateral_m,
                    relative_yaw_deg=0.0, score=0.1),),
            )
            for index, (distance_m, lateral_m)
            in enumerate(measurements, 1)
        ])

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()
                self.telem = SimpleNamespace(yaw_deg=-180.0)

            @property
            def field_pose(self):
                return next(poses)

        cfg = FirstTaskConfig(
            delivery_tag_control_period_s=0.5,
            delivery_tag_translation_median_frames=1,
        )
        clock = FakeClock()
        robot = FakeRobot()
        program = CompetitionProgram(robot, cfg)
        program._heading_zero_deg = 0.0

        with patch('Strategy.competition.time.monotonic', clock.monotonic), \
                patch('Strategy.competition.time.sleep', clock.sleep):
            program._align_delivery_tag()

        self.assertGreaterEqual(clock.now, 1.0)
        self.assertEqual(robot.chassis.commands[-1], (0, 0, 0, 0))

    def test_long_search_stops_as_soon_as_orange_is_seen(self):
        class FakeClock:
            def __init__(self):
                self.now = 0.0

            def monotonic(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        class FakeChassis:
            def __init__(self):
                self.commands = []

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return [vy, vy, -vy, -vy]

            def set_speeds(self, rpm):
                self.commands.append(tuple(rpm))

        target = SimpleNamespace(
            color_name='Orange', confidence=80, x=70, y=0, z=200)
        now = time.time()
        observations = iter([
            None, None, None,
            SimpleNamespace(timestamp=now, all_blocks=[target]),
            SimpleNamespace(timestamp=now + 0.1, all_blocks=[target]),
        ])

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()

            @property
            def vision_result(self):
                return next(observations)

        clock = FakeClock()
        robot = FakeRobot()
        program = CompetitionProgram(robot)

        with patch('Strategy.competition.time.monotonic', clock.monotonic), \
                patch('Strategy.competition.time.sleep', clock.sleep):
            self.assertIs(program._find_orange(), target)

        self.assertEqual(robot.chassis.commands[-1], (0, 0, 0, 0))
        self.assertEqual(len(robot.chassis.commands), 6)
        self.assertAlmostEqual(program._search_position_mm, 24.0)

    def test_alignment_slew_prevents_startup_speed_spike(self):
        cfg = FirstTaskConfig()
        first = CompetitionProgram._slew_alignment_speed(
            250.0, 0.0, 0.05, cfg)
        second = CompetitionProgram._slew_alignment_speed(
            250.0, first, 0.05, cfg)

        self.assertEqual(first, 40.0)
        self.assertEqual(second, 55.0)
        self.assertEqual(
            CompetitionProgram._slew_alignment_speed(
                -250.0, second, 0.05, cfg),
            0.0,
        )

    def test_alignment_holds_still_on_window_edge_jitter(self):
        class FakeChassis:
            def __init__(self):
                self.commands = []

            def set_speeds(self, rpm):
                self.commands.append(tuple(rpm))

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return [vy, vy, -vy, -vy]

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()
                now = time.time()
                self.results = iter([
                    SimpleNamespace(
                        timestamp=now + index,
                        all_blocks=[SimpleNamespace(
                            color_name='Orange', confidence=80,
                            x=x, y=0, z=200)])
                    for index, x in enumerate((-5, 6, -4, -2, 1, 0), start=1)
                ])

            @property
            def vision_result(self):
                return next(self.results)

        cfg = FirstTaskConfig(
            align_confirm_frames=2,
            align_control_period_s=0,
        )
        robot = FakeRobot()
        program = CompetitionProgram(robot, cfg)

        aligned = program._align_orange(
            SimpleNamespace(x=-5, y=0, z=200, confidence=80))

        self.assertTrue(aligned)
        self.assertTrue(all(command == (0, 0, 0, 0)
                            for command in robot.chassis.commands))

    def test_alignment_tracks_same_orange_instead_of_nearest(self):
        cfg = FirstTaskConfig()
        result = SimpleNamespace(
            timestamp=time.time(),
            all_blocks=[
                SimpleNamespace(color_name='Orange', confidence=80,
                                x=45, y=0, z=500),
                SimpleNamespace(color_name='Orange', confidence=90,
                                x=-150, y=0, z=100),
            ],
        )

        tracked = CompetitionProgram._tracked_orange_from_result(
            result, 50.0, cfg)

        self.assertIs(tracked, result.all_blocks[0])

    def test_first_task_grabs_three_cubes_before_finishing(self):
        class FakeActions:
            def __init__(self):
                self.grab_count = 0

            def grap3(self):
                self.grab_count += 1

        class FakeRobot:
            def __init__(self):
                self.actions = FakeActions()
                self.reset_count = 0
                self.moves = []
                self.cube_profiles = []

            def move_chassis(self, direction, distance, speed, **kwargs):
                self.moves.append((direction, distance, speed, kwargs))
                return SimpleNamespace(timed_out=False, cancelled=False)

            def reset_vision_filter(self):
                self.reset_count += 1

            def set_cube_detection_profile(self, profile_name):
                self.cube_profiles.append(profile_name)

        cfg = FirstTaskConfig(wall_settle_s=0, post_grab_settle_s=0)
        robot = FakeRobot()
        program = CompetitionProgram(robot, cfg)
        program._drive_until_wall = lambda: None
        heading_recalibrations = []
        program._recalibrate_heading_zero = (
            lambda: heading_recalibrations.append(True))
        press_count = []
        program._press_wall_before_grab = lambda **kwargs: (
            press_count.append(kwargs.get('recalibrate_heading_zero', False)))
        program._find_orange = lambda: SimpleNamespace(x=-10, confidence=80)
        program._align_orange = lambda block: True
        program._capture_lateral_origin = lambda: 'task1-origin'
        program._measure_lateral_displacement_mm = lambda origin: 615.0

        program._run_first_task()

        self.assertEqual(robot.actions.grab_count, 3)
        self.assertEqual(robot.cube_profiles, ['default'])
        self.assertEqual(heading_recalibrations, [True])
        self.assertEqual(press_count, [True, True, True])
        self.assertEqual(robot.reset_count, 4)
        self.assertEqual(program._cube_lateral_displacement_mm, 615.0)
        self.assertEqual(robot.moves, [])

    def test_pre_grab_wall_press_uses_shorter_stall_timing(self):
        cfg = FirstTaskConfig(pre_grab_wall_settle_s=0)
        robot = SimpleNamespace(telem=SimpleNamespace(yaw_deg=2.0))
        program = CompetitionProgram(robot, cfg)
        program._heading_zero_deg = 0.0
        calls = []
        program._drive_until_wall = lambda **kwargs: calls.append(kwargs)

        program._press_wall_before_grab(recalibrate_heading_zero=True)

        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]['timeout_s'], 1.0)
        self.assertEqual(calls[0]['speed_mm_s'], 150.0)
        self.assertEqual(calls[0]['startup_grace_s'], 0.1)
        self.assertEqual(calls[0]['confirm_s'], 0.15)
        self.assertEqual(program._heading_zero_deg, 2.0)

    def test_delivery_route_order_and_parameters(self):
        events = []

        class FakeChassis:
            pass

        class FakeActions:
            def hatch_open(self):
                events.append(('hatch_open',))

            def hatch_close(self):
                events.append(('hatch_close',))

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()
                self.actions = FakeActions()

            def reset_field_localization_filter(self):
                events.append(('reset_field_localization',))

            def move_chassis(self, direction, distance, speed, **kwargs):
                events.append(('move', direction, distance, speed, kwargs))
                return SimpleNamespace(timed_out=False, cancelled=False)

        robot = FakeRobot()
        program = CompetitionProgram(robot)
        program._drive_until_wall = lambda **kwargs: events.append(
            ('wall', kwargs['speed_mm_s'], kwargs['timeout_s']))
        program._align_delivery_tag = lambda: events.append(
            ('tag_align', 6))
        program._turn_to_heading = lambda target, **kwargs: events.append(
            ('turn_to_heading', target, kwargs))
        program._cube_lateral_displacement_mm = 600.0

        program._run_delivery_route()

        self.assertEqual(events, [
            ('move', 'backward', 400.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('turn_to_heading', 90.0, {'hold_ms': 0}),
            ('move', 'forward', 2200.0, 750.0,
             {'hold_ms': 0, 'accel_ms': 800}),
            ('turn_to_heading', 180.0, {}),
            ('reset_field_localization',),
            ('tag_align', 6),
            ('wall', 200.0, 4.0),
            ('hatch_open',),
            ('move', 'backward', 300.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('hatch_close',),
            ('turn_to_heading', 360.0, {'hold_ms': 0}),
        ])

    def test_round2_delivery_route_uses_shorter_base_and_lateral_moves(self):
        events = []

        class FakeActions:
            def hatch_open(self):
                events.append(('hatch_open',))

            def hatch_close(self):
                events.append(('hatch_close',))

        class FakeRobot:
            actions = FakeActions()

            def reset_field_localization_filter(self):
                events.append(('reset_field_localization',))

            def move_chassis(self, direction, distance, speed, **kwargs):
                events.append(('move', direction, distance, speed, kwargs))
                return SimpleNamespace(timed_out=False, cancelled=False)

        cfg = Task1Round2Config()
        self.assertEqual(cfg.delivery_forward_base_mm, 2500.0)
        self.assertEqual(cfg.post_tag_lateral_right_mm, 300.0)
        self.assertEqual(cfg.pre_final_turn_lateral_left_mm, 300.0)

        program = Task1Round2Program(FakeRobot(), cfg)
        program._drive_until_wall = lambda **kwargs: events.append(
            ('wall', kwargs['speed_mm_s'], kwargs['timeout_s']))
        program._align_delivery_tag = lambda: events.append(
            ('tag_align', 6))
        program._turn_to_heading = lambda target, **kwargs: events.append(
            ('turn_to_heading', target, kwargs))
        program._cube_lateral_displacement_mm = 600.0

        program._run_delivery_route()

        self.assertEqual(events, [
            ('move', 'backward', 400.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('turn_to_heading', 90.0, {'hold_ms': 0}),
            ('move', 'forward', 1900.0, 750.0,
             {'hold_ms': 0, 'accel_ms': 800}),
            ('turn_to_heading', 180.0, {}),
            ('reset_field_localization',),
            ('tag_align', 6),
            ('move', 'right', 300.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('wall', 200.0, 4.0),
            ('hatch_open',),
            ('move', 'backward', 300.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('hatch_close',),
            ('move', 'left', 300.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('turn_to_heading', 360.0, {'hold_ms': 0}),
        ])

    def test_turn_to_heading_corrects_against_startup_zero(self):
        events = []

        class FakeChassis:
            def turn(self, angle, speed, **kwargs):
                events.append((angle, speed, kwargs))

        robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=38.0),
            chassis=FakeChassis(),
        )
        program = CompetitionProgram(robot)
        program._heading_zero_deg = 123.0

        program._turn_to_heading(90.0)

        self.assertEqual(events, [
            (5.0, 90.0, {'hold_ms': 0, 'settle_cycles': 1}),
        ])

    def test_final_turn_is_180_degrees_clockwise(self):
        events = []

        class FakeChassis:
            def turn(self, angle, speed, **kwargs):
                events.append((angle, speed, kwargs))

        robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=-180.0),
            chassis=FakeChassis(),
        )
        program = CompetitionProgram(robot)
        program._heading_zero_deg = 0.0

        program._turn_to_heading(360.0)

        self.assertEqual(events, [
            (180.0, 90.0, {'hold_ms': 0, 'settle_cycles': 1}),
        ])

    def test_delivery_route_rejects_nonpositive_dynamic_distance(self):
        class FakeRobot:
            pass

        program = CompetitionProgram(FakeRobot())
        program._cube_lateral_displacement_mm = 2800.0

        with self.assertRaisesRegex(
                RuntimeError, 'delivery forward distance must be positive'):
            program._run_delivery_route()

    def test_delivery_tag_uses_translation_pid_and_gyro_heading(self):
        class FakeChassis:
            def __init__(self):
                self.commands = []

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return (vx, vy, wz, 0.0)

            def set_speeds(self, values):
                self.commands.append(tuple(values))

        def solution(distance_m, lateral_m, relative_yaw_deg):
            return SimpleNamespace(
                tag_id=6, distance_m=distance_m, lateral_m=lateral_m,
                relative_yaw_deg=relative_yaw_deg, score=0.1)

        now = time.time()
        poses = iter([
            SimpleNamespace(timestamp=now + 1,
                            tag_solutions=(solution(
                                (425.0 + 180.0 * TAG_FOV_RETUNE_SCALE)
                                / 1000.0,
                                0.5 * TAG_FOV_RETUNE_SCALE, 0.0),)),
            SimpleNamespace(timestamp=now + 2,
                            tag_solutions=(solution(
                                0.425, 0.0, 0.0),)),
            SimpleNamespace(timestamp=now + 3,
                            tag_solutions=(solution(
                                0.425, 0.0, 0.0),)),
            SimpleNamespace(timestamp=now + 4,
                            tag_solutions=(solution(
                                0.425, 0.0, 0.0),)),
            SimpleNamespace(timestamp=now + 5,
                            tag_solutions=(solution(
                                0.425, 0.0, 0.0),)),
            SimpleNamespace(timestamp=now + 6,
                            tag_solutions=(solution(
                                0.425, 0.0, 0.0),)),
            SimpleNamespace(timestamp=now + 7,
                            tag_solutions=(solution(
                                0.425, 0.0, 0.0),)),
            SimpleNamespace(timestamp=now + 8,
                            tag_solutions=(solution(
                                0.425, 0.0, 0.0),)),
        ])

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()
                self.telem = SimpleNamespace(yaw_deg=-170.0)
                self.pose_count = 0

            @property
            def field_pose(self):
                pose = next(poses)
                if self.pose_count > 0:
                    self.telem.yaw_deg = -180.0
                self.pose_count += 1
                return pose

        cfg = FirstTaskConfig(delivery_tag_control_period_s=0)
        robot = FakeRobot()
        program = CompetitionProgram(robot, cfg)
        program._heading_zero_deg = 0.0
        program._align_delivery_tag()

        first = next(command for command in robot.chassis.commands
                     if command != (0, 0, 0, 0))
        self.assertGreater(first[0], 0.0)
        # Positive tag lateral offset requires positive chassis vy (right).
        self.assertGreater(first[1], 0.0)
        self.assertLess(first[2], 0.0)
        self.assertEqual(
            sum(command != (0, 0, 0, 0)
                for command in robot.chassis.commands),
            1,
        )
        self.assertEqual(robot.chassis.commands[-1], (0, 0, 0, 0))

    def test_delivery_tag_loss_stops_and_uses_loss_timeout(self):
        class FakeClock:
            def __init__(self):
                self.now = 0.0

            def monotonic(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        class FakeChassis:
            def __init__(self):
                self.commands = []

            def set_speeds(self, values):
                self.commands.append(tuple(values))

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()
                self.pose = SimpleNamespace(
                    timestamp=time.time(), tag_solutions=())

            @property
            def field_pose(self):
                return self.pose

        clock = FakeClock()
        robot = FakeRobot()
        cfg = FirstTaskConfig(
            delivery_tag_lost_timeout_s=0.5,
            delivery_tag_control_period_s=0.1,
        )
        program = CompetitionProgram(robot, cfg)
        with patch('Strategy.competition.time.monotonic', clock.monotonic), \
                patch('Strategy.competition.time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, 'tag 6 lost'):
                program._align_delivery_tag()

        self.assertTrue(robot.chassis.commands)
        self.assertTrue(all(command == (0, 0, 0, 0)
                            for command in robot.chassis.commands))


if __name__ == '__main__':
    unittest.main()
