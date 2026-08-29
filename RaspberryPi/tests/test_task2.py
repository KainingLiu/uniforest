import os
import sys
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Strategy.competition import (
    FirstTaskConfig,
    LONG_DISTANCE_FORWARD_ACCEL_MS,
    LONG_DISTANCE_MOVE_SPEED_MM_S,
    TAG_FOV_RETUNE_SCALE,
)
from Strategy.task2 import (
    Task2Config,
    Task2Program,
    Task2Round2Config,
    Task2Round2Program,
    Task2State,
)


class Task2Tests(unittest.TestCase):
    def test_config_matches_planned_first_section(self):
        cfg = Task2Config()
        self.assertEqual(cfg.initial_distance_mm, 2350.0)
        self.assertEqual(
            cfg.initial_speed_mm_s, LONG_DISTANCE_MOVE_SPEED_MM_S)
        self.assertEqual(
            cfg.long_distance_forward_accel_ms,
            LONG_DISTANCE_FORWARD_ACCEL_MS)
        self.assertEqual(cfg.delivery_heading_target_cw_deg, -90.0)
        self.assertEqual(cfg.delivery_tag_id, 3)
        self.assertEqual(cfg.delivery_tag_distance_mm, 250.0)
        self.assertAlmostEqual(
            cfg.delivery_tag_distance_tolerance_mm,
            30.0 * TAG_FOV_RETUNE_SCALE)
        self.assertAlmostEqual(
            cfg.delivery_tag_lateral_tolerance_mm,
            25.0 * TAG_FOV_RETUNE_SCALE)
        self.assertEqual(cfg.delivery_heading_tolerance_deg, 3.0)
        self.assertEqual(cfg.delivery_tag_fine_gain_scale, 1.5)
        self.assertEqual(cfg.delivery_tag_vision_stale_s, 0.7)
        self.assertEqual(cfg.delivery_tag_lost_timeout_s, 2.0)
        self.assertEqual(cfg.post_tag_lateral_mm, 100.0)
        self.assertEqual(cfg.post_tag_lateral_speed_mm_s, 300.0)
        self.assertEqual(cfg.wall_premove_mm, 250.0)
        self.assertEqual(cfg.wall_premove_speed_mm_s, 300.0)
        self.assertEqual(cfg.far_wall_speed_mm_s, 200.0)
        self.assertEqual(cfg.far_wall_timeout_s, 4.0)
        self.assertEqual(cfg.near_wall_speed_mm_s, 150.0)
        self.assertEqual(cfg.near_wall_timeout_s, 1.0)
        self.assertTrue(cfg.wall_timeout_is_success)
        self.assertEqual(cfg.search_speed_mm_s, 300.0)
        self.assertEqual(cfg.purple_search_max_distance_mm, 600.0)
        self.assertEqual((cfg.align_min_x_mm, cfg.align_max_x_mm),
                         (-5.0, 5.0))
        self.assertEqual(cfg.align_target_x_mm, 0.0)
        self.assertEqual((cfg.orange_fine_min_x_mm,
                          cfg.orange_fine_max_x_mm), (-3.0, 3.0))
        self.assertEqual(cfg.post_grab_reverse_mm, 100.0)
        self.assertEqual(cfg.post_grab_reverse_speed_mm_s, 300.0)
        self.assertEqual(cfg.post_grab_heading_target_cw_deg, 0.0)
        self.assertEqual(cfg.post_grab_forward_base_mm, 400.0)
        self.assertEqual(cfg.post_grab_forward_speed_mm_s, 400.0)
        self.assertTrue(cfg.left_wall_approach_enabled)
        self.assertEqual(cfg.orange_target_count, 2)
        self.assertEqual(cfg.orange_target_count_without_purple, 3)
        self.assertEqual(
            (cfg.orange_align_min_x_mm, cfg.orange_align_max_x_mm),
            (-20.0, 5.0))
        self.assertEqual(cfg.orange_align_target_x_mm, 0.0)
        self.assertEqual(cfg.orange_track_ambiguity_margin_mm, 18.0)
        self.assertEqual(cfg.post_orange_reverse_mm, 500.0)
        self.assertEqual(cfg.post_orange_reverse_speed_mm_s, 300.0)
        self.assertEqual(cfg.post_orange_lateral_base_mm, 700.0)
        self.assertEqual(cfg.post_orange_lateral_speed_mm_s, 300.0)
        self.assertEqual(cfg.final_turn_target_cw_deg, 180.0)
        self.assertEqual(cfg.build_route_distance_mm, 2100.0)
        self.assertEqual(
            cfg.build_route_speed_mm_s, LONG_DISTANCE_MOVE_SPEED_MM_S)
        self.assertEqual(cfg.build_tag_id, 6)
        self.assertEqual(cfg.build_tag_vision_stale_s, 0.7)
        self.assertEqual(cfg.build_tag_lost_timeout_s, 2.0)
        self.assertEqual(
            cfg.build_tag_distance_mm,
            FirstTaskConfig().delivery_tag_distance_mm)
        self.assertEqual(cfg.build_tag_heading_target_cw_deg, 180.0)
        self.assertEqual(cfg.post_tag6_lateral_right_mm, 0.0)
        self.assertEqual(cfg.post_tag6_lateral_speed_mm_s, 300.0)
        self.assertFalse(cfg.finish_after_build)
        self.assertEqual(
            cfg.build_tag_distance_tolerance_mm,
            FirstTaskConfig().delivery_tag_distance_tolerance_mm)
        self.assertEqual(
            cfg.build_tag_lateral_tolerance_mm,
            FirstTaskConfig().delivery_tag_lateral_tolerance_mm)
        self.assertEqual(
            cfg.build_tag_heading_tolerance_deg,
            FirstTaskConfig().delivery_heading_tolerance_deg)
        self.assertEqual(
            cfg.build_tag_fine_gain_scale,
            FirstTaskConfig().delivery_tag_fine_gain_scale)
        self.assertEqual(cfg.building_target_x_mm, 0.0)
        self.assertEqual(cfg.building_target_z_mm, 155.0)
        self.assertEqual(cfg.building_min_confidence, 35.0)
        self.assertEqual(
            (cfg.building_min_height_width_ratio,
             cfg.building_max_height_width_ratio),
             (0.35, 1.50))
        self.assertEqual(cfg.building_confirm_frames, 3)
        self.assertEqual(cfg.building_align_timeout_s, 10.0)
        self.assertEqual(cfg.building_lost_timeout_s, 3.0)
        self.assertEqual(cfg.building_track_max_x_jump_mm, 90.0)
        self.assertEqual(cfg.building_track_max_z_jump_mm, 140.0)
        self.assertEqual(cfg.building_forward_kp, 1.5)
        self.assertEqual(cfg.building_lateral_kp, 1.8)
        self.assertEqual(cfg.building_min_linear_mm_s, 100.0)
        self.assertEqual(cfg.building_max_forward_mm_s, 250.0)
        self.assertEqual(cfg.building_max_lateral_mm_s, 250.0)
        self.assertEqual(cfg.building_linear_accel_mm_s2, 500.0)
        self.assertEqual(cfg.building_track_lock_frames, 2)
        self.assertEqual(cfg.post_build_reverse_mm, 200.0)
        self.assertEqual(cfg.post_build_reverse_speed_mm_s, 300.0)
        self.assertEqual(cfg.post_build_turn_target_cw_deg, 270.0)
        self.assertEqual(cfg.post_build_route_distance_mm, 2200.0)
        self.assertEqual(
            cfg.post_build_route_speed_mm_s,
            LONG_DISTANCE_MOVE_SPEED_MM_S)
        self.assertEqual(cfg.post_build_tag_id, 1)
        self.assertEqual(cfg.post_build_tag_distance_mm, 200.0)
        self.assertEqual(cfg.post_build_tag_distance_tolerance_mm, 20.0)
        self.assertEqual(cfg.post_build_tag_lateral_tolerance_mm, 10.0)
        self.assertEqual(cfg.post_build_tag_vision_stale_s, 0.7)
        self.assertEqual(cfg.post_build_tag_lost_timeout_s, 2.5)
        self.assertEqual(cfg.post_build_tag_heading_tolerance_deg, 4.0)
        self.assertEqual(cfg.post_build_tag_heading_target_cw_deg, 270.0)
        self.assertEqual(cfg.final_right_turn_target_cw_deg, 360.0)

    def test_preflight_uses_startup_heading_and_tag_localization(self):
        robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=27.5),
            has_vision=True,
            has_field_localization=True,
        )
        program = Task2Program(robot)

        program._preflight()

        self.assertEqual(program.state, Task2State.READY)
        self.assertEqual(program._heading_zero_deg, 27.5)

    def test_partial_task_order_and_parameters(self):
        events = []
        tag_align_options = []

        class FakeRobot:
            def __init__(self):
                self.actions = SimpleNamespace(
                    grap2=lambda: events.append(('grap2',)),
                    grap1=lambda: events.append(('grap1',)),
                    build=lambda: events.append(('build',)))

            def move_chassis(self, direction, distance, speed, **kwargs):
                events.append(
                    ('move', direction, distance, speed, kwargs))
                return SimpleNamespace(timed_out=False, cancelled=False)

            def reset_field_localization_filter(self):
                events.append(('reset_field_localization',))

            def reset_vision_filter(self):
                events.append(('reset_vision_filter',))

            def set_cube_detection_profile(self, profile_name):
                events.append(('cube_profile', profile_name))

        program = Task2Program(
            FakeRobot(), Task2Config(post_grab_settle_s=0.0))
        program._turn_to_heading = lambda target: events.append(
            ('turn_to_heading', target))
        def align_delivery_tag(**kwargs):
            tag_align_options.append(kwargs.copy())
            events.append((
                'tag_align',
                kwargs.get('tag_id', program.config.delivery_tag_id),
                kwargs.get('target_distance_mm',
                           program.config.delivery_tag_distance_mm),
                kwargs.get('heading_target_cw_deg',
                           program.config.delivery_heading_target_cw_deg),
            ))

        program._align_delivery_tag = align_delivery_tag
        program._align_building = lambda: events.append(
            ('building_align',))
        program._drive_until_wall = lambda **kwargs: events.append(
            ('wall', kwargs['speed_mm_s'], kwargs['timeout_s'],
             kwargs.get('direction', 'forward'), kwargs['context']))
        program._recalibrate_heading_zero = lambda: events.append(
            ('recalibrate_heading_zero',))
        def find_cube(**kwargs):
            events.append(('find_cube', kwargs))
            if kwargs['color_name'] == 'purple':
                program._search_position_mm = 175.0
            elif program._search_position_mm == 0.0:
                program._search_position_mm = 120.0
            else:
                program._search_position_mm = 260.0
            return SimpleNamespace(x=0.0, confidence=80.0)

        program._find_cube = find_cube
        program._align_cube = lambda block, **kwargs: (
            events.append(('align_cube', block.x, kwargs)) or True)
        program._fine_align_orange = lambda block: (
            events.append(('fine_align_orange', block.x)) or True)
        program._press_wall_before_grab = lambda **kwargs: events.append(
            ('press_wall_before_grab',
             kwargs.get('recalibrate_heading_zero', False)))
        lateral_origins = iter(('purple-origin', 'orange-origin'))
        lateral_measurements = {
            'purple-origin': -190.0,
            'orange-origin': 310.0,
        }
        program._capture_lateral_origin = lambda: next(lateral_origins)
        program._measure_lateral_displacement_mm = (
            lambda origin: lateral_measurements[origin])

        program._run_partial_task()

        self.assertEqual(events, [
            ('move', 'forward', 2350.0, 750.0,
             {'hold_ms': 0, 'accel_ms': 800}),
            ('turn_to_heading', -90.0),
            ('reset_field_localization',),
            ('tag_align', 3, 250.0, -90.0),
            ('move', 'right', 100.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('move', 'forward', 250.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('wall', 200.0, 4.0, 'forward', 'Wall contact'),
            ('reset_vision_filter',),
            ('find_cube', {
                'color_name': 'purple',
                'min_confidence': 25.0,
                'search_direction': -1.0,
                'max_distance_mm': 600.0,
            }),
            ('align_cube', 0.0, {
                'color_name': 'purple',
                'min_confidence': 25.0,
            }),
            ('press_wall_before_grab', False),
            ('grap2',),
            ('move', 'backward', 100.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('turn_to_heading', 0.0),
            ('move', 'forward', 590.0, 400.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('wall', 200.0, 4.0, 'left', 'Left wall contact'),
            ('wall', 200.0, 4.0, 'forward', 'Forward wall contact'),
            ('recalibrate_heading_zero',),
            ('cube_profile', 'task2_orange'),
            ('reset_vision_filter',),
            ('find_cube', {
                'color_name': 'orange',
                'min_confidence': 25.0,
                'search_direction': 1.0,
                'lock_x_jump_mm': 80.0,
                'ambiguity_margin_mm': 18.0,
            }),
            ('align_cube', 0.0, {
                'color_name': 'orange',
                'min_confidence': 25.0,
                'align_min_x_mm': -20.0,
                'align_max_x_mm': 5.0,
                'align_target_x_mm': 0.0,
                'ambiguity_margin_mm': 18.0,
            }),
            ('fine_align_orange', 0.0),
            ('press_wall_before_grab', True),
            ('grap1',),
            ('reset_vision_filter',),
            ('find_cube', {
                'color_name': 'orange',
                'min_confidence': 25.0,
                'search_direction': 1.0,
                'lock_x_jump_mm': 80.0,
                'ambiguity_margin_mm': 18.0,
            }),
            ('align_cube', 0.0, {
                'color_name': 'orange',
                'min_confidence': 25.0,
                'align_min_x_mm': -20.0,
                'align_max_x_mm': 5.0,
                'align_target_x_mm': 0.0,
                'ambiguity_margin_mm': 18.0,
            }),
            ('fine_align_orange', 0.0),
            ('press_wall_before_grab', True),
            ('grap1',),
            ('reset_vision_filter',),
            ('cube_profile', 'default'),
            ('move', 'backward', 500.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('move', 'right', 390.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('turn_to_heading', 180.0),
            ('move', 'forward', 2100.0, 750.0,
             {'hold_ms': 0, 'accel_ms': 800}),
            ('reset_field_localization',),
            ('tag_align', 6, 425.0, 180.0),
            ('reset_vision_filter',),
            ('building_align',),
            ('build',),
            ('move', 'backward', 200.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('turn_to_heading', 270.0),
            ('move', 'forward', 2200.0, 750.0,
             {'hold_ms': 0, 'accel_ms': 800}),
            ('reset_field_localization',),
            ('tag_align', 1, 200.0, 270.0),
            ('turn_to_heading', 360.0),
        ])
        self.assertNotIn('distance_tolerance_mm', tag_align_options[0])
        self.assertEqual(
            tag_align_options[1]['distance_tolerance_mm'],
            program.config.build_tag_distance_tolerance_mm)
        self.assertEqual(
            tag_align_options[1]['lateral_tolerance_mm'],
            program.config.build_tag_lateral_tolerance_mm)
        self.assertEqual(
            tag_align_options[1]['heading_tolerance_deg'],
            program.config.build_tag_heading_tolerance_deg)
        self.assertEqual(
            tag_align_options[1]['fine_gain_scale'],
            program.config.build_tag_fine_gain_scale)
        self.assertEqual(
            tag_align_options[2]['distance_tolerance_mm'],
            program.config.post_build_tag_distance_tolerance_mm)
        self.assertEqual(
            tag_align_options[2]['lateral_tolerance_mm'],
            program.config.post_build_tag_lateral_tolerance_mm)
        self.assertEqual(
            tag_align_options[2]['heading_tolerance_deg'],
            program.config.post_build_tag_heading_tolerance_deg)
        self.assertEqual(
            tag_align_options[2]['fine_gain_scale'],
            program.config.build_tag_fine_gain_scale)

    def test_building_selector_rejects_single_cube_shape(self):
        cfg = Task2Config()
        single_cube = SimpleNamespace(
            color_name='Orange', confidence=90.0,
            x=0.0, z=150.0, height_width_ratio=0.30)
        building = SimpleNamespace(
            color_name='Orange', confidence=55.0,
            x=0.0, z=155.0, height_width_ratio=1.080)
        result = SimpleNamespace(
            timestamp=time.time(), all_blocks=[single_cube, building])
        program = Task2Program(SimpleNamespace(), cfg)

        self.assertIs(program._building_from_result(result), building)

    def test_round2_moves_right_after_tag6_and_finishes_after_build(self):
        events = []

        class FakeRobot:
            actions = SimpleNamespace(
                build=lambda: events.append(('build',)))

            def move_chassis(self, direction, distance, speed, **kwargs):
                events.append(('move', direction, distance, speed, kwargs))
                return SimpleNamespace(timed_out=False, cancelled=False)

            def reset_vision_filter(self):
                events.append(('reset_vision_filter',))

            def reset_field_localization_filter(self):
                events.append(('reset_field_localization',))

        cfg = Task2Round2Config()
        self.assertEqual(cfg.post_tag_lateral_mm, 0.0)
        self.assertFalse(cfg.left_wall_approach_enabled)
        self.assertEqual(cfg.post_orange_lateral_base_mm, 500.0)
        self.assertEqual(
            Task2Program._lateral_correction_command(
                cfg.post_orange_lateral_base_mm, 700.0),
                ('left', 200.0),
        )
        self.assertEqual(cfg.post_tag6_lateral_right_mm, 300.0)
        self.assertTrue(cfg.finish_after_build)

        program = Task2Round2Program(FakeRobot(), cfg)
        program._align_building_or_continue = lambda: (
            events.append(('building_align',)) or True)
        program._turn_to_heading = lambda target: events.append(
            ('turn_to_heading', target))
        program._align_delivery_tag = lambda **kwargs: events.append(
            ('tag_align', kwargs))

        program._run_build_phase()

        self.assertEqual(events, [
            ('move', 'right', 300.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('reset_vision_filter',),
            ('building_align',),
            ('build',),
        ])
        self.assertEqual(program.state, Task2State.BUILD)

    def test_round2_skips_post_tag3_move_and_left_wall(self):
        events = []
        program = Task2Round2Program(SimpleNamespace())
        program._drive_until_wall = lambda **kwargs: events.append(
            ('wall', kwargs.get('direction', 'forward'), kwargs['context']))
        program._recalibrate_heading_zero = lambda: events.append(
            ('recalibrate_heading_zero',))
        program._checked_move = lambda *args, **kwargs: events.append(
            ('move', args, kwargs))

        program._run_post_tag3_lateral()
        program._run_post_return_wall_approach()

        self.assertEqual(events, [
            ('wall', 'forward', 'Forward wall contact'),
            ('recalibrate_heading_zero',),
        ])

    def test_building_alignment_accepts_current_calibrated_position(self):
        class FakeChassis:
            def __init__(self):
                self.commands = []

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return (vx, vy, wz, 0.0)

            def set_speeds(self, values):
                self.commands.append(tuple(values))

        now = time.time()
        frames = iter([
            SimpleNamespace(
                timestamp=now + index,
                all_blocks=[SimpleNamespace(
                    color_name='Orange', confidence=55.0,
                    x=0.0, z=155.0, height_width_ratio=1.080)])
            for index in range(1, 4)
        ])

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()
                self.telem = SimpleNamespace(yaw_deg=-180.0)

            @property
            def vision_result(self):
                return next(frames)

        cfg = Task2Config(
            building_confirm_frames=3,
            building_median_frames=1,
            building_control_period_s=0.0,
        )
        robot = FakeRobot()
        program = Task2Program(robot, cfg)
        program._heading_zero_deg = 0.0

        program._align_building()

        self.assertEqual(robot.chassis.commands[-1], (0, 0, 0, 0))
        self.assertFalse(any(command != (0, 0, 0, 0)
                             for command in robot.chassis.commands))

    def test_building_pid_moves_toward_forward_right_offset(self):
        class FakeChassis:
            def __init__(self):
                self.commands = []

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return (vx, vy, wz, 0.0)

            def set_speeds(self, values):
                self.commands.append(tuple(values))

        now = time.time()
        observations = iter([
            (20.0, 155.0 + 30.0),
            (0.0, 155.0),
        ])

        class FakeRobot:
            def __init__(self):
                self.chassis = FakeChassis()
                self.telem = SimpleNamespace(yaw_deg=-180.0)
                self.index = 0

            @property
            def vision_result(self):
                x_mm, z_mm = next(observations)
                self.index += 1
                return SimpleNamespace(
                    timestamp=now + self.index,
                    all_blocks=[SimpleNamespace(
                        color_name='Orange', confidence=55.0,
                        x=x_mm, z=z_mm, height_width_ratio=1.080)])

        cfg = Task2Config(
            building_confirm_frames=1,
            building_median_frames=1,
            building_control_period_s=0.0,
        )
        robot = FakeRobot()
        program = Task2Program(robot, cfg)
        program._heading_zero_deg = 0.0

        program._align_building()

        motion = next(command for command in robot.chassis.commands
                      if command != (0, 0, 0, 0))
        self.assertGreater(motion[0], 0.0)
        self.assertGreater(motion[1], 0.0)

    def test_building_loss_or_timeout_stops_then_continues(self):
        for message in (
                'three-layer orange building lost before Build',
                'building visual alignment timed out'):
            commands = []
            robot = SimpleNamespace(chassis=SimpleNamespace(
                set_speeds=lambda values: commands.append(tuple(values))))
            program = Task2Program(robot)

            def fail_alignment(error=message):
                raise RuntimeError(error)

            program._align_building = fail_alignment

            self.assertFalse(program._align_building_or_continue())
            self.assertEqual(commands[-1], (0, 0, 0, 0))

    def test_unexpected_building_error_is_not_ignored(self):
        robot = SimpleNamespace(chassis=SimpleNamespace(
            set_speeds=lambda values: None))
        program = Task2Program(robot)

        def fail_alignment():
            raise RuntimeError('chassis communication failed')

        program._align_building = fail_alignment

        with self.assertRaisesRegex(RuntimeError, 'communication failed'):
            program._align_building_or_continue()

    def test_purple_alignment_window_is_plus_or_minus_five_mm(self):
        cfg = Task2Config()
        for x_mm in (-5.0, 0.0, 5.0):
            self.assertTrue(cfg.align_min_x_mm <= x_mm <= cfg.align_max_x_mm)
        for x_mm in (-5.1, 5.1):
            self.assertFalse(cfg.align_min_x_mm <= x_mm <= cfg.align_max_x_mm)

    def test_purple_search_limit_skips_grap2_and_adds_orange_target(self):
        actions = []
        robot = SimpleNamespace(actions=SimpleNamespace(
            grap2=lambda: actions.append('grap2')))
        program = Task2Program(robot)
        calls = []

        def not_found(**kwargs):
            calls.append(kwargs)
            raise RuntimeError('purple cube not found within search range')

        program._find_cube = not_found

        purple_grabbed = program._search_and_align_purple()

        self.assertFalse(purple_grabbed)
        self.assertEqual(calls, [{
            'color_name': 'purple',
            'min_confidence': 25.0,
            'search_direction': -1.0,
            'max_distance_mm': 600.0,
        }])
        self.assertEqual(
            program._orange_target_count_for_run(purple_grabbed), 3)
        self.assertEqual(program._orange_target_count_for_run(True), 2)

        program._search_and_align_purple = lambda: False
        program._press_wall_before_grab = lambda: actions.append('wall')
        self.assertFalse(program._try_grab_purple())
        self.assertEqual(actions, [])

    def test_post_orange_lateral_correction_can_reverse(self):
        self.assertEqual(
            Task2Program._lateral_correction_command(800.0, 310.0),
            ('right', 490.0),
        )
        self.assertEqual(
            Task2Program._lateral_correction_command(800.0, 1022.0),
            ('left', 222.0),
        )
        self.assertEqual(
            Task2Program._lateral_correction_command(800.0, 800.0),
            (None, 0.0),
        )

    def test_purple_search_commands_left_lateral_motion(self):
        class FakeChassis:
            def __init__(self):
                self.commands = []

            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return [vy, vy, -vy, -vy]

            def set_speeds(self, rpm):
                self.commands.append(tuple(rpm))

        purple = SimpleNamespace(
            color_name='Purple', confidence=80.0, x=20.0, y=0.0, z=200.0)
        robot = SimpleNamespace(
            chassis=FakeChassis(),
            vision_result=SimpleNamespace(
                timestamp=time.time(), all_blocks=[purple]),
        )
        program = Task2Program(robot)

        found = program._find_cube(
            color_name='purple', min_confidence=25.0,
            search_direction=-1.0)

        self.assertIs(found, purple)
        self.assertLess(robot.chassis.commands[0][0], 0.0)
        self.assertEqual(robot.chassis.commands[-1], (0, 0, 0, 0))

    def test_left_wall_timeout_defaults_to_success(self):
        class FakeChassis:
            def __init__(self):
                self.velocity_calls = []

            def mecanum_rpm(self, vx, vy, wz):
                self.velocity_calls.append((vx, vy, wz))
                return [1, 1, 1, 1]

            @staticmethod
            def set_speeds(rpm):
                pass

        chassis = FakeChassis()
        program = Task2Program(SimpleNamespace(chassis=chassis))

        program._drive_until_wall(
            timeout_s=0.0, speed_mm_s=200.0, direction='left')

        self.assertEqual(chassis.velocity_calls, [(0.0, -20.0, 0.0)])

    def test_wall_timeout_can_use_strict_failure_mode(self):
        class FakeChassis:
            @staticmethod
            def mecanum_rpm(vx, vy, wz):
                return [1, 1, 1, 1]

            @staticmethod
            def set_speeds(rpm):
                pass

        program = Task2Program(SimpleNamespace(chassis=FakeChassis()))

        with self.assertRaisesRegex(RuntimeError, 'wall contact'):
            program._drive_until_wall(
                timeout_s=0.0, timeout_is_success=False)

    def test_left_heading_produces_counterclockwise_turn(self):
        events = []

        class FakeChassis:
            def turn(self, angle, speed, **kwargs):
                events.append((angle, speed, kwargs))

        robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=0.0),
            chassis=FakeChassis(),
        )
        program = Task2Program(robot)
        program._heading_zero_deg = 0.0

        program._turn_to_heading(-90.0)

        self.assertEqual(events, [
            (-90.0, 90.0, {'hold_ms': 0, 'settle_cycles': 1}),
        ])


if __name__ == '__main__':
    unittest.main()
