import os
import sys
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Strategy.competition import FirstTaskConfig, TAG_FOV_RETUNE_SCALE
from Strategy.task2 import Task2Config, Task2Program, Task2State


class Task2Tests(unittest.TestCase):
    def test_config_matches_planned_first_section(self):
        cfg = Task2Config()
        self.assertEqual(cfg.initial_distance_mm, 2350.0)
        self.assertEqual(cfg.initial_speed_mm_s, 500.0)
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
        self.assertEqual(cfg.wall_premove_mm, 250.0)
        self.assertEqual(cfg.wall_premove_speed_mm_s, 300.0)
        self.assertEqual(cfg.wall_speed_mm_s, 150.0)
        self.assertEqual(cfg.wall_timeout_s, 4.0)
        self.assertTrue(cfg.wall_timeout_is_success)
        self.assertEqual((cfg.align_min_x_mm, cfg.align_max_x_mm),
                         (-5.0, 5.0))
        self.assertEqual(cfg.align_target_x_mm, 0.0)
        self.assertEqual(cfg.post_grab_reverse_mm, 100.0)
        self.assertEqual(cfg.post_grab_reverse_speed_mm_s, 300.0)
        self.assertEqual(cfg.post_grab_heading_target_cw_deg, 0.0)
        self.assertEqual(cfg.post_grab_forward_base_mm, 400.0)
        self.assertEqual(cfg.post_grab_forward_speed_mm_s, 400.0)
        self.assertEqual(cfg.post_grab_wall_speed_mm_s, 200.0)
        self.assertEqual(cfg.orange_target_count, 2)
        self.assertEqual(
            (cfg.orange_align_min_x_mm, cfg.orange_align_max_x_mm),
            (-20.0, 5.0))
        self.assertEqual(cfg.orange_align_target_x_mm, 0.0)
        self.assertEqual(cfg.post_orange_reverse_mm, 500.0)
        self.assertEqual(cfg.post_orange_reverse_speed_mm_s, 300.0)
        self.assertEqual(cfg.post_orange_lateral_base_mm, 800.0)
        self.assertEqual(cfg.post_orange_lateral_speed_mm_s, 300.0)
        self.assertEqual(cfg.final_tag_id, 4)
        self.assertEqual(cfg.final_tag_distance_mm, 300.0)
        self.assertEqual(cfg.final_tag_heading_target_cw_deg, 0.0)
        self.assertEqual(cfg.final_turn_target_cw_deg, 180.0)
        self.assertEqual(cfg.build_route_distance_mm, 2100.0)
        self.assertEqual(cfg.build_route_speed_mm_s, 500.0)
        self.assertEqual(cfg.build_tag_id, 6)
        self.assertEqual(
            cfg.build_tag_distance_mm,
            FirstTaskConfig().delivery_tag_distance_mm)
        self.assertEqual(cfg.build_tag_heading_target_cw_deg, 180.0)
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
            ('move', 'forward', 2350.0, 500.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('turn_to_heading', -90.0),
            ('reset_field_localization',),
            ('tag_align', 3, 250.0, -90.0),
            ('move', 'forward', 250.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('wall', 150.0, 4.0, 'forward', 'Wall contact'),
            ('reset_vision_filter',),
            ('find_cube', {
                'color_name': 'purple',
                'min_confidence': 25.0,
                'search_direction': -1.0,
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
            ('reset_vision_filter',),
            ('find_cube', {
                'color_name': 'orange',
                'min_confidence': 25.0,
                'search_direction': 1.0,
            }),
            ('align_cube', 0.0, {
                'color_name': 'orange',
                'min_confidence': 25.0,
                'align_min_x_mm': -20.0,
                'align_max_x_mm': 5.0,
                'align_target_x_mm': 0.0,
            }),
            ('press_wall_before_grab', True),
            ('grap1',),
            ('reset_vision_filter',),
            ('find_cube', {
                'color_name': 'orange',
                'min_confidence': 25.0,
                'search_direction': 1.0,
            }),
            ('align_cube', 0.0, {
                'color_name': 'orange',
                'min_confidence': 25.0,
                'align_min_x_mm': -20.0,
                'align_max_x_mm': 5.0,
                'align_target_x_mm': 0.0,
            }),
            ('press_wall_before_grab', True),
            ('grap1',),
            ('reset_vision_filter',),
            ('move', 'backward', 500.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('move', 'right', 490.0, 300.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('reset_field_localization',),
            ('tag_align', 4, 300.0, 0.0),
            ('turn_to_heading', 180.0),
            ('move', 'forward', 2100.0, 500.0,
             {'hold_ms': 0, 'accel_ms': 200}),
            ('reset_field_localization',),
            ('tag_align', 6, 425.0, 180.0),
            ('build',),
        ])
        self.assertNotIn('distance_tolerance_mm', tag_align_options[0])
        self.assertNotIn('distance_tolerance_mm', tag_align_options[1])
        self.assertEqual(
            tag_align_options[2]['distance_tolerance_mm'],
            program.config.build_tag_distance_tolerance_mm)
        self.assertEqual(
            tag_align_options[2]['lateral_tolerance_mm'],
            program.config.build_tag_lateral_tolerance_mm)
        self.assertEqual(
            tag_align_options[2]['heading_tolerance_deg'],
            program.config.build_tag_heading_tolerance_deg)
        self.assertEqual(
            tag_align_options[2]['fine_gain_scale'],
            program.config.build_tag_fine_gain_scale)

    def test_purple_alignment_window_is_plus_or_minus_five_mm(self):
        cfg = Task2Config()
        for x_mm in (-5.0, 0.0, 5.0):
            self.assertTrue(cfg.align_min_x_mm <= x_mm <= cfg.align_max_x_mm)
        for x_mm in (-5.1, 5.1):
            self.assertFalse(cfg.align_min_x_mm <= x_mm <= cfg.align_max_x_mm)

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
