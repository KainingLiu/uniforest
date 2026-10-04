"""PlanD site accounting, height selection and skip routes without hardware.

All one/two-layer scales in this test are synthetic and intentionally differ
from the shipped, unverified profiles.  No physical placement is simulated.
"""

import contextlib
from dataclasses import replace
import io
import itertools
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from Strategy.building_profiles import (
    load_building_profiles, validate_building_profile_targets,
)
from Strategy.common import VisualAlignmentUnavailable
from Strategy.plan_d_state import PlanDState
from Strategy.task5 import Task5State
from tests.test_task5 import task_fixture


def synthetic_profiles():
    return {
        'schema_version': 1,
        'profiles': {
            str(height): {
                'status': 'verified',
                'z_scale_mm_px': height * 5000.0,
                'calibration_date': '2026-10-04',
                'measured_distance_mm': 100.0,
                'top_row_px': height * 50.0,
                'result': 'Synthetic software fixture; no field test performed.',
            } for height in (1, 2, 3)
        },
    }


class PlanDBuildingTests(unittest.TestCase):
    def setUp(self):
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.profile_path = Path(temporary.name) / 'profiles.json'
        self.profile_path.write_text(json.dumps(synthetic_profiles()), encoding='utf-8')

    def fixture(self, a, b, *, defaults=False):
        robot, context, task, events = task_fixture()
        context.plan_d = PlanDState()
        for source, count in (('task1-1', a), ('task1-2', b)):
            context.plan_d.record_cargo(source, count)
            context.plan_d.commit_unload(source)
        if not defaults:
            task.config = replace(task.config, building_profiles_path=str(self.profile_path))
            task._route_config = task.config

        def build(height, *, chassis_followup):
            events.append(('build_on_base', height))
            # Completion is recorded only after the monitored return and the
            # firmware action have both returned, so callbacks see pending.
            site = 'A' if context.plan_d.sites['A'].topping_status == 'pending' else 'B'
            self.assertEqual(context.plan_d.sites[site].topping_status, 'pending')
            chassis_followup(lambda: events.append('action_check'))
            self.assertEqual(context.plan_d.sites[site].topping_status, 'pending')
            events.append(('build_done', height))

        robot.actions.build_on_base = Mock(side_effect=build)
        return robot, context, task, events

    def test_all_nine_site_pairs_choose_independent_heights_and_profiles(self):
        for a, b in itertools.product((1, 2, 3), repeat=2):
            with self.subTest(a=a, b=b):
                robot, context, task, events = self.fixture(a, b)
                selected_scales = []

                def align():
                    selected_scales.append(task.config.building_z_scale_mm_px)
                    self.assertTrue(task.config.building_require_top_edge)
                    block = SimpleNamespace(quad=((300, 100), (340, 100),
                                                  (340, 200), (300, 200)))
                    x, z = task._building_top_reference(block)
                    self.assertEqual(x, 0)
                    self.assertEqual(z, selected_scales[-1] / 100)

                task._align_building.side_effect = align
                self.assertEqual(task.run(), 0)
                self.assertEqual([c.args[0] for c in robot.actions.build_on_base.call_args_list], [a, b])
                self.assertEqual(selected_scales, [a * 5000, b * 5000])
                self.assertEqual([s.topping_status for s in context.plan_d.sites.values()], ['done', 'done'])
                robot.actions.build.assert_not_called()
                self.assertEqual(robot.actions.hatch_open.call_count, 2)
                self.assertIs(task.config, task._route_config)
                self.assertIs(task.state, Task5State.FINISHED)
                robot.transport.emergency_stop.assert_not_called()

    def test_skip_first_moves_directly_to_second_source_before_hatch(self):
        for count in (0, None):
            with self.subTest(count=count):
                robot, context, task, events = self.fixture(count, 2)
                self.assertEqual(task.run(), 0)
                simplified = [e for e in events if e != 'action_check']
                self.assertEqual(simplified[:4], [
                    ('move', 'left', 700, 1000, dict(hold_ms=0, accel_ms=800, route_mode=True)),
                    ('wall', 'left', 300, 2.5),
                    ('move', 'right', 300, 400, dict(hold_ms=0, accel_ms=300, route_mode=True)),
                    ('open', {'settle_ms': 200}),
                ])
                self.assertEqual(robot.actions.hatch_open.call_count, 1)
                self.assertEqual(robot.actions.build_on_base.call_args.args, (2,))
                self.assertEqual(context.plan_d.sites['A'].topping_status, 'skipped')
                self.assertEqual(context.plan_d.sites['B'].topping_status, 'done')
                self.assertFalse(any(e[:3] == ('move', 'left', 840)
                                     for e in events if isinstance(e, tuple)))

    def test_skip_second_stops_at_second_source_without_hatch_or_final_route(self):
        for count in (0, None):
            with self.subTest(count=count):
                robot, context, task, events = self.fixture(1, count)
                self.assertEqual(task.run(), 0)
                self.assertEqual(robot.actions.hatch_open.call_count, 1)
                self.assertEqual(context.plan_d.sites['A'].topping_status, 'done')
                self.assertEqual(context.plan_d.sites['B'].topping_status, 'skipped')
                moves = [e for e in events if isinstance(e, tuple) and e[0] == 'move']
                self.assertEqual(moves[-1][1:3], ('right', 300))
                self.assertFalse(any(e[1:3] == ('left', 440) for e in moves))

    def test_both_skipped_preserve_both_saved_groups(self):
        for a, b in itertools.product((0, None), repeat=2):
            with self.subTest(a=a, b=b):
                robot, context, task, events = self.fixture(a, b)
                self.assertEqual(task.run(), 0)
                robot.actions.hatch_open.assert_not_called()
                robot.actions.hatch_close.assert_not_called()
                robot.actions.build_on_base.assert_not_called()
                robot.reset_vision_filter.assert_not_called()
                self.assertEqual(len(events), 3)
                self.assertEqual([s.topping_status for s in context.plan_d.sites.values()], ['skipped', 'skipped'])

    def test_default_low_profiles_skip_and_legacy_three_layer_remains_available(self):
        profiles = load_building_profiles()
        self.assertFalse(profiles[1].available)
        self.assertFalse(profiles[2].available)
        self.assertEqual(profiles[3].status, 'legacy')
        for a, b in ((1, 3), (3, 2), (1, 2)):
            with self.subTest(a=a, b=b):
                robot, context, task, events = self.fixture(a, b, defaults=True)
                self.assertEqual(task.run(), 0)
                self.assertEqual(robot.actions.hatch_open.call_count, int(a == 3) + int(b == 3))
                skipped = [c.kwargs for c in robot.diagnostics.write.call_args_list
                           if c.args[0] == 'plan_d_topping_skipped']
                self.assertTrue(all(c['reason'] == 'calibration_missing' for c in skipped))

    def test_faults_and_keyboard_interrupt_abort_and_never_pick_second_group(self):
        for fault in ('open-stop', 'close-disconnect', 'reverse-stale', 'build', 'vision', 'keyboard'):
            with self.subTest(fault=fault):
                robot, context, task, events = self.fixture(2, 1)
                if fault == 'open-stop':
                    robot.actions.hatch_open.side_effect = lambda **kw: robot.transport.emergency_stop()
                elif fault == 'close-disconnect':
                    robot.actions.hatch_close.side_effect = lambda **kw: setattr(robot.transport, 'connected', False)
                elif fault == 'reverse-stale':
                    original = robot.move_chassis.side_effect
                    def move(direction, *args, **kwargs):
                        result = original(direction, *args, **kwargs)
                        if direction == 'backward':
                            robot.telemetry_age = 1
                        return result
                    robot.move_chassis.side_effect = move
                elif fault == 'vision':
                    task._align_building.side_effect = VisualAlignmentUnavailable('no building')
                else:
                    robot.actions.build_on_base.side_effect = (
                        KeyboardInterrupt() if fault == 'keyboard' else RuntimeError('action failure'))
                with self.assertRaises(KeyboardInterrupt if fault == 'keyboard' else RuntimeError):
                    task.run()
                self.assertEqual(context.plan_d.sites['A'].topping_status, 'interrupted')
                self.assertEqual(context.plan_d.sites['B'].topping_status, 'pending')
                self.assertEqual(robot.actions.hatch_open.call_count, 1)
                self.assertIs(task.state, Task5State.FAULT)
                self.assertIs(task.config, task._route_config)
                self.assertGreaterEqual(robot.transport.emergency_stop.call_count, 1)

    def test_skip_transfer_fault_does_not_open_either_hatch(self):
        robot, context, task, events = self.fixture(0, 2)
        original = robot.move_chassis.side_effect
        def move(direction, *args, **kwargs):
            result = original(direction, *args, **kwargs)
            if direction == 'right':
                result.cancelled = True
            return result
        robot.move_chassis.side_effect = move
        with self.assertRaises(RuntimeError):
            task.run()
        robot.actions.hatch_open.assert_not_called()
        self.assertEqual(context.plan_d.sites['A'].topping_status, 'skipped')
        self.assertEqual(context.plan_d.sites['B'].topping_status, 'pending')

    def test_action_fault_during_return_does_not_commit_success_or_start_next_load(self):
        robot, context, task, events = self.fixture(1, 3)
        def build(height, *, chassis_followup):
            checks = 0
            def monitor():
                nonlocal checks
                checks += 1
                if checks >= 3:
                    raise RuntimeError('mechanism fault during return')
            chassis_followup(monitor)
        robot.actions.build_on_base.side_effect = build
        with self.assertRaisesRegex(RuntimeError, 'mechanism fault'):
            task.run()
        self.assertEqual(context.plan_d.sites['A'].topping_status, 'interrupted')
        self.assertEqual(context.plan_d.sites['B'].topping_status, 'pending')
        self.assertEqual(robot.actions.hatch_open.call_count, 1)
        self.assertEqual(robot.actions.build_on_base.call_count, 1)

    def test_second_site_fault_preserves_first_site_completed_record(self):
        robot, context, task, events = self.fixture(3, 1)
        original = robot.actions.build_on_base.side_effect
        def build(height, *, chassis_followup):
            if height == 1:
                raise RuntimeError('second build failed')
            original(height, chassis_followup=chassis_followup)
        robot.actions.build_on_base.side_effect = build
        with self.assertRaisesRegex(RuntimeError, 'second build failed'):
            task.run()
        self.assertEqual(context.plan_d.sites['A'].topping_status, 'done')
        self.assertEqual(context.plan_d.sites['B'].topping_status, 'interrupted')

    def test_profile_lock_rejects_adjacent_target_in_same_frame(self):
        _, _, task, _ = self.fixture(1, 2)
        task.config = replace(task.config, building_z_scale_mm_px=5000)
        import time
        def block(u):
            return SimpleNamespace(color_name='orange', confidence=90, height_width_ratio=1,
                quad=((u - 20, 100), (u + 20, 100), (u + 20, 150), (u - 20, 150)))
        near, far = block(320), block(900)
        result = SimpleNamespace(timestamp=time.time(), all_blocks=[far, near])
        self.assertIs(task._building_from_result(result, (0, 50)), near)
        result.all_blocks = [far]
        self.assertIsNone(task._building_from_result(result, (0, 50)))

    def test_height_profile_rejects_missing_or_invalid_top_edge(self):
        _, _, task, _ = self.fixture(1, 2)
        task.config = replace(task.config, building_require_top_edge=True)
        import time
        for quad in (None, [], ((0, float('nan')), (30, 100), (30, 150), (0, 150)),
                     ((0, 480), (30, 480), (30, 500), (0, 500))):
            with self.subTest(quad=quad):
                block = SimpleNamespace(color_name='orange', confidence=90,
                                        height_width_ratio=1, quad=quad, x=0, z=75)
                result = SimpleNamespace(timestamp=time.time(), all_blocks=[block])
                self.assertIsNone(task._building_from_result(result))

    def test_invalid_profiles_fail_before_any_chassis_command(self):
        for field, value in (('z_scale_mm_px', float('nan')),
                             ('measured_distance_mm', True),
                             ('top_row_px', 480), ('calibration_date', '2026-02-30'),
                             ('result', ''), ('z_scale_mm_px', 42)):
            with self.subTest(field=field, value=value):
                data = synthetic_profiles()
                data['profiles']['1'][field] = value
                self.profile_path.write_text(json.dumps(data), encoding='utf-8')
                robot, context, task, events = self.fixture(1, 2)
                with self.assertRaises(ValueError):
                    task.run()
                self.assertEqual(events, [])
                robot.actions.hatch_open.assert_not_called()

    def test_out_of_frame_target_is_rejected_before_route_or_pickup(self):
        data = synthetic_profiles()
        data['profiles']['1'].update(
            measured_distance_mm=120.0, top_row_px=400.0, z_scale_mm_px=48000.0)
        self.profile_path.write_text(json.dumps(data), encoding='utf-8')
        profiles = load_building_profiles(self.profile_path)
        # The observed calibration row is valid, but placement at 75 mm
        # would need row 640 in a 480 px image.
        with self.assertRaisesRegex(ValueError, 'target top row 640'):
            validate_building_profile_targets(profiles, 75)
        robot, _, task, events = self.fixture(1, 2)
        with self.assertRaisesRegex(ValueError, 'cannot align at 75 mm'):
            task.run()
        self.assertEqual(events, [])
        robot.actions.hatch_open.assert_not_called()
        robot.actions.build_on_base.assert_not_called()

    def test_target_override_is_validated_against_every_available_profile(self):
        data = synthetic_profiles()
        data['profiles']['1'].update(
            measured_distance_mm=120.0, top_row_px=400.0, z_scale_mm_px=48000.0)
        self.profile_path.write_text(json.dumps(data), encoding='utf-8')
        robot, _, task, _ = self.fixture(1, 2)
        task.config = replace(task.config, building_target_z_mm=120.0)
        task._route_config = task.config
        self.assertEqual(task.run(), 0)
        self.assertEqual(robot.actions.build_on_base.call_count, 2)

        profiles = load_building_profiles()
        validate_building_profile_targets(profiles, 75)
        # Legacy three-layer profiles also honor overrides; unverified
        # profiles are ignored instead of manufacturing distance scales.
        for invalid in (0, -1, True, float('nan'), float('inf'), 20, 20000):
            with self.subTest(target=invalid), self.assertRaises(ValueError):
                validate_building_profile_targets(profiles, invalid)


if __name__ == '__main__':
    unittest.main()
