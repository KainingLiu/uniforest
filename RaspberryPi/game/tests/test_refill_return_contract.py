"""Exercise real inspection/refill orchestration with measured retreat feedback.

The chassis/arm are substitutes: these checks establish sequencing, not traction
or actual wall contact. No production timeout or recovery policy is changed.
"""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Strategy.flows.transitions import ActionTransitions
from Strategy.flows.model import ActionSpec
from Strategy.transition_config import TransitionConfig
from Strategy.transition_switches import TransitionSwitches


class RefillReturnTests(unittest.TestCase):
    def replay(self, first_count, *, profile='ground-1', fail_return=False):
        events = []
        route = 'ground_delivery_reverse' if profile.startswith('ground') else 'orange_depart_reverse'
        retreat_mm = 382.5 if profile.startswith('ground') else 91.75
        position = [0.]
        counts = iter((first_count, 3))
        collection = dict(profile=profile, pickups=3, exhausted=False, acquired=False,
                          origin=(7, 8, 9, 10))
        control = SimpleNamespace(config=SimpleNamespace(delivery_reverse_speed_mm_s=400.,
            post_orange_reverse_speed_mm_s=400.), _search_position_mm=650.,
            _set_cube_profile=Mock(), _recalibrate_heading_zero=lambda: events.append('rebase'))
        def move(direction, distance, speed):
            events.append(('return', direction, distance, speed))
            if fail_return:
                raise RuntimeError('return telemetry lost')
            position[0] += distance
        control._checked_move = move
        control._drive_until_wall = lambda **kw: events.append('wall')
        robot = SimpleNamespace(diagnostics=SimpleNamespace(write=Mock()),
            reset_vision_filter=lambda: events.append('vision_reset'),
            chassis=SimpleNamespace(capture_motor_positions=lambda: position[0],
                forward_displacement_mm=lambda origin: position[0]-origin))
        env = SimpleNamespace(robot=robot, control=lambda _:control, phase=Mock(),
            data={'collection':collection, 'purple_grabbed':profile.startswith('highland')},
            transition_config=TransitionConfig(switches=TransitionSwitches(
                overrides={'inspect-departure':True})))
        flag = 'ground_reverse_done' if profile.startswith('ground') else 'orange_reverse_done'
        def run_route(*args):
            if not env.data.get(flag):
                position[0] -= retreat_mm
                env.data[flag] = True
                events.append('retreat')
        env.run_route = run_route
        def begin(**kw):
            value = next(counts)
            def inspect(*, chassis_followup=None):
                if chassis_followup:
                    chassis_followup()
                events.append(('count', value))
                return value
            return SimpleNamespace(inspect=inspect,
                finish_restore=lambda: events.append('restore'), close=lambda:events.append('close'))
        robot.begin_carried_cube_inspection = begin
        transitions = ActionTransitions(env)
        transitions.start_grab = lambda _: events.append('grab')
        transitions.wait_grab_clear = Mock()
        transitions.finish_grab = Mock()
        def acquire(*args):
            events.append('acquire')
            return True
        spec = ActionSpec('inspect_cargo','inspect',profile,{'method':'grap3','exit_route':route})
        with patch('Strategy.flows.transitions.operations.acquire_cube',side_effect=acquire):
            if fail_return:
                with self.assertRaisesRegex(RuntimeError,'return telemetry lost'):
                    transitions.inspect(spec)
            elif first_count is None and profile.startswith('highland'):
                with self.assertRaisesRegex(RuntimeError,'mixed cargo count unconfirmed'):
                    transitions.inspect(spec)
            else:
                self.assertEqual(transitions.inspect(spec), 3 if first_count is not None else None)
        return events, control, collection, retreat_mm

    def test_empty_and_partial_counts_restore_return_actual_distance_reanchor_and_refill(self):
        for profile in ('ground-1','highland-1'):
            for count in (0,1,2):
                with self.subTest(profile=profile,count=count):
                    events,control,collection,distance = self.replay(count,profile=profile)
                    move = ('return','forward',distance,400.)
                    self.assertEqual(events.count(move),1)
                    self.assertLess(events.index('close'),events.index(move))
                    self.assertLess(events.index(move),events.index('wall'))
                    self.assertLess(events.index('wall'),events.index('rebase'))
                    self.assertLess(events.index('vision_reset'),events.index('acquire'))
                    self.assertEqual(events.count('grab'),3-count)
                    self.assertEqual(events.count('retreat'),2)
                    self.assertEqual(control._search_position_mm,650.)
                    self.assertEqual(collection['origin'],(7,8,9,10))

    def test_unknown_is_distinct_from_empty_and_does_not_trigger_refill(self):
        for profile in ('ground-1','highland-1'):
            events,*_ = self.replay(None,profile=profile)
            self.assertNotIn('acquire',events)
            self.assertNotIn('wall',events)

    def test_failed_return_never_reanchors_or_grabs(self):
        events,*_ = self.replay(0,fail_return=True)
        self.assertNotIn('wall',events)
        self.assertNotIn('acquire',events)
