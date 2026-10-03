"""Compare direct and classic transfers at their retained destination goals.

The exact-pose recorder is geometry only: walls and Tag observations add no
simulated displacement. Separate wheel-model tests exercise curve tracking.
"""
from dataclasses import replace
import contextlib
import io
import math
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from control.trajectory import BodyVelocity, CubicRoute, Waypoint
from Strategy.context import ExecutionContext
from Strategy.flows.factory import ActionEnvironment
from Strategy.flows.refill_routes import transfer
from Strategy.settings import PROFILES
from Strategy.transition_config import TransitionConfig


class TransferReplay:
    def __init__(self, *, offset=1800., heading=0., planned=True):
        self.pose = Waypoint(0., 0., heading)
        self.events = []
        self.offset = offset
        a = math.radians(heading)
        self.origin = Waypoint(offset*math.sin(a), -offset*math.cos(a), heading)
        self.robot = SimpleNamespace(
            telem=SimpleNamespace(yaw_deg=-heading, uptime_ms=1000),
            transport=SimpleNamespace(connected=True, emergency_stop_generation=0,
                                      emergency_stop=Mock()),
            diagnostics=SimpleNamespace(write=Mock()),
            set_collection_context=Mock(), reset_field_localization_filter=Mock(),
            actions=SimpleNamespace(hatch_open=Mock(), hatch_close=Mock()))
        self.robot.inspection_link_snapshot = lambda: (
            self.robot.telem, time.monotonic(), time.monotonic(), 0)
        self.robot.chassis = SimpleNamespace(
            set_speeds=self.stop, turn=self.turn,
            capture_motor_positions=lambda: self.pose,
            lateral_displacement_mm=self.lateral,
            measured_body_velocity=Mock(return_value=BodyVelocity()),
            follow_trajectory=Mock(side_effect=self.follow))
        self.context = ExecutionContext(self.robot, heading_zero_deg=0.)
        self.env = ActionEnvironment(self.robot, self.context,
            transition_config=TransitionConfig(motion_planning_enabled=planned))
        for name in PROFILES:
            c = self.env.control(name)
            c._checked_move = Mock(side_effect=lambda *args, key=name, **kw: self.move(key, *args, **kw))
            c._drive_until_wall = Mock(side_effect=lambda key=name, **kw: self.boundary('wall', key, kw))
            c._align_delivery_tag = Mock(side_effect=lambda key=name, **kw: self.boundary('tag', key, kw))
            c._align_delivery_tag_or_continue = Mock(side_effect=lambda key=name, **kw: self.boundary('tag', key, kw))
            original = c._recalibrate_heading_zero
            def rebase(reference_cw_deg=0., *, key=name, callback=original):
                self.boundary('rebase', key, {'reference_cw_deg': reference_cw_deg})
                callback(reference_cw_deg)
            c._recalibrate_heading_zero = Mock(side_effect=rebase)
        self.env.data.update(ground_origin=self.origin, orange_origin=self.origin,
                             collection={'origin': self.origin})
        self.env.record_transition = Mock()

    def boundary(self, kind, profile, values):
        self.context.check_active()
        self.events.append(dict(kind=kind, profile=profile, pose=self.pose, values=values))
        return True

    def stop(self, speeds):
        if list(speeds) != [0]*4:
            raise AssertionError('unexpected raw speed command')
        self.events.append(dict(kind='stop', pose=self.pose))
        return True

    def lateral(self, origin):
        a = math.radians(origin.yaw_deg)
        return -(self.pose.x_mm-origin.x_mm)*math.sin(a)+(self.pose.y_mm-origin.y_mm)*math.cos(a)

    def advance(self, local):
        start = self.pose
        a = math.radians(start.yaw_deg)
        self.pose = Waypoint(start.x_mm+math.cos(a)*local.x_mm-math.sin(a)*local.y_mm,
            start.y_mm+math.sin(a)*local.x_mm+math.cos(a)*local.y_mm, start.yaw_deg+local.yaw_deg)
        self.robot.telem.yaw_deg = -self.pose.yaw_deg

    def move(self, profile, direction, distance, speed, **kwargs):
        self.context.check_active()
        start = self.pose
        dx, dy = {'forward':(distance,0), 'backward':(-distance,0),
                  'right':(0,distance), 'left':(0,-distance)}[direction]
        self.advance(Waypoint(dx,dy,0))
        self.events.append(dict(kind='move', start=start, pose=self.pose, profile=profile,
            direction=direction, distance=distance, speed=speed, options=kwargs))

    def turn(self, angle, speed, **kwargs):
        start = self.pose
        self.advance(Waypoint(0,0,angle))
        self.events.append(dict(kind='turn', start=start, pose=self.pose, speed=speed))

    def follow(self, points, settings, *, check, initial_velocity, **kwargs):
        check()
        start = self.pose
        curve = CubicRoute(points,settings)
        samples = [curve.sample(curve.duration_s*i/120)[0] for i in range(121)]
        self.advance(points[-1])
        self.events.append(dict(kind='trajectory', start=start, pose=self.pose,
            points=points, settings=settings, samples=samples, duration=curve.duration_s))
        check()

    def run(self, source='ground', round_index=1):
        self.context.anchor = 'ground_area' if source == 'ground' else 'upper_orange_area'
        transfer(self.env, source, 'highland' if source == 'ground' else 'ground',
                 ground_profile=f'ground-{round_index}', highland_profile=f'highland-{min(round_index,2)}')
        return self


class RefillPlanningTests(unittest.TestCase):
    def setUp(self):
        silence = contextlib.redirect_stdout(io.StringIO())
        silence.__enter__()
        self.addCleanup(silence.__exit__, None, None, None)

    def assertPose(self, a, b):
        for name in ('x_mm','y_mm'):
            self.assertAlmostEqual(getattr(a,name),getattr(b,name),delta=1e-6)
        self.assertAlmostEqual((a.yaw_deg-b.yaw_deg+180)%360-180, 0., delta=1e-6)

    def test_actual_downhill_joins_have_matching_spatial_tangents(self):
        for index in (1,2,3):
            for offset in (0.,550.,1800.):
                for heading in (0.,7.):
                    r = TransferReplay(offset=offset,heading=heading).run('highland',index)
                    before,ramp,after = [e for e in r.events if e['kind']=='trajectory']
                    def direction(e, end):
                        c = CubicRoute(e['points'],e['settings'])
                        _,v = c.sample(c.duration_s-.001 if end else .001)
                        a = math.radians(e['start'].yaw_deg)
                        x,y = (math.cos(a)*v.vx_mm_s-math.sin(a)*v.vy_mm_s,
                               math.sin(a)*v.vx_mm_s+math.cos(a)*v.vy_mm_s)
                        n = math.hypot(x,y)
                        self.assertGreater(n,1e-8)
                        return x/n,y/n
                    for left,right in ((before,ramp),(ramp,after)):
                        a,b = direction(left,True),direction(right,False)
                        self.assertAlmostEqual(a[0],b[0],delta=1e-5)
                        self.assertAlmostEqual(a[1],b[1],delta=1e-5)

    def test_downhill_windows_preserve_original_destination(self):
        old = TransferReplay(planned=False).run('highland')
        new = TransferReplay().run('highland')
        original = next(e for e in old.events if e['kind']=='move' and e['distance']==2750.)
        ramp = [e for e in new.events if e['kind']=='trajectory'][1]
        self.assertAlmostEqual(abs(ramp['points'][-1].x_mm), 1450.)
        self.assertAlmostEqual(ramp['pose'].x_mm, original['pose'].x_mm+550.)
        self.assertAlmostEqual(ramp['pose'].y_mm, original['pose'].y_mm)
        self.assertPose(old.pose, new.pose)

    def test_arrival_is_curved_with_bounded_length_and_contact_facing(self):
        for source in ('ground','highland'):
            r = TransferReplay().run(source)
            e = [e for e in r.events if e['kind']=='trajectory'][-1]
            self.assertGreater(len(e['points']), 3)
            length = sum(math.hypot(b.x_mm-a.x_mm,b.y_mm-a.y_mm)
                         for a,b in zip(e['samples'],e['samples'][1:]))
            chord = math.hypot(e['points'][-1].x_mm,e['points'][-1].y_mm)
            self.assertGreater(length,chord+1.)
            self.assertLess(length,1.05*chord)
            c = CubicRoute(e['points'],e['settings'])
            _, v = c.sample(c.duration_s-.001)
            if source == 'ground':
                self.assertGreater(v.vx_mm_s,0.)
                self.assertAlmostEqual(v.vy_mm_s,0.,delta=1e-6)
            else:
                self.assertLess(v.vy_mm_s,0.)
                self.assertAlmostEqual(v.vx_mm_s,0.,delta=1e-6)

    def test_flat_lead_is_configurable_and_invalid_values_refuse_before_motion(self):
        for lead in (0.,500.,750.):
            r = TransferReplay()
            c = r.env.control('highland-1')
            c.config = replace(c.config,refill_downhill_flat_lead_mm=lead)
            r.run('highland')
            ramp = [e for e in r.events if e['kind']=='trajectory'][1]
            self.assertAlmostEqual(abs(ramp['points'][-1].x_mm),2750.-lead-550.)
        for lead in (-1.,2750.,float('nan'),True):
            r = TransferReplay()
            c = r.env.control('highland-1')
            c.config = replace(c.config,refill_downhill_flat_lead_mm=lead)
            with self.assertRaisesRegex(ValueError,'flat lead'): r.run('highland')
            self.assertFalse(any(e['kind']!='stop' for e in r.events))

    def test_exit_window_validation_and_zero_trim_fallback(self):
        r = TransferReplay()
        c = r.env.control('highland-1')
        c.config = replace(c.config,refill_downhill_flat_lead_mm=0.,refill_downhill_flat_tail_mm=0.)
        r.run('highland')
        ramp = [e for e in r.events if e['kind']=='trajectory'][1]
        self.assertAlmostEqual(abs(ramp['points'][-1].x_mm),2750.)
        for tail in (-1.,2000.,float('nan'),True,100.):
            r = TransferReplay()
            c = r.env.control('highland-1')
            c.config = replace(c.config,refill_downhill_flat_tail_mm=tail)
            with self.assertRaises(ValueError): r.run('highland')
            self.assertFalse(any(e['kind']!='stop' for e in r.events))

    def test_clearance_flows_into_a_curve_without_a_corner_stop(self):
        r = TransferReplay().run()
        self.assertFalse(any(e['kind'] == 'move' for e in r.events))
        first = next(e for e in r.events if e['kind'] == 'trajectory')
        curve = CubicRoute(first['points'], first['settings'])
        # The old retreat endpoint becomes an inside corner control, so the
        # robot starts retreating and rounds inward before that old stop.
        self.assertLess(curve.points[1].x_mm, 0.)
        self.assertAlmostEqual(curve.points[1].y_mm, 0.)
        self.assertGreater(min(p.x_mm for p in curve.points), -400.)
        for knot in curve.knots[1:-1]:
            _, a = curve.sample(knot-1e-6)
            _, b = curve.sample(knot+1e-6)
            self.assertGreater(math.hypot(a.vx_mm_s,a.vy_mm_s), 1e-3)
            self.assertLess(math.hypot(a.vx_mm_s-b.vx_mm_s,a.vy_mm_s-b.vy_mm_s), .01)

    def test_flat_paths_shorter_than_previous_bulging_version(self):
        # Measured from the previous executed interpolations at the user's
        # round-1 / 1800-mm scan position, not a drawing-layer measurement.
        for source, previous in (('ground',4640.0563901),('highland',7492.7486925)):
            r = TransferReplay().run(source)
            curves = [e for e in r.events if e['kind'] == 'trajectory']
            length = sum(math.hypot(b.x_mm-a.x_mm,b.y_mm-a.y_mm)
                         for e in curves for a,b in zip(e['samples'],e['samples'][1:]))
            self.assertLess(length, previous-400.)
            after = curves[-1]
            chord = math.hypot(after['points'][-1].x_mm,after['points'][-1].y_mm)
            arc = sum(math.hypot(b.x_mm-a.x_mm,b.y_mm-a.y_mm)
                      for a,b in zip(after['samples'],after['samples'][1:]))
            self.assertLess(arc,1.05*chord)
            # Ramp boundaries remain intentional zero-speed handoffs. Requiring
            # equal spatial tangents there caused the unwanted detours.
            for e in curves:
                curve = CubicRoute(e['points'], e['settings'])
                self.assertEqual(curve.sample(0.)[1], BodyVelocity())
                self.assertEqual(curve.sample(curve.duration_s)[1], BodyVelocity())

    def test_direct_refill_bypasses_tag6_and_intermediate_contact(self):
        for source in ('ground', 'highland'):
            r = TransferReplay().run(source)
            self.assertFalse(any(e['kind'] == 'tag' for e in r.events))
            walls = [e for e in r.events if e['kind'] == 'wall']
            self.assertEqual([e['values']['direction'] for e in walls], ['left', 'forward'])
            self.assertTrue(all(e['profile'] == ('highland-1' if source == 'ground'
                                                 else 'ground-1') for e in walls))
            self.assertFalse(any(e['kind'] == 'turn' for e in r.events))
            r.robot.reset_field_localization_filter.assert_not_called()

    def test_flat_connectors_do_not_bulge_beyond_original_corner_bounds(self):
        for source in ('ground','highland'):
            r = TransferReplay().run(source)
            curves = [e for e in r.events if e['kind'] == 'trajectory']
            before, _, after = curves
            clearance = -400. if source == 'ground' else -100.
            for e, extra in ((before, [Waypoint(clearance,0.,0.)]), (after, [])):
                bounds = [e['points'][0], e['points'][-1], *extra]
                for axis in ('x_mm','y_mm'):
                    lo, hi = min(getattr(p,axis) for p in bounds), max(getattr(p,axis) for p in bounds)
                    for p in e['samples']:
                        self.assertGreaterEqual(getattr(p,axis),lo-1e-6)
                        self.assertLessEqual(getattr(p,axis),hi+1e-6)

    def test_bounded_interpolation_hulls_for_both_directions_rounds_and_scan_positions(self):
        for source in ('ground','highland'):
            for index in (1,2,3):
                for offset in (0.,550.,750.,1800.):
                    r = TransferReplay(offset=offset).run(source,index)
                    for e in r.events:
                        if e['kind'] != 'trajectory' or not e['settings'].monotone_xy: continue
                        c = CubicRoute(e['points'],e['settings'])
                        for i,(a,b) in enumerate(zip(c.points,c.points[1:])):
                            dt = (c.knots[i+1]-c.knots[i])/3
                            for axis,name in enumerate(('x_mm','y_mm')):
                                lo,hi = sorted((getattr(a,name),getattr(b,name)))
                                # Convex hull certification covers every point
                                # of the polynomial, including between samples.
                                for value in (getattr(a,name)+dt*c.tangents[i][axis],
                                              getattr(b,name)-dt*c.tangents[i+1][axis]):
                                    self.assertGreaterEqual(value,lo-1e-6)
                                    self.assertLessEqual(value,hi+1e-6)

    def test_direct_ground_approach_does_not_visit_original_tag6_point(self):
        old = TransferReplay(planned=False).run()
        r = TransferReplay().run()
        tag = next(e['pose'] for e in old.events if e['kind'] == 'tag')
        samples = [e['pose'] for e in r.events]
        for e in r.events:
            if e['kind'] != 'trajectory': continue
            a = math.radians(e['start'].yaw_deg)
            for p in e['samples']:
                samples.append(Waypoint(e['start'].x_mm + math.cos(a)*p.x_mm - math.sin(a)*p.y_mm,
                                        e['start'].y_mm + math.sin(a)*p.x_mm + math.cos(a)*p.y_mm, 0))
        self.assertGreater(min(math.hypot(p.x_mm-tag.x_mm,p.y_mm-tag.y_mm) for p in samples), 100.)

    def test_preserves_nominal_destination_for_each_direction_round_and_scan_end(self):
        for source in ('ground','highland'):
            for index in (1,2,3):
                for offset in (0.,750.,1800.):
                    for heading in (0.,7.):
                        with self.subTest(source=source, index=index, offset=offset, heading=heading):
                            classic = TransferReplay(offset=offset,heading=heading,planned=False).run(source,index)
                            planned = TransferReplay(offset=offset,heading=heading).run(source,index)
                            self.assertPose(classic.pose,planned.pose)
                            for e in planned.events:
                                if e['kind'] == 'wall': self.assertPose(e['pose'], classic.pose)
                            self.assertEqual(classic.context.anchor,planned.context.anchor)
                            self.assertTrue(any(e['kind']=='trajectory' for e in planned.events))
                            planned.robot.actions.hatch_open.assert_not_called()

    def test_scan_position_changes_original_compensation_without_nominal_endpoint_assumption(self):
        a = TransferReplay(offset=200).run()
        b = TransferReplay(offset=1500).run()
        self.assertAlmostEqual(a.env.data['ground_lateral_mm'],200)
        self.assertAlmostEqual(b.env.data['ground_lateral_mm'],1500)
        self.assertAlmostEqual(a.pose.y_mm-b.pose.y_mm,1300)

    def test_already_retreated_does_not_repeat_clearance(self):
        from Strategy.flows.routes import ROUTES
        for source, route, profile in (
                ('ground','ground_delivery_reverse','ground-1'),
                ('highland','orange_depart_reverse','highland-1')):
            old = TransferReplay(offset=550., planned=False)
            new = TransferReplay(offset=550.)
            for r in (old, new):
                ROUTES[route](r.env, profile)
                r.events.clear()
                r.run(source)
            self.assertPose(old.pose, new.pose)
            self.assertFalse(any(e['kind'] == 'move' for e in new.events))

    def test_missing_source_origin_refuses_before_motion(self):
        r = TransferReplay()
        r.env.data.clear()
        with self.assertRaisesRegex(RuntimeError, 'encoder origin unavailable'): r.run()
        self.assertFalse(any(e['kind'] != 'stop' for e in r.events))
        self.assertEqual(r.context.anchor, 'refill_transit')

    def test_destination_wall_failure_does_not_commit_or_replay(self):
        r = TransferReplay()
        r.env.control('highland-1')._drive_until_wall.side_effect = RuntimeError('wall timeout')
        with self.assertRaisesRegex(RuntimeError, 'wall timeout'): r.run()
        self.assertEqual(r.robot.chassis.follow_trajectory.call_count, 3)
        self.assertEqual(r.context.anchor, 'refill_transit')
        self.assertNotIn('ground_reverse_done', r.env.data)
        self.assertFalse(any(e['kind'] == 'rebase' for e in r.events))

    def test_cancel_between_connectors_blocks_ramp_and_destination(self):
        r = TransferReplay()
        def cancel_after_connector(*args, **kwargs):
            r.follow(*args, **kwargs)
            r.context.cancel_event.set()
        r.robot.chassis.follow_trajectory.side_effect = cancel_after_connector
        with self.assertRaises(RuntimeError): r.run()
        self.assertEqual(r.robot.chassis.follow_trajectory.call_count, 1)
        self.assertEqual(r.context.anchor, 'refill_transit')
        self.assertFalse(any(e['kind'] == 'wall' for e in r.events))

    def test_highland_transfer_has_direct_connectors_around_protected_descent(self):
        r = TransferReplay().run('highland')
        first_wall = next(i for i,e in enumerate(r.events) if e['kind']=='wall')
        prefix = r.events[:first_wall]
        curves = [e for e in prefix if e['kind']=='trajectory']
        self.assertEqual(len(curves),3)
        self.assertFalse(any(e['kind']=='move' for e in prefix))
        self.assertEqual(curves[0]['points'][1].x_mm, -100.)
        self.assertLess(curves[0]['points'][1].x_mm, 0.)
        self.assertFalse(any(e['kind']=='turn' for e in prefix))
        self.assertAlmostEqual(curves[1]['points'][-1].x_mm, -1450.)
        self.assertAlmostEqual(curves[1]['points'][-1].y_mm, 0.)
        self.assertAlmostEqual(curves[1]['points'][-1].yaw_deg, 0.)
        self.assertLessEqual(curves[1]['settings'].max_speed_mm_s, 1000.)

    def test_ramp_stays_straight_with_zero_lateral_and_heading_reference(self):
        r = TransferReplay().run()
        ramps = [e for e in r.events if e['kind']=='trajectory' and len(e['points'])==2
                 and abs(e['points'][-1].x_mm)==2500.]
        self.assertEqual(len(ramps),1)
        self.assertTrue(all(abs(p.y_mm)<1e-9 and abs(p.yaw_deg)<1e-9 for p in ramps[0]['samples']))
        self.assertLessEqual(ramps[0]['settings'].max_speed_mm_s,1000.)

    def test_fault_does_not_replay_or_reach_later_anchor_or_commit_recipe_flags(self):
        r = TransferReplay()
        r.robot.chassis.follow_trajectory.side_effect = RuntimeError('tracking lost')
        with self.assertRaisesRegex(RuntimeError,'tracking lost'): r.run()
        self.assertFalse(any(e['kind'] in ('wall','tag') for e in r.events))
        self.assertNotIn('ground_reverse_done',r.env.data)
        self.assertEqual(r.context.anchor,'refill_transit')
        self.assertEqual(r.robot.chassis.follow_trajectory.call_count,1)

    def test_classic_switch_and_moving_tag_registry_remain_independent(self):
        r = TransferReplay(planned=False).run()
        r.robot.chassis.follow_trajectory.assert_not_called()
        r = TransferReplay()
        r.env.motion_planning._routes.completed_actions.add('original.tag')
        r.run()
        self.assertEqual(r.env.motion_planning._routes.completed_actions,{'original.tag'})

    def test_unsettled_ramp_refuses_climb_and_never_claims_destination(self):
        r = TransferReplay()
        r.robot.chassis.measured_body_velocity.return_value = BodyVelocity(0,100,0)
        with self.assertRaisesRegex(RuntimeError,'ramp entry is not settled'): r.run()
        self.assertEqual(r.context.anchor,'refill_transit')
        self.assertTrue(r.robot.transport.emergency_stop.called)
        self.assertFalse(any(e['kind']=='wall' and e['profile'].startswith('highland') for e in r.events))

    def test_generated_curves_track_in_four_wheel_model(self):
        from simulation.core import MecanumPlant
        for source in ('ground','highland'):
            for index in (1,2,3):
                r = TransferReplay().run(source,index)
                for i,e in enumerate(r.events):
                    if e['kind'] != 'trajectory': continue
                    with self.subTest(source=source, index=index, stage=i):
                        plant = MecanumPlant()
                        plant.follow_local(e['points'],profile=e['settings'],initial_velocity=BodyVelocity())
                        goal = e['points'][-1]
                        self.assertLess(math.hypot(plant.x-goal.x_mm,plant.y-goal.y_mm),12.)
                        self.assertLess(abs(plant.yaw-goal.yaw_deg),2.)
                        self.assertEqual(plant.emergency_stops,0)


if __name__ == '__main__': unittest.main()
