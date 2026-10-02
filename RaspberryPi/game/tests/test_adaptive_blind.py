"""Synthetic observations and motion only; no field calibration is implied."""

from dataclasses import asdict, replace
import contextlib
import io
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from Strategy.optimizations.adaptive_blind import (
    AdaptiveBlindProfile, NeighborObserver, NextCubeObservation, expected_blind_distance,
)
from Strategy.optimizations.trial import trial_configuration
from Strategy.execution.blind import BlindStatus
from Strategy.flows.operations import _neighbor_observer
from tests.test_fast_alignment import InertialPlant, profile as alignment_profile
from tests.test_registered_pickup_transitions import Fixture, action, calibration
from tests import test_transition_config


def block(x, *, y=100):
    return SimpleNamespace(x=x, y=0., z=150., color_name='Orange', confidence=80.,
                           quad=((x, y), (x+40, y), (x+40, y+30), (x, y+30)))


def frame(t, *xs, pose=3):
    return SimpleNamespace(captured_monotonic=t, pose_epoch=pose,
                           orange_lookahead=[block(x) for x in xs],
                           orange_lookahead_complete=True)


def profile(**changes):
    return replace(AdaptiveBlindProfile(validated=True), **changes)


def observe(observer, t, *xs, target=0., position=200., continuous=False, complete=True, **changes):
    values = dict(position_mm=position, speed_mm_s=0., target_x_mm=0.,
        yaw_deg=0., now=t, link_epoch=1, stop_generation=0, telemetry_id=round(t*1000),
        stopped=True, acceleration_mm_s2=200.)
    values.update(changes)
    result = frame(t, *xs)
    result.orange_lookahead_complete = complete
    current = block(target)
    current.continuous_row = continuous
    observer.observe(result, current, **values)


def consume(observer, t=10.02, **changes):
    values = dict(now=t, pose_epoch=3, link_epoch=1, stop_generation=0)
    values.update(changes)
    return observer.consume(**values)


def distance(p, observation, **changes):
    values = dict(position_mm=200., yaw_deg=0., now=10.1,
                  link_epoch=1, stop_generation=0, expected_pose_epoch=4, legacy_distance_mm=80.)
    values.update(changes)
    return expected_blind_distance(p, observation, **values)


class NeighborTests(unittest.TestCase):
    def test_continuous_row_uses_100mm_pitch_without_individual_boundaries(self):
        observer = NeighborObserver(profile())
        observe(observer, 10., continuous=True, complete=False)
        observe(observer, 10.02, continuous=True, complete=False)
        sample = consume(observer)
        self.assertEqual(sample.source, 'continuous_row_pitch')
        self.assertEqual(distance(profile(), sample), (100., 'continuous_row_pitch'))
        self.assertEqual(distance(profile(), sample, position_mm=210.)[0], 90.)

    def test_clear_current_cube_but_no_next_cube_uses_longer_search(self):
        observer = NeighborObserver(profile())
        observe(observer, 10., 0.)
        observe(observer, 10.02, 0.)
        sample = consume(observer)
        self.assertEqual(sample.source, 'unseen_next_search')
        self.assertEqual(distance(profile(), sample), (400., 'unseen_next_search'))
        self.assertGreater(distance(profile(), sample)[0], profile().observed_distance_limit_mm)

    def test_unresolved_or_cropped_scene_cannot_trigger_long_search(self):
        observer = NeighborObserver(profile())
        observe(observer, 10., 0., complete=False)
        observe(observer, 10.02, 0., complete=False)
        self.assertIsNone(consume(observer))
        self.assertEqual(distance(profile(), None)[0], 0.)

    def test_evidence_source_change_requires_new_confirmation(self):
        observer = NeighborObserver(profile())
        observe(observer, 10., 100.)
        observe(observer, 10.02, 0.)
        self.assertIsNone(consume(observer))

    def test_nearest_same_row_neighbor_confirms_and_consumes_once(self):
        observer = NeighborObserver(profile())
        observe(observer, 10., 200., 0., 100.)
        observe(observer, 10.02, 200., 0., 100.)
        sample = consume(observer)
        self.assertEqual(sample.next_position_mm, 300.)
        self.assertIsNone(consume(observer))
        desired, reason = distance(profile(), sample)
        self.assertAlmostEqual(desired, 100-20-5-100*math.sin(math.radians(2)))
        self.assertEqual(reason, 'observed_neighbor')

    def test_duplicates_and_one_frame_do_not_confirm(self):
        for same_frame in (True, False):
            observer = NeighborObserver(profile())
            observe(observer, 10., 100.)
            observe(observer, 10. if same_frame else 10.02, 100., telemetry_id=10000)
            self.assertIsNone(consume(observer))

    def test_lateral_motion_and_capture_latency_are_compensated(self):
        observer = NeighborObserver(profile(camera_x_to_lateral_scale=2.))
        observe(observer, 10., 50., position=200., speed_mm_s=10., now=10.01)
        observe(observer, 10.02, 49.9, target=-.1, position=200.2, speed_mm_s=10., now=10.03)
        sample = consume(observer, 10.03)
        self.assertAlmostEqual(sample.next_position_mm, 299.9)
        desired, _ = distance(observer.profile, sample, position_mm=210.)
        self.assertLess(desired, 90.)

    def test_ambiguity_nearer_invalid_gap_and_wrong_row_do_not_skip_to_far_cube(self):
        for xs in ((100., 105.), (40., 200.), (), (600.,)):
            with self.subTest(xs=xs):
                observer = NeighborObserver(profile())
                observe(observer, 10., *xs)
                observe(observer, 10.02, *xs)
                self.assertIsNone(consume(observer))
        observer = NeighborObserver(profile())
        for t in (10., 10.02):
            result = frame(t, 100.)
            result.orange_lookahead = [block(100., y=300)]
            observer.observe(result, block(0.), position_mm=200., speed_mm_s=0., target_x_mm=0.,
                yaw_deg=0., now=t, link_epoch=1, stop_generation=0, telemetry_id=t,
                stopped=True, acceleration_mm_s2=200.)
        self.assertIsNone(consume(observer))

    def test_incomplete_scene_invalidates_a_confirmed_neighbor(self):
        observer = NeighborObserver(profile())
        observe(observer, 10., 100.)
        observe(observer, 10.02, 100.)
        result = frame(10.04, 100.)
        result.orange_lookahead_complete = False
        observer.observe(result, block(0.), position_mm=200., speed_mm_s=0., target_x_mm=0.,
            yaw_deg=0., now=10.04, link_epoch=1, stop_generation=0, telemetry_id=10040,
            stopped=True, acceleration_mm_s2=200.)
        self.assertIsNone(consume(observer, 10.04))

    def test_target_switch_or_candidate_jump_restarts_confirmation(self):
        for target, next_x in ((100., 200.), (0., 200.)):
            observer = NeighborObserver(profile())
            observe(observer, 10., 100.)
            observe(observer, 10.02, next_x, target=target)
            self.assertIsNone(consume(observer))

    def test_moving_stale_future_and_still_commanded_frames_cannot_confirm(self):
        for kw in ({'speed_mm_s':30.}, {'stopped':False}, {'now':10.5}, {'now':9.}):
            observer = NeighborObserver(profile())
            observe(observer, 10., 100., **kw)
            observe(observer, 10.02, 100., **kw)
            self.assertIsNone(consume(observer))

    def test_pose_link_stop_generation_and_age_invalidate_consume(self):
        for kw in ({'pose_epoch':4}, {'link_epoch':2}, {'stop_generation':1}, {'t':11.}):
            observer = NeighborObserver(profile())
            observe(observer, 10., 100.)
            observe(observer, 10.02, 100.)
            self.assertIsNone(consume(observer, **kw))

    def test_prediction_expiry_heading_change_and_passed_target_stop_blind_travel(self):
        observer = NeighborObserver(profile())
        observe(observer, 10., 100.)
        observe(observer, 10.02, 100.)
        sample = consume(observer)
        for kw in ({'now':16.}, {'now':9.}, {'yaw_deg':5.}, {'position_mm':310.},
                   {'link_epoch':2}, {'stop_generation':1}, {'expected_pose_epoch':5}):
            with self.subTest(kw=kw):
                self.assertEqual(distance(profile(), sample, **kw)[0], 0.)
        self.assertEqual(distance(profile(), None)[0], 0.)
        self.assertEqual(distance(profile(fallback_distance_mm=40), None)[0], 40.)

    def test_invalid_profile_fails_before_observation(self):
        for kw in ({'min_gap_mm':600}, {'frame_timeout_s':.6}, {'confirm_frames':1},
                   {'fallback_distance_mm':-1}, {'camera_x_to_lateral_scale':float('nan')},
                   {'max_yaw_change_deg':11}, {'validated':1}, {'unseen_search_mm':100.},
                   {'observed_distance_limit_mm':90.}):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                profile(**kw)


class AlignmentObservationTests(unittest.TestCase):
    def test_stopped_alignment_frames_also_confirm_next_cube_without_extra_wait(self):
        plant = InertialPlant(position=100., target=100.)
        observer = NeighborObserver(profile())
        def previews(result):
            for b in result.all_blocks:
                b.quad = block(b.x).quad
            result.orange_lookahead = [block(result.all_blocks[0].x+100.)]
            result.orange_lookahead_complete = True
            return result
        plant.frame_override = previews
        self.assertTrue(plant.fast(neighbor_observer=observer))
        sample = observer.consume(now=plant.now, pose_epoch=plant.epoch,
                                  link_epoch=0, stop_generation=0)
        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample.next_position_mm, 200.)
        self.assertLess(plant.now-100., .1)

    def test_moving_handoff_collects_a_new_preview_after_camera_restoration(self):
        plant = InertialPlant(position=100., speed=20., target=150.)
        observer = NeighborObserver(profile())
        def previews(result):
            for b in result.all_blocks:
                b.quad = block(b.x).quad
            result.orange_lookahead = [block(result.all_blocks[0].x+100.)]
            result.orange_lookahead_complete = True
            return result
        plant.frame_override = previews
        self.assertTrue(plant.run(alignment=alignment_profile(), neighbor_observer=observer))
        sample = observer.consume(now=plant.now, pose_epoch=plant.epoch,
                                  link_epoch=0, stop_generation=0)
        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample.next_position_mm, 250., delta=1.)
        self.assertTrue(plant.commands[-1][3])


class AdaptiveTransitionTests(unittest.TestCase):
    def fixture(self):
        # These transition tests exercise sequencing/budgets; the inertial
        # plant above separately exercises the real alignment controller.
        self.enterContext(patch('Strategy.flows.operations._fast_align', return_value=True))
        p = replace(calibration(), next_adaptive=profile(),
                    next_blind=replace(calibration().next_blind, max_distance_mm=500.))
        f = Fixture(policy=p)
        f.env.transition_config = replace(f.env.transition_config,
                                         alignments={'ground-1/orange': alignment_profile()})
        f.control._measure_lateral_displacement_mm = lambda origin: 200.
        return f

    def run_pair(self, f, next_x, *, change_position=0., after_lift=None, continuous=False):
        observer = _neighbor_observer(f.env, 'orange')
        import time
        now = time.monotonic()
        f.robot.vision_result.pose_epoch = 3
        f.pose = 3
        for t in (now-.02, now):
            observe(observer, t, next_x, link_epoch=0, yaw_deg=12., continuous=continuous)
        original = f.control._grab_press_step
        def press(**kw):
            f.control._measure_lateral_displacement_mm = lambda origin: 200.+change_position
            return original(**kw)
        f.control._grab_press_step = press
        seen = []
        def blind(p, **kw):
            seen.append(p)
            if after_lift:
                after_lift(f)
                kw['guard']()
            kw['stop']()
            return SimpleNamespace(status=BlindStatus.BOUND_REACHED)
        f.run([action('grab_cube', 'grab.1', method='grap3', index=1),
               action('acquire_cube', 'find.2', index=2)], blind=blind)
        return seen

    def test_distance_adapts_both_ways_and_accounts_for_intervening_motion(self):
        distances = []
        for x in (90., 150.):
            f = self.fixture()
            seen = self.run_pair(f, x, change_position=10.)
            distances.append(seen[0].max_distance_mm-f.env.transition_config.pickup('ground-1', 'grap3').next_blind.braking_margin_mm)
            self.assertAlmostEqual(distances[-1], min(100., x-10-20-5-(x-10)*math.sin(math.radians(2))))
        self.assertLess(distances[0], 80.)
        self.assertGreater(distances[1], 80.)

    def test_continuous_and_unseen_branches_reach_transition_with_distinct_distances(self):
        continuous = self.fixture()
        unseen = self.fixture()
        # The synthetic calibration has 5 mm braking reserve. Travel and
        # stopping reserve must remain separate for both branches.
        self.assertEqual(self.run_pair(continuous, 0., continuous=True)[0].max_distance_mm, 105.)
        self.assertEqual(self.run_pair(unseen, 0.)[0].max_distance_mm, 405.)

    def test_original_hard_limit_and_phase_budget_are_preserved(self):
        f = self.fixture()
        p = f.env.transition_config.pickup('ground-1', 'grap3')
        f.env.transition_config = replace(f.env.transition_config, pickups={
            'ground-1/grap3': replace(p, next_blind=replace(p.next_blind, max_distance_mm=60.))})
        self.assertEqual(self.run_pair(f, 200.)[0].max_distance_mm, 60.)
        f = self.fixture()
        f.control._search_position_mm = 980.
        self.assertEqual(self.run_pair(f, 200.)[0].max_distance_mm, 20.)

    def test_missing_preview_skips_blind_and_uses_ordinary_acquisition(self):
        f = self.fixture()
        f.run([action('grab_cube', 'grab.1', method='grap3', index=1),
               action('acquire_cube', 'find.2', index=2)])
        self.assertNotIn(('blind',), f.events)
        f.warm.assert_not_called()
        f.control._find_cube.assert_called_once()

    def test_excessive_yaw_during_blind_aborts_whole_flow(self):
        f = self.fixture()
        with self.assertRaisesRegex(RuntimeError, 'adaptive blind heading'):
            self.run_pair(f, 150., after_lift=lambda f: setattr(f.robot.telem, 'yaw_deg', 20.))
        self.assertTrue(f.runtime.closed)
        f.robot.transport.emergency_stop.assert_called_once()
        f.control._find_cube.assert_not_called()


class ConfigurationTests(unittest.TestCase):
    def test_trial_is_separate_and_has_no_field_validation_claim(self):
        ordinary = trial_configuration()
        adaptive = trial_configuration(adaptive_blind=True)
        self.assertTrue(all(p.next_adaptive is None for p in ordinary.pickups.values()))
        for p in adaptive.pickups.values():
            if p.next_blind is not None:
                self.assertFalse(p.next_adaptive.validated)
                self.assertTrue(p.next_adaptive.trial_enabled)
                self.assertEqual(p.next_blind.max_distance_mm, 420.)
                self.assertEqual(p.next_adaptive.continuous_pitch_mm, 100.)
                self.assertEqual(p.next_adaptive.unseen_search_mm, 400.)

    def test_validated_config_requires_fast_alignment_and_valid_adaptive_profile(self):
        helper = test_transition_config.TransitionConfigTests()
        document = helper.document()
        document['pickups']['ground-1/grap3']['next_cube']['adaptive'] = asdict(profile())
        with self.assertRaisesRegex(ValueError, 'requires fast orange alignment'):
            helper.load(document)
        document['alignments'] = {'ground-1/orange': {
            'validated':True, 'verified_on':'2026-10-02', 'notes':'synthetic test only',
            'profile':asdict(alignment_profile())}}
        self.assertIsNotNone(helper.load(document).pickup('ground-1', 'grap3').next_adaptive)
        document['pickups']['ground-1/grap3']['next_cube']['adaptive']['validated'] = False
        with self.assertRaises(ValueError):
            helper.load(document)

    def test_cli_preview_and_invalid_combinations_do_not_connect_to_hardware(self):
        import main
        base = ['main.py', '--strategy', 'PlanA', '--show-plan']
        for flags, valid in ((['--trial-adaptive-blind'], False),
                             (['--trial-optimizations', '--trial-adaptive-blind'], False),
                             (['--trial-optimizations', '--trial-adaptive-blind', '--enable-fast-alignment'], True),
                             (['--trial-optimizations', '--trial-adaptive-blind', '--disable-fast-alignment'], False)):
            output = io.StringIO()
            with patch('sys.argv', base+flags), patch('main.Robot') as robot, \
                    contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                if valid:
                    main.main()
                    self.assertIn('Adaptive blind preview: ground-1/grap3', output.getvalue())
                else:
                    self.assertEqual(main.main(), 2)
                robot.assert_not_called()


if __name__ == '__main__':
    unittest.main()
