"""Opt-in capture windows against inertial motion and delayed observations.

Every numeric motion profile in this file is synthetic, not field calibration.
"""

from dataclasses import asdict, replace
import contextlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

from Strategy.optimizations.fast_alignment import (
    AlignmentFault, CaptureWindow, FastAlignmentProfile, align_cube,
)
from Strategy.settings import GroundCollectionConfig, HighlandCollectionConfig
from Strategy.transition_config import TransitionConfig
from Strategy.flows import operations
from tests.test_functional_operations import operation_fixture, spec
from tests.test_pickup_motion import Plant, profile as moving_profile


def profile(**changes):
    values = dict(capture_tolerance_mm=8.0, mechanical_tolerance_mm=10.0,
                  position_uncertainty_mm=2.0, max_speed_mm_s=100.0,
                  acceleration_mm_s2=200.0, braking_mm_s2=200.0,
                  settled_speed_mm_s=1.0, max_duration_s=4.0,
                  max_travel_mm=250.0, frame_timeout_s=.15,
                  telemetry_timeout_s=.15, tick_s=.02,
                  max_command_delay_s=.01, confirm_frames=2,
                  max_reversals=1, validated=True)
    values.update(changes)
    return FastAlignmentProfile(**values)


class InertialPlant(Plant):
    def __init__(self, **kwargs):
        kwargs.setdefault('speed',0.0)
        super().__init__(**kwargs)
        self.desired = self.speed
        self.history = [(self.now,self.position)]
        self.link_epoch = 0
        self.received_offset = 0.0
        self.color = 'orange'
        self.noise = 0.0
        self.freeze_position = False

    def inspection_link_snapshot(self):
        return self.telem,self.now-self.received_offset,self.now,self.link_epoch

    @property
    def vision_result(self):
        frame = super().vision_result
        capture = frame.captured_monotonic
        past = next((p for t,p in reversed(self.history) if t <= capture),self.history[0][1])
        if frame.all_blocks:
            frame.all_blocks[0].x += self.position-past+self.noise
            frame.all_blocks[0].color_name = self.color
        return frame

    def command(self, values):
        stopped = list(values) == [0,0,0,0]
        self.desired = 0.0 if stopped else values[1]*10.0/self.chassis.lateral_distance_scale
        self.commands.append((self.now,self.desired,0.0 if stopped else values[2],stopped))
        return not self.reject_command

    def sleep(self, seconds):
        new_speed = self.speed+max(-200*seconds,min(200*seconds,self.desired-self.speed))
        if not self.freeze_position:
            self.position += (self.speed+new_speed)*.5*seconds
        self.speed = new_speed
        self.now += seconds
        self.history.append((self.now,self.position))
        self.update_speed()
        if not self.freeze_uptime:
            self.telem.uptime_ms += round(seconds*1000)

    def fast(self, **changes):
        options = dict(color_name=self.color,phase_origin=(0,0,0,0),profile=profile(),
                       clock=lambda:self.now,sleep=self.sleep)
        options.update(changes)
        seed = self.vision_result.all_blocks
        return align_cube(self,seed[0] if seed else None,**options)


class CaptureWindowTests(unittest.TestCase):
    def test_inside_window_does_not_center_again(self):
        controller = CaptureWindow(profile())
        self.assertFalse(controller.update(7,0,0,.02,evidence=(1,1)).complete)
        decision = controller.update(7,0,0,.02,evidence=(2,2))
        self.assertTrue(decision.complete)
        self.assertEqual(decision.speed_mm_s,0)

    def test_duplicate_image_or_telemetry_cannot_confirm(self):
        for evidence in ((1,2),(2,1)):
            controller = CaptureWindow(profile())
            controller.update(0,0,0,.02,evidence=(1,1))
            self.assertFalse(controller.update(0,0,0,.02,evidence=evidence).complete)

    def test_position_inside_window_while_moving_cannot_confirm(self):
        controller = CaptureWindow(profile(),80)
        for i in range(8):
            self.assertFalse(controller.update(5,80,.02,.02,evidence=(i,i)).complete)

    def test_brakes_for_predicted_landing_before_entering_window(self):
        controller = CaptureWindow(profile(),60)
        decision = controller.update(15,60,.02,.02,evidence=(1,1))
        self.assertTrue(controller.latched)
        self.assertLess(decision.speed_mm_s,60)
        self.assertFalse(decision.complete)

    def test_one_outside_frame_does_not_kick_latched_chassis(self):
        controller = CaptureWindow(profile())
        controller.update(7,0,0,.02,evidence=(1,1))
        self.assertEqual(controller.update(8.5,0,0,.02,evidence=(2,2)).speed_mm_s,0)
        self.assertFalse(controller.update(7,0,0,.02,evidence=(3,3)).complete)
        self.assertTrue(controller.update(7,0,0,.02,evidence=(4,4)).complete)

    def test_bounded_reversal_does_not_emit_second_direction(self):
        controller = CaptureWindow(profile(max_reversals=0),10)
        decision = None
        for i in range(8):
            decision = controller.update(-80,0,0,.02,evidence=(i,i))
            self.assertGreaterEqual(decision.speed_mm_s,0)
            if decision.exhausted:
                break
        self.assertTrue(decision.exhausted)

    def test_invalid_calibration_rejected(self):
        for value in ({'capture_tolerance_mm':9}, {'braking_mm_s2':float('nan')},
                      {'confirm_frames':1}, {'max_reversals':True}, {'tick_s':.1},
                      {'max_speed_mm_s':float('inf')}):
            with self.subTest(value=value),self.assertRaises(ValueError):
                profile(**value)


class FastAlignmentMotionTests(unittest.TestCase):
    def test_stopped_seven_mm_error_is_accepted_without_movement(self):
        plant = InertialPlant(target=107)
        self.assertTrue(plant.fast())
        self.assertTrue(all(speed==0 for _,speed,_,_ in plant.commands))
        self.assertEqual(plant.position,100)

    def test_inertial_approach_finishes_inside_mechanical_window(self):
        for delay in (0.0,.06):
            for offset in (-60,60):
                with self.subTest(delay=delay,offset=offset):
                    plant = InertialPlant(target=100+offset)
                    plant.capture_delay=delay
                    self.assertTrue(plant.fast())
                    self.assertLessEqual(abs(plant.position-plant.target),8)
                    self.assertLessEqual(abs(plant.speed),1)
                    self.assertTrue(plant.commands[-1][3])

    def test_lost_target_stops_and_returns_search_outcome(self):
        plant = InertialPlant(target=None)
        self.assertFalse(plant.fast())
        self.assertTrue(plant.commands[-1][3])

    def test_distinct_fresh_frames_are_required(self):
        plant = InertialPlant(target=100)
        plant.freeze_frame=True
        self.assertFalse(plant.fast())

    def test_pose_change_is_a_hard_fault(self):
        plant = InertialPlant(target=140)
        original = plant.sleep
        def change(seconds):
            original(seconds)
            plant.epoch += 1
        plant.sleep=change
        with self.assertRaises(AlignmentFault):plant.fast()
        self.assertTrue(plant.commands[-1][3])

    def test_link_telemetry_and_send_faults_raise(self):
        for case in ('stale','frozen_uptime','send','cancel','reboot','reconnect'):
            with self.subTest(case=case):
                plant=InertialPlant(target=150)
                if case=='stale':plant.received_offset=1
                if case=='frozen_uptime':plant.freeze_uptime=True
                if case=='send':plant.reject_command=True
                original=plant.sleep
                def fault(seconds):
                    original(seconds)
                    if case=='cancel':plant.transport.emergency_stop_generation+=1
                    if case=='reboot':plant.telem.uptime_ms=0
                    if case=='reconnect':plant.link_epoch+=1
                plant.sleep=fault
                with self.assertRaises(AlignmentFault):plant.fast()

    def test_timeout_never_accepts_coarse_minus_twenty(self):
        plant=InertialPlant(target=80)
        plant.freeze_position=True
        self.assertFalse(plant.fast(profile=profile(max_duration_s=.3)))

    def test_moving_entry_uses_same_capture_policy(self):
        plant=InertialPlant(speed=40,target=170)
        self.assertTrue(plant.run(alignment=profile()))
        self.assertEqual(plant.commands[0][1],40)
        self.assertLessEqual(abs(plant.position-plant.target),8)
        self.assertLessEqual(abs(plant.speed),1)

    def test_moving_entry_old_pose_cannot_grab(self):
        plant=InertialPlant(target=100)
        plant.capture_delay=.1
        plant.freeze_frame=True
        self.assertFalse(plant.run(alignment=profile(),arm_reset_at_s=plant.now))

    def test_purple_uses_its_own_target_and_signed_search_origin(self):
        cfg=replace(HighlandCollectionConfig(),align_target_x_mm=10,
                    orange_align_target_x_mm=999,align_confirm_frames=2)
        plant=InertialPlant(position=-100,target=-106,config=cfg)
        plant.color='purple'
        # Override the orange fixture's camera-X origin for the purple pose.
        plant.frame_override=lambda frame: self.purple_frame(frame,cfg)
        self.assertTrue(plant.fast())
        self.assertEqual(plant.position,-100)

    @staticmethod
    def purple_frame(frame,cfg):
        for block in frame.all_blocks:
            block.x += cfg.align_target_x_mm-cfg.orange_align_target_x_mm
        return frame


class FastAlignmentIntegrationTests(unittest.TestCase):
    def test_disabled_uses_original_alignment_only(self):
        env,c=operation_fixture()
        env.transition_config=TransitionConfig()
        operations.begin_collection(env,spec('begin_collection'))
        c._find_cube=Mock(return_value=object())
        c._align_orange=Mock(return_value=True)
        with patch('Strategy.optimizations.fast_alignment.align_cube') as fast:
            self.assertTrue(operations.acquire_cube(env,spec('acquire_cube')))
            fast.assert_not_called()
        c._align_orange.assert_called_once()

    def test_enabled_skips_both_legacy_alignment_stages(self):
        for name,color in (('ground-1','orange'),('highland-1','orange'),('highland-1','purple')):
            with self.subTest(name=name,color=color):
                env,c=operation_fixture(name)
                env.transition_config=TransitionConfig(alignments={f'{name}/{color}':profile()})
                operations.begin_collection(env,spec('begin_collection',name,color=color))
                c._find_cube=Mock(return_value=object())
                c._align_cube=c._align_orange=c._fine_align_orange=Mock(side_effect=AssertionError('legacy invoked'))
                with patch('Strategy.optimizations.fast_alignment.align_cube',return_value=True) as fast:
                    self.assertTrue(operations.acquire_cube(env,spec('acquire_cube',name)))
                    self.assertEqual(fast.call_args.kwargs['color_name'],color)

    def test_soft_failure_retries_search_without_legacy_timeout_success(self):
        from Strategy.errors import SearchRangeExhausted
        env,c=operation_fixture()
        env.transition_config=TransitionConfig(alignments={'ground-1/orange':profile()})
        operations.begin_collection(env,spec('begin_collection'))
        c._find_cube=Mock(side_effect=[object(),SearchRangeExhausted('end')])
        c._align_orange=Mock(side_effect=AssertionError('legacy invoked'))
        with patch('Strategy.optimizations.fast_alignment.align_cube',return_value=False):
            self.assertFalse(operations.acquire_cube(env,spec('acquire_cube')))
        self.assertTrue(env.data['collection']['exhausted'])

    def test_repeated_visual_failure_has_bounded_attempts(self):
        env,c=operation_fixture()
        env.transition_config=TransitionConfig(alignments={'ground-1/orange':profile()})
        operations.begin_collection(env,spec('begin_collection'))
        c._find_cube=Mock(return_value=object())
        with patch('Strategy.optimizations.fast_alignment.align_cube',return_value=False) as fast:
            self.assertFalse(operations.acquire_cube(env,spec('acquire_cube')))
            self.assertEqual(fast.call_count,2)
        self.assertFalse(env.data['collection']['acquired'])

    def test_registered_blind_handoff_passes_selected_plugin(self):
        from tests.test_registered_pickup_transitions import Fixture, action
        fixture=Fixture()
        selected=profile()
        fixture.env.transition_config=replace(fixture.env.transition_config,
            alignments={'ground-1/orange':selected})
        fixture.run([action('grab_cube','grab',method='grap3',index=1),
                     action('acquire_cube','find',index=2)])
        self.assertIs(fixture.warm.call_args.kwargs['alignment'],selected)
        fixture.control._find_cube.assert_not_called()

    def test_config_is_independent_of_pickup_firmware_flags_and_reversible(self):
        document={'version':1,'alignments':{'ground-1/orange':{
            'validated':True,'verified_on':'2026-10-02','notes':'synthetic only','profile':asdict(profile())}}}
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'config.json'
            path.write_text(json.dumps(document),encoding='utf8')
            loaded=TransitionConfig.load(path)
            self.assertIsNotNone(loaded.alignment('ground-1','orange'))
            self.assertIsNone(loaded.alignment('highland-1','purple'))
            self.assertFalse(loaded.firmware_full_lift_validated)
            import main
            output=io.StringIO()
            with patch('sys.argv',['main.py','--transition-config',str(path),'--enable-fast-alignment','--show-plan']),patch('main.Robot') as robot,contextlib.redirect_stdout(output):
                self.assertEqual(main.main(),0)
                robot.assert_not_called()
            self.assertIn('Fast pickup alignment: ground-1/orange',output.getvalue())
            output=io.StringIO()
            with patch('sys.argv',['main.py','--transition-config',str(path),'--disable-fast-alignment','--show-plan']),patch('main.Robot') as robot,contextlib.redirect_stdout(output):
                self.assertEqual(main.main(),0)
                robot.assert_not_called()
            self.assertIn('Fast pickup alignment: disabled: legacy alignment',output.getvalue())
            output=io.StringIO()
            with patch('sys.argv',['main.py','--transition-config',str(path),'--show-plan']),patch('main.Robot') as robot,contextlib.redirect_stdout(output):
                self.assertEqual(main.main(),0)
                robot.assert_not_called()
            self.assertIn('Fast pickup alignment: disabled: legacy alignment',output.getvalue())
            document['alignments']={}
            path.write_text(json.dumps(document),encoding='utf8')
            self.assertIsNone(TransitionConfig.load(path).alignment('ground-1','orange'))


if __name__=='__main__':unittest.main()
