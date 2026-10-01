"""Calibration loading and launch gates; all enabled values are synthetic."""

from dataclasses import asdict
import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from Strategy.transition_config import TransitionConfig
from tests.test_continuous_trajectory import profile as trajectory_profile
from tests.test_registered_pickup_transitions import calibration


class TransitionConfigTests(unittest.TestCase):
    def document(self):
        pickup=calibration()
        record={'validated':True,'verified_on':'2026-10-01','notes':'synthetic test fixture only'}
        return {'version':1,'firmware_full_lift_validated':True,'firmware_id':'test-fixture',
            'pickups':{'ground-1/grap3':{**record,'arm_restore_s':.2,'last_departure':True,
                'next_cube':{'blind':asdict(pickup.next_blind),'acquire':asdict(pickup.next_acquire)}}},
            'curves':{'building-1/build_return':{**record,'profile':asdict(trajectory_profile()),
                'points':[{}, {'dx_scale':1,'dy_scale':1,'dyaw_scale':1}]}}}

    def load(self, document):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'transitions.json'
            path.write_text(json.dumps(document),encoding='utf-8')
            return TransitionConfig.load(path)

    def test_empty_example_keeps_pickups_and_curves_disabled(self):
        path=Path(__file__).resolve().parents[1]/'Strategy/transition_config.example.json'
        config=TransitionConfig.load(path)
        self.assertFalse(config.firmware_full_lift_validated)
        self.assertEqual(dict(config.pickups),{})
        self.assertEqual(dict(config.curves),{})

    def test_complete_records_enable_only_named_profiles(self):
        config=self.load(self.document())
        self.assertTrue(config.pickup('ground-1','grap3').last_departure)
        self.assertEqual(config.pickup('ground-1','grap3').arm_restore_s,.2)
        self.assertIsNone(config.pickup('ground-2','grap3'))
        self.assertEqual(set(config.curves),{'building-1/build_return'})
        with self.assertRaises(TypeError):config.curves['other']=None

    def test_unsafe_or_incomplete_calibration_rejected_on_load(self):
        edits=(
            lambda d:d.update(firmware_full_lift_validated=False),
            lambda d:d.update(firmware_id=''),
            lambda d:d['pickups']['ground-1/grap3'].update(validated=False),
            lambda d:d['pickups']['ground-1/grap3'].update(arm_restore_s=-.1),
            lambda d:d['pickups']['ground-1/grap3'].update(departure=True),
            lambda d:d['pickups']['ground-1/grap3']['next_cube']['blind'].update(direction=-1),
            lambda d:d['curves']['building-1/build_return']['profile'].update(max_speed_mm_s=-1),
            lambda d:d['curves']['building-1/build_return']['profile'].update(max_wheel_rpm=40000),
            lambda d:d['curves']['building-1/build_return']['points'][-1].update(x_mm=100),
            lambda d:d['curves'].update({'ground-1/unload_approach':d['curves']['building-1/build_return']}),
            lambda d:d.update(extra='unexpected'),
        )
        for edit in edits:
            document=self.document()
            edit(document)
            with self.subTest(edit=edit),self.assertRaises((ValueError,TypeError,KeyError)):
                self.load(document)

    def test_bad_configuration_cli_rejects_before_robot_construction(self):
        import main
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'bad.json'
            path.write_text('{"version":999}',encoding='utf-8')
            with patch('sys.argv',['main.py','--transition-config',str(path)]),patch('main.Robot') as robot,contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main.main(),2)
                robot.assert_not_called()

    def test_preview_describes_enabled_keys_without_hardware(self):
        import main
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fixture.json'
            path.write_text(json.dumps(self.document()),encoding='utf-8')
            text=io.StringIO()
            with patch('sys.argv',['main.py','--strategy','PlanA','--transition-config',str(path),'--show-plan']),patch('main.Robot') as robot,contextlib.redirect_stdout(text):
                self.assertEqual(main.main(),0)
                robot.assert_not_called()
            self.assertIn('ground-1/grap3',text.getvalue())
            self.assertIn('building-1/build_return',text.getvalue())


if __name__=='__main__':unittest.main()
