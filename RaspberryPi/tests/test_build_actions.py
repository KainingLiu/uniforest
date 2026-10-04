"""Protocol/client checks and a host replay of the production C action engine."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from agent.direct import build_parser
from agent.tools import tool_definitions
from control.actions import Actions
from protocol import commands
from robot import Robot


class BuildInterfacesTests(unittest.TestCase):
    def test_build_wire_ids_and_legacy_alias(self):
        for name, ident in (('build', 4), ('build3', 4), ('build2', 5), ('build1', 6)):
            action_id = getattr(commands, 'ACTION_' + name.upper())
            self.assertEqual(commands.encode_action_start(0x12345678, action_id),
                             bytes.fromhex('12345678') + bytes((ident, 0)))
            with self.assertRaises(ValueError):
                commands.encode_action_start(1, action_id, True)
        with self.assertRaises(ValueError):
            commands.encode_action_start(1, 7)

    def test_client_and_robot_entry_select_the_expected_firmware_action(self):
        actions = Actions(Mock(), SimpleNamespace(_t=Mock()))
        actions._run_action = Mock()
        callback = Mock()
        robot = Robot.__new__(Robot)
        robot.actions = actions
        for name, ident in (('build', 4), ('build3', 4), ('build2', 5), ('build1', 6)):
            with self.subTest(name=name):
                getattr(actions, name)(chassis_followup=callback)
                actions._run_action.assert_called_with(ident, chassis_followup=callback)
                robot.run_action(name)
                actions._run_action.assert_called_with(ident, chassis_followup=None)
                self.assertEqual(build_parser().parse_args(['--action', name]).action, name)
        arm = next(t for t in tool_definitions() if t['name'] == 'execute_arm_action')
        names = arm['parameters']['properties']['action']['enum']
        self.assertTrue({'build', 'build1', 'build2', 'build3'}.issubset(names))


class FirmwareBuildTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[2]
        firmware = root / 'Uniforest_A/Core'
        if not firmware.exists():
            raise unittest.SkipTest('firmware replay runs on the development checkout')
        compiler = shutil.which('gcc') or shutil.which('cc')
        if not compiler:
            # CLion exposes its bin in PATH; its bundled host compiler is below it.
            compiler = next((str(Path(p) / 'mingw/bin/gcc.exe')
                             for p in os.get_exec_path()
                             if (Path(p) / 'mingw/bin/gcc.exe').is_file()), None)
        if not compiler:
            raise unittest.SkipTest('host C compiler not available')
        cls.temp = tempfile.TemporaryDirectory(prefix='uniforest-actions-')
        cls.addClassCleanup(cls.temp.cleanup)
        temp = Path(cls.temp.name)
        fixtures = Path(__file__).parent / 'firmware'
        for name in ('protocol.h', 'servo.h', 'stepper.h', 'suction.h'):
            (temp / name).write_text('#include "action_stub.h"\n', encoding='utf-8')
        cls.executable = temp / ('actions.exe' if os.name == 'nt' else 'actions')
        subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I', str(temp), '-I', str(fixtures), '-I', str(firmware / 'Inc'),
                        str(fixtures / 'actions_harness.c'), str(firmware / 'Src/actions.c'),
                        '-o', str(cls.executable)], check=True, capture_output=True, timeout=30)
        native = temp / 'real-stepper'
        native.mkdir()
        (native / 'stm32f4xx_hal.h').write_text('#include "stepper_hal_stub.h"\n', encoding='utf-8')
        cls.stepper_executable = native / cls.executable.name
        subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I', str(native), '-I', str(fixtures), '-I', str(firmware / 'Inc'),
                        str(fixtures / 'stepper_endpoint_harness.c'), str(firmware / 'Src/stepper.c'),
                        '-o', str(cls.stepper_executable)], check=True, capture_output=True, timeout=30)

    def replay(self, variant, scenario, rate=4):
        result = subprocess.run([str(self.executable), str(variant), scenario, str(rate)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_build1_lifts_at_15cm_releases_once_then_returns_home(self):
        for rate in (2, 80):  # cover either the arm or the remaining 5 cm finishing first
            self.replay(1, 'normal', rate)

    def test_build2_releases_twice_then_returns_home(self):
        for rate in (4, 80):  # lift can finish before or after the linked stepper group
            self.replay(2, 'normal', rate)

    def test_build3_still_releases_three_and_returns_home(self):
        self.replay(3, 'normal')

    def test_real_stepper_triggers_at_first_leg_endpoint(self):
        result = subprocess.run([str(self.stepper_executable)], capture_output=True,
                                text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_cancellation_during_chassis_ready_tail_stops_both_axes(self):
        for variant in (1, 2, 3):
            self.replay(variant, 'cancel')

    def test_stalled_motion_times_out_without_releasing(self):
        for variant in (1, 2, 3):
            self.replay(variant, 'timeout')

    def test_build1_single_axis_rise_handles_stall_and_cancel_during_lift(self):
        self.replay(1, 'stall_lift')
        self.replay(1, 'cancel_lift')


if __name__ == '__main__':
    unittest.main()
