"""Launcher contract: no robot process is started by these tests."""
from pathlib import Path
import unittest
from unittest.mock import patch, Mock
from tools import managed_task


class ManagedTaskTests(unittest.TestCase):
    def test_detached_unit_preserves_arguments_and_shared_lock(self):
        with patch.object(managed_task.os,'getuid',return_value=1000,create=True), \
             patch.dict(managed_task.os.environ,{'XDG_RUNTIME_DIR':'/run/user/1000'}), \
             patch.object(Path,'is_dir',return_value=True):
            command = managed_task.start_command(['--strategy','PlanA','--classic-motion'],Path('/robot'))
        self.assertIn('--property=Restart=no',command)
        self.assertIn('--property=KillSignal=SIGINT',command)
        self.assertIn(str(Path('/run/user/1000')/'uniforest-desktop-1000.lock'),command)
        self.assertNotIn('--pty',command)
        self.assertEqual(command[-3:],['--strategy','PlanA','--classic-motion'])

    def test_missing_selection_cannot_launch_default_mission(self):
        for args in ([],['--classic-motion']):
            with self.assertRaises(ValueError):
                managed_task.start_command(args)

    def test_stop_uses_service_stop_without_restarting(self):
        with patch.object(managed_task.sys,'platform','linux'), \
             patch.object(managed_task.subprocess,'call',return_value=0) as call:
            self.assertEqual(managed_task.main(['stop']),0)
        call.assert_called_once_with(['systemctl','--user','stop',managed_task.UNIT,'--no-pager'])

    def test_no_linger_refuses_start(self):
        with patch.object(managed_task.sys,'platform','linux'), \
             patch.object(managed_task.os,'getuid',return_value=1000,create=True), \
             patch.object(managed_task,'start_command',return_value=['systemd-run']), \
             patch.object(managed_task.subprocess,'run',return_value=Mock(stdout='no\n')), \
             patch.object(managed_task.subprocess,'call') as call:
            self.assertEqual(managed_task.main(['start','--','--strategy','PlanA']),1)
        call.assert_not_called()
