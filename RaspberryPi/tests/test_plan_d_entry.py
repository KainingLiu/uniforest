"""Public PlanD entry points remain explicit and never start hardware in preview."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from Strategy.plans import PLANS, StrategyPlan, validate_plan
from Strategy.runner import resolve_selection, run_plan
from Strategy.tasks import TaskStep
from agent.tools import RobotToolExecutor, tool_definitions


class PlanDEntryTests(unittest.TestCase):
    def test_unreachable_calibration_rejected_before_first_round(self):
        from Strategy.building_profiles import DEFAULT_BUILDING_PROFILES_PATH
        from tests.test_strategy_composition import robot_fixture
        data = json.loads(DEFAULT_BUILDING_PROFILES_PATH.read_text(encoding='utf-8'))
        data['profiles']['1'] = dict(status='verified', z_scale_mm_px=48000,
                                      measured_distance_mm=120, top_row_px=400,
                                      calibration_date='2026-10-04',
                                      result='Synthetic invalid-target test')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'profiles.json'
            path.write_text(json.dumps(data), encoding='utf-8')
            plan = StrategyPlan('PlanD', (
                TaskStep('task0-2'), TaskStep('task5', {'building_profiles_path': str(path)})))
            robot = robot_fixture()
            with patch('Strategy.task0.Task0_2Program.run') as move:
                with self.assertRaises(ValueError):
                    run_plan(robot, plan)
                move.assert_not_called()
            robot.set_collection_context.assert_not_called()

    def test_plan_d_reuses_plan_b_route_and_preserves_default(self):
        self.assertEqual(PLANS['PlanD'].steps, PLANS['PlanB'].steps)
        self.assertIs(resolve_selection('pland'), PLANS['PlanD'])
        self.assertIs(resolve_selection(), PLANS['PlanA'])
        validate_plan(PLANS['PlanD'])

    def test_cli_preview_explains_height_and_refill_contract_without_hardware(self):
        import main
        for selection in ('PlanD', 'pland'):
            output = io.StringIO()
            with self.subTest(selection=selection), \
                    patch('sys.argv', ['main.py', '--strategy', selection, '--show-plan']), \
                    patch('main.Robot') as robot, contextlib.redirect_stdout(output):
                self.assertEqual(main.main(), 0)
                robot.assert_not_called()
            text = output.getvalue()
            self.assertIn('4-4-4 / 4-4-5 / 4-5-6', text)
            self.assertIn('calibration', text)

    def test_agent_can_select_plan_d_without_changing_action_tool(self):
        tools = {tool['name']: tool for tool in tool_definitions()}
        selections = tools['run_strategy']['parameters']['properties']['selection']['enum']
        self.assertIn('PlanD', selections)
        result = RobotToolExecutor(dry_run=True).run_strategy('PlanD').value
        self.assertEqual(result['would_start_strategy'], 'PlanD')
        self.assertEqual(result['tasks'], [s.task_id for s in PLANS['PlanB'].steps])


if __name__ == '__main__':
    unittest.main()
