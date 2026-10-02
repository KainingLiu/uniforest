#!/usr/bin/env python3
"""Official RoboGame entry point for the full mission or one task."""

if __name__ == '__main__':
    from utils.runtime_launcher import launch
    raise SystemExit(launch('main'))

import argparse
import sys

from robot import Robot
from Strategy.runner import (
    TASK_CHOICES, STRATEGY_CHOICES, resolve_selection, run_tasks,
)
from Strategy.context import BuildApproach
from Strategy.plans import PLANS, validate_plan
from Strategy.tasks import TASK_IDS
from vision import default_camera_selector
from vision.yolo.collector import add_collection_arguments
from utils.diagnostics import classify_failure


def parse_args(argv=None):
    default_port = Robot.SERIAL_PORT
    default_camera = default_camera_selector()
    parser = argparse.ArgumentParser(
        description='Uniforest RoboGame competition program')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--strategy', choices=STRATEGY_CHOICES,
                           help='Run a strategy package (default: PlanA)')
    selection.add_argument('--task', choices=TASK_CHOICES,
                           help='Run one reusable task; old selectors remain explicit aliases')
    parser.add_argument('--heading-zero-deg', type=float,
                        help='Known gyro yaw zero for standalone Task3/Task4 at the Task2 exit (180 degrees); never infer from current yaw')
    parser.add_argument('--show-plan', action='store_true',
                        help='Print the selected sequence without connecting to hardware')
    parser.add_argument('--list-tasks', action='store_true',
                        help='List tasks and strategy packages without connecting to hardware')
    parser.add_argument('--port', default=default_port,
                        help=f'A-board serial port (default: {default_port})')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--telem-rate', type=int, default=50)
    parser.add_argument('--camera', default=default_camera,
                        help='Camera role, stable path, or diagnostic index '
                             f'(default: {default_camera})')
    parser.add_argument('--tag-camera', default='tag',
                        help='Tag camera role or stable path (default: tag)')
    parser.add_argument('--no-field-localization', action='store_true',
                        help='Disable AprilTag full-field localization')
    parser.add_argument('--localization-gui', action='store_true')
    parser.add_argument('--vision-gui', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--diagnostics-log', default=None,
                        help='Append JSONL diagnostics to this path')
    add_collection_arguments(parser)
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.list_tasks:
        print('Tasks: ' + ', '.join(TASK_IDS))
        for plan in PLANS.values():
            print(f'{plan.name}: ' + ' -> '.join(step.task_id for step in plan.steps))
        return 0
    selection = args.strategy or args.task or 'PlanA'
    plan = resolve_selection(selection)
    if args.show_plan:
        print(f'{plan.name}: ' + ' -> '.join(step.task_id for step in plan.steps))
        if plan.steps[0].task_id.startswith(('task3-', 'task4-')):
            print('Entry: Task2 exit, heading 180 degrees, loaded for Build (Task3) '
                  'or hatch unloading (Task4); --heading-zero-deg is required.')
        elif plan.steps[0].task_id.startswith('task2-'):
            print('Entry: heading 180 degrees; Task2 starts with a backward move.')
        elif plan.steps[0].task_id == 'task0-3':
            print('Entry: heading 180 degrees at the Task4/Task1 exit; '
                  'turn to 0 degrees, move left 2600 mm, then approach the left wall.')
        elif plan.steps[0].task_id == 'task5':
            print('Entry: Task1-2 exit, heading 180 degrees; two wall loading '
                  'and building cycles, open-hatch wait 200 ms, close-hatch wait 400 ms; '
                  'each third release starts a 100 mm reverse before the lateral route.')
        return 0
    try:
        if args.heading_zero_deg is not None:
            BuildApproach(args.heading_zero_deg, 'explicit entry pose')
        validate_plan(plan, initial_handoff=args.heading_zero_deg is not None)
    except (TypeError, ValueError) as exc:
        print(f'[Strategy] {exc}', file=sys.stderr)
        return 2
    robot = Robot(
        port=args.port,
        baud=args.baud,
        enable_vision=True,
        camera_id=args.camera,
        vision_gui=args.vision_gui,
        enable_localization=not args.no_field_localization,
        localization_camera=args.tag_camera,
        localization_gui=args.localization_gui,
        debug=args.debug,
        diagnostics_path=args.diagnostics_log,
        collect_data=False if args.no_collect_data else None,
        dataset_dir=args.dataset_dir,
    )

    try:
        if not robot.connect():
            return 1
        robot.start(telem_rate=args.telem_rate)
        return run_tasks(robot, selection, heading_zero_deg=args.heading_zero_deg)
    except KeyboardInterrupt:
        print('\n[Competition] Interrupted')
        return 130
    except Exception as exc:
        robot.transport.emergency_stop()
        category = classify_failure(exc)
        robot.diagnostics.write('fatal', category=category,
                                error_type=type(exc).__name__,
                                message=str(exc))
        print(f'[Competition] Fatal error [{category}]: {exc}')
        return 1
    finally:
        robot.stop()


if __name__ == '__main__':
    sys.exit(main())
