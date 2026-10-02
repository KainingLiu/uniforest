#!/usr/bin/env python3
"""RoboGame entry point for declarative strategies and functional action flows."""

if __name__ == '__main__':
    from utils.run_logs import run_entry
    raise SystemExit(run_entry('main'))

import argparse
import sys

from robot import Robot
from Strategy.runner import SELECTION_CHOICES, resolve_selection, run_selection
from Strategy.plans import PLANS, validate_plan
from Strategy.cli import add_execution_arguments, load_execution_config, print_plan
from vision import default_camera_selector
from vision.yolo.collector import add_collection_arguments
from utils.diagnostics import classify_failure


def parse_args(argv=None):
    default_port = Robot.SERIAL_PORT
    default_camera = default_camera_selector()
    parser = argparse.ArgumentParser(
        description='Uniforest RoboGame competition program')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--strategy', choices=SELECTION_CHOICES,
                           help='Run a declarative strategy (default: PlanA)')
    selection.add_argument('--flow', choices=SELECTION_CHOICES,
                           help='Run a named functional action flow')
    selection.add_argument('--task', choices=SELECTION_CHOICES,
                           help='Deprecated selector spelling; expands to functional actions')
    parser.add_argument('--heading-zero-deg', type=float,
                        help='Known calibrated yaw zero for a standalone build/unload entry')
    parser.add_argument('--show-plan', action='store_true',
                        help='Show every functional action without connecting hardware')
    parser.add_argument('--list-flows', '--list-tasks', dest='list_flows', action='store_true',
                        help='List named strategies and action flows')
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
    add_execution_arguments(parser)
    add_collection_arguments(parser)
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.list_flows:
        for plan in PLANS.values():
            print(f'{plan.name}: {len(plan.steps)} functional actions; entry={plan.entry_anchor}')
        return 0
    selection = args.strategy or args.flow or args.task or 'PlanA'
    plan = resolve_selection(selection)
    try:
        transitions = load_execution_config(args, plan)
    except (OSError, TypeError, ValueError, KeyError) as exc:
        print(f'[Transitions] Invalid calibration: {exc}',file=sys.stderr)
        return 2
    if args.show_plan:
        print_plan(plan, transitions)
        return 0
    try:
        validate_plan(plan, heading_zero_deg=args.heading_zero_deg)
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
        pickup_full_lift_validated=transitions.firmware_full_lift_validated,
        pickup_trial_enabled=transitions.trial_run,
        execution_extensions=True,
    )

    try:
        if transitions.trial_run:
            from dataclasses import asdict
            robot.diagnostics.write('field_trial_settings',validated=False,
                pickups={k:asdict(v) for k,v in transitions.pickups.items()},
                alignments={k:asdict(v) for k,v in transitions.alignments.items()})
            print('[Trial] Pickup/alignment trial parameters UNVALIDATED; route planning selected separately')
        if not robot.connect():
            return 1
        robot.start(telem_rate=args.telem_rate)
        return run_selection(robot, selection, heading_zero_deg=args.heading_zero_deg,
                             transition_config=transitions)
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
