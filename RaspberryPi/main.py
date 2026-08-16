#!/usr/bin/env python3
"""Official RoboGame entry point for the full mission or one task."""

import argparse
import sys

from robot import Robot
from Strategy.runner import TASK_CHOICES, run_tasks
from vision import default_camera_selector


def parse_args():
    default_port = Robot.SERIAL_PORT
    default_camera = default_camera_selector()
    parser = argparse.ArgumentParser(
        description='Uniforest RoboGame competition program')
    parser.add_argument(
        '--task', choices=TASK_CHOICES, default='all',
        help='Run the full Task1 -> Task2 sequence or one task '
             '(default: all)')
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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
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
    )

    try:
        if not robot.connect():
            return 1
        robot.start(telem_rate=args.telem_rate)
        return run_tasks(robot, args.task)
    except KeyboardInterrupt:
        print('\n[Competition] Interrupted')
        return 130
    except Exception as exc:
        robot.transport.emergency_stop()
        print(f'[Competition] Fatal error: {exc}')
        return 1
    finally:
        robot.stop()


if __name__ == '__main__':
    sys.exit(main())
