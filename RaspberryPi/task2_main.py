#!/usr/bin/env python3
"""Task2 entry point; never runs the Task1 mission flow."""

import argparse
import sys

from robot import Robot
from Strategy.task2 import Task2DebugProgram
from Strategy.runner import run_tasks
from vision import default_camera_selector
from vision.yolo.collector import add_collection_arguments


def parse_args():
    default_camera = default_camera_selector()
    parser = argparse.ArgumentParser(
        description='Task2 starts at heading 180 degrees with a backward move; '
                    'collection and approach only (Build is Task3)')
    parser.add_argument('--variant', choices=('1', '2'), default='1')
    parser.add_argument('--port', default=Robot.SERIAL_PORT)
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--telem-rate', type=int, default=50)
    parser.add_argument('--camera', default=default_camera,
                        help='Cube camera role, stable path, or diagnostic '
                             f'index (default: {default_camera})')
    parser.add_argument('--tag-camera', default='tag')
    parser.add_argument('--vision-gui', action='store_true')
    parser.add_argument('--localization-gui', action='store_true')
    parser.add_argument('--preflight-only', action='store_true',
                        help='Check hardware without commanding motion')
    parser.add_argument('--debug', action='store_true')
    add_collection_arguments(parser)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    robot = Robot(
        port=args.port,
        baud=args.baud,
        enable_vision=True,
        camera_id=args.camera,
        vision_gui=args.vision_gui,
        enable_localization=True,
        localization_camera=args.tag_camera,
        localization_gui=args.localization_gui,
        debug=args.debug,
        collect_data=False if args.no_collect_data else None,
        dataset_dir=args.dataset_dir,
    )

    try:
        if not robot.connect():
            return 1
        robot.start(telem_rate=args.telem_rate)
        if args.preflight_only:
            return Task2DebugProgram(robot).run()
        return run_tasks(robot, f'task2-{args.variant}')
    except KeyboardInterrupt:
        print('\n[Task2] Interrupted')
        return 130
    except Exception as exc:
        robot.transport.emergency_stop()
        print(f'[Task2] Fatal error: {exc}')
        return 1
    finally:
        robot.stop()


if __name__ == '__main__':
    sys.exit(main())
