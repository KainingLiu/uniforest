#!/usr/bin/env python3
"""Standalone hardware test for building alignment followed by Build."""

import argparse
import sys

from robot import Robot
from dataclasses import replace

from Strategy.task2 import Task2DebugProgram, Task2Config, Task2Program, Task2State
from vision import default_camera_selector


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Test building visual alignment followed by Build')
    parser.add_argument('--port', default=Robot.SERIAL_PORT)
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--telem-rate', type=int, default=50)
    parser.add_argument('--camera', default=default_camera_selector(),
                        help='Cube camera role/path (default: cube role)')
    parser.add_argument('--tag-camera', default='tag')
    parser.add_argument('--vision-gui', action='store_true')
    parser.add_argument('--localization-gui', action='store_true')
    parser.add_argument('--preflight-only', action='store_true',
                        help='Check telemetry and vision without alignment or Build')
    args = parser.parse_args()

    robot = Robot(port=args.port, baud=args.baud, enable_vision=True,
                  camera_id=args.camera, vision_gui=args.vision_gui,
                  enable_localization=True,
                  localization_camera=args.tag_camera,
                  localization_gui=args.localization_gui)
    # This is a stationary local test: no Tag6 offset move and no yaw turn.
    # The current pose captured by preflight is the heading reference.
    config = replace(Task2Config(),
                     post_tag6_lateral_right_mm=0.0,
                     build_tag_heading_target_cw_deg=0.0)
    program = Task2Program(robot, config)
    try:
        if not robot.connect():
            return 1
        robot.start(telem_rate=args.telem_rate)
        program._preflight()
        if args.preflight_only:
            return Task2DebugProgram(robot).run()
        print('[BuildingTest] Aligning building in place (no chassis move)')
        program.robot.reset_vision_filter()
        aligned = program._align_building_or_continue()
        if aligned:
            print('[BuildingTest] Alignment confirmed; running Build')
        else:
            print('[BuildingTest] Alignment skipped after timeout; running Build')
        program.state = Task2State.BUILD
        robot.actions.build()
        print('[BuildingTest] Build complete')
        return 0
    except KeyboardInterrupt:
        print('\n[BuildingTest] Interrupted')
        return 130
    except Exception as exc:
        robot.transport.emergency_stop()
        print(f'[BuildingTest] Fatal error: {exc}')
        return 1
    finally:
        robot.stop()


if __name__ == '__main__':
    sys.exit(main())
