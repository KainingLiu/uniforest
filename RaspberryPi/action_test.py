#!/usr/bin/env python3
"""Temporary standalone entry point for mechanical action testing."""

import argparse
import time

from robot import Robot


TEST_ACTIONS = ('grap1', 'grap2', 'grap3', 'build')


def main():
    parser = argparse.ArgumentParser(
        description='Run one composite mechanical action without competition logic')
    parser.add_argument('action', nargs='?', choices=TEST_ACTIONS,
                        default='grap3',
                        help='action to test (default: grap3)')
    parser.add_argument('--port', default=Robot.SERIAL_PORT,
                        help=f'serial port (default: {Robot.SERIAL_PORT})')
    parser.add_argument('--baud', type=int, default=115200,
                        help='baud rate (default: 115200)')
    parser.add_argument('--telem-rate', type=int, default=50,
                        help='telemetry rate in Hz (default: 50)')
    args = parser.parse_args()

    robot = Robot(port=args.port, baud=args.baud)
    try:
        if not robot.connect():
            raise SystemExit(1)
        robot.start(telem_rate=args.telem_rate)
        time.sleep(0.5)
        robot.run_action(args.action, test_mode=True)
    finally:
        robot.stop()


if __name__ == '__main__':
    main()
