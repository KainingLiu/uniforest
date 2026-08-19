#!/usr/bin/env python3
"""Temporary W/S/A/D 1000 mm chassis position-loop test."""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from typing import TYPE_CHECKING

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

if TYPE_CHECKING:
    from robot import Robot

from control.chassis import DEFAULT_MOVE_SPEED_MM_S


MOVE_DISTANCE_MM = 1000.0


class PositionKeyTest:
    MOVES = {
        'w': ('forward', MOVE_DISTANCE_MM),
        's': ('forward', -MOVE_DISTANCE_MM),
        'a': ('right', -MOVE_DISTANCE_MM),
        'd': ('right', MOVE_DISTANCE_MM),
    }

    def __init__(self, robot: Robot, speed_mm_s: float):
        self.robot = robot
        self.speed_mm_s = speed_mm_s
        self._pressed = set()
        self._lock = threading.Lock()
        self._move_thread = None
        self._cancel = threading.Event()
        self._exit = threading.Event()

    @staticmethod
    def _token(key):
        char = getattr(key, 'char', None)
        if char:
            return char.lower()
        name = getattr(key, 'name', None)
        return name.lower() if name else None

    def _on_press(self, key):
        token = self._token(key)
        if token is None:
            return None

        with self._lock:
            first_press = token not in self._pressed
            self._pressed.add(token)

        if not first_press:
            return None

        if token == 'space':
            self._cancel.set()
            self.robot.transport.emergency_stop()
            print('[Test] EMERGENCY STOP')
        elif token == 'esc':
            self._cancel.set()
            self.robot.transport.emergency_stop()
            self._exit.set()
            return False
        elif token in self.MOVES:
            self._start_move(token)
        return None

    def _on_release(self, key):
        token = self._token(key)
        if token is not None:
            with self._lock:
                self._pressed.discard(token)

    def _start_move(self, token: str):
        if self._move_thread is not None and self._move_thread.is_alive():
            print('[Test] Move in progress; key ignored')
            return

        axis, distance_mm = self.MOVES[token]
        self._cancel.clear()

        def worker():
            direction = {
                'w': 'forward', 's': 'backward',
                'a': 'left', 'd': 'right',
            }[token]
            print(f'[Test] Moving {direction} 1000 mm at '
                  f'{self.speed_mm_s:.0f} mm/s')
            try:
                if axis == 'forward':
                    result = self.robot.chassis.move_forward(
                        distance_mm, self.speed_mm_s, self._cancel)
                else:
                    result = self.robot.chassis.move_right(
                        distance_mm, self.speed_mm_s, self._cancel)

                state = ('cancelled' if result.cancelled else
                         'timeout' if result.timed_out else 'complete')
                spread = max(result.wheel_counts) - min(result.wheel_counts)
                print(f'[Test] {state}: wheel={result.encoder_distance_mm:.1f} mm, '
                      f'chassis_est={result.estimated_chassis_distance_mm:.1f} mm, '
                      f'error={result.requested_mm - result.estimated_chassis_distance_mm:+.1f} mm, '
                      f'wheel_spread={spread} counts, time={result.elapsed_ms:.0f} ms')
            except Exception as exc:
                self.robot.transport.emergency_stop()
                print(f'[Test] Move failed: {exc}')

        self._move_thread = threading.Thread(
            target=worker, name='position-1000mm-test', daemon=True)
        self._move_thread.start()

    def run(self):
        try:
            from pynput import keyboard
        except ImportError as exc:
            raise RuntimeError(
                'This test needs pynput: python -m pip install pynput') from exc

        print('\n1000 mm chassis position test')
        print('  W: forward   S: backward   A: left   D: right')
        print('  SPACE: emergency stop      ESC: stop and exit')
        print('  One move runs at a time; repeated keys are ignored.\n')

        listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release)
        listener.start()
        try:
            while not self._exit.wait(0.1):
                pass
        finally:
            self._cancel.set()
            self.robot.transport.emergency_stop()
            listener.stop()
            listener.join(timeout=1.0)
            if self._move_thread is not None:
                self._move_thread.join(timeout=2.0)


def main():
    from robot import Robot

    parser = argparse.ArgumentParser(
        description='W/S/A/D single-step 1000 mm chassis position test')
    parser.add_argument('--port', default=Robot.SERIAL_PORT)
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--speed', type=float, default=DEFAULT_MOVE_SPEED_MM_S,
                        help='Move speed in mm/s (default: 600 = 0.6 m/s)')
    parser.add_argument('--telem-rate', type=int, default=50)
    parser.add_argument('--lateral-scale', type=float, default=None,
                        help='Override lateral compensation (calibrated default: 1.07527)')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    if args.speed <= 0.0:
        parser.error('--speed must be positive')
    if args.telem_rate < 50:
        parser.error('--telem-rate must be at least 50 Hz')
    if args.lateral_scale is not None and args.lateral_scale <= 0.0:
        parser.error('--lateral-scale must be positive')

    robot = Robot(port=args.port, baud=args.baud, debug=args.debug)
    try:
        if not robot.connect():
            return 1
        robot.start(telem_rate=args.telem_rate)
        if args.lateral_scale is not None:
            robot.chassis.set_lateral_distance_scale(args.lateral_scale)

        deadline = time.monotonic() + 2.0
        while robot.telem is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if robot.telem is None:
            raise RuntimeError('No telemetry received from A-board')

        PositionKeyTest(robot, args.speed).run()
        return 0
    finally:
        robot.stop()


if __name__ == '__main__':
    raise SystemExit(main())
