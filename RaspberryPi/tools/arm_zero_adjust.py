#!/usr/bin/env python3
"""Interactive front-arm angle calibration, outside competition logic."""

import argparse
from decimal import Decimal, InvalidOperation
from pathlib import Path
import queue
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol.commands import ACTION_RUNNING, ACTION_CHASSIS_READY, SERVO_ARM_FRONT
from robot import Robot


def parse_angle(text):
    """Reject invalid inputs rather than silently clamp or round them."""
    try:
        angle = Decimal(text)
        if (not angle.is_finite() or not 0 <= angle <= 180
                or angle * 10 != (angle * 10).to_integral_value()):
            raise ValueError
    except (InvalidOperation, ValueError):
        raise ValueError('请输入 0～180 之间的角度，最多一位小数。') from None
    return float(angle)


def read_inputs(inbox):
    # Keep supervising communication while the user is at the input prompt.
    while True:
        try:
            text = input('前臂角度（q 退出）> ').strip()
        except EOFError:
            text = 'q'
        inbox.put(text)
        if text.casefold() in ('q', 'quit', 'exit'):
            return


def wait_for_stable_link(robot):
    """Establish the baseline after streaming heartbeats, not connect's PONG.

    Robot.start waits 100 ms and its first heartbeat waits another 50 ms.
    That first streaming PONG can advance the link epoch at the 150 ms
    boundary. Do not arm against the earlier connection-handshake epoch.
    """
    started = time.monotonic()
    deadline = started + 3.0
    stable_since = None
    candidate_epoch = None
    while True:
        telem, received, pong, epoch = robot.inspection_link_snapshot()
        now = time.monotonic()
        fresh = (telem is not None and received is not None
                 and received >= started and pong >= started
                 and now - received < .15 and now - pong < .15)
        if not fresh or epoch != candidate_epoch:
            candidate_epoch = epoch
            stable_since = now if fresh else None
        elif stable_since is None:
            stable_since = now
        if fresh and stable_since is not None and now - stable_since >= .3:
            return epoch
        if now >= deadline:
            raise RuntimeError('启动后未建立连续稳定的遥测和心跳，不能调零。')
        time.sleep(.01)


def adjust(robot, *, check_link=False):
    transport = robot.transport
    epoch = wait_for_stable_link(robot)
    generation = transport.emergency_stop_generation

    def check():
        telem, received, pong, current_epoch = robot.inspection_link_snapshot()
        now = time.monotonic()
        if not transport.connected:
            raise RuntimeError('串口已断开，请检查后重新运行。')
        if transport.emergency_stop_generation != generation:
            raise RuntimeError('已收到本程序急停请求，请重新运行。')
        if current_epoch != epoch:
            raise RuntimeError(f'链路代次变化 {epoch}→{current_epoch}：'
                               '收到数据曾中断超过 150 ms 或 A 板重启，请重新运行。')
        if telem is None or received is None:
            raise RuntimeError('遥测不可用，请重新运行。')
        if now - received > .15 or now - pong > .15:
            raise RuntimeError(f'通信数据过期：遥测 {(now - received) * 1000:.0f} ms，'
                               f'心跳 {(now - pong) * 1000:.0f} ms（限制 150 ms）。')
        if telem.stepper_busy or any(abs(m.speed_rpm) > 10 for m in telem.motors):
            raise RuntimeError('底盘或步进仍在运动，不能调零。')

    def check_action_idle():
        check()
        queried = time.monotonic()
        if not transport.query_action_status():
            raise RuntimeError('动作状态查询发送失败。')
        while True:
            check()
            status = transport.get_action_status()
            if status is not None and status[1] >= queried:
                if status[0].state in (ACTION_RUNNING, ACTION_CHASSIS_READY):
                    raise RuntimeError('Grap/Build 尚未结束，不能调零。')
                return
            if time.monotonic() - queried > .15:
                raise RuntimeError('动作状态查询超时。')
            time.sleep(.005)

    check_action_idle()
    if check_link:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            check()
            time.sleep(.02)
        print('通信预检通过：启动握手及连续 3 秒监视正常，未发送舵机角度。', flush=True)
        return
    print('只控制前臂 S2 / ID 1；输入角度后回车，例如 90、91、91.5。', flush=True)
    print('输入为逻辑角度，输出取决于已烧录固件的偏置；程序不能读回该偏置。', flush=True)
    print('找到原来 90° 对应的姿态后，记下角度并告知；q 退出，不自动复位。', flush=True)
    inbox = queue.Queue()
    threading.Thread(target=read_inputs, args=(inbox,), daemon=True).start()
    last_angle = None
    while True:
        check()
        try:
            text = inbox.get(timeout=.02)
        except queue.Empty:
            continue
        if text.casefold() in ('q', 'quit', 'exit'):
            if last_angle is not None:
                print(f'最后已确认接收的指令：{last_angle:.1f}°。')
                print('请以实际姿态确定默认角度；程序未保存或启用新偏置。')
            return
        if not text:
            continue
        try:
            angle = parse_angle(text)
        except ValueError as exc:
            print(exc, flush=True)
            continue
        check_action_idle()
        transport.set_servo_angle_checked(SERVO_ARM_FRONT, angle, check)
        last_angle = angle
        print(f'A 板已接收前臂 {angle:.1f}°；请观察是否到达原默认姿态。', flush=True)
        print(f'若此位置是原默认姿态，需在已烧录偏置上增加 {angle - 90:+.1f}°。', flush=True)


def main():
    parser = argparse.ArgumentParser(description='前臂舵机独立调零：输入角度后直接控制 S2。')
    parser.add_argument('--port', default=Robot.SERIAL_PORT, help='A 板串口')
    parser.add_argument('--baud', type=int, default=115200, help='串口波特率')
    parser.add_argument('--check-link', action='store_true',
                        help='仅检查启动握手和连续 3 秒通信，不发送舵机角度')
    args = parser.parse_args()
    robot = Robot(port=args.port, baud=args.baud, enable_vision=False,
                  enable_localization=False)
    try:
        if not robot.connect():
            return 1
        robot.start(telem_rate=50)
        adjust(robot, check_link=args.check_link)
        return 0
    except KeyboardInterrupt:
        print('\n调零已结束，不自动复位前臂。')
        return 0
    except RuntimeError as exc:
        print(f'调零中止：{exc}', file=sys.stderr)
        return 1
    finally:
        robot.stop()


if __name__ == '__main__':
    raise SystemExit(main())
