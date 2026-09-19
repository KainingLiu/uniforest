#!/usr/bin/env python3
"""One-shot storage inspection and labelled capture, outside competition code."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2

from protocol.commands import (
    ACK_OK, CMD_SERVO_ANGLE, ACTION_RUNNING, ACTION_CHASSIS_READY, TELEM_ACK,
)
from protocol.transport import Transport
from robot import default_serial_port
from vision.camera_devices import resolve_camera_source
from vision.cube_detector import load_camera_settings
from vision.carried_cube_count import observe, classify

POSE = {'flip_inspect_deg': 37.2, 'arm_inspect_deg': 120,
        'arm_home_deg': 90, 'flip_home_deg': 97.2,
        'flip_wait_ms': 300, 'arm_wait_ms': 500, 'return_wait_ms': 200}


class LinkFault(RuntimeError):
    """Latched communications failure: no further pose commands are allowed."""


class InspectionTransport(Transport):
    """Match servo ACK sequence numbers without changing the shared protocol API."""

    def __init__(self, port):
        super().__init__(port, 115200)
        self.command_gate = threading.RLock()
        self.servo_sequence = None

    def send(self, cmd, data=b''):
        with self.command_gate:
            sent = super().send(cmd, data)
            if cmd == CMD_SERVO_ANGLE:
                self.servo_sequence = self._seq if sent else None
            return sent

    def _dispatch(self, cmd, seq, data):
        with self.command_gate:
            if cmd == TELEM_ACK and data and data[0] == CMD_SERVO_ANGLE:
                if seq != self.servo_sequence:
                    return
            super()._dispatch(cmd, seq, data)


class CheckedLink:
    """Single-command ACK checking plus continuous telemetry/heartbeat watchdog."""

    def __init__(self, port):
        self.transport = InspectionTransport(port)
        self.last_telem = self.last_pong = 0.0
        self.telem = None
        self.uptime = None
        self.fault = None
        self.armed = False
        self.motion_started = False
        self.ack = None
        self.ack_event = threading.Event()
        self.stop_event = threading.Event()
        self.thread = None
        self.transport.on_telemetry(self._on_telem)
        self.transport.on_pong(self._on_pong)
        self.transport.on_ack(self._on_ack)

    def _on_pong(self, uptime):
        self.last_pong = time.monotonic()

    def _on_telem(self, telem):
        # On the real DAPLink, opening the port can first deliver telemetry
        # buffered before an earlier MCU reset. Establish a PONG handshake
        # before accepting a telemetry epoch or allowing any pose command.
        if not self.last_pong:
            return
        now = time.monotonic()
        if self.armed and self.last_telem and now - self.last_telem > .15:
            self.fault = 'telemetry gap exceeded 150 ms'
        if self.uptime is not None and ((telem.uptime_ms - self.uptime) & 0xffffffff) > 0x7fffffff:
            self.fault = 'A-board restarted'
        self.telem, self.uptime, self.last_telem = telem, telem.uptime_ms, now

    def _on_ack(self, ack):
        if ack.status != ACK_OK:
            self.fault = f'A-board rejected command {ack.echoed_cmd:#x}: {ack.status}'
        if ack.echoed_cmd == CMD_SERVO_ANGLE:
            self.ack = ack
            self.ack_event.set()

    def _heartbeat(self):
        while not self.stop_event.wait(.05):
            if not self.transport.ping():
                self.fault = 'heartbeat send failed'
            if self.armed:
                try:
                    self.check()
                except LinkFault:
                    if self.motion_started:
                        self.transport.emergency_stop()
                    return

    def start(self):
        if not self.transport.connect():
            raise LinkFault('serial open failed')
        self.thread = threading.Thread(target=self._heartbeat, daemon=True)
        self.thread.start()
        if not self.transport.set_telemetry_rate(50):
            raise LinkFault('telemetry setup failed')
        deadline = time.monotonic() + 3
        while not (self.last_telem and self.last_pong):
            if self.fault or time.monotonic() >= deadline:
                raise LinkFault(self.fault or 'no fresh telemetry/PONG')
            time.sleep(.01)
        self.armed = True
        self.check()
        if self.telem.stepper_busy or any(abs(m.speed_rpm) > 10 for m in self.telem.motors):
            raise LinkFault('robot must be stationary before inspection')
        self.transport.query_action_status()
        deadline = time.monotonic() + .15
        while self.transport.get_action_status() is None:
            self.check()
            if time.monotonic() >= deadline:
                raise LinkFault('A-board action status unavailable')
            time.sleep(.005)
        if self.transport.get_action_status()[0].state in (ACTION_RUNNING, ACTION_CHASSIS_READY):
            raise LinkFault('A-board composite action is running')

    def check(self):
        now = time.monotonic()
        if not self.transport.connected:
            self.fault = 'serial disconnected'
        if self.armed and (now - self.last_telem > .15 or now - self.last_pong > .15):
            self.fault = 'telemetry/PONG stale (>150 ms)'
        if self.fault:
            raise LinkFault(self.fault)

    def wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check()
            time.sleep(min(.01, max(0, deadline - time.monotonic())))
        self.check()

    def set_servo(self, servo_id, angle):
        self.check()
        # Reset and send under the same gate used by receive dispatch so an
        # old ACK cannot arrive between resetting the event and assigning SEQ.
        with self.transport.command_gate:
            self.ack = None
            self.ack_event.clear()
            self.motion_started = True
            if not self.transport.set_servo_angle(servo_id, angle):
                self.fault = 'servo command send failed'
                raise LinkFault(self.fault)
        deadline = time.monotonic() + .15
        while not self.ack_event.wait(.005):
            self.check()
            if time.monotonic() >= deadline:
                self.fault = 'servo ACK timeout'
                raise LinkFault(self.fault)
        self.check()
        print(f'SERVO id={servo_id} angle={angle}', flush=True)

    def close(self):
        self.armed = False
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=.3)
        if self.transport.connected and self.motion_started:
            self.transport.emergency_stop()
        self.transport.disconnect()


class LatestCamera:
    """Continuously drain capture buffers; main thread never blocks in read()."""

    def __init__(self, selector):
        self.source = resolve_camera_source(selector)
        backend = cv2.CAP_DSHOW if sys.platform == 'win32' else cv2.CAP_V4L2
        self.cap = cv2.VideoCapture(self.source, backend)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f'cannot open camera {self.source}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.settings = load_camera_settings()
        if self.settings.get('exposure') is not None:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, .25 if sys.platform == 'win32' else 1)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self.settings['exposure'])
        if self.settings.get('gain') is not None:
            self.cap.set(cv2.CAP_PROP_GAIN, self.settings['gain'])
        if self.settings.get('white_balance') is not None:
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, self.settings['white_balance'])
        self.actual_settings = {name: self.cap.get(prop) for name, prop in [
            ('width', cv2.CAP_PROP_FRAME_WIDTH), ('height', cv2.CAP_PROP_FRAME_HEIGHT),
            ('exposure', cv2.CAP_PROP_EXPOSURE), ('gain', cv2.CAP_PROP_GAIN),
            ('auto_wb', cv2.CAP_PROP_AUTO_WB)]}
        self.lock = threading.Lock()
        self.sample = None
        self.running = True
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        sequence = 0
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                sequence += 1
                with self.lock:
                    self.sample = (sequence, time.monotonic(), frame)
            else:
                time.sleep(.01)

    def frames(self, count, check, timeout=4):
        with self.lock:
            # Discard three additional frames after entering the stable pose.
            previous = (self.sample[0] if self.sample else 0) + 3
        entered = time.monotonic()
        output = []
        while len(output) < count:
            check()
            if time.monotonic() - entered > timeout:
                raise RuntimeError('camera fresh-frame timeout')
            with self.lock:
                sample = self.sample
            if sample and sample[0] > previous and sample[1] >= entered:
                if time.monotonic() - sample[1] > .2:
                    raise RuntimeError('camera frame is stale')
                previous = sample[0]
                output.append(sample[2].copy())
            time.sleep(.01)
        return output

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        self.cap.release()


def run_sequence(link, inspect):
    """Exact requested sequence; never restore automatically after a link fault/Ctrl+C."""
    def restore():
        link.set_servo(1, POSE['arm_home_deg'])
        link.wait(POSE['return_wait_ms'] / 1000)
        link.set_servo(0, POSE['flip_home_deg'])

    link.set_servo(0, POSE['flip_inspect_deg'])
    link.wait(POSE['flip_wait_ms'] / 1000)
    link.set_servo(1, POSE['arm_inspect_deg'])
    link.wait(POSE['arm_wait_ms'] / 1000)
    try:
        result = inspect()
    except (KeyboardInterrupt, LinkFault):
        raise
    except Exception:
        # A camera/analysis failure can restore only while communications remain healthy.
        link.check()
        restore()
        raise
    restore()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=default_serial_port())
    parser.add_argument('--camera', default='cube')
    parser.add_argument('--known-count', type=int, choices=range(4),
                        help='manual ground truth for later calibration; never used as prediction')
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).with_name('carried_cube_count_config.json'))
    parser.add_argument('--output', type=Path, default=ROOT / '.diagnostics' / 'carried_cube_count')
    parser.add_argument('--image', type=Path, help='analyse saved image without serial/camera/motion')
    parser.add_argument('--dry-run', action='store_true', help='print sequence without hardware access')
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding='utf-8'))
    if args.dry_run:
        print(json.dumps(POSE, ensure_ascii=False, indent=2))
        return 0
    run_dir = args.output / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    run_dir.mkdir(parents=True, exist_ok=False)
    camera = link = None
    resources = ExitStack()
    frames = []
    observations = []
    metadata = {'date': datetime.now().astimezone().isoformat(), 'pose': POSE,
                'config': config, 'known_count': args.known_count,
                'scope': 'standalone_test_only', 'field_confirmed': False,
                'status': 'started', 'result': None}

    def inspect():
        nonlocal frames, observations
        if camera:
            frames = camera.frames(8, link.check)
        observations = [observe(frame, config)[0] for frame in frames]
        return classify(observations, config)

    try:
        if args.image:
            frame = cv2.imread(str(args.image))
            if frame is None:
                raise ValueError(f'cannot read {args.image}')
            frames = [frame]
            metadata['result'] = inspect()
        else:
            # Cooperates with desktop launchers. Manual non-cooperating programs
            # must also be stopped before running this entry point.
            if sys.platform != 'win32':
                import fcntl
                lock_path = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / f'uniforest-desktop-{os.getuid()}.lock'
                lock_file = resources.enter_context(open(lock_path, 'a'))
                fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            camera = LatestCamera(args.camera)
            preflight_frame = camera.frames(1, lambda: None)[0]
            classify([observe(preflight_frame, config)[0]], config)
            link = CheckedLink(args.port)
            link.start()
            metadata['result'] = run_sequence(link, inspect)
        metadata['status'] = 'completed'
        print(json.dumps(metadata['result'], ensure_ascii=False))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        metadata['status'] = 'aborted'
        metadata['error'] = f'{type(exc).__name__}: {exc}'
        print(f'ABORTED: {metadata["error"]}', file=sys.stderr)
        return 1
    finally:
        if link:
            link.close()
        if camera:
            metadata['camera'] = {'source': str(camera.source),
                                  'requested_settings': camera.settings,
                                  'actual_settings': camera.actual_settings}
            camera.close()
        resources.close()
        metadata['observations'] = observations
        for index, frame in enumerate(frames):
            if not cv2.imwrite(str(run_dir / f'raw_{index:02d}.png'), frame):
                raise RuntimeError('failed to save raw calibration image')
        if frames and observations:
            _, preview = observe(frames[-1], config)
            cv2.imwrite(str(run_dir / 'preview.png'), preview)
        (run_dir / 'result.json').write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'SAVED {run_dir}', flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
