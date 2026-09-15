"""Inspect storage using the running Robot camera/link, then restore the arm."""

import json
from pathlib import Path
import time

from protocol.commands import ACTION_DONE
from vision.carried_cube_count import observe, classify

CONFIG_PATH = Path(__file__).resolve().parents[1] / 'tools/carried_cube_count_config.json'


def inspect_carried_cubes(robot):
    """Return 0..3/None; hardware/cancellation failures always propagate."""
    config = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    transport = robot.transport
    actions = robot.actions
    generation = transport.emergency_stop_generation
    link_generation = robot.inspection_link_snapshot()[3]

    def check():
        actions._check_cancelled()
        telem, received_at, pong_at, epoch = robot.inspection_link_snapshot()
        now = time.monotonic()
        if (not transport.connected or not robot._running
                or transport.emergency_stop_generation != generation
                or epoch != link_generation or received_at is None
                or now - received_at > .15 or now - pong_at > .15):
            raise RuntimeError('carried-count inspection communication/cancellation fault')
        return telem

    def wait(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            check()
            time.sleep(min(.005, max(0, deadline - time.monotonic())))
        check()

    def servo(servo_id, angle):
        transport.set_servo_angle_checked(servo_id, angle, check)

    if not actions._action_lock.acquire(blocking=False):
        raise RuntimeError('another mechanical action is running')
    try:
        check()
        probe_at = time.monotonic()
        if not transport.query_action_status():
            raise RuntimeError('inspection action status query failed')
        while True:
            check()
            status = transport.get_action_status()
            if status is not None and status[1] >= probe_at:
                if status[0].state != ACTION_DONE:
                    raise RuntimeError(f'A-board action not complete before inspection: {status[0].state}')
                break
            if time.monotonic() - probe_at > .15:
                raise RuntimeError('inspection action status unavailable')
            wait(.005)
        # ACTION_DONE and full telemetry are separate messages. Do not reject
        # the last pre-completion telemetry packet as an active mechanism.
        stopped_at = time.monotonic()
        if not robot.chassis.set_speeds([0, 0, 0, 0]):
            raise RuntimeError('inspection chassis zero-speed command failed')
        last_uptime = None
        confirmed = 0
        initial = check()
        print('[Count] Waiting for stationary telemetry: '
              f'stepper_busy={initial.stepper_busy}, '
              f'rpm={[m.speed_rpm for m in initial.motors]}', flush=True)
        while True:
            check()
            telem, received_at, _, _ = robot.inspection_link_snapshot()
            rpm = [m.speed_rpm for m in telem.motors]
            after_action = ((telem.uptime_ms - status[0].uptime_ms) & 0xffffffff) < 0x80000000
            if received_at >= stopped_at and after_action and telem.uptime_ms != last_uptime:
                last_uptime = telem.uptime_ms
                quiet = not telem.stepper_busy and all(abs(speed) <= 10 for speed in rpm)
                confirmed = confirmed + 1 if quiet else 0
                if confirmed >= 3:
                    break
            if time.monotonic() - stopped_at >= 1.0:
                robot.diagnostics.write('carried_count_stationary_timeout',
                                        stepper_busy=telem.stepper_busy, rpm=rpm,
                                        fresh_after_action=after_action,
                                        confirmed_frames=confirmed)
                raise RuntimeError(
                    'carried-count inspection stationary timeout (1s): '
                    f'stepper_busy={telem.stepper_busy}, rpm={rpm}, '
                    f'fresh_after_action={after_action}, confirmed={confirmed}/3')
            wait(.005)
        print(f'[Count] Stationary confirmed in {time.monotonic() - stopped_at:.3f}s',
              flush=True)
        # Validate configuration and a live raw frame before any pose command.
        sample = robot.cube_raw_frame
        if sample is None or time.monotonic() - sample[1] > .5:
            raise RuntimeError('inspection camera frame unavailable')
        classify([observe(sample[0], config)[0]], config)
        servo(0, 37.2)
        wait(.300)
        servo(1, 120)
        wait(.500)
        first_after = time.monotonic()
        last_timestamp = first_after
        skipped = 0
        observations = []
        while len(observations) < 8:
            check()
            if time.monotonic() - first_after > 4:
                raise RuntimeError('inspection fresh camera frames timed out')
            sample = robot.cube_raw_frame
            if sample is not None and sample[1] > last_timestamp:
                frame, last_timestamp = sample
                if skipped < 3:
                    skipped += 1
                else:
                    observations.append(observe(frame, config)[0])
            wait(.005)
        result = classify(observations, config)
        check()
        servo(1, 90)
        wait(.200)
        servo(0, 97.2)
        check()
        robot.reset_vision_filter(after_inspection=True)
        robot.diagnostics.write('carried_cube_count', result=result,
                                observations=observations, feature_version=config['feature_version'])
        print(f'[Count] {result}', flush=True)
        return result['count']
    except BaseException:
        # Do not turn a hardware/pose failure into the user's null-success path.
        # After cancellation/fault no further servo commands are sent.
        transport.emergency_stop()
        raise
    finally:
        actions._action_lock.release()
