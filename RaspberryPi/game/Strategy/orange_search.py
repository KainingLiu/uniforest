"""Bounded left-edge recovery during the normal rightward orange search."""

import copy
from contextlib import suppress
from dataclasses import dataclass
import math
import time


@dataclass
class OrangeSearchRecovery:
    # Shared across searches/grabs, reset only at the orange phase origin.
    origin: object = None
    armed: bool = True
    retry_after_mm: float = float('-inf')
    clear_frames: int = 0
    search_elapsed_s: float = 0.0
    encoder_min_mm: float | None = None
    encoder_max_mm: float | None = None


def _same_row(block, band):
    """Associate a recovered candidate with the original component's rows."""
    quad = getattr(block, 'quad', None)
    if quad is None:
        return False
    rows = [float(point[1]) for point in quad]
    low, high = min(rows), max(rows)
    overlap = min(high, band[1]) - max(low, band[0])
    return overlap > 0 and overlap >= 0.5 * min(high - low, band[1] - band[0])


def find_orange(program, tracker, search_limit_mm):
    """Return a confirmed block, or None on budget exhaustion.

    The existing rightward budget counts only commanded right-search time.
    Camera loss continues the bounded rightward search. Link, telemetry,
    cancellation and command failures still propagate. Recovery has independent
    encoder, commanded-distance and time bounds; active search time is shared
    across pickups/refills and excludes time spent executing mechanical actions.
    """
    cfg, robot = program.config, program.robot
    state = program._orange_recovery
    now = time.monotonic()
    started = now
    last_update = last_telem_at = now
    last_uptime = last_frame = None
    stop_generation = robot.transport.emergency_stop_generation
    speed = 0.0
    clipped_frames = 0
    recovering = False
    recovery_start = recovery_x = recovery_travel = 0.0
    recovery_band = None
    lost_frames = 0
    stop_until = now
    hold_start = None
    hold_spent = False
    vision_missing = False
    outcome = 'aborted'
    fresh_frames = candidate_frames = max_candidates = 0
    detected_frames = clipped_observations = 0
    search_time_budget_s = (search_limit_mm / cfg.search_speed_mm_s
                            + cfg.orange_edge_timeout_s
                            + cfg.target_cube_count * cfg.vision_observe_s)
    print(f'[{program.operation_name}] Orange search: '
          f'{program._search_position_mm:.0f}/{search_limit_mm:.0f} mm right budget')

    def encoder_span():
        return (0.0 if state.encoder_min_mm is None else
                state.encoder_max_mm - state.encoder_min_mm)

    def exhausted(reason):
        nonlocal outcome
        outcome = reason
        return None

    try:
        if state.origin is None:
            state.origin = program._capture_lateral_origin()
        while True:
            program._check_active()
            now = time.monotonic()
            dt = max(0.0, now - last_update)
            last_update = now
            state.search_elapsed_s += dt
            program._search_position_mm = min(
                search_limit_mm, program._search_position_mm + max(0.0, speed) * dt)
            if search_limit_mm - program._search_position_mm < 1e-6:
                program._search_position_mm = search_limit_mm
            recovery_travel += max(0.0, -speed) * dt

            # Never reinterpret lost communication/telemetry as an empty image.
            if not robot.transport.connected:
                raise RuntimeError('communication lost during orange search')
            if robot.transport.emergency_stop_generation != stop_generation:
                raise RuntimeError('orange search cancelled by emergency stop')
            telem = robot.telem
            if telem is None:
                raise RuntimeError('telemetry unavailable during orange search')
            if telem.uptime_ms != last_uptime:
                last_uptime, last_telem_at = telem.uptime_ms, now
            if now - last_telem_at >= cfg.telemetry_stale_s:
                raise RuntimeError('telemetry lost during orange search')
            x = program._measure_lateral_displacement_mm(state.origin)
            if not math.isfinite(x):
                raise RuntimeError('invalid orange search encoder position')
            state.encoder_min_mm = x if state.encoder_min_mm is None else min(state.encoder_min_mm, x)
            state.encoder_max_mm = x if state.encoder_max_mm is None else max(state.encoder_max_mm, x)
            if state.search_elapsed_s >= search_time_budget_s:
                exhausted('time_budget')
                program._search_position_mm = search_limit_mm
                return None

            result = robot.vision_result
            fresh = (result is not None
                     and 0.0 <= time.time() - result.timestamp <= cfg.vision_stale_s)
            new_frame = fresh and result.timestamp != last_frame
            candidates = []
            clipped_band = None
            if fresh:
                candidates = [b for b in result.all_blocks
                              if b.color_name.casefold() == 'orange'
                              and b.confidence >= cfg.orange_min_confidence]
                clipped_band = getattr(result, 'orange_left_clipped_y_range', None)
                if new_frame:
                    detected_frames += bool(candidates)
                    clipped_observations += clipped_band is not None
                if recovering:
                    candidates = [b for b in candidates if _same_row(b, recovery_band)]
                # Recover the clipped left edge before accepting a complete
                # block to its right. Once the bounded attempt is consumed,
                # ordinary candidates remain available until recovery rearms.
                if clipped_band is not None and (state.armed or recovering):
                    candidates = []

            if not fresh:
                if not vision_missing:
                    print(f'[{program.operation_name}] Orange vision unavailable; '
                          'continue bounded rightward search')
                vision_missing = True
                clipped_frames = 0
                state.clear_frames = 0
                tracker.reset()
            elif vision_missing:
                vision_missing = False
                print(f'[{program.operation_name}] Orange vision recovered')

            if new_frame:
                fresh_frames += 1
                candidate_frames += bool(candidates)
                max_candidates = max(max_candidates, len(candidates))
                last_frame = result.timestamp
                tracked_result = copy.copy(result)
                tracked_result.all_blocks = candidates
                block = tracker.update(
                    tracked_result, color_name='orange',
                    min_confidence=cfg.orange_min_confidence,
                    max_age_s=cfg.vision_stale_s)
                if block is not None:
                    outcome = 'acquired'
                    print(f'[{program.operation_name}] Orange acquired: '
                          f'x={block.x:+.0f} mm, right budget='
                          f'{program._search_position_mm:.0f} mm')
                    return block

                # A continuous clipped row gets one attempt. Rearm only after
                # clear frames AND progress past the previous trigger position.
                if not state.armed and not recovering:
                    state.clear_frames = (state.clear_frames + 1
                                          if clipped_band is None else 0)
                    if (state.clear_frames >= cfg.orange_edge_confirm_frames
                            and x >= state.retry_after_mm):
                        state.armed = True
                clipped_frames = (clipped_frames + 1
                                  if not candidates and clipped_band is not None
                                  else 0)
                if recovering:
                    lost_frames = (lost_frames + 1
                                   if not candidates and clipped_band is None else 0)

            if (not recovering and state.armed and new_frame
                    and clipped_frames >= cfg.orange_edge_confirm_frames):
                state.armed = False
                state.clear_frames = 0
                state.retry_after_mm = x + cfg.orange_edge_retry_spacing_mm
                clipped_frames = 0
                recovering = True
                recovery_start, recovery_x, recovery_travel = now, x, 0.0
                recovery_band = clipped_band
                lost_frames = 0
                hold_start, hold_spent = None, False
                stop_until = now + cfg.orange_edge_stop_s if speed > 0 else now
                tracker.reset()
                print(f'[{program.operation_name}] Orange left edge clipped; '
                      f'recover left at {cfg.orange_edge_speed_mm_s:.0f} mm/s, '
                      f'max {cfg.orange_edge_max_distance_mm:.0f} mm')

            # Candidate confirmation takes place stopped, with a bounded hold.
            # Merely receiving the same frame cannot confirm or trigger recovery.
            if candidates:
                if hold_start is None:
                    hold_start = now
                if now - hold_start >= cfg.vision_observe_s:
                    hold_spent = True
            else:
                hold_start, hold_spent = None, False

            if recovering:
                reason = None
                if not fresh:
                    reason = 'vision unavailable'
                elif (recovery_x - x >= cfg.orange_edge_max_distance_mm - 1e-6
                      or recovery_travel >= cfg.orange_edge_max_distance_mm - 1e-6):
                    reason = 'distance limit'
                elif now - recovery_start >= cfg.orange_edge_timeout_s:
                    reason = 'time limit'
                elif lost_frames >= cfg.orange_edge_confirm_frames:
                    reason = 'row lost'
                elif candidates and hold_spent:
                    reason = 'candidate confirmation failed'
                if reason is not None:
                    recovering = False
                    tracker.reset()
                    clipped_frames = 0
                    stop_until = now + cfg.orange_edge_stop_s
                    print(f'[{program.operation_name}] Orange recovery ended: {reason}; '
                          'continue right')

            if (program._search_position_mm >= search_limit_mm and not recovering
                    and not (candidates and not hold_spent)):
                return exhausted('distance_budget')
            if (now < stop_until or (candidates and not hold_spent)
                    or (state.armed and clipped_frames > 0 and not recovering)):
                desired = 0.0
            else:
                desired = (-cfg.orange_edge_speed_mm_s if recovering
                           else cfg.search_speed_mm_s)
            if speed * desired < 0.0:
                stop_until = now + cfg.orange_edge_stop_s
                desired = 0.0
            rpm = robot.chassis.mecanum_rpm(0.0, desired / 10.0, 0.0)
            if robot.chassis.set_speeds(rpm) is False:
                raise RuntimeError('orange search speed command failed')
            speed = desired
            delay = cfg.search_control_period_s
            if speed > 0:
                delay = min(delay, (search_limit_mm - program._search_position_mm) / speed)
            elif speed < 0:
                remaining = cfg.orange_edge_max_distance_mm - max(recovery_x-x,recovery_travel)
                delay = min(delay, max(0.0,remaining) / -speed)
            time.sleep(delay)
    finally:
        robot.chassis.set_speeds([0, 0, 0, 0])
        duration = time.monotonic() - started
        span = encoder_span()
        if outcome in ('distance_budget', 'time_budget'):
            print(f'[{program.operation_name}] Orange search ended: {outcome}, '
                  f'command budget={program._search_position_mm:.1f} mm, '
                  f'encoder span={span:.1f} mm, elapsed={duration:.3f}s, '
                  f'fresh frames={fresh_frames}, detected frames={detected_frames}, '
                  f'clipped frames={clipped_observations}, candidate frames={candidate_frames}, '
                  f'max candidates={max_candidates}')
        diagnostics = getattr(robot, 'diagnostics', None)
        if diagnostics is not None:
            with suppress(Exception):
                diagnostics.write('orange_search_result', source=program.operation_name,
                    reason=outcome, command_budget_mm=program._search_position_mm,
                    encoder_span_mm=span, duration_s=duration,
                    phase_search_elapsed_s=state.search_elapsed_s, fresh_frames=fresh_frames,
                    detected_frames=detected_frames, clipped_frames=clipped_observations,
                    candidate_frames=candidate_frames, max_candidates=max_candidates)
