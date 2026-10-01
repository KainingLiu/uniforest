"""Reusable building visual alignment, independent of mission recipes."""

from collections import deque
from statistics import median
import time

from .tag_controller import PID as _Pid
from .common import VisualAlignmentUnavailable, report_visual_fallback


class BuildingAlignment:
    """Geometric target selection and bounded X/Z/heading alignment."""


    def _building_from_result(self, result, locked_position=None):
        cfg = self.config
        if (result is None
                or time.time() - result.timestamp
                > cfg.building_vision_stale_s):
            return None
        candidates = [
            block for block in result.all_blocks
            if block.color_name.casefold() == 'orange'
            and block.confidence >= cfg.building_min_confidence
            and cfg.building_min_height_width_ratio
            <= getattr(block, 'height_width_ratio', 0.0)
            <= cfg.building_max_height_width_ratio
        ]
        if not candidates:
            return None
        if locked_position is not None:
            previous_x, previous_z = locked_position
            tracked = [
                block for block in candidates
                if abs(self._building_top_reference(block)[0] - previous_x)
                    <= cfg.building_track_max_x_jump_mm
                and abs(self._building_top_reference(block)[1] - previous_z)
                    <= cfg.building_track_max_z_jump_mm
            ]
            if tracked:
                return min(tracked, key=lambda block:
                           (self._building_top_reference(block)[0] - previous_x) ** 2
                           + (self._building_top_reference(block)[1] - previous_z) ** 2)
            return None
        return min(candidates, key=lambda block:
                   (self._building_top_reference(block)[0]
                    - cfg.building_target_x_mm) ** 2
                   + (self._building_top_reference(block)[1]
                      - cfg.building_target_z_mm) ** 2)

    def _building_top_reference(self, block):
        """Return X/Z measured from the visible upper edge of the contour."""
        quad = getattr(block, 'quad', None)
        if quad is None or len(quad) != 4:
            return block.x, block.z
        cfg = self.config
        # order_corners() puts upper-left and upper-right at indices 0/1.
        top_u = (float(quad[0][0]) + float(quad[1][0])) * 0.5
        top_v = (float(quad[0][1]) + float(quad[1][1])) * 0.5
        # The upper edge moves downward as the robot approaches, so Z must
        # decrease with top_v.  Normalize by the measured scale constant.
        z_top = cfg.building_z_scale_mm_px / max(top_v, 1.0)
        x_top = ((top_u - cfg.building_reference_cx_px)
                 * z_top / cfg.building_reference_fx_px)
        return x_top, z_top

    @staticmethod
    def _building_motion_command(pid_output, error, *, deadband,
                                 minimum, maximum, kick_error=25.0):
        """Clamp every non-zero correction above chassis static friction."""
        if abs(error) <= deadband:
            return 0.0
        value = max(-maximum, min(maximum, pid_output))
        if abs(value) < minimum:
            value = minimum if value >= 0.0 else -minimum
        return max(-maximum, min(maximum, value))

    @staticmethod
    def _building_z_command(pid_output, z_error, cfg):
        """Floor the forward/back command, but drop to a creep band near zero.

        A flat 100 mm/s floor near the +/-6 mm acceptance window makes the
        robot overshoot it every control tick and oscillate.  Inside the creep
        start distance the command is clamped to a smaller band so the chassis
        can glide to a stop inside the window; the 100 mm/s floor still starts
        motion from standstill when far away.
        """
        if abs(z_error) <= cfg.building_z_creep_start_mm:
            minimum = cfg.building_z_creep_min_mm_s
            maximum = cfg.building_z_creep_speed_mm_s
        else:
            minimum = cfg.building_min_linear_mm_s
            maximum = cfg.building_max_forward_mm_s
        if abs(z_error) <= cfg.building_z_deadband_mm:
            return 0.0
        value = max(-maximum, min(maximum, pid_output))
        if abs(value) < minimum:
            value = minimum if value >= 0.0 else -minimum
        return max(-maximum, min(maximum, value))

    def _align_building(self):
        cfg = self.config
        forward_pid = _Pid(
            cfg.building_forward_kp, cfg.building_forward_ki,
            cfg.building_forward_kd, cfg.building_linear_integral_limit,
            cfg.building_max_forward_mm_s)
        lateral_pid = _Pid(
            cfg.building_lateral_kp, cfg.building_lateral_ki,
            cfg.building_lateral_kd, cfg.building_linear_integral_limit,
            cfg.building_max_lateral_mm_s)
        heading_pid = _Pid(
            cfg.building_heading_kp, cfg.building_heading_ki,
            cfg.building_heading_kd, cfg.building_heading_integral_limit,
            cfg.building_max_yaw_deg_s)
        samples = deque(maxlen=cfg.building_median_frames)
        started = time.monotonic()
        last_seen = started
        last_update = started
        first_frame_after = time.time()
        last_frame_timestamp = None
        confirmed = 0
        vx = vy = wz = 0.0
        locked_position = None
        pending_position = None
        lock_frames = 0
        set_profile = self._set_cube_profile
        if set_profile is not None:
            # Building align must see the bright top surface so the tracked
            # upper edge is the real top of the stack, not the front-face
            # boundary that biases Z low and forces a spurious forward/back.
            set_profile('building')
        print(f'[{self.operation_name}] Building visual alignment: '
              f'x={cfg.building_target_x_mm:+.1f} mm, '
              f'z={cfg.building_target_z_mm:.1f} mm')
        try:
            while time.monotonic() - started < cfg.building_align_timeout_s:
                self._check_active()
                now = time.monotonic()
                result = self.robot.vision_result
                tracking_position = (locked_position
                                     if locked_position is not None
                                     else pending_position)
                block = self._building_from_result(result, tracking_position)
                frame_timestamp = result.timestamp if result is not None else None
                if (frame_timestamp is not None
                        and frame_timestamp <= first_frame_after):
                    block = None
                if (block is not None
                        and frame_timestamp == last_frame_timestamp):
                    time.sleep(cfg.building_control_period_s)
                    continue
                if frame_timestamp is not None:
                    last_frame_timestamp = frame_timestamp

                if block is None:
                    confirmed = 0
                    samples.clear()
                    vx = vy = wz = 0.0
                    for pid in (forward_pid, lateral_pid, heading_pid):
                        pid.reset()
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    if now - last_seen >= cfg.building_lost_timeout_s:
                        raise VisualAlignmentUnavailable(
                            'three-layer orange building lost before Build')
                    time.sleep(cfg.building_control_period_s)
                    continue

                last_seen = now
                ref_x, ref_z = self._building_top_reference(block)
                if locked_position is None:
                    if pending_position is None:
                        pending_position = (ref_x, ref_z)
                        lock_frames = 1
                    else:
                        pending_position = (ref_x, ref_z)
                        lock_frames += 1
                    if lock_frames >= cfg.building_track_lock_frames:
                        locked_position = pending_position
                        pending_position = None
                samples.append((ref_x, ref_z))
                x_mm = median(item[0] for item in samples)
                z_mm = median(item[1] for item in samples)
                # Gate the next candidate against the filtered position so a
                # single top-edge spike cannot yank the tracker sideways.
                locked_position = (x_mm, z_mm)
                x_error = x_mm - cfg.building_target_x_mm
                z_error = z_mm - cfg.building_target_z_mm
                heading_error = self._heading_error(
                    cfg.build_tag_heading_target_cw_deg)
                x_ok = abs(x_error) <= cfg.building_x_tolerance_mm
                z_ok = abs(z_error) <= cfg.building_z_tolerance_mm
                heading_ok = (
                    abs(heading_error)
                    <= cfg.building_heading_tolerance_deg)
                if x_ok and z_ok and heading_ok:
                    self.robot.chassis.set_speeds([0, 0, 0, 0])
                    vx = vy = wz = 0.0
                    confirmed += 1
                    print(f'[{self.operation_name}] Building aligned '
                          f'{confirmed}/{cfg.building_confirm_frames}: '
                          f'x={x_mm:+.1f} mm, z={z_mm:.1f} mm, '
                          f'gyro={heading_error:+.1f} deg')
                    if confirmed >= cfg.building_confirm_frames:
                        return
                else:
                    confirmed = 0
                    dt = max(0.001, min(0.2, now - last_update))
                    # Lateral alignment has priority: while X is out of
                    # tolerance, hold Z still and re-center X; once X is in
                    # tolerance, hold X and close Z.  Each axis only moves when
                    # it is actually outside its own window, so an aligned axis
                    # is never re-kicked at minimum speed, and an axis that
                    # later drifts out is corrected again (no one-shot latch).
                    if not x_ok:
                        forward_pid.reset()
                        desired_vx = 0.0
                        if abs(x_error) <= cfg.building_x_deadband_mm:
                            lateral_pid.reset()
                            desired_vy = 0.0
                        else:
                            desired_vy = lateral_pid.update(x_error, dt)
                            desired_vy = self._building_motion_command(
                                desired_vy, x_error,
                                deadband=cfg.building_x_deadband_mm,
                                minimum=cfg.building_min_linear_mm_s,
                                maximum=cfg.building_max_lateral_mm_s)
                    else:
                        lateral_pid.reset()
                        desired_vy = 0.0
                        if abs(z_error) <= cfg.building_z_deadband_mm:
                            forward_pid.reset()
                            desired_vx = 0.0
                        elif not z_ok:
                            # Camera Z is positive forward (away from the
                            # camera), matching chassis +vx, so a positive
                            # error commands forward motion toward the target.
                            desired_vx = forward_pid.update(z_error, dt)
                            desired_vx = self._building_z_command(
                                desired_vx, z_error, cfg)
                        else:
                            forward_pid.reset()
                            desired_vx = 0.0
                    if (abs(heading_error)
                            <= cfg.building_heading_deadband_deg):
                        heading_pid.reset()
                        desired_wz = 0.0
                    elif not heading_ok:
                        desired_wz = heading_pid.update(heading_error, dt)
                        desired_wz = self._minimum_command(
                            desired_wz, cfg.building_min_yaw_deg_s)
                    else:
                        heading_pid.reset()
                        desired_wz = 0.0
                    vx = self._slew_command(
                        desired_vx, vx,
                        cfg.building_linear_accel_mm_s2, dt)
                    vy = self._slew_command(
                        desired_vy, vy,
                        cfg.building_linear_accel_mm_s2, dt)
                    wz = self._slew_command(
                        desired_wz, wz,
                        cfg.building_yaw_accel_deg_s2, dt)
                    rpm = self.robot.chassis.mecanum_rpm(
                        vx / 10.0, vy / 10.0, wz)
                    self.robot.chassis.set_speeds(rpm)
                    print(f'[{self.operation_name}] Building PID: '
                          f'x={x_mm:+.1f} mm, z={z_mm:.1f} mm, '
                          f'gyro={heading_error:+.1f} deg; '
                          f'vx={vx:+.0f}, vy={vy:+.0f} mm/s, '
                          f'wz={wz:+.1f} deg/s')
                last_update = now
                time.sleep(cfg.building_control_period_s)
        finally:
            self.robot.chassis.set_speeds([0, 0, 0, 0])
            if set_profile is not None:
                set_profile('default')
        raise VisualAlignmentUnavailable('building visual alignment timed out')

    def _align_building_or_continue(self) -> bool:
        try:
            self._align_building()
            return True
        except VisualAlignmentUnavailable as exc:
            self._check_active()
            report_visual_fallback(self.robot, self.operation_name, 'building alignment', exc)
            return False
