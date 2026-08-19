"""
Chassis control — replicates the upper-layer logic from motor3508.c.

Provides:
- Mecanum inverse kinematics (vx, vy, wz → 4×wheel RPM)
- PID controller for position and yaw
- High-level move commands: forward, right, turn
- Runtime PID tuning via transport

All math matches the STM32 implementation (same constants, same sign conventions).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, List, Optional, Tuple

from protocol.commands import (
    TelemBatch, M3508_IDX_TR, M3508_IDX_TL, M3508_IDX_BL, M3508_IDX_BR,
)

if TYPE_CHECKING:
    from protocol.transport import Transport

# ====================== Geometry Constants ===================================
# Must match motor3508.h exactly.

ENCODER_COUNTS_PER_REV = 8192
M3508_GEAR_RATIO       = 3591.0 / 187.0    # ≈ 19.203
WHEEL_DIAMETER_MM      = 152.4
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM
WHEEL_RADIUS_MM        = WHEEL_DIAMETER_MM / 2.0

CHASSIS_WHEELBASE_MM    = 240.0
CHASSIS_TRACK_MM        = 391.0
CHASSIS_HALF_DIAGONAL_MM = (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) / 2.0

# RPM per cm/s:  60 * gear_ratio / (π * diameter_cm)
MECANUM_RPM_PER_CM_S = (60.0 * M3508_GEAR_RATIO
                        / (math.pi * WHEEL_DIAMETER_MM / 10.0))

# Counts per cm of chassis travel
COUNTS_PER_CM = int(10.0 * M3508_GEAR_RATIO
                    * ENCODER_COUNTS_PER_REV
                    / WHEEL_CIRCUMFERENCE_MM)

# °/s → motor RPM conversion
TURN_DEG_S_TO_RPM = (MECANUM_RPM_PER_CM_S
                      * (CHASSIS_HALF_DIAGONAL_MM / 10.0)
                      * math.pi / 180.0)

# ====================== Default PID Gains ====================================

# Speed loop (runs on STM32 at 1 kHz)
DEFAULT_SPEED_KP = 10.0
DEFAULT_SPEED_KI = 0.05
DEFAULT_SPEED_KD = 0.0
DEFAULT_SPEED_I_LIM = 6000.0
DEFAULT_SPEED_O_LIM = 15000.0

# Position loop (runs on Pi at control rate)
DEFAULT_POS_KP = 0.10
DEFAULT_POS_KI = 0.003
DEFAULT_POS_KD = 0.0
DEFAULT_POS_I_LIM = 800.0
DEFAULT_POS_O_LIM = 800.0

# Yaw correction
DEFAULT_YAW_KP = 80.0
DEFAULT_YAW_KI = 0.4
DEFAULT_YAW_I_LIM = 1000.0

# ====================== Move Parameters ======================================

LONG_DISTANCE_MOVE_SPEED_MM_S = 600.0
LONG_DISTANCE_FORWARD_ACCEL_MS = 800
DEFAULT_MOVE_SPEED_MM_S = LONG_DISTANCE_MOVE_SPEED_MM_S
FWD_BASE_SPEED_RPM   = 1800.0
FWD_ACCEL_MS         = LONG_DISTANCE_FORWARD_ACCEL_MS
FWD_BASE_DECEL_DIST  = 90000   # encoder counts
FWD_BASE_PID_LIMIT   = 800.0
FWD_HOLD_MS          = 700
FWD_TIMEOUT_MS       = 5000
FWD_SETTLE_COUNTS    = 400
FWD_SETTLE_MS        = 50
FWD_SETTLE_SPEED_RPM = 20
FWD_TIMEOUT_MARGIN_MS = 2000

# Field calibration on the competition mat: a 500 mm wheel-side command moves
# the chassis about 465 mm laterally. Command the reciprocal wheel travel.
LATERAL_DISTANCE_SCALE = 500.0 / 465.0

TURN_BASE_SPEED_DEG_S = 90.0
TURN_ACCEL_MS        = 600
TURN_BASE_DECEL_DEG  = 30.0
TURN_BASE_PID_LIMIT  = 60.0
TURN_HOLD_MS         = 500
TURN_TIMEOUT_MS      = 5000
TURN_SETTLE_DEG      = 1.5
TURN_SETTLE_CYCLES   = 30


@dataclass(frozen=True)
class LinearMoveResult:
    requested_mm: float
    target_counts: int
    projected_counts: int
    wheel_counts: Tuple[int, int, int, int]
    elapsed_ms: float
    timed_out: bool
    cancelled: bool = False
    distance_scale: float = 1.0

    @property
    def encoder_distance_mm(self) -> float:
        return self.projected_counts * 10.0 / COUNTS_PER_CM

    @property
    def estimated_chassis_distance_mm(self) -> float:
        return self.encoder_distance_mm / self.distance_scale


class PID:
    """Generic PID controller (matches STM32 PID_Compute)."""
    def __init__(self, kp: float, ki: float, kd: float,
                 integral_limit: float, output_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        p_term = self.kp * error

        self.integral += error * dt
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
        i_term = self.ki * self.integral

        d_term = 0.0
        if dt > 0.000001:
            d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        output = p_term + i_term + d_term
        if output > self.output_limit:
            output = self.output_limit
        elif output < -self.output_limit:
            output = -self.output_limit
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class Chassis:
    """
    Chassis motor control.

    Usage:
        chassis = Chassis(transport)
        chassis.start()                    # enable telemetry
        chassis.set_speeds([100, -100, 100, -100])  # 4×RPM
        chassis.move_forward(500, 200)     # forward 500mm at 200mm/s
    """

    def __init__(self, transport: Transport,
                 lateral_distance_scale: float = LATERAL_DISTANCE_SCALE):
        if lateral_distance_scale <= 0.0:
            raise ValueError('lateral_distance_scale must be positive')
        self._t = transport
        self._telem: Optional[TelemBatch] = None
        self.lateral_distance_scale = lateral_distance_scale

        # Position PID controllers (one per motor)
        self.pos_pid = [
            PID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD,
                DEFAULT_POS_I_LIM, DEFAULT_POS_O_LIM)
            for _ in range(4)
        ]

        # Yaw PID
        self.yaw_pid = PID(DEFAULT_YAW_KP, DEFAULT_YAW_KI, 0.0,
                           DEFAULT_YAW_I_LIM, DEFAULT_YAW_I_LIM)

        # State
        self._yaw_offset = 0.0
        self._motor_signs = [-1, 1, 1, -1]  # fwd sign per motor

    # ==================== Telemetry ===========================================

    @property
    def telem(self) -> Optional[TelemBatch]:
        return self._telem

    def update_telem(self, telem: TelemBatch):
        """Called by main loop when new telemetry arrives."""
        self._telem = telem

    def has_telem(self) -> bool:
        return self._telem is not None

    def set_lateral_distance_scale(self, scale: float):
        """Set empirical mecanum lateral compensation (1.0 = geometric)."""
        if scale <= 0.0:
            raise ValueError('lateral distance scale must be positive')
        self.lateral_distance_scale = scale

    @property
    def yaw(self) -> float:
        if self._telem:
            return self._telem.yaw_deg
        return 0.0

    @property
    def yaw_rate(self) -> float:
        if self._telem:
            return self._telem.yaw_rate_ds
        return 0.0

    def motor_speed(self, idx: int) -> int:
        """Get current motor speed in RPM."""
        if self._telem and idx < 4:
            return self._telem.motors[idx].speed_rpm
        return 0

    def motor_position(self, idx: int) -> int:
        """Get the A-board's multi-turn encoder position."""
        if self._telem and idx < 4:
            return self._telem.motors[idx].cumulative_pos
        return 0

    def capture_motor_positions(self) -> Tuple[int, int, int, int]:
        """Snapshot all chassis multi-turn encoders for odometry deltas."""
        telem = self._telem
        if telem is None:
            raise RuntimeError('encoder snapshot requires active telemetry')
        return tuple(m.cumulative_pos for m in telem.motors)

    def lateral_displacement_mm(
            self, origin: Tuple[int, int, int, int]) -> float:
        """Return signed chassis displacement, positive toward robot right."""
        if len(origin) != 4:
            raise ValueError('lateral encoder origin must contain 4 positions')
        telem = self._telem
        if telem is None:
            raise RuntimeError('lateral displacement requires active telemetry')
        _, projected_counts = self._project_wheel_positions(
            telem, origin, [1, 1, -1, -1])
        wheel_distance_mm = projected_counts * 10.0 / COUNTS_PER_CM
        return wheel_distance_mm / self.lateral_distance_scale

    # ==================== Mecanum Kinematics ==================================

    @staticmethod
    def mecanum_rpm(vx_cm_s: float, vy_cm_s: float,
                    wz_deg_s: float) -> List[float]:
        """
        Convert chassis velocity to 4 wheel RPMs.

        Args:
            vx_cm_s:  forward velocity in cm/s (+=forward)
            vy_cm_s:  lateral velocity in cm/s (+=right)
            wz_deg_s: angular velocity in °/s (+=CCW from top view)

        Returns:
            [TR_rpm, TL_rpm, BL_rpm, BR_rpm] — motor sign convention

        Matches Motor3508_MecanumRPM() in motor3508.c.
        """
        rpm_per = MECANUM_RPM_PER_CM_S
        base = vx_cm_s * rpm_per
        lat  = vy_cm_s * rpm_per
        wz_rad = wz_deg_s * math.pi / 180.0
        rot = wz_rad * (CHASSIS_HALF_DIAGONAL_MM / 10.0) * rpm_per

        return [
            -base + lat - rot,   # TR: γ=-1, +vx→negative, +vy→positive
            +base + lat - rot,   # TL: γ=-1, +vx→positive, +vy→positive
            +base - lat - rot,   # BL: γ=-1, +vx→positive, +vy→negative
            -base - lat - rot,   # BR: γ=-1, +vx→negative, +vy→negative
        ]

    # ==================== Low-Level Control ===================================

    def set_speeds(self, rpm: List[float]):
        """Send 4×RPM targets to STM32 speed PID."""
        self._t.set_chassis_speed([int(round(r)) for r in rpm[:4]])

    def set_torques(self, torque: List[int]):
        """Send 4×raw torque commands (bypasses PID)."""
        self._t.set_chassis_torque(torque[:4])

    def stop(self):
        """Emergency stop all chassis motors."""
        self._t.emergency_stop()
        for pid in self.pos_pid:
            pid.reset()
        self.yaw_pid.reset()

    # ==================== PID Tuning ==========================================

    def set_speed_pid(self, motor_id: int, kp: float, ki: float,
                      kd: float, ilim: float, olim: float):
        """Adjust speed-loop PID on STM32 for one motor."""
        self._t.set_chassis_speed_pid(motor_id, kp, ki, kd, ilim, olim)

    def set_pos_pid(self, motor_id: int, kp: float, ki: float,
                    kd: float, ilim: float, olim: float):
        """Adjust position-loop PID on STM32 for one motor."""
        self._t.set_chassis_pos_pid(motor_id, kp, ki, kd, ilim, olim)

    def reset_pid(self, motor_id: int):
        """Reset PID integrators on STM32 for one motor."""
        self._t.reset_chassis_pid(motor_id)

    def set_all_speed_pid(self, kp: float, ki: float, kd: float,
                          ilim: float, olim: float):
        """Set same speed PID for all 4 motors."""
        for i in range(4):
            self.set_speed_pid(i, kp, ki, kd, ilim, olim)

    def set_all_pos_pid(self, kp: float, ki: float, kd: float,
                        ilim: float, olim: float):
        """Set same position PID for all 4 motors."""
        for i in range(4):
            self.set_pos_pid(i, kp, ki, kd, ilim, olim)

    # ==================== High-Level Move Commands =============================
    # These replicate MoveForward/MoveRight/Turn from motor3508.c.
    # They run BLOCKING on the Pi — call from a task thread.
    # STM32 handles the speed PID; Pi handles the position loop.

    @staticmethod
    def _smoothstep(r: float) -> float:
        """S-curve: r²(3-2r), zero-slope at both ends."""
        return r * r * (3.0 - 2.0 * r)

    @staticmethod
    def _mm_s_to_rpm(mm_s: float) -> float:
        return mm_s * MECANUM_RPM_PER_CM_S / 10.0

    @staticmethod
    def _project_wheel_positions(telem: TelemBatch,
                                 origin: Tuple[int, int, int, int],
                                 signs: List[int]) -> Tuple[
                                     Tuple[int, int, int, int], int]:
        wheel_counts = tuple(
            signs[i] * (telem.motors[i].cumulative_pos - origin[i])
            for i in range(4)
        )
        return wheel_counts, sum(wheel_counts) // 4

    @staticmethod
    def _linear_settled(telem: TelemBatch, remaining: int) -> bool:
        return (abs(remaining) <= FWD_SETTLE_COUNTS
                and max(abs(m.speed_rpm) for m in telem.motors)
                <= FWD_SETTLE_SPEED_RPM)

    def _move_linear(self, target_counts: int, signs: List[int],
                     speed_rpm: float, t: Transport,
                     telem_getter, requested_mm: float,
                     sleep_fn=time.sleep,
                     cancel_event=None,
                     distance_scale: float = 1.0,
                     hold_ms: int = FWD_HOLD_MS,
                     accel_ms: int = FWD_ACCEL_MS) -> LinearMoveResult:
        """
        Blocking position-loop linear move with S-curve feedforward.
        Matches _move_linear() in motor3508.c.
        """
        control_period = 0.02  # 50 Hz, matching the default telemetry rate
        scale = speed_rpm / FWD_BASE_SPEED_RPM
        decel_dist = FWD_BASE_DECEL_DIST * scale
        pid_limit = FWD_BASE_PID_LIMIT * scale

        if target_counts <= 0 or speed_rpm <= 0.0:
            raise ValueError('distance and speed must be positive')

        target_wheel_mm = target_counts * 10.0 / COUNTS_PER_CM
        wheel_speed_mm_s = speed_rpm * 10.0 / MECANUM_RPM_PER_CM_S
        timeout_ms = max(
            FWD_TIMEOUT_MS,
            target_wheel_mm / wheel_speed_mm_s * 1000.0
            + accel_ms + hold_ms + FWD_TIMEOUT_MARGIN_MS,
        )

        first_telem = telem_getter()
        if first_telem is None:
            raise RuntimeError('position move requires active telemetry')
        origin = tuple(m.cumulative_pos for m in first_telem.motors)
        start_yaw = first_telem.yaw_deg

        # Reset state
        for pid in self.pos_pid:
            pid.reset()
            pid.output_limit = 0.0  # ramp up from zero
        self.yaw_pid.reset()

        t_start = time.monotonic()
        last_update = t_start
        settle_cnt = 0
        settle_cycles = max(1, math.ceil(FWD_SETTLE_MS / (control_period * 1000.0)))
        hold_active = False
        hold_start = 0.0
        position_lock = False
        timed_out = False
        cancelled = False
        wheel_counts = (0, 0, 0, 0)
        projected_counts = 0

        while True:
            t0 = time.monotonic()

            if cancel_event is not None and cancel_event.is_set():
                cancelled = True
                break

            # Read latest telemetry
            telem = telem_getter()
            if telem is None:
                if (time.monotonic() - t_start) * 1000.0 >= timeout_ms:
                    timed_out = True
                    break
                sleep_fn(control_period)
                continue

            now = time.monotonic()
            dt = max(0.001, min(0.1, now - last_update))
            last_update = now

            # Project each wheel's action-local displacement onto the requested
            # chassis axis. This is the same reference used by the 0714 loop.
            wheel_counts, projected_counts = self._project_wheel_positions(
                telem, origin, signs)
            remaining = target_counts - projected_counts
            if remaining <= 0:
                position_lock = True

            # Feedforward
            elapsed_ms = (now - t_start) * 1000.0
            if position_lock:
                # No feedforward in the position-lock phase or after an
                # overshoot. The signed position PID must be able to reverse.
                for pid in self.pos_pid:
                    pid.output_limit = pid_limit
                ff = 0.0
            elif accel_ms > 0 and elapsed_ms < accel_ms:
                r = elapsed_ms / accel_ms
                for pid in self.pos_pid:
                    pid.output_limit = pid_limit * r
                ff = speed_rpm * self._smoothstep(r)
            else:
                for pid in self.pos_pid:
                    pid.output_limit = pid_limit
                if remaining > decel_dist:
                    ff = speed_rpm
                else:
                    ff = speed_rpm * self._smoothstep(remaining / decel_dist)

            pid_corr = self.pos_pid[0].compute(
                float(target_counts), float(projected_counts), dt)
            speed_sp = ff + pid_corr

            # Match IMU_ResetYaw() in the 0714 implementation without changing
            # the global telemetry reference used by other upper-computer code.
            yaw_delta = telem.yaw_deg - start_yaw
            while yaw_delta > 180.0:
                yaw_delta -= 360.0
            while yaw_delta < -180.0:
                yaw_delta += 360.0
            yaw_corr = self.yaw_pid.compute(0.0, -yaw_delta, dt)

            # Distribute to 4 wheels
            rpm = [speed_sp * signs[i] + yaw_corr for i in range(4)]
            if cancel_event is not None and cancel_event.is_set():
                cancelled = True
                break
            t.set_chassis_speed([int(round(r)) for r in rpm])

            # Settle → hold
            elapsed = (time.monotonic() - t_start) * 1000.0

            if elapsed >= timeout_ms:
                timed_out = True
                break

            settled = self._linear_settled(telem, remaining)

            if not hold_active:
                if settled:
                    settle_cnt += 1
                    if settle_cnt >= settle_cycles:
                        hold_active = True
                        position_lock = True
                        hold_start = elapsed
                else:
                    settle_cnt = 0
            elif not settled:
                # Position lock must be continuous. Re-enter acquisition if
                # load, inertia or floor slip moves the chassis out of bounds.
                hold_active = False
                settle_cnt = 0
            elif elapsed - hold_start >= hold_ms:
                break

            # Maintain ~50 Hz
            elapsed_loop = time.monotonic() - t0
            if elapsed_loop < control_period:
                sleep_fn(control_period - elapsed_loop)

        t.set_chassis_speed([0, 0, 0, 0])
        elapsed_ms = (time.monotonic() - t_start) * 1000.0
        return LinearMoveResult(
            requested_mm=requested_mm,
            target_counts=target_counts,
            projected_counts=projected_counts,
            wheel_counts=wheel_counts,
            elapsed_ms=elapsed_ms,
            timed_out=timed_out,
            cancelled=cancelled,
            distance_scale=distance_scale,
        )

    def move_forward(self, distance_mm: float,
                     speed_mm_s: float = DEFAULT_MOVE_SPEED_MM_S,
                     cancel_event=None,
                     hold_ms: int = FWD_HOLD_MS,
                     accel_ms: int = FWD_ACCEL_MS) -> LinearMoveResult:
        """
        Move forward by distance_mm at speed_mm_s (blocking).

        Uses encoder-based position loop + IMU yaw correction.
        Speed PID runs on STM32 at 1 kHz; position loop runs here at ~50 Hz.
        """
        direction = 1 if distance_mm >= 0 else -1
        requested_mm = abs(distance_mm)
        target = int(requested_mm * COUNTS_PER_CM / 10.0)
        signs = [direction * s for s in (-1, 1, 1, -1)]
        speed_rpm = self._mm_s_to_rpm(speed_mm_s)
        return self._move_linear(target, signs, speed_rpm,
                                 self._t, lambda: self._telem, requested_mm,
                                 cancel_event=cancel_event,
                                 distance_scale=1.0,
                                 hold_ms=hold_ms,
                                 accel_ms=accel_ms)

    def move_right(self, distance_mm: float,
                   speed_mm_s: float = DEFAULT_MOVE_SPEED_MM_S,
                   cancel_event=None,
                   hold_ms: int = FWD_HOLD_MS,
                   accel_ms: int = FWD_ACCEL_MS) -> LinearMoveResult:
        """
        Move right (lateral) by distance_mm at speed_mm_s (blocking).
        """
        direction = 1 if distance_mm >= 0 else -1
        requested_mm = abs(distance_mm)
        target = int(requested_mm * self.lateral_distance_scale
                     * COUNTS_PER_CM / 10.0)
        signs = [direction * s for s in (1, 1, -1, -1)]
        speed_rpm = self._mm_s_to_rpm(speed_mm_s)
        return self._move_linear(target, signs, speed_rpm,
                                 self._t, lambda: self._telem, requested_mm,
                                 cancel_event=cancel_event,
                                 distance_scale=self.lateral_distance_scale,
                                 hold_ms=hold_ms,
                                 accel_ms=accel_ms)

    def turn(self, target_deg: float, speed_deg_s: float,
             hold_ms: int = TURN_HOLD_MS,
             settle_cycles: int = TURN_SETTLE_CYCLES):
        """
        Turn by target_deg degrees (+ = CW, - = CCW) at speed_deg_s (blocking).

        Uses IMU yaw for feedback. Speed PID on STM32; position PID here.
        """
        # Internal convention: + = CCW. Flip user convention.
        target = -target_deg

        telem = self._telem
        if telem is None:
            return

        dt = 0.02
        sign_dir = 1 if target >= 0 else -1
        scale = speed_deg_s / TURN_BASE_SPEED_DEG_S
        decel_deg = TURN_BASE_DECEL_DEG * scale
        pid_limit = TURN_BASE_PID_LIMIT * scale

        self.yaw_pid.reset()
        self.yaw_pid.output_limit = 0.0

        # Integrate adjacent yaw samples so crossing +/-180 degrees does not
        # turn a small overshoot into an apparent full-revolution error.
        previous_yaw = telem.yaw_deg
        accumulated_yaw = 0.0

        pos_pid = PID(3.0, 0.15, 0.0, pid_limit, pid_limit)

        t_start = time.time()
        settle_cnt = 0
        hold_active = False
        hold_start = 0.0

        while True:
            telem = self._telem
            if telem is None:
                time.sleep(dt)
                continue

            current_yaw = telem.yaw_deg
            yaw_delta = current_yaw - previous_yaw
            while yaw_delta > 180:
                yaw_delta -= 360
            while yaw_delta < -180:
                yaw_delta += 360
            accumulated_yaw += yaw_delta
            previous_yaw = current_yaw
            yaw = accumulated_yaw

            err = target - yaw
            remaining = abs(err)

            elapsed_ms = (time.time() - t_start) * 1000.0

            if not hold_active:
                if remaining <= TURN_SETTLE_DEG:
                    settle_cnt += 1
                    if settle_cnt >= settle_cycles:
                        hold_active = True
                        hold_start = elapsed_ms
                        pos_pid.reset()
                else:
                    settle_cnt = 0

            if hold_active:
                pos_pid.output_limit = pid_limit
                ff_deg_s = 0.0
            elif elapsed_ms < TURN_ACCEL_MS:
                r = elapsed_ms / TURN_ACCEL_MS
                pos_pid.output_limit = pid_limit * r
                ff_deg_s = speed_deg_s * self._smoothstep(r)
            else:
                pos_pid.output_limit = pid_limit
                if remaining > decel_deg:
                    ff_deg_s = speed_deg_s
                else:
                    ff_deg_s = speed_deg_s * self._smoothstep(
                        remaining / decel_deg if decel_deg > 0 else 1.0)

            pid_deg_s = pos_pid.compute(target, yaw, dt)
            rot_deg_s = ff_deg_s * sign_dir + pid_deg_s

            rot_rpm = rot_deg_s * TURN_DEG_S_TO_RPM
            self._t.set_chassis_speed([int(round(-rot_rpm))] * 4)

            elapsed_ms = (time.time() - t_start) * 1000.0
            if elapsed_ms >= TURN_TIMEOUT_MS:
                break

            if hold_active and (elapsed_ms - hold_start) >= hold_ms:
                break

            time.sleep(dt)

        self.stop()
