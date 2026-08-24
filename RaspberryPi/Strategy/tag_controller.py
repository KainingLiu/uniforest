"""Reusable PID primitives for visual-tag alignment."""

from __future__ import annotations

from dataclasses import dataclass


def profiled_command(pid_output: float, error: float, *,
                     slowdown_start: float, creep_start: float,
                     fast_speed: float, max_speed: float,
                     min_speed: float) -> float:
    """Apply a smooth distance-based speed envelope to a PID command."""
    magnitude = abs(error)
    if magnitude <= creep_start:
        return max(-max_speed, min(max_speed, pid_output))
    if magnitude >= slowdown_start:
        floor = fast_speed
    else:
        ratio = ((magnitude - creep_start) /
                 max(slowdown_start - creep_start, 1e-6))
        floor = min_speed + ratio * (fast_speed - min_speed)
    sign = 1.0 if pid_output >= 0.0 else -1.0
    value = max(abs(pid_output), floor)
    return sign * min(max_speed, value)


class PID:
    """Bounded PID with explicit reset, suitable for a 50 Hz control loop."""

    def __init__(self, kp: float, ki: float, kd: float,
                 integral_limit: float, output_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.previous_error = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        dt = max(float(dt), 1e-6)
        self.integral = max(-self.integral_limit,
                            min(self.integral_limit,
                                self.integral + error * dt))
        derivative = (0.0 if self.previous_error is None else
                      (error - self.previous_error) / dt)
        self.previous_error = error
        output = (self.kp * error + self.ki * self.integral +
                  self.kd * derivative)
        return max(-self.output_limit, min(self.output_limit, output))


@dataclass
class TagPidSet:
    distance: PID
    lateral: PID
    heading: PID

    def reset(self):
        self.distance.reset()
        self.lateral.reset()
        self.heading.reset()


__all__ = ['PID', 'TagPidSet', 'profiled_command']
