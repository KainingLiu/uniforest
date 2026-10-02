"""Scalar motion laws shared by classic moves and continuous navigation."""


def smoothstep(r):
    """Existing cubic velocity ramp, r in [0, 1]. Also accepts numpy arrays."""
    return r * r * (3.0 - 2.0 * r)


def route_feedforward(speed, elapsed, accel_time, remaining, braking_distance):
    """Classic route feedforward; units must be consistent at the caller."""
    ramp = min(1.0, elapsed / accel_time) if accel_time > 0 else 1.0
    distance_ratio = min(1.0, max(0.0, remaining / braking_distance))
    return speed * min(smoothstep(ramp), smoothstep(distance_ratio))
