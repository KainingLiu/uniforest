"""Run-scoped map pose from every encoder/IMU frame, including non-route motion.

The initial pose is a placement convention, not a camera measurement. No route
completion, heading rebase, or nominal waypoint may reset this measured progress.
"""
import math
import threading
import time

from control.chassis import COUNTS_PER_CM, MECANUM_RPM_PER_CM_S, TURN_DEG_S_TO_RPM
from control.trajectory import BodyVelocity
from .planner import Pose, NavigationError, wrap
from .control import MotionFeedback


class RouteOdometry:
    def __init__(self, start, lateral_scale, link_generation, stop_generation):
        if not isinstance(start, Pose) or not math.isfinite(lateral_scale) or lateral_scale <= 0:
            raise ValueError('finite initial pose and positive lateral scale required')
        self.pose = start
        self.lateral_scale = lateral_scale
        self.link_generation, self.stop_generation = link_generation, stop_generation
        self.previous = None
        self.received_at = None
        self.velocity = BodyVelocity()
        self.fault = None
        self._lock = threading.Lock()

    def update(self, telem, received_at, link_generation, stop_generation):
        # The serial reader must remain alive to deliver stops and diagnostics.
        # A malformed/discontinuous frame latches a fault for the route owner.
        with self._lock:
            if self.fault is not None:
                return
            try:
                self._update(telem, received_at, link_generation, stop_generation)
            except (ValueError, TypeError, AttributeError, NavigationError) as exc:
                self.fault = str(exc)

    def _update(self, telem, stamp, link, stop):
        if link != self.link_generation or stop != self.stop_generation:
            raise NavigationError('route odometry continuity lost; restart the task explicitly')
        counts = tuple(m.cumulative_pos for m in telem.motors)
        speeds = tuple(m.speed_rpm for m in telem.motors)
        yaw, uptime = telem.yaw_deg, telem.uptime_ms
        if len(counts) != 4 or len(speeds) != 4 or not all(
                isinstance(v, (int, float)) and math.isfinite(v)
                for v in (*counts, *speeds, yaw, stamp, uptime)):
            raise NavigationError('invalid route odometry telemetry')
        if self.previous is not None:
            old_counts, old_yaw, old_uptime = self.previous
            if not 0 <= stamp-self.received_at <= .15:
                raise NavigationError('route odometry telemetry gap')
            if ((uptime-old_uptime) & 0xffffffff) >= 0x80000000:
                raise NavigationError('A-board restarted during route odometry')
            delta = tuple((b-a+0x80000000) % 0x100000000-0x80000000
                          for a,b in zip(old_counts, counts))
            tr,tl,bl,br = delta
            dx = (-tr+tl+bl-br)*10/(4*COUNTS_PER_CM)
            dy = (tr+tl-bl-br)*10/(4*COUNTS_PER_CM*self.lateral_scale)
            da = -wrap(yaw-old_yaw)
            angle = math.radians(self.pose.yaw+da/2)
            self.pose = Pose(self.pose.x+math.cos(angle)*dx-math.sin(angle)*dy,
                             self.pose.y+math.sin(angle)*dx+math.cos(angle)*dy,
                             self.pose.yaw+da)
        tr,tl,bl,br = speeds
        self.velocity = BodyVelocity((-tr+tl+bl-br)*10/(4*MECANUM_RPM_PER_CM_S),
            (tr+tl-bl-br)*10/(4*MECANUM_RPM_PER_CM_S*self.lateral_scale),
            (tr+tl+bl+br)/(4*TURN_DEG_S_TO_RPM))
        self.previous = counts, yaw, uptime
        self.received_at = stamp

    def snapshot(self, now=None):
        with self._lock:
            now = time.monotonic() if now is None else now
            if self.fault is not None:
                raise NavigationError(self.fault)
            if self.received_at is None or not 0 <= now-self.received_at <= .15:
                raise NavigationError('route odometry unavailable or stale')
            return MotionFeedback(self.pose, self.velocity, self.received_at)
