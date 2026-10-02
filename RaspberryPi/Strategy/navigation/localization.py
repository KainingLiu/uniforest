"""Delayed visual pose updates in a single, explicit field frame.

Camera extrinsics must already have been applied by the observation producer.
The estimator uses historical odometry at capture time, never receive-time pose.
"""
from collections import deque
from dataclasses import dataclass
import math
from .planner import Pose, wrap


def compose(a, b):
    c,s=math.cos(math.radians(a.yaw)),math.sin(math.radians(a.yaw))
    return Pose(a.x+c*b.x-s*b.y,a.y+s*b.x+c*b.y,a.yaw+b.yaw)


def inverse(a):
    c,s=math.cos(math.radians(a.yaw)),math.sin(math.radians(a.yaw))
    return Pose(-c*a.x-s*a.y,s*a.x-c*a.y,-a.yaw)


@dataclass(frozen=True)
class Observation:
    pose: Pose
    captured_at: float
    source: str = 'tag6'
    sigma_mm: float = 8.0
    reprojection_px: float = 1.0
    calibrated: bool = True
    frame: str = 'field_mm_cw'


class VisualOdometry:
    def __init__(self):
        self.history=deque()
        self.offset=Pose(0,0,0)
        self.accepted={'tag6':0,'building':0}
        self.last={'tag6':float('-inf'),'building':float('-inf')}
        self.events=[]
        self._seen={'tag6':float('-inf'),'building':float('-inf')}

    def predict(self, stamp, odometry):
        if self.history and stamp<self.history[-1][0]:
            raise RuntimeError('odometry timestamp regressed')
        self.history.append((stamp,odometry))
        while len(self.history)>2 and self.history[1][0]<stamp-1.0:
            self.history.popleft()
        return compose(self.offset,odometry)

    def update(self, observation, now):
        o=observation
        reason=None
        if o.source not in self.accepted: reason='unknown_source'
        elif (not math.isfinite(o.captured_at) or not 0<=now-o.captured_at<=.4): reason='stale_or_future'
        elif not o.calibrated or o.frame!='field_mm_cw': reason='uncalibrated_frame'
        elif (not math.isfinite(o.sigma_mm) or not 0<o.sigma_mm<=30
              or not math.isfinite(o.reprojection_px) or not 0<=o.reprojection_px<=3): reason='quality'
        elif o.captured_at<=self._seen[o.source]: return False
        if reason is None:
            self._seen[o.source]=o.captured_at
            h=list(self.history)
            if not h or o.captured_at<h[0][0] or o.captured_at>h[-1][0]: reason='no_capture_history'
            else:
                old=h[-1][1]
                for (a,p),(b,q) in zip(h,h[1:]):
                    if a<=o.captured_at<=b:
                        t=(o.captured_at-a)/max(b-a,1e-9)
                        old=Pose(p.x+t*(q.x-p.x),p.y+t*(q.y-p.y),p.yaw+t*wrap(q.yaw-p.yaw))
                        break
                predicted=compose(self.offset,old)
                if math.hypot(o.pose.x-predicted.x,o.pose.y-predicted.y)>180 or abs(wrap(o.pose.yaw-predicted.yaw))>12:
                    reason='innovation'
                else:
                    proposed=compose(o.pose,inverse(old))
                    alpha=.35 if o.source=='tag6' else .5
                    self.offset=Pose(self.offset.x+alpha*(proposed.x-self.offset.x),
                                     self.offset.y+alpha*(proposed.y-self.offset.y),
                                     self.offset.yaw+alpha*wrap(proposed.yaw-self.offset.yaw))
                    self.accepted[o.source]+=1
                    self.last[o.source]=o.captured_at
        self.events.append(dict(t=now,source=o.source,accepted=reason is None,reason=reason or 'fused'))
        return reason is None

    @property
    def tag_locked(self):
        return self.accepted['tag6']>=3

    def terminal_ready(self,now):
        return (self.tag_locked and self.accepted['building']>=3
                and now-self.last['building']<=.3)
