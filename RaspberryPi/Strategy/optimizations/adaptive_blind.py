"""Pre-grab neighbor observations for a bounded, encoder-driven blind shift.

The observer consumes existing stopped alignment frames without another wait.
It never authorizes a grip or extends the blind controller's safety envelope.
"""

from dataclasses import dataclass, fields, replace
import math

from ..common import wrap_angle


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class AdaptiveBlindProfile:
    handoff_reserve_mm: float = 20.0
    position_uncertainty_mm: float = 5.0
    camera_x_to_lateral_scale: float = 1.0
    min_gap_mm: float = 60.0
    max_gap_mm: float = 500.0
    max_candidate_drift_mm: float = 10.0
    max_yaw_change_deg: float = 2.0
    settled_speed_mm_s: float = 20.0
    frame_timeout_s: float = .3
    max_prediction_age_s: float = 12.0
    fallback_distance_mm: float = 300.0
    continuous_pitch_mm: float = 100.0
    observed_distance_limit_mm: float = 100.0
    unseen_search_mm: float = 300.0
    confirm_frames: int = 2
    validated: bool = False
    trial_enabled: bool = False

    def __post_init__(self):
        for item in fields(self):
            if item.name in ('validated', 'trial_enabled', 'confirm_frames'):
                continue
            value = getattr(self, item.name)
            if not _finite(value) or value < 0 or (item.name != 'fallback_distance_mm' and value == 0):
                raise ValueError(f'{item.name} must be finite and positive (fallback may be zero)')
        if type(self.validated) is not bool or type(self.trial_enabled) is not bool:
            raise ValueError('adaptive validation flags must be boolean')
        if type(self.confirm_frames) is not int or self.confirm_frames < 2:
            raise ValueError('adaptive preview requires at least two distinct frames')
        if (self.min_gap_mm >= self.max_gap_mm or self.frame_timeout_s > .5
                or self.max_prediction_age_s < self.frame_timeout_s
                or self.max_yaw_change_deg > 10
                or not self.min_gap_mm <= self.continuous_pitch_mm <= self.max_gap_mm
                or not self.continuous_pitch_mm <= self.observed_distance_limit_mm < self.unseen_search_mm):
            raise ValueError('invalid adaptive observation limits')


@dataclass(frozen=True)
class NextCubeObservation:
    captured_s: float
    pose_epoch: int
    link_epoch: int
    stop_generation: int
    target_position_mm: float
    next_position_mm: float
    yaw_deg: float
    uncertainty_mm: float
    source: str = 'measured_neighbor'


def _row_overlap(a, b):
    """Require overlapping top-face image bands, not just orange color."""
    try:
        ay = [float(p[1]) for p in a.quad]
        by = [float(p[1]) for p in b.quad]
        if len(ay) != 4 or len(by) != 4 or not all(map(math.isfinite, ay + by)):
            return False
        height = min(max(ay)-min(ay), max(by)-min(by))
        return height > 0 and min(max(ay), max(by))-max(min(ay), min(by)) >= .5*height
    except (AttributeError, TypeError, ValueError):
        return False


class NeighborObserver:
    """One acquisition's stopped-frame history; consume once before arm motion."""

    def __init__(self, profile):
        if not isinstance(profile, AdaptiveBlindProfile) or not (profile.validated or profile.trial_enabled):
            raise ValueError('adaptive blind motion needs a validated profile or explicit trial')
        self.profile = profile
        self._last_evidence = None
        self._candidate = None
        self._count = 0

    def observe(self, result, target, *, position_mm, speed_mm_s, target_x_mm,
                yaw_deg, now, link_epoch, stop_generation, telemetry_id,
                stopped, acceleration_mm_s2):
        p = self.profile
        captured = getattr(result, 'captured_monotonic', None)
        pose = getattr(result, 'pose_epoch', None)
        values = (captured, position_mm, speed_mm_s, target_x_mm, yaw_deg, now,
                  acceleration_mm_s2)
        if (not all(map(_finite, values)) or not 0 <= now-captured <= p.frame_timeout_s
                or type(pose) is not int or pose < 0 or target is None
                or not all(_finite(getattr(target, k, None)) for k in ('x', 'y', 'z'))
                or not stopped or abs(speed_mm_s) > p.settled_speed_mm_s):
            self._candidate, self._count = None, 0
            return
        evidence = (captured, telemetry_id)
        if self._last_evidence is not None:
            if captured < self._last_evidence[0]:
                self._candidate, self._count = None, 0
                return
            if captured == self._last_evidence[0] or telemetry_id == self._last_evidence[1]:
                return
        self._last_evidence = evidence
        scale = p.camera_x_to_lateral_scale
        if getattr(target, 'continuous_row', False) is True:
            # A row need not expose each seam. Apply one nominal cube-width
            # step, explicitly labelled so it is never mistaken for measurement.
            offset = p.continuous_pitch_mm
            source = 'continuous_row_pitch'
        else:
            candidates = [b for b in getattr(result, 'orange_lookahead', ())
                          if getattr(b, 'color_name', '').casefold() == 'orange'
                          and all(_finite(getattr(b, key, None)) for key in ('x', 'y', 'z'))
                          and (b.x-target.x)*scale > p.max_candidate_drift_mm
                          and _row_overlap(target, b)]
            candidates.sort(key=lambda b: b.x)
            if (candidates and (not p.min_gap_mm <= (candidates[0].x-target.x)*scale <= p.max_gap_mm
                        or len(candidates) > 1
                        and (candidates[1].x-candidates[0].x)*scale <= p.max_candidate_drift_mm)):
                self._candidate, self._count = None, 0
                return
            if candidates:
                offset = (candidates[0].x-target_x_mm)*scale
                source = 'measured_neighbor'
            else:
                # A confirmed current cube in a fresh frame justifies a
                # bounded search when no next target is available. Completeness
                # of the optional lookahead list is not proof of camera health.
                current_visible = any(
                    getattr(b, 'color_name', '').casefold() == 'orange'
                    and _finite(getattr(b, 'x', None))
                    and abs(b.x-target.x)*scale <= p.max_candidate_drift_mm
                    and _row_overlap(target, b)
                    for b in getattr(result, 'all_blocks', getattr(result, 'orange_lookahead', ())))
                if not current_visible:
                    self._candidate, self._count = None, 0
                    return
                offset = p.unseen_search_mm
                source = 'unseen_next_search'
        age = now-captured
        # Shift the encoder sample back to the image time; residual acceleration
        # and calibrated image/encoder error remain an explicit reserve.
        at_capture = position_mm-speed_mm_s*age
        sample = NextCubeObservation(captured, pose, link_epoch, stop_generation,
            at_capture+(target.x-target_x_mm)*scale,
            at_capture+offset, yaw_deg,
            p.position_uncertainty_mm+.5*acceleration_mm_s2*age**2, source)
        previous = self._candidate
        consistent = (previous is not None and previous.pose_epoch == pose
            and previous.link_epoch == link_epoch and previous.stop_generation == stop_generation
            and previous.source == source
            and captured-previous.captured_s <= p.frame_timeout_s
            and abs(wrap_angle(yaw_deg-previous.yaw_deg)) <= p.max_yaw_change_deg
            and abs(sample.target_position_mm-previous.target_position_mm) <= p.max_candidate_drift_mm
            and abs(sample.next_position_mm-previous.next_position_mm) <= p.max_candidate_drift_mm)
        self._count = self._count+1 if consistent else 1
        # Account for observed jitter without moving the endpoint to a mean
        # that could lie beyond a nearer candidate.
        if consistent:
            sample = replace(sample, uncertainty_mm=max(
                sample.uncertainty_mm,
                p.position_uncertainty_mm+abs(sample.next_position_mm-previous.next_position_mm)))
        self._candidate = sample

    def consume(self, *, now, pose_epoch, link_epoch, stop_generation):
        sample = self._candidate
        count = self._count
        self._candidate, self._count = None, 0
        if (sample is None or count < self.profile.confirm_frames
                or not 0 <= now-sample.captured_s <= self.profile.frame_timeout_s
                or (sample.pose_epoch, sample.link_epoch, sample.stop_generation)
                != (pose_epoch, link_epoch, stop_generation)):
            return None
        return sample


def expected_blind_distance(profile, observation, *, position_mm, yaw_deg, now,
                            link_epoch, stop_generation, expected_pose_epoch,
                            legacy_distance_mm):
    """Desired travel only; the existing controller adds its braking margin once."""
    fallback = min(profile.fallback_distance_mm, profile.unseen_search_mm)
    if observation is None:
        return fallback, 'no_confirmed_neighbor_search'
    if (not all(map(_finite, (position_mm, yaw_deg, now)))
            or not 0 <= now-observation.captured_s <= profile.max_prediction_age_s
            or (observation.link_epoch, observation.stop_generation, observation.pose_epoch+1)
            != (link_epoch, stop_generation, expected_pose_epoch)):
        return 0.0, 'observation_expired'
    angle = abs(wrap_angle(yaw_deg-observation.yaw_deg))
    if angle > profile.max_yaw_change_deg:
        return 0.0, 'heading_changed'
    remaining = observation.next_position_mm-position_mm
    if observation.source in ('continuous_row_pitch', 'unseen_next_search'):
        # User-requested nominal travel, minus any motion already made.
        # The blind controller still reserves braking distance and hands off
        # as soon as the restored camera publishes a fresh frame.
        return max(0.0, remaining), observation.source
    yaw_reserve = abs(remaining)*math.sin(math.radians(profile.max_yaw_change_deg))
    return min(profile.observed_distance_limit_mm,
               max(0.0, remaining-profile.handoff_reserve_mm-observation.uncertainty_mm-yaw_reserve)), 'observed_neighbor'
