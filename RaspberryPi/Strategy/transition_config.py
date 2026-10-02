"""Explicit calibration data for opt-in pickup and continuous-route transitions."""

from dataclasses import dataclass, field, fields
from datetime import date
import json
import math
from pathlib import Path
from types import MappingProxyType

from .settings import PROFILES
from .execution.blind import BlindMotionProfile
from .transition_switches import TransitionSwitches
from .optimizations.adaptive_blind import AdaptiveBlindProfile

DEFAULT_NEXT_CUBE_SHIFT_MM = 80.0


def _number(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} must be finite')
    if value < 0 or positive and value <= 0:
        raise ValueError(f'{label} must be nonnegative' if not positive else f'{label} must be positive')
    return float(value)


def _record(value):
    if value.get('validated') is not True:
        raise ValueError('enabled transition requires validated=true and a field record')
    date.fromisoformat(value['verified_on'])
    if not isinstance(value.get('notes'), str) or not value['notes'].strip():
        raise ValueError('transition calibration notes are required')


def _strict(value, keys, label):
    if not isinstance(value, dict) or set(value) - set(keys):
        raise ValueError(f'{label}: unknown fields or invalid object')


def _profile(cls, value):
    _strict(value, {f.name for f in fields(cls)}, cls.__name__)
    result = cls(**value)
    validator = getattr(result,'validate',None)
    if validator is not None:
        validator()
    if not result.validated:
        raise ValueError(f'{cls.__name__} must be field validated')
    return result


@dataclass(frozen=True)
class PickupCalibration:
    arm_restore_s: float
    next_blind: object = None
    next_acquire: object = None
    last_departure: bool = False
    departure: bool = False
    next_cube_distance_mm: float = DEFAULT_NEXT_CUBE_SHIFT_MM
    next_adaptive: object = None

    def __post_init__(self):
        _number(self.next_cube_distance_mm,'next_cube_distance_mm',positive=True)
        if self.next_adaptive is not None:
            if not isinstance(self.next_adaptive, AdaptiveBlindProfile) or self.next_blind is None or self.next_acquire is None:
                raise ValueError('adaptive preview requires blind and acquire profiles')
            if self.next_adaptive.fallback_distance_mm > self.next_cube_distance_mm:
                raise ValueError('adaptive fallback cannot exceed the legacy expected distance')


@dataclass(frozen=True)
class CurveCalibration:
    points: tuple
    profile: object


@dataclass(frozen=True)
class TransitionConfig:
    firmware_full_lift_validated: bool = False
    firmware_id: str = ''
    pickups: object = field(default_factory=dict)
    curves: object = field(default_factory=dict)
    alignments: object = field(default_factory=dict)
    motion_planning_enabled: bool = False
    trial_run: bool = False
    switches: TransitionSwitches = field(default_factory=TransitionSwitches)
    navigation: object = None

    def __post_init__(self):
        if type(self.trial_run) is not bool:
            raise ValueError('trial_run must be boolean')
        if not isinstance(self.switches, TransitionSwitches):
            raise TypeError('switches must be TransitionSwitches')
        if type(self.motion_planning_enabled) is not bool:
            raise ValueError('motion_planning_enabled must be boolean')
        if type(self.firmware_full_lift_validated) is not bool:
            raise ValueError('firmware capability must be boolean')
        if self.pickups and not (self.firmware_full_lift_validated or self.trial_run):
            raise ValueError('pickup calibration requires verified firmware capability')
        object.__setattr__(self, 'pickups', MappingProxyType(dict(self.pickups)))
        object.__setattr__(self, 'curves', MappingProxyType(dict(self.curves)))
        object.__setattr__(self, 'alignments', MappingProxyType(dict(self.alignments)))

    def pickup(self, profile, method):
        return self.pickups.get(f'{profile}/{method}')

    def alignment(self, profile, color):
        return self.alignments.get(f'{profile}/{color}')

    @classmethod
    def load(cls, path):
        from .execution.pickup_motion import PickupMotionProfile
        from control.trajectory import TrajectoryProfile
        from .flows.factory import ROUTE_PROFILES
        from .flows.curves import CURVE_ROUTES
        document = json.loads(Path(path).read_text(encoding='utf-8'))
        _strict(document, {'version','firmware_full_lift_validated','firmware_id','pickups','curves','alignments','motion_planning_enabled'}, 'transition config')
        if document.get('version') != 1:
            raise ValueError('unsupported transition configuration version')
        verified = document.get('firmware_full_lift_validated', False)
        if type(verified) is not bool:
            raise ValueError('firmware_full_lift_validated must be boolean')
        firmware_id = document.get('firmware_id','')
        if verified and (not isinstance(firmware_id,str) or not firmware_id.strip()):
            raise ValueError('verified firmware requires its deployed version identifier')
        pickups = {}
        if any(not isinstance(document.get(key,{}),dict) for key in ('pickups','curves','alignments')):
            raise ValueError('pickups, curves and alignments must be named objects')
        for key, value in document.get('pickups',{}).items():
            _strict(value, {'validated','verified_on','notes','arm_restore_s',
                            'next_cube','last_departure','departure'}, key)
            _record(value)
            profile, method = key.split('/')
            allowed = ({'grap3'} if profile.startswith('ground-') else
                       {'grap1','grap2'} if profile.startswith('highland-') else set())
            if profile not in PROFILES or method not in allowed or not verified:
                raise ValueError(f'{key}: pickup calibration/firmware capability mismatch')
            restore = _number(value['arm_restore_s'], 'arm_restore_s')
            blind = acquire = adaptive = None
            next_distance = DEFAULT_NEXT_CUBE_SHIFT_MM
            if 'next_cube' in value:
                _strict(value['next_cube'], {'blind','acquire','expected_distance_mm','adaptive'}, 'next_cube')
                if method == 'grap2':
                    raise ValueError('purple pickup has no same-row next-cube transition')
                blind = _profile(BlindMotionProfile, value['next_cube']['blind'])
                acquire = _profile(PickupMotionProfile, value['next_cube']['acquire'])
                next_distance = _number(value['next_cube'].get('expected_distance_mm',DEFAULT_NEXT_CUBE_SHIFT_MM),
                                        'next_cube.expected_distance_mm',positive=True)
                if blind.direction != 1:
                    raise ValueError('current orange recipes search toward robot right')
                if 'adaptive' in value['next_cube']:
                    adaptive = _profile(AdaptiveBlindProfile, value['next_cube']['adaptive'])
            for flag in ('last_departure','departure'):
                if type(value.get(flag,False)) is not bool:
                    raise ValueError(f'{flag} must be boolean')
            if value.get('departure',False) and method != 'grap2':
                raise ValueError('departure flag applies to purple-to-orange routes')
            if value.get('last_departure',False) and method == 'grap2':
                raise ValueError('last_departure flag applies to final orange pickup')
            pickups[key] = PickupCalibration(restore, blind, acquire,
                                            value.get('last_departure',False),value.get('departure',False),next_distance,adaptive)
        curves = {}
        for key, value in document.get('curves',{}).items():
            _strict(value, {'validated','verified_on','notes','points','profile'}, key)
            _record(value)
            profile, route = key.split('/')
            if (profile not in PROFILES or route not in CURVE_ROUTES or
                    profile not in ROUTE_PROFILES[route]):
                raise ValueError(f'{key}: unknown route calibration profile')
            points=[]
            for point in value['points']:
                _strict(point, {'x_mm','y_mm','yaw_deg','dx_scale','dy_scale','dyaw_scale'},'curve point')
                data={k:float(point.get(k,0)) for k in ('x_mm','y_mm','yaw_deg','dx_scale','dy_scale','dyaw_scale')}
                if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in point.values()):
                    raise ValueError('curve coordinates must be finite numbers')
                points.append(MappingProxyType(data))
            if len(points)<2 or any(points[0].values()):
                raise ValueError('curve needs at least two points, starting at local (0,0,0)')
            if points[-1] != {'x_mm':0.0,'y_mm':0.0,'yaw_deg':0.0,
                              'dx_scale':1.0,'dy_scale':1.0,'dyaw_scale':1.0}:
                raise ValueError('last curve point must use the full calibrated endpoint with no offset')
            curves[key]=CurveCalibration(tuple(points),_profile(TrajectoryProfile,value['profile']))
        alignments = {}
        for key, value in document.get('alignments',{}).items():
            from .optimizations.fast_alignment import FastAlignmentProfile
            _strict(value, {'validated','verified_on','notes','profile'}, key)
            _record(value)
            profile, color = key.split('/')
            allowed = ({'orange'} if profile.startswith('ground-') else
                       {'orange','purple'} if profile.startswith('highland-') else set())
            if profile not in PROFILES or color not in allowed:
                raise ValueError(f'{key}: no pickup alignment calibration contract')
            alignments[key] = _profile(FastAlignmentProfile,value['profile'])
        for key, pickup in pickups.items():
            if pickup.next_adaptive is not None and f'{key.split("/")[0]}/orange' not in alignments:
                raise ValueError(f'{key}: adaptive preview requires fast orange alignment')
        return cls(verified,firmware_id,pickups,curves,alignments,
                   document.get('motion_planning_enabled',False))
