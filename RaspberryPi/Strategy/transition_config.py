"""Explicit calibration data for opt-in pickup and continuous-route transitions."""

from dataclasses import dataclass, field, fields
from datetime import date
import json
import math
from pathlib import Path
from types import MappingProxyType

from .settings import PROFILES
from .execution.blind import BlindMotionProfile


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

    def __post_init__(self):
        if type(self.firmware_full_lift_validated) is not bool:
            raise ValueError('firmware capability must be boolean')
        if self.pickups and not self.firmware_full_lift_validated:
            raise ValueError('pickup calibration requires verified firmware capability')
        object.__setattr__(self, 'pickups', MappingProxyType(dict(self.pickups)))
        object.__setattr__(self, 'curves', MappingProxyType(dict(self.curves)))

    def pickup(self, profile, method):
        return self.pickups.get(f'{profile}/{method}')

    @classmethod
    def load(cls, path):
        from .execution.pickup_motion import PickupMotionProfile
        from control.trajectory import TrajectoryProfile
        from .flows.factory import ROUTE_PROFILES
        from .flows.curves import CURVE_ROUTES
        document = json.loads(Path(path).read_text(encoding='utf-8'))
        _strict(document, {'version','firmware_full_lift_validated','firmware_id','pickups','curves'}, 'transition config')
        if document.get('version') != 1:
            raise ValueError('unsupported transition configuration version')
        verified = document.get('firmware_full_lift_validated', False)
        if type(verified) is not bool:
            raise ValueError('firmware_full_lift_validated must be boolean')
        firmware_id = document.get('firmware_id','')
        if verified and (not isinstance(firmware_id,str) or not firmware_id.strip()):
            raise ValueError('verified firmware requires its deployed version identifier')
        pickups = {}
        if not isinstance(document.get('pickups',{}),dict) or not isinstance(document.get('curves',{}),dict):
            raise ValueError('pickups and curves must be named objects')
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
            blind = acquire = None
            if 'next_cube' in value:
                _strict(value['next_cube'], {'blind','acquire'}, 'next_cube')
                if method == 'grap2':
                    raise ValueError('purple pickup has no same-row next-cube transition')
                blind = _profile(BlindMotionProfile, value['next_cube']['blind'])
                acquire = _profile(PickupMotionProfile, value['next_cube']['acquire'])
                if blind.direction != 1:
                    raise ValueError('current orange recipes search toward robot right')
            for flag in ('last_departure','departure'):
                if type(value.get(flag,False)) is not bool:
                    raise ValueError(f'{flag} must be boolean')
            if value.get('departure',False) and method != 'grap2':
                raise ValueError('departure flag applies to purple-to-orange routes')
            if value.get('last_departure',False) and method == 'grap2':
                raise ValueError('last_departure flag applies to final orange pickup')
            pickups[key] = PickupCalibration(restore, blind, acquire,
                                            value.get('last_departure',False),value.get('departure',False))
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
        return cls(verified,firmware_id,pickups,curves)
