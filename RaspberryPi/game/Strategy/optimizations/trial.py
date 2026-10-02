"""Explicit, one-run trial settings. These are NOT field-validation records.

Requested on 2026-10-02 for a full PlanA with all pickup handoffs and fast
capture, retaining classic navigation. Legacy limits supply the initial values;
new clearance/uncertainty assumptions remain visible in the diagnostic snapshot.
"""
from ..settings import PROFILES
from ..transition_config import TransitionConfig,PickupCalibration
from ..execution.blind import BlindMotionProfile
from ..execution.pickup_motion import PickupMotionProfile
from .fast_alignment import FastAlignmentProfile
from .adaptive_blind import AdaptiveBlindProfile


def trial_configuration(*, adaptive_blind=False):
    pickups={}; alignments={}
    for name in ('ground-1','ground-2','ground-3','highland-1','highland-2'):
        cfg=PROFILES[name]
        adaptive = AdaptiveBlindProfile(trial_enabled=True) if adaptive_blind else None
        shared=dict(acceleration_mm_s2=cfg.align_accel_mm_s2,braking_mm_s2=cfg.align_accel_mm_s2,
                    braking_margin_mm=20.,telemetry_timeout_s=.2,frame_timeout_s=.3,
                    tick_s=.02,max_command_delay_s=.05,validated=False,trial_enabled=True)
        blind=BlindMotionProfile(direction=1,cruise_speed_mm_s=2*cfg.align_start_speed_mm_s,
                                 # Longer no-next-target search plus the
                                 # existing braking reserve, only in this trial.
                                 max_distance_mm=(adaptive.unseen_search_mm+shared['braking_margin_mm']
                                                  if adaptive is not None else 100.),
                                 max_duration_s=(8. if adaptive is not None else 4.),**shared)
        acquire=PickupMotionProfile(max_distance_mm=cfg.search_max_distance_mm,
                    max_duration_s=cfg.align_timeout_s,settled_speed_mm_s=20.,**shared)
        method='grap3' if name.startswith('ground') else 'grap1'
        pickups[f'{name}/{method}']=PickupCalibration(cfg.post_grab_settle_s,blind,acquire,last_departure=True,
            next_adaptive=adaptive)
        if name.startswith('highland'):
            pickups[f'{name}/grap2']=PickupCalibration(cfg.post_grab_settle_s,departure=True)
        for color in (('orange',) if name.startswith('ground') else ('orange','purple')):
            alignments[f'{name}/{color}']=FastAlignmentProfile(
                # User's capture window is +/-10 mm. Zero disables the extra
                # fixed 2 mm reserve; it does not claim a perfect camera.
                capture_tolerance_mm=10.,mechanical_tolerance_mm=10.,position_uncertainty_mm=0.,
                max_speed_mm_s=min(cfg.search_speed_mm_s,cfg.align_max_speed_mm_s),
                min_correction_speed_mm_s=min(cfg.align_start_speed_mm_s,cfg.search_speed_mm_s,cfg.align_max_speed_mm_s),
                acceleration_mm_s2=cfg.align_accel_mm_s2,braking_mm_s2=cfg.align_accel_mm_s2,
                settled_speed_mm_s=20.,max_duration_s=cfg.align_timeout_s,
                max_travel_mm=cfg.search_max_distance_mm if color=='orange' else cfg.purple_search_max_distance_mm,
                frame_timeout_s=.3,telemetry_timeout_s=.2,tick_s=.02,max_command_delay_s=.05,
                validated=False,trial_enabled=True)
    return TransitionConfig(pickups=pickups,alignments=alignments,motion_planning_enabled=False,trial_run=True)
