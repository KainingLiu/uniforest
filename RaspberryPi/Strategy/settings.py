"""Immutable control and route calibration profiles for functional actions.

Numerical values are preserved from the current competition implementation.
Profiles select calibration and destinations; they do not execute missions.
"""

from dataclasses import dataclass
from control.chassis import (
    LONG_DISTANCE_FORWARD_ACCEL_MS,
    LONG_DISTANCE_MOVE_SPEED_MM_S,
    NORMAL_DISTANCE_MOVE_SPEED_MM_S,
    NORMAL_DISTANCE_MOVE_ACCEL_MS,
)
from .vision_targets import (
    TASK1_ORANGE as GROUND_ORANGE,
    TASK2_ORANGE as HIGHLAND_ORANGE,
    TASK2_PURPLE as HIGHLAND_PURPLE,
)

# Retain tuned physical stopping positions after the tag-camera FOV correction.
TAG_FOV_RETUNE_SCALE = 0.300549527


@dataclass(frozen=True)
class GroundCollectionConfig:
    target_cube_count: int = 3
    far_wall_speed_mm_s: float = 300.0
    far_wall_timeout_s: float = 4.0
    near_wall_speed_mm_s: float = 150.0
    near_wall_timeout_s: float = 1.0
    wall_timeout_is_success: bool = True
    wall_settle_s: float = 0.0
    stall_startup_grace_s: float = 0.5
    stall_confirm_s: float = 0.3
    stall_dropout_s: float = 0.08
    pre_grab_stall_startup_grace_s: float = 0.1
    pre_grab_stall_confirm_s: float = 0.15
    pre_grab_wall_settle_s: float = 0.0
    telemetry_stale_s: float = 0.3
    stall_speed_rpm: int = 80
    stall_current_raw: int = 2500
    # Default/lateral-wall confirmation remains the original three-wheel
    # criterion. Forward contact uses the two rear wheel indices explicitly.
    stall_motor_count: int = 3
    search_speed_mm_s: float = 300.0
    search_max_distance_mm: float = 1800.0
    search_control_period_s: float = 0.02
    vision_observe_s: float = 0.35
    vision_stale_s: float = 0.5
    orange_min_confidence: float = 25.0
    # Keep the initially selected cube when several orange cubes are visible.
    orange_search_lock_x_jump_mm: float = 80.0
    orange_search_lock_z_jump_mm: float = 100.0
    orange_search_lock_lost_frames: int = 3
    orange_search_confirm_frames: int = 2
    orange_edge_confirm_frames: int = 3
    orange_edge_speed_mm_s: float = 200.0
    orange_edge_max_distance_mm: float = 1000.0
    orange_edge_timeout_s: float = 6.0
    orange_edge_stop_s: float = 0.10
    orange_edge_origin_margin_mm: float = 10.0
    orange_edge_retry_spacing_mm: float = 100.0
    # Absolute camera-X windows are maintained in vision_targets.py.
    align_min_x_mm: float = GROUND_ORANGE.align_min_x_mm
    align_max_x_mm: float = GROUND_ORANGE.align_max_x_mm
    align_target_x_mm: float = GROUND_ORANGE.target_x_mm
    align_confirm_frames: int = 2
    orange_coarse_align_timeout_s: float = 5.0
    orange_fine_align_timeout_s: float = 0.5
    orange_fine_timeout_retry_count: int = 2
    orange_fine_min_x_mm: float = GROUND_ORANGE.fine_min_x_mm
    orange_fine_max_x_mm: float = GROUND_ORANGE.fine_max_x_mm
    align_kp: float = 1.5
    # Integral action is disabled for lateral visual pickup: camera latency
    # and the minimum breakout speed make accumulated error overshoot-prone.
    align_ki: float = 0.0
    align_kd: float = 0.0
    align_integral_limit: float = 300.0
    align_min_speed_mm_s: float = 100.0
    align_creep_min_speed_mm_s: float = 100.0
    align_start_speed_mm_s: float = 80.0
    align_max_speed_mm_s: float = 250.0
    align_fast_speed_mm_s: float = 500.0
    # Shorter approach ramp; retain the near-target breakout speed.
    align_slowdown_start_mm: float = 150.0
    align_creep_start_mm: float = 30.0
    align_accel_mm_s2: float = 800.0
    # One-frame control feedback avoids commanding motion from an old median
    # while preserving target confirmation in the search tracker.
    align_filter_frames: int = 1
    align_track_max_x_jump_mm: float = 45.0
    align_track_max_z_jump_mm: float = 80.0
    # Zero keeps the legacy nearest-candidate behavior. Highland collection overrides this
    # for its adjacent orange-cube pickup area.
    align_track_ambiguity_margin_mm: float = 12.0
    align_window_hysteresis_mm: float = 5.0
    align_window_hold_s: float = 0.20
    align_control_period_s: float = 0.03
    align_lost_timeout_s: float = 0.5
    align_timeout_s: float = 10.0
    post_grab_settle_s: float = 0.0
    delivery_reverse_mm: float = 400.0
    delivery_reverse_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    delivery_turn_deg: float = 90.0
    delivery_turn_speed_deg_s: float = 120.0
    delivery_turn_heading_hold_ms: int = 0
    delivery_forward_base_mm: float = 2800.0
    delivery_forward_speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
    delivery_tag_id: int = 6
    delivery_tag_distance_mm: float = 425.0
    delivery_tag_distance_tolerance_mm: float = 8.0
    delivery_tag_lateral_tolerance_mm: float = 8.0
    delivery_tag_distance_deadband_mm: float = 5.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_lateral_deadband_mm: float = 5.0 * TAG_FOV_RETUNE_SCALE
    delivery_heading_target_cw_deg: float = 180.0
    delivery_heading_tolerance_deg: float = 3.0
    delivery_heading_deadband_deg: float = 0.5
    delivery_tag_confirm_frames: int = 4
    delivery_tag_fine_align_timeout_s: float = 1.0
    delivery_tag_fine_gain_scale: float = 1.5
    delivery_tag_vision_stale_s: float = 0.3
    delivery_tag_lost_timeout_s: float = 1.0
    delivery_tag_align_timeout_s: float = 12.0
    delivery_tag_control_period_s: float = 0.05
    delivery_tag_translation_median_frames: int = 3
    delivery_tag_max_distance_jump_mm: float = 250.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_max_lateral_jump_mm: float = 250.0 * TAG_FOV_RETUNE_SCALE
    delivery_tag_distance_kp: float = 0.8 / TAG_FOV_RETUNE_SCALE
    delivery_tag_distance_ki: float = 0.0
    delivery_tag_distance_kd: float = 0.0
    delivery_tag_lateral_kp: float = 1.2 / TAG_FOV_RETUNE_SCALE
    delivery_tag_lateral_ki: float = 0.0
    delivery_tag_lateral_kd: float = 0.0
    delivery_heading_kp: float = 1.5
    delivery_heading_ki: float = 0.02
    delivery_heading_kd: float = 0.03
    # Tag6 gyro hold uses angular-speed units, independent of visual updates.
    tag6_heading_kp: float = 6.0
    tag6_heading_ki: float = 0.0
    tag6_heading_kd: float = 0.0
    tag6_heading_control_period_s: float = 0.02
    delivery_tag_linear_integral_limit: float = (
        500.0 * TAG_FOV_RETUNE_SCALE)
    delivery_heading_integral_limit: float = 100.0
    delivery_tag_max_forward_mm_s: float = 250.0
    delivery_tag_max_lateral_mm_s: float = 200.0
    delivery_tag_fast_forward_mm_s: float = 350.0
    delivery_tag_fast_lateral_mm_s: float = 280.0
    delivery_tag_slowdown_distance_mm: float = 100.0
    delivery_tag_slowdown_lateral_mm: float = 80.0
    delivery_tag_creep_distance_mm: float = 25.0
    delivery_tag_creep_lateral_mm: float = 20.0
    delivery_heading_max_yaw_deg_s: float = 45.0
    delivery_tag_min_linear_mm_s: float = 100.0
    delivery_heading_min_yaw_deg_s: float = 8.0
    delivery_tag_linear_accel_mm_s2: float = 300.0
    delivery_heading_yaw_accel_deg_s2: float = 90.0
    post_tag_lateral_right_mm: float = 100.0
    post_tag_lateral_direction: str = 'right'
    post_tag_lateral_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    unload_reverse_mm: float = 300.0
    unload_reverse_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    pre_final_turn_lateral_left_mm: float = 100.0
    pre_final_turn_lateral_direction: str = 'left'
    pre_final_turn_lateral_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    delivery_linear_accel_ms: int = NORMAL_DISTANCE_MOVE_ACCEL_MS
    long_distance_forward_accel_ms: int = LONG_DISTANCE_FORWARD_ACCEL_MS


@dataclass(frozen=True)
class GroundCollection2Config(GroundCollectionConfig):
    post_tag_lateral_right_mm: float = 400.0
    pre_final_turn_lateral_left_mm: float = 400.0


@dataclass(frozen=True)
class GroundCollection3Config(GroundCollectionConfig):
    post_tag_lateral_right_mm: float = 500.0
    post_tag_lateral_direction: str = 'left'
    pre_final_turn_lateral_left_mm: float = 500.0
    pre_final_turn_lateral_direction: str = 'right'


@dataclass(frozen=True)
class HighlandCollectionConfig(GroundCollectionConfig):
    initial_heading_cw_deg: float = 180.0
    initial_distance_mm: float = 2500.0
    initial_speed_mm_s: float = 800.0  # Highland collection ramp approach, independent of cruise speed.
    delivery_heading_target_cw_deg: float = -90.0
    delivery_tag_id: int = 3
    # Temporary route: skip Tag3 and its post-alignment lateral move together.
    tag3_alignment_enabled: bool = False
    delivery_tag_distance_mm: float = 250.0
    delivery_tag_distance_tolerance_mm: float = 10.0
    delivery_tag_lateral_tolerance_mm: float = 10.0
    delivery_heading_tolerance_deg: float = 3.0
    delivery_tag_fine_gain_scale: float = 1.5
    # Highland collection tags use the wide-angle, uncalibrated tag camera.  Keep the
    # chassis stopped briefly on a missed frame and reacquire a fresh pose.
    delivery_tag_vision_stale_s: float = 0.7
    delivery_tag_lost_timeout_s: float = 2.0
    post_tag_lateral_mm: float = 100.0
    post_tag_lateral_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    wall_premove_mm: float = 250.0
    wall_premove_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    purple_min_confidence: float = 25.0
    purple_align_timeout_s: float = 5.0
    purple_search_max_distance_mm: float = 750.0
    align_min_x_mm: float = HIGHLAND_PURPLE.align_min_x_mm
    align_max_x_mm: float = HIGHLAND_PURPLE.align_max_x_mm
    # Highland collection purple-cube calibration: stable centered sample measured X=-0.5 mm.
    align_target_x_mm: float = HIGHLAND_PURPLE.target_x_mm
    # Highland collection orange-cube calibration: choose the candidate nearest camera
    # center, measured at X=-0.1 mm. Preserve the previous relative window.
    orange_fine_min_x_mm: float = HIGHLAND_ORANGE.fine_min_x_mm
    orange_fine_max_x_mm: float = HIGHLAND_ORANGE.fine_max_x_mm
    post_grab_reverse_mm: float = 100.0
    post_grab_reverse_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    post_grab_heading_target_cw_deg: float = 0.0
    post_grab_forward_base_mm: float = 350.0
    post_grab_forward_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    compensation_fast_distance_mm: float = 500.0
    compensation_fast_speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
    left_wall_approach_enabled: bool = True
    orange_target_count: int = 2
    orange_target_count_without_purple: int = 3
    orange_align_min_x_mm: float = HIGHLAND_ORANGE.align_min_x_mm
    orange_align_max_x_mm: float = HIGHLAND_ORANGE.align_max_x_mm
    orange_align_target_x_mm: float = HIGHLAND_ORANGE.target_x_mm
    orange_track_ambiguity_margin_mm: float = 18.0
    post_orange_reverse_mm: float = 100.0
    post_orange_reverse_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    post_orange_lateral_base_mm: float = 550.0
    post_orange_lateral_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    final_turn_target_cw_deg: float = 180.0
    build_route_distance_mm: float = 2750.0
    build_route_speed_mm_s: float = 800.0  # Highland collection second ramp section.
    # Preserve the existing optional Tag3 profile if that route is re-enabled.
    delivery_tag_fast_forward_mm_s: float = 260.0
    delivery_tag_fast_lateral_mm_s: float = 200.0
    delivery_tag_min_linear_mm_s: float = 100.0
    delivery_tag_slowdown_distance_mm: float = 140.0
    delivery_tag_slowdown_lateral_mm: float = 100.0
    delivery_tag_creep_distance_mm: float = 35.0
    delivery_tag_creep_lateral_mm: float = 25.0


@dataclass(frozen=True)
class HighlandCollection2Config(HighlandCollectionConfig):
    initial_distance_mm: float = 2350.0
    purple_search_max_distance_mm: float = 650.0
    post_grab_forward_base_mm: float = 500.0
    post_tag_lateral_mm: float = 0.0


@dataclass(frozen=True)
class BuildingConfig(GroundCollectionConfig):
    delivery_tag_vision_stale_s: float = 0.7
    delivery_tag_lost_timeout_s: float = 2.0
    build_tag_id: int = 6
    build_tag_distance_mm: float = GroundCollectionConfig().delivery_tag_distance_mm
    build_tag_heading_target_cw_deg: float = 180.0
    build_tag_distance_tolerance_mm: float = 8.0
    build_tag_lateral_tolerance_mm: float = 8.0
    build_tag_heading_tolerance_deg: float = (
        GroundCollectionConfig().delivery_heading_tolerance_deg)
    build_tag_fine_gain_scale: float = (
        GroundCollectionConfig().delivery_tag_fine_gain_scale)
    build_tag_vision_stale_s: float = 0.7
    build_tag_lost_timeout_s: float = 2.0
    # Tag6 is approached after a long straight run. Slow the far-field
    # profile and enter deceleration earlier without changing final tolerances.
    delivery_tag_fast_forward_mm_s: float = 260.0
    delivery_tag_fast_lateral_mm_s: float = 200.0
    delivery_tag_min_linear_mm_s: float = 100.0
    delivery_tag_slowdown_distance_mm: float = 140.0
    delivery_tag_slowdown_lateral_mm: float = 100.0
    delivery_tag_creep_distance_mm: float = 35.0
    delivery_tag_creep_lateral_mm: float = 25.0
    post_tag6_lateral_right_mm: float = 100.0
    post_tag6_lateral_direction: str = 'right'
    post_tag6_lateral_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    building_target_x_mm: float = 0.0
    # Z target is the horizontal robot-to-building distance at which Build
    # places correctly.  With the cube camera a lower top edge means nearer.
    building_target_z_mm: float = 75.0
    # Inverse-row Z model: z = building_z_scale / top_row.  The scale comes
    # from one measured pose where the building top edge sat at image row 82.4
    # while the robot was 132.8 mm away, so the scale is 132.8*82.4.  The Z
    # target above then corresponds to the top edge around row scale/75 ~= 146,
    # i.e. the upper-middle part of the frame.
    building_z_scale_mm_px: float = 132.8 * 82.4
    # Lateral pixels convert with the same calibrated focal length and optical
    # center as camera_calib.json for the cube camera.
    building_reference_fx_px: float = 331.93
    building_reference_cx_px: float = 320.0
    # Forward/back creep: inside this remaining Z error the approach slows to
    # the creep band so the 100 mm/s static-friction floor cannot overshoot
    # the +/-6 mm acceptance window and set up a forward/back limit cycle.
    building_z_creep_start_mm: float = 30.0
    building_z_creep_speed_mm_s: float = 90.0
    building_z_creep_min_mm_s: float = 60.0
    # Building contours vary with occlusion and camera pitch. Keep the
    # geometric gate permissive; position and multi-frame confirmation still
    # reject isolated orange cube candidates.
    building_min_confidence: float = 35.0
    building_min_height_width_ratio: float = 0.35
    building_max_height_width_ratio: float = 2.20
    building_x_tolerance_mm: float = 3.0
    building_z_tolerance_mm: float = 6.0
    building_heading_tolerance_deg: float = 2.4
    building_x_deadband_mm: float = 1.0
    building_z_deadband_mm: float = 2.0
    building_heading_deadband_deg: float = 0.5
    building_confirm_frames: int = 3
    building_median_frames: int = 5
    building_align_timeout_s: float = 7.0
    building_lost_timeout_s: float = 4.0
    building_vision_stale_s: float = 0.7
    building_track_max_x_jump_mm: float = 50.0
    building_track_max_z_jump_mm: float = 60.0
    building_track_lock_frames: int = 2
    building_control_period_s: float = 0.05
    building_forward_kp: float = 1.5
    building_forward_ki: float = 0.01
    building_forward_kd: float = 0.02
    building_lateral_kp: float = 1.8
    building_lateral_ki: float = 0.01
    building_lateral_kd: float = 0.02
    building_heading_kp: float = 1.5
    building_heading_ki: float = 0.02
    building_heading_kd: float = 0.03
    building_linear_integral_limit: float = 150.0
    building_heading_integral_limit: float = 100.0
    building_max_forward_mm_s: float = 250.0
    building_max_lateral_mm_s: float = 250.0
    building_max_yaw_deg_s: float = 30.0
    # Static-friction floor for starting motion. Required for pure lateral
    # commands (mecanum rollers) and far forward/back approach; once the
    # chassis is rolling the building_z_* creep band lets forward speed drop
    # near the target instead of slamming every frame and overshooting.
    building_min_linear_mm_s: float = 100.0
    building_far_linear_mm_s: float = 150.0
    building_min_yaw_deg_s: float = 6.0
    # Soften command changes; the accel limiter still allows a smooth stop as
    # an axis enters its acceptance window.
    building_linear_accel_mm_s2: float = 1000.0
    building_yaw_accel_deg_s2: float = 60.0
    post_build_reverse_mm: float = 100.0
    post_build_reverse_speed_mm_s: float = NORMAL_DISTANCE_MOVE_SPEED_MM_S
    post_build_turn_cw_deg: float = 180.0
    post_build_route_distance_mm: float = 2500.0
    post_build_route_speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S


@dataclass(frozen=True)
class Building2Config(BuildingConfig):
    post_tag6_lateral_right_mm: float = 400.0
    post_build_route_distance_mm: float = 2200.0


@dataclass(frozen=True)
class Building3Config(BuildingConfig):
    post_tag6_lateral_right_mm: float = 500.0
    post_tag6_lateral_direction: str = 'left'
    post_build_route_distance_mm: float = 3000.0


@dataclass(frozen=True)
class WallUnloadConfig(GroundCollectionConfig):
    initial_lateral_left_mm: float = 600.0
    lateral_speed_mm_s: float = 400.0
    long_route_speed_mm_s: float = 1000.0
    post_wall_lateral_right_mm: float = 0.0
    final_lateral_right_mm: float = 800.0


@dataclass(frozen=True)
class WallUnload2Config(WallUnloadConfig):
    post_wall_lateral_right_mm: float = 300.0
    final_lateral_right_mm: float = 500.0


@dataclass(frozen=True)
class StagedBuildingConfig(BuildingConfig):
    initial_heading_cw_deg: float = 180.0
    initial_left_mm: float = 700.0
    route_speed_mm_s: float = 400.0
    long_route_speed_mm_s: float = 1000.0
    build_followup_speed_mm_s: float = 800.0
    hatch_open_settle_ms: int = 200
    hatch_close_settle_ms: int = 400
    load_reverse_mm: float = 250.0
    building_approach_right_mm: float = 940.0
    first_build_left_mm: float = 840.0
    second_load_right_mm: float = 300.0
    final_left_mm: float = 440.0
    # Staged building goes directly from its own 940 mm leg to building alignment.
    post_tag6_lateral_right_mm: float = 0.0


@dataclass(frozen=True)
class DepartureConfig:
    distance_mm: float = 1200.0
    speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
    hold_ms: int = 0
    accel_ms: int = LONG_DISTANCE_FORWARD_ACCEL_MS
    telemetry_wait_s: float = 2.0


@dataclass(frozen=True)
class Departure2Config(DepartureConfig):
    distance_mm: float = 900.0
    lateral_right_mm: float = 2700.0
    final_heading_cw_deg: float = 180.0
    turn_speed_deg_s: float = 120.0


@dataclass(frozen=True)
class ReturnToOrangeConfig(GroundCollectionConfig):
    initial_heading_cw_deg: float = 180.0
    target_heading_cw_deg: float = 0.0
    lateral_left_mm: float = 2600.0
    lateral_speed_mm_s: float = LONG_DISTANCE_MOVE_SPEED_MM_S
    far_wall_speed_mm_s: float = 300.0


# Common controllers use this profile's shared motion/vision fields. Round
# profiles override only the calibrated differences required by each route.
CommonControlConfig = GroundCollectionConfig
Departure1Config = DepartureConfig

PROFILES = {
    'ground-1': GroundCollectionConfig(),
    'ground-2': GroundCollection2Config(),
    'ground-3': GroundCollection3Config(),
    'highland-1': HighlandCollectionConfig(),
    'highland-2': HighlandCollection2Config(),
    'building-1': BuildingConfig(),
    'building-2': Building2Config(),
    'building-3': Building3Config(),
    'unload-1': WallUnloadConfig(),
    'unload-2': WallUnload2Config(),
    'staged-building': StagedBuildingConfig(),
    'depart-a': DepartureConfig(),
    'depart-b': Departure2Config(),
    'return-orange': ReturnToOrangeConfig(),
}
