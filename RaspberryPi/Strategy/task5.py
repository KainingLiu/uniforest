"""Task5: two wall loading and building cycles after the PlanB Task1 routes."""

from dataclasses import dataclass
from enum import Enum, auto

from .common import wrap_angle
from .task3 import Task3Config, Task3Program


class Task5State(Enum):
    STARTUP = auto()
    READY = auto()
    INITIAL_LEFT = auto()
    LEFT_WALL_APPROACH = auto()
    HATCH_OPEN = auto()
    FORWARD_WALL_APPROACH = auto()
    HATCH_CLOSE = auto()
    BACKWARD = auto()
    BUILD_APPROACH = auto()
    BUILDING_ALIGN = auto()
    BUILD = auto()
    POST_BUILD_REVERSE = auto()
    FIRST_BUILD_LEFT = auto()
    SECOND_LOAD_APPROACH = auto()
    FINAL_LEFT = auto()
    FINISHED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class Task5Config(Task3Config):
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
    # Task5 goes directly from its own 940 mm leg to building alignment.
    post_tag6_lateral_right_mm: float = 0.0


class Task5Program(Task3Program):
    """Reuse building vision and Build without the Task3 Tag6/return route."""

    TASK_LABEL = 'task5'

    def __init__(self, robot, config: Task5Config = Task5Config(), *, context=None):
        super().__init__(robot, config, context=context)
        self.state = Task5State.STARTUP

    def _preflight(self):
        self._wait_ready()
        self._heading_zero_deg = wrap_angle(
            self.robot.telem.yaw_deg + self.config.initial_heading_cw_deg)
        self.state = Task5State.READY
        print(f'[{self.TASK_LABEL}] Entry heading 180 deg; '
              f'heading zero={self._heading_zero_deg:+.1f} deg')

    def _left_wall(self):
        self._check_active()
        self.state = Task5State.LEFT_WALL_APPROACH
        self._drive_until_wall(
            timeout_s=self.config.far_wall_timeout_s,
            speed_mm_s=self.config.far_wall_speed_mm_s,
            direction='left', context='Task5 left wall contact')

    def _load_and_approach_building(self):
        cfg = self.config
        self._check_active()
        self.state = Task5State.HATCH_OPEN
        self.robot.actions.hatch_open(settle_ms=cfg.hatch_open_settle_ms)
        self._check_active()
        self.state = Task5State.FORWARD_WALL_APPROACH
        self._drive_until_wall(
            timeout_s=cfg.far_wall_timeout_s, speed_mm_s=cfg.far_wall_speed_mm_s,
            direction='forward', context='Task5 open-hatch forward wall contact')
        self._check_active()
        self.state = Task5State.HATCH_CLOSE
        self.robot.actions.hatch_close(settle_ms=cfg.hatch_close_settle_ms)
        self._check_active()
        self.state = Task5State.BACKWARD
        self._checked_move('backward', cfg.load_reverse_mm, cfg.route_speed_mm_s)
        self.state = Task5State.BUILD_APPROACH
        self._checked_move('right', cfg.building_approach_right_mm, cfg.long_route_speed_mm_s,
                           accel_ms=cfg.long_distance_forward_accel_ms)

    def _build_with_route(self, route):
        self._check_active()
        self.state = Task5State.BUILDING_ALIGN
        self.robot.reset_vision_filter()
        self._align_building_or_continue()
        self._check_active()
        self.state = Task5State.BUILD
        self.robot.actions.build(chassis_followup=self._chassis_followup(route))
        self._check_active()

    def _first_build_route(self):
        cfg = self.config
        self.state = Task5State.POST_BUILD_REVERSE
        self._checked_move('backward', cfg.post_build_reverse_mm,
                           cfg.post_build_reverse_speed_mm_s)
        self.state = Task5State.FIRST_BUILD_LEFT
        self._checked_move('left', cfg.first_build_left_mm, cfg.build_followup_speed_mm_s,
                           accel_ms=cfg.long_distance_forward_accel_ms)
        self._left_wall()
        self.state = Task5State.SECOND_LOAD_APPROACH
        self._checked_move('right', cfg.second_load_right_mm, cfg.route_speed_mm_s)

    def _final_build_route(self):
        cfg = self.config
        self.state = Task5State.POST_BUILD_REVERSE
        self._checked_move('backward', cfg.post_build_reverse_mm,
                           cfg.post_build_reverse_speed_mm_s)
        self.state = Task5State.FINAL_LEFT
        self._checked_move('left', cfg.final_left_mm, cfg.build_followup_speed_mm_s,
                           accel_ms=cfg.long_distance_forward_accel_ms)

    def _run_mission(self):
        cfg = self.config
        self.state = Task5State.INITIAL_LEFT
        self._checked_move('left', cfg.initial_left_mm, cfg.long_route_speed_mm_s,
                           accel_ms=cfg.long_distance_forward_accel_ms)
        self._left_wall()
        self._load_and_approach_building()
        self._build_with_route(self._first_build_route)
        # Actions.build returns only after both its cleanup and monitored
        # chassis route finish. Keep hatch commands out of that overlap.
        self._load_and_approach_building()
        self._build_with_route(self._final_build_route)

    def run(self) -> int:
        try:
            self._preflight()
            self._run_mission()
            self._check_active()
            self.state = Task5State.FINISHED
            return 0
        except Exception:
            self.state = Task5State.FAULT
            self.robot.transport.emergency_stop()
            raise


__all__ = ['Task5Config', 'Task5Program', 'Task5State']
