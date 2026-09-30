"""Task4: wall-guided hatch unloading, entering at 180 degrees after Task2."""

from dataclasses import dataclass
from enum import Enum, auto

from .competition import FirstTaskConfig, TaskControl


class Task4State(Enum):
    STARTUP = auto()
    READY = auto()
    INITIAL_LATERAL = auto()
    LEFT_WALL_APPROACH = auto()
    POST_WALL_LATERAL = auto()
    FORWARD_WALL_APPROACH = auto()
    UNLOAD = auto()
    FINAL_LATERAL = auto()
    FINISHED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class Task4Config(FirstTaskConfig):
    initial_lateral_left_mm: float = 600.0
    lateral_speed_mm_s: float = 400.0
    long_route_speed_mm_s: float = 1000.0
    post_wall_lateral_right_mm: float = 0.0
    final_lateral_right_mm: float = 800.0


@dataclass(frozen=True)
class Task4_2Config(Task4Config):
    post_wall_lateral_right_mm: float = 300.0
    final_lateral_right_mm: float = 500.0


class Task4Program(TaskControl):
    TASK_LABEL = 'task4-1'

    def __init__(self, robot, config: Task4Config = Task4Config(), *, context=None):
        super().__init__(robot, config, context=context)
        self.state = Task4State.STARTUP

    def _preflight(self):
        self._wait_ready()
        if self.context is None:
            raise RuntimeError('Task4 requires a TaskContext with the Task2 exit heading')
        handoff = self.context.take_build_approach()
        # Task2 exits at 180 degrees. Preserve its zero just as Task3 does;
        # measuring the current yaw as zero here would reverse the final turn.
        self._heading_zero_deg = handoff.heading_zero_deg
        self.state = Task4State.READY
        print(f'[{self.TASK_LABEL}] Entry heading 180 deg; zero inherited from '
              f'{handoff.source_task}: {self._heading_zero_deg:+.1f} deg')

    def _run_mission(self):
        cfg = self.config
        self.state = Task4State.INITIAL_LATERAL
        self._checked_move('left', cfg.initial_lateral_left_mm, cfg.long_route_speed_mm_s,
                           accel_ms=cfg.long_distance_forward_accel_ms)
        self.state = Task4State.LEFT_WALL_APPROACH
        self._drive_until_wall(
            timeout_s=cfg.far_wall_timeout_s, speed_mm_s=cfg.far_wall_speed_mm_s,
            direction='left', context='Task4 left wall contact')
        if cfg.post_wall_lateral_right_mm > 0.0:
            self.state = Task4State.POST_WALL_LATERAL
            self._checked_move('right', cfg.post_wall_lateral_right_mm,
                               cfg.lateral_speed_mm_s)
        self.state = Task4State.FORWARD_WALL_APPROACH
        self._drive_until_wall(
            timeout_s=cfg.far_wall_timeout_s, speed_mm_s=cfg.far_wall_speed_mm_s,
            direction='forward', context='Task4 unload wall contact')
        self.state = Task4State.UNLOAD
        self._unload_cubes()
        self.state = Task4State.FINAL_LATERAL
        self._checked_move('right', cfg.final_lateral_right_mm, cfg.long_route_speed_mm_s,
                           accel_ms=cfg.long_distance_forward_accel_ms)

    def run(self) -> int:
        try:
            self._preflight()
            self._run_mission()
            self._check_active()
            self.state = Task4State.FINISHED
            return 0
        except Exception:
            self.state = Task4State.FAULT
            self.robot.transport.emergency_stop()
            raise


class Task4_2Program(Task4Program):
    TASK_LABEL = 'task4-2'

    def __init__(self, robot, config: Task4_2Config = Task4_2Config(), *, context=None):
        super().__init__(robot, config, context=context)


__all__ = ['Task4Config', 'Task4Program', 'Task4State', 'Task4_2Config', 'Task4_2Program']
