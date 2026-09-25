"""One-shot local robot control scaffold without an LLM or API request."""

from __future__ import annotations

import argparse
import json
import sys
import time

from .tools import RobotToolExecutor


def build_parser():
    parser = argparse.ArgumentParser(description="Direct Uniforest robot control")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--vision", action="store_true")
    parser.add_argument("--localization", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--state", action="store_true")
    group.add_argument("--cubes", action="store_true")
    group.add_argument("--tags", action="store_true")
    group.add_argument("--snapshot", nargs="+", choices=("cube", "tag"))
    group.add_argument("--move", nargs=3, metavar=("DIRECTION", "DISTANCE_MM", "SPEED_MM_S"))
    group.add_argument("--rotate", nargs=2, metavar=("ANGLE_DEG", "SPEED_DEG_S"))
    group.add_argument("--action", choices=("home", "hatch_open", "hatch_close", "grap1", "grap2", "grap3", "build"))
    group.add_argument("--grab-right-orange", action="store_true",
                       help="选择当前最右侧橙色方块，视觉对准后执行 Grap3")
    group.add_argument("--stop", action="store_true")
    return parser


def _print_result(result):
    print(json.dumps(result.value, ensure_ascii=False, default=str))


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    robot = None
    try:
        if not args.dry_run:
            from robot import Robot
            robot = Robot(port=args.port, baud=args.baud,
                          enable_vision=args.vision,
                          enable_localization=args.localization,
                          quiet_heartbeat=True)
            if not robot.connect():
                return 2
            robot.start(telem_rate=50)
            time.sleep(1.0 if (args.vision or args.localization) else 0.2)
        executor = RobotToolExecutor(robot, dry_run=args.dry_run)
        if args.state:
            result = executor.get_robot_state()
        elif args.cubes:
            result = executor.detect_cubes()
        elif args.tags:
            result = executor.detect_tags()
        elif args.snapshot:
            result = executor.get_camera_snapshot(args.snapshot, fresh=True)
        elif args.move:
            direction, distance, speed = args.move
            result = executor.move_chassis(direction, float(distance), float(speed))
        elif args.rotate:
            angle, speed = args.rotate
            result = executor.rotate_chassis(float(angle), float(speed))
        elif args.action:
            result = executor.execute_arm_action(args.action)
        elif args.grab_right_orange:
            if not args.vision:
                raise ValueError("--grab-right-orange 需要同时指定 --vision")
            from Strategy.competition import CompetitionProgram
            vision = robot.vision_result
            blocks = [block for block in (getattr(vision, "all_blocks", []) or [])
                      if str(getattr(block, "color_name", "")).lower() == "orange"]
            if not blocks:
                raise RuntimeError("当前没有可用的橙色方块识别结果")
            target = max(blocks, key=lambda block: float(getattr(block, "x", 0.0)))
            program = CompetitionProgram(robot)
            aligned = program._align_cube(
                target,
                color_name="orange",
                min_confidence=program.config.orange_min_confidence,
            )
            if not aligned:
                raise RuntimeError("右侧橙色方块视觉对准失败")
            robot.run_action("grap3")
            result = type("ActionResult", (), {
                "value": {"state": "complete", "action": "grap3",
                          "target_color": "orange",
                          "initial_x_mm": getattr(target, "x", None),
                          "initial_z_mm": getattr(target, "z", None)},
                "images": [],
            })()
        else:
            result = executor.emergency_stop()
        _print_result(result)
        return 0
    except Exception as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=False))
        return 1
    finally:
        if robot is not None:
            robot.stop()


if __name__ == "__main__":
    sys.exit(main())
