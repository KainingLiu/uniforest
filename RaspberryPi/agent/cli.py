"""Command-line natural-language interface for Uniforest."""

from __future__ import annotations

import argparse
import sys

from .client import RobotAgent
from .tools import RobotToolExecutor


def build_parser():
    parser = argparse.ArgumentParser(description="Uniforest natural-language robot agent")
    parser.add_argument("--port", default=None, help="STM32 串口；默认使用 Robot 的平台默认值")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--model", default=None,
                        help="模型；默认读取 API.md 中的 APIFUN gpt 配置")
    parser.add_argument("--api-config", default=None,
                        help="API.md 配置文件路径；默认自动查找项目 Docs/API.md")
    parser.add_argument("--profile", default=None,
                        help="API.md 配置分组；例如 APIFUN gpt 或 rightcode gpt")
    parser.add_argument("--dry-run", action="store_true", help="不连接硬件，只模拟执行有副作用的工具")
    parser.add_argument("--vision", action="store_true", help="启动方块摄像头视觉")
    parser.add_argument("--localization", action="store_true", help="启动 Tag 摄像头定位")
    parser.add_argument("--no-context-images", action="store_true",
                        help="不发送 Agent 固定场地/坐标系图片")
    parser.add_argument("--context-image", action="append", default=None,
                        help="额外发送一张固定参考图片，可重复指定")
    parser.add_argument("--quiet-tools", action="store_true",
                        help="不在终端显示工具调用痕迹")
    parser.add_argument("--debug", action="store_true")
    return parser


def decode_command_line(raw: bytes | str) -> str:
    """Decode terminal input from UTF-8 or common Chinese SSH locales."""
    if isinstance(raw, str):
        return raw
    for encoding in ("utf-8", "gb18030"):
        try:
            return raw.decode(encoding)
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", errors="replace")


def read_command() -> str:
    print("\n你 > ", end="", flush=True)
    stream = getattr(sys.stdin, "buffer", sys.stdin)
    raw = stream.readline()
    if not raw:
        raise EOFError
    return decode_command_line(raw).strip()


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    robot = None
    try:
        if args.dry_run:
            print("[Agent] dry-run 模式：不会连接 STM32 或执行真实动作")
        else:
            from robot import Robot
            robot = Robot(port=args.port, baud=args.baud,
                          enable_vision=args.vision,
                          enable_localization=args.localization,
                          quiet_heartbeat=True,
                          debug=args.debug)
            if not robot.connect():
                return 1
            robot.start(telem_rate=50)
            print("[Agent] 机器人已连接。输入自然语言，输入 exit 或 quit 退出。")
        executor = RobotToolExecutor(robot, dry_run=args.dry_run)
        context_images = [] if args.no_context_images else args.context_image
        agent = RobotAgent(executor, model=args.model,
                           context_image_paths=context_images,
                           api_config_path=args.api_config,
                           api_profile=args.profile,
                           show_tool_trace=not args.quiet_tools)
        while True:
            try:
                text = read_command()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not text:
                continue
            if text.lower() in {"exit", "quit", "q", "退出"}:
                break
            try:
                print("Agent > " + agent.ask(text))
            except KeyboardInterrupt:
                print("\nAgent > 已中断当前请求")
            except Exception as exc:
                print(f"Agent > 请求失败：{exc}")
        return 0
    finally:
        if robot is not None:
            robot.stop()


if __name__ == "__main__":
    sys.exit(main())
