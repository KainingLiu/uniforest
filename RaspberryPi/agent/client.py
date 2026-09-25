"""OpenAI Responses API loop for the command-line robot agent."""

from __future__ import annotations

import json
import os
import base64
from pathlib import Path
from typing import Any

from .config import load_api_config
from .tools import RobotToolExecutor, tool_definitions


SYSTEM_INSTRUCTIONS = """
你是 Uniforest 队的机器人控制 Agent。你的任务是用中文和队员自然交流，并通过工具可靠地观察、规划和控制机器人。回答简洁，但必须报告真实工具结果；不要凭空声称动作完成。

【比赛背景】
Uniforest 参加 RoboGame 2026 竞技组。机器人由 Raspberry Pi 5 上位机和 DJI RoboMaster A 板（STM32F427）下位机组成。树莓派负责视觉、比赛策略、路线编排、底盘位置外环和高层动作请求；STM32 负责四轮电机 1 kHz 速度环、舵机、双步进、吸盘、IMU、遥测和通信失联保护。你是高层任务 Agent，不是实时电机控制器。

比赛软件当前围绕两轮 Task1/Task2 工作，并有一个只在完整比赛入口开始时执行的 Task0。当前代码的完整默认顺序是：Task0 → Task1-R1 → Task2-R1 → Task1-R2 → Task2-R2。`round1` 是 Task0 → Task1-R1 → Task2-R1；`round2` 是 Task0 → Task1-R2 → Task2-R2；单独的 `task1`、`task2` 和带轮次的任务入口会跳过 Task0。

【场地和视觉】
当前软件场地模型约为宽 4.8 m、长 7.2 m，标签使用 AprilTag 36h11，标签编号为 1 到 6。场地坐标、标签坐标和安装偏移来自 `vision/opencv/field_map.json`，是控制用配置；Tag 相机当前标定文件仍标记为未完成实测标定，因此不要把模型坐标或距离当作绝对真值，必须通过 `detect_tags` 或摄像头工具确认。

机器人有两个固定角色的摄像头：`cube` 摄像头用于识别橙色、紫色方块和建筑轮廓；`tag` 摄像头用于识别 AprilTag 和场地定位。Tag3 主要用于 Task2 前段投放区对准，Tag6 用于最终投放/Build 区对准。方块和标签的连续跟踪、滤波、几何投影、PID 对准和目标丢失处理由本地视觉与策略代码完成。模型可以查看静态图像，但不能把单张图像猜测当成精确的毫米级控制量。

【机器人主要功能】
可用能力包括：读取 STM32 连接和遥测；查看两个摄像头；读取本地方块检测结果；读取 Tag 编号、距离、横向偏差和位姿；让麦克纳姆底盘按毫米移动或按角度旋转；执行已经标定好的 `home`、`hatch_open`、`hatch_close`、`grap1`、`grap2`、`grap3`、`build`；启动、查询和停止比赛策略。

机械动作含义：Grap1 用于 Task2 橙色方块，Grap2 用于 Task2 紫色方块，Grap3 用于 Task1 起始路线的橙色方块。用户只说“抓一个方块”时，必须先询问任务和颜色，绝不能默认 Grap1；启动区出发的 Task1 应使用 Grap3。当前 STM32 的 Build 仍是连续三次拾取/放置复合动作，单块放置需要新增下位机动作表和协议动作 ID。

【Task0 默认策略】
Task0 只在 `all`、`round1`、`round2` 中执行：等待遥测、方块视觉和 Tag 定位就绪后，以 750 mm/s、800 ms 加速向前移动 1200 mm。Task0 失败时不继续后续比赛。

【Task1 默认策略】
Task1 的目标是搜索、抓取橙色方块并运送到 Tag6 投放区。
1. 以约 200 mm/s 向前顶墙并重新校准航向。
2. 以约 300 mm/s 向右搜索橙色方块，累计搜索上限 1800 mm；发现目标后本地粗对准和末端微调。
3. 抓取前以约 150 mm/s 短压墙，执行 Grap3，默认目标为 3 个橙色方块。
4. 后退约 400 mm，转到启动零点顺时针 90°。
5. 以约 750 mm/s 前进，目标基准为 2800 mm，并扣除抓取阶段编码器测得的净右移量；随后转到 180°。
6. 用 Tag6 对准到约 425 mm、横向偏差接近零；按轮次横移后顶墙卸载。
7. 打开舱门，后退约 300 mm，关闭舱门；按轮次横移并回到约 0° 航向。
Task1-R1 在 Tag6 后右移约 100 mm、最后左移约 100 mm；Task1-R2 对应约为 400 mm 和 400 mm。

【Task2 默认策略】
Task2 的目标是处理紫色可选目标和橙色目标，最后在 Tag6 区域完成 Build。
1. 以约 750 mm/s 前进 2350 mm，转到约 -90°，用 Tag3 对准到约 250 mm。
2. R1 在 Tag3 后右移约 100 mm，R2 跳过该横移；两轮随后前进约 250 mm，并以约 200 mm/s 顶墙。
3. 以约 300 mm/s 向左搜索紫色方块；R1 搜索上限约 750 mm，R2 约 650 mm。找到后本地对准并执行 Grap2；找不到时跳过紫色抓取。
4. 后退约 100 mm，转回约 0°，以 `400 mm - 紫色阶段实测净右移量` 的补偿距离前进。
5. 先向左顶墙，再向前顶墙，重新校准航向。
6. 以约 300 mm/s 向右搜索橙色，累计搜索上限 1800 mm；逐个本地对准并执行 Grap1。紫色未抓取时，橙色目标数量按当前策略增加。
7. 后退约 500 mm，再按 `700 mm - 橙色阶段实测净右移量` 做横向补偿。
8. 转到 180°，以约 750 mm/s 前进约 2100 mm，用 Tag6 对准到约 425 mm。
9. 按轮次横移，进行建筑视觉对准并执行 Build。
10. Task2-R1 在 Build 后默认后退约 100 mm、顺时针转 180°、以约 750 mm/s 左移 2500 mm，最后左顶墙；Task2-R2 在 Build 完成后结束。
Task2-R1 在 Tag6 后右移约 100 mm，Task2-R2 约为 400 mm。当前 Task2 不执行 Build 后的 Tag1 对准流程。

【默认运行和决策规则】
- 用户说“运行完整比赛”或明确选择 `all` 时，调用 `run_strategy` 的 `all`。
- 用户说“第一轮”或“R1”时使用对应的 `*-r1` 或 `round1`；用户说“第二轮”或“R2”时使用对应的 `*-r2` 或 `round2`。只说“运行比赛”但没有说明范围时，先询问是完整比赛、第一轮还是第二轮。
- 用户说“按默认策略”时遵循上面的当前代码路线，不自行发明新路线；如果用户要求改变路线，先说明改变的动作和风险，再按用户明确指令执行。
- 运行策略后，策略在后台执行；向用户说明已启动，并用 `get_robot_state` 查询运行状态和结果。
- 策略运行期间不要调用独立的底盘移动或机械臂工具。用户要求停止时立即调用 `emergency_stop` 或 `cancel_current_action`。

【工具调用和安全规则】
涉及机器人状态、摄像头、识别、移动、机械臂和策略时，先调用工具，不要猜测。所有移动和机械动作都必须依赖工具返回值判断成功、超时、取消或失败。工具返回错误时停止后续动作并解释原因。

工具层的距离、速度、遥测新鲜度、动作互斥和策略互斥检查优先于你的判断。不要绕过这些检查，不要调用未定义工具，不要生成原始 PWM、CAN、UART、电机 RPM 或舵机角度。没有视觉数据时明确说“当前没有可用视觉数据”。急停始终可以调用，并且优先于其他计划。

上面列出的路线和参数是当前仓库的软件默认实现，不等同于官方规则全文，也不等同于现场实测精度。若用户询问规则得分、未在上下文中的场地区域或实测尺寸，明确说明信息不足，并通过工具或让队员确认，不要编造。
""".strip()


AGENT_DIR = Path(__file__).resolve().parent
DEFAULT_CONTEXT_IMAGE_PATHS = (
    AGENT_DIR / "assets" / "rules_field_3d.png",
    AGENT_DIR / "assets" / "rules_field_layout_tags.png",
    AGENT_DIR / "assets" / "plan_strategy_overview.png",
    AGENT_DIR / "assets" / "plan_strategy_main.png",
    AGENT_DIR / "assets" / "plan_strategy_sub.png",
)


def _image_data_url(path: Path) -> str:
    suffix = path.suffix.lower()
    mime = {".jpg": "image/jpeg", ".jpeg": "image/jpeg",
            ".webp": "image/webp", ".png": "image/png"}.get(suffix)
    if mime is None:
        raise ValueError(f"不支持的 Agent 图片格式: {path.suffix}")
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime};base64,{encoded}"


def build_initial_input(text: str, image_paths=None):
    """Build the first user message, optionally with fixed visual context."""
    paths = DEFAULT_CONTEXT_IMAGE_PATHS if image_paths is None else tuple(
        Path(path) for path in image_paths)
    content = [{
        "type": "input_text",
        "text": (
            "以下图片是 Uniforest Agent 的固定参考资料，只用于理解场地、"
            "机器人方向和视觉坐标。图片不是实时状态，精确数据必须通过工具获取。\n\n"
            + text
        ),
    }]
    for path in paths:
        if path.is_file():
            content.append({"type": "input_image", "image_url": _image_data_url(path)})
    return [{"role": "user", "content": content}]


def _get(item: Any, key: str, default=None):
    if isinstance(item, dict):
        return item.get(key, default)
    return getattr(item, key, default)


class RobotAgent:
    def __init__(self, executor: RobotToolExecutor, *, model: str | None = None,
                 context_image_paths=None, api_config_path=None,
                 api_profile=None,
                 show_tool_trace: bool = True):
        try:
            from openai import OpenAI
        except ImportError as exc:
            raise RuntimeError("缺少 openai 包，请执行 pip install -r requirements.txt") from exc
        config = load_api_config(api_config_path, profile=api_profile)
        api_key = config["api_key"]
        if not api_key:
            raise RuntimeError("未设置 OPENAI_API_KEY")
        client_kwargs = {"api_key": api_key}
        if config["base_url"]:
            client_kwargs["base_url"] = config["base_url"]
        self.client = OpenAI(**client_kwargs)
        self.executor = executor
        self.model = model or config["model"]
        self.api_profile = config["profile"]
        # Third-party API gateways used by this project do not expose
        # previous_response_id reliably. Keep the full input history and send
        # our prompt as a developer message, which compatible gateways preserve.
        self.stateless = True
        self._input_history = []
        self.previous_response_id = None
        self.show_tool_trace = show_tool_trace
        self.context_image_paths = (DEFAULT_CONTEXT_IMAGE_PATHS
                                    if context_image_paths is None else
                                    tuple(Path(path) for path in context_image_paths))

    def _request(self, input_items):
        if (not self.stateless and self.previous_response_id is None
                and isinstance(input_items, str)):
            input_items = build_initial_input(input_items, self.context_image_paths)
        params = {
            "model": self.model,
            "instructions": SYSTEM_INSTRUCTIONS,
            "tools": tool_definitions(),
            "input": input_items,
            "parallel_tool_calls": False,
            "store": True,
        }
        if self.previous_response_id and not self.stateless:
            params["previous_response_id"] = self.previous_response_id
        return self.client.responses.create(**params)

    def ask(self, text: str) -> str:
        if self.stateless:
            if self._input_history:
                request_input = list(self._input_history) + [{
                    "role": "user",
                    "content": [{"type": "input_text", "text": text}],
                }]
            else:
                request_input = [
                    {"role": "developer", "content": [
                        {"type": "input_text", "text": SYSTEM_INSTRUCTIONS}
                    ]},
                    *build_initial_input(text, self.context_image_paths),
                ]
            response = self._request(request_input)
            self._input_history = request_input
        else:
            response = self._request(text)
        for _ in range(24):
            self.previous_response_id = _get(response, "id")
            calls = [item for item in (_get(response, "output", []) or [])
                     if _get(item, "type") == "function_call"]
            if not calls:
                if self.stateless:
                    self._input_history.extend(_get(response, "output", []) or [])
                return _get(response, "output_text", "") or "（模型没有返回文字）"
            outputs = []
            for call in calls:
                name = _get(call, "name")
                call_id = _get(call, "call_id")
                raw_arguments = _get(call, "arguments", "{}")
                try:
                    arguments = json.loads(raw_arguments)
                    if self.show_tool_trace:
                        print(f"[Tool] {name} {json.dumps(arguments, ensure_ascii=False)}",
                              flush=True)
                    result = self.executor.execute(name, arguments)
                    if self.show_tool_trace:
                        summary = json.dumps(result.value, ensure_ascii=False, default=str)
                        if len(summary) > 500:
                            summary = summary[:500] + "..."
                        if result.images:
                            summary += f" images={len(result.images)}"
                        print(f"[Tool] {name} -> {summary}", flush=True)
                    outputs.append({"type": "function_call_output", "call_id": call_id,
                                    "output": result.response_output()})
                except Exception as exc:
                    if self.show_tool_trace:
                        print(f"[Tool] {name} -> ERROR: {exc}", flush=True)
                    outputs.append({"type": "function_call_output", "call_id": call_id,
                                    "output": [{"type": "input_text", "text": json.dumps({"error": str(exc)}, ensure_ascii=False)}]})
            if self.stateless:
                self._input_history.extend(_get(response, "output", []) or [])
                self._input_history.extend(outputs)
                response = self._request(list(self._input_history))
            else:
                response = self._request(outputs)
        raise RuntimeError("工具调用超过最大轮数，已停止本次请求")
