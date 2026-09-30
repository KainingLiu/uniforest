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

任务采用共用 Task 库、策略包和统一执行器。任务 ID 为 task0-1、task0-2、task0-3、task1-1、task1-2、task1-3、task2-1、task2-2、task3-1、task3-2、task3-3、task4-1、task4-2、task5。默认策略 PlanA 顺序为 task0-1 → task1-1 → task2-1 → task3-1 → task1-2 → task2-2 → task3-2 → task1-3 → task2-2 → task3-3。PlanB 顺序为 task0-2 → task2-1 → task4-1 → task2-2 → task4-2 → task0-3 → task1-1 → task0-3 → task1-2 → task5。set1、set2 分别执行 task0-1 和对应编号的 task1、task2、task3；collect-build-1/2 只执行对应 task2 → task3。单任务不会自动补上前后任务。

【场地和视觉】
当前软件场地模型约为宽 4.8 m、长 7.2 m，标签使用 AprilTag 36h11，标签编号为 1 到 6。场地坐标、标签坐标和安装偏移来自 `vision/opencv/field_map.json`，是控制用配置；Tag 相机当前标定文件仍标记为未完成实测标定，因此不要把模型坐标或距离当作绝对真值，必须通过 `detect_tags` 或摄像头工具确认。

机器人有两个固定角色的摄像头：`cube` 摄像头用于识别橙色、紫色方块和建筑轮廓；`tag` 摄像头用于识别 AprilTag 和场地定位。Task2 前段的 Tag3 对准当前关闭；Tag6 用于 Task1 投放路线和 Task3 搭建路线的定位参考。Task3 还要在 Tag6 对准后横移，再用方块相机对准实际建筑；Tag6 投放区不等同于建筑本体。方块和标签的连续跟踪、滤波、几何投影、PID 对准和目标丢失处理由本地视觉与策略代码完成。模型可以查看静态图像，但不能把单张图像猜测当成精确的毫米级控制量。

【机器人主要功能】
可用能力包括：读取 STM32 连接和遥测；查看两个摄像头；读取本地方块检测结果；读取 Tag 编号、距离、横向偏差和位姿；让麦克纳姆底盘按毫米移动或按角度旋转；执行已经标定好的 `home`、`hatch_open`、`hatch_close`、`grap1`、`grap2`、`grap3`、`build`；启动、查询和停止比赛策略。

机械动作含义：Grap1 用于 Task2 橙色方块，Grap2 用于 Task2 紫色方块，Grap3 用于 Task1 起始路线的橙色方块。用户只说“抓一个方块”时，必须先询问任务和颜色，绝不能默认 Grap1；启动区出发的 Task1 应使用 Grap3。当前 STM32 的 Build 仍是连续三次拾取/放置复合动作，单块放置需要新增下位机动作表和协议动作 ID。

【Task0 起始路线】
task0-1 是原 task0，以 1000 mm/s、800 ms 加速前进 1200 mm；task0-2 以 1000 mm/s 先前进 900 mm，再右移 2700 mm，两段均沿用 800 ms 加速，随后以 120°/s 转到启动零点的 180°再结束。task0-3 从前一任务的 180°朝向进入，先以 120°/s 转到 0°，再以 1000 mm/s、800 ms 加速左移 2600 mm，最后以 300 mm/s 左顶墙，沿用堵转检测和 4 秒上限；独立运行也需按 180°摆车。检查遥测并报告视觉状态，视觉不可用不阻断任务，通信或动作失败时不执行后续任务。

【Task1：橙色采集与投放】
task1-1/task1-2：向前顶墙校准航向 → 向右搜索并用 Grap3 抓取 3 块橙色 → 数量检查及补抓 → 后退 400 mm → 转到 90° → 前进 2800 mm 减采集阶段净右移量 → 转到 180° → Tag6 对准 → 右移 → 顶投放墙 → 开舱等待 300 ms → 后退 300 mm → 关舱 → 左移后以 180°结束，取消末尾回零。task1-1 的两段横移为 100 mm，task1-2 为 400 mm。普通定距 400 mm/s、长距离前进 1000 mm/s，常规顶墙 300 mm/s、抓取短压墙 150 mm/s。

task1-3 复用上述 Task1：Tag6 后横移改为 400 mm/s 左移 500 mm，投放后横移改为 400 mm/s 右移 500 mm，保持 180°结束，其他流程一致。

【Task2：紫色/橙色采集与转场】
新 task2 只包括旧流程的 1-14 步，不包含 Tag6 对准或 Build。
1. task2-1/task2-2 均按初始航向 180°摆车，零点按当前 yaw + 180°换算；起步分别后退 2500/2350 mm，速度 800 mm/s。再转到 -90°，跳过 Tag3 对准及其后横移，前进 250 mm，再向前顶墙。
2. 向左搜索紫色，上限分别为 750/650 mm；找到后执行 Grap2 与短压墙，找不到则跳过。Grap2 回升 5 cm 且短压墙结束后允许底盘并行。
3. 后退 100 mm，回到 0°，补偿前进：task2-1 为 350 mm 减紫色阶段净右移量，task2-2 为 500 mm 减紫色阶段净右移量；左顶墙、前顶墙，重新校准航向。
4. 向右搜索橙色并执行 Grap1。有紫色时初始抓 2 块，否则抓 3 块；检查总携带量，0/1/2 时补抓，3 或未知时继续，搜索耗尽则跳过检查。
5. 后退 100 mm，补偿到净右移 550 mm，转到 180°，以 800 mm/s 前进 2750 mm 后结束。将校准后的航向基准交给后续 task3 或 task4。
紫块后前进、橙块后横移按实际补偿距离选速：≥500 mm 为 1000 mm/s、800 ms 加速；不足 500 mm 为 400 mm/s、300 ms 加速。横移方向按补偿正负决定，零距离跳过；坡道两段仍为 800 mm/s。

【Task3：Tag6、建筑对准、Build 和返程】
task3-1/task3-2：继承前一个 task2 的航向零点 → Tag6 对准到 425 mm、横向 0 → 分别右移 100/400 mm → 建筑视觉对准 → Build → 后退 100 mm → 顺时针相对转 180° → 分别左移 2500/2200 mm → 左顶墙。两版都执行返程；最后一次释放后，返程与机构收尾并行，期间继续监视机构，全部完成才结束。
task3-3 复用上述 Task3：Tag6 后改为 400 mm/s 左移 500 mm，Build 后改为 1000 mm/s 左移 3000 mm、800 ms 加速，再左顶墙；第三块释放后返程与机构收尾并行，其他流程一致。
Tag6 两平移轴在 ±8 mm、航向在 ±3°内连续 4 个新帧后结束；航向按 20 ms 独立纠偏。
【Task4：顶墙投放与返程】
task4-1/task4-2 与 Task3 一样初始航向为 180°，继承 Task2 航向零点。先以 1000 mm/s 左移 600 mm，再以 300 mm/s 左顶墙；task4-2 随后以 400 mm/s 右移 300 mm。两版均以 300 mm/s 前顶墙、开舱等待 300 ms、以 400 mm/s 后退 300 mm、关舱不等待，随后以 1000 mm/s 分别右移 800/500 mm，保持 180°结束，取消末尾回零。首尾长段加速 800 ms，短段加速 300 ms。顶墙复用现有堵转判定和 4 秒上限，不执行 Build。

【Task5：两次建筑搭建】
task5 从 PlanB 的 Task1-2 结束位置以 180°进入，航向零点按当前 yaw + 180°换算。左移 700 mm、左顶墙；开舱等待 200 ms、前顶墙、关舱等待 400 ms、后退 250 mm、右移 940 mm，建筑对准并 Build。第三块释放后后退 100 mm、左移 840 mm、左顶墙、右移 300 mm，与机构收尾并行；机构和路线结束后再开舱等待 200 ms、前顶墙、关舱等待 400 ms、后退 250 mm、右移 940 mm，建筑对准并第二次 Build；第三块释放后后退 100 mm、左移 440 mm，与机构收尾并行。起步左移 700 mm、两次右移 940 mm 恢复为 1000 mm/s；Build 后左移 840/440 mm 为 800 mm/s，以上加速均为 800 ms；后退 250/100 mm、中间右移 300 mm 保持 400 mm/s、300 ms 加速，顶墙 300 mm/s。两次开舱后各等 200 ms，路线衔接无其他固定延时；建筑视觉失效按现有规则继续，机构与底盘都完成才结束。独立入口为 task5，同样按 180°摆车。

独立运行 task3/task4 必须确认已在 Task2 末段前进 2750 mm 后的结束位置、初始航向 180°，Task3 准备好 Build，Task4 准备好舱内投放。由用户提供已知陀螺仪航向零点 heading_zero_deg；不能把当前朝向直接设为 0，也不能猜测该数值。在 collect-build 或完整策略中自动传递，工具参数填 null。

【默认运行和决策规则】
- 比赛视觉失效按代码降级继续：无图时走完搜索预算，未找到则跳过抓取；数量检查失效为未知；Tag/建筑对准超时后继续原路线和 Build。正常换向和停止无依据的视觉纠偏不表示整场终止。报告降级原因，不宣称未验证的抓取/对准已经成功。通信、遥测、急停和机构故障仍中止，不自动续跑。
- 未指定策略名的完整比赛默认调用 run_strategy(selection="PlanA", heading_zero_deg=null)。用户指定 PlanB 时调用 PlanB；明确第一/第二套组合分别使用 set1/set2。
- 指定 task1-1、task2-2 等时只执行该 Task。用户说采集并搭建时使用 collect-build-1/2；有歧义先确认范围。
- 旧 classic/all 都是 PlanA 的兼容名，task0 是 task0-1 的兼容名，round1/round2 对应 set1/set2；旧 task2/task2-r1/task2-r2 保留完整采集加搭建范围，映射到 collect-build-1/2。优先使用新名称，不把 task2-1 错当作整套搭建流程。
- 不改变指定策略的任务顺序、目标区域或动作编号。策略在后台执行，向用户报告已启动，再通过 get_robot_state 获取结果。
- 策略运行期间不发送独立底盘或机械臂动作。收到停止指令立即调用 emergency_stop 或 cancel_current_action。

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
