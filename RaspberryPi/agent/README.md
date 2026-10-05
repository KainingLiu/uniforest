# Uniforest 自然语言 Agent

第一版是命令行交互。Agent 在树莓派侧运行，通过 OpenAI Responses API 理解中文自然语言并调用本地机器人工具。首次请求会附带 `assets/` 中从队伍计划书和比赛规则手册截取的场地、标签和策略图片，作为固定视觉参考；实时摄像头图片通过工具按需返回。

如果由外部 Agent（例如上层对话控制器）直接决定动作，不需要再次调用 LLM，可以使用 `direct.py` 一次性调用本地工具：

```bash
python -m agent.direct --state
python -m agent.direct --vision --localization --cubes
python -m agent.direct --move forward 100 400
python -m agent.direct --action grap1
python -m agent.direct --stop
```

选择当前最右侧橙色方块，使用现有视觉闭环对准后执行 Grap3：

```bash
python -m agent.direct --vision --grab-right-orange
```

该脚手架不访问 OpenAI API，输出 JSON 结果；真实移动仍经过本地遥测、距离、速度和策略互斥检查。

## 安装

在 `RaspberryPi/` 安装依赖。配置查找顺序为项目私有 `Docs/API.md`、用户 `~/.config/uniforest/API.md`；可用 `--api-config` 显式指定。默认分组为 `APIFUN gpt`，环境变量优先。私有配置不提交仓库：

```bash
python -m pip install -r requirements.txt
export OPENAI_API_KEY='可选：覆盖 API.md 中的 key'
export OPENAI_BASE_URL='可选：覆盖 API.md 中的 base url'
```

Windows PowerShell：

```powershell
$env:OPENAI_API_KEY = '你的 API Key'
```

## 运行

只测试对话和工具编排，不连接硬件：

```bash
python -m agent.cli --dry-run
```

树莓派真实运行：

```bash
python -m agent.cli --vision --localization
```

使用 `Docs/API.md` 中的 RightAPI 配置：

```bash
python -m agent.cli --profile rightcode --vision --localization
```

RightAPI 会自动使用其 Codex Responses 地址 `/codex/v1`。不指定 `--profile` 时仍默认使用 `APIFUN gpt`。

## 本机中转

如果树莓派可以连接开发机，但不能直接访问公网，在开发机的 `RaspberryPi/` 目录启动 APIFUN 中转：

```bash
python -m agent.relay --profile APIFUN --host 192.168.137.1 --port 8765
```

然后在树莓派上将 API 地址指向开发机热点网关：

```bash
export OPENAI_BASE_URL='http://192.168.137.1:8765'
python -m agent.cli --profile APIFUN --vision --localization
```

中转服务只记录请求路径、状态码和耗时，不记录 API Key 或请求正文。

工具调用默认会显示在终端，例如 `[Tool] detect_tags {...}` 和工具返回摘要。使用 `--quiet-tools` 可以关闭。

Agent 模式会隐藏后台 STM32 心跳的 `PONG` 日志，避免打断 `你 >` 输入提示；心跳通信本身仍持续运行。

不发送固定参考图片：

```bash
python -m agent.cli --no-context-images
```

也可以追加自定义参考图片：

```bash
python -m agent.cli --context-image /path/to/your/reference.png
```

可尝试：

```text
现在机器人状态怎么样？
查看两个摄像头，告诉我识别到了什么
向前移动 100 毫米
执行 grap1
运行 task2-1
运行 collect-build-1 策略（采集后搭建）
停止当前动作
```

移动、机械臂和比赛策略工具仍由树莓派本地安全检查和现有控制器执行。模型不会直接生成电机或串口指令。

## 策略入口与边界

`run_strategy` 与 CLI 共用 `Strategy/runner.py`，任务枚举从同一注册表生成。
当前支持 PlanA/B/C、set1/2、collect-build-1/2 和共用 Task；默认仍为 PlanA。
唯一完整路线和补抓说明见 [策略文档](../Strategy/README.md)，复制命令见 [TEST_COMMANDS.md](../TEST_COMMANDS.md)。

- 独立 Task3/Task4 必须提供已知 `heading_zero_deg`，确认 Task2 出口位置和 180°朝向；不能猜测。
- Task1-0 / Task2-0 的抓取数量可在 Python `TaskStep` 中配置为 1/2/3；当前 Agent 工具没有数量覆盖参数，不可把用户指定 1/2 个替换为默认 3 个。
- Task1 用 Grap3 抓橙块，Task2 橙块用 Grap1、紫块用 Grap2；未明确任务/颜色时应澄清。
- Build1/2/3 在下位机执行，`build` 兼容 Build3；动作结果以工具返回为准。
- 比赛视觉按既有预算降级，通信、急停、陈旧遥测和机构异常终止；停止指令优先。
- 语音转写应合并同一条指令，避免分段或重复文本导致重复动作。

本轮只整理说明，不修改 Agent 行为。配置项和参数以 `config.py`、`cli.py`、`direct.py` 与 `tools.py` 为准。
