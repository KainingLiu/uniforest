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

在 `RaspberryPi/` 环境安装依赖。默认配置读取项目根目录的 `Docs/API.md` 中 `APIFUN gpt` 段落，环境变量可以覆盖配置：

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

## 模块化策略入口（更新至 2026-09-28）

`run_strategy` 与命令行共用 `Strategy/runner.py`，任务枚举由同一任务库和策略包生成。
机械搭建入口支持 `--action build1`（一块，ID 6）、`--action build2`（两块，ID 5）及
`--action build3`（三块，ID 4；`build` 为兼容名）。Build1/2 需要配套同步上位机、烧录新固件后测试，
尚未实机验证；Task3 默认使用 Build3。Task1-0 补抓搜索耗尽时沿退场路线并行复检，后续 Task3
按舱内复检总数选择 Build1/2/3；0 块跳过建筑对准与搭建、继续路线；未知默认缺 1 块，执行 Build2。
此次复检不再触发补抓，Tag6/建筑视觉失效仍继续，通信、急停、遥测或机构故障仍中止；Task5 保持 Build3。

`task2-1/2` 只执行采集和 2750 mm 转场；Tag6、建筑对准、Build 与返程属于 `task3-1/2`。
`PlanA`（默认，旧 classic）在原 7 个任务后追加 task1-3 → task2-2 → task3-3，共 10 个任务；`set1/2` 执行 task0-1 和相应的一套任务；
`PlanB` 执行 task0-2 → task2-1 → task4-1 → task2-2 → task4-2 → task0-3 → task1-1 → task0-3 → task1-2 → task5，复用已有 Task2、Task1，末尾 Task5 执行两次建筑对准与 Build。
`PlanC` 执行 task0-1 → task1-1 → task0-3 → task1-2 → task2-1 → task3-1 → task2-2 → task3-2 → task1-3 → task2-2 → task3-3，沿用统一补抓规则；可指定 `run_strategy(selection="PlanC")`，默认仍为 PlanA。Task1-1 后先通过 Task0-3 转场，再执行 Task1-2；现场验证状态见 Strategy/README.md。
原 task0 命名为 task0-1；task0-2 以 1000 mm/s 前进 900 mm、右移 2700 mm，再转到 180°。
task0-3 从 180°进入，转到 0°，以 1000 mm/s 左移 2600 mm，再以 300 mm/s 向左顶墙；独立运行也需按 180°摆车。
task5 从 Task1-2 结束位置以 180°进入，执行两次开舱顶墙、建筑对准和 Build。开舱各等 200 ms、关舱各等 400 ms、各后退 250 mm；每次第三块释放后先以 400 mm/s 后退 100 mm，再执行原左移路线，与机构收尾并行，机构与路线均结束才完成。
Task5 起步左移 700 mm、两次右移 940 mm 恢复为 1000 mm/s；Build 后左移 840/440 mm 为 800 mm/s，以上加速均为 800 ms。后退 250/100 mm、中间右移 300 mm 保持 400 mm/s、300 ms 加速。
Task4 起步左移 600 mm、末尾右移 800/500 mm 保持 1000 mm/s、800 ms 加速；短退与中间右移 300 mm 为 400 mm/s、300 ms 加速。
Task2 两版的紫块后前进和橙块后横移按实际补偿距离选速：≥500 mm 为 1000 mm/s、800 ms；不足 500 mm 为 400 mm/s、300 ms，零横移跳过。协议未变，现场效果待确认。
紫块后前进的基础补偿量：task2-1 为 350 mm，task2-2 为 500 mm；实际距离为各自基础量减紫色阶段实测净右移量。橙色横移基础量仍为 550 mm。
Task1-3 复用 Task1，Tag6 后 400 mm/s 左移 500 mm，投放后同速右移 500 mm。
Task3-3 复用 Task3，Tag6 后 400 mm/s 左移 500 mm，Build 后 1000 mm/s 左移 3000 mm，再左顶墙；第三块释放后返程与机构收尾并行。
Task1 和 Task4 均取消末尾回零转向，以 180°结束；Task2 两版入口均为 180°，
先以 800 mm/s 后退 2500/2350 mm。独立 Task2 或 collect-build 也需按此入口朝向摆车。
`collect-build-1/2` 执行 task2 → task3，保留航向基准。
单独 task3/task4 必须确认处于 Task2 结束位置、初始航向 180°，并明确提供已标定的
`heading_zero_deg`；组合策略传 null。Task4 顶墙投放、右移后保持 180°结束，不重置航向零点。
停止指令会锁存取消并发送急停，阻止后台线程随后启动下一任务。
详细接口和扩展方法见 [策略架构](../Strategy/README.md)。
