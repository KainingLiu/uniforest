# 共用 Task 库、策略包与统一执行器

任务拆分与 2200 mm 新返程于 2026-09-26 完成。09-27 统一橙色搜索、回找、比赛视觉降级和 OpenCV 目录；现有策略命名为 PlanA，新增 PlanB、Task0-2 和 Task4 两个变体。09-28 新增 Task0-3，PlanB 追加两次转场与 Task1 两版，再衔接 Task5 两次搭建。部署和现场验证状态见上级 CHANGELOG.md。

## 代码职责

| 位置 | 职责 |
| --- | --- |
| `tasks/__init__.py` | 共用任务注册表 `TASK_LIBRARY`、单次调用 `TaskStep`、任务定义 `TaskDefinition` |
| `plans/plan_a.py`、`plans/plan_b.py` | 策略包中的任务组合，引用共用任务 ID，不复制任务代码；classic.py 保留旧导入 |
| `plans/__init__.py` | 策略类型、参数及交接依赖的预检查 |
| `runner.py` | 唯一任务执行器、结果处理、互斥、取消及采集/诊断上下文 |
| `context.py` | 一次运行的上下文、航向交接、通信与急停状态检查 |
| `task0.py` 至 `task5.py` | 各任务及其参数；保留原模块路径方便既有调用方迁移 |
| `competition.py` | 共用 `TaskControl` 控制过程和原 Task1 实现；Task0-3/Task2/Task3/Task4/Task5 继承控制过程，Task1/Task4 共用开舱、后退、关舱过程 |

动作、视觉、通信仍分别调用 `Robot`、`control/`、`vision/`、`protocol/`。
不新增依赖，不改 A 板动作表。新架构不自动规划转场或判断场地物理位置。

## 任务边界

| ID | 范围 | 起点与输出 |
| --- | --- | --- |
| `task0-1` | 原 Task0：1000 mm/s 前进 1200 mm | 调用者摆好 PlanA 初始位置 |
| `task0-2` | 1000 mm/s 前进 900 mm、右移 2700 mm，随后转到 180°；两段加速均 800 ms | 调用者摆好 PlanB 初始位置；以 180°结束接 Task2-1 |
| `task0-3` | 转到 0°，1000 mm/s 左移 2600 mm，再以 300 mm/s 左顶墙 | 从 Task4/Task1 结束位置以 180°进入，0°结束接 Task1 |
| `task1-1` / `task1-2` | 采集、运输、投放；取消末尾转到 0° | Tag6 后右移/投放后左移分别 100/400 mm；以 180°结束 |
| `task1-3` | 复用 Task1 采集、运输、投放 | Tag6 后 400 mm/s 左移 500 mm；投放后同速右移 500 mm，以 180°结束 |
| `task2-1` / `task2-2` | 起步后退 2500/2350 mm，再采集、补抓、补偿、前进 2750 mm | 入口航向 180°；输出 `BuildApproach`，停在 Tag6 对准前 |
| `task3-1` / `task3-2` | 原步骤 15-17：Tag6、建筑对准、Build、返程 | 需要 Task2 的入口姿态和航向；返程左移 2500/2200 mm，均以左顶墙收尾 |
| `task3-3` | 复用 Task3 对准、Build 与并行返程 | Tag6 后 400 mm/s 左移 500 mm；Build 后 1000 mm/s 左移 3000 mm，再左顶墙 |
| `task4-1` / `task4-2` | 顶墙、开舱投放、退离、右移后结束 | 在 Task2 结束位置以 180°进入和结束；消费同一航向交接 |
| `task5` | 两次开舱顶墙取块、建筑对准及 Build，第三块释放后并行底盘路线 | 从 Task1-2 结束位置以 180°进入；两次机构及路线都完成后结束 |

Task3 的 Tag6 后右移保留原差异：`task3-1=100 mm`，`task3-2=400 mm`。
Task3-2 新增完整返程，2200 mm 为用户指定参数，待现场确认。
Task1-3、Task3-3 的方向与距离为 2026-09-30 用户指定参数，复用相同动作与视觉控制，现场待确认。
Grap2 与后续底盘并行仍在 Task2 内；Build 与返程并行在 Task3 内，Task5 复用同一机制执行两次搭建。
Task 返回成功前需等本 Task 的机械动作和配套路段全部结束，不把活动机械动作留给下一 Task。

## 现有策略包

| 名称 | 顺序 |
| --- | --- |
| `PlanA`（默认） | task0-1 → task1-1 → task2-1 → task3-1 → task1-2 → task2-2 → task3-2 → task1-3 → task2-2 → task3-3 |
| `PlanB` | task0-2 → task2-1 → task4-1 → task2-2 → task4-2 → task0-3 → task1-1 → task0-3 → task1-2 → task5 |
| `set1` | task0-1 → task1-1 → task2-1 → task3-1 |
| `set2` | task0-1 → task1-2 → task2-2 → task3-2 |
| `collect-build-1` | task2-1 → task3-1 |
| `collect-build-2` | task2-2 → task3-2 |

在 RaspberryPi 目录使用：

```bash
python main.py --list-tasks
python main.py --strategy PlanA --show-plan
python main.py --strategy PlanB --show-plan
python main.py --task task0-3 --show-plan
python main.py --task task5 --show-plan
python main.py --strategy collect-build-2 --show-plan
python main.py --task task2-1 --show-plan
```

以上均只打印信息，不连接硬件。去掉 `--show-plan` 会运行实际动作。
`--strategy` 和 `--task` 互斥。`task2_main.py --variant 1/2` 只运行新 Task2，不包含 Build。

## Task0-3 转场（2026-09-28 用户指定，待现场验证）

1. 以 180°进入，按 `零点 = 当前 yaw + 180°` 换算，以 120°/s 转到 0°，无额外保持等待。
2. 以 1000 mm/s 向左平移 2600 mm，加速 800 ms。
3. 以 300 mm/s 向左顶墙，沿用共用堵转检测和 4 秒上限，达到上限按原规则继续。

PlanB 在 Task4-2、Task1-1 后各复用一次 Task0-3，再分别执行原有 Task1-1、Task1-2。
测试入口为 `--task task0-3` 或 `--strategy PlanB`，桌面继续只保留 PlanA、PlanB。
独立 Task0-3 同样要求从对应位置以 180°摆车。视觉缺失不阻断，通信异常、急停或遥测陈旧立即中止。
协议未变；尚未执行实机动作，方向、顶墙和后续 Task1 衔接需现场确认。

## Task4 路线（2026-09-27 用户指定，待现场验证）

两版初始及结束航向均为 180°，方向按机器人本体描述。首尾长段为 1000 mm/s、800 ms 加速，短段为 400 mm/s、300 ms 加速，
顶墙速度 300 mm/s，复用堵转检测和 4 秒上限，达到上限后沿原规则继续。

| 顺序 | task4-1 | task4-2 |
| --- | --- | --- |
| 1 | 1000 mm/s 左移 600 mm | 同左 |
| 2 | 300 mm/s 左顶墙 | 同左 |
| 3 | 无额外横移 | 400 mm/s 右移 300 mm |
| 4 | 300 mm/s 前顶墙 | 同左 |
| 5 | 开舱等待 300 ms | 同左 |
| 6 | 400 mm/s 后退 300 mm | 同左 |
| 7 | 关舱，不等待 | 同左 |
| 8 | 1000 mm/s 右移 800 mm | 1000 mm/s 右移 500 mm |

Task4 不重置当前 180° 为 0°，不新增视觉对准或 Build。PlanB 原样复用 Task2 两个变体。
取消第 9 步转向，第 8 步右移完成后直接结束。PlanB 第一次由 Task0-2 末尾转到 180°衔接，
第二次由 Task4-1 保持 180°衔接；PlanA 两次 Task1 也保持 180°交给 Task2。
Task2 入口按 `零点 = 当前 yaw + 180°` 换算，再执行后退；独立 Task2 或 collect-build 也按 180°摆车。
Task0-2 新增转向使用启动零点计算目标 180°，速度 120°/s，无额外保持等待。
测试入口为 `--strategy PlanB` 或独立 `--task task4-1/2`（后者填写实际 ID 并提供航向零点）。
上述参数为软件设定，路线衔接、顶墙判定与投放效果尚未经过现场确认。

## Task5 路线（2026-09-29 更新，待现场验证）

入口为 PlanB 中 Task1-2 的结束位置、航向 180°，零点按当前 yaw + 180°换算。
复用 Task3 建筑视觉对准及现有 Build，直接对准建筑，无额外 Tag6 步骤或横移。
恢复 Task5 指定路段提速：起步左移 700 mm、两次右移 940 mm 为 1000 mm/s；Build 后左移 840/440 mm 为 800 mm/s，以上加速均为 800 ms。
后退 250/100 mm、中间右移 300 mm 保持 400 mm/s、300 ms 加速。
所有定距路线保持 0 ms；顶墙 300 mm/s、最多 4 s。

| 顺序 | 流程 |
| --- | --- |
| 1 | 1000 mm/s 左移 700 mm，向左顶墙 |
| 2 | 开舱等待 200 ms，向前顶墙，关舱等待 400 ms，后退 250 mm |
| 3 | 1000 mm/s 右移 940 mm，建筑对准，第一次 Build |
| 4 | 第三块释放后 400 mm/s 后退 100 mm，再以 800 mm/s 左移 840 mm，向左顶墙，再以 400 mm/s 右移 300 mm；这些底盘动作与机构收尾并行 |
| 5 | 第一次 Build 与上述路段均结束后，开舱等待 200 ms，向前顶墙，关舱等待 400 ms，后退 250 mm |
| 6 | 1000 mm/s 右移 940 mm，建筑对准，第二次 Build |
| 7 | 第三块释放后 400 mm/s 后退 100 mm，再以 800 mm/s 左移 440 mm，与机构收尾并行；两者均结束后 Task5 完成 |

Task5 两次开舱后各等 200 ms，两次关舱后各等 400 ms，其他路线衔接不加延时；Build 内部取块与抬臂时序沿用原动作。
第一次并行段仅包含底盘动作，再次开舱在其机构收尾完成后执行，防止舵机复位覆盖舱门命令。
沿用视觉失效继续的规则，建筑失效仍执行 Build；通信、急停、遥测或机构故障中止。
现有 ACTION_CHASSIS_READY 在第三块释放后通知底盘，协议未变，无需为 Task5 新增下位机动作。
独立运行：`python main.py --task task5`，按上述入口摆车。桌面仍仅 PlanA / PlanB。

## Task2 补偿段速度（2026-09-28）

紫块后前进基础量按变体分别设定：Task2-1 为 `350 - 紫色阶段净右移量`，Task2-2 为
`500 - 紫色阶段净右移量`。两版橙块后横移距离均为
`abs(550 - 橙色阶段净右移量)`，横移方向沿用原补偿计算。
实际距离 ≥500 mm 时用 1000 mm/s、800 ms 加速；不足 500 mm 时用 400 mm/s、300 ms 加速，
零横移仍跳过。起步后退 2500/2350 mm、末段前进 2750 mm 保持 800 mm/s、800 ms 加速。
上述提速复用提前减速，协议未变；软件参数和无硬件验证不代表现场已确认。

## 共用顶墙判定（2026-09-28）

Task0-3、Task1/2/3/4/5 以及 Agent 顶墙入口共用此规则，抓取时的短压墙也适用。
单轮要求 `abs(speed_rpm) <= 80` 且 `abs(torque_current) >= 2500`；
前向要求两个后轮均满足，其他方向要求四轮中至少三轮满足。
常规顶墙经过 0.5 s 起步保护后累计 0.3 s 有效接触；短压墙分别为 0.1 s / 0.15 s。
最长 80 ms 的短暂不满足暂停累计，超出则清零重来；不满足期间不计时、不判成功。
仅使用新遥测帧。4/1 s 超时上限与原有降级继续规则保持，通信、遥测或急停故障仍中止。
本次将轮速上限从 35 放宽到 80 rpm，电流与轮数门槛保持；参数待现场确认，协议未变。

## 任务交接与单独运行 Task3/Task4

Task2 输出 `BuildApproach(heading_zero_deg, source_task)`：保存最近一次顶墙校准的
陀螺仪航向零点，不用当前朝向替代它，也不把预计数量当作实际携带量。
Task3 或 Task4 消费该交接一次。策略预检查要求前一步提供该交接；插入其他普通任务后，
旧交接失效。输入 Task2 失败、取消、通信中断或重启时，下一 Task 不执行。

在同一策略中执行 Task2 → Task3/Task4 自动传递。执行器结束时关闭上下文，不用于断点续跑。
单独运行 Task3/Task4 时，调用者必须摆到 Task2 末段前进 2750 mm 后的结束位置，初始航向 180°；
Task3 准备好 Build，Task4 准备好舱内投放。通过 `--heading-zero-deg` 提供已知的原任务陀螺仪零点。
它不是 Tag6 目标 180°，也不是默认把当前 yaw 当成零点；缺少值或非有限值在连接前拒绝。
航向零点可来自前一次 Task2 的实际校准记录，机器人重启/IMU 基准变化后旧值不能沿用。

## 新增策略与任务

新策略只需实例化 `StrategyPlan`。例如在策略文件中复用两项任务并局部覆盖参数：

```python
from . import StrategyPlan
from ..tasks import TaskStep

MY_PLAN = StrategyPlan('my-plan', (
    TaskStep('task2-1'),
    TaskStep('task3-1', {'post_build_route_distance_mm': 2200.0}),
))
```

将新策略导入 `plans/__init__.py` 并加入 `PLANS`（在计算 `PLAN_IDS` 前）；命令行和 Agent
选择项随该注册表生成。该例只展示复用方式，不代表新增路线已经现场验证。
新策略可按需要使用任意现有 Task；调用者需保证物理起止位置衔接，执行器只验证软件交接依赖。

新增 Task 的约定：

1. 新建任务模块，配置使用不可变 dataclass；构造函数接受 `robot, config, *, context=None`。
2. `run()` 完成本任务并返回 `0` 或 `TaskResult`；失败抛出异常或返回非零结果。
   `TaskResult` 状态不是 SUCCESS 时，即使 code 为 0 也按失败处理。
3. 通过现有机器人接口执行，循环及动作开始前调用 `context.check_active()`。
   若继承 `TaskControl`，使用 `_check_active()`；机械动作沿用 Actions 的取消检查。
4. 在 `TASK_LIBRARY` 注册 `TaskDefinition(程序类, 配置类)`，然后在策略包引用新 ID。
   如任务确实把机器人带到同一个搭建入口，可声明 `provides='build_approach'`，
   成功后显式发布实际航向基准；不能只声明输出却不生成数据。

`TaskStep.parameters` 仅覆盖该次调用的配置，每次构造新 Task，避免搜索预算、航向或
视觉锁定状态在重复运行时泄漏。未知 Task、未知参数和缺失交接在整套执行前检查。
统一执行器只管理任务运行；串口、相机、心跳的启动和最终清理仍由外层 `main.py`/Agent 管理。
同一 Robot 只允许一个策略执行器运行。失败/急停关闭本次上下文，无自动重试或自动续跑。

视觉不可用属于可继续的比赛降级：方块搜索耗尽后跳过抓取、数量检查失效记为未知，
Tag/建筑对准到现有超时后继续既定路线和 Build。相机不可用不阻断任务预检；所有硬件
保护保留。只捕获明确的视觉目标丢失/超时类型，不用宽泛异常捕获掩盖通信或机构故障。
橙色搜索由所有 Task 共用同一实现，视觉左边缘信号、回找状态与补抓预算成套同步。
09-30 左缘截断提示从正常可抓目标的形状/角点过滤中分离，窄条及全宽整排可发出
回找提示；起点限位阻止左移时保留回找资格并继续右搜。仍不越过阶段起点，回找
速度、距离/时间上限和故障处理保持。适用于 Task1 三变体和 Task2 两变体；协议未变。

## 兼容入口

旧 `classic/all` 都映射到 `PlanA`，`round1/round2` 映射到 `set1/set2`，`task0` 映射到 `task0-1`。
`plana/planb` 是大小写便利别名。原 Python `Task0Program` 保留，新任务库使用 `Task0_1Program`。
旧 `task1/task1-r1/task1-r2` 映射到 `task1-1/task1-1/task1-2`。
旧 `task2/task2-r1/task2-r2` 保留原完整任务范围，映射到 `collect-build-1/1/2`，
不会静默缩短为只采集。所有兼容选择器打印展开后的任务列表；Task3-2 的新返程同样适用。
旧 `Task1Round2*`、`Task2Round2*` Python 导入名仅保留为新变体类的别名；
`Task2Program.run()` 本身已是新步骤 1-14，旧完整调用应迁移到统一执行器的组合策略。

## 协议与验证边界

协议未变：命令编号、载荷长度、字段大端/CRC 小端、80 B 完整遥测、11 B 动作状态，
以及 A 板 200 ms 失联停止底盘/步进/吸盘并取消动作的行为均保持。
动作状态 6 继续用于 Grap2、Build 的底盘并行节点，不代表机构已完成。
Task 拆分不涉及协议同步或固件参数变更。

无硬件入口：`python tools/check.py`，全量测试：`python -m unittest discover -s tests -q`，
固件编译：在 Uniforest_A 中执行 `cmake --preset Debug` 和 `cmake --build build/Debug`。
组合测试检查实际任务函数的调用边界、配置、航向传递和中止逻辑，不模拟物理动作效果。
协议检查与固件编译不能代替实际串口/相机/标定检查和小范围动作测试。
