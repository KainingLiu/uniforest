# 机器人测试命令

更新：2026-10-05。除 SSH 外，命令均在树莓派终端执行；每条命令都包含进入工程目录的步骤，可单独复制。一次只运行一个任务或动作程序，`Ctrl+C` 停止。

## 策略包 · 常用入口

### PlanA：完整三轮采集与搭建

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy PlanA
```

### PlanB：采集、投放及末尾两次搭建

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy PlanB
```

### PlanC：先执行 Task1-1 / Task1-2，再采集与搭建

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy PlanC
```

PlanC 共 11 个模块，包含 Task1-1→Task0-3→Task1-2 转场；已于 2026-10-04 同步树莓派，无硬件检查通过，现场衔接待验证。顺序：task0-1 → task1-1 → task0-3 → task1-2 → task2-1 → task3-4 → task2-2 → task3-2 → task1-3 → task2-2 → task3-3。
Task1-1 后通过 Task0-3 转到 0°、左移 2600 mm 并左顶墙，再执行 Task1-2。Task3-4 保持 180°接 Task2-2；随后 Task3-2 按原返程接 Task1-3。

### set1：起步 + 第一套任务

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy set1
```

### set2：起步 + 第二套任务

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy set2
```

### collect-build-1：Task0-2 + Task2-1 + Task3-1

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy collect-build-1
```

### collect-build-2：Task0-2 + Task2-2 + Task3-2

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy collect-build-2
```

collect-build-1/2 从 Task0-2 的起点摆车：先前进 900 mm、右移 2700 mm、转到 180°，再执行对应 Task2→Task3。策略包自动传递航向基准；两个独立进程不会自动衔接。

## 机械动作 · Build1 / Build2 / Build3 / Grap

上位机 Build1/Build2/Build3 入口已于 2026-10-04 同步树莓派；下位机动作以实际烧录固件为准，通过 CLion 的 OpenOCD + DAPLink 更新。以下单动作不执行建筑视觉对准，也不启动底盘后续路线。

### Build1：搭建一块

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --action build1
```

当前源码的取块下降与随后提升均为 20 cm，升到 15 cm 时抬臂至 4°。此距离修改需更新下位机固件。

### Build2：搭建两块

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --action build2
```

### Build3：搭建三块（原 Build）

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --action build3
```

### Grap1：Task2 橙色抓取

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --action grap1
```

### Grap2：紫色抓取

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --action grap2
```

### Grap3：Task1 橙色抓取

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --action grap3
```

旧 `--action build` 仍等同 `build3`。Task3 默认使用 Build3；若 Task1-0 补抓也搜索耗尽，则按退场复检的舱内总数 1/2/3 调用 Build1/2/3，0 块跳过建筑对准与搭建、继续路线，复检失败按默认缺 1 块调用 Build2。Task5 仍使用 Build3。

Build2 最后一次上升 11.5 cm，升到 5 cm 时水平伸出；第二块释放后发送底盘可继续通知，机构随后垂直上升 **14 cm**、水平回收 **23 cm**，两轴回到本次起点后全部舵机复位。单动作命令仍等待机构完整结束。

### 舵机复位与舱门

**全部舵机复位**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --action home
```

**开舱**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --action hatch_open
```

**关舱**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --action hatch_close
```

`home` 仅复位舵机，不为步进寻找机械原点。Grap 单动作不自动执行比赛短压墙。当前下位机源码前臂输出偏置为 **+12.3°**（宏值 125），不要手动再加一次；此处描述源码，不代表板上固件已经更新。完整动作见[下位机动作流程](C:/Users/LTY/Desktop/Uniforest/Uniforest_A/ACTIONS.md)。

## 单 Task

Task1-0/1/2/3 按 Task1 的 0°入口摆车；Task2-0/1/2 从相应位置以 180°进入。Task0-3 从前一 Task4/Task1 的结束位置、180°进入；Task5 从 Task1-2 结束位置、180°进入。Task0-1/2 从各自策略起点摆车。

**Task0-1：PlanA / PlanC 起步，前进 1150 mm**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task0-1
```

**Task0-2：PlanB 起步，前进 900 mm、右移 2700 mm、转到 180°**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task0-2
```

**Task0-3：回零转向、左移、左顶墙**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task0-3
```

**Task1-1**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task1-1
```

**Task1-2**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task1-2
```

**Task1-3**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task1-3
```

**Task2-1：采集与转场**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task2-1
```

**Task2-2：采集与转场**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task2-2
```

**Task5：两次建筑对准与 Build3**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task5
```

### 补抓模块 Task1-0 / Task2-0

Task1-0 默认抓取 3 个橙块，执行到 Tag6 对准前结束，不执行对准和后续投放。

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task1-0
```

Task2-0 默认抓取 3 个橙块：后退 2900 mm、转到 0°、左移 400 mm、左顶墙与前顶墙，然后抓取并完成 Task2-1 的退出路线。旧 task2-3 已改名为 task2-0。

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task task2-0
```

抓取数量接口在配置层：`Task1_0Config.target_cube_count`、`Task2_0Config.orange_target_count`，取值均为 1、2 或 3；上述 CLI 使用默认 3，没有单独的数量命令行参数。补抓执行器会自动传入缺失数量。

### Task3 / Task4：需要输入已知航向零点

摆在对应 Task2 末段前进 2750 mm 后的位置，初始航向 180°；Task3 装好搭建方块，Task4 装好投放方块。下列脚本会询问之前标定的航向零点；留空取消，不要把当前 yaw 或目标 180°当成零点。

**task3-1**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && bash tools/desktop_task.sh task3-1
```

**task3-2**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && bash tools/desktop_task.sh task3-2
```

**task3-3**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && bash tools/desktop_task.sh task3-3
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && bash tools/desktop_task.sh task3-4
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && bash tools/desktop_task.sh task3-5
```

**task4-1**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && bash tools/desktop_task.sh task4-1
```

**task4-2**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && bash tools/desktop_task.sh task4-2
```

### 当前单次补抓规则

- 仅 Task1-1/2 和 Task2-1/2 在搜索耗尽后触发跨区补抓；单 Task 入口同样使用统一执行器。
- Task1-1/2：执行 Tag6 前路线并检查数量，缺块时插入 Task2-0；补抓后续接原任务的 Tag6 对准、投放和后续动作。
- Task2-1/2：完成退出路线并检查数量，缺块时插入 Task0-3 + Task1-0，然后回到原任务队列。
- 搜索耗尽后的计数失败默认缺 1 块；补抓没有抓满也继续。Task1-0、Task2-0 不再递归补抓。
- Task1-0 若也搜索耗尽，数量检查与退场路线并行，最新舱内总数传给后续 Task3：1/2/3 块选择 Build1/2/3，0 块跳过搭建，检查失败使用 Build2。未耗尽时保持原流程和默认 Build3。
- 比赛视觉失效按现有降级继续；通信、遥测、急停或机构故障仍中止。

## 策略包流程与只读预览

**PlanA**：task0-1 → task1-1 → task2-1 → task3-1 → task1-2 → task2-2 → task3-2 → task1-3 → task2-2 → task3-3。

**PlanB**：task0-2 → task2-1 → task4-1 → task2-2 → task4-2 → task0-3 → task1-1 → task0-3 → task1-2 → task5。

**PlanC**：task0-1 → task1-1 → task0-3 → task1-2 → task2-1 → task3-4 → task2-2 → task3-2 → task1-3 → task2-2 → task3-3。

**set1**：task0-1 → task1-1 → task2-1 → task3-1。

**set2**：task0-1 → task1-2 → task2-2 → task3-2。

**collect-build-1**：task0-2 → task2-1 → task3-1。

**collect-build-2**：task0-2 → task2-2 → task3-2。

以下只打印计划，不连接机器人；预览列出固定任务序列，运行时可能按上述规则插入补抓。

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy PlanA --show-plan
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy PlanB --show-plan
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy PlanC --show-plan
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy set1 --show-plan
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy set2 --show-plan
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --list-tasks
```

## 连接、检查与桌面入口

**从电脑终端连接树莓派**，按提示输入密码：

```bash
ssh uniforest@192.168.137.50
```

**无硬件检查**：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/check.py
```

**设备预检**：连接 A 板，检查通信、遥测和两路视觉，不执行比赛动作。

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u robot.py --preflight --vision --localization
```

树莓派桌面当前提供 **PlanA、PlanB、PlanC、set1、set2、collect-build-1、collect-build-2** 七个机器人测试入口。桌面入口共用防重复启动锁；直接运行 main.py 的命令不受该锁保护。重新安装入口时使用：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python tools/install_desktop_entries.py
```

## 携带数量检查与前臂调零

**单独检查数量**：会移动前臂和翻转舵机。

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/carried_cube_count_test.py
```

**按仓内真实数量保存采样**，选择对应的一条：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/carried_cube_count_test.py --known-count 0
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/carried_cube_count_test.py --known-count 1
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/carried_cube_count_test.py --known-count 2
```

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/carried_cube_count_test.py --known-count 3
```

`--known-count` 只标记采样，不自动修改标定。结果保存在 `.diagnostics/carried_cube_count/`；结束时复位舵机、发送急停并关闭吸气，不适合用吸盘持续悬持负载。

**只看检查动作计划**，不连接硬件：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/carried_cube_count_test.py --dry-run
```

**前臂调零**：按提示输入逻辑角度，输入 `q` 退出；不额外叠加固件偏置，退出不自动复位。

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/arm_zero_adjust.py
```

## 视觉只读检查与图片采集

先退出占用同一摄像头的任务。以下视觉检查不驱动机器人。

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/camera_roles_test.py
```

**Task1 橙色**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u vision/opencv/cube_detector.py --camera cube --no-gui --profile default
```

**Task2 橙色**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u vision/opencv/cube_detector.py --camera cube --no-gui --profile task2_orange
```

**紫色**

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u vision/opencv/cube_detector.py --camera cube --no-gui --profile task2_purple
```

**建筑识别，只采集 5 秒图像信息**：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/building_vision_probe.py --camera cube --profile building --duration 5
```

**Tag 摄像头识别与位姿采样**：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u tools/tag_pose_probe.py --camera tag --frames 30
```

图片自动采集默认关闭。需要在本次 PlanA 比赛中开启时使用以下命令（会运行整车策略）：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && UNIFOREST_COLLECT_DATA=1 .venv/bin/python -u main.py --strategy PlanA
```

**查看图片采集数量**：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python tools/collect_cube_data.py stats
```

## 停止与其他说明

持续运行的程序按 `Ctrl+C` 停止。动作中断后先确认机构位置；不要同时运行多个占用串口或同一摄像头的程序。

实机顺序沿用项目约定：工作区检查 → Python 无硬件检查 → 开发电脑 Debug 配置/编译 → CLion 烧录需要更新的固件 → 设备和标定检查 → 小范围动作 → 完整任务。单纯同步树莓派不会更新 A 板固件。

旧 `classic/all` 对应 PlanA，`round1/round2` 对应 set1/set2，`task0` 对应 task0-1；新测试优先使用文首规范名称。已删除的 `action_test.py`、`building_build_test.py`、键盘遥控和按键定距脚本不再使用。

路线参数与历史部署记录见[策略说明](Strategy/README.md)和[变更记录](CHANGELOG.md)。机械动作只在 [ACTIONS.md](../Uniforest_A/ACTIONS.md) 维护；视觉参数见 [视觉说明](vision/opencv/README.md)。
