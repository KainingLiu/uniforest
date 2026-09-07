# Raspberry Pi 上位机程序

当前运行基线：2026-09-07。历史参数和验证结果集中在 [CHANGELOG.md](CHANGELOG.md)，下位机接口见 [PROJECT.md](../Uniforest_A/PROJECT.md)。

## 环境与目录

运行平台为 Raspberry Pi 5 8GB；开发主机 `192.168.137.50`，用户名 `uniforest`，项目目录 `/home/uniforest/Uniforest/RaspberryPi`。密码由队内管理员提供，不保存到仓库。

| 入口/目录 | 职责 |
| --- | --- |
| `main.py` | 比赛统一入口和轮次选择 |
| `robot.py` | 通信生命周期、设备聚合、预检与调试交互 |
| `Strategy/` | Task0/Task1/Task2、视觉对准、目标跟踪、顶墙与路线编排 |
| `control/` | 底盘位置外环、舵机/步进调试、A 板动作客户端、键盘控制 |
| `protocol/` | 帧编解码、传输和 schema v2 契约 |
| `vision/` | cube/tag 相机、方块检测、AprilTag 定位和标定配置 |
| `sensors/`、`utils/` | 传感器封装和通用辅助 |
| `tools/`、`tests/` | 人工调试工具、自动化测试和导入检查 |

依赖方向为 `main.py → Strategy → robot → control/protocol/vision`。比赛代码不导入 `tools/` 或 `tests/`。历史备份不属于当前实现。

### 安装

Linux：

```bash
cd /home/uniforest/Uniforest/RaspberryPi
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt -r vision/requirements.txt
```

Windows，在工作区根目录执行：

```powershell
cd RaspberryPi
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt -r vision\requirements.txt
.\.venv\Scripts\python.exe tests\import_smoke.py
```

实际运行解释器需有 pyserial、NumPy、OpenCV contrib 和 pynput。Linux 无桌面的 SSH 环境中，pynput 的 X 后端可能无法初始化；导入检查通过包元数据检查安装情况。

## 验证与启动顺序

1. 检查工作区状态、当前固件版本和机构起始位置。
2. 运行 Python 语法、导入和无硬件测试。
3. 在 `Uniforest_A/` 执行 `cmake --preset Debug`、`cmake --build build/Debug`。
4. 由用户通过 CLion 的 OpenOCD + DAPLink 配置烧录需要更新的固件。
5. 检查串口、相机角色、标定文件和预检结果。
6. 小范围动作测试通过后再进入完整任务；通信异常、遥测陈旧或急停后不自动续跑。

无硬件验证，在 `RaspberryPi/` 执行：

```bash
python -m compileall -q control protocol Strategy robot.py main.py
python tests/import_smoke.py
python -m unittest discover -s tests -t . -v
python ../Uniforest_A/tests/run_actions_host.py
```

最后一项需原生 C 编译器 `cc`，可在树莓派运行；仅编译真实 C 动作执行器与虚拟 HAL，不访问串口。全量 Python 测试的已知失败见 [CHANGELOG.md](CHANGELOG.md)，不能把当前全量结果写为全部通过。

### 设备与预检

串口优先采用 CMSIS-DAP 的 `/dev/serial/by-id/...` 路径；没有对应设备时依次回退到 `/dev/ttyACM0`、`/dev/serial0`。UART 参数为 115200、8N1。

相机角色由 `vision/camera_devices.json` 的 USB 序列号路径决定：

| 角色 | 设备 | 图像节点 |
| --- | --- | --- |
| `cube` | LRCP USB3.0 方块相机 | `video-index0` |
| `tag` | icSpring 标签相机 | `video-index0` |

角色设备缺失时直接报错，不按 `/dev/videoN` 的插入顺序替换。预检命令：

```bash
python robot.py --preflight --vision --localization
python tools/camera_roles_test.py
python task2_main.py --preflight-only
python building_build_test.py --preflight-only
```

预检读取通信、遥测和已启用的视觉状态，不执行比赛动作。Windows 调试可显式指定 `--port COM5 --camera 1`。Tag 标定文件当前为 `calibrated=false`，估算内参不等价于完成实测标定。

### 比赛入口

```bash
python main.py --task all
python main.py --task round1
python main.py --task round2
python main.py --task task1-r1
python main.py --task task2-r1
python main.py --task task1-r2
python main.py --task task2-r2
```

无参数 `main.py` 等价于 `--task all`；`task1`、`task2` 是第一轮别名。`task2_main.py` 保留第一轮 Task2 独立入口。任一任务失败后不继续后续任务。

完整顺序为 Task0 → Task1-R1 → Task2-R1 → Task1-R2 → Task2-R2。Task0 仅在完整流程中执行，以 750 mm/s、800 ms 加速前进 1200 mm。

可选诊断日志：

```bash
python main.py --task all --diagnostics-log /tmp/uniforest-run.jsonl
```

日志记录连接、定距请求/实际进度、耗时、任务状态和异常类型；默认不写日志。

## 底盘参数

### 平移速度与加速

| 场景 | 速度 | 加速 |
| --- | ---: | ---: |
| 两轮 Task1/Task2 普通定距前进、后退、左右移动 | 400 mm/s | 300 ms |
| Task2 紫色抓取后补偿前进 | 400 mm/s | 300 ms |
| Task0、Task1 投放前进、Task2 起步及前往 Build | 750 mm/s | 800 ms |
| 第一轮 Task2 Build 后左移 2500 mm | 750 mm/s | 800 ms |
| 连续横向搜索方块 | 300 mm/s | 无额外软件起步斜坡 |
| 常规向前/向左顶墙 | 200 mm/s | 无额外软件起步斜坡 |
| 抓取前短压墙 | 150 mm/s | 无额外软件起步斜坡 |
| 手动 `move` 定距命令默认值 | 750 mm/s | 800 ms |

公共参数在 `control/chassis.py`：
`NORMAL_DISTANCE_MOVE_SPEED_MM_S=400`、`NORMAL_DISTANCE_MOVE_ACCEL_MS=300`、
`LONG_DISTANCE_MOVE_SPEED_MM_S=750`、`LONG_DISTANCE_FORWARD_ACCEL_MS=800`。
路线层通过 `CompetitionProgram._checked_move()` 选择普通/高速加速参数。

视觉对准使用加速度限制，不是固定加速时长：

| 场景 | 速度配置 | 加速度限制 |
| --- | --- | ---: |
| 方块横向对准 | 起步 40，常规 100–250，远处最高 420 mm/s | 300 mm/s² |
| Task1 标签对准 | 前后最高 350，左右最高 280 mm/s | 300 mm/s² |
| Task2 标签对准 | 前后最高 260，左右最高 200 mm/s | 300 mm/s² |
| Build 前建筑对准 | 最高 250；近距离前后 60–90 mm/s | 1000 mm/s² |

键盘三挡为 250/600/1000 mm/s，线加速度 2000 mm/s²、减速度 2600 mm/s²。旋转参数不随普通平移参数改变。上述为软件目标值，实际速度受位置 PID、剩余距离、负载和地面影响。

### 位置环与停止条件

A 板以 1 kHz 运行四轮速度环并上报累计编码器；树莓派根据遥测运行位置外环、S 曲线速度规划和 IMU 航向保持。

- `move_forward()`、`move_right()` 支持正负距离，返回 `LinearMoveResult`。
- 到位窗口：位置误差不超过 1000 counts（约 3 mm），四轮转速绝对值均不超过 50 RPM，连续满足 50 ms。
- 到位后关闭速度前馈，由位置 PID 锁定；路线动作使用 `hold_ms=0`，不增加额外保持。
- 策略允许定距控制器超时但编码器进度达到 90% 以上的结果；取消、遥测丢失和进度不足不适用。
- 横移补偿系数当前为 `500/465`，来源于先前地胶测试记录。换场地或负载后需复测左右方向及不同距离，不能把编码器位移当作无滑移的实际位移。

```text
新横移系数 = 旧系数 × 指令距离 / 实测距离
```

### 顶墙

常规顶墙为 200 mm/s、最长 4 秒；抓取前短压墙为 150 mm/s、最长 1 秒。前向顶墙检查两个后轮，侧向顶墙使用四轮中至少三轮，低速/高电流阈值及确认时间在 `FirstTaskConfig`。

达到顶墙时间上限后先停车，默认按顶墙成功处理；`wall_timeout_is_success=False` 可启用严格超时报错。遥测丢失等通信故障始终终止任务。该超时容错与 A 板 200 ms 通信失联急停不同。

## 两轮比赛路线

### Task1

两轮共用 `Strategy/competition.py` 状态机，稳定导入入口为 `Strategy/task1.py`。

1. 以 200 mm/s 向前顶墙并重新标定当前航向零点。
2. 以 300 mm/s 连续向右搜索橙色，累计搜索上限 1500 mm；锁定目标后进行粗对准和末端微调。
3. 抓取前以 150 mm/s 短压墙并重新校准航向，执行 Grap3；完成 3 个方块。
4. 以 400 mm/s 后退 400 mm，转到启动零点顺时针 90° 航向。
5. 以 750 mm/s 前进 `2800 mm - 抓取阶段编码器实测净右移量`，再转到 180° 航向。
6. 对准 Tag6 至距离 425 mm、横向零点；按轮次向右定距移动后顶墙卸载。
7. 打开舱门，以 400 mm/s 后退 300 mm，关闭舱门；按轮次向左定距移动，再转回 360°（等价于 0°）航向。

转向额外保持已设为 0；不恢复历史文档中的 500 ms 等待。目标暂时丢失时停车并按当前目标跟踪流程处理，持续丢失或通信故障触发任务失败。

### Task2

1. 以 750 mm/s 前进 2350 mm，转到 -90° 航向，对准 Tag3 至 250 mm。
2. 第一轮以 400 mm/s 右移 100 mm，第二轮跳过；两轮均以 400 mm/s 前进 250 mm，再以 200 mm/s 向前顶墙。
3. 以 300 mm/s 向左搜索紫色，搜索上限 600 mm；视觉对准后短压墙并执行 Grap2。未找到紫色则跳过，后续橙色数量从 2 个增至 3 个。
4. 以 400 mm/s 后退 100 mm，转回 0°；以前进 `400 mm - 紫色阶段实测净右移量` 返回，速度 400 mm/s。净左移为负，因此增加返回距离。
5. 两轮均先以 200 mm/s 向左顶墙，再向前顶墙，重新校准航向。
6. 以 300 mm/s 向右搜索橙色，逐块粗对准、末端微调和短压墙，执行 Grap1。
7. 以 400 mm/s 后退 500 mm，再按 `700 mm - 橙色阶段实测净右移量` 补偿。两轮目标均为 700 mm；正值向右、负值向左、零值跳过，速度 400 mm/s。
8. 转至 180° 航向，以 750 mm/s 前进 2100 mm，对准 Tag6 至 425 mm。
9. 按轮次右移，进行建筑视觉对准并执行 Build。
10. 第一轮 Build 后以 400 mm/s 后退 200 mm，顺时针相对转 180°，以 750 mm/s 左移 2500 mm，最后以 200 mm/s 左顶墙结束。第二轮在 Build 完成后直接结束。

| 轮次差异 | 第一轮 | 第二轮 |
| --- | ---: | ---: |
| Task1 前进补偿基准 | 2800 mm | 2800 mm |
| Task1 Tag6 后右移 / 最终转向前左移 | 100 / 100 mm | 400 / 400 mm |
| Task2 Tag3 后右移 | 100 mm | 跳过 |
| Task2 橙色净右移目标 | 700 mm | 700 mm |
| Task2 Tag6 后右移 | 100 mm | 400 mm |
| Task2 Build 后路线 | 后退、转向、左移、左顶墙 | 无 |

上述普通定距平移均用 400 mm/s、300 ms 加速。当前 Task2 不再执行 Build 后 Tag1 对准流程。

## A 板机械动作

`control/actions.py` 只请求整套动作、等待状态并处理故障；实际距离、方向、舵机角度、吸盘时序和等待位于 `../Uniforest_A/Core/Src/actions.c`。修改机械时序后需重新编译并烧录 A 板。

- Grap1：水平伸出 22 cm、竖直下降 18 cm，重叠返回并释放。
- Grap2：水平伸出 27 cm、竖直下降 18 cm，重叠返回并释放。
- Grap3：伸出 27 cm并下降 9 cm；收回时竖直上升 9 cm，在 5 cm 处启动水平收回 22 cm，水平收回 14 cm 时触发下降 9 cm；释放后上升 9 cm并收回剩余 5 cm。
- Build：执行 C 动作表中的三次拾取/放置；完整输出和等待时序由 `actions_legacy_trace.json` 对照验证。

单动作测试，选择一条运行：

```bash
python action_test.py grap1
python action_test.py grap2
python action_test.py grap3
python action_test.py build
```

不传动作名默认 Grap3；Grap 测试模式结束后额外等待 1000 ms。Build 不使用该标志。每次测试前确认起始位置和运动空间。

建筑对准 + Build 单项入口：

```bash
python building_build_test.py
```

该入口不执行 Task2 路线或 Tag6 横移，只在建筑连续对准后执行 Build。正式 Task2 对建筑目标丢失/对准超时会先停车告警后继续 Build；底盘通信等其他异常仍终止任务。

### 协议与中止

| 消息 | 编号 | 载荷 |
| --- | --- | --- |
| 整套动作启动 | `0x50` | 6 字节 `>IBB`：非零请求编号、动作 ID、测试标志 |
| 动作状态查询 | `0x51` | 0 字节 |
| 动作状态 | `0x83` | 11 字节 `>IBBBI`：请求编号、ID、状态、阶段、uptime |

ID 1/2/3/4 对应 Grap1/Grap2/Grap3/Build；状态 0/1/2/3/4/5 为 idle/running/done/cancelled/timeout/rejected，拒绝时阶段字节为 ACK 错误码。完整协议以 `protocol/schema.json` 及双方实现为准。

上位机先确认新固件接口，再发动作；每 50 ms 查询状态，超过 1 秒未确认启动或状态陈旧 500 ms 即报错急停。A 板单段步进保护上限 30 秒、整套 120 秒，步进完成后等待 100 ms。完成依据是匹配请求编号的动作状态，不是普通 ACK。

运行时 A 板拒绝其他步进、舵机和吸盘修改指令；步进停止会中止整套动作。通信失联、显式急停或取消后清除剩余动作，重新连接不会自动恢复，也不会自动重发动作请求。

吸盘的 PD12 气泵和 PD13 电磁阀由 50 Hz PWM 电子开关控制，禁止改为持续 GPIO 高电平。`gripper_close()` 启动气泵，`gripper_open()` 关闭气泵并非阻塞释放阀门 1 秒。独立心跳线程每 50 ms 发送 PING，A 板 200 ms 看门狗阈值不变。

## 视觉配置

### 方块

目标坐标统一在 `Strategy/vision_targets.py`，单位为相机坐标毫米：

| 目标 | X 目标 | 粗对准范围 | 微调范围 |
| --- | ---: | --- | --- |
| Task1 橙色 | 0.0 | [-20, 5] | [-3, 3] |
| Task2 紫色 | 0.0 | [-5, 5] | [-3, 3] |
| Task2 橙色 | 0.0 | [-20, 5] | [-3, 3] |

橙色使用 `orange_cluster.py` 识别顶面簇左首方块，通过 `orange_fixed_geometry.py` 内嵌的 Task1/Task2 独立固定平面投影定位；不会每帧重新估计整个平面。两份 `task*_orange_fixed_calibration.json` 记录标定资料，运行时矩阵以 Python 常量为准。

`orange_config.py` 管理 HSV、面积、形态学和两任务独立 X 偏置；当前两个偏置均为 +5.0 mm，仅影响橙色返回值。`default` 对应 Task1，`task2_orange` 对应 Task2。固定置信度 80 只代表通过几何门槛，不代表测量准确率；视角改变、遮挡或图像裁边后需复测。

`cube_tracker.py` 管理连续确认、X/Z 跳变拒绝、丢帧保持、候选歧义和位置平滑。Task2 橙色跟踪使用 18 mm 歧义间隔，无法区分相邻目标时停车。

cube Linux 曝光配置为 `exposure=312`、`gain=32`、自动白平衡，保存于 `vision/camera_settings.json`。光照、镜头、相机位置或曝光改变后须重新核对色带与平面标定；软件配置不等于已验证的硬件事实。

### 建筑

使用 `building` 橙色色带（H0–50、S≥35）和轮廓上边缘定位，避免下半部遮挡干扰。目标为 X=0 mm、Z=75 mm，容差 X±3 mm、Z±6 mm、航向±2.4°，连续 3 帧确认。

采用横向优先控制；前后误差进入 30 mm 内使用 60–90 mm/s 减速带。轮廓锁定、跳变检查、帧新鲜度和超时参数集中在 `Task2Config.building_*`。顶边行标定比例为 `building_z_scale_mm_px`，当前表达式 `132.8 * 82.4`；改变现场条件后通过 `tools/building_vision_probe.py` 复测。

### AprilTag

`field_localizer.py` 仅使用 tag 相机图像，解析 AprilTag 36h11，以 IPPE 平面位姿候选和场地约束定位；不读取 IMU。比赛对准另由 IMU 保持航向。场地与标签参数在 `vision/field_map.json`，标签边长配置为 0.15 m。

`tag_camera_calib.json` 当前使用 125° 水平视场估算内参，`calibrated=false`，未提供实测广角畸变参数。场地标签坐标、高度、相机高度和安装偏移也需现场复核，不能将配置值当作实测结论。

### 只读视觉与调试工具

```bash
python vision/cube_detector.py --camera cube --no-gui --profile task2_orange
python tools/cube_profile_probe.py IMAGE --profile default
python tools/cube_profile_probe.py IMAGE --profile task2_orange
python tools/building_vision_probe.py --profile building
python vision/camera_tuner.py --camera cube
python vision/field_localizer.py --camera tag --duration 15
python tools/vision_subsystems_test.py --duration 10
```

调试控制台和底盘定距工具会根据操作者命令驱动机器人：

```bash
python tools/debug_console.py --port COM5
python tools/chassis_distance_test.py --port COM5
python tools/vofa_bridge.py --help
```

调参记录应包含日期、参数、适用轮次、测试入口和现场结果；协议变化必须同步双方代码和 schema。每天结束由用户另存历史快照，不覆盖旧备份。
