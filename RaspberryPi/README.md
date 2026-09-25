# Raspberry Pi 上位机程序

当前源码基线：2026-09-26。历史参数和验证结果集中在 [CHANGELOG.md](CHANGELOG.md)，下位机接口见 [PROJECT.md](../Uniforest_A/PROJECT.md)。

## 环境与目录

运行平台为 Raspberry Pi 5 8GB；开发主机 `192.168.137.50`，用户名 `uniforest`，项目目录 `/home/uniforest/Uniforest/RaspberryPi`。密码由队内管理员提供，不保存到仓库。

| 入口/目录 | 职责 |
| --- | --- |
| `main.py` | 比赛统一入口和轮次选择 |
| `robot.py` | 通信生命周期、设备聚合、预检与调试交互 |
| `Strategy/` | Task0/Task1/Task2、视觉对准、目标跟踪、顶墙与路线编排 |
| `control/` | 底盘位置外环、舵机/步进调试、A 板动作客户端 |
| `protocol/` | 帧编解码、传输和 schema v3 契约 |
| `vision/opencv/` | 当前 OpenCV 方块检测、AprilTag 定位、相机与标定；旧模块路径保留兼容入口 |
| `vision/yolo/` | 自动原图采集；训练、分割推理和 hybrid 接入尚未实现 |
| `agent/` | 自然语言控制、本地直控和 API 中转 |
| `sensors/`、`utils/` | 传感器封装和通用辅助 |
| `tools/`、`tests/` | 实机/图像调试、协议格式、Agent 与数据采集检查 |

依赖方向为 `main.py → Strategy → robot → control/protocol/vision`。比赛代码不导入
`tools/` 或 `tests/`，历史备份不属于当前实现。

自然语言 Agent 的部署、API 中转和命令行使用见 [`agent/README.md`](agent/README.md)。

启用 Robot 方块视觉后默认自动采集原图，保存到本机 `vision/yolo/data/collection/`，
按运行批次记录任务阶段、相机设置并建立 SQLite 索引。`python tools/collect_cube_data.py stats`
查看数量；本轮加 `--no-collect-data` 可停用，`--dataset-dir` 可更换目录。
采样、存储上限、独立补拍和标注要求见 [自动采集说明](vision/yolo/docs/DATA_COLLECTION.md)。

维护入口：[前臂零点调整](tools/arm_zero_adjust.md)、
[携带数量检查与标定](tools/carried_cube_count_test.md)、
[Grap/Build 动作流程](../Uniforest_A/ACTIONS.md)。角度、检查时序和机械参数在对应
文档维护。正常心跳日志静默，50 ms 后台心跳及通信失联保护保留。

### 仓库与树莓派部署状态

截至 2026-09-26：文件上传、固件烧录和现场验证分别记录，不等同于整库一致。

| 项目 | 当前状态 |
| --- | --- |
| Tag6 三帧滤波、平移 Ki/Kd=0 | 09-25 已定向同步两任务两轮；速度和 ±8 mm 容差不变，远端四套配置核对通过 |
| Task1 投放顶墙按 180° 更新航向基准 | 09-25 已同步，两轮换算检查通过 |
| 路线、数量检查复位重叠、橙色/紫色 5 秒超时抓取、常规转向最多额外 1.5 秒 | 此前已定向同步；最新远端 63 文件语法、导入、协议检查通过 |
| A 板动作及步进 | 当前 Grap3 回收 21.5+5.5 cm、下降触发 15 cm；步进 400/60/400。源码编译通过，须 CLion 烧录，现场效果待确认 |
| 橙色搜索 | 仓库包含左边缘回找，树莓派保留既有旧实现；未整体覆盖 |
| 独立步进调试默认值 | 仓库起步/巡航/加减速为 400/60/400；尚未同步，远端此前巡航为 100 μs，不能据此推断板上整套动作速度 |
| Agent、视觉目录拆分和自动采集 | 已合并 GitHub 的 590dca2；此次未同步树莓派，远端部署需另行核对 |
| Robot 兼容接口 | 仓库已合并 `quiet_heartbeat` 参数；远端具体版本与新增采集接口仍需核对 |

未由助手执行实机任务。详细备份与各次验证结果见 [CHANGELOG.md](CHANGELOG.md)。

### 安装

Linux：

```bash
cd /home/uniforest/Uniforest/RaspberryPi
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt -r vision/opencv/requirements.txt
```

Windows，在工作区根目录执行：

```powershell
cd RaspberryPi
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt -r vision\opencv\requirements.txt
.\.venv\Scripts\python.exe tests\import_smoke.py
```

实际运行解释器需有 pyserial、NumPy 和 OpenCV contrib。键盘遥控已移除，不再依赖 pynput 或桌面键盘后端。

### 自然语言 Agent

部署、API 中转、直控和自然语言入口统一见 [Agent 使用说明](agent/README.md)。
Agent 复用现有机器人接口；API 密钥通过私有配置或环境变量提供，不提交仓库。

## 验证与启动顺序

树莓派桌面提供 **Uniforest 全任务流程**、**Uniforest round1**、**Uniforest round2**
三个快捷入口，分别执行 `main.py --task all` / `round1` / `round2`，均包含 Task0。
打开入口即运行，在终端显示实时日志；按 Ctrl+C 停止，结束后按回车关闭窗口。
入口使用项目 `.venv/bin/python` 和 `tools/desktop_task.sh`，无需手动激活环境。
三个入口共用防重复启动锁；该锁仅约束桌面入口，不约束手动命令。

1. 检查工作区状态、当前固件版本和机构起始位置。
2. 运行 Python 语法、导入和无硬件测试。
3. 在 `Uniforest_A/` 执行 `cmake --preset Debug`、`cmake --build build/Debug`。
4. 由用户通过 CLion 的 OpenOCD + DAPLink 配置烧录需要更新的固件。
5. 检查串口、相机角色、标定文件和预检结果。
6. 小范围动作测试通过后再进入完整任务；通信异常、遥测陈旧或急停后不自动续跑。

无硬件验证，在 `RaspberryPi/` 执行：

```bash
python tools/check.py
```

这个入口检查 Python 语法、依赖导入、6 项通信协议格式和数据采集功能，成功时显示简短
汇总，失败时显示错误。不打开串口或相机，不执行实机动作，不代表动作、视觉
准确率或现场标定通过。`import_smoke.py` 仍可独立使用。

| 场景 | 命令 |
| --- | --- |
| 语法、导入、协议格式与数据采集 | `python tools/check.py` |
| 同时编译 A 板 | `python tools/check.py --firmware` |

`--firmware` 需要 CMake、Ninja 和 ARM GCC 已在 PATH 中，执行 Debug 配置和构建，
不烧录。2026-09-14 已移除模拟机器人、模拟时钟、合成视觉和旧动作轨迹测试，
同时移除 `--native-actions`、`--module`。实机功能按对应调试入口和现场记录验证；
此次精简不能视为原有测试失败已被修复。历史结果留在 [CHANGELOG.md](CHANGELOG.md)。

### 设备与预检

串口优先采用 CMSIS-DAP 的 `/dev/serial/by-id/...` 路径；没有对应设备时依次回退到 `/dev/ttyACM0`、`/dev/serial0`。UART 参数为 115200、8N1。

相机角色由 `vision/opencv/camera_devices.json` 的 USB 序列号路径决定：

| 角色 | 设备 | 图像节点 |
| --- | --- | --- |
| `cube` | LRCP USB3.0 方块相机 | `video-index0` |
| `tag` | icSpring 标签相机 | `video-index0` |

角色设备缺失时直接报错，不按 `/dev/videoN` 的插入顺序替换。预检命令：

```bash
python robot.py --preflight --vision --localization
```

预检读取通信、遥测和已启用的视觉状态，不执行比赛动作。Windows 调试可显式指定 `--port COM5 --camera 1`。Tag 标定文件当前为 `calibrated=false`，估算内参不等价于完成实测标定。
仅排查相机映射时使用 `python tools/camera_roles_test.py`；`task2_main.py`
的 `--preflight-only` 保留为独立任务入口的可选检查，
无需每次重复执行全部预检。正常顺序是无硬件检查、一次设备预检、一个单动作、完整任务。

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

已连接树莓派时，可直接复制整行启动，无需提前切换目录或激活环境：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task all
```

无参数 `main.py` 等价于 `--task all`；`task1`、`task2` 是第一轮别名。`task2_main.py` 保留第一轮 Task2 独立入口。任一任务失败后不继续后续任务。

完整顺序为 Task0 → Task1-R1 → Task2-R1 → Task1-R2 → Task2-R2。
`round1` 执行 Task0 → Task1-R1 → Task2-R1；`round2` 执行 Task0 → Task1-R2 → Task2-R2。
三个入口都只在开头运行一次 Task0，以 1000 mm/s、800 ms 加速前进 1200 mm。
单独的 `task1`/`task2` 及 `task*-r*` 入口仍跳过 Task0。Task0 失败时不执行后续任务。

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
| Task0、Task1 投放前进 | 1000 mm/s | 800 ms |
| Task2 起步：第一轮 2500 mm、第二轮 2350 mm；两轮前往 Build 2800 mm（坡道） | 800 mm/s | 800 ms |
| 第一轮 Task2 Build 后左移 2500 mm | 1000 mm/s | 800 ms |
| 连续横向搜索方块 | 300 mm/s | 无额外软件起步斜坡 |
| 常规向前/向左顶墙 | 300 mm/s | 无额外软件起步斜坡 |
| 与抓取同时启动的短压墙 | 150 mm/s | 无额外软件起步斜坡 |
| 手动 `move` 定距命令默认值 | 1000 mm/s | 800 ms |

公共参数在 `control/chassis.py`：
`NORMAL_DISTANCE_MOVE_SPEED_MM_S=400`、`NORMAL_DISTANCE_MOVE_ACCEL_MS=300`、
`LONG_DISTANCE_MOVE_SPEED_MM_S=1000`、`LONG_DISTANCE_FORWARD_ACCEL_MS=800`。
路线层通过 `CompetitionProgram._checked_move()` 选择普通/高速加速参数；Task2 两段
坡道显式传入 800 ms，避免独立速度不再等于公共长距离常量后退回普通 300 ms。

视觉对准使用加速度限制，不是固定加速时长：

| 场景 | 速度配置 | 加速度限制 |
| --- | --- | ---: |
| 方块横向对准 | 起步 80，近处期望 100，远处最高 500 mm/s | 800 mm/s² |
| Build 前建筑对准 | 最高 250；近距离前后 60–90 mm/s | 1000 mm/s² |

Tag6 参数集中在下文“Tag6 对准”表。旋转参数不随普通平移参数改变。上述为软件目标值，实际速度受位置 PID、剩余距离、负载和地面影响。

### 位置环与停止条件

A 板以 1 kHz 运行四轮速度环并上报累计编码器；树莓派根据遥测运行位置外环、S 曲线速度规划和 IMU 航向保持。

- 常规路线转向：前馈 120°/s、600 ms 加速、40° 减速区、位置 PID 3.0/0.15/0、
  修正限幅 ±80°/s。容差外最低期望转速 8°/s（若调用指定更低速度，以指定速度为限），
  进入 ±1.5° 即发零速；越过目标按当前误差反向修正，清除旧积分。
  两任务路线使用 1 次到位确认、额外保持 0 ms。总超时为
  `abs(转角)/指定速度×1000 + 600 + hold_ms + 1500` ms，最后 1500 ms 为额外余量；
  120°/s 时转 90° 上限 2.85 秒，转 180° 上限 3.60 秒，到位即可提前结束。
  超时仍未完成则先停底盘，打印
  已转角度、剩余角度与末次输出，按降级成功继续路线，不取消并行 Grap2/Build。
  失联、遥测陈旧、急停、发送失败和机构故障仍中止；8°/s 的现场效果待确认。
- `move_forward()`、`move_right()` 支持正负距离，返回 `LinearMoveResult`。
- 到位窗口：位置误差不超过 1000 counts（约 3 mm），四轮转速绝对值均不超过 50 RPM，连续满足 50 ms。
- 到位后关闭速度前馈，由位置 PID 锁定；路线动作使用 `hold_ms=0`，不增加额外保持。
- 策略允许定距控制器超时但编码器进度达到 90% 以上的结果；取消、遥测丢失和进度不足不适用。
- 直线总超时为 `max(2000 ms, 预计匀速行驶时间 + accel_ms + hold_ms + 2000 ms)`。
  已取消原来的 5 秒最低总时长；2 秒为估算行程后的额外余量，不是所有动作总共只运行 2 秒。
  例如前进 100 mm、400 mm/s、加速 300 ms、无额外保持时，总超时约 2.55 秒。
- 横移补偿系数当前为 `500/465`，来源于先前地胶测试记录。换场地或负载后需复测左右方向及不同距离，不能把编码器位移当作无滑移的实际位移。

```text
新横移系数 = 旧系数 × 指令距离 / 实测距离
```

### 顶墙

2026-09-16：两轮 Task1 Grap3、Task2 紫色 Grap2 和橙色 Grap1（含补抓），对准后
连续发送抓取启动与短压墙速度命令。短压墙随动作状态轮询每 50 ms 更新，不先等待
压墙完成；堵转成立或达到 1 秒后给底盘零速。Grap1/3 等机构完整结束；Grap2 在压墙结束且回程上升达到 5 cm 后可并行执行后续路线。
橙色短压墙结束时仍重校准航向；紫色沿用不重校准的设置。动作拒绝、取消、通信或遥测
异常会中止组合动作。常规顶墙 300 mm/s 及并行抓取的效果待现场验证，协议未变。

两轮 Task1 投放前向前顶墙结束后，以当前姿态为任务航向 **180°** 修正航向零点，
再开舱投放。换算为“新零点 = 当前陀螺仪 yaw + 180°”，归一化到 ±180°；
后续回转到 0°使用新基准。原有起始顶墙、橙色短压墙仍以当前姿态为 0°。
这是上位机航向基准更新，不发送陀螺仪硬件校准命令；顶墙超时接受后同样执行。
2026-09-25 已定向同步树莓派，两轮航向换算检查通过，现场效果待确认。

常规顶墙为 300 mm/s、最长 4 秒；与抓取同时启动的短压墙为 150 mm/s、最长 1 秒。前向顶墙检查两个后轮，侧向顶墙使用四轮中至少三轮，低速/高电流阈值及确认时间在 `FirstTaskConfig`。

达到顶墙时间上限后先停车，默认按顶墙成功处理；`wall_timeout_is_success=False` 可启用严格超时报错。遥测丢失等通信故障始终终止任务。该超时容错与 A 板 200 ms 通信失联急停不同。

## 两轮比赛路线

### Task1

两轮共用 `Strategy/competition.py` 状态机，稳定导入入口为 `Strategy/task1.py`。

1. 以 300 mm/s 向前顶墙并重新标定当前航向零点。
2. 以 300 mm/s 连续向右搜索橙色，累计搜索上限 1800 mm；锁定目标后进行粗对准和末端微调。
3. 对准后同时启动 Grap3 和 150 mm/s 短压墙，短压墙结束后重新校准航向；抓取后执行数量检查及补抓。3 或 null 继续，搜索距离耗尽则跳过检查。
4. 以 400 mm/s 后退 400 mm，转到启动零点顺时针 90° 航向。
5. 以 1000 mm/s 前进 `2800 mm - 抓取阶段编码器实测净右移量`，再转到 180° 航向。
6. 对准 Tag6 至距离 425 mm、横向零点；单轴进入自身容差即停车，两帧超差后重启，不执行精对准，三项合格连续确认 4 个新帧后继续。按轮次向右定距移动后顶墙卸载。
7. 打开舱门并等待 300 ms，以 400 mm/s 后退 300 mm，关闭舱门后不等待；按轮次向左定距移动，再转回 360°（等价于 0°）航向。

转向额外保持已设为 0；不恢复历史文档中的 500 ms 等待。橙色目标持续丢失 0.5 秒返回搜索，通信故障中止任务；橙色粗对准超时按下文规则直接抓取。

两轮 Task1/Task2 的常规路线转向前馈速度为 **120°/s**，起步加速 **600 ms**。
现有转向控制器按速度比例缩放，减速区为剩余 **40°**，位置 PID 修正限幅
**±80°/s**；前馈叠加修正后指令可能超过 120°/s。到位容差仍为 **±1.5°**。
这不改变视觉对准的转向速度配置。

### Task2

1. 以 800 mm/s、800 ms 加速前进：第一轮 2500 mm，第二轮 2350 mm；然后转到 -90° 航向。两轮暂时跳过 Tag3 对准及其后的横移。
2. 两轮均直接以 400 mm/s 前进 250 mm，再以 300 mm/s 向前顶墙。
3. 以 300 mm/s 向左搜索紫色，第一轮搜索上限 750 mm、第二轮 650 mm；搜索和对准使用 `task2_purple`，屏蔽上方 40%。视觉对准后同时启动 Grap2 和短压墙。未找到紫色则跳过，后续橙色数量从 2 个增至 3 个。
4. Grap2 回程上升到 5 cm 且短压墙结束后，机构收尾与底盘路线并行：以 400 mm/s 后退 100 mm，转回 0°；以前进 `400 mm - 紫色阶段实测净右移量` 返回，速度 400 mm/s。净左移为负，因此增加返回距离。
5. 两轮均先以 300 mm/s 向左顶墙，再向前顶墙，重新校准航向。
6. 以 300 mm/s 向右搜索橙色，累计上限 1800 mm，逐块粗对准、末端微调后同时启动 Grap1 和短压墙。
7. 以 400 mm/s 后退 100 mm，再按 `550 mm - 橙色阶段实测净右移量` 补偿。两轮目标均为 550 mm；正值向右、负值向左、零值跳过，速度 400 mm/s。
8. 转至 180° 航向，以 800 mm/s、800 ms 加速前进 2800 mm，对准 Tag6 至 425 mm、横向 0 mm；距离与横向容差均为 ±8 mm，航向容差 ±3°。取消精对准，三项合格后停车，连续确认 4 个新帧即继续。
9. 按轮次右移，进行建筑视觉对准并执行 Build。
10. 第一轮 Build 第 14 步最后一次释放后，机构收尾与底盘路线并行：以 400 mm/s 后退 100 mm，顺时针相对转 180°，以 1000 mm/s 左移 2500 mm，最后以 300 mm/s 左顶墙结束。第二轮在 Build 完成后直接结束。

| 轮次差异 | 第一轮 | 第二轮 |
| --- | ---: | ---: |
| Task1 前进补偿基准 | 2800 mm | 2800 mm |
| Task1 Tag6 后右移 / 最终转向前左移 | 100 / 100 mm | 400 / 400 mm |
| Task2 起步前进 | 2500 mm | 2350 mm |
| Task2 Tag3 对准及随后横移 | 暂时跳过 | 暂时跳过 |
| Task2 紫色累计向左搜索上限 | 750 mm | 650 mm |
| Task2 橙色净右移目标 | 550 mm | 550 mm |
| Task2 Tag6 后右移 | 100 mm | 400 mm |
| Task2 Build 后路线 | 后退、转向、左移、左顶墙 | 无 |

上述普通定距平移均用 400 mm/s、300 ms 加速。当前 Task2 不再执行 Build 后 Tag1 对准流程。

`Task2Config.tag3_alignment_enabled=False` 控制临时跳过 Tag3 及其后横移。
如以后恢复该开关，Tag3 原目标 250 mm、距离/横向 ±10 mm、航向 ±3°、
无精对准及 4 个新帧确认仍保留；第一轮恢复右移 100 mm，第二轮仍不横移。

### Tag6 对准

Task1、Task2 各自两轮使用同一套参数。位置采用 **3 帧中值滤波**，控制间隔
50 ms，仅用有效新帧更新；航向由陀螺仪保持。两任务均取消精对准。

| 参数 | Task1 两轮 | Task2 两轮 |
| --- | ---: | ---: |
| 距离目标 / 容差 | 425 / ±8 mm | 425 / ±8 mm |
| 横向目标 / 容差 | 0 / ±8 mm | 0 / ±8 mm |
| 航向目标 / 容差 | 180° / ±3° | 180° / ±3° |
| 前后 / 横向最高速度 | 350 / 280 mm/s | 260 / 200 mm/s |
| 平移最低非零期望速度 | 80 mm/s | 80 mm/s |
| 前后减速区 / 近距离区 | 100 / 25 mm | 140 / 35 mm |
| 横向减速区 / 近距离区 | 80 / 20 mm | 100 / 25 mm |
| 平移加速度限制 | 300 mm/s² | 300 mm/s² |
| 转向最低 / 最高速度 | 8 / 45°/s | 8 / 45°/s |
| 转向加速度限制 | 90°/s² | 90°/s² |
| 视觉过期阈值 / 丢失超时 | 0.3 / 1 秒 | 0.7 / 2 秒 |
| 总对准超时 | 12 秒 | 12 秒 |

两任务共用增益（位置误差单位 mm，航向误差单位 °）：

| 控制轴 | Kp | Ki | Kd |
| --- | ---: | ---: | ---: |
| 前后 | 0.8 / 0.300549527 ≈ 2.66179 | 0 | 0 |
| 横向 | 1.2 / 0.300549527 ≈ 3.99269 | 0 | 0 |
| 航向 | 1.5 | 0.02 | 0.03 |

单轴进入容差后立即零速并清零该轴 PID 和速度状态，跳过软件减速；其他轴继续
对准。该轴连续两个有效新帧超差才重新启动，三轴同时合格连续四个新帧后完成。
丢帧或跳变会打断确认，重复帧不计数。最低速度限幅在加速度限制之前，起步下发
速度可以低于 80 mm/s；合格轴允许零速。视觉丢失超时或总超时仍报错中止，不按
常规路线转向的超时降级规则放行。

09-25 已同步树莓派，现场效果待验证。当前关闭的 Tag3 共用滤波和增益，但不启用
上述单轴停车保持策略；重新启用前应核对配置。

### 搜索距离与回找

两轮 Task1/Task2 在橙色搜索累计达到两轮 Task1/Task2 在橙色搜索累计达到 1800 mm 时，即使数量不足（包括零块），
也结束抓取并继续对应的卸载或 Build 路线。只允许搜索范围耗尽这一结果继续；
其他通信、取消或动作异常仍按原故障流程处理。橙色预算只累计正常向右搜索速度乘
实际指令时间，不包括停车确认、左移回找和视觉对准，抓取单块后不重置；后续路线
补偿仍使用编码器实测净横移。到极限时已出现候选，允许停车完成确认。
紫色累计向左搜索上限为第一轮 750 mm、第二轮 650 mm。

橙色左缘回找（2026-09-10，适用两轮 Task1/Task2）：没有有效橙色候选，且有效 ROI
内面积足够的橙色簇连续 3 个新帧触及画面左边界时，先停 100 ms，再以 200 mm/s
向左寻找真实左缘。候选出现后停车确认，再沿用对准、抓取流程。左半屏普通橙色
像素、右侧裁剪和重复帧不触发回找；Task2 仍屏蔽上半屏。

单次回找最多 1000 mm、6 s，并受橙色阶段编码器起点限制（预留 10 mm 加一个控制
周期行程）。同一连续截断区域只尝试一次；再触发要求连续 3 个新帧不再截断，且净
右移超过上次触发位置 100 mm。回找耗尽、到起点或目标连续消失后停车再继续向右；
图像/遥测失效、通信发送失败或搜索期间本地急停则终止任务。参数尚未实机确认，
起点约束不代表消除了打滑和惯性。协议未变，无需重新烧录。

保存图像验证可用 `python tools/cube_profile_probe.py IMAGE --profile task2_orange`，
输出 `left_clipped_y_range` 为左侧截断区域在原图中的纵向像素范围；`None` 表示无提示。

## A 板机械动作

Grap/Build 完整时序集中在[机械动作流程](../Uniforest_A/ACTIONS.md)，
速度与计时换算集中在[下位机技术说明](../Uniforest_A/PROJECT.md#当前步进参数)。
修改固件动作或速度需要 CLion 烧录；Task1 开舱等待属于上位机，不需要烧录。

`control/actions.py` 请求并监督动作。Grap2 回程上升 5 cm、Build 最后释放后可
并行后续底盘路线，机构继续收尾；独立动作入口及无后续路线的第二轮 Build 等完整
完成。下一次机械动作前必须等机构与底盘都结束，通信、取消或机构故障仍整机急停。

单动作测试，选择一条运行：

```bash
python robot.py --action grap1
python robot.py --action grap2
python robot.py --action grap3
python robot.py --action build
```

必须显式指定动作；入口等待整套动作完成，不追加后续底盘路线。每次测试前确认起始位置和运动空间。

正式 Task2 对建筑目标丢失/对准超时会先停车告警后继续 Build；底盘通信等其他异常仍终止任务。

### 协议与中止

命令、载荷和状态定义集中在 [schema.json](protocol/schema.json) 与
[下位机协议说明](../Uniforest_A/PROJECT.md#4-上下位机协议)。协议未变：完整遥测
80 B，动作状态 11 B，A 板 200 ms 通信失联急停。

上位机每 50 ms 查询动作，超过 1 秒未确认启动或状态陈旧 500 ms 即报错急停；
完成依据是匹配请求编号的动作状态，不是普通 ACK。运行期间拒绝其他机构修改，
取消、失联或重连不会自动续跑、重发动作。

若报 `A-board action failed: state=6`，应核对协议常量、动作客户端、底盘监督和
策略是否配套更新。状态 6 仍是机构忙，不能改成 DONE；新客户端配旧固件会等 DONE。

## 视觉配置

### 方块

目标坐标统一在 `Strategy/vision_targets.py`，单位为相机坐标毫米：

| 目标 | X 目标 | 粗对准范围 | 微调范围 |
| --- | ---: | --- | --- |
| Task1 橙色 | 0.0 | [-20, 5] | [-5, 5] |
| Task2 紫色 | 0.0 | [-5, 5] | 不执行独立微调阶段 |
| Task2 橙色 | 0.0 | [-20, 5] | [-5, 5] |

橙色使用 `vision/opencv/orange_cluster.py` 识别顶面簇左首方块，通过 `vision/opencv/orange_fixed_geometry.py` 内嵌的 Task1/Task2 独立固定平面投影定位；不会每帧重新估计整个平面。`vision/opencv/` 中两份 `task*_orange_fixed_calibration.json` 记录标定资料，运行时矩阵以 Python 常量为准。

`vision/opencv/orange_config.py` 管理 HSV、面积、形态学和两任务独立 X 偏置；当前两个偏置均为 +5.0 mm，仅影响橙色返回值。`default` 对应 Task1，`task2_orange` 对应 Task2。固定置信度 80 只代表通过几何门槛，不代表测量准确率；视角改变、遮挡或图像裁边后需复测。

`cube_tracker.py` 管理连续确认、X/Z 跳变拒绝、丢帧保持、候选歧义和位置平滑。
对准时 X/Z 跳变门槛为 45/80 mm，置信度至少 25%，位置滤波窗口为 1 帧。
默认候选关联分数差门槛为 12 mm；Task2 橙色搜索及粗对准传入 18 mm，末端微调
沿用默认 12 mm。关联分数为横向差加 0.5 倍纵深差，分数过近时本帧不用于控制。

#### 方块对准的底盘控制

`competition.py` 的 `_align_cube()` 每轮等待 30 ms，以相机 X 偏差控制左右横移，
前后和旋转指令均为零；该阶段没有额外 IMU 航向纠偏。Z 只参与目标关联。
横移指令经麦克纳姆解算转为四轮 RPM，下位机以 1 kHz 速度环执行。

- 基础比例增益 Kp=1.5，Ki=Kd=0；基础输出限幅 250 mm/s。
- 距目标 120 mm 及以上时期望速度 500 mm/s，30–120 mm 间随距离收缩；
  30 mm 内、尚未进入合格窗口时期望速度为 100 mm/s。最终期望上限为 500 mm/s。
  距离分段给出速度下限，再与基础比例输出取较大值，因此 Kp 不变也会改变速度曲线。
- 从静止起步指令为 80 mm/s，速度变化限制为 800 mm/s²；反向前先发零速度。
  进入合格范围时直接发零速度，不经缓慢减速，也不发送整车急停命令。
- 合格窗口内累计 2 个新帧样本后通过；已进入窗口后若只越界 5 mm 以内，
  最多停车保持 200 ms。边缘保持不增加确认次数，但保留已有次数。
- 橙色粗对准后执行最长 0.5 s 的 ±5 mm 微调，使用同一速度规则。微调超时、
  本阶段有有效样本且最后记录位置仍新鲜并在粗窗口内时允许抓取；不保证每次达到 ±5 mm。
- 新画面中目标无效时先停车，持续丢失 0.5 s 返回搜索。两任务两轮橙色粗对准
  达到 5 s 上限后停车，跳过精对准，直接执行 Grap3（Task1）或 Grap1（Task2）
  与短压墙；正常粗对准完成仍进行精对准。两轮 Task2 紫色对准同样采用独立
  5 s 上限，超时停车后直接执行 Grap2 与短压墙，紫色没有精对准阶段。
  对准成功后，Grap3（Task1 橙色）、Grap1（Task2 橙色）或 Grap2（紫色）
  与 150 mm/s 短压墙同时启动。

2026-09-23 修复：重复帧仅在图像年龄与目标丢失计时均未到 0.5 s 时等待新帧，
不重复确认；到期进入停车/丢失处理（按 30 ms 循环检查）。缺失、过期或未来时间戳
不参与控制；无效目标时清零内部速度、确认和窗口保持状态，清除旧位置，恢复识别后
从 80 mm/s 起步。微调超时也不得用已丢失或过期的位置放行抓取。

本次提速适用两轮 Task1/Task2 橙色及紫色共用对准；紫色仍无独立微调阶段。
搜索 300 mm/s、近距离期望 100 mm/s、Kp=1.5、30 ms 循环及原粗窗口保持。
协议未变，无新依赖；最初仅本地修改，后续按用户要求已于 2026-09-23 定向同步树莓派，
远端离线检查和两轮配置核对通过，保留远端既有搜索实现。
实机入口为两轮 Task1/Task2 或完整比赛，耗时和抓取成功率待现场复测。

两轮紫色搜索及对准自动使用 `task2_purple`：只检测紫色，下方 60% 为有效区域，
640×480 时排除第 0–191 行。掩码处理保留完整图像坐标和相机内参，不裁图后
重新计算中心。成功、搜索耗尽或异常退出后恢复 `default`，切换时清除旧检测结果。
`task2_orange` 仍屏蔽上方 50%，`default`/`building` 不屏蔽。紫色色带未调整。

cube Linux 曝光配置为 `exposure=312`、`gain=32`、自动白平衡，保存于 `vision/opencv/camera_settings.json`。光照、镜头、相机位置或曝光改变后须重新核对色带与平面标定；软件配置不等于已验证的硬件事实。

### 建筑

使用 `building` 橙色色带（H0–50、S≥35）和轮廓上边缘定位，避免下半部遮挡干扰。目标为 X=0 mm、Z=75 mm，容差 X±3 mm、Z±6 mm、航向±2.4°，连续 3 帧确认。

采用横向优先控制；前后误差进入 30 mm 内使用 60–90 mm/s 减速带。轮廓锁定、跳变检查、帧新鲜度和超时参数集中在 `Task2Config.building_*`。顶边行标定比例为 `building_z_scale_mm_px`，当前表达式 `132.8 * 82.4`；改变现场条件后通过 `tools/building_vision_probe.py` 复测。

### AprilTag

`field_localizer.py` 仅使用 tag 相机图像，解析 AprilTag 36h11，以 IPPE 平面位姿候选和场地约束定位；不读取 IMU。比赛对准另由 IMU 保持航向。场地与标签参数在 `vision/opencv/field_map.json`，标签边长配置为 0.15 m。

`tag_camera_calib.json` 当前使用 125° 水平视场估算内参，`calibrated=false`，未提供实测广角畸变参数。场地标签坐标、高度、相机高度和安装偏移也需现场复核，不能将配置值当作实测结论。

### 只读视觉与调试工具

```bash
python vision/cube_detector.py --camera cube --no-gui --profile task2_orange
python vision/cube_detector.py --camera cube --no-gui --profile task2_purple
python tools/cube_profile_probe.py IMAGE --profile default
python tools/cube_profile_probe.py IMAGE --profile task2_orange
python tools/cube_profile_probe.py IMAGE --profile task2_purple
python tools/building_vision_probe.py --profile building
python vision/camera_tuner.py --camera cube
python vision/field_localizer.py --camera tag --duration 15
```

调试控制台会根据操作者输入的完整命令驱动机器人；不提供连续按键遥控：

```bash
python robot.py --port COM5
python tools/vofa_bridge.py --help
```

维护工具保留设备预检、前臂调零、携带数量标定、视觉采样/离线分析、相机调参和 VOFA 遥测。一次性部署、回退与标定修订脚本已清理；既有标定文件、实拍图片、日志和备份保留。

调参记录应包含日期、参数、适用轮次、测试入口和现场结果；协议变化必须同步双方代码和 schema。每天结束由用户另存历史快照，不覆盖旧备份。
