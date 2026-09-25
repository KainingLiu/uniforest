# Raspberry Pi 上位机程序

当前源码基线：2026-09-25。历史参数和验证结果集中在 [CHANGELOG.md](CHANGELOG.md)，下位机接口见 [PROJECT.md](../Uniforest_A/PROJECT.md)。

## 环境与目录

运行平台为 Raspberry Pi 5 8GB；开发主机 `192.168.137.50`，用户名 `uniforest`，项目目录 `/home/uniforest/Uniforest/RaspberryPi`。密码由队内管理员提供，不保存到仓库。

| 入口/目录 | 职责 |
| --- | --- |
| `main.py` | 比赛统一入口和轮次选择 |
| `robot.py` | 通信生命周期、设备聚合、预检与调试交互 |
| `Strategy/` | Task0/Task1/Task2、视觉对准、目标跟踪、顶墙与路线编排 |
| `control/` | 底盘位置外环、舵机/步进调试、A 板动作客户端 |
| `protocol/` | 帧编解码、传输和 schema v3 契约 |
| `vision/` | cube/tag 相机、方块检测、AprilTag 定位和标定配置 |
| `sensors/`、`utils/` | 传感器封装和通用辅助 |
| `tools/`、`tests/` | 实机/图像调试工具；少量协议格式和导入检查 |

自然语言 Agent 的部署、API 中转和命令行使用见 [`agent/README.md`](agent/README.md)。

启用 Robot 方块视觉后默认自动采集原图，保存到本机 `vision/yolo/data/collection/`，
按运行批次记录任务阶段、相机设置并建立 SQLite 索引。`python tools/collect_cube_data.py stats`
查看数量；本轮加 `--no-collect-data` 可停用，`--dataset-dir` 可更换目录。
采样、存储上限、独立补拍和标注要求见 [自动采集说明](vision/yolo/docs/DATA_COLLECTION.md)。
更换前臂舵机后，使用独立入口 `python tools/arm_zero_adjust.py` 输入角度寻找默认姿态，
详见[前臂零点调整](tools/arm_zero_adjust.md)。前臂角度仍沿用原坐标（默认 90°），
固定偏置统一在 A 板输出层处理；用户确认新默认输出为 102°，源码偏置为 +12°，
需通过 CLion 烧录后生效。正常心跳不打印日志，通信异常时提示，后台心跳仍保留。

2026-09-24 连续 Grap1 吸气故障排查：A 板原 `Suction_PumpOn()` 在取消上一轮释放
计时后未实际关闭释放阀，短于 1 秒的连续抓取可能持续漏气。固件已补齐关阀 PWM；
用户已于 2026-09-24 现场确认这是 Task2-R2 最后一次抓取失败、数量检查后补抓
成功的原因。仅上传上位机不能应用该修复，须烧录对应 A 板固件，详见下位机 README。

依赖方向为 `main.py → Strategy → robot → control/protocol/vision`。比赛代码不导入 `tools/` 或 `tests/`。历史备份不属于当前实现。

Task1/Task2（两轮）完成原定橙色抓取后执行携带数量检查：3 或 null 继续路线，
0/1/2 则补抓 3/2/1 块并复查；搜索距离耗尽则跳过检查直接继续。补抓共用原搜索
距离和编码器原点。`tools/carried_cube_count_test.py` 保留为独立标定采样入口。
识别 null 不包括通信、相机采集和动作故障，这些异常仍中止任务。动作参数、使用方法和
协议核对见 [携带数量检查说明](tools/carried_cube_count_test.md)。本功能协议未变。

当前检查时序为翻转 37.2°→等待 200 ms→前臂 120°→等待 300 ms→丢弃 3 帧、
采集 8 帧→前臂 90°→等待 200 ms→翻转 97.2°→清除检查视角旧结果。识别仅用
下方 30% 橙色宽度，仓内紫色计 1。抓取结束后先确认 ACTION_DONE、发送底盘零速，
最多等待 1 秒内连续 3 帧新遥测确认静止，满足即执行检查。
2026-09-24：结果为 3/null 时，前臂复位指令确认后立即开始后续后退（Task1 400 mm、
Task2 100 mm，两轮均适用），200 ms 复位等待和翻转复位与后退重叠；不重复后退。
橙色净横移量在后退前冻结。需要补抓时先复位并清掉旧视觉结果，再开始搜索。
独立检查入口仍完成复位后返回，不启动底盘。新时序和比赛衔接待现场验证。

### 仓库与树莓派部署记录

截至 2026-09-24，已定向上传 Task2 路线、紫色抓取前进 250 mm、两任务两轮
Tag6 ±8 mm/最低平移 80 mm/s/单轴停车、常规转向前馈 120°/s，以及数量检查
200/300/200 ms 与复位重叠。橙色/紫色 5 秒超时抓取、常规转向末段修复与
1500 ms 超时余量均已同步。清理键盘及重复临时入口后，远端最新 63 文件语法、
导入和协议检查通过；不自动启动实机任务。

| 项目 | 当前源码 / 已知部署状态 |
| --- | --- |
| 上位机路线、Tag6、数量检查 | 上述参数已定向上传，两轮入口已核对；现场效果待复测 |
| 方块粗对准 5 秒超时直接抓取 | 两任务两轮橙色、两轮 Task2 紫色均已同步树莓派，远端检查通过，现场效果待确认 |
| 常规转向末段与超时降级 | 最低 8°/s、容差内零速、超调反向；超时按预计匀速时间＋加速/保持时间＋1500 ms 计算，停车后继续；现场效果待确认 |
| A 板前臂零点 | 源码 +12°，逻辑 90°→输出 102°；须 CLion 烧录，尚未收到现场确认 |
| A 板 Grap1/2 行程 | 源码垂直下降/上升各 18.5 cm；须 CLion 烧录，尚未收到现场确认 |
| A 板 Grap3 收尾与吸气关阀 | 上升 5 cm 复位前臂、PumpOn 明确关阀；源码已改，固件版本由现场确认 |
| 橙色搜索实现 | 仓库含左边缘搜索恢复，远端保留既有旧搜索实现；定向同步未覆盖 |
| 步进调试默认半周期参数 | 仓库 83 μs；远端此前保留 100 μs，不能据此推断板上动作速度 |
| Robot 兼容接口 | 远端保留 quiet_heartbeat 参数；正常心跳已静默，失联提示和保护保留 |

文件上传、固件烧录与现场验证是不同状态；上传 Python 或文档不会改变 A 板动作表。
备份路径和各次检查结果见 [CHANGELOG.md](CHANGELOG.md)。

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

实际运行解释器需有 pyserial、NumPy 和 OpenCV contrib。键盘遥控已移除，不再依赖 pynput 或桌面键盘后端。

### 自然语言 Agent 部署

当前网络方案为“树莓派连接开发机，开发机转发 APIFUN”。开发机的 FlClash 负责访问公网，树莓派只访问开发机热点网关 `192.168.137.1`。

在开发机启动中转服务：

```powershell
cd D:\PROJECTS\Uniforest\RaspberryPi
python -m agent.relay --host 192.168.137.1 --port 8765 --profile APIFUN
```

保持这个窗口运行。中转服务会从项目根目录的 `Docs/API.md` 读取 `APIFUN gpt` 配置，并转发到 APIFUN 的 Responses 接口；日志只显示请求路径、状态码和耗时，不显示 API Key 或请求正文。

然后通过 SSH 登录树莓派，在树莓派上启动 Agent：

```bash
ssh uniforest@192.168.137.50
cd /home/uniforest/Uniforest/RaspberryPi
source .venv/bin/activate
export OPENAI_BASE_URL=http://192.168.137.1:8765
python -m agent.cli --profile APIFUN --vision --localization
```

启动后在 `你 >` 提示符直接输入自然语言。工具调用默认会在树莓派终端显示，例如 `get_robot_state`、`detect_tags`、`move_chassis` 和返回摘要；使用 `--quiet-tools` 可以关闭工具痕迹。输入 `退出` 或按 `Ctrl+C` 会停止视觉、发送急停并断开机器人。

Agent 模式会隐藏后台 `PONG` 心跳输出，避免它插入用户输入行；心跳线程和下位机通信看门狗仍保持运行。

如果不使用本机中转，直接运行 `python -m agent.cli --vision --localization`；这要求树莓派自身能够访问 API 域名。`Docs/API.md` 含有密钥，不要提交到 Git；部署时使用树莓派上的 `/home/uniforest/.config/uniforest/API.md` 私有副本。

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

这个入口只检查 Python 语法、依赖导入和 6 项通信协议格式，成功时只显示简短
汇总，失败时显示错误。不打开串口或相机，不模拟机器人，不代表动作、视觉
准确率或现场标定通过。`import_smoke.py` 仍可独立使用。

| 场景 | 命令 |
| --- | --- |
| 语法、导入、协议格式 | `python tools/check.py` |
| 同时编译 A 板 | `python tools/check.py --firmware` |

`--firmware` 需要 CMake、Ninja 和 ARM GCC 已在 PATH 中，执行 Debug 配置和构建，
不烧录。2026-09-14 已移除模拟机器人、模拟时钟、合成视觉和旧动作轨迹测试，
同时移除 `--native-actions`、`--module`。实机功能按对应调试入口和现场记录验证；
此次精简不能视为原有测试失败已被修复。历史结果留在 [CHANGELOG.md](CHANGELOG.md)。

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
| Task1 标签对准 | 前后最高 350，左右最高 280；容差外非零期望最低 80 mm/s | 300 mm/s² |
| Task2 标签对准 | 前后最高 260，左右最高 200；容差外非零期望最低 80 mm/s | 300 mm/s² |
| Build 前建筑对准 | 最高 250；近距离前后 60–90 mm/s | 1000 mm/s² |

旋转参数不随普通平移参数改变。上述为软件目标值，实际速度受位置 PID、剩余距离、负载和地面影响。

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
Tag6 对准和相机预检仍保留。Task1/Task2 两轮 Tag6 距离和横向容差统一为 ±8 mm。两个任务均取消精对准，航向容差均为 ±3°。
两任务标签对准在容差外的非零平移期望速度最低为 80 mm/s，前后和横向共用；
转向最低仍为 8°/s。该限幅位于加减速限制之前，起步和停车时下发速度可以更低，
进入容差后允许零速。此配置也适用于以后重新开启的 Tag3；当前 Tag3 仍跳过。

Task1/Task2 两轮 Tag6 单轴进入容差后立即下发该轴零速，清零该轴 PID 与保存的速度，
跳过软件减速过程；其他轴继续对准。停下的轴须连续两个有效新帧超差才恢复纠偏，
重新起步保留加速限制及最低平移期望速度（两任务均为 80 mm/s）。丢帧、过期或位置跳变会
打断超差确认，重复帧不计数；三轴实际合格连续 4 个新帧才结束。
PID、5 帧中值滤波和原超时不变；暂时关闭的 Tag3 不启用此单轴停车策略。

当前两轮 Task2 Tag6 参数：

| 参数 | 前后 | 横向 | 航向 |
| --- | ---: | ---: | ---: |
| 目标 | 425 mm | 0 mm | 180° |
| 容差 | ±8 mm | ±8 mm | ±3° |
| 速度上限 | 260 mm/s | 200 mm/s | 45°/s |
| 容差外非零期望最低速度 | 80 mm/s | 80 mm/s | 8°/s |
| 加减速限制 | 300 mm/s² | 300 mm/s² | 90°/s² |
| 减速区 / 近距离区 | 140 / 35 mm | 100 / 25 mm | — |

控制周期 50 ms，位置采用 5 帧中值滤波；结果超过 0.7 秒失效，丢失时停车，
持续丢失 2 秒或总对准超时 12 秒报错。无精对准模式下，每轴合格后期望速度为零；
三轴同时合格立即发送底盘零速，连续确认 4 个新帧后继续路线。

两轮 Task1/Task2 在橙色搜索累计达到 1800 mm 时，即使数量不足（包括零块），
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

基础格式检查：

```bash
python tools/check.py
```

保存图像验证可用 `python tools/cube_profile_probe.py IMAGE --profile task2_orange`，
输出 `left_clipped_y_range` 为左侧截断区域在原图中的纵向像素范围；`None` 表示无提示。

## 步进电机动作参数

Grap1、Grap2、Grap3 和 Build 的步进电机默认巡航速度约为 **5020 步/秒**（约
125.5 mm/s，按 40 步/mm 计算）；起步速度约 417 步/秒。加速和减速各 400 步，
使用 S 曲线。下位机目标半周期延时为 83 μs，实际值受 TIM7 10 μs tick 和 6/5
安全倍率量化。修改后需重新烧录 A 板。

## A 板机械动作

完整步骤见[机械动作流程](../Uniforest_A/ACTIONS.md)。`control/actions.py` 请求整套动作、监督状态与故障，并在指定节点协调后续底盘路线；机构时序位于 `Uniforest_A/Core/Src/actions.c`。

- Grap1/2/3 均无动作内固定等待，仍等步进运动完成；`robot.py --action` 使用正式动作，不追加测试等待。
- Grap1/2 的垂直下降和回程上升均为 18.5 cm；水平行程及交叉触发位置不变，须重新烧录 A 板后生效。
- Grap3 释放后的收尾段同时上升 9 cm、水平回收 5 cm；本段上升到 5 cm 时前臂先复位到逻辑 90°，两轴完成后全部舵机复位，无新增等待，须重新烧录 A 板后生效。
- Build 仅保留两次取件等待和三次分段抬臂等待，指令合计 2500 ms，第三次抬臂与上升重叠。
- Build 回收达到 18 cm 后下降 11.5 cm 并执行第三次取件姿态；随后上升 20.5 cm 与抬臂并行，上升到 15 cm 时启动水平伸出 23 cm。
- Grap2 回程上升到 5 cm、Build 最后一次释放后，上位机可开始后续底盘路线，机构继续收尾。独立测试不新增底盘路线，第二轮 Build 等完整结束。
- 并行路线中正常转弯结束只停底盘，动作异常、取消或遥测陈旧仍整机急停；下一次机械动作前确认本次完整结束。

修改 C 动作时序需通过 CLion 烧录；Task1 开舱 300 ms、关舱 0 ms 属于上位机参数，不需要烧录。最新部署与现场验证边界见[变更记录](CHANGELOG.md)。

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

| 消息 | 编号 | 载荷 |
| --- | --- | --- |
| 整套动作启动 | `0x50` | 6 字节 `>IBB`：非零请求编号、动作 ID、测试标志 |
| 动作状态查询 | `0x51` | 0 字节 |
| 动作状态 | `0x83` | 11 字节 `>IBBBI`：请求编号、ID、状态、阶段、uptime |

ID 1/2/3/4 对应 Grap1/Grap2/Grap3/Build；状态 0/1/2/3/4/5 为 idle/running/done/cancelled/timeout/rejected；schema v3 新增状态 **6：ACTION_CHASSIS_READY**，仍属于机构忙，仅允许底盘后续路线并行。拒绝时阶段字节为 ACK 错误码。完整协议以 `protocol/schema.json` 及双方实现为准。

若在指定节点报 `A-board action failed: state=6`，先核对树莓派的协议常量、动作客户端、底盘监视和策略是否一起更新，并重新启动程序。不要将状态 6 改成 DONE 或关闭急停。旧固件不产生状态 6，新客户端会等 DONE 后才执行后续路线。

上位机先确认新固件接口，再发动作；每 50 ms 查询状态，超过 1 秒未确认启动或状态陈旧 500 ms 即报错急停。A 板单段步进保护上限 30 秒、整套 120 秒，Grap 和 Build 均无步进结束后的额外固定等待。完成依据是匹配请求编号的动作状态，不是普通 ACK。

运行时 A 板拒绝其他步进、舵机和吸盘修改指令；步进停止会中止整套动作。通信失联、显式急停或取消后清除剩余动作，重新连接不会自动恢复，也不会自动重发动作请求。

吸盘的 PD12 气泵和 PD13 电磁阀由 50 Hz PWM 电子开关控制，禁止改为持续 GPIO 高电平。`gripper_close()` 启动气泵，`gripper_open()` 关闭气泵并非阻塞释放阀门 1 秒。独立心跳线程每 50 ms 发送 PING，A 板 200 ms 看门狗阈值不变。

## 视觉配置

### 方块

目标坐标统一在 `Strategy/vision_targets.py`，单位为相机坐标毫米：

| 目标 | X 目标 | 粗对准范围 | 微调范围 |
| --- | ---: | --- | --- |
| Task1 橙色 | 0.0 | [-20, 5] | [-5, 5] |
| Task2 紫色 | 0.0 | [-5, 5] | 不执行独立微调阶段 |
| Task2 橙色 | 0.0 | [-20, 5] | [-5, 5] |

橙色使用 `orange_cluster.py` 识别顶面簇左首方块，通过 `orange_fixed_geometry.py` 内嵌的 Task1/Task2 独立固定平面投影定位；不会每帧重新估计整个平面。两份 `task*_orange_fixed_calibration.json` 记录标定资料，运行时矩阵以 Python 常量为准。

`orange_config.py` 管理 HSV、面积、形态学和两任务独立 X 偏置；当前两个偏置均为 +5.0 mm，仅影响橙色返回值。`default` 对应 Task1，`task2_orange` 对应 Task2。固定置信度 80 只代表通过几何门槛，不代表测量准确率；视角改变、遮挡或图像裁边后需复测。

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
