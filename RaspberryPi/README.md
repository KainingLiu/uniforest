# Raspberry Pi 上位机程序

`RaspberryPi/` 是 Uniforest 机器人的持续迭代上位机工程，运行于 Raspberry Pi 5 8GB，负责算法、视觉、策略、任务编排以及与 RoboMaster A 板下位机通信。

## 目录职责

- `main.py`：正式比赛统一入口，可运行完整流程或选择单项任务。
- `robot.py`：通信生命周期和底层子系统聚合。
- `vision/`：视觉采集、标定与识别。
- `Strategy/`：比赛状态机与任务流程。
- `control/`：面向下位机的运动和执行机构控制接口。
- `protocol/`：上位机与下位机通信协议及传输层。
- `sensors/`：上位机侧传感器接口。
- `utils/`：通用工具。
- `tools/`：人工调试、标定和诊断入口，不由比赛程序导入。
- `tests/`：自动化测试和导入检查。
- `requirements.txt`：Python 依赖。

协议契约集中记录在 `protocol/schema.json`，由 `protocol/schema.py` 校验 Python
常量；修改命令编号、载荷长度、遥测布局或安全超时前，先更新 schema，再同步
`Uniforest_A/Core/Inc/protocol.h`、`Core/Src/protocol.c` 和协议测试。

依赖方向固定为：`main.py` → `Strategy/` → `robot.py` → `control/`、`protocol/`、`vision/`。比赛代码不得导入 `tools/` 或 `tests/`，硬件模块也不得反向依赖比赛策略。

## 开发约定

当前实现只以本目录和 `../Uniforest_A/` 为准。工作区根目录的 `../备份/` 保存每日成果快照，默认只读，不应在其中继续开发。

树莓派开发连接信息：主机 `192.168.137.50`，用户名 `uniforest`。密码由队内管理员线下提供，不写入代码、配置、日志或项目文档。日常开发优先使用 VSCode Remote-SSH；修改通信协议时，必须同步核对 `../Uniforest_A/` 中的下位机实现，并更新双方文档与测试。

## 底盘位置环

比赛长距离移动速度由 `control/chassis.py` 中的公共参数
`LONG_DISTANCE_MOVE_SPEED_MM_S` 统一管理。Task1 起步与投放前进、Task2
起步、Build 前进以及 Build 后前进都引用该参数，当前为
`750 mm/s`（0.75 m/s）。

直线位置环由上、下位机协同完成：A 板以 1 kHz 运行四轮速度环并上报多圈累计编码器位置；树莓派以遥测频率运行位置外环、S 曲线速度规划和 IMU 航向保持。`control/chassis.py` 中的 `move_forward()`、`move_right()` 支持正负距离，并返回 `LinearMoveResult` 供调试记录。

直线动作的结束条件同时检查位置误差和四轮转速。进入目标区后关闭速度前馈，只保留位置 PID 锁定；位置或轮速再次超限会重新开始稳定计时。连续稳定 700 ms 后才切换为零速闭环并返回。策略层的路线定距动作使用 `hold_ms=0`，由位置环自身完成停止确认。

正式主程序与临时测试共用 `control/chassis.py` 中的同一套位置环参数。当前长距离移动公共速度为 750 mm/s（0.75 m/s），由 `LONG_DISTANCE_MOVE_SPEED_MM_S` 统一管理。交互模式可使用 `move forward|backward|left|right MM [MM/S]`，例如 `move right 1000`。

### 方块视觉快速调参

三种方块的目标 X、粗对准允许范围和末端微调范围统一维护在
`Strategy/vision_targets.py`，现场标定后只修改该文件即可：

| profile | 目标 X | 粗对准范围 | 末端微调范围 |
| --- | ---: | ---: | ---: |
| Task1 橙色 | 0.0 mm | [-20.0, 5.0] mm | [-3.0, 3.0] mm |
| Task2 紫色 | 0.0 mm | [-5.0, 5.0] mm | [-3.0, 3.0] mm |
| Task2 橙色 | 0.0 mm | [-20.0, 5.0] mm | [-3.0, 3.0] mm |

贴合的橙色方块在整体轮廓内部存在黑色接缝时，检测器会先保留橙色连通区域，
再检查贯穿大部分区域的低亮度接缝，并将其分割为独立候选。分割结果仍需通过
四边形、尺寸和三维距离校验；普通单方块不会进入接缝分割路径。

视觉只读标定命令（不会驱动底盘或机械爪）：

```bash
cd /home/uniforest/Uniforest/RaspberryPi
timeout 8s .venv/bin/python vision/cube_detector.py \
  --camera cube --no-gui --profile task2_orange
```

机械动作单项测试入口仍为 `action_test.py`；执行前确认机器人上电且机械爪周围无障碍：

```bash
.venv/bin/python action_test.py grap1 --port /dev/serial/by-id/<cmsis-dap-id>
.venv/bin/python action_test.py grap2 --port /dev/serial/by-id/<cmsis-dap-id>
.venv/bin/python action_test.py grap3 --port /dev/serial/by-id/<cmsis-dap-id>
.venv/bin/python action_test.py build --port /dev/serial/by-id/<cmsis-dap-id>
```

当前末端执行器为吸盘：PD12 气泵、PD13 电磁阀均使用 50 Hz 舵机式 PWM 电子开关，0° 关闭、180° 开启，禁止将任一路改为持续高电平。原 `gripper_close()` 语义为启动并保持气泵 PWM，原 `gripper_open()` 语义为关闭气泵并非阻塞触发电磁阀 PWM 1 秒。

上位机启动后由独立线程每 50 ms 发送一次 PING，维持 A 板 200 ms 通信看门狗；动作线程中的等待不会导致气泵被误判为通信失联而关闭。

麦克纳姆轮横移可能因滚子变形、轮压、地面摩擦和重心偏置产生编码器无法观测的滑移。根据比赛地胶上 500 mm 指令实测约 465 mm，默认横移系数标定为 `500/465 = 1.07527`。若场地或负载变化，可调用 `chassis.set_lateral_distance_scale(scale)` 重新标定：

```text
新系数 = 旧系数 × 指令距离 / 实测距离
```

应分别测试左移、右移以及至少两个距离。若误差随方向或距离显著变化，它不是单一比例换算问题，应先检查轮子安装方向、滚子灵活度、四轮着地压力和底盘重心。

## 程序入口

树莓派首次部署：

```bash
cd /home/uniforest/Uniforest/RaspberryPi
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

Windows 调试环境使用项目本地虚拟环境：

```powershell
cd RaspberryPi
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt -r vision\requirements.txt
.\.venv\Scripts\python.exe tests\import_smoke.py
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

树莓派 Linux 环境使用同样的依赖清单；若系统 Python 已提供 NumPy/OpenCV，也可以按 README 开头的 `--system-site-packages` 方式创建虚拟环境。`pyserial`、`opencv-contrib-python` 和 `pynput` 必须在实际运行比赛程序的解释器中可导入。

树莓派正式比赛程序默认优先使用 CMSIS-DAP 的 `/dev/serial/by-id/...`
稳定串口路径。摄像头按 USB 硬件序列号映射角色，不使用 `/dev/videoN`
的插入顺序：`cube` 是 LRCP USB3.0 方块相机，`tag` 是 icSpring
视觉标签相机。角色映射保存在 `vision/camera_devices.json`，并固定使用
各摄像头的 `video-index0` 图像节点；设备缺失时程序直接报错，不回退到
其他摄像头。若没有 CMSIS-DAP，串口依次回退到 `/dev/ttyACM0` 和
`/dev/serial0`：

```bash
source .venv/bin/activate
python main.py
```

只做实机连接和传感器预检、不执行底盘、舵机或步进动作：

```bash
python robot.py --preflight --vision --localization
```

预检会检查 PING/遥测新鲜度、协议 schema、串口帧和 CRC 统计，以及已启用的
cube/tag 子系统；返回码为 `0` 才允许进入比赛入口。预检失败不会自动重试任务。

比赛运行时如需保存动作和故障诊断，可显式指定 JSONL 日志：

```bash
python main.py --task all --diagnostics-log /tmp/uniforest-run.jsonl
```

日志记录串口连接、每次底盘定距动作的请求/实际距离/耗时/完成比例，以及顶层
异常分类。异常分类包括 `hardware_fault`、`perception_fault`、
`motion_degraded` 和 `strategy_fault`；同时记录每个 Task0/Task1/Task2 的开始、
完成或返回码失败事件。默认不写日志文件。

`main.py` 默认在同一次硬件连接中依次运行 Task0、第一轮 Task1/Task2、第二轮
Task1/Task2。Task0 仅在完整流程中执行，以 600 mm/s 前进 1200 mm；任何轮次
或单项调试入口都会跳过 Task0。各部分入口如下：

```bash
.venv/bin/python main.py --task all
.venv/bin/python main.py --task round1
.venv/bin/python main.py --task round2
.venv/bin/python main.py --task task1-r1
.venv/bin/python main.py --task task2-r1
.venv/bin/python main.py --task task1-r2
.venv/bin/python main.py --task task2-r2
.venv/bin/python main.py --task task1
.venv/bin/python main.py --task task2
```

`task1` 和 `task2` 是兼容旧命令的第一轮别名，分别等价于 `task1-r1` 和
`task2-r1`。`round1`、`round2` 只执行对应轮次的 Task1 和 Task2。

`task2_main.py` 继续保留 Task2 独立预检功能；任何任务返回失败或抛出异常时，
统一入口不会继续启动下一任务。

摄像头角色可显式指定，数字编号只用于临时诊断：

```bash
python main.py --camera cube
python vision/cube_detector.py --camera cube --no-gui
python vision/camera_tuner.py --camera cube
python tools/camera_roles_test.py
```

Windows 调试时显式指定端口和摄像头：

```powershell
python main.py --port COM5 --camera 1
```

调试控制台与键盘遥控：

```powershell
python tools/debug_console.py --port COM5
```

底盘定距测试：

```powershell
python tools/chassis_distance_test.py --port COM5
```

VOFA+ 遥测桥接：

```powershell
python tools/vofa_bridge.py --help
```

自动化测试：

```powershell
python -m unittest discover -s tests -v
```

临时机械动作实机测试入口默认只执行一次 Grap3，不进入比赛策略。后续调整
Grap1、Grap2 或 Build 时可将动作名作为位置参数传入：

```powershell
python action_test.py
python action_test.py grap1
```

建筑视觉对准 + Build 的独立测试入口如下。该程序跳过 Task2 路线，只使用当前
`cube` 相机画面对准建筑；只有连续确认对准后才触发 Build：

```powershell
python building_build_test.py
python building_build_test.py --preflight-only
```

独立测试固定当前位置，不执行 Tag6 后横移或航向转动；它只验证建筑视觉对准和
Build 执行器动作。比赛流程中的建筑对准和本测试入口共用 `Strategy/task2.py`。

Grap3 收回阶段使用 `CMD_STEPPER_MOVE_DUAL3` 一次下发交叉触发轨迹：竖直轴
连续上升 10 cm，在 5 cm处启动水平轴收回 22 cm，水平轴到 14 cm时触发竖直轴
反向下降 11 cm。该命令需要配套的最新 A 板固件；修改后应使用 CLion 配置的
OpenOCD + DAPLink 工具链烧录 `Uniforest_A`，再运行上述临时入口测试。
Build 放置完第一个方块后同样使用 `CMD_STEPPER_MOVE_DUAL3`：竖直轴
连续上升 10 cm，在 3 cm 处启动水平轴收回 23 cm，水平轴收回到
20 cm 时触发竖直轴反向下降 2 cm。

## AprilTag 全场定位

全场定位由 `vision/field_localizer.py` 在独立后台线程运行，只使用 `tag`
摄像头的 AprilTag 36h11 图像，不读取 IMU、键盘航向或其他外部姿态输入。
场地中心为原点，x 向右、y 向上，单位为米。标签 ID 1-6 的世界坐标、
朝向和 15 cm 边长定义在 `vision/field_map.json`。

每个标签使用 `SOLVEPNP_IPPE_SQUARE` 生成候选位姿，再按正深度、场地边界、
摄像头安装高度和重投影误差进行纯视觉消歧。多标签结果先选取空间一致簇，
再按标签面积和重投影误差加权融合。结果通过 `Robot.field_pose` 读取。

`vision/tag_camera_calib.json` 当前根据镜头规格的 125 度水平视场角估算内参，
返回结果的 `calibrated` 为 `False`。规格同时给出的 160 度视场角不能代替广角
镜头畸变系数；完成 icSpring 标签摄像头的棋盘格标定后，应写入真实
内参和畸变系数并将 `calibrated` 改为 `true`。同时需要实测
`vision/field_map.json` 中的摄像头高度、相对机器人参考点偏移和安装偏航角。
当前标签相机实测离地高度为 0.25 m；标签位置与朝向沿用参考目录中的场地表，
正式使用前必须按实际场地复核。
标签中心分为两组：Tag3、Tag4 为 0.125 m；Tag1、Tag2、Tag5、Tag6 为
0.325 m。定位器优先使用标签自身高度，未配置时回退到全局值。由于 125 度
广角镜头尚无真实畸变系数，低位 Tag3、Tag4 的垂直 PnP 高度仅用于宽松候选
检查，其高度误差门限单独设为 0.35 m。Tag6 在新广角估算内参下也会产生约
0.20 m 的垂直 PnP 偏差，因此同样使用 0.35 m 门限；Tag1、Tag2、Tag5 也使用
标签级高度容错，重投影误差门限分别为 4.5、4.0、4.0 px。Tag3、Tag4、Tag6
使用 4.5 px 门限。候选仍必须通过对应标签的重投影误差限制，防止放宽高度检查后
接受错误姿态解。

不启动底盘即可独立测试定位：

```bash
python vision/field_localizer.py --camera tag --duration 15
python tools/vision_subsystems_test.py --duration 10
```

正式比赛程序默认同时启动 `cube` 方块识别和 `tag` 全场定位；临时排障可使用
`python main.py --no-field-localization` 禁用全场定位。

## 比赛任务 1

### 方块识别目标跟踪

Task1 的 `cube` 相机已经固定对准识别区域，方块识别采用“检测候选 → 目标锁定 →
对准跟踪”的分层流程。`vision/cube_detector.py` 只负责逐帧产生候选方块；
`Strategy/cube_tracker.py` 的 `CubeTargetTracker` 负责连续确认、X/Z 跳变拒绝、
短暂丢帧保持锁定、候选歧义停车和位置平滑。搜索与视觉对准共用该生命周期，
不会因为最近轮廓变化而切换到另一块橙色方块。抓取完成后由现有
`reset_vision_filter()` 清理相机滤波；通信协议和 A 板行为未改变。

Task1 的稳定导入入口为 `Strategy/task1.py`，两轮复用同一个状态机，现有实现保留在
`Strategy/competition.py` 以兼容已经部署的脚本。单独运行 Task1 时使用
`main.py --task task1-r1` 或 `main.py --task task1-r2`。

完整任务先由 `Strategy/task0.py` 以 0.6 m/s 前进 1200 mm；这一步不属于
Task1，单独运行 Task1 时不会执行。`Strategy/competition.py` 的 Task1 从
0.2 m/s 低速前进开始，并依据至少三个底盘电机持续低速且高电流确认撞墙；确认机器人以 `0°` 航向顶墙后，将当前陀螺仪读数重新定义为后续航向零点。随后以 300 mm/s 持续向右搜索橙色方块，视觉每 20 ms 检查一次，发现目标立即停车，不再使用短距离分段位置环；累计搜索上限为 1500 mm。发现后锁定首次目标，不把摄像头 X 当作绝对移动距离，而是由带速度斜坡的视觉 PI 持续输出横移速度（100-250 mm/s），以 x=0 mm 为控制目标，同时为机械限制保留非对称容差，连续三个新视觉帧进入 [-20, 5] mm 窗口即可完成对准。每次抓取前再以 150 mm/s 向前顶墙，最长 1 秒，复用三电机低速高电流堵转判定，并在确认后重新标定陀螺仪零点，再执行 Grap3。目标丢失时停车并恢复向右搜索；每次抓取后清空视觉滤波，完成 3 个方块后任务结束。

比赛预检完成、底盘尚未运动时，将陀螺仪航向记录为 `0°` 基准，并在后续 `0°` 向前顶墙后更新该基准。三个方块抓取阶段开始前记录四轮多圈编码器，抓取完成后通过麦轮横移投影得到包含搜索、视觉 PID 和贴墙漂移在内的实测净横移。随后执行投放路线：以 300 mm/s 后退 400 mm，通过陀螺仪 PID 对准当前零点右转 `90°` 的绝对航向，到位后闭环维持航向 500 ms，再以 600 mm/s 前进 `Task1 标定基准 - 编码器实测净右移距离`；当前标定基准为 2800 mm。前进完成后直接通过陀螺仪 PID 对准当前零点右转 `180°` 的绝对航向，随后识别 6 号 AprilTag。标签视觉 PID 按 125° 水平视场角模型调整垂直距离至 425 mm 并将横向位置调整至零，旋转轴仅由陀螺仪保持右转 `180°` 的绝对航向。标签相机固定使用短曝光和适中增益以降低运动模糊；对准只接受启动后的新视觉帧，距离或横向位姿发生超过约 75 mm 的不可能跳变时立即停车，连续 3 个彼此一致的新位姿可安全重建基准。PID 使用最近 5 个有效帧的距离和横向中值，抑制单帧 PnP 抖动。首次进入距离 425±7.2 mm、横向偏差约 ±6.0 mm、陀螺仪航向偏差 ±2.4° 的允许范围后，PID 将细调输出放大 1.5 倍并最多继续精调 2 秒，向约 1.5 mm、1.5 mm、0.5° 的小死区收敛；短暂越界不会重置精调计时，满 2 秒后在下一帧回到允许范围时完成。在小死区连续确认 4 个新帧则立即完成。之后以 200 mm/s 前进至堵转确认墙面，打开双舱门，以 300 mm/s 后退 300 mm，关闭双舱门，再通过陀螺仪闭环右转 `180°`，回到当前零点的绝对航向；转向过程中连续累计相邻陀螺仪帧的角度变化，避免跨越 `±180°` 时产生整圈假误差，到位后保持航向 500 ms，再明确停止底盘。标签丢失时立即停车并重置精调，超过 1 秒或总对准超过 12 秒则进入故障急停。

投放路线启用快速衔接：直线动作使用 200 ms 起步斜坡，到位后保留约 50 ms 稳定确认但取消额外保持；转向保留一个控制周期（约 20 ms）的 IMU 到位确认并取消额外保持。顶墙确认和抓取后的额外等待当前为 0 秒，舱门舵机的实际开关时间仍保留。

上段投放流程中的“保持 500 ms”属于历史描述，当前代码以
`delivery_turn_heading_hold_ms=0` 和 `unload_final_heading_hold_ms=0` 为准；不要
据此恢复额外转向等待。

第二轮 Task1 将前进补偿基准由第一轮的 2800 mm 改为 2500 mm。Tag6 对准后，
第一轮和第二轮均以 300 mm/s 向右平移：第一轮 100 mm、第二轮 400 mm；
卸载完成后、最终右转 180° 前，两轮均以 300 mm/s 向左平移：第一轮 100 mm、
第二轮 400 mm。

堵转阈值、搜索步长、最大搜索距离、视觉置信度、目标锁定跳变、对准容差和各阶段超时集中定义在 `FirstTaskConfig`。实车调试只调整该配置，不在任务流程中散落参数。定距动作超时但编码器进度达到至少 90% 时，策略层接受该结果并继续；被取消、遥测丢失或进度不足仍立即报错。顶墙统一维护两套参数：抓取前近距离顶墙使用 `near_wall_speed_mm_s=150`、`near_wall_timeout_s=1`；Task1 常规顶墙、卸载顶墙以及 Task2 路线顶墙均使用 `far_wall_speed_mm_s=200`、`far_wall_timeout_s=4`。所有顶墙动作达到时间上限后都会先停止底盘，再默认按顶墙成功继续流程；遥测丢失或通信异常仍会故障急停。将 `wall_timeout_is_success` 设为 `False` 可恢复严格的超时报错模式。

## 比赛任务 2

Task2 使用独立的 `Strategy/task2.py` 和 `task2_main.py`，不会执行 Task1 任务
流程；底盘距离控制、陀螺仪转向、标签 PID 和堵转检测复用已经验证的公共实现。
Task2 默认同时按稳定角色 `cube` 启动方块相机、按稳定角色 `tag` 启动标签相机，
不依赖 `/dev/videoN` 的插入顺序；需要排障时可分别使用 `--camera` 和
`--tag-camera` 覆盖。
当前实现的第一段流程为：启动时记录陀螺仪零点，以 600 mm/s 前进 2350 mm，
左转 90°，识别并对准 3 号标签，将垂直距离调整到 250 mm、横向位置调整到零，
同时由陀螺仪保持左转 90° 的绝对航向；对准后先以 300 mm/s 向右横移 100 mm，
随后以 300 mm/s 前进 250 mm，再以
200 mm/s 前进，检测到底盘堵转后确认顶墙。顶墙后以 300 mm/s 向左连续搜索
紫色方块，发现目标后通过视觉 PID 将横向位置调整到 `x=0 mm`，连续 3 个新帧
进入 `[-5, 5] mm` 允许范围后再次短压墙，执行 Grap2 抓取一个紫色方块。
紫色搜索的累计向左距离上限为 600 mm；到达上限仍未发现目标时，
停止搜索并跳过 Grap2，但仍使用编码器实测的搜索横移量完成后续距离补偿。
Task2 相邻橙色方块跟踪增加 18 mm 候选歧义间隔；无法明确区分相邻目标时保持停车，不盲目切换目标。
正常抓到紫色方块时后续抓取 2 个橙色方块；放弃紫色方块时改为抓取 3 个。
紫色抓取或放弃搜索后，以 300 mm/s 后退 100 mm，通过陀螺仪闭环右转 90° 回到启动零点的绝对
航向，再以 400 mm/s 前进 `400 mm - 编码器实测净横移`（向左为负，因此等效
于增加实测净左移距离），最后以 200 mm/s
向左移动至堵转确认侧墙，再以 200 mm/s 前进至堵转确认前墙并停车。搜索范围、
目标丢失停车与重新搜索、视觉帧新鲜度和超时保护均复用 Task1 的实现。
Task2 的紫色与橙色抓取阶段也分别记录四轮编码器起点和终点，后续距离补偿使用
包含搜索和视觉 PID 在内的实测净横移；时间积分距离只用于限制最大搜索范围。
完成左、前两侧顶墙后，以前墙约束重新标定当前 `0°` 航向，再清空横移累计并以 300 mm/s 向右连续搜索 2 或 3 个橙色方块；每个橙色方块抓取前再次向前顶墙，并重新标定航向零点；
每个方块均按 Task1 的流程通过视觉 PID 对准 `x=0 mm`，连续 3 个新帧进入
`[-20, 5] mm` 允许范围后短压墙并执行 Grap1，目标丢失时停车并恢复向右搜索。
目标数量的橙色方块抓取完成后以 300 mm/s 后退 500 mm，再以 300 mm/s 向右横移
第一轮按 `800 mm - 编码器实测净右移距离` 补偿，第二轮将总里程基准改为
600 mm；计算结果为正时向右补偿，为负时取绝对值向左补偿，
为零时跳过横移。横移完成后不再识别或对准 Tag4，直接通过陀螺仪闭环右转
180°。转向完成后以 600 mm/s 前进 2100 mm，重置标签定位
滤波并识别 Tag6，使用宽角相机专用容错参数，将距离调整到 425 mm、横向
位置调整到零，同时由陀螺仪保持右转 180° 的绝对航向。Tag6 对准后、执行
Build 前，使用 cube 摄像头从斜上方识别三层橙色建筑，并以轮廓上边缘作为 X/Z 对准参照，避免下半部分遮挡造成偏移；首次稳定锁定时采集当前摄像头数据作为本次 Z 对准目标；通过轮廓高宽比
`0.35-2.20` 适应完整路线后观察角度造成的轮廓变化，并排除明显扁平的橙色轮廓；置信度门槛为 35%，
视觉结果允许保留 0.7 秒，目标丢失保持时间为 3 秒。以当前连续 8 秒复测的中位位置
以当前配置目标 `X=0.0 mm、Z=75.0 mm` 作为当前二维平移 PID 收敛目标；对准先执行左右横移，横向进入容差后再执行前后移动。连续 3 个新帧进入横向
`±3 mm`、前向 `±6 mm`、航向 `±2.4°` 的允许范围后执行 Build。目标、容差、
PID、限速和超时均集中在 `Task2Config` 的 `building_*` 参数中，供后续精调。
建筑平移 PID 的最低线速度保持为 100 mm/s，以克服小位移横移时的底盘静摩擦；最大前进和横移
速度为 250 mm/s；目标先经过 2 个连续帧锁定，锁定后只接受横向跳变不超过 90 mm、
前后跳变不超过 140 mm 的同一建筑轮廓，前后和横向比例增益分别为 1.5 和 1.8；
平移加速度为 500 mm/s²，最低有效平移速度保持为 100 mm/s；只要误差超出死区，
控制输出就不会低于该最低速度。
若建筑目标丢失或视觉对准超时，程序会先停车并输出警告，然后继续执行 Build；
底盘通信等其他异常仍按故障处理。
Build 完成后以 300 mm/s 后退 200 mm，陀螺仪闭环右转至启动零点顺时针
270° 航向，再以 600 mm/s 前进 2200 mm。随后重置标签定位滤波并识别 Tag1，
使用 Tag1 独立的近距离容差，将距离调整到 200 mm、横向误差控制在 10 mm 内并保持
270° 航向；Tag1 使用 0.35 m 高度误差和 4.5 px 重投影误差门限，以适配当前未标定
广角标签相机在近距离的 PnP 偏差。Tag1 视觉帧允许 0.7 秒新鲜度，短暂丢失时底盘
保持停止并等待最多 2.5 秒重新捕获，重新捕获后才恢复 PID；持续丢失仍进入故障急停。
对准后再次右转 90°，回到启动零点的 360°（等价于 0°）航向。
上述 Build 后路线仅在第一轮执行。第二轮 Task2 不执行 Tag3 对准后的 100 mm
向右平移，也不执行返回后的向左顶墙，但仍保留随后向前顶墙和航向重新标定。
第一轮和第二轮在 Tag6 对准后均先以 300 mm/s 向右平移，第一轮 100 mm、
第二轮 400 mm，再进行建筑视觉对准和 Build；Build 完成后直接结束第二轮，
不再后退、转向、前进或对准 Tag1。

可在不连接底盘的情况下复测建筑视觉位置：

```bash
.venv/bin/python tools/building_vision_probe.py --camera cube --duration 5
```

只执行 Task2 硬件预检，不发送运动命令：

```bash
cd /home/uniforest/Uniforest/RaspberryPi
.venv/bin/python task2_main.py --preflight-only
```

运行当前完整 Task2 流程（会驱动机器人）：

```bash
cd /home/uniforest/Uniforest/RaspberryPi
.venv/bin/python task2_main.py
```

`task2_main.py` 保留为第一轮 Task2 的独立入口及无运动预检入口。轮次明确的
调试统一使用 `main.py --task task2-r1` 或 `main.py --task task2-r2`；完整比赛
使用 `main.py`（默认 `--task all`）。
