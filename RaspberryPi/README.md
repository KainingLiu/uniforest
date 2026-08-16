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

依赖方向固定为：`main.py` → `Strategy/` → `robot.py` → `control/`、`protocol/`、`vision/`。比赛代码不得导入 `tools/` 或 `tests/`，硬件模块也不得反向依赖比赛策略。

## 开发约定

当前实现只以本目录和 `../Uniforest_A/` 为准。工作区根目录的 `../备份/` 保存每日成果快照，默认只读，不应在其中继续开发。

树莓派用户名为 `uniforest`；密码向队内管理员获取，不写入代码、配置、日志或项目文档。修改通信协议时，必须同步核对 `../Uniforest_A/` 中的下位机实现，并更新双方文档与测试。

## 底盘位置环

直线位置环由上、下位机协同完成：A 板以 1 kHz 运行四轮速度环并上报多圈累计编码器位置；树莓派以遥测频率运行位置外环、S 曲线速度规划和 IMU 航向保持。`control/chassis.py` 中的 `move_forward()`、`move_right()` 支持正负距离，并返回 `LinearMoveResult` 供调试记录。

直线动作的结束条件同时检查位置误差和四轮转速。进入目标区后关闭速度前馈，只保留位置 PID 锁定；位置或轮速再次超限会重新开始稳定计时。连续稳定 700 ms 后才切换为零速闭环并返回。

正式主程序与临时测试共用 `control/chassis.py` 中的同一套位置环参数。默认距离移动速度为 500 mm/s（0.5 m/s）。交互模式可使用 `move forward|backward|left|right MM [MM/S]`，例如 `move right 1000`。

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

`main.py` 默认在同一次硬件连接中依次运行 Task1 和 Task2。也可显式选择完整
流程或单独任务：

```bash
.venv/bin/python main.py --task all
.venv/bin/python main.py --task task1
.venv/bin/python main.py --task task2
```

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
0.20 m 的垂直 PnP 偏差，因此同样使用 0.35 m 门限；Tag1、Tag2、Tag5 使用
全局 0.20 m 门限。候选仍必须通过 3 px 重投影误差限制，防止放宽高度检查后
接受错误姿态解。

不启动底盘即可独立测试定位：

```bash
python vision/field_localizer.py --camera tag --duration 15
python tools/vision_subsystems_test.py --duration 10
```

正式比赛程序默认同时启动 `cube` 方块识别和 `tag` 全场定位；临时排障可使用
`python main.py --no-field-localization` 禁用全场定位。

## 比赛任务 1

Task1 的稳定导入入口为 `Strategy/task1.py`，现有实现保留在
`Strategy/competition.py` 以兼容已经部署的脚本。单独运行 Task1 时使用
`main.py --task task1`。

`Strategy/competition.py` 当前实现第一项任务：以 0.5 m/s 前进 1200 mm，到位确认后不执行额外保持，立即以 0.15 m/s 低速前进，并依据至少三个底盘电机持续低速且高电流确认撞墙；确认机器人以 `0°` 航向顶墙后，将当前陀螺仪读数重新定义为后续航向零点。随后以 180 mm/s 持续向右搜索橙色方块，视觉每 20 ms 检查一次，发现目标立即停车，不再使用短距离分段位置环；累计搜索上限为 1500 mm。发现后锁定首次目标，不把摄像头 X 当作绝对移动距离，而是由带速度斜坡的视觉 PI 持续输出横移速度（100-250 mm/s），以 x=0 mm 为控制目标，同时为机械限制保留非对称容差，连续三个新视觉帧进入 [-20, 5] mm 窗口即可完成对准。每次抓取前再以 150 mm/s 短时向前顶墙，复用三电机低速高电流堵转判定，并在确认后重新标定陀螺仪零点，再执行 Grap3。目标丢失时停车并恢复向右搜索；每次抓取后清空视觉滤波，完成 3 个方块后任务结束。

比赛预检完成、底盘尚未运动时，将陀螺仪航向记录为 `0°` 基准，并在后续 `0°` 向前顶墙后更新该基准。三个方块抓取阶段开始前记录四轮多圈编码器，抓取完成后通过麦轮横移投影得到包含搜索、视觉 PID 和贴墙漂移在内的实测净横移。随后执行投放路线：以 300 mm/s 后退 400 mm，通过陀螺仪 PID 对准当前零点右转 `90°` 的绝对航向，到位后闭环维持航向 500 ms，再以 500 mm/s 前进 `Task1 标定基准 - 编码器实测净右移距离`；当前标定基准为 2800 mm。前进完成后直接通过陀螺仪 PID 对准当前零点右转 `180°` 的绝对航向，随后识别 6 号 AprilTag。标签视觉 PID 按 125° 水平视场角模型调整垂直距离至 425 mm 并将横向位置调整至零，旋转轴仅由陀螺仪保持右转 `180°` 的绝对航向。标签相机固定使用短曝光和适中增益以降低运动模糊；对准只接受启动后的新视觉帧，距离或横向位姿发生超过约 75 mm 的不可能跳变时立即停车，连续 3 个彼此一致的新位姿可安全重建基准。PID 使用最近 5 个有效帧的距离和横向中值，抑制单帧 PnP 抖动。首次进入距离 425±7.2 mm、横向偏差约 ±6.0 mm、陀螺仪航向偏差 ±2.4° 的允许范围后，PID 将细调输出放大 1.5 倍并最多继续精调 2 秒，向约 1.5 mm、1.5 mm、0.5° 的小死区收敛；短暂越界不会重置精调计时，满 2 秒后在下一帧回到允许范围时完成。在小死区连续确认 4 个新帧则立即完成。之后以 200 mm/s 前进至堵转确认墙面，打开双舱门，以 300 mm/s 后退 300 mm，关闭双舱门，再通过陀螺仪闭环右转 `180°`，回到当前零点的绝对航向；转向过程中连续累计相邻陀螺仪帧的角度变化，避免跨越 `±180°` 时产生整圈假误差，到位后保持航向 500 ms，再明确停止底盘。标签丢失时立即停车并重置精调，超过 1 秒或总对准超过 12 秒则进入故障急停。

投放路线启用快速衔接：直线动作使用 200 ms 起步斜坡，到位后保留约 50 ms 稳定确认但取消额外 700 ms 保持；转向保留一个控制周期（约 20 ms）的 IMU 到位确认并取消额外 500 ms 保持。舱门舵机的实际开关时间仍保留。

堵转阈值、搜索步长、最大搜索距离、视觉置信度、重捕获范围、对准容差和各阶段超时集中定义在 `FirstTaskConfig`。实车调试只调整该配置，不在任务流程中散落参数。Task1 常规顶墙、卸载顶墙以及 Task2 顶墙的时间上限均为 4 秒，抓取前短压墙保持 1.5 秒。所有顶墙动作达到时间上限后都会先停止底盘，再默认按顶墙成功继续流程；遥测丢失或通信异常仍会故障急停。将 `wall_timeout_is_success` 设为 `False` 可恢复严格的超时报错模式。

## 比赛任务 2

Task2 使用独立的 `Strategy/task2.py` 和 `task2_main.py`，不会执行 Task1 任务
流程；底盘距离控制、陀螺仪转向、标签 PID 和堵转检测复用已经验证的公共实现。
Task2 默认同时按稳定角色 `cube` 启动方块相机、按稳定角色 `tag` 启动标签相机，
不依赖 `/dev/videoN` 的插入顺序；需要排障时可分别使用 `--camera` 和
`--tag-camera` 覆盖。
当前实现的第一段流程为：启动时记录陀螺仪零点，以 500 mm/s 前进 2350 mm，
左转 90°，识别并对准 3 号标签，将垂直距离调整到 250 mm、横向位置调整到零，
同时由陀螺仪保持左转 90° 的绝对航向；随后以 300 mm/s 前进 250 mm，再以
150 mm/s 前进，检测到底盘堵转后确认顶墙。顶墙后以 180 mm/s 向左连续搜索
紫色方块，发现目标后通过视觉 PID 将横向位置调整到 `x=0 mm`，连续 3 个新帧
进入 `[-5, 5] mm` 允许范围后再次短压墙，执行 Grap2 抓取一个紫色方块。抓取
完成后以 300 mm/s 后退 100 mm，通过陀螺仪闭环右转 90° 回到启动零点的绝对
航向，再以 400 mm/s 前进 `400 mm - 编码器实测净横移`（向左为负，因此等效
于增加实测净左移距离），最后以 200 mm/s
向左移动至堵转确认侧墙，再以 200 mm/s 前进至堵转确认前墙并停车。搜索范围、
目标丢失停车与重新搜索、视觉帧新鲜度和超时保护均复用 Task1 的实现。
Task2 的紫色与橙色抓取阶段也分别记录四轮编码器起点和终点，后续距离补偿使用
包含搜索和视觉 PID 在内的实测净横移；时间积分距离只用于限制最大搜索范围。
完成左、前两侧顶墙后，以前墙约束重新标定当前 `0°` 航向，再清空横移累计并以 180 mm/s 向右连续搜索两个橙色方块；每个橙色方块抓取前再次向前顶墙，并重新标定航向零点；
每个方块均按 Task1 的流程通过视觉 PID 对准 `x=0 mm`，连续 3 个新帧进入
`[-20, 5] mm` 允许范围后短压墙并执行 Grap1，目标丢失时停车并恢复向右搜索。
两个方块抓取完成后以 300 mm/s 后退 500 mm，再以 300 mm/s 向右横移
`800 mm - 编码器实测净右移距离`；若计算结果不大于零则故障停车，不发送
反向距离命令。横移完成后重置标签定位滤波，识别 Tag4；视觉 PID 将距离调整到
300 mm、横向位置调整到零，陀螺仪保持启动零点绝对航向。对准完成后通过
陀螺仪闭环右转 180°。转向完成后以 500 mm/s 前进 2100 mm，重置标签定位
滤波并识别 Tag6，完全复用 Task1 的 Tag6 参数，将距离调整到 425 mm、横向
位置调整到零，同时由陀螺仪保持右转 180° 的绝对航向；对准成功后执行 Build。

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

Task2 后续动作继续添加到该独立状态机和独立测试中。单独运行 Task2 可使用
`main.py --task task2` 或保留的 `task2_main.py`；完整比赛使用 `main.py`（默认
`--task all`）。
