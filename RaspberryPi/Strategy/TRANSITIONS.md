# 动作衔接与现场配置

日期：2026-10-01。本地代码已接入 PlanA/PlanB；尚未部署、烧录或实机验证。
本文件描述当前实现，整体架构见 [README.md](README.md)。

## 当前执行行为

| 衔接 | 执行过程 | 启用条件 |
| --- | --- | --- |
| 橙块抓取→下一橙块 | 完整抬升→右盲移→复位新帧→实测速度接管搜索/对准；不重复搜块 | 本轮抓取标定含 next_cube |
| 最后一抓→退离 | 收臂期间先走原后退段；少块时按实际退距返回补抓；第三槽跳过也支持 | 本轮抓取标定含 last_departure=true |
| 紫块抓取→找橙路线 | 完整抬升后在机构监督下转场，收臂与路线共同完成 | 本轮 grap2 标定含 departure=true |
| 数量检查→运输 | 计数后直线后退与复位并行，复位完成再走剩余路线 | 实际 Robot 检查会话接口，沿用原直线后退参数 |
| Build→返程 | 第三块释放后返程，下一次装载前等机构完成 | 原 Build 里程碑及匹配的返程动作 |
| 路线内平移/转向 | 曲线跟踪位置和航向，中间航点不强制停，终点确认停车 | 为指定轮次/路线配置曲线 |

新增抓取优化和曲线默认关闭。`transition_config.example.json` 是可读取的空配置；
先在副本中记录现场参数，再逐项启用。没有新依赖。

```bash
# 只读检查：不打开串口/相机，不执行动作
python main.py --strategy PlanA --transition-config Strategy/transition_config.example.json --show-plan
python main.py --strategy PlanB --transition-config Strategy/transition_config.example.json --show-plan
```

实机执行沿用同一命令并去掉 `--show-plan`，配置路径换成已完成现场验证的文件。
可用 `--flow collect-orange-1` 或 `--flow collect-mixed-1` 限定验证范围；
独立搭建入口仍要求物理摆位和已知航向零点。先按项目的构建、烧录和小范围测试顺序验证。

## 配置结构

JSON 顶层仅接受 `version`（1）、`firmware_full_lift_validated`（布尔）、`firmware_id`（字符串）、
`pickups`（对象）、`curves`（对象）。错误字段、标定与路线不匹配、无效数值会在构造 Robot 前拒绝。

启用抓取项须声明已验证的完整抬升固件能力，并填写现场使用的固件版本标识。
这个标识是操作者记录，当前串口协议没有能力协商；必须与实际烧录版本核对。
每条抓取/曲线记录须含 `validated: true`、`verified_on: YYYY-MM-DD` 和非空 `notes`。
notes 记录负载、适用轮次、测试入口和现场结果；只填写开关无法替代实际验证。

### 抓取条目

`pickups` 的键：`ground-1/grap3`、`ground-2/grap3`、`ground-3/grap3`、
`highland-1/grap1`、`highland-2/grap1`、`highland-1/grap2`、`highland-2/grap2`。

每条记录除验证信息外包含：

| 字段 | 含义 |
| --- | --- |
| arm_restore_s | 观察到 DONE 后到相机可按现有标定使用的等待时间，秒，须现场测量 |
| next_cube | 可选对象，含 blind 和 acquire 两套参数；仅适用橙块 |
| last_departure | 可选布尔，最后一个有效橙块开始提前后退 |
| departure | 可选布尔，仅适用紫块后的转场 |

`next_cube.blind` 对应 `execution/blind.py:BlindMotionProfile`，所有数值均由现场提供：

| 字段 | 单位/语义 |
| --- | --- |
| direction | 当前橙块流程要求 +1，向机器人右侧 |
| cruise_speed_mm_s | 盲移巡航速度，mm/s |
| acceleration_mm_s2、braking_mm_s2 | 加速与制动能力，mm/s² |
| max_distance_mm、max_duration_s | 本次盲移的最大位移和持续时间，mm、秒 |
| braking_margin_mm | 额外制动空间，mm，小于最大位移 |
| telemetry_timeout_s、frame_timeout_s | 遥测和采集帧的最大年龄，秒 |
| tick_s、max_command_delay_s | 控制周期与允许发送延迟，秒 |
| validated | 必须为 true |

`next_cube.acquire` 对应 `execution/pickup_motion.py:PickupMotionProfile`。
字段与 blind 相同，去掉 `direction`、`cruise_speed_mm_s`，增加 `settled_speed_mm_s`。
搜索速度沿用该轮标定，`settled_speed_mm_s` 是确认目标对准时允许的实测横向速度。
这里 `max_distance_mm` 从整个采集阶段原点计算，包含前面搜块和盲移；`max_duration_s`
限制本次视觉接管。原采集阶段的累计距离与时间上限同时生效。

前臂复位目前依靠标定时间估计，缺少真实角度传感器。相机仅发布同一姿态代次的复位后新帧。
约 100 mm 只是讨论中的盲移目标，没有写成已验证默认距离；吸附牢固程度和允许加速度仍需实测。

### 曲线条目

`curves` 的键为 `标定配置/路线名`，例如 `building-1/build_return`。
只接受 factory.ROUTE_PROFILES 中的合法配对，支持以下 15 个路线族：
depart_a、depart_b、return_orange、ground_to_delivery、purple_to_orange、orange_to_build、
build_return、staged_initial、staged_to_build、staged_return_first、staged_return_final、
ground_tag_offset、ground_delivery_depart、build_offset、unload_depart。

每条记录除验证信息外含 `points` 数组和 `profile`。
局部坐标 x 向前、y 向右，单位 mm；航向采用连续的顺时针角度，单位度。
每点接受 `x_mm/y_mm/yaw_deg` 偏移及 `dx_scale/dy_scale/dyaw_scale` 比例，缺省为 0。
实际坐标为“偏移 + 比例 × 本轮实时算出的路线出口”。这样保留编码器横移补偿及已后退标志。
首点必须为全 0，可写 `{}`；末点必须为
`{"dx_scale":1,"dy_scale":1,"dyaw_scale":1}`，禁止更改路线出口。至少两个点。

`profile` 对应 `control/trajectory.py:TrajectoryProfile`，全部字段必填：

| 字段 | 单位/语义 |
| --- | --- |
| validated | 必须为 true |
| max_speed_mm_s、max_accel_mm_s2 | 机体平移速度与指令加速度上限 |
| max_yaw_speed_deg_s、max_yaw_accel_deg_s2 | 航向速度与指令角加速度上限 |
| max_wheel_rpm | 单轮转速上限，1～32767 RPM，实际机械上限需实测 |
| position_gain_s、yaw_gain_s | 位置/航向反馈增益，1/s |
| position_tolerance_mm、yaw_tolerance_deg | 终点位置与航向容差 |
| settle_speed_mm_s、settle_yaw_speed_deg_s、settle_time_s | 终点低速阈值与持续时间 |
| max_tracking_error_mm、max_tracking_yaw_error_deg | 允许跟踪误差，须大于终点容差 |
| timeout_s | 整条曲线超时，秒 |
| control_period_s、max_telemetry_age_s | 控制周期和遥测时效上限，秒 |

轨迹使用三次 Hermite 插值，可能离开航点连线围成的范围，现场需验证整个车体/机构扫过区域。
接墙、重标和装载等边界保留；同一路线内航点连续，终点及未注册动作组合仍停车。
轨迹没有对手避让、障碍感知或最优路径求解，实际速度收益尚未测量。

## 故障与验证

盲移到边界、相机未及时恢复或接管丢目标：停止并在原搜索预算内使用已有恢复流程。
通信中断、陈旧遥测、动作失败、错误会话/姿态代次或取消：中止当前执行，禁止自动续跑。
少块回补只处理有效数量检查的结果；目前的图像未知结果继续沿用原比赛降级流程。
一次机构动作完成仍不能证明单块实际吸取成功。

```bash
python tools/check.py
python -m unittest discover -s tests -p "test_*.py"
```

测试包含真实注册表段替换、所有橙块轮次、紫块转场、跳过第三槽、提前离场后少块返回、
检查锁与复位异常、累计搜索边界、连续轨迹速度及遥测故障。测试中的运动参数是合成参数，
不能复制成比赛标定；无硬件测试不测物理净空、吸附能力或实际制动距离。

本轮协议未变：底盘 0x10/8 B、动作启动 0x50/6 B、动作状态 0x83/11 B、遥测 0x80/80 B，
数值字段大端、CRC 小端，A 板 200 ms 失联急停保持。沿用前一阶段的完整抬升状态 6 和 StopAll 清目标修改，
本轮没有新增固件动作表改动。动态比赛策略、世界状态持久化与异常后智能重开仍待实现。
