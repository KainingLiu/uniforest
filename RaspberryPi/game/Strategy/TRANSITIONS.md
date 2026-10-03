# 动作衔接与现场配置

更新：2026-10-03。代码已接入 PlanA/PlanB；部署和固件记录见上位机 [CHANGELOG](../CHANGELOG.md)，
新优化的物理净空与实际提速仍待现场验证。本文件维护配置字段；
启动命令、开关组合与入口限制统一见[运行方式与优化插拔](../README.md#运行方式与优化插拔)，
整体架构见 [README.md](README.md)。

## 当前执行行为

| 衔接 | 执行过程 | 启用条件 |
| --- | --- | --- |
| 橙块抓取→下一橙块 | 完整抬升→右盲移→复位新帧→实测速度接管搜索/对准；不重复搜块 | 本轮抓取标定含 next_cube |
| 最后一抓→退离 | 收臂期间先走原后退段；少块时按实际退距返回补抓；第三槽跳过也支持 | 本轮抓取标定含 last_departure=true |
| 紫块抓取→找橙路线 | 完整抬升后在机构监督下转场，收臂与路线共同完成 | 本轮 grap2 标定含 departure=true |
| 数量检查→运输 | 抓取完成并停稳后，直线后退与低头采样并行；复位完成再走剩余路线 | 实际 Robot 检查会话接口，沿用原直线后退参数 |
| Build→返程 | 第三块释放后返程，下一次装载前等机构完成 | 原 Build 里程碑及匹配的返程动作 |
| 路线内平移/转向 | 曲线跟踪位置和航向，中间航点不强制停，终点确认停车 | 为指定轮次/路线配置曲线 |
| 快速抓取对准 | 一次入窗、提前制动、停止后新帧确认，省去重复精对准 | 为指定轮次/颜色配置 alignments |
| 自适应盲移 | 抓取前观察相邻块，抬升后扣除实际位移，再按原包络限幅 | next_cube.adaptive 与本轮快速橙块对准；见[说明](optimizations/ADAPTIVE_BLIND.md) |

五类跨动作衔接默认全部关闭，曲线在未配置时关闭。`transition_config.example.json` 是可读取的空配置；
先在副本中记录现场参数，再逐项启用。没有新依赖。

表中前三项的参数来自 `pickups`；加载标定或试跑参数后，五类衔接仍默认关闭。
五类跨动作衔接均支持 `--enable-transition 类型` / `--disable-transition 类型`；
类型为 `next-cube`、`last-departure`、`purple-departure`、`inspect-departure`、`build-return`。
加 `@轮次配置` 可限定一轮，例如 `--disable-transition next-cube@ground-1`。
`--enable-transitions` 全开、`--disable-transitions` 全关，再应用类型和轮次选择；整组开关互斥。
轮次选择优先于类型选择，类型选择优先于整组开关，其余默认关闭；同一选择同时开关报错。
开关不改写标定文件；显式启用抓取衔接仍须提供参数，缺失时在创建 Robot 前拒绝。
已有 `--trial-optimizations` 显式试跑入口保留，试跑参数仍标记为未验证；开启衔接还需整组或单项开启参数。
关闭后运行时恢复退出完成、停车、下一动作进入的路径，不跳过抓取或检查复位。
命令及优先级示例见[操作手册](../OPERATIONS.md#按具体衔接启用或关闭)。
抓取内部短顶墙、固件内部步进/舵机并行也没有独立 CLI 开关。

原路线局部平滑由 `motion_planning_enabled` 控制，默认 `false`；不要求地图JSON或Tag起点。
比赛入口须显式传入 `--enable-motion-planning`，
即使 JSON 中为 `true` 也不会自动开启；`--classic-motion` 显式关闭，与开启参数互斥。
这些参数与 `--disable-fast-alignment` 均仅覆盖本次运行，不会修改配置文件。
加载所需配置并指定对应开启参数后重新启动即可启用。
行进Tag6持续修正单独由`moving_tag6_enabled`控制，默认`false`；CLI须同时指定
`--enable-motion-planning --enable-moving-tag6`才挂载模块。`--disable-moving-tag6`保留局部平滑、
原运输目标、原位置Tag对准与后续横移步骤；原Tag对准内部多帧反馈保持。开关仅在本次启动生效。

```bash
# 只读检查：不打开串口/相机，不执行动作
python main.py --strategy PlanA --transition-config Strategy/transition_config.example.json --show-plan
python main.py --strategy PlanB --transition-config Strategy/transition_config.example.json --show-plan
```

实机执行沿用同一命令并去掉 `--show-plan`，配置路径换成已完成现场验证的文件。
可用 `--flow collect-orange-1` 或 `--flow collect-mixed-1` 限定验证范围；
独立搭建入口仍要求物理摆位和已知航向零点。先按项目的构建、烧录和小范围测试顺序验证。

## 配置结构

JSON 顶层仅接受 `version`（1）、`motion_planning_enabled`（布尔，默认 false）、
`firmware_full_lift_validated`（布尔）、
`firmware_id`（字符串）、`pickups`（对象）、`curves`（对象）、`alignments`（可选对象）。
错误字段、标定与路线不匹配、无效数值会在构造 Robot 前拒绝；文件校验早于 CLI 关闭参数应用。

启用抓取项须声明已验证的完整抬升固件能力，并填写现场使用的固件版本标识。
这个标识是操作者的现场记录；新比赛入口还会通过增量通信接口核对版本和能力，
协商成功后才能执行。能力回复不确认物理净空，仍须与实际烧录版本和现场记录核对。
每条抓取/曲线记录须含 `validated: true`、`verified_on: YYYY-MM-DD` 和非空 `notes`。
notes 记录负载、适用轮次、测试入口和现场结果；只填写开关无法替代实际验证。

`alignments` 提供快速抓取对准参数；比赛入口默认关闭，须加 `--enable-fast-alignment`。
未填写的颜色/轮次仍使用原粗、精对准。
它不要求抓取提前放行标志为 true；当前比赛入口原有的增量固件会话要求保持。
`--disable-fast-alignment` 显式关闭，与开启参数互斥；抓取衔接和运动规划各自独立选择。
完整字段、记录格式和验证入口见 [快速对准](optimizations/README.md)。

### 抓取条目

`pickups` 的键：`ground-1/grap3`、`ground-2/grap3`、`ground-3/grap3`、
`highland-1/grap1`、`highland-2/grap1`、`highland-1/grap2`、`highland-2/grap2`。

每条记录除验证信息外包含：

| 字段 | 含义 |
| --- | --- |
| arm_restore_s | 观察到 DONE 后到相机可按现有标定使用的等待时间，秒，须现场测量 |
| next_cube | 可选对象，含 blind 和 acquire 两套参数；仅适用橙块；expected_distance_mm 默认 100；可加 adaptive 自适应预观测 |
| last_departure | 可选布尔，最后一个有效橙块开始提前后退 |
| departure | 可选布尔，仅适用紫块后的转场 |

2026-10-03：按用户要求，抓取后预先右移的**固定期望距离为 100 mm（10 cm）**。
可在 `next_cube` 中显式设置 `"expected_distance_mm": 100`，省略时同样使用 100。
此参数适用于地面三轮、高地两轮橙块连续抓取。内置固定试跑包络为 120 mm（含 20 mm 制动余量）；
自适应连续块排仍取 100 mm，已定位下一块最多 200 mm，无可靠下一块时取 200 mm，试跑包络为 220 mm。
现场 JSON 的显式值和硬上限保持；省略字段时使用新的默认值。新距离尚待现场验证。
执行时将盲移包络限制为“期望距离 + 已标定制动余量”，同时服从原 max_distance_mm 和剩余搜索预算。
因此相机提前恢复可以早于 100 mm 交接，安全上限或时限更紧也会提前退出；交接后的视觉移动另计。
原盲移开关、速度、加速度及制动参数保持，实际位移仍需现场确认。

`next_cube.blind` 对应 `execution/blind.py:BlindMotionProfile`，运动包络数值由现场提供。
方向反馈容差初值为 `feedback_position_tolerance_mm=2`、`feedback_speed_tolerance_mm_s=10`，
均可覆盖（允许 0）；前者最多使用制动余量的一半。这是软件初值，仍需现场确认。
小幅反向反馈不再取消抓取。超出容差且反向制动包络未超出原制动余量时，
局部结束盲移，完成机械臂复位后由下一动作继续视觉对准；超出恢复包络或通信/机构异常仍中止。
每段的 `blind_feedback` 日志保留原始有符号位移/速度和结果，便于区分回弹与持续反向运动。

2026-10-02 显式试跑快速对准窗口调整为 ±10 mm，额外固定误差预留从 2 mm 改为 0；
仍评估运动延迟与制动位置，并要求停稳后两帧确认。入窗执行抓取，出窗继续调整；
已有标定 JSON 不自动改值。协议未变，无下位机修改或自动续跑。

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
当前 100/200 mm 是用户指定的预移期望或上限，尚无现场验证结果；吸附牢固程度和允许加速度仍需实测。

### 后退检查与少块返回（当前源码核对）

`inspect-departure` 开启时优先占用后退段，`last-departure` 不提前消耗同一段。
最后一次抓取完成后，检查会话查询 DONE（零次抓取才允许 IDLE），发送停车，等待三份新遥测确认
步进空闲且四轮绝对转速均不超过 10 RPM；1 秒内不能满足则中止。
随后舵机 0 到 37.2°，启动原直线后退；200 ms 后舵机 1 到 120°，再等待 300 ms，
丢弃前三帧，采集八张独立新帧。退离尚未结束时可采样，退离结束后不足八帧则原地补齐。
地面退离目标 400 mm、高地 100 mm，速度设定均 400 mm/s；判定完成后先发舵机 1 回 90°，
200 ms 后发舵机 0 回 97.2°。这些是现有命令与时间估计，缺少角度反馈。

明确计数 0/1/2 时先完成复位并释放机构锁，再按四轮编码器记录的实际后退距离向前返回，
然后向前靠墙、重标航向并清理视觉滤波，清除已后退标记，补抓 `3-count` 次并重新检查。
搜块原点和累计搜索预算保留；没有记录整段二维返回轨迹，返回也不单独修正横向漂移。
通信、遥测或动作故障中止；地面计数未知沿用继续运输的降级，高地已抓紫块但计数未知会中止。
计数未知不能按 0 块理解；紫色 ROI 规则会把稳定紫色判为 1，该结果不确认紫块与橙块分别剩几块。

当前返回仍使用原 `_checked_move` 的超时进度 >=90% 接受规则；末端靠墙默认 300 mm/s、4 秒，
并继承 `wall_timeout_is_success=True`。靠墙超时也会重标和补抓，因此软件流程可核对，实机稳定返回尚无保证。
本次只调整盲移距离，不改变返回、靠墙或识别降级行为。计数画面底部 30% ROI 与分支说明见
`control/carried_cube_inspection.py`、`vision/carried_cube_count.py`；比赛会话丢弃 observe 的预览图，
默认不弹出带 ROI 的计数窗口，调试工具可生成叠图。

### 历史曲线条目（仅文件兼容与离线工具）

2026-10-03起，以下curves结构继续校验和读取，但不再通过比赛入口选择独立曲线后端；比赛统一使用原路线局部平滑。

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

旧接口保持：底盘 0x10/8 B、动作启动 0x50/6 B、动作状态 0x83/11 B、遥测 0x80/80 B，
数值字段大端、CRC 小端，A 板原 200 ms 失联阈值保持。增强抓取和停止由新增会话接口提供，
旧 0x50 始终执行 main 动作表；见 [兼容说明](../protocol/ADDITIVE_COMPATIBILITY.md)。
动态比赛策略、世界状态持久化与异常后智能重开仍待实现。
