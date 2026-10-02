# 可插拔快速抓取对准

新增[相邻块预观测与自适应盲移](ADAPTIVE_BLIND.md)：复用停车确认帧，在原硬上限内计算下一抓预移量。
通过 `next_cube.adaptive` 或独立试跑参数 `--trial-adaptive-blind` 启用；默认关闭，现场待验证。

独立的[可插拔运动规划与位置 PID](MOTION_PLANNING.md) 默认启用选择器，支持 `--classic-motion` 经典路线回退。
它与本文的抓取对准开关互不影响。

2026-10-02。实现位于 `fast_alignment.py`，通过现有 `--transition-config` 选择。
默认没有启用项，不修改 `vision_targets.py`、原 PID 参数、Tag/建筑定位、抓取动作编号或路线。
用户确认水平误差 ±10 mm 内能稳定抓取；±8 mm 软件窗口是待实测的候选值。

## 行为

普通流程仍先用原搜索锁定方块。开启本模块后，后续粗对准/精对准合并为一次接近：

1. 沿误差方向接近，速度受距离、制动能力、指令延迟及加速度限制；无原有 100 mm/s 最低速度夹紧。
2. 根据图像采集年龄、实测速度和上一条速度指令估计当前位置与停车落点。
3. 预计能停进窗口时开始制动并保持停车，不继续追求零误差。
4. 速度指令归零、实测横向速度足够低、误差及不确定度在机械容差内，且多张独立新帧/遥测都合格才成功。
5. 单个出窗抖动不触发修正；连续确认停车后仍出窗才修正，换向次数和重试次数均有限。

机械容差、软件窗口和误差预留分别配置，必须满足：
`capture_tolerance_mm + position_uncertainty_mm <= mechanical_tolerance_mm`。
2026-10-02 修复：静止后采集的新帧进入 ±10 mm（试跑配置）即确认，不再追求居中；
保留两份独立画面/遥测及低速条件。运动中仍使用保守制动包络，已下发速度不能单独触发到位锁定。
`min_correction_speed_mm_s` 默认 0；试跑复用原起步速度 80 mm/s，
受加速度、最高速度和窗口另一侧边界的停车距离限制，现场效果待验证。
图像年龄还增加基于加速度上限的误差预留。这里的停车预测依赖标定模型，无法确认真实吸附成功。
采集时间来自摄像头 read 完成时刻；缓冲、曝光和几何标定误差需计入 position_uncertainty_mm。

同一个模块也接在“盲移→下一橙块”处，接收实测初速度和相机姿态代次，保留原采集起点和累计搜索预算。
盲移后的两套速度/时效限制共同生效；入口速度须在快速对准速度上限内。
搜索边界只约束未锁定目标的搜索。锁定后按视觉误差双向对准，使用自身行程、时间和制动限制；
对准位移与时间不挪用搜索预算，也不重置此前已消费的搜索额度。
启用快速对准无需同时启用盲移；两项可以独立插拔。

## 启用与回退

在现有衔接 JSON 的顶层增加 `alignments`。合法键：

- `ground-1/orange`、`ground-2/orange`、`ground-3/orange`。
- `highland-1/orange`、`highland-2/orange`。
- `highland-1/purple`、`highland-2/purple`。

只列需要验证的条目。每条记录包含 `validated: true`、`verified_on`（YYYY-MM-DD）、
`notes`（机械状态、适用轮次、入口及现场结果）及 `profile`。
profile 对应 `FastAlignmentProfile`；参数均应按现场数据填写，测试中的合成数值不属于比赛配置。

| profile 字段 | 单位/要求 |
| --- | --- |
| capture_tolerance_mm | 以现有标定目标 X 为中心的接受半宽，mm；候选 ±8 mm |
| mechanical_tolerance_mm | 机械可抓半宽，mm；用户反馈 ±10 mm |
| position_uncertainty_mm | 视觉、标定、缓冲及剩余未建模位置误差预留，mm |
| max_speed_mm_s | 接近与移动搜索速度上限 |
| acceleration_mm_s2、braking_mm_s2 | 速度指令加速限制和经过验证的制动能力 |
| settled_speed_mm_s | 允许确认抓取的实测横向低速阈值，小于接近速度 |
| max_duration_s、max_travel_mm | 单次对准的时间与累计行程上限 |
| frame_timeout_s、telemetry_timeout_s | 图像采集/遥测年龄上限，秒，不能大于 0.5 |
| tick_s | 控制周期，秒，不超过 0.05 且不超过两种时效上限 |
| max_command_delay_s | 允许的速度命令发送耗时及预测延迟，秒 |
| confirm_frames | 合格新帧数量，至少 2；默认 2，重复图像或重复遥测不累计 |
| max_reversals | 单次对准最多换向次数，0～2；默认 1 |
| max_attempts | 一次 acquire_cube 中最多搜到目标后对准尝试次数，1～3；默认 2 |
| validated | true；代表此组参数的显式标定记录，程序无法代替现场测量 |

以下是**填写结构**，尖括号内容必须替换成实际数值/日期后才能通过检查：

```text
"alignments": {
  "ground-1/orange": {
    "validated": true,
    "verified_on": "<现场日期>",
    "notes": "<机械状态、测试入口、次数、成功率和耗时>",
    "profile": {
      "capture_tolerance_mm": <软件窗口>,
      "mechanical_tolerance_mm": <机械窗口>,
      "position_uncertainty_mm": <误差预留>,
      "max_speed_mm_s": <速度>,
      "acceleration_mm_s2": <加速度>,
      "braking_mm_s2": <制动能力>,
      "settled_speed_mm_s": <低速阈值>,
      "max_duration_s": <超时>,
      "max_travel_mm": <最大行程>,
      "frame_timeout_s": <图像时效>,
      "telemetry_timeout_s": <遥测时效>,
      "tick_s": <周期>,
      "max_command_delay_s": <发送延迟>,
      "confirm_frames": 2,
      "max_reversals": 1,
      "max_attempts": 2,
      "validated": true
    }
  }
}
```

预览不连接硬件。将文件名换成自己的标定副本：

```bash
python main.py --flow collect-orange-1 --transition-config Strategy/field-transitions.json --enable-fast-alignment --show-plan
python main.py --flow collect-orange-1 --transition-config Strategy/field-transitions.json --disable-fast-alignment --show-plan
```

第一条显示启用的具体轮次/颜色；第二条显示 `disabled: legacy alignment`，其他优化保留。
比赛入口默认关闭快速对准，加载标定或试跑参数后仍须加 `--enable-fast-alignment`；与关闭参数互斥。
移除 alignments 中的单个条目即可恢复该轮次/颜色的原对准。空配置同样保留原行为。
配置加载在 Robot 创建前完成，错误数值、错误目标和未验证条目都拒绝启动。

## 失败、验证与观测

丢目标、超时、次数或行程预算耗尽返回未对准。普通流程在有限次数内重新搜索；最终失败跳过本次抓取，
盲移接管失败则停止并返回普通搜索。原采集范围和时间预算继续约束恢复。
不会使用旧的“粗对准超时算成功”来放行新模式，也不会跳回旧精对准以绕过新容差。
取消、通信异常、遥测陈旧、A 板重启、相机姿态改变或发送异常直接中止执行，禁止自动续跑。

日志事件 `fast_alignment` 记录动作名、颜色、结果原因、耗时、换向次数及最后停车偏差预测。
现场对照应比较原/新模式的对准耗时、反向次数、抓取成功率及补抓总耗时。
普通路径和盲移后的路径应分别验证，三类抓取姿态也应分别记录。

```bash
python -m unittest tests.test_fast_alignment -q
python tools/check.py
```

无新依赖。本轮协议未变：沿用工作区已有原 schema v3 与增量会话接口，不增加任何命令、载荷或固件改动。
旧底盘速度 0x10/8 B、动作启动 0x50/6 B、原遥测 0x80/80 B、动作状态 0x83/11 B，
增量命令 0x52～0x55、回复 0x84～0x86、数值大端/CRC 小端及 200 ms 失联/会话期限保持。
本模块代码和虚拟底盘验证均不等于实际制动距离、抓取成功率或提速已经验证。
