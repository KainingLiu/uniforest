# 可插拔运动规划

2026-10-02：PlanA/PlanB 的原路线终点现由完整位置间规划器执行。开关默认关闭；
`--enable-motion-planning` 开启，`--disable-motion-planning` / `--classic-motion` 关闭。
部署和实机状态见 [CHANGELOG](../../CHANGELOG.md)。

## 原目标与新的规划器

默认调用链：`ActionEnvironment.run_route → MotionPlanning → RecipeRoutes → FieldPlanner.plan_between
→ CompetitionMotion/route_timing → RobotNavigation.execute → PositionTracker → 轮速接口`。
沿用原 `ROUTES` 的距离、终点朝向、搜块编码器补偿和完成标志。原中间移动/转向点仅用于计算终点，
不传给路径搜索。19 类具名路线都经该入口；接墙、Tag 校正、航向重标和原有净空退离仍是操作边界。
每个边界之后，从当时的实际里程计位置计算后续原代码的相对目标。

规划器比较直达、搜索及圆滑候选，选择预计耗时较短的可行曲线；不保证数学上的全局最优。
曲率、墙面余量、制动、轮速和转向约束由同一套速度规划处理，执行器使用完整位置 PID 与加加速度限制。
`LocalRoutes`、`CalibratedCurves` 保留为显式注入的独立后端，普通比赛开关不再选择它们。
加载旧 `curves` 参数不会绕过完整规划器。

```bash
python main.py --strategy PlanA --enable-motion-planning --show-plan
python main.py --strategy PlanB --enable-motion-planning --show-plan
```

预览不创建 Robot；移除 `--show-plan` 才进入硬件启动。默认不要求新增 JSON 或 Tag 标定。
场地图与车体模型直接使用现有 `simulation/field_model.json`、`robot_model.json` 数据；
仿真动作、虚拟传感器和场景中的目标点不进入实机执行。模型保留原有 CAD/未确认当前机械外包的来源标记。
整局启动坐标沿用现有 `start_blue` 约定；`RouteOdometry` 从启动时的编码器基线连续积累所有遥测帧，
包含搜索、抓取顶墙、对准及其它控制器的运动。每次换路线都不重置到理想终点。
独立的中途任务缺少整局起点时，需通过已有的显式视觉导航入口提供当前位置；不假设已在启动区。

当前默认适配器要求每段入口静止且机构空闲；不支持未完成机构动作的全场曲线接管。
旧提前移动开关与其混用时仍执行入口检查，不会静默回退经典路线。
故障、遥测陈旧、重连、A 板重启或急停会中止本轮，通信恢复后不自动继续。
原目标在 CAD 模型内碰撞时直接报告冲突，禁止自动裁剪终点或修改原距离。
当前 PlanA 出发原终点与旧 CAD 外包存在冲突，具体数值和验证边界见 CHANGELOG。

诊断 `motion_backend_selected=field_curve:recipe_endpoints` 表示完整规划器，
`planned_recipe_curve` 记录原目标、实际起点、曲线长度、候选数和模型来源。
`route_planner_started` 记录整局起点与速度/PID 实现。
协议未变：沿用 0x10 轮速、现有遥测、取消与 200 ms 失联急停；没有新增下位机命令。

## 显式视觉导航

`--navigation-config` 仍可选用 `CompetitionRoutes` 的已标定 Tag 起点。
下面的标定要求只适用于该显式入口。默认整局编码器模式不借此伪造标定记录。

## 可选的全场导航配置与执行条件

以下条件只适用于显式传入 `--navigation-config` 的全场导航；默认整局编码器模式不要求该文件或 Tag 标定。

`--navigation-config` JSON 包含 `version: 1`、`verified`、`record`、`capture_delay_s`、`field_file`、`robot_file`。
几何文件路径相对于该 JSON；地图/车体格式与现有规划器一致，坐标为 `field_mm_cw`。
`competition.example.json` 引用名义 CAD，`verified: false`，只允许 `--show-plan`。
现场测量并确认几何、相机外参和采集延迟后才填写验证记录，不能通过改布尔值代替测量。

实机开启前校验配置，运行时要求已标定、有效且延迟修正后不超过 300 ms 的原始 Tag 位姿。
无有效定位、多 Tag 解算不一致、机构忙、非静止入口、起终点碰撞或规划期间位姿变化均在轮速指令前拒绝。
规划或执行失败沿用策略急停，不执行原路线重放，也不自动续跑。协议未变。

覆盖：`depart_a`、`depart_b`、`return_orange`、`ground_to_delivery`、`purple_to_orange`、
`orange_to_build`、`build_return`、`staged_initial`、`staged_to_build`、`staged_return_first`、
`staged_return_final`、`ground_tag_offset`、`ground_delivery_depart`、`build_offset`、`unload_depart`。
`to_purple`、`unload_approach` 等带内部视觉/接墙屏障的路线保留原执行，日志注明 `classic:contact_or_visual_barrier`。
机构并行窗口保留原路线，注明 `classic:mechanism_overlap`；关闭对应衔接后可在机构完成时规划。
预览逐条列出路线覆盖，运行日志 `planned_competition_route` 记录实测起点、计算终点、路径长度及候选数。

接入已通过虚拟底盘测试；当前 Tag 相机仍报告 FOV estimate、未标定，整场实机规划效果尚未验证。

### 注入或移除规划实现

下面是已有机器人生命周期内的接入函数，调用前须完成 Robot 连接、增量会话协商和原有设备准备。
`bindings` 和 `adapter_factory` 必须由实际场地接入代码提供；示例不填入名义坐标或测试标定。
首次接入只绑定机构已经空闲、底盘静止且满足原路线出口契约的路线。

```python
from dataclasses import replace
from Strategy.runner import run_selection
from Strategy.optimizations.motion_planning import (
    MotionPlanning, FieldNavigation, CalibratedCurves,
)

def run_with_navigation(robot, config, bindings, adapter_factory):
    config = replace(config, motion_planning_enabled=True)
    planning = MotionPlanning(optimizers=[
        FieldNavigation(bindings, adapter_factory),
        CalibratedCurves(),
    ])
    return run_selection(robot, 'PlanA', transition_config=config,
                         motion_planning=planning)
```

`bindings` 的键为合法的 `profile/route`，回调 `request(env)` 返回
`(Location, Location, measured_start_pose)` 或 `None`。
`adapter_factory(env)` 返回持有已验证 `NavigationCalibration` 的 `RobotNavigation`。
必须保留原路线出口、上下文锚点及后续操作条件；此接口用于自定义后端。
比赛内置接入使用上文的 `CompetitionRoutes`，无需手写位置族绑定。

列表顺序即选择优先级。移除 `FieldNavigation(...)` 后，保留标定曲线后端；删除某个绑定时，
仅该路线继续尝试后续后端。全部返回 `None` 时使用经典路线。
显式传入 `optimizers=[]` 可仅使用经典路线；配置的 `motion_planning_enabled: false`
具有更高优先级，即使注入规划对象也关闭替换。
准备阶段的错误直接传播，只有返回 `None` 才表示允许尝试下一实现。

`RobotNavigation.prepare/execute` 均检查机构空闲和底盘静止。将它用于仍持有抓取/检查/Build
会话的衔接窗口会被拒绝；当前不支持任意非零速度或未完成机构动作的导航接管。
单条导航内部保持连续，两个独立导航调用之间仍有停车边界。

## 参数来源和速度

位置间导航使用 `navigation.motion.CompetitionMotion`，复用 `Strategy.settings.PROFILES` 和
`control.chassis` 的长短距速度、加速时长、坡道速度、横移补偿、转向、制动距离与到位窗口。
长距离巡航设定默认乘 1.10，即 1000→1100 mm/s；短距 400、坡道 800、搜索/沿墙 300、接近 150 保持来源配置。
倍率允许调整到 1.20，超过时拒绝配置。它不绕过弯道、轮速或制动限制。

旧直线控制器的速度前馈外还有位置 PID 修正，1000 mm/s 设定在当前虚拟底盘内可达到约 1444 mm/s。
新参考速度包络保留对应的有限位置修正预算，并根据平移转向叠加后的每个轮速再次限幅。
轮速上限由经典纯横移的前馈＋修正上限及巡航倍率推导；这是指令上限，不构成电机能力实测结论。
`control.motion_law` 提取原有三次平滑与距离制动函数，原经典执行器数值行为不变。

位置间导航另外有曲率加速度、模型响应补偿、误差余量等新增参数，均明确为待实机确认项。
普通到位复用 8 mm 窗口，并预留 1 mm 模型定位余量；搭建保持 5 mm 和新鲜视觉证据要求。

## PID 跟踪

`navigation.tracking.PositionTracker` 对世界坐标 X/Y 和顺时针航向分别计算 P、I、D 修正，叠加轨迹速度前馈，
再转换为车体速度、轮速。`TrackingGains` 可传给 `FieldPlanner(..., tracking=...)`。
初始平移参数为 Kp=2.5、Ki=0.12、Kd=0.08；航向为 2.5、0.10、0.05，均待现场验证。
Ki 有独立积分区间和限幅；轮速、墙面或加速度约束挡住输出时，撤销推动饱和方向的本次积分。
D 来自参考速度与测量速度之差，滤波时间 60 ms；视觉修正不会被除以采样周期变成微分脉冲。
到位低速保持时清空积分与输出，避免加加速度状态把已停稳的底盘再次推走。
遥测过期、通信中断和急停仍终止本次执行。

## 验证入口

```bash
python -m unittest tests.test_competition_navigation tests.test_motion_planning_plugin tests.test_navigation_motion tests.test_position_tracking -q
python -m simulation.linear_comparison
python -m simulation.navigation_verify
python -m simulation.navigation_server --port 8766
```

直线对照通过同一个四轮模型运行真实 `Chassis._move_linear` 与新版执行器，覆盖不同距离和四个方向。
报告区分控制器差异，不把模型时间当成实机时间；网页可调巡航倍率并显示 PID 误差。
本轮协议未变：0x10/8 B、0x80/80 B、0x83/11 B、数值大端/CRC 小端和 200 ms 失联急停保持。
现有扩展会话、机构动作和协议兼容工作区改动均保留，无新增依赖。
