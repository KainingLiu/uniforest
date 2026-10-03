# 功能动作驱动的比赛程序

2026-10-01：旧编号 Task 程序与任务注册表已移除。PlanA/PlanB 直接展开成功能动作，
整套策略由一个执行运行时完成。最新部署和固件状态见上位机 [CHANGELOG](../CHANGELOG.md)，
现场优化效果仍待验证。启动与各项开关见[运行方式与优化插拔](../README.md#运行方式与优化插拔)。

## 当前调用链

2026-10-03 [运动规划模块](optimizations/MOTION_PLANNING.md)采用唯一局部动作优化链路。
--enable-motion-planning开启；单段移动保持原控制，连续移动/转向保持终点和关键经过区域并平滑衔接。
Tag6持续修正默认关闭，运输平滑到原目标后执行原Tag对准和偏置；额外--enable-moving-tag6才合并末段。
--disable-moving-tag6可独立关闭该模块。建筑视觉仍确认最终位置。无需名义地图起点或启动Tag。
```text
main.py
  → plans：PlanA/PlanB 或独立功能流程，生成 ActionSpec 列表
  → runner.run_plan：预检、机器人互斥、共享执行上下文
  → flows.factory：把每个 ActionSpec 绑定为进入/主体/退出段
  → execution：一次编译整条动作流，逐边选择 T(A,B)
  → flows.operations / routes + controllers / building_alignment
  → Robot / control / protocol → A 板
```

没有 TaskStep、TaskDefinition、TASK_LIBRARY 或 TaskNProgram。旧 task0.py 至 task5.py、
competition.py 已删除。控制算法保留在独立控制模块，那里没有比赛任务 run 方法。

## 模块职责

| 模块 | 职责 |
| --- | --- |
| plans/plan_a.py、plan_b.py | 宏观流程顺序；只构造数据，不调用硬件 |
| plans/recipes.py | 将一轮采集运输等目标展开成可检查的平铺动作；无执行循环 |
| flows/model.py | ActionSpec：动作类型、名称、标定配置、参数、入口/出口锚点 |
| flows/operations.py | 靠墙、采集初始化、搜块、抓取、检查补抓、卸货、装载 |
| flows/refill.py、refill_routes.py | 区域耗尽后的单次跨区补抓、数量复核及保留货物的往返路线 |
| flows/routes.py | 去某区的标定路线及编码器补偿；路线可内部包含多段运动 |
| flows/factory.py、transitions.py | 参数/能力验证、动作绑定、抓取/检查/Build 专门衔接与少块回补 |
| flows/curves.py、control/trajectory.py | 保留路线出口和接墙约束，执行局部连续曲线及平移旋转 |
| optimizations/local_routes.py、tag_approach.py | 原动作局部等价平滑、行进Tag6修正与后续步骤一次性接管 |
| execution/ | 通用 Action/ActionFlow、编译、段替换、取消、盲移与带速度视觉接管 |
| transition_config.py | 加载逐轮抓取与逐路线的现场标定；未提供的项保持原执行方式 |
| controllers.py | 底盘、靠墙、方块/Tag 对准等可复用控制；不决定比赛顺序 |
| building_alignment.py | 建筑视觉几何与对准 |
| settings.py | 标定与轮次配置，PROFILES 中统一命名 |
| context.py | 本次执行的航向、路线锚点、当前动作和硬件连续性；不冒充世界模型 |

## 动作词表

当前包括 navigate、anchor_wall、rebase_heading、begin_collection、acquire_cube、grab_cube、
inspect_cargo、unload、load_staged、align_tag、align_building、build。
宏观策略选择动作数量与顺序；条件橙块槽显式带 conditional_on_purple，保留有紫抓两橙/无紫抓三橙。
其中“已抓紫”沿用机构动作完成语义，尚无单块吸取成功反馈。
同一区域的数量检查补抓共享当次搜索原点及范围；搜索耗尽后可在独立检查动作内转往备用橙块区。

2026-10-03：紫块搜索耗尽时跳过 Grap2，沿原
purple_to_orange 路线转入三橙采集；有紫时仍抓两橙。PlanA/PlanB 共用这一分支，
快速对准尝试耗尽只表明对准未完成，不能确认紫块已无，因此不触发三橙替代。
后续沿用原卸货、装载和 Build 动作；高地采集后的 Build 保留数量为三块的检查要求。
通信故障、遥测陈旧和取消继续中止执行。协议未变，无新增依赖、路线或机构动作。
本分支通过无硬件验证，三橙实际抓取与搭建效果待现场确认。

### 区域耗尽后的补抓

PlanA/PlanB 以及共用采集流程的 inspect_cargo 默认带 cross_region_refill=true。
地面三轮与高地两轮均支持：当前橙块搜索预算耗尽后，先恢复检查机构、结束已有直线退离，
重新检查货物；未满三块时，携带已有方块转往另一橙块区，只尝试抓取缺少的数量。
地面→高地使用 Grap1，高地→地面使用 Grap3；途中紫块区仅作为原路线的经过位置，不执行 Grap2。
有紫块时仍保留该块及本批计数，补抓仅增加橙块。

与主代码约定一致：检查不足三块且当前区域搜索耗尽才换区；尚有搜索余量时留在当前区找。
我方区对应 ground/Task1/Grap3，对侧区对应 highland/Task2 橙块/Grap1；两侧互为备用。
出发、到达备用区、每次补抓后及回程后均重新检查数量，按实际检查结果决定剩余缺额。
动作完成次数不代表捕获数量，二者不同不会直接阻止补抓；备用区继续搜索/抓取/检查，
直到满三块或其搜索范围也耗尽。同一区的累计搜索预算不因补抓重置。
数量未知、通信中断、陈旧遥测或急停仍中止执行。未引入整场材料归属与携带状态账本。

转场复用原运输、Tag6、卸货接近/离开和 return_orange 路段，仅保留卸货动作中的底盘后退，
不打开舱门。补抓完毕返回原取料区定位点，重建编码器原点及退离标记，再执行原定投放/搭建。
因此绕行较长；本次没有新增捷径距离。跨区段串行执行，不参与轨迹融合或移动Tag步骤消费；
原计划之后的运动优化仍有效。地面转高地使用第1/2轮高地参数，地面第3轮使用高地第2轮参数。
新组合的带货行驶、抓取和回程效果均待实机确认。

两侧均扫描耗尽且检查仍不足三块时，进入 both_areas_exhausted_pending 待定出口：
停车并终止本次执行，不自动少块返程、卸货、搭建或继续往返。后续动作待讨论。
仅数量满三块才返程接原任务；数量已满时无需启动换区。
未启用 cross_region_refill 的自定义检查动作保留原有行为；CLI 的 --show-plan 会显示该字段。
诊断日志记录 cross_region_refill 的出发、完整返回或两侧耗尽待定等结果。
refill_policy.py 与主代码同内容并有一致性测试。协议未变，无新增依赖或下位机动作。

新增组合用 ActionSpec 构造 StrategyPlan，调用 run_plan。未知参数、路线/标定不匹配、
缺采集初始化、已声明锚点不衔接、缺独立搭建航向等在任何动作前拒绝。
锚点表示路线约定，实际精度依赖已有控制与标定；尚无全局地图自动验证任意物理组合。

## 策略与只读预览

```bash
python main.py --strategy PlanA --show-plan
python main.py --strategy PlanB --show-plan
python main.py --list-flows
python main.py --flow collect-orange-1 --show-plan
python main.py --flow build-1 --show-plan
```

预览逐行显示功能动作、标定配置和参数，不连接硬件。去掉 --show-plan 才执行动作。
独立 build-1/2/3、unload-1/2 要求摆在搭建接近点，航向180°，并提供实际 --heading-zero-deg。

PlanA 保留三轮采集卸货、混合采集、搭建返程；PlanB 保留两轮混合采集暂存、
两轮橙块采集卸货、最后两次装载搭建。未启用新衔接标定时沿用原路线与参数。
PlanB 仍采用固定批次，没有“已抢空所有材料”的感知结束条件。

--task task1-1 等旧命令行选择器仅翻译为 collect-orange-1 等新动作数据计划；
不导入或执行旧 Task 类。task2_main.py 仅保留为混合采集启动适配器。
老文档的 Task 编号用于辨认历史标定路线，不代表当前代码模块。

## 已实现与未实现

已实现：宏观固定计划与动作控制分离、平铺动作编译、全流程共享执行上下文、上下文衔接匹配、
抓取→下一块的盲移及带速度视觉接管、最后一抓→退离及缺块回补、紫块抓取→转场、
数量检查→退离、Build→返程重叠、局部连续曲线、动作会话、相机旧帧隔离、故障后关闭执行。
这些衔接已接入真实动作注册表；五类跨动作衔接默认全部关闭，需启动参数显式开启。
新增抓取衔接还需加载标定或显式试跑参数。19类路线统一派发，单动作复用原控制、复合动作局部平滑。
Tag6仅在显式开启持续修正模块后参与运输末段；旧curves字段保留文件兼容，详见[规划模块](optimizations/MOTION_PLANNING.md)。
数量检查恢复期间只重叠原有直线后退，完成恢复后再执行剩余运输路线。
运动规划默认关闭，由 `--enable-motion-planning` 独立开启、`--classic-motion` 独立关闭；
快速对准默认关闭，由 `--enable-fast-alignment` 独立开启、`--disable-fast-alignment` 独立关闭。
五类跨动作衔接通过 `--enable-transition 类型` / `--disable-transition 类型` 单独控制；
`类型@轮次` 限定一轮，`--enable-transitions` 整组开启、`--disable-transitions` 整组关闭；逐项选择优先。
抓取参数继续由标定或已有显式试跑入口提供；关闭衔接后等待前动作完整退出。
这些 CLI 参数均保留抓取内部短顶墙及固件内部双轴/舵机并行。

2026-10-02：新增独立的 [快速抓取对准模块](optimizations/README.md)，按轮次/颜色显式配置。
普通抓取和盲移后的接管共用捕获窗口、制动与多帧低速确认；开启时跳过原来的粗/精两段。
未传 --enable-fast-alignment 或指定 --disable-fast-alignment 则保留旧对准路径。Tag/建筑定位、原标定文件不变。

未实现：动态宏观决策、建筑/材料世界状态持久化、智能续赛、对手识别与抢块决策、
全局最优轨迹求解、可靠前臂水平检测。当前曲线是手调航点的连续跟踪，路线之间的任意新组合仍停车兜底。
前臂复位采用 DONE 后的现场标定等待与新帧门控，尚无真实角度反馈；未标定时关闭新增抓取衔接。

当前比赛入口通过新增通信会话使用完整抬升通知和目标清除；固件旧命令始终保留 main 行为。
需先烧录一次增量固件；之后回退上位机无需重新烧录。见 [兼容说明](../protocol/ADDITIVE_COMPATIBILITY.md)。
物理净空未验证时抓取仍默认等待 DONE。见 [执行契约](execution/README.md)。
标定字段、启用方法与检查入口见 [衔接配置](TRANSITIONS.md)。尚无本轮实机效果或提速数据。
