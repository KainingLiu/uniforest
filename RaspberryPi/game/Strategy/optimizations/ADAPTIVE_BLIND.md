# 相邻橙块预观测与自适应盲移

2026-10-02，已部署树莓派，现场效果待验证。适用于 ground-1/2/3 的 Grap3、highland-1/2 的 Grap1。
启用条件为本轮同时开启 `next-cube` 衔接、快速橙块对准和自适应配置。紫块路线保持原行为。

## 行为与边界

| 行为变化 | 受影响模块 | 验证 |
| --- | --- | --- |
| 首块抓取列表保持，额外发布相邻块预观测与连续块排提示 | vision/opencv/orange_lookahead、orange_cluster、cube_detector | 合成双姿态图像、贴边、缺块、无暗缝、小暗斑、预观测失败 |
| 利用停车确认帧区分连续块排、可定位下一块、正常画面无下一块 | optimizations/adaptive_blind、fast_alignment、flows/operations | 独立新帧/遥测、行关联、歧义、跳变、位移与延迟补偿 |
| 抓取前消费观测，完整抬升后计算有限预移 | flows/transitions、execution/pickup_motion、transition_config、cli | 缩短/延长、原硬上限、阶段预算、代次/航向/时效、回退与接管 |

`VisionResult.all_blocks` 继续只提供每个橙色簇的首块作为原抓取候选。
新增 `orange_lookahead` 保存完整方块的毫米坐标与四角点；`orange_lookahead_complete`
表示当前橙块区域可完整解释。首块的 `continuous_row` 表示其后仍有连续块排，
不声称已测得排内每块的位置。预观测列表不会直接参与抓取目标选择。

视觉使用现有 Task1/Task2 固定平面标定，把顶面投影到固定平面后寻找外边缘及暗缝。
只有实际观测的相邻边界、合理块宽和充分橙色支持才能产生候选；一整条无暗缝色带不会按
100 mm 等距补出隐藏方块坐标。连续块排另作判断：完整左缘之后至少 1.8 个名义块宽的
可见橙色顶面，允许薄暗缝，较大缺口拒绝；无需逐块分割即可按用户要求预移一个块宽。
分界不完整不再等同于禁止搜索：当前抓取已确认、没有下一块可靠坐标时使用有界搜索，
这不代表已经测得下一块很远。过期或通信代次不符的已有观测仍拒绝使用。

分界是图像启发式判断，阴影或表面纹理仍可能误判；需要实际图像回放与现场验证。
初始软件门槛：1 mm 平面采样、17 个纵深样本；暗缝相对两侧亮度 <65%、至少 70% 纵深一致；
候选宽为名义块宽的 80%～120%、内部橙色占比至少 80%。这些值没有现场精度或成功率承诺。

快速对准停车确认期间顺带处理预观测，至少两张独立图像和两份独立遥测确认同一目标和同一分支。
不增加抓取前等待。目标选择为同一图像行上、右侧最近的完整候选；过近、跳变或歧义会取消预观测。
两帧不足时按回退策略处理。当前橙块 `confidence=80` 是固定输出，不能用作统计置信度。

## 距离计算

当前试跑预移 160 mm/s（原 80 mm/s 的两倍）、最长 8 秒，预观测有效期 12 秒；总距离包络 320 mm 含 20 mm 制动余量。
原始视觉误差也须在 ±10 mm 内，实际速度合格及两份独立画面确认后才允许抓取。
本轮按用户要求未运行测试；行为与速度效果待实机确认。

2026-10-02 按用户明确指定，试跑分三种情况：

| 观测情况 | 期望预移 | 日志原因 |
| --- | --- | --- |
| 当前抓取块属于连续块排 | 100 mm，使用名义块宽，无需每块精确位置 | continuous_row_pitch |
| 可靠识别并定位同排下一块 | 根据距离计算，最多 100 mm | observed_neighbor |
| 新鲜、完整画面中只见当前块，同排右侧未见下一块 | 300 mm 搜索预移，不假定一定还有矿 | unseen_next_search |
| 当前块已经确认抓取，但下一块预观测未建立 | 300 mm 有界搜索；不能把它当成实测间距 | no_confirmed_neighbor_search |

实测位置分支在图像采集时刻估计下一块在本次采集阶段坐标中的位置：

```text
下一块位置 = 当前编码器位置 - 实测横向速度 × 图像年龄
             + (下一块相机 X - 抓取目标 X) × 相机到横移比例
期望盲移 = min(已见目标预移上限, max(0, 下一块位置 - 抬升后的编码器位置
                   - 视觉接管预留 - 位置不确定度 - 航向误差预留))
盲移距离包络 = min(原标定最大距离, 剩余阶段距离, 期望盲移 + 原制动余量)
```

相机到横移比例与底盘编码器校正比例独立，需要按轮次标定。位置不确定度还包含图像延迟期间的
加速度误差与相邻帧位置差；航向预留按允许的最大偏航计算。制动余量由原盲移控制器在包络内保留，
仅加入一次，不能用自适应计算扩大原硬上限。时间同时受原盲移、累计搜索预算和观测有效期约束。

连续块排和未见下一块的分支分别以 100/300 mm 为名义行程，扣除观测后已经发生的横向位移。
这两个分支使用用户指定的步长，保留控制器的制动余量，不额外扣除实测位置分支的接管预留。
移动期间两者仍受航向、时间、搜索距离与相机新帧交接约束，100/300 mm 均不等于保证实走距离。

抓取前消费一次观测，绑定相机姿态代次、通信代次及急停代次。抬升后按当前编码器位置扣除期间位移。
机械臂运动期间沿用编码器控制；相机复位后，本次姿态的新帧中出现有效橙块才提前接管，无需走满期望距离。
接管仍通过现有搜索/快速对准重新确认实际目标，绝不根据预观测直接放行下一次抓取。
接管阶段会为再下一抓创建新的观察器，不复用上一抓的历史。

下一块预观测未建立时，默认按 300 mm 搜索；当前抓取仍须先通过视觉对准。
已有观测过期、通信断链或急停仍中止/拒绝使用，不把这些情况当作正常无目标。

## 配置与入口

旧 JSON 无需修改，仍采用原固定期望距离。生产配置在本轮抓取条目的 `next_cube` 中增加
`adaptive` 对象，对应 `AdaptiveBlindProfile`，必须 `validated: true`；同轮需有 `alignments[轮次/orange]`。
父抓取条目的 `verified_on`、`notes` 应同时记录自适应参数、相机几何、负载、入口与现场结果。

| 字段 | 含义；软件试验初值 |
| --- | --- |
| handoff_reserve_mm | 视觉重新确认的距离预留；20 mm |
| position_uncertainty_mm | 视觉/标定/编码器固定误差预留；5 mm，与快速入窗容差独立 |
| camera_x_to_lateral_scale | 相机横向毫米到车体横移毫米的正比例；1.0 |
| min_gap_mm、max_gap_mm | 同排下一块中心距范围；60～500 mm |
| max_candidate_drift_mm | 连续帧候选漂移及相邻候选歧义门槛；10 mm |
| max_yaw_change_deg | 允许观测与盲移期间的航向变化；2° |
| settled_speed_mm_s | 允许积累停车观测的实测速度；20 mm/s，同时要求速度指令为零 |
| frame_timeout_s | 单帧及抓取前消费的最大年龄；0.3 s，最大允许 0.5 s |
| max_prediction_age_s | 从图像采集到盲移结束的有效期；5 s |
| fallback_distance_mm | 无有效预观测时的期望预移；0 mm，即跳过盲移 |
| continuous_pitch_mm | 连续块排默认步长；用户指定 100 mm |
| observed_distance_limit_mm | 已定位下一块时的预移上限；试验值 100 mm |
| unseen_search_mm | 画面正常、未见下一块时的搜索预移；用户指定 300 mm |
| confirm_frames | 至少两张独立新帧/遥测；2 |
| validated、trial_enabled | 标定/显式试跑标志，禁止把软件试验初值写成实测参数 |

显式试跑有单独开关，普通 `--trial-optimizations` 保持原行为。以下为只读预览，不连接硬件：

```bash
python main.py --flow collect-orange-1 --trial-optimizations --trial-adaptive-blind --enable-transitions --enable-fast-alignment --show-plan
python main.py --flow collect-mixed-1 --trial-optimizations --trial-adaptive-blind --enable-transitions --enable-fast-alignment --show-plan
python main.py --strategy PlanA --trial-optimizations --trial-adaptive-blind --enable-transitions --enable-fast-alignment --show-plan
```

`--trial-adaptive-blind` 必须与 `--trial-optimizations` 一起使用，不能同时关闭快速对准。
新模式试跑包络为 320 mm，即最大期望搜索预移 300 mm 加原 20 mm 制动余量。
只有加上 `--trial-adaptive-blind` 才使用这组值；原试跑包络 100 mm、原固定期望 80 mm 保持。
原速度、4 秒盲移时限和搜索总预算保持，可能使实际盲移更短。
已标定 JSON 的原硬上限仍生效：要完整容纳 300 mm 期望搜索预移，需独立确认
`blind.max_distance_mm >= 300 + braking_margin_mm`，代码不自动改写现场参数。
关闭 `next-cube` 会回到完整抓取后普通搜索；生产配置使用 `--disable-fast-alignment`
会同时停用自适应观察，盲移恢复该文件中的固定期望距离。

## 验证与记录

```bash
python -m unittest tests.test_adaptive_blind tests.test_orange_lookahead -q
python tools/check.py
python -m unittest discover -s tests -q
```

实机前依次完成语法/无硬件测试、`cmake --preset Debug`、`cmake --build build/Debug`，
核对串口、相机角色与两个橙块标定，然后按单次抓取→相邻两抓→本轮采集验证。烧录只经 CLion。
已部署到树莓派，编号 `20261002-adaptive-be945c5e`；部署过程未烧录、未执行实机动作。
同日按用户要求试跑完整 PlanA，关闭运动规划、其余优化开启；首轮六次对准均因
`search_boundary` 结束，未抓取，随后数量检查拒绝 A 板空闲状态而中止。停止后四轮 0 RPM、
步进忙标志 0，未重跑。此次未进入自适应盲移，100/300 mm 现场效果继续待验证；详见 CHANGELOG。
后续软件修复恢复锁定目标后的双向对准，将其行程预算与搜索边界分开，并处理零抓取时的安全 IDLE 检查。
这些修复不改变本页的 100/300 mm 配置，尚未自动重跑实机。
修复部署编号 `20261002-pickup-fix-aaf7442f`；本地 195 项相关检查通过，远端 193 项通过、2 项源码检查跳过。

2026-10-02 本机验证：36 项专项测试及全量 464 项单元测试通过，导入检查通过。
Debug 配置缺少 `arm-none-eabi-gcc/g++`；未生成 `build.ninja`，固件构建未完成。
部署验证：远端 160 文件语法检查通过，465 项全量测试中 455 项通过、10 项下位机源码检查明确跳过。
正式目录的导入、相关专项和 PlanA/PlanB/Task2/旧模式只读预览通过；25 文件哈希核对一致，18 个 JSON 保持。
旧文件及清单：`/home/uniforest/Uniforest/.deploy-backups/20261002-adaptive-be945c5e/`。

`adaptive_blind` 衔接记录包含分支原因与期望距离；它不代表实际位移。
实际移动继续看 `blind_feedback`，入窗结果继续看 `fast_alignment`。
现场记录应包括日期、轮次、启动参数、各项阈值、原最大包络、观测间距、期望/实际位移、
回退原因、入窗耗时、反向次数、抓取成功率以及缺块/遮挡回放结果。

**协议未变**。已核对双方 schema、commands 与 protocol.h/protocol.c：
底盘 0x10/8 B、动作启动 0x50/6 B、原遥测 0x80/80 B、动作状态 0x83/11 B；
现有增量命令 0x52～0x55 与回复 0x84～0x86 保持；数值大端、CRC 小端和 200 ms 失联/会话约束保持。
预观测及距离计算仅发生在树莓派，复用现有底盘速度与动作会话；无新依赖、无本功能下位机改动。
