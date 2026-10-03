# 上位机操作与参数参考

当前比赛路径优化仅有原路线局部平滑：--enable-motion-planning开启，--classic-motion关闭。
单段直线仍走原控制器，连续移动转向按相对位移与最终朝向合并，靠墙/退离/抓取等边界保留。
Tag6持续修正默认关闭：运输平滑到原目标后按原顺序对准Tag及横移。额外--enable-moving-tag6才接入移动反馈，
--disable-moving-tag6关闭反馈且保留局部平滑；建筑视觉最终确认后才允许优化流程Build。
无全场地图或Tag起点门禁；--navigation-config已从比赛入口移除，详见[规划模块](Strategy/optimizations/MOTION_PLANNING.md)。

2026-10-03盲移：固定100 mm，连续排100 mm，已定位下一块最多200 mm，无可靠下一块时200 mm；
160 mm/s试跑速度、固定4秒/自适应8秒、12秒预观测有效期保持。物理效果待验证。
后退检查、明确少块返回和未知分支见[衔接说明](Strategy/TRANSITIONS.md)。

## 通过 SSH 启动持续运行的任务

使用 `tools/managed_task.py` 交由树莓派 systemd 用户服务管理，SSH 日志连接断开不终止任务。
需要 `Linger=yes`（`loginctl show-user uniforest -p Linger`）；启动器会拒绝未满足条件的启动。
它与桌面入口共用运行锁，禁止重启续跑；停止通过 SIGINT 执行现有清理，串口失联急停保持。

```bash
.venv/bin/python tools/managed_task.py start -- --strategy PlanA --classic-motion --trial-optimizations --trial-adaptive-blind --enable-transition next-cube --enable-transition last-departure --enable-transition purple-departure --enable-transition inspect-departure --enable-transition build-return --enable-fast-alignment
.venv/bin/python tools/managed_task.py logs
.venv/bin/python tools/managed_task.py status
.venv/bin/python tools/managed_task.py stop
```

日志跟踪窗口的 Ctrl+C 只结束跟踪；停止机器人须执行上述 `stop` 或使用现场急停。
运行日志同时自动保存在项目的 `logs/runs/`，终端退出或 journal 滚动后仍可查看。
`main.py`、`task2_main.py`、`robot.py` 每次直接启动独立记录 `console.log`、`diagnostics.jsonl`、`run.json`，
共保留最近 100 次运行；正在写入的记录跳过清理，运行结束后补清理。
帮助、预览、参数错误和导入失败也归档；异常强制退出时 `run.json` 可能停留在 `running`，不代表进程仍存活。
日志入口在加载机器人模块前创建，磁盘不可写时在启动硬件前退出；运行中日志写入失败仅提示一次，继续原控制与清理流程。
自动归档无需 `--diagnostics-log`；该参数指定的额外诊断文件保留原追加行为，不参与自动清理。
完整目录说明见 [自动运行日志](README.md#自动运行日志)。

2026-10-02 试跑入窗半宽 10 mm、两帧确认；窗外最低纠偏速度复用原起步设定 80 mm/s，
仍受剩余窗口宽度的停车包络限制。左边缘恢复允许越过采集原点，保持 1000 mm/6 秒独立上限；
这些参数及新逻辑现场效果待验证，不代表硬件性能事实。协议未变，下位机无需修改。

快速启动见 [README](README.md)。本文保留详细操作、参数与历史说明，最新部署状态以 [CHANGELOG](CHANGELOG.md) 为准。

启动命令、默认行为和各项优化开关集中在[运行方式与优化插拔](#运行方式与优化插拔)。

2026-10-02 [相邻块预观测与自适应盲移](Strategy/optimizations/ADAPTIVE_BLIND.md) 已部署，编号
`20261002-adaptive-be945c5e`；远端无硬件检查通过。
同日全程试跑在首轮橙块对准边界/空抓取后的检查处中止，已停止；100/400 mm 盲移尚未执行到。
对应软件修复恢复锁定目标后的双向对准，并允许零抓取时通过静止遥测确认后检查 IDLE，仍待实机复验。
修复已部署，编号 `20261002-pickup-fix-aaf7442f`；原启动参数保持，无新增对准边界参数。
现场日志和修复说明见 CHANGELOG。
独立试跑预览：`python main.py --flow collect-orange-1 --trial-optimizations --trial-adaptive-blind --enable-transitions --enable-fast-alignment --show-plan`。
原试跑入口保持；新模式连续块排预移 100 mm，正常画面未见下一块时搜索预移 400 mm，
可定位时按实测距离调整、试验上限 100 mm。无效/歧义观测默认跳过盲移。以上均待实测，协议未变。

2026-10-02 较早的上位机架构部署编号为 `20261002-motion-pid-bb252a93`；
保留现场相机/标定/采集配置，完成远端无硬件测试与入口预览，未运行机器人动作。
回滚和验证记录见 CHANGELOG 对应部署条目。下面较早条目中的“未部署”描述为当时状态。

2026-10-02：[位置间连续导航与移动 Tag6 定位](Strategy/navigation/README.md) 已提供独立仿真入口。
新增[可插拔运动规划与位置 PID](Strategy/optimizations/MOTION_PLANNING.md)：优化选择器默认启用，
`--classic-motion` 恢复原路线；位置间导航复用比赛参数，长距巡航默认提高 10%，位置/航向 PID 闭环跟踪。
运行 `python -m simulation.navigation_server --port 8766` 后访问本机 8766 端口的 simulator.html，
可选择任意合法沿墙位置并验证运动；当前比赛入口与实机标定保持。

当前架构更新：2026-10-01（本机，未部署）。历史参数和验证结果集中在 [CHANGELOG.md](CHANGELOG.md)，下位机接口见开发电脑上的 `Uniforest_A/PROJECT.md`。树莓派只部署上位机，不保留下位机源码副本。

当前模块说明与扩展入口见 [Strategy/README.md](Strategy/README.md)。2026-10-01 已移除旧 Task 编排，PlanA/PlanB 直接展开功能动作；本轮未同步树莓派。历史部署记录见 CHANGELOG。

2026-10-02 新增 [可插拔快速抓取对准](Strategy/optimizations/README.md)：按轮次/颜色配置入窗控制，
默认关闭；--disable-fast-alignment 单独恢复原抓取对准。制动参数和软件容差仍需现场验证。

比赛入口现在要求支持增量会话的固件；首次需通过 CLion 烧录。之后回退上位机到 main f6e9be1，
同一份固件按旧命令执行主分支行为，无需重刷。见 [单固件兼容说明](protocol/ADDITIVE_COMPATIBILITY.md)。

离线场地/车辆与比赛仿真见 [simulation/README.md](simulation/README.md)：运行 `python -m simulation`，
打开生成的 `simulation/output/simulator.html`，从启动区连续回放完整 PlanA/PlanB，并显示方块去向。
使用 `--routes-only` 可单独比较 15 类曲线路线、28 个参数组合。
仿真保留平面包络干涉和名义感知/机构假设；不会连接硬件，结果不能直接用作实机标定。

## 环境与目录

运行平台为 Raspberry Pi 5 8GB；开发主机 `192.168.137.50`，用户名 `uniforest`，项目目录 `/home/uniforest/Uniforest/RaspberryPi`。密码由队内管理员提供，不保存到仓库。

| 入口/目录 | 职责 |
| --- | --- |
| `main.py` | 统一执行入口、策略与功能动作流选择、只读逐动作预览 |
| `robot.py` | 通信生命周期、设备聚合、预检与调试交互 |
| `Strategy/` | 声明式 PlanA/PlanB、功能动作/路线、统一执行器、独立控制算法与标定配置 |
| `control/` | 底盘位置外环、舵机/步进调试、A 板动作客户端 |
| `protocol/` | 帧编解码、传输和 schema v3 契约 |
| `vision/opencv/` | 当前 OpenCV 方块检测、AprilTag 定位、相机与标定；旧模块路径保留兼容入口 |
| `vision/yolo/` | 自动原图采集及离线分割/关键点实验；实时 hybrid 尚未接入比赛 |
| `sensors/`、`utils/` | 传感器封装和通用辅助 |
| `tools/`、`tests/` | 实机/图像调试、协议格式、比赛策略与数据采集检查 |

依赖方向为 `main.py → Strategy → robot → control/protocol/vision`。比赛代码不导入
`tools/` 或 `tests/`，历史备份不属于当前实现。

2026-10-01 已移除自然语言/语音 Agent 及其专属依赖。比赛启动使用 `main.py`，
单次机构动作和调试使用 `robot.py`。本次移除协议未变，无下位机固件改动。

同日执行架构改造见 [执行说明](Strategy/execution/README.md)：PlanA/PlanB 直接生成平铺动作流，
整套比赛使用统一执行器，抓取→下一块/退离、检查→退离及 Build→返程已接入动作流。
新增盲移、带速度视觉接管和连续路线通过现场标定配置启用，见 [衔接配置](Strategy/TRANSITIONS.md)。
配套固件增加完整抬升通知并清除停止后的旧速度目标，报文格式未变；尚未部署或实机验证。

图片自动采集默认关闭。需要采集时，在树莓派终端使用
`UNIFOREST_COLLECT_DATA=1 .venv/bin/python main.py --strategy PlanA` 显式开启本次采集。
开启采集及 Robot 方块视觉后，仅在抓取前的橙色/紫色搜索和对准阶段自动采集原图，
抓取、数量检查、Tag/建筑对准、投放与普通移动不采集。保存到本机
`vision/yolo/data/collection/`，按运行批次记录阶段、相机设置并建立 SQLite 索引。
`python tools/collect_cube_data.py stats`
查看数量；本轮加 `--no-collect-data` 可停用，`--dataset-dir` 可更换目录。
采样、存储上限、独立补拍和标注要求见 [自动采集说明](vision/yolo/docs/DATA_COLLECTION.md)。

维护入口：[前臂零点调整](tools/arm_zero_adjust.md)、
[携带数量检查与标定](tools/carried_cube_count_test.md)、
开发电脑上的 `Uniforest_A/ACTIONS.md`（Grap/Build 动作流程）。角度、检查时序和机械参数在对应
文档维护。正常心跳日志静默，50 ms 后台心跳及通信失联保护保留。

### 仓库与树莓派部署状态

截至 2026-09-27：文件上传、固件烧录和现场验证分别记录。YOLO 文件及数据按用户要求保留各端现状，不纳入此次统一。

| 项目 | 当前状态 |
| --- | --- |
| 模块化 Task 库与 Task3-2 2200 mm 返程 | 09-26 已定向同步，树莓派 83 文件语法和 65 项测试通过；本地 84 文件语法、65 项测试、Debug 构建通过；现场待验证 |
| PlanA/PlanB 与 Task0-2、Task4-1/2 | 09-27 已同步；本地 103 项测试及 Debug 构建通过，树莓派 103 项测试通过（2 项头文件检查跳过），16 个桌面入口预览通过；新路线现场待验证 |
| Tag6 航向试调 | 09-26 已同步独立 50 Hz、P/I/D=6/0/0；三轴合格连续 4 帧完成；现场待验证 |
| 转向第二轮衔接优化 | 09-26 本地与树莓派均已回退为 600 ms，加速恢复原值；第一轮定距优化保留 |
| 定距路线第一轮衔接优化 | 09-26 已定向同步 `route_mode`：±8 mm 内停车确认、加速同时受剩余距离约束；远端检查通过，现场效果待确认 |
| Tag6 三帧滤波、平移 Ki/Kd=0、最低速度 100 mm/s | 09-26 已定向同步两任务两轮；±8 mm 容差不变，远端四套配置核对通过 |
| Task1 投放顶墙按 180° 更新航向基准 | 09-25 已同步，两轮换算检查通过 |
| 路线、数量检查复位重叠、橙色/紫色 5 秒超时抓取、常规转向最多额外 1.5 秒 | 此前已定向同步；最新远端 63 文件语法、导入、协议检查通过 |
| A 板动作及步进 | 当前 Grap3 回收 21.5+5.5 cm、下降触发 16 cm；步进恢复 400/60/400，巡航约 7143 脉冲/秒。须 CLion 烧录，现场效果待确认 |
| 橙色搜索与视觉降级 | 09-27 已统一并同步搜索与回找链路，所有比赛视觉环节支持降级继续；本地 85 / 远端 84 文件语法、两端 92 项测试通过，现场待验证 |
| 独立步进调试默认值 | 09-27 两端统一为当前下位机源码的 400/60/400；Stepper 与 Transport 双步进接口共用常量，显式参数优先；单电机沿用 A 板默认配置 |
| 自动采集 | 09-26 已定向同步：仅抓取前橙色/紫色搜索和对准采集；远端 66 文件检查、27 项现有检查通过，现场效果待确认 |
| 视觉目录拆分 | 09-27 两端统一至 `vision/opencv/`；原视觉模块导入和相机调试入口保留兼容转发，现场标定与配置值保持 |
| 清理及检查 | 09-27 旧 A 板源码、旧测试与橙色实验入口已归档至运行目录外；本地 95 项通过，远端 93 项通过、2 项下位机头文件比对跳过；100 个受管文件对齐，YOLO 原有差异保留 |
| Robot 兼容接口 | 两端一致，保留 `quiet_heartbeat` 与采集接口 |

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

下文 Task 编号保留用于辨认历史标定路线与兼容命令行别名，当前源码已无对应 Task 类；
新入口使用 `--flow`，`--show-plan` 会展开全部功能动作。

## 运行方式与优化插拔

本节按 2026-10-02 当前工作区源码整理。以下命令均在 `RaspberryPi/` 目录内执行，
使用已安装依赖的项目 Python；树莓派可先 `source .venv/bin/activate`。
文档命令与本地源码对应，部署端是否已有相同开关可用 `python main.py --help` 核对。

**五类跨动作衔接默认全部关闭，需用命令行显式开启，也可限定某一轮。** 抓取参数仍从标定文件或显式试跑入口加载。
路线规划与快速抓取对准保留各自开关。抓取内部短顶墙及固件内部并行保持原实现。
这些选择在启动时加载；切换配置前先停止当前程序，确认物理起点和机构状态，再重新启动。
运行中编辑 JSON 不会改变本次执行，也没有在线热插拔命令。

### 选择运行范围

| 目的 | 入口与范围 |
| --- | --- |
| 完整比赛 | `python main.py --strategy PlanA` 或 `--strategy PlanB` |
| 单轮组合 | `--strategy set1`、`--strategy set2`；混合采集接搭建用 `--strategy collect-build-1` / `collect-build-2` |
| 单段采集 | `--flow collect-orange-1/2/3`、`--flow collect-mixed-1/2`；斜杠表示分别选择一个名称 |
| 独立搭建或投放 | `--flow build-1/2/3`、`--flow unload-1/2`；要求对应物理起点，并传入已标定的 `--heading-zero-deg` |
| 单次机构调试 | `python robot.py --action grap1`，也支持 `grap2`、`grap3`、`build`；等待整套动作完成，不附加比赛后续路线 |
| 设备预检 | `python robot.py --preflight --vision --localization`；连接设备读取状态，不执行比赛动作 |
| 离线场地导航 | `python -m simulation.navigation_server --port 8766`；浏览器打开 `http://127.0.0.1:8766/simulator.html`，不连接机器人 |

无参数启动 `main.py` 等价于 PlanA。`--strategy`、`--flow`、`--task` 三者互斥。
独立动作流按其约定起点执行，不会自动从启动区导航过去；具体起点见下文比赛入口与策略说明。

先查看可选流程与配置，以下命令不打开串口或相机：

```bash
python main.py --list-flows
python main.py --strategy PlanA --show-plan
python main.py --strategy PlanB --show-plan
python main.py --flow collect-orange-1 --show-plan
python main.py --help
```

执行实机命令前，完成下文[验证与启动顺序](#验证与启动顺序)。停止使用 Ctrl+C；
通信异常、遥测陈旧或急停后，本次执行关闭，恢复通信不会自动续跑。

### 按具体衔接启用或关闭

在 `main.py` 或 `task2_main.py` 后追加以下参数，可重复指定：
`--enable-transition 类型` 开启，`--disable-transition 类型` 关闭。
`--enable-transitions` 整组开启、`--disable-transitions` 整组关闭，两者互斥。
加载标定文件或试跑参数也不会自动开启衔接；关闭后先等待前动作完整退出，再停车、进入下一动作。

| 类型 | 行为 | 可选轮次后缀 |
| --- | --- | --- |
| `next-cube` | 橙块抓取后提前右移并接管搜索 | `ground-1/2/3`、`highland-1/2` |
| `last-departure` | 最后一抓收臂时提前退离，含跳过第三槽 | 同上 |
| `purple-departure` | 紫块抓取后提前转场 | `highland-1/2` |
| `inspect-departure` | 数量检查复位与直线退离并行 | `ground-1/2/3`、`highland-1/2` |
| `build-return` | 第三块释放后提前返程 | `building-1/2/3`、`staged-building` |

表中斜杠表示分别选择一个轮次，例如 `next-cube@ground-1`；实际可选范围由本次流程决定。
不写后缀时作用于所选流程中该类型的全部轮次。两个暂存搭建动作共用 `staged-building` 配置。
轮次选择优先于类型选择，类型选择优先于整组开关，其余默认关闭。
参数书写顺序不影响优先级；相同选择同时开启和关闭会报错。指定当前流程不存在的轮次也会报错。
某类型在本次流程完全不存在时，全局类型选择无动作效果，预览会显示 `not present in this flow`。

```bash
# 开启全部五类衔接；标定必须覆盖本次流程的全部抓取轮次
python main.py --strategy PlanA --transition-config Strategy/field-transitions.json --enable-transitions --enable-fast-alignment --show-plan

# 关闭检查和搭建返程的并行衔接
python main.py --strategy PlanA --disable-transition inspect-departure --disable-transition build-return --show-plan

# 五类先全部关闭，再只开启搭建返程
python main.py --strategy PlanA --disable-transitions --enable-transition build-return --show-plan

# 全开后只关闭第一轮地面抓取预移
python main.py --strategy PlanA --transition-config Strategy/field-transitions.json --enable-transitions --disable-transition next-cube@ground-1 --enable-fast-alignment --show-plan

# 使用已标定参数，开启下一块预移，关闭最后一抓提前退离
python main.py --flow collect-orange-1 --transition-config Strategy/field-transitions.json --enable-transition next-cube --disable-transition last-departure --show-plan

# Task2 使用同一组开关
python task2_main.py --variant 2 --disable-transitions --enable-transition inspect-departure --show-plan
```

去掉 `--show-plan` 才执行。预览的 `Action transitions` 逐轮显示最终 enabled/disabled；
enabled 表示允许使用，现场仍需满足抓取成功路径、完整抬升通知、相机恢复和路线锚点等运行条件。
`Pickup transitions` 一行列出已加载参数，最终开关状态以 `Action transitions` 为准。
开关不修改 JSON，也不生成缺失的速度、制动或复位参数；显式启用时缺少所选轮次参数会在连接机器人前报错。
现有 `--trial-optimizations` 仍须显式选择，其未验证试跑参数加载后仍需本节开启参数，不能与标定文件混用。
`--disable-transitions` 不影响快速对准、路线规划、抓取短顶墙或固件内部步进/舵机时序。
需要同时关闭前两项时再追加 `--disable-fast-alignment --classic-motion`。

### 路线规划与快速对准开关

| 命令行参数 | 配置对应项 | 关闭范围 | 保留的行为 |
| --- | --- | --- | --- |
| `--enable-motion-planning` | `motion_planning_enabled: true` | 开启完整曲线/速度规划（默认关闭） | 原局部目标＋关键经过区域；单动作原控制、复合动作平滑 |
| `--enable-moving-tag6` / `--disable-moving-tag6` | `moving_tag6_enabled: true/false` | 独立选择行进中Tag6持续修正，默认关闭 | 需同时开启运动规划；关闭后在原目标处执行原Tag对准和横移 |
| `--enable-fast-alignment` | 保留已加载的 `alignments` | 开启快速抓取入窗对准（默认关闭） | 衔接和运动规划独立选择；缺参数时拒绝启动 |
| `--classic-motion` | `motion_planning_enabled: false` | 原路线局部平滑和行进Tag6接管 | 使用原 `ROUTES` 路线；动作衔接和快速对准仍按各自开关选择 |
| `--disable-fast-alignment` | `alignments: {}` | 快速抓取入窗对准 | 使用原抓取对准路径；路线与跨动作衔接保持各自选择 |

比赛入口的运动规划须显式传入 `--enable-motion-planning`；仅加载配置文件仍关闭。
开启参数与 `--disable-motion-planning`（兼容 `--classic-motion`）互斥；命令行参数在读取配置后覆盖本次选择，不改写文件。
快速对准也须显式传入 `--enable-fast-alignment`，与 `--disable-fast-alignment` 互斥。
原七项优化及新增Tag6持续修正模块均默认关闭；标定和试跑参数只提供数据。
配置文件会先完整校验，无效标定仍会拒绝启动，
关闭参数不能跳过文件校验。

下面用仓库自带的空标定配置演示，均为只读预览：

```bash
# 局部平滑只读预览；不连接机器人
python main.py --strategy PlanA --enable-motion-planning --show-plan

# 只使用经典路线
python main.py --strategy PlanA --transition-config Strategy/transition_config.example.json --classic-motion --show-plan

# 只使用原抓取对准
python main.py --strategy PlanA --transition-config Strategy/transition_config.example.json --disable-fast-alignment --show-plan

# 同时关闭规划替换和快速抓取对准，且不加载抓取提前移动标定
python main.py --strategy PlanA --classic-motion --disable-fast-alignment --show-plan
```

两个参数同时使用后，数量检查→运输、Build→返程仍由各自衔接开关决定；未显式开启时均关闭。
抓取内部短顶墙、固件内部双轴/舵机并行、步进平滑加减速、经典底盘控制和卸货关舱后的
无等待衔接也保留。`--disable-transitions` 关闭五类跨动作衔接；它不关闭曲线、快速对准或动作内部并行。
该组合也继续要求增量固件会话；切换旧上位机程序的步骤见[单固件兼容说明](protocol/ADDITIVE_COMPATIBILITY.md)。

### 默认配置与逐项插拔

未提供 `--transition-config` 时使用 `TransitionConfig()`：运动规划选择器关闭，
`pickups`、`curves`、`alignments` 均为空。此时新增抓取提前移动和快速对准未启用，
运动规划关闭时执行经典实现；五类衔接（含数量检查→运输、Build→返程）均默认关闭。
仅加载标定文件或 `--trial-optimizations` 也保持五类衔接关闭，须使用整组或单项开启参数。

将 [transition_config.example.json](Strategy/transition_config.example.json) 另存为
`Strategy/field-transitions.json`，保留已有现场文件。空配置的完整结构如下：

```json
{
  "version": 1,
  "motion_planning_enabled": false,
  "firmware_full_lift_validated": false,
  "firmware_id": "",
  "pickups": {},
  "curves": {},
  "alignments": {}
}
```

空配置只建立配置结构，不会产生新标定。逐项启用方法如下：

| 优化 | 插入/启用 | 拔出/关闭 | 条件与限制 |
| --- | --- | --- | --- |
| 橙块抓取→下一块 | 提供本轮 `next_cube` 的 `blind`、`acquire` 标定，并加 `--enable-transition next-cube` | `--disable-transition next-cube` | 要求完整抬升能力与现场记录；地面使用 `grap3`，高台橙块使用 `grap1` |
| 最后一抓→提前退离 | 提供本轮橙块抓取标定，并加 `--enable-transition last-departure` | `--disable-transition last-departure` | 要求完整抬升能力与现场记录；少块时按实测退距返回补抓 |
| 紫块抓取→转场 | 提供本轮 `grap2` 标定，并加 `--enable-transition purple-departure` | `--disable-transition purple-departure` | 要求完整抬升能力与现场记录；机构与转场均完成后才结束衔接 |
| 数量检查复位→运输 | `--enable-transition inspect-departure` | `--disable-transition inspect-departure` | 默认关闭；开启后按当前检查会话和路线条件执行 |
| Build→返程 | `--enable-transition build-return` | `--disable-transition build-return` | 默认关闭；关闭后先完成机构退出再返程；开启仍要求 `after_build` 和锚点匹配 |
| 旧曲线记录 | curves字段仅保留读取兼容 | 不参与比赛规划 | 不再提供独立曲线后端或注入入口 |
| 快速抓取对准 | 在 `alignments` 增加 `轮次配置/颜色` 条目及 `profile`，并加 `--enable-fast-alignment` | 移除开启参数或使用 `--disable-fast-alignment` | 与抓取提前移动独立，普通搜索及盲移接管均可使用；不改变 Tag/建筑对准 |
| 原路线局部平滑 | --enable-motion-planning | --classic-motion | 19类路线统一派发，单动作保留原控制；默认保持原Tag识别位置 |
| 行进Tag6持续修正 | --enable-moving-tag6 | --disable-moving-tag6 | 独立模块，默认关闭；开启需同时启用局部平滑，仅匹配现有Tag/偏置动作 |

配置键的范围：

- 抓取：`ground-1/grap3`、`ground-2/grap3`、`ground-3/grap3`；
  `highland-1/grap1`、`highland-2/grap1`；`highland-1/grap2`、`highland-2/grap2`。
- 对准：`ground-1/orange` 至 `ground-3/orange`；`highland-1/orange`、`highland-2/orange`，
  以及 `highland-1/purple`、`highland-2/purple`。
- 曲线：例如 `building-1/build_return`；合法路线族及参数字段见[衔接配置](Strategy/TRANSITIONS.md)。

启用的抓取、曲线、对准条目都要求 `validated: true`、`verified_on: YYYY-MM-DD` 和非空 `notes`。
记录实际日期、参数、机械状态/负载、适用轮次、测试入口和现场结果；测试中的合成参数不能直接充当现场标定。
抓取条目另外要求 `firmware_full_lift_validated: true`、实际固件标识 `firmware_id`、
实测相机复位等待 `arm_restore_s`。程序仍会协商固件能力，标志本身不证明物理净空。
完整抓取/曲线字段见[衔接配置](Strategy/TRANSITIONS.md)，快速对准字段见[快速对准 README](Strategy/optimizations/README.md)。

橙块 `next_cube.expected_distance_mm` 默认 **80 mm**。相机提前恢复会立即接管，
安全上限或剩余搜索预算也可能提前结束盲移；最终位移需要现场测量。
仅开启检查/Build 衔接时，使用 `--enable-transition inspect-departure --enable-transition build-return`，
无需抓取标定。显式开启某项抓取衔接后，删除其必需标定会导致启动校验失败。
修改注册表属于源码开发，需要重新验证受影响流程。

关闭衔接使用上表的命令行参数；JSON 中的 `last_departure` / `departure` 不再决定启用状态。
`next_cube` 必须是含参数的对象，不能写成 `false`；`validated: false` 会使保留的条目校验失败。
例如仅停用第一轮地面预移时，加 `--disable-transition next-cube@ground-1`，保留原标定参数。

### 带现场配置运行与对照

以下 `Strategy/field-transitions.json` 由上一步另存并填写；仓库不附带已验证的启用参数。
先预览单段，核对输出中的 `Pickup transitions`、`Continuous routes`、
`Motion planning`、`Fast pickup alignment` 和 `Action transitions`。最终衔接开关以 `Action transitions` 为准。
下列全开命令要求标定覆盖所选流程，实际衔接还要满足运行时条件。

```bash
python main.py --flow collect-orange-1 --transition-config Strategy/field-transitions.json --enable-transitions --enable-motion-planning --enable-fast-alignment --show-plan
python main.py --flow collect-mixed-1 --transition-config Strategy/field-transitions.json --enable-transitions --enable-motion-planning --enable-fast-alignment --show-plan
python main.py --strategy PlanB --transition-config Strategy/field-transitions.json --enable-transitions --classic-motion --enable-fast-alignment --show-plan
```

完成验证顺序并满足所选流程起始条件后，去掉 `--show-plan` 执行。下面为会驱动机器人的命令：

```bash
# 小范围采集流程，记录本次实际选择及衔接结果
python main.py --flow collect-orange-1 --transition-config Strategy/field-transitions.json --enable-transitions --enable-motion-planning --enable-fast-alignment --diagnostics-log /tmp/uniforest-opt-test.jsonl

# 完整策略，使用同一份已验证配置
python main.py --strategy PlanA --transition-config Strategy/field-transitions.json --enable-transitions --enable-motion-planning --enable-fast-alignment

# 只关闭路线规划替换，保留配置中的抓取提前移动和快速对准
python main.py --strategy PlanA --transition-config Strategy/field-transitions.json --enable-transitions --classic-motion --enable-fast-alignment

# 只关闭快速对准，保留配置中的抓取提前移动和路线规划选择
python main.py --strategy PlanA --transition-config Strategy/field-transitions.json --enable-transitions --enable-motion-planning --disable-fast-alignment

# 同时关闭上述两项，仍保留配置中的抓取提前移动
python main.py --strategy PlanA --transition-config Strategy/field-transitions.json --enable-transitions --classic-motion --disable-fast-alignment

# 对照：经典路线、原抓取对准，仅开启检查/Build 衔接
python main.py --strategy PlanA --classic-motion --disable-fast-alignment --enable-transition inspect-departure --enable-transition build-return
```

需要先检查任意一条启动命令时，在其末尾加 `--show-plan` 即可只读预览。
其中 `Continuous routes` 列出文件中的曲线条目，指定 `--classic-motion` 后仍可能显示这些键；
同时检查 `Motion planning: classic (optimizers disabled)`，才能确认本次已关闭规划替换。
实际是否使用某条优化，以运行日志中的选择与衔接结果为准。

替换为 PlanB 时沿用相同配置规则。对照前重新准备相同物理起点、负载与机构状态，
停止原进程后再启动，配置切换不会恢复车辆位置。每次只改变目标开关，记录耗时、成功率和恢复次数。
诊断事件 `motion_backend_selected`、`transition_result`、`fast_alignment` 用于核对实际选中路径；
开启总开关不保证某条路线具有优化标定。

`task2_main.py` 与 `main.py` 共用配置及全部优化开关，支持 `--variant 1/2` 和 `--show-plan`。
例如 `python task2_main.py --variant 2 --disable-transitions --show-plan`。
桌面脚本 `tools/desktop_task.sh` 只接受任务名和可选 `--show-plan`，也不会透传这些参数。
自定义优化运行使用终端 `main.py`，并确保没有另一控制进程占用机器人。

### 开发接口与组合限制

比赛运行器只保留LocalRoutes，原动作决定终点、朝向、速度和操作边界。
MotionPlanning不接受后端列表；Tag6为独立可选引导模块，默认不挂载。
开启后在同一轨迹循环中修正目标，不创建第二个底盘控制线程。
详见[规划模块](Strategy/optimizations/MOTION_PLANNING.md)。

动作衔接通过 `TransitionRegistry` 注册匹配条件、优先级和执行函数。当前比赛入口内部组装注册表；
新增或单独替换检查/Build 衔接，需要修改 `flows/transitions.py` 或 `flows/factory.py`，
`run_selection()` 尚无自定义衔接注册表参数。移除某条注册后，运行时回到
“前动作退出完成 → 停车 → 后动作进入”，后动作的主体仍照常执行。
快速对准可以独立启停，换成另一套算法仍需要修改现有调用处。

比赛局部规划复用Chassis.follow_trajectory及原直线控制器，不调用RobotNavigation的全场位置接口。
原直线速度、坡道限速及加速时间保持，连续转向减少中间停车；未引入1.10倍速度提升。
显式开启移动Tag6后，接管需要两帧稳定目标，末端四帧与停稳确认才消费原Tag/偏置步骤。
无可靠Tag时保留原步骤；接管后长时间丢Tag、断链或取消则停止，禁止重复回放。
目标偏置与建筑目标保持区分；优化模式最终建筑视觉不成功会阻止Build。

本次文档整理**协议未变**：旧 0x10/8 B、0x50/6 B、0x80/80 B、0x83/11 B，
已有扩展 0x52～0x55 / 0x84～0x86、数值大端/CRC 小端、200 ms 失联与会话期限均保持。
配置启停只改变上位机选择；修改固件内部动作或控制实现后，按 CLion 流程烧录。
实机前仍需完成下节规定的 Debug 编译验证。

## 验证与启动顺序

树莓派桌面仅保留 **PlanA、PlanB** 两个机器人测试入口。
单 Task、set1/set2 和 collect-build-1/2 仍可通过终端命令运行。
PlanA/set1/set2 包含 task0-1，PlanB 从 task0-2 开始；collect-build 只运行 task2 → task3。
打开策略图标即运行。终端使用 `desktop_task.sh task3-1` 等启动独立 task3/task4 时，
脚本先要求填写已标定的航向零点；留空或无效值不会连接机器人。
二者均从 Task2 结束位置、航向 180° 起步，分别准备好 Build 或舱内投放，不能把当前朝向当零点。
终端实时显示日志；按 Ctrl+C 停止，结束后按回车关闭窗口。
入口使用项目 `.venv/bin/python` 和 `tools/desktop_task.sh`，共用防重复启动锁；
该锁仅约束桌面入口，不约束手动命令。
可用 `python tools/install_desktop_entries.py` 重新安装两个入口并清理其管理的旧任务图标；
`--dry-run` 只查看入口内容及待清理列表。
`bash tools/desktop_task.sh <任务或策略包> --show-plan` 只预览流程，不连接硬件。

1. 检查工作区状态、当前固件版本和机构起始位置。
2. 运行 Python 语法、导入和无硬件测试。
3. 在开发电脑的 `Uniforest_A/` 执行 `cmake --preset Debug`、`cmake --build build/Debug`；树莓派不部署该源码目录。全量单元测试在仅有上位机的部署中明确跳过依赖下位机源码的检查（当前 10 项），其余检查照常执行；源码比对与固件宿主测试在开发电脑执行。
4. 由用户通过 CLion 的 OpenOCD + DAPLink 配置烧录需要更新的固件。
5. 检查串口、相机角色、标定文件和预检结果。
6. 小范围动作测试通过后再进入完整任务；通信异常、遥测陈旧或急停后不自动续跑。

无硬件验证，在 `RaspberryPi/` 执行：

```bash
python tools/check.py
```

这个入口检查 Python 语法、依赖导入、协议格式、数据采集、策略及衔接/连续路线，成功时显示简短
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
python main.py --strategy PlanA
python main.py --strategy PlanB
python main.py --strategy set1
python main.py --strategy set2
python main.py --strategy collect-build-1
python main.py --task task1-1
python main.py --task task2-2
```

任务 ID 为 `task0-1/2/3`、`task1-1/2/3`、`task2-1/2`、`task3-1/2/3`、`task4-1/2`、`task5`（斜杠分隔的编号代表独立 ID）。
无参数运行等价于 `--strategy PlanA`。完整流程：

`PlanA: task0-1 → task1-1 → task2-1 → task3-1 → task1-2 → task2-2 → task3-2 → task1-3 → task2-2 → task3-3`

`PlanB: task0-2 → task2-1 → task4-1 → task2-2 → task4-2 → task0-3 → task1-1 → task0-3 → task1-2 → task5`

Task0-2 以 2000 mm/s 前进 900 mm、右移 2700 mm，两段均使用 1600 ms 加速。
右移结束后以 120°/s 转到相对启动零点的 180°，再进入 Task2。
Task0-3 从 Task4/Task1 的 180°结束朝向进入：先转到 0°，以 2000 mm/s 左移 2600 mm，
再以 300 mm/s 向左顶墙，随后交给 Task1；定距左移加速 800 ms，顶墙复用堵转检测和 4 秒上限。
独立运行 `main.py --task task0-3` 时也需从相应位置以 180°摆车。
Task5 接在 PlanB 的 Task1-2 后，以 180°进入，执行两次开舱顶墙取块、建筑对准和 Build。
开舱各等 200 ms，关舱各等 400 ms，再以 400 mm/s 各后退 250 mm；每次 Build 第三块释放后，先以 400 mm/s 后退 100 mm，再执行原左移路线，与机构收尾并行。详见策略说明。
Task5 恢复指定路段提速：起步左移 700 mm、两次右移 940 mm 及 Build 后左移 840/440 mm 为 2000 mm/s，以上加速均为 1600 ms。后退 250/100 mm、中间右移 300 mm 保持 400 mm/s、300 ms 加速。
Task4 初始航向 180°：2000 mm/s 左移 600 mm、300 mm/s 左顶墙；Task4-2 再以 400 mm/s 右移 300 mm。
随后两版均 300 mm/s 前顶墙，开舱等待 300 ms，400 mm/s 后退 300 mm、关舱不等待；
2000 mm/s 分别右移 800/500 mm 后保持 180° 结束；首尾长段加速 1600 ms，短段加速 300 ms。Task1 也取消末尾回零转向，
Task2 两版均从 180° 起步，以 1000 mm/s 后退 2500/2350 mm。详见 [策略说明](Strategy/README.md)，现场效果待验证。

`set1`/`set2` 分别运行 task0-1 和对应编号的 task1、task2、task3。
`collect-build-1`/`collect-build-2` 只运行对应 task2 → task3。
单 Task 不自动执行 Task0 或补齐后续动作。新 Task2 在前进 2750 mm 后结束，
Tag6 对准、建筑对准、Build 和返程属于 Task3。`task2_main.py --variant 1/2` 也只执行新 Task2。

只读查看任务库与顺序（不打开串口、相机）：

```bash
python main.py --list-tasks
python main.py --strategy PlanA --show-plan
python main.py --strategy PlanB --show-plan
python main.py --task task3-2 --show-plan
```

组合执行时 Task2 自动向 Task3/Task4 传递顶墙校准后的陀螺仪航向零点。单独运行二者
须先摆到 Task2 末段前进 2750 mm 后的结束位置、航向 180°，准备好搭建或舱内投放，并通过
`--heading-zero-deg` 提供已知的原任务航向零点；该值不是当前 yaw，也不是目标 180°。
缺少该参数会在连接硬件前拒绝执行；不得猜测数值。接口细节见策略说明。

旧 `classic/all` 都兼容 PlanA，旧 `task0` 兼容 task0-1，`round1/round2` 对应 set1/set2；旧 `task1/task1-r1/task1-r2`
映射到对应新 Task1。旧 `task2/task2-r1/task2-r2` 保留原完整范围，映射到
`collect-build-1/2`，并打印实际展开的任务顺序。新命令优先使用简洁 ID。

已连接树莓派并完成代码同步时，可以直接执行：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy PlanA
```

可选诊断日志：

```bash
python main.py --strategy PlanA --diagnostics-log /tmp/uniforest-run.jsonl
```

日志记录连接、定距请求/实际进度、耗时、任务状态和异常类型；默认不写日志。

## 底盘参数

### 平移速度与加速

| 场景 | 速度 | 加速 |
| --- | ---: | ---: |
| Task1 三个变体普通定距、Task2 短退与顶墙前进 250 mm | 400 mm/s | 300 ms |
| Task2 紫块后前进 / 橙块后横移：实际补偿距离 <500 mm | 400 mm/s | 300 ms |
| Task2 紫块后前进 / 橙块后横移：实际补偿距离 ≥500 mm | 2000 mm/s | 1600 ms |
| Task0、Task1 投放前进 | 2000 mm/s | 1600 ms |
| Task0-2 前进 900 mm、右移 2700 mm | 2000 mm/s | 1600 ms |
| Task4 起步左移 600 mm、末尾右移 800/500 mm | 2000 mm/s | 1600 ms |
| Task4 中间右移 300 mm、后退 300 mm | 400 mm/s | 300 ms |
| Task5 起步左移 700 mm、两次右移 940 mm | 2000 mm/s | 1600 ms |
| Task5 Build 后左移 840/440 mm | 2000 mm/s | 1600 ms |
| Task5 后退 250/100 mm、中间右移 300 mm | 400 mm/s | 300 ms |
| Task2 起步后退：第一轮 2500 mm、第二轮 2350 mm；两轮末段前进 2750 mm（坡道） | 1000 mm/s | 1000 ms |
| Task3-1/Task3-2/Task3-3 Build 后左移 2500/2200/3000 mm | 2000 mm/s | 1600 ms |
| 连续横向搜索方块 | 300 mm/s | 无额外软件起步斜坡 |
| 常规向前/向左顶墙 | 300 mm/s | 无额外软件起步斜坡 |
| 与抓取同时启动的短压墙 | 150 mm/s | 无额外软件起步斜坡 |
| 手动 `move` 定距命令默认值 | 1000 mm/s | 800 ms |

公共参数在 `control/chassis.py`：
`NORMAL_DISTANCE_MOVE_SPEED_MM_S=400`、`NORMAL_DISTANCE_MOVE_ACCEL_MS=300`、
`LONG_DISTANCE_MOVE_SPEED_MM_S=1000`、`LONG_DISTANCE_FORWARD_ACCEL_MS=800`。
比赛路线参数在 `Strategy/settings.py`：平地2000 mm/s、1600 ms；坡道1000 mm/s、1000 ms。
路线层按巡航速度缩放加速时长，Task2两段坡道显式使用1000 ms。通用手动默认值保持。
上坡2500/2350 mm整段禁止切弯：局部规划在坡口停稳，以两点固定航向参考直行，段尾停稳后才转向。
PlanB融合首程同样执行上坡保护，仍省去180°中间掉头；这里采用原整段路线范围，实际坡脚/坡顶待现场确认。
新值为2026-10-03用户指定的软件试跑目标，尚未实机验证；短段与弯道可能无法达到巡航上限。

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
- 比赛定距路线：Task0 和 Task1/Task2/Task3/Task4 两套变体的定距调用显式启用
  `route_mode=True`，到位窗口按估计底盘距离为 **±8 mm**，横移按 `500/465` 同步
  换算编码器容差；不足 16 mm 的短补偿以半程为最大容差，避免在起点直接完成。
- 路线一旦进入窗口就将四轮速度设为零，清空位置/航向积分，停止追逐最后几毫米。
  滑出窗口则恢复有符号位置纠偏；只有位置合格且四轮转速绝对值均不超过
  **50 RPM**（轮速折算约 21 mm/s），通过原 50 ms 确认配置后才完成。
  控制周期为 20 ms，确认按新遥测推进，重复旧帧不累计；路线 `hold_ms=0`。
- 路线起步前馈取时间加速曲线和剩余距离减速曲线的较小值，短距离在加速阶段
  也能提前减速；位置 PID 增益、速度和加速时间不变。此轮仍为停车后进入下一段，
  常规转向保持 600 ms 加速，未加入连续转弯。
- 手动单次定位默认 `route_mode=False`，保留 1000 counts（约 3 mm）窗口、
  原前馈和位置锁定；视觉对准、顶墙控制不使用此到位窗口。
- 新路线模式检测连接、急停代次、0.5 秒遥测陈旧和发送失败，异常中止，不作为到位。
- 非坡道策略允许定距控制器超时但编码器进度达到 90% 以上的结果；受保护上坡必须完整到位，超时即中止。
  取消、遥测丢失和进度不足始终不适用进度放行。
- 直线总超时为 `max(2000 ms, 预计匀速行驶时间 + accel_ms + hold_ms + 2000 ms)`。
  已取消原来的 5 秒最低总时长；2 秒为估算行程后的额外余量，不是所有动作总共只运行 2 秒。
  例如前进 100 mm、400 mm/s、加速 300 ms、无额外保持时，总超时约 2.55 秒。
- 横移补偿系数当前为 `500/465`，来源于先前地胶测试记录。换场地或负载后需复测左右方向及不同距离，不能把编码器位移当作无滑移的实际位移。

09-26 第一轮优化已定向同步树莓派，远端 67 文件语法、导入、协议、采集和路线控制检查通过。小范围测试通过现有 `Robot.move_chassis()` 传入
`route_mode=True, hold_ms=0, accel_ms=300`，比较后退 100 mm、转向、前进的末端停顿、
日志耗时及终点偏差；再进入对应单任务。无硬件检查只验证控制输出与异常处理，
不证明实机速度或停止距离。协议未变，无新增依赖，无需为此修改下位机固件。

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
Task1 卸载横移后保持 180°结束，交给 Task2；原有起始顶墙、橙色短压墙仍以当前姿态为 0°。
这是上位机航向基准更新，不发送陀螺仪硬件校准命令；顶墙超时接受后同样执行。
2026-09-25 已定向同步树莓派，两轮航向换算检查通过，现场效果待确认。

常规顶墙为 300 mm/s、最长 4 秒；与抓取同时启动的短压墙为 150 mm/s、最长 1 秒。前向顶墙检查两个后轮，侧向顶墙使用四轮中至少三轮，低速/高电流阈值及确认时间在 `FirstTaskConfig`。

2026-09-28 根据轻微打滑时经常等待超时的反馈，低速上限由 35 放宽到 **80 rpm**，
电流原始值仍须达到 **2500**，上述轮数要求保持。允许最长 **80 ms** 的短暂不满足，
期间暂停累计，恢复后继续；超过 80 ms 则重新确认，最终只在满足条件的新遥测帧上判定成功。
常规顶墙保留 0.5 s 起步保护和 0.3 s 有效确认时间；抓取短压墙保留 0.1 s / 0.15 s。
所有共用此控制过程的 Task 顶墙入口适用；无硬件回放通过，新阈值的现场效果待确认。
协议未变，通信异常、遥测陈旧和急停中止规则保持。

达到顶墙时间上限后先停车，默认按顶墙成功处理；`wall_timeout_is_success=False` 可启用严格超时报错。遥测丢失等通信故障始终终止任务。该超时容错与 A 板 200 ms 通信失联急停不同。

## 任务库流程

### task1-1 / task1-2 / task1-3

三个变体共用 `Strategy/competition.py` 状态机，稳定导入入口为 `Strategy/task1.py`。

1. 以 300 mm/s 向前顶墙并重新标定当前航向零点。
2. 以 300 mm/s 连续向右搜索橙色，累计搜索上限 1800 mm；锁定目标后进行粗对准和末端微调。
3. 对准后同时启动 Grap3 和 150 mm/s 短压墙，短压墙结束后重新校准航向；抓取后执行数量检查及补抓。3 或 null 继续，搜索距离耗尽则跳过检查。
4. 以 400 mm/s 后退 400 mm，转到启动零点顺时针 90° 航向。
5. 以 2000 mm/s 前进 `2800 mm - 抓取阶段编码器实测净右移量`，再转到 180° 航向。
6. 对准 Tag6 至距离 425 mm、横向零点；前后/横向单轴进入 ±8 mm 即停止该轴平移，两帧超差后重启。航向独立向 180° 纠偏，0.5° 死区；前后、横向和航向 ±3° 均合格连续确认 4 个新帧后停车继续，不执行精对准。按变体定距横移后顶墙卸载。
7. 打开舱门并等待 300 ms，以 400 mm/s 后退 300 mm，关闭舱门后不等待；按变体定距横移后以 180° 结束，取消末尾转回 0°。

Task1-1/2 的 Tag6 后右移、投放后左移分别为 100/400 mm；Task1-3 改为
Tag6 后左移 500 mm、投放后右移 500 mm，两段均为 400 mm/s、300 ms 加速。

转向额外保持已设为 0；不恢复历史文档中的 500 ms 等待。橙色目标持续丢失 0.5 秒返回搜索，通信故障中止任务；橙色粗对准超时按下文规则直接抓取。

两轮 Task1/Task2 的常规路线转向前馈速度为 **120°/s**，起步加速 **600 ms**。
现有转向控制器按速度比例缩放，减速区为剩余 **40°**，位置 PID 修正限幅
**±80°/s**；前馈叠加修正后指令可能超过 120°/s。到位容差仍为 **±1.5°**。
这不改变视觉对准的转向速度配置。

### task2-1 / task2-2：采集与转场

对应原 Task2 的第 1-14 步，代码在 `Strategy/task2.py`：

1. 从 180° 起步，以 1000 mm/s、1000 ms 加速后退 2500/2350 mm，再转到 -90°。入口按 `零点 = 当前 yaw + 180°` 换算，不把当前朝向设为零。当前跳过 Tag3 对准及其后横移。
2. 前进 250 mm，再以 300 mm/s 向前顶墙。
3. 向左搜索紫色，搜索上限 750/650 mm；找到后执行 Grap2 和短压墙。找不到则跳过紫色，初始橙色数量从 2 个增至 3 个。
4. Grap2 回升到 5 cm 且短压墙结束后，机构收尾与后退 100 mm、转回 0°、补偿前进并行，保持机构监视。Task2-1 前进 `350 mm - 紫色阶段实测净右移量`；Task2-2 前进 `500 mm - 紫色阶段实测净右移量`。
5. 向左顶墙、向前顶墙，重新校准航向。
6. 向右搜索橙色，累计上限 1800 mm，使用 Grap1；执行原数量检查及补抓流程，搜索耗尽则跳过数量检查。
7. 后退 100 mm（可与数量检查复位重叠），再按 `550 mm - 橙色阶段实测净右移量` 横移补偿。
8. 转到 180°，以 1000 mm/s、1000 ms 加速前进 2750 mm，结束并输出航向交接数据。这里不执行 Tag6 或 Build。

Task2 的单次执行返回时，此前 Grap2/Grap1 和数量检查均已完成，不留跨 Task 的后台机械动作。
紫块后前进、橙块后横移按补偿计算后的实际距离选速：≥500 mm 为 2000 mm/s、1600 ms 加速，
不足 500 mm 为 400 mm/s、300 ms 加速；橙块后横移取补偿绝对值，方向沿用原计算，零距离跳过。
`Task2Config.tag3_alignment_enabled=False` 保持；如显式启用，原 Tag3 目标和控制参数保留，
task2-1 对准后右移 100 mm，task2-2 不执行这段横移。

### task3-1 / task3-2 / task3-3：搭建与返程

对应原 Task2 的第 15-17 步，代码在 `Strategy/task3.py`：

1. 消费 Task2 交接的航向零点，不把当前 180° 朝向重新设为 0°。
2. 对准 Tag6：距离 425 mm、横向 0 mm，两平移轴容差 ±8 mm，合格平移轴停止。航向独立向 180° 纠偏，0.5° 死区；三轴均合格（含航向 ±3°）连续 4 个新帧后完成，无精对准。
3. Task3-1/2 分别右移 100/400 mm，Task3-3 左移 500 mm，速度均为 400 mm/s；再用方块相机对准实际建筑。沿用建筑丢失/超时停车告警后继续 Build 的既有分支，通信及取消异常仍终止。
4. 执行 Build。最后一次释放后，机构收尾与返程并行：后退 100 mm → 顺时针相对转 180° → 左移 → 左顶墙。
5. 三个变体都等机构和返程全部完成后结束；task3-1 左移 **2500 mm**，task3-2 左移 **2200 mm**，task3-3 左移 **3000 mm**，速度均为 2000 mm/s、加速 1600 ms。

| 任务差异 | `-1` | `-2` |
| --- | ---: | ---: |
| Task1 Tag6 后右移 / 投放后左移 | 100 / 100 mm | 400 / 400 mm |
| Task2 起步后退（入口航向 180°） | 2500 mm | 2350 mm |
| Task2 紫色向左搜索上限 | 750 mm | 650 mm |
| Task3 Tag6 后右移 | 100 mm | 400 mm |
| Task3 Build 后左移 | 2500 mm | 2200 mm |
| Task3 Build 后顶墙 | 向左 | 向左 |

2200 mm 是 2026-09-26 用户指定的软件路线参数，已同步，现场效果待确认；没有新增下位机动作。
Task1-3、Task3-3 为 2026-09-30 用户指定变体；PlanA 在原程序后追加
task1-3 → task2-2 → task3-3，第三套复用原 Task2-2。新衔接位置与路线现场待确认。
Task1/Task3 普通定距仍为 400 mm/s、300 ms 加速；Task2/Task4/Task5 分路段参数见上表。不执行 Build 后 Tag1 对准。

### Tag6 对准

Task1、Task3 各自三个变体使用下列参数。位置采用 **3 帧中值滤波**，按原 50 ms
节奏仅用有效新帧更新（20 ms 调度会量化单次间隔）。航向使用最新陀螺仪数据，
独立按 **20 ms，约 50 Hz** 更新，不等待视觉新帧；两任务均取消精对准。

| 参数 | Task1 三个变体 | Task3 三个变体 |
| --- | ---: | ---: |
| 距离目标 / 容差 | 425 / ±8 mm | 425 / ±8 mm |
| 横向目标 / 容差 | 0 / ±8 mm | 0 / ±8 mm |
| 航向目标 / 容差 | 180° / ±3° | 180° / ±3° |
| 前后 / 横向最高速度 | 350 / 280 mm/s | 260 / 200 mm/s |
| 平移最低非零期望速度 | 100 mm/s | 100 mm/s |
| 前后减速区 / 近距离区 | 100 / 25 mm | 140 / 35 mm |
| 横向减速区 / 近距离区 | 80 / 20 mm | 100 / 25 mm |
| 平移加速度限制 | 300 mm/s² | 300 mm/s² |
| 航向最低 / 最高速度 | 无最低限幅 / 45°/s | 无最低限幅 / 45°/s |
| 航向控制死区 | ±0.5° | ±0.5° |
| 转向加速度限制 | 90°/s² | 90°/s² |
| 视觉过期阈值 / 丢失超时 | 0.3 / 1 秒 | 0.7 / 2 秒 |
| 总对准超时 | 12 秒 | 12 秒 |

两任务共用增益（位置误差单位 mm，航向误差单位 °）：

| 控制轴 | Kp | Ki | Kd |
| --- | ---: | ---: | ---: |
| 前后 | 0.8 / 0.300549527 ≈ 2.66179 | 0 | 0 |
| 横向 | 1.2 / 0.300549527 ≈ 3.99269 | 0 | 0 |
| 航向 | 6 | 0 | 0 |

前后、横向进入自身容差后立即停止该轴并清零其 PID 和速度状态，跳过软件减速；
连续两个有效新帧超差才重新启动。航向不使用 ±3° 轴锁定或两帧超差重启：
±3° 仅用于完成判定，±0.5° 才停止旋转；更小输出不再强制升至 8°/s。
三轴同时合格连续四个有效新帧后完成，确认期间继续航向维持，完成后四轮零速。
视觉帧间航向超过 ±3° 也会清空确认计数。目标仍基于已有校准零点的 180°。
视觉无效、过期或跳变时所有轴停车并中断确认，拒绝的缓存帧不得恢复旋转；
有效缓存帧只更新航向，不更新平移和完成计数。最低平移限幅在加速度限制之前，起步下发
速度可以低于 100 mm/s；合格轴允许零速。比赛调用在视觉丢失超时或总超时后记录
降级并继续原定路线，不终止整场程序；通信、遥测、取消及机构故障仍中止。

09-26 独立航向试调已同步树莓派并通过无硬件检查，现场效果待确认，记录见 CHANGELOG。
协议未变，无新增依赖或下位机固件改动。测试入口沿用 task1-1/2、task3-1/2 及相应
策略包，独立 Task3 仍需已知入口航向零点。当前关闭的 Tag3 保留原 1.5/0.02/0.03
航向 PID、8°/s 最低转速及视觉更新节奏；建筑对准也不启用本次独立航向模式。

### 搜索距离与回找

task1-1/2/3、task2-1/2 在橙色搜索累计达到 1800 mm 时，即使数量不足（包括零块），
也结束抓取并继续对应的投放或转场路线。搜索距离或累计搜索时间耗尽均降级继续；
其他通信、取消或动作异常仍按原故障流程处理。橙色预算只累计正常向右搜索速度乘
实际指令时间，不包括停车确认、左移回找和视觉对准，抓取单块后不重置；后续路线
补偿仍使用编码器实测净横移。到极限时已出现候选，允许停车完成确认。
紫色累计向左搜索上限为第一轮 750 mm、第二轮 650 mm。

橙色左缘回找（09-27 接通统一链路，09-30 修复过滤及起点资格，适用 task1-1/2/3、task2-1/2）：没有有效橙色候选，且有效 ROI
内面积足够的橙色簇连续 3 个新帧触及画面左边界时，先停 100 ms，再以 200 mm/s
向左寻找真实左缘。候选出现后停车确认，再沿用对准、抓取流程。左半屏普通橙色
像素、右侧裁剪和重复帧不触发回找；Task2 橙色仍沿用原 ROI。
截断提示在正常目标的宽高比、相对面积、ROI 边界及角点拟合过滤之前独立提取：左侧
窄条、横贯左右边界的整排、接触 Task2 ROI 上沿的截断仍可提示。保留最小面积及 ROI
有效范围；普通单侧顶/底边干扰仍过滤，全宽整排允许接触画面顶/底边。提示不提供
可抓坐标，正常目标的坐标过滤保持原样。

单次回找最多 1000 mm、6 s，并受橙色阶段编码器起点限制（预留 10 mm 加一个控制
周期行程，当前约 14 mm）。起点附近不能左移时继续右搜，不消耗回找资格，也不因
等待截断确认停在起点；进入允许左移的范围后再尝试。同一连续截断区域实际开始
回找后只尝试一次；再触发要求连续 3 个新帧不再截断，且净
右移超过上次触发位置 100 mm。回找耗尽、到起点或目标连续消失后停车再继续向右；
图像失效时放弃本次回找，继续原右移搜索预算，不用旧画面确认目标；遥测失效、
通信发送失败或搜索期间本地急停仍终止任务。参数尚未实机确认，
起点约束不代表消除了打滑和惯性。协议未变，无需重新烧录。

整个橙色采集阶段还累计搜索控制耗时，跨抓取/补抓共享，但不累计机械动作与视觉对准
耗时。上限由已有参数推导：1800/300 + 6 + 3×0.35 = 13.05 秒。耗尽后跳过剩余抓取，
继续原路线，避免反复出现候选/回找导致无界等待。此时间限额为 09-27 软件设置，现场待验证。

### 比赛视觉降级

用户 2026-09-27 确认所有比赛视觉环节失效时继续后续流程。相机启动失败不再阻止
Task0 或任务预检；方块搜索走完原预算，未找到则跳过抓取。目标丢失返回搜索，粗对准
超时直接抓取的既有规则保留。Tag/建筑对准在现有丢失或总超时后结束该视觉环节，继续
原路线和 Build；暂停无依据的视觉纠偏及正常换向零速不等同于终止比赛。
携带数量检查无图时返回未知；摆出检查姿态后丢帧则先按原时序复位，再结束检查，
后续底盘与复位的既有受监视并行保持。独立数量诊断入口仍采用严格故障处理。
降级会输出原因，视觉未知不作为“已抓到/已对准”的证明。通信、遥测、急停、机构故障
不会被视觉容错捕获，也不会自动续跑。新增无硬件检查为 tests/test_visual_fallback.py。

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

正式 Task3 对建筑目标丢失/对准超时会先停车告警后继续 Build；底盘通信等其他异常仍终止任务。

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
- 距目标 150 mm 及以上时期望速度 500 mm/s，30–150 mm 间随距离收缩；
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

采用横向优先控制；前后误差进入 30 mm 内使用 60–90 mm/s 减速带。轮廓锁定、跳变检查、帧新鲜度和超时参数集中在 `Task3Config.building_*`。顶边行标定比例为 `building_z_scale_mm_px`，当前表达式 `132.8 * 82.4`；改变现场条件后通过 `tools/building_vision_probe.py` 复测。

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
