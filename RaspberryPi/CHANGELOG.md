# 变更与验证记录

当前操作方法与参数以 [README.md](README.md) 和源码为准。本文件保留变更日期、
适用范围及当时验证结果；历史通过记录不代表当前全量测试通过。

## 2026-09-07

### 第一轮 Build 后退距离

`Task2Config.post_build_reverse_mm` 从 200 改为 100 mm，速度仍为 400 mm/s、加速
300 ms；第二轮仍在 Build 后结束。对应更新路线测试和 README。协议未变，无需
重新烧录。按用户要求，简单参数修改仅做针对性验证，本次使用
`tests.test_task2.Task2Tests.test_partial_task_order_and_parameters`。
现场入口：`python main.py --task task2-r1`；新增距离尚未实机确认。

### 整轮测试包含 Task0

`round1` 改为 Task0 → Task1-R1 → Task2-R1，`round2` 改为 Task0 → Task1-R2 → Task2-R2。
模块：`Strategy/runner.py` 编排表和 `main.py` 帮助；测试/文档：`test_task_runner`
覆盖两轮顺序、完整流程只运行一次 Task0、单任务跳过 Task0、Task0 失败阻止后续
任务，双方 README 同步入口说明。Task0 参数仍为前进 1200 mm、750 mm/s、800 ms
加速。协议未变：命令编号、载荷长度、字节序、80 字节遥测及急停路径未修改，
A 板 200 ms 通信失联停止不变；无需重新烧录，无新增依赖。
适用入口：`python main.py --task round1`、`python main.py --task round2`；
现场结果：按用户要求调整，尚未实机验证整轮起步。
本地验证：语法、导入、任务编排/Task0/协议 15 项及 A 板 Debug 构建通过。

### 紫色搜索距离分轮次调整

第一轮 Task2 紫色累计向左搜索上限改为 750 mm，第二轮改为 650 mm；由
`Task2Config.purple_search_max_distance_mm` 和 `Task2Round2Config` 分别设置。
对应更新 `test_task2` 的配置、完整路线和两轮搜索调用断言，以及 README 轮次表。
搜索仍为 300 mm/s，紫色上方 40% ROI、搜索耗尽跳过 Grap2 和后续路线补偿不变。
协议未变：双方命令编号、载荷长度、字节序、80 字节遥测和急停路径均未修改，
A 板 200 ms 失联保护不变，无新增依赖，无需重新烧录。
现场入口：`python main.py --task task2-r1` 或 `--task task2-r2`；
现场结果：按用户要求修改，尚未实机验证新增搜索距离。
本地验证：语法、导入、Task2/紫色 ROI/协议相关 36 项及 A 板 Debug 构建通过。

### 紫色 ROI 与测试流程整理

| 行为变化 | 受影响模块 | 测试/文档 |
| --- | --- | --- |
| 两轮紫色搜索/对准屏蔽上方 40%，只检测下方 60% 的紫色目标 | vision/cube_detector.py 新增 task2_purple；Strategy/task2.py 自动切换并在 finally 恢复 default | 两种分辨率的上下目标隔离、下方目标坐标不变、缓存清除和成功/耗尽/异常恢复 |
| 紫色搜索耗尽与橙色共用 SearchRangeExhausted，移除按异常文本判断的分支 | Strategy/task2.py | 紫色跳过抓取及普通异常传播 |
| 语法、导入、全量单测统一到 tools/check.py，可选固件编译和 C 动作测试 | tools/check.py、tests/import_smoke.py | 完整运行验证退出码；导入失败不再返回成功 |
| 清理无调用的旧视觉示例及不参与构建的主程序副本 | vision/comp1.py、Uniforest_A/Core/Src/main.c.bak | 当前调用链及 CMake 构建核对；历史版本仍在 Git 中 |

ROI 在 640×480 时排除第 0–191 行，坐标仍以原图为准。Task2 橙色 50% ROI、
1800 mm 橙色搜索上限、600 mm 紫色搜索上限、颜色阈值、曝光与机械动作保持当前
配置。协议未变：核对 schema v2、双方命令编号、载荷长度、字节序、80 字节
完整遥测与急停路径，200 ms 失联停止不变；无需重新烧录，无新增依赖。

测试整理中修正了与当前代码不符的 Task1 标签参数/横移路线断言、平滑后复制对象
的身份断言，以及紫色搜索只给一帧却要求完成多帧确认的测试输入；未调整运行参数
或删除测试来消除失败。本地完整入口发现 138 项，剩余 11 个橙色几何断言失败
（含子测试）、0 个错误；语法、导入和 A 板 Debug 构建通过。

现场入口：`python vision/cube_detector.py --camera cube --no-gui --profile task2_purple`
只读观察，之后按 README 的预检/单动作/任务顺序验证两轮 Task2。
现场结果：ROI 已通过合成图像测试，尚未完成机器人运动中的现场确认。
清理本轮重复临时验证脚本、同步清单、日志和缓存；标定图片、已返回的相机画面及
历史备份保留。无硬件检查今后统一使用 README 中的 `python tools/check.py`。

树莓派同步校验 16 个文件；21 个历史副本移至
`/home/uniforest/.uniforest-sync/20260907-183823/retired/`，包含旧顶层模块/测试和
6 个仅用于旧 orange_new 引擎的测试，不再混入当前测试发现。远端复跑 138 项，
同样为 11 个橙色几何失败、0 错误；C 动作 2 项通过。紫色 ROI/Task2 专项 31 项
通过。对 18:23:23 的 cube 原图回放，启用前后均识别到一个紫色目标，
X=+7.8 mm、Y=-135.8 mm、Z=273.6 mm，确认该帧下方目标坐标未因 ROI 改变。

### 橙色搜索上限再次修订

按用户要求，两轮 Task1/Task2 橙色累计搜索上限从 1900 改为 1800 mm。
受影响模块为 `Strategy/competition.py` 公共配置；对应更新
`test_search_exhaustion`、`test_competition_task1` 和 README。
搜索耗尽后仍继续后续路线，紫色 600 mm 与直线超时参数保持当前配置。
协议未变：双方命令编号、载荷长度、字节序、80 字节遥测和急停路径均未修改，
无需重新烧录。测试入口为两轮 `main.py --task task1-r1/task1-r2/task2-r1/task2-r2`
（实际选择一个任务名）；现场结果：尚未实机验证搜索距离。

### 橙色搜索及直线超时后续修订

| 行为变化 | 受影响模块 | 测试/文档 |
| --- | --- | --- |
| 两轮 Task1/Task2 橙色累计搜索上限由 1500 改为 1900 mm | Strategy/competition.py 的公共配置，Task2/第二轮继承 | test_search_exhaustion；README 两轮路线 |
| 搜索耗尽时不因橙色数量不足终止任务，按已抓取数量（含零）继续卸载/Build | competition.py、task2.py；专用 SearchRangeExhausted 异常 | 两轮不足/零块、紫色有无、净位移补偿和普通异常仍急停 |
| 直线最低总超时从 5000 改为 2000 ms，估算行程后的额外余量从 1000 改为 2000 ms | control/chassis.py | test_chassis_position 虚拟时钟卡住/停车测试 |

实际总超时为 `max(2000, 行程/目标速度 + 加速 + 保持 + 2000)`，各项换算为毫秒。
短距离不再被原来的 5 秒下限拖长；长距离仍保留完成行程所需的时间。例如前进
100 mm、400 mm/s、300 ms 加速、hold=0，总超时约 2.55 秒。旋转参数未调整。
搜索速度仍为 300 mm/s，紫色上限仍为 600 mm；橙色预算按搜索时间累计，
不将视觉对准位移计入预算，后续净横移补偿继续使用编码器实测结果。

协议未变：核对双方命令编号、载荷长度、字节序、80 字节完整遥测、动作状态和
急停路径；schema v2 与 A 板 200 ms 失联保护不变。本次无需额外烧录或新增依赖。
适用轮次：两轮 Task1/Task2，直线超时适用于所有上位机位置环平移。
现场入口：`main.py --task task1-r1`、`task1-r2`、`task2-r1`、`task2-r2`；
直线单项入口 `tools/chassis_distance_test.py`。现场结果：尚未实机确认。

本次本地验证：搜索/位置环/协议 20 项通过；全量 135 项仍为原有 15 个失败
（含子测试）和 1 个错误，失败列表未变。语法、import_smoke、A 板 Debug 配置/
构建通过，未执行实机动作。无硬件回归入口：

```bash
python -m unittest tests.test_search_exhaustion tests.test_chassis_position tests.test_protocol_schema -v
```

### 行为与参数

| 行为变化 | 受影响模块 | 测试/文档 |
| --- | --- | --- |
| Grap1/2/3、Build 从 Python 迁移到 A 板非阻塞动作表，保留原有机械输出和等待 | control/actions.py、robot.py；A 板 actions.c/.h、main.c、CMakeLists.txt | test_actions_build、test_actions_grap3；C 原生时序及安全测试；双方 README 和 PROJECT |
| 新增带请求编号的动作启动、查询与状态接口；运行时忙碌保护，取消后不续跑 | protocol/schema.json、commands.py、transport.py；A 板 protocol.h/.c | test_protocol_schema；动作客户端故障与状态测试 |
| 第二轮 Task2 橙色净右移目标由 500 mm 改为继承第一轮的 700 mm，仍扣除编码器实测净右移量 | Strategy/task2.py | test_task2 的第二轮配置与正反补偿断言 |
| 两轮 Task1/Task2 普通定距平移统一为 400 mm/s、300 ms 加速 | control/chassis.py；Strategy/competition.py、task2.py | 两轮路线、四方向加速参数传递及协议测试 |

动作接口采用 schema v2：启动 `0x50` 为 6 字节 `>IBB`，查询 `0x51` 无载荷，
状态 `0x83` 为 11 字节 `>IBBBI`。命令数据按大端编码，帧 CRC 小端；旧命令和
80 字节完整遥测布局不变。A 板 200 ms 失联阈值不变，失联处理扩展为停止双步进、
吸盘并取消动作。客户端每 50 ms 查询状态，接口/启动确认上限 1 秒，状态陈旧
500 ms 报错急停。单段步进上限 30 秒，整套动作上限 120 秒。

机械动作适用于两轮和独立测试。Grap3 当前下降/返回段为 9 cm，步进完成稳定等待
100 ms，Grap 测试模式末尾等待 1000 ms。完整机械参数以 A 板动作表为准。
测试入口：`python action_test.py grap1`、`grap2`、`grap3`、`build`（每次选择一个）。
现场结果：未执行烧录或机械动作，需用户通过 CLion 更新固件后确认。

普通定距参数覆盖短距前后移动、标签后横移、橙色净位移补偿和紫色抓取后补偿前进。
750 mm/s 长距离动作仍用 800 ms 加速；搜索、顶墙、视觉及键盘参数未调整。
手动 `move` 默认仍为 750 mm/s、800 ms。700 mm 是橙色阶段的净位移目标，
不是抓取结束后固定再走 700 mm。

上述路线/速度调整本身协议未变：已核对双方命令编号、载荷长度、字节序、遥测
和急停路径。它们不另需固件修改，但同批机械动作迁移必须配套新固件。
适用入口：`python main.py --task task1-r1`、`task1-r2`、`task2-r1`、`task2-r2`。
现场结果：按用户要求修改，尚未实机确认。相关上位机代码已同步树莓派并核对文件哈希；
同步不表示已烧录或完成实机测试。

### 验证状态

本地环境为 Python 3.12.14、NumPy 2.3.5、OpenCV 5.0.0。

| 检查 | 结果 |
| --- | --- |
| Python 语法与 tests/import_smoke.py | 通过 |
| 动作客户端及协议相关 17 项 | 通过，使用下方列出的三个测试模块复跑 |
| 普通平移/路线及协议相关 11 项 | 通过，与上一组有重叠，不可直接相加 |
| 本地全量 Python 130 项 | 15 个失败（含子测试）、1 个错误，与本轮参数修改前列表相同 |
| A 板 cmake --preset Debug 与 cmake --build build/Debug | 通过，仅编译，未烧录 |
| 树莓派原生 C 动作测试 2 项 | 通过，含 7 组输出/等待时序及逐阶段取消、忙碌、超时、重复请求、时钟回绕 |

当前 Python 失败范围：Task1 路线顺序、长搜索停止和参数断言；橙色簇旋转、单块/
移动相机、紧贴三块及间隙子测试；tracker 对象身份断言。错误为 Task2 的
`test_purple_search_commands_left_lateral_motion`。这些问题未在本次文档整理中修复，
也未通过恢复已取消的等待或修改实机流程来使测试通过。

树莓派全量发现 144 项，存在相同 15 个失败和 5 个错误；额外 4 个导入错误来自
远端历史测试文件 `test_engine_dispatch`、`test_orange_new_adapter`、
`test_orange_new_merge`、`test_probe_offline_stats`，不属于当前本地检出。
远端执行发现测试时使用 `-s tests -t .`，避免顶层同名历史模块干扰。

可复现的无硬件入口，在 RaspberryPi 目录执行：

```bash
python -m compileall -q control protocol Strategy robot.py main.py
python tests/import_smoke.py
python -m unittest tests.test_actions_build tests.test_actions_grap3 tests.test_protocol_schema -v
python -m unittest discover -s tests -t . -v
python ../Uniforest_A/tests/run_actions_host.py
```

最后一项需原生 `cc`，详情见[下位机说明](../Uniforest_A/README.md)。本日另整理
根目录导航、上位机操作说明及下位机协议文档，移除过期参数和互相矛盾的流程描述。

## 2026-09-06

| 行为变化 | 受影响模块 | 测试/文档 |
| --- | --- | --- |
| 第一轮 Task2 Build 后改为后退 200 mm、顺时针转 180°、左移 2500 mm、左顶墙；第二轮不执行 | Strategy/task2.py | test_task2 路线顺序、轮次隔离和故障急停 |
| 第二轮 Task1 补偿基准由 2500 mm 改为继承第一轮的 2800 mm | Strategy/task1.py | test_competition_task1 第二轮补偿路线 |
| 根据用户反馈，将 750 mm/s 左移加速从 200 ms 改为 800 ms | Strategy/competition.py | 高速前进/左右移动加速参数传递 |
| 第二轮恢复左顶墙，两轮共用左顶墙、前顶墙、航向校准流程 | Strategy/task2.py | 两轮顶墙方向、顺序、速度和超时 |
| 位置到位窗口从 400 counts/20 RPM 改为 1000 counts/50 RPM，连续确认仍为 50 ms | control/chassis.py | test_chassis_position 到位、轮速及保持测试 |

Build 后左移速度为 750 mm/s，左顶墙 200 mm/s、最多 4 秒。该日低速横移的
200 ms 加速已被 09-07 的 300 ms 覆盖。协议未变：核对双方命令、载荷、字节序、
80 字节遥测和急停路径，无新增依赖；A 板仍执行 200 ms 通信失联保护。

适用入口：两轮 Task2、第二轮 Task1，以及 `tools/chassis_distance_test.py`。
现场结果：用户报告旧的左移加速过快；800 ms 加速、第二轮顶墙及放宽的到位窗口
尚待实机确认。相关 17 项测试、语法、导入、A 板 Debug 配置/构建通过。
当日全量从 122 项增至 124 项，均为原有 15 个失败（含子测试）、1 个错误。

## 2026-09-05

橙色检测切换为顶面簇左首方块定位，保留毫米单位和 X 右/Y 上/Z 前坐标接口、
Task2 ROI 与策略目标跟踪。当前最终实现采用 `orange_fixed_geometry.py` 内嵌的
Task1/Task2 独立固定投影；JSON 是标定记录，不是运行时加载来源。
此前“逐帧重新估计平面”的说明已过期。两任务橙色 X 偏置当前分别为 +5.0 mm，
不影响紫色和建筑。无新增依赖，协议未变。

模块：orange_cluster、orange_config、orange_fixed_geometry、cube_detector；测试：
test_orange_clusters。入口：`tools/cube_profile_probe.py IMAGE --profile default`
或 `--profile task2_orange`，以及 `vision/cube_detector.py --camera cube --no-gui`。
现场结果：尚未在比赛机器人上验证固定几何与各种间隙/遮挡情况。

当日早期检测版本记录为新增 7 项识别测试通过，全量 121 项有 5 失败、1 错误；
这是历史结果，后续固定几何及当前依赖环境应以 09-07 验证状态为准。

## 2026-09-04

cube Linux 相机设为手动曝光 312、增益 32、自动白平衡。建筑对准切换到包含
亮顶面的 building 色带，目标 X=0/Z=75 mm，横向优先并使用近距离减速带；
Purple 色带记录为 H105-155。模块：camera_settings、检测 profile、Task2Config；
入口：`tools/building_vision_probe.py`、`building_build_test.py`。
适用两轮 Task2，协议未变；现场记录不足以保证不同光照下的效果，改变曝光/安装
后须复测。当前建筑比例以源码 `132.8 * 82.4` 为准。

## 2026-09-03 及更早

- 09-03：Tag6 附近横移统一为第一轮 100 mm、第二轮 400 mm；Tag 对准采用距离、
  横向和航向闭环；新增建筑对准加 Build 独立入口。协议未变。
- 08-25：视觉目标集中至 Strategy/vision_targets.py；三种方块 X 目标均为 0，
  橙色粗调 [-20,5]/微调 [-3,3] mm，紫色粗调 [-5,5]/微调 [-3,3] mm。协议未变。
- 08-23：增加目标跳变/歧义保护、橙色微调和定距超时后编码器进度至少 90% 的策略
  容错，取消部分额外等待；不豁免遥测丢失、取消或 A 板 200 ms 失联急停。协议未变。

更早的记录不替代当前源码或现场确认。完整提交历史见 Git，备份快照默认只读。
