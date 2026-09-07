# 变更与验证记录

当前操作方法与参数以 [README.md](README.md) 和源码为准。本文件保留变更日期、
适用范围及当时验证结果；历史通过记录不代表当前全量测试通过。

## 2026-09-07

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
