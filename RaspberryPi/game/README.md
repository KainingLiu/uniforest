# game 上位机运行指南

本文件只说明 game 实现的启动、优化开关、日志和验证要求。

2026-10-03：此处 game 指主分支内嵌模块。默认固定盲移期望 100 mm；自适应连续块排 100 mm，
已定位下一块最多 200 mm，未见或未建立下一块预观测时搜索 200 mm。内置试跑速度保持 160 mm/s，
固定/自适应包络分别 120/220 mm，均含 20 mm 制动余量。现场文件显式参数与硬上限保持。
这些距离为待实机确认的期望或上限；提前视觉接管可缩短实走距离。后退检查及返回限制见
[动作衔接](Strategy/TRANSITIONS.md#后退检查与少块返回当前源码核对)。

## 选择 game

在树莓派的 `RaspberryPi/game/` 目录直接启动 game：

```bash
cd /home/uniforest/Uniforest/RaspberryPi/game
source ../.venv/bin/activate
python main.py --strategy PlanA --show-plan
python main.py --strategy PlanB --show-plan
```

确认预览与车辆起始状态后，去掉 `--show-plan` 才执行动作。
同样可以使用 `python task2_main.py --variant 1 --show-plan`；
`python robot.py --help` 查看 game 的调试入口。最新主代码已移除上层 `--runtime game` 转发入口。

## 本文其余命令的执行目录

下面的优化和工具示例都在 **game 实现目录内**执行，可直接使用该目录的入口：

```bash
cd /home/uniforest/Uniforest/RaspberryPi/game
source ../.venv/bin/activate
python main.py --strategy PlanA --show-plan
```

已检出独立 `game` Git 分支时，game 实现位于其 `RaspberryPi/` 根目录；
进入该目录并 `source .venv/bin/activate` 后，后面的命令同样适用。
相对配置路径按执行命令时的当前目录解释；切换实现前先停止并退出原任务。

## Tag 相机配置同步

2026-10-04已同步GitHub main `ead61d5` 的Tag曝光150、增益32、断流恢复与识别失败诊断。
配置来自本目录下的 `vision/opencv/field_map.json`，内参来自同目录的 `tag_camera_calib.json`；
主代码与game仍各自加载独立副本。更新部署文件后重启game入口加载，game现场效果待确认。
Tag6目标距离仍为425 mm，内参文件仍标记 `calibrated=false`。
保留game行进Tag接管需要的解算前单调时钟时间戳；相机恢复不自动重启已经失败的任务。

## 运行方式与优化插拔

去掉 `--show-plan` 才驱动机器人：

```bash
python main.py --strategy PlanA --show-plan
```

策略可换 `PlanB`；`--list-flows` 列出流程。

2026-10-03路线巡航目标：平地最高2000 mm/s（加速1600 ms），上下坡1000 mm/s（加速1000 ms）。
适用于原路线和局部规划；坡道按既有Task2两段整段限速，未使用姿态自动判断坡度。
短退/末端接近400、搜索/贴墙300、抓取压墙150、盲移160 mm/s保持。
这些是软件目标上限，短段和转弯会降速，新速度尚待实机验证；`--show-plan`会显示当前值。

### 优化模块

**七项优化全部默认关闭，各自独立控制。** 加载标定文件或 `--trial-optimizations` 只提供参数，不会自动开启。

| 模块 | 开启参数 | 关闭参数 |
| --- | --- | --- |
| 橙块抓取→下一块 | `--enable-transition next-cube` | `--disable-transition next-cube` |
| 最后一抓→退离 | `--enable-transition last-departure` | `--disable-transition last-departure` |
| 紫块抓取→转场 | `--enable-transition purple-departure` | `--disable-transition purple-departure` |
| 检查复位→运输 | `--enable-transition inspect-departure` | `--disable-transition inspect-departure` |
| Build→返程 | `--enable-transition build-return` | `--disable-transition build-return` |
| 运动规划 | `--enable-motion-planning` | `--disable-motion-planning`（兼容 `--classic-motion`） |
| 快速抓取对准 | `--enable-fast-alignment` | `--disable-fast-alignment` |

只添加所需模块的开启参数即可；其余保持关闭。同一模块的开/关参数互斥。
`--enable-transitions` / `--disable-transitions` 是前五项的批量开关，后两项始终独立。
运动规划统一使用原路线局部平滑：单段移动继续复用原控制器，连续移动转向按局部坐标等价合并。
不需要全场地图或Tag起点；前往Tag6的末段可边识别边修正，并融合原横移偏置。最终建筑位置仍由建筑视觉确认。
抓取衔接和快速对准缺少必需参数时同样在连接机器人前报错。

### 常用组合

使用内置试跑参数，开启全部七项优化：

```bash
python main.py --strategy PlanA --trial-optimizations \
  --enable-transition next-cube \
  --enable-transition last-departure \
  --enable-transition purple-departure \
  --enable-transition inspect-departure \
  --enable-transition build-return \
  --enable-motion-planning \
  --enable-fast-alignment \
  --show-plan
```

此命令不需要额外地图JSON，试跑抓取参数及新平滑/Tag接管限制仍待现场验证。
去掉--show-plan才执行动作。机构监督、净空退离和停止条件保持，原单段动作不因缺Tag被拒绝。
前五个开启参数也可简写为 `--enable-transitions`，运动规划和快速对准的开启参数仍需保留。

**当前树莓派没有 `Strategy/field-transitions.json`，请使用上面的内置试跑命令。**
后续完成现场标定后，可用 `--transition-config 实际文件路径` 替换 `--trial-optimizations`；
抓取文件需包含所选轮次的抓取和快速对准参数，详见[标定配置](Strategy/TRANSITIONS.md)。
空配置无法开启抓取衔接；标定文件与内置试跑参数互斥。

仅开启搭建返程（无需抓取标定）：

```bash
python main.py --strategy PlanA --enable-transition build-return --show-plan
```

### 动作衔接单项选择

`--enable-transition 名称` 开、`--disable-transition 名称` 关，可重复；五类默认全部关闭：

加 `@轮次` 限定范围，如 `--disable-transition next-cube@ground-1`。

优先级：指定轮次 > 指定类型 > 整组开关 > 默认关闭。关闭后先完成前动作再继续。
抓取提前移动仍需完整标定或显式试跑参数，开启后仍检查运行条件。衔接开关保留短顶墙及固件内部并行。

### 比赛路线规划

开启后使用唯一的LocalRoutes：读取原动作 → 保留局部终点、朝向和关键经过点 → 平滑移动/转向 → 编码器/IMU反馈。
前进1200 mm等单动作直接走原定距控制，净空后退/靠墙/Tag3校正保留边界。
PlanB完整首程将出发、逻辑航向重标和紫矿转场合并为一次运输，省去中途180°掉头与倒车。
运输内部在坡口停稳，整段2500 mm上坡保持直线和固定航向，段尾停稳后再左转90°接近紫矿。
独立第二轮上坡2350 mm同样保护；保持原终点与最终朝向。--show-plan会列出Fused transport。
Tag6持续修正为独立模块，默认关闭；关闭时平滑到原目标，再执行原Tag对准与横移。
仅额外指定--enable-moving-tag6才融合相邻Tag对准与偏置动作；--disable-moving-tag6显式关闭且保留局部平滑。
开启后未确认时保留原步骤，已接管后长时间丢目标则停车；移动观测精度仍待验证。
搭建接近成功后仍由建筑视觉完成末端对准，失败时阻止Build。地图不会作为普通移动的可行性门禁。

在上层RaspberryPi目录只读预览：

```bash
python main.py --runtime game --strategy PlanA --enable-motion-planning --show-plan
```

在game目录运行时省略--runtime game。PlanB替换策略名即可。--navigation-config已从比赛入口移除。
详见[路线优化说明](Strategy/optimizations/MOTION_PLANNING.md)。

## 自动运行日志

运行 game 的 `main.py`、`task2_main.py` 或 `robot.py` 时自动保存日志，无需额外参数。
内嵌部署时目录为 `RaspberryPi/game/logs/runs/时间-入口-编号/`；独立 game 分支则为
`RaspberryPi/logs/runs/时间-入口-编号/`。按北京时间命名，每次运行一个目录：

- `console.log`：Python 终端输出、报错和未捕获异常；终端仍正常显示。
- `diagnostics.jsonl`：本次运行的结构化诊断事件。
- `run.json`：入口、开始/结束时间、进程号与退出码，`runtime` 为 `game`。

自动保留最近 **100 次运行**，在启动和结束时清理更早记录；正在运行的日志受保护，结束后补清理。
帮助、预览和启动失败也各算一次。强制断电/杀进程可能丢失末尾内容或结束记录，已落盘的日志仍可查看。
`--diagnostics-log 路径` 仍可额外写一份诊断日志，该自选文件不参与自动清理。
需要换目录时设置 `UNIFOREST_RUN_LOG_DIR`；默认日志目录已排除 Git 跟踪。

## 验证与启动顺序

在本节约定的 game 目录运行 `python tools/check.py`。完成代码检查、配套 Debug 构建、设备与标定检查后，
先做小范围动作，再进入完整任务。固件通过 CLion 的 OpenOCD + DAPLink 配置烧录。
内嵌部署的可选固件工程位于开发机 `Uniforest_A/game/`；独立 game 分支位于 `Uniforest_A/`。
game 比赛入口要求扩展会话固件，握手失败会拒绝启动。当前尚未完成实机验证。

2026-10-02 同步了 GitHub 新场地视觉标定和视觉坐标补偿，未重新调整机械偏置。
game 全量测试仍有已记录的失败；本地 ARM 编译环境也缺交叉编译器，
详细结果见 [整合验证记录](INTEGRATION_VALIDATION.md)，不能将代码上传等同于实机验证通过。

Ctrl+C 停止。切换需重启；异常后不自动续跑。

详见[操作手册](OPERATIONS.md)、[标定配置](Strategy/TRANSITIONS.md)、[自适应盲移](Strategy/optimizations/ADAPTIVE_BLIND.md)和[变更记录](CHANGELOG.md)。
