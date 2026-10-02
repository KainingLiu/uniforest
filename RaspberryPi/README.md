# game 上位机运行指南

本文件只说明 game 实现的启动、优化开关、日志和验证要求。

## 选择 game

在树莓派的上层 `RaspberryPi/` 目录，通过参数显式启动 game：

```bash
cd /home/uniforest/Uniforest/RaspberryPi
source .venv/bin/activate
python main.py --runtime game --strategy PlanA --show-plan
python main.py --runtime game --strategy PlanB --show-plan
```

确认预览与车辆起始状态后，去掉 `--show-plan` 才执行动作。
同样可以使用 `python task2_main.py --runtime game --variant 1 --show-plan`；
`python robot.py --runtime game --help` 查看 game 的调试入口。

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

## 运行方式与优化插拔

去掉 `--show-plan` 才驱动机器人：

```bash
python main.py --strategy PlanA --show-plan
```

策略可换 `PlanB`；`--list-flows` 列出流程。

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
运动规划使用原路线计算的起终点，交给完整 `FieldPlanner` 搜索曲线，再执行其速度规划与位置 PID；
PlanA/PlanB 复用现有场地图和编码器/IMU，**无需额外 JSON 或 Tag 标定**。
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

此命令无需额外 JSON。路线使用完整曲线规划；抓取和快速对准的内置试跑参数仍标记为未验证。
完整规划器目前要求每段入口底盘静止、机构空闲；与提前移动衔接混用时会检查该条件，失败会停止，
不会静默改用旧路线。首次验证建议仅开启运动规划。
预览确认后，去掉 `--show-plan` 才执行。
前五个开启参数也可简写为 `--enable-transitions`，运动规划和快速对准的开启参数仍需保留。

**当前树莓派没有 `Strategy/field-transitions.json`，请使用上面的内置试跑命令。**
后续完成现场标定后，可用 `--transition-config 实际文件路径` 替换 `--trial-optimizations`；
标定需包含所选轮次的抓取参数，以及要启用的曲线、快速对准参数，详见[标定配置](Strategy/TRANSITIONS.md)。
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

开启后从原 `ROUTES` 读取终点、终点朝向和实际取块位移补偿；原中间横移/转向点不再作为必经点。
19 类具名路线使用之前的位置间规划器：位姿搜索、五次曲线候选比较、`CompetitionMotion` 速度规划、
`PositionTracker` 位置 PID 和加加速度限制。靠墙、视觉校正、航向重标及原净空退离保持独立边界。
控制目标沿用原代码，不改任务顺序、目的区域或抓取/投放动作。最快指候选曲线的预计耗时比较，未证明全局最优。

仅开启路线优化：

```bash
python main.py --strategy PlanA --enable-motion-planning --show-plan
```

去掉 `--show-plan` 即执行。无需 `--trial-optimizations`、`--transition-config` 或 `--navigation-config`。
PlanB 将上面命令中的 PlanA 换成 PlanB。运行期用连续编码器/IMU 更新起点，搜索、靠墙和视觉对准期间也累计位置。
起步地图坐标复用现有 `start_blue` 约定，场地与车体几何沿用现有 CAD 模型，均保留其未实测来源标记。
若原终点与模型发生碰撞，规划器会明确拒绝，既不修改目标，也不回放旧路线。
当前模型检查发现 PlanA 起步原终点与旧车体外包冲突，详情见 [CHANGELOG](CHANGELOG.md)；尚未完成实机验证。
`--navigation-config` 保留显式选择已标定视觉导航的入口，其 Tag 标定条件只适用于该入口。
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
