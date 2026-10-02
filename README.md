# Uniforest — RoboGame 2026

Raspberry Pi 5 上位机负责视觉、比赛策略和底盘位置外环；DJI RoboMaster A 板
（STM32F427）负责电机速度环、机械动作、传感器和通信失联保护。
文档整理于 2026-10-02，具体行为以源码与构建配置为准。

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [上位机说明](RaspberryPi/README.md) | 安装、设备预检、比赛入口、两轮路线、底盘与视觉参数、当前部署状态 |
| [下位机说明](Uniforest_A/README.md) | 固件入口、构建与 CLion 烧录 |
| [机械动作流程](Uniforest_A/ACTIONS.md) | Grap1/2/3、Build 的角度、距离、等待和并行节点 |
| [下位机技术说明](Uniforest_A/PROJECT.md) | 实时控制、步进参数、通信协议与硬件排障 |
| [数量检查与标定](RaspberryPi/tools/carried_cube_count_test.md) | 检查时序、宽度判别、紫色规则、实拍采样和补抓 |
| [自然语言 Agent](RaspberryPi/agent/README.md) | 自然语言、本地直控及 API 中转 |
| [自动原图采集](RaspberryPi/vision/yolo/docs/DATA_COLLECTION.md) | 数据目录、采样与存储上限、独立补拍 |
| [前臂零点调整](RaspberryPi/tools/arm_zero_adjust.md) | 调零入口、逻辑角度与 +12° 固定偏置 |
| [变更与验证记录](RaspberryPi/CHANGELOG.md) | 日期、历史参数、部署备份、检查及现场验证结果 |
| [协作约定](AGENTS.md) | 修改范围、协议核对、实机操作与备份规则 |

## 通过 SSH 连接树莓派

项目记录的树莓派开发地址为 `192.168.137.50`，用户名为 `uniforest`。
密码由队内管理员提供，不写入仓库、日志或聊天记录。

1. 给树莓派通电，让电脑和树莓派连接同一个 Wi-Fi／热点，或处于可互通的有线网络。
2. 在 Windows 电脑上打开 PowerShell 或 Windows 终端，执行：

   ```powershell
   ssh uniforest@192.168.137.50
   ```

3. 首次连接可能提示确认主机指纹，向管理员核对后输入 `yes`。
   提示输入密码时，屏幕不会显示字符，输完直接按回车。
4. 登录成功后，在树莓派终端进入项目目录：

   ```bash
   cd /home/uniforest/Uniforest/RaspberryPi
   ```

退出 SSH 连接时输入 `exit`。

| 连接报错 | 排查方向 |
| --- | --- |
| `Connection timed out` | 检查树莓派供电、网络连通性，以及当前 IP 是否仍为 `192.168.137.50`；若地址变化，将 SSH 命令中的地址替换为当前 IP |
| `Connection refused` | 检查树莓派是否启用了 SSH 服务，以及 SSH 端口是否正确 |
| `Permission denied` | 检查用户名、密码或 SSH 密钥配置 |

## 启动与检查

完成[环境安装与设备预检](RaspberryPi/README.md)后，在树莓派终端一键启动完整比赛：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --strategy PlanA
```

无参数运行同样选择 PlanA；`--strategy PlanB` 选择另一套主分支策略。
`--task all` 继续作为 PlanA 的兼容命令，`round1`/`round2` 对应 set1/set2。
单任务使用 `--task task1-1` 等明确编号。完整任务顺序见[上位机运行说明](RaspberryPi/README.md#比赛入口)。
程序会驱动整车，按 Ctrl+C 停止。只查看流程时添加 `--show-plan`。
三个运行入口自动将日志保存到 `RaspberryPi/logs/runs/`，保留最近 100 次运行。

无硬件检查在 `RaspberryPi/` 执行 `python tools/check.py`；开发机加 `--firmware`
可同时配置、编译 A 板，不烧录。检查包含协议和数据采集功能。固件烧录统一使用 CLion 的 OpenOCD + DAPLink。
通过软件检查不代表动作、视觉或场地参数已经实机验证。

## 源码与部署边界

- `RaspberryPi/`、`Uniforest_A/` 是当前正式实现；参数分别集中在上面的专业文档。
- `备份/` 为本地快照，默认不提交 GitHub。用户指定维护的遥控副本也不代表正式固件入口。
- 上传上位机、烧录 A 板、实机验证是三个不同状态。当前同步保留主分支策略、参数和原有指令；历史部署记录见[部署表](RaspberryPi/README.md#仓库与树莓派部署状态)。
- 动作协议为 schema v3；状态 6 表示机构仍忙、允许底盘继续。A 板通信失联 200 ms 后停止底盘、双步进和吸盘并取消动作，重连不续跑。
- 不提交密码、密钥、虚拟环境、构建产物、IDE 缓存、现场诊断图片或本地备份。

规则手册、队伍计划书和 A 板硬件 PDF 不在当前检出中，需要时另行查阅原资料。
每天结束由用户另存工作快照，不覆盖已有备份。
