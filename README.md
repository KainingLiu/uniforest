# Uniforest — RoboGame 2026

Raspberry Pi 5 上位机负责视觉、比赛策略和底盘位置外环；DJI RoboMaster A 板
（STM32F427）负责电机速度环、机械动作、传感器和通信失联保护。
文档整理于 2026-09-26，具体行为以源码与构建配置为准。

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [上位机说明](RaspberryPi/README.md) | 安装、设备预检、比赛入口、两轮路线、底盘与视觉参数、当前部署状态 |
| [下位机说明](Uniforest_A/README.md) | 固件入口、构建与 CLion 烧录 |
| [机械动作流程](Uniforest_A/ACTIONS.md) | Grap1/2/3、Build 的角度、距离、等待和并行节点 |
| [下位机技术说明](Uniforest_A/PROJECT.md) | 实时控制、步进参数、通信协议与硬件排障 |
| [数量检查与标定](RaspberryPi/tools/carried_cube_count_test.md) | 检查时序、宽度判别、紫色规则、实拍采样和补抓 |
| [前臂零点调整](RaspberryPi/tools/arm_zero_adjust.md) | 调零入口、逻辑角度与 +12° 固定偏置 |
| [变更与验证记录](RaspberryPi/CHANGELOG.md) | 日期、历史参数、部署备份、检查及现场验证结果 |
| [协作约定](AGENTS.md) | 修改范围、协议核对、实机操作与备份规则 |

## 启动与检查

树莓派开发地址 `192.168.137.50`，用户名 `uniforest`。密码由管理员提供，不写入仓库。
完成[环境安装与设备预检](RaspberryPi/README.md)后，在树莓派终端一键启动完整比赛：

```bash
cd /home/uniforest/Uniforest/RaspberryPi && .venv/bin/python -u main.py --task all
```

顺序为 Task0 → Task1-R1 → Task2-R1 → Task1-R2 → Task2-R2，Task0 只运行一次。
`--task round1`、`round2` 各自也先运行 Task0；`task1-r1`、`task2-r1`、
`task1-r2`、`task2-r2` 只运行指定任务。程序会驱动整车，按 Ctrl+C 停止。

无硬件检查在 `RaspberryPi/` 执行 `python tools/check.py`；开发机加 `--firmware`
可同时配置、编译 A 板，不烧录。固件烧录统一使用 CLion 的 OpenOCD + DAPLink。
通过软件检查不代表动作、视觉或场地参数已经实机验证。

## 源码与部署边界

- `RaspberryPi/`、`Uniforest_A/` 是当前正式实现；参数分别集中在上面的专业文档。
- `备份/` 为本地快照，默认不提交 GitHub。用户指定维护的遥控副本也不代表正式固件入口。
- 上传上位机、烧录 A 板、实机验证是三个不同状态。树莓派采用定向同步，仍有旧搜索实现、步进调试默认值和兼容接口差异；详情见[部署表](RaspberryPi/README.md#仓库与树莓派部署状态)。
- 动作协议为 schema v3；状态 6 表示机构仍忙、允许底盘继续。A 板通信失联 200 ms 后停止底盘、双步进和吸盘并取消动作，重连不续跑。
- 不提交密码、密钥、虚拟环境、构建产物、IDE 缓存、现场诊断图片或本地备份。

规则手册、队伍计划书和 A 板硬件 PDF 不在当前检出中，需要时另行查阅原资料。
每天结束由用户另存工作快照，不覆盖已有备份。
