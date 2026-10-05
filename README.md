# Uniforest · RoboGame 2026

Uniforest 队机器人电控、算法与视觉项目。队伍已获得本届比赛冠军（队内确认）；赛后整理日期为 **2026-10-05**。
本仓库保存整理时的本地源码。决赛使用的策略、固件烧录版本、成绩和视频证据应在赛后技术总结中补充。

## 项目结构

| 目录 | 用途 |
| --- | --- |
| `RaspberryPi/` | Raspberry Pi 5 8GB 上位机：视觉、策略、底盘位置外环和通信 |
| `Uniforest_A/` | RoboMaster A 板 STM32F427 下位机：四轮速度环、机械动作、IMU、遥测与失联保护 |
| `备份/` | 本地历史快照，默认只读且不上传 |

当前实现以主目录的实际入口和构建配置为准。

## 文档入口

同一项事实只在负责该主题的文档维护，其他文档提供链接。

| 文档 | 负责内容 |
| --- | --- |
| [上位机运行说明](RaspberryPi/README.md) | 安装、启动、设备预检与部署边界 |
| [机器人测试命令](RaspberryPi/TEST_COMMANDS.md) | 策略包、单 Task、机械动作及只读检查的可复制命令 |
| [策略与 Task 库](RaspberryPi/Strategy/README.md) | PlanA/B/C、任务路线、补抓、数量交接与扩展 |
| [底盘控制](RaspberryPi/control/README.md) | 速度/加速、提前减速、到位与顶墙 |
| [视觉与标定](RaspberryPi/vision/opencv/README.md) | 相机、识别、对准与视觉降级 |
| [下位机入口](Uniforest_A/README.md) / [技术说明](Uniforest_A/PROJECT.md) | 构建、CLion 烧录、实时控制、硬件接口与协议 |
| [机械动作](Uniforest_A/ACTIONS.md) | Grap1/2/3、Build1/2/3 的完整动作表 |
| [数量检查](RaspberryPi/tools/carried_cube_count_test.md) / [前臂调零](RaspberryPi/tools/arm_zero_adjust.md) | 独立调试和标定方法 |
| [YOLO 与数据采集](RaspberryPi/vision/yolo/README.md) | 已有离线实验、训练报告与采集说明 |
| [变更与验证记录](RaspberryPi/CHANGELOG.md) | 历次调参、回退、同步和验证；历史值不是当前值 |
| [协作约定](AGENTS.md) | 开发范围、备份、协议核对与实机规范 |

## 当前策略与启动

默认策略为 **PlanA**。PlanB、PlanC 及 set1/set2、collect-build-1/2 共用同一 Task 库和执行器。
最新 PlanC 为：

```text
Task0-1 → Task1-1 → Task0-3 → Task1-2
→ Task2-1 → Task3-4 → Task2-2 → Task3-2
→ Task1-3 → Task2-2 → Task3-3
```

在树莓派上只读查看流程：

```bash
cd /home/uniforest/Uniforest/RaspberryPi
.venv/bin/python main.py --strategy PlanC --show-plan
```

去掉 `--show-plan` 会执行机器人动作。完整命令及起点要求见测试命令文档。
上次已确认的树莓派同步为 2026-10-04，桌面有 PlanA、PlanB、PlanC、set1、set2、collect-build-1、collect-build-2 七个策略入口。

## 复现与归档边界

- 上位机无硬件检查：在 `RaspberryPi/` 执行 `python tools/check.py` 与 `python -m unittest discover -s tests -q`。
- 下位机无硬件编译：在 `Uniforest_A/` 执行 `cmake --preset Debug`、`cmake --build build/Debug`。烧录统一使用 CLion 的 OpenOCD + DAPLink。
- 协议为 [schema v5](RaspberryPi/protocol/schema.json)。状态 6 允许底盘与机构收尾并行，仍须等待最终完成。200 ms 通信失联会停止底盘、步进及吸盘并取消动作，重连不续跑。
- 视觉失效按任务预算和超时降级继续；通信、遥测陈旧、急停及机构故障仍终止。
- GitHub `main` 本轮以本地可提交文件树为准；密码、密钥、私有配置、虚拟环境、构建产物、采集数据和本地备份不属于上传内容。
- 树莓派既有自动运行日志封装与本地略有差异；GitHub 与本地一致不代表树莓派所有文件一致。设备部署版本以实际同步记录为准。
- 规则手册、队伍计划书和 A 板硬件 PDF 不在当前检出中；规则引用与硬件设计论证需补原始资料，不能用过往对话替代正式依据。
