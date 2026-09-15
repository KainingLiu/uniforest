# Uniforest - RoboGame 2026

RoboGame 2026 竞技组 Uniforest 队机器人软件仓库。当前实现由 Raspberry Pi 5 上位机和 DJI RoboMaster A 板（STM32F427）下位机组成。

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [上位机使用说明](RaspberryPi/README.md) | 环境安装、运行入口、两轮路线、底盘参数、视觉配置和排障 |
| [变更与验证记录](RaspberryPi/CHANGELOG.md) | 日期、参数变化、适用轮次、测试结果和实机验证状态 |
| [数量检查与标定](RaspberryPi/tools/carried_cube_count_test.md) | 底部宽度判别、检查时序、抓取补充及实拍标定 |
| [下位机说明](Uniforest_A/README.md) | 固件入口、构建与烧录、验证边界 |
| [下位机技术说明](Uniforest_A/PROJECT.md) | 实时控制、执行机构、协议和安全边界 |
| [协作约定](AGENTS.md) | 开发范围、协议核对、实机操作和备份约定 |

## 当前版本基线（2026-09-16）

- 两轮 Task1/Task2 普通定距平移：400 mm/s、300 ms 加速。
- 长距离前进和第一轮 Task2 Build 后左移：750 mm/s、800 ms 加速。
- 两轮 Task1 前进补偿基准均为 2800 mm；两轮 Task2 橙色抓取阶段净右移目标均为 700 mm，均扣除对应阶段编码器实测净右移量。
- 第一轮 Task2 Build 后执行后退 100 mm、顺时针转 180°、左移 2500 mm、左顶墙；第二轮在 Build 后结束。
- Grap1/2/3、Build 的整套机械时序在 A 板执行，上位机只发送动作请求并监视状态。动作接口扩展到 schema v2，80 字节完整遥测布局不变。
- 两轮 Task1/Task2 原定橙色抓取结束后检查数量：3 或 null 继续；0/1/2 补抓 3/2/1 块并复查。搜索距离耗尽时跳过检查，补抓不重置搜索距离和编码器原点。
- 数量判别只使用画面下方 30% 的橙色宽度，仓内紫色计 1。检查等待为翻转后 300 ms、前臂后 500 ms、前臂复位后 200 ms；先用新遥测确认静止。
- 当前源码的步进巡航目标半周期延时为 83 μs；定时器量化及参数来源见上、下位机说明。
- A 板 200 ms 通信失联时停止底盘、双步进和吸盘并取消动作，重连不会续跑原动作。

上位机必须配套包含动作接口的 A 板固件。固件烧录由用户通过 CLion 的 OpenOCD + DAPLink 配置完成。数量算法已用 22 组实拍照片回放；最新比赛衔接和等待时序仍待现场验证，回放不能替代完整任务实测。

## 工程结构

| 目录 | 职责 |
| --- | --- |
| `RaspberryPi/` | 视觉、策略、位置外环、速度规划、上位机调试工具和通信 |
| `Uniforest_A/` | 1 kHz 底盘速度环、100 kHz 步进脉冲、机械动作执行、传感器与协议 |
| `备份/` | 本地历史快照，默认只读；GitHub 仅保留说明文件 |

只以两个当前代码目录和构建配置判断实际行为。规则手册、队伍计划书及 A 板硬件 PDF 不在当前检出中，涉及规则或硬件设计时需另行查阅原资料。

## 安装与无硬件验证

树莓派用户名为 `uniforest`，开发地址为 `192.168.137.50`。密码向队内管理员获取，不写入仓库、日志或文档。

在树莓派的 `RaspberryPi/` 目录执行：

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt -r vision/requirements.txt
python tools/check.py
```

Windows 环境、设备预检和单项测试见[上位机使用说明](RaspberryPi/README.md)。C 固件无硬件编译：

```powershell
cd Uniforest_A
cmake --preset Debug
cmake --build build/Debug
```

当前检查入口仅执行 Python 语法、依赖导入和 6 项协议格式检查；开发机可加 `--firmware` 编译 A 板，不烧录。模拟机器人、合成视觉及虚拟 C 动作测试已按用户要求移除，`--native-actions`、`--module` 已取消。历史测试数字保存在[验证记录](RaspberryPi/CHANGELOG.md)，不代表当前现场验证状态。

## 比赛与调试入口

以下命令会连接机器人；运行前先完成无硬件验证、固件更新、设备与标定检查和小范围动作测试。

```bash
python robot.py --preflight --vision --localization
python main.py --task all
python main.py --task round1
python main.py --task round2
python main.py --task task1-r2
python main.py --task task2-r2
```

`all` 按 Task0、Task1-R1、Task2-R1、Task1-R2、Task2-R2 执行；`round1` 和 `round2` 也先执行一次 Task0，再执行对应轮次 Task1、Task2，单任务入口跳过 Task0。单机械动作使用 `python action_test.py grap1|grap2|grap3|build`，实际调用时选择其中一个动作名。

## 提交与备份

- 修改协议时同步维护双方实现、schema、测试和文档；仅改策略时也要核对并记录“协议未变”。
- 每次现场调参记录日期、参数、轮次、测试入口和现场结果，未现场确认的值仅作为软件配置。
- 不提交密码、密钥、令牌、虚拟环境、编译产物、IDE 配置、调试输出或本地备份。
- 当前工作成果由用户在每日结束后复制到 `备份/`，不覆盖已有快照。
