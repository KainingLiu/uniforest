# Raspberry Pi 上位机运行说明

整理日期：2026-10-05。本文维护安装、启动与部署边界；路线和参数归入专业文档。

## 文档分工

| 内容 | 文档 |
| --- | --- |
| 可复制的策略、动作和诊断命令 | [TEST_COMMANDS.md](TEST_COMMANDS.md) |
| PlanA/B/C、Task 路线和补抓 | [Strategy/README.md](Strategy/README.md) |
| 底盘速度、到位和顶墙 | [control/README.md](control/README.md) |
| 相机、识别、对准与视觉降级 | [vision/opencv/README.md](vision/opencv/README.md) |
| Grap/Build 动作表 | [下位机 ACTIONS.md](../Uniforest_A/ACTIONS.md) |
| 数据采集与离线实验 | [vision/yolo/README.md](vision/yolo/README.md) |
| 历史参数与验证记录 | [CHANGELOG.md](CHANGELOG.md) |

## 环境与目录

运行平台 Raspberry Pi 5 8GB，队内连接地址 `192.168.137.50`，用户名 `uniforest`，部署目录 `/home/uniforest/Uniforest/RaspberryPi`。密码由管理员提供，不写入仓库。

| 入口/目录 | 职责 |
| --- | --- |
| `main.py` | 策略、单 Task、任务列表和只读预览 |
| `robot.py` | 串口、设备聚合、心跳、相机生命周期和单动作 |
| `Strategy/` | 不可变配置、Task 库、策略包、执行器及上下文 |
| `control/` | 位置外环、动作客户端、步进/舵机及装载检查 |
| `protocol/` | 帧编解码、传输与 schema v5 |
| `vision/opencv/` | 当前比赛视觉与标定；旧模块路径保留兼容导入 |
| `vision/yolo/` | 原图采集和离线训练；比赛推理集成尚未完成 |
| `tools/`、`tests/` | 诊断、采集、桌面入口与无硬件检查 |
| `sensors/`、`utils/` | 传感器封装及辅助代码 |

调用关系为 `main.py → Strategy → robot → control / protocol / vision`。执行器负责编排，设备启动及最终关闭由入口程序负责。

## 安装

树莓派：

```bash
cd /home/uniforest/Uniforest/RaspberryPi
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt -r vision/opencv/requirements.txt
python tests/import_smoke.py
```

Windows 开发机，在工作区根目录：

```powershell
cd RaspberryPi
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt -r vision\opencv\requirements.txt
.\.venv\Scripts\python.exe tests\import_smoke.py
```

requirements 记录版本下限，不是精确锁定环境；复现时另存实际 Python、OpenCV、NumPy、系统和固件版本。

## 检查与启动

1. 检查工作区状态、当前烧录固件及机构初始位置。
2. 完成 Python 语法/无硬件检查；开发机完成下位机 Debug 编译。
3. 核对串口、cube/tag 角色和标定，再做小范围动作，最后运行完整策略。

```bash
python tools/check.py
python -m unittest discover -s tests -q
python main.py --list-tasks
python main.py --strategy PlanC --show-plan
```

上述命令不驱动机器人。实际运行示例为 `python main.py --strategy PlanA`，也可选 PlanB、PlanC，默认仍是 PlanA。
单 Task 通常不补前后任务，但 Task1-1/2、Task2-1/2 搜索耗尽可触发一次补抓，详见策略说明。

树莓派桌面七个入口为 PlanA、PlanB、PlanC、set1、set2、collect-build-1、collect-build-2。
入口使用 `.venv/bin/python` 与 `tools/desktop_task.sh`，共用防重复启动锁；手动 CLI 不受该桌面锁约束。
Ctrl+C 停止，结束后回车关闭。管理入口：

```bash
python tools/install_desktop_entries.py --dry-run
python tools/install_desktop_entries.py
bash tools/desktop_task.sh PlanC --show-plan
```

独立 Task3/Task4 必须位于 Task2 结束位置、初始航向 180°，提供已知 `--heading-zero-deg`；不能猜测或直接填当前朝向。

## 设备、采集与日志

- 串口使用平台默认选择或显式 `--port`；Linux 核对 `/dev/serial/by-id/`，波特率 115200。
- 相机稳定路径在 `vision/opencv/camera_devices.json`；cube 用于方块、装载、建筑，tag 用于 AprilTag。
- 采集默认关闭，显式 `UNIFOREST_COLLECT_DATA=1` 开启；`--no-collect-data` 关闭本次采集。
- 自动采集仅覆盖抓取前橙/紫搜索和对准；图片、SQLite 索引和批次在 `vision/yolo/data/collection/`，不上传 GitHub。
- 树莓派既有 `runtime_launcher` / `run_logs` 自动日志封装，本地当前主入口没有这套封装。本地需要日志时使用 `--diagnostics-log /tmp/uniforest-run.jsonl`。普通同步应保留远端日志功能。

## 协议与部署状态

契约以 [schema.json](protocol/schema.json) 与[下位机说明](../Uniforest_A/PROJECT.md)为准。
动作 ID：Grap1/2/3=1/2/3，Build3/2/1=4/5/6；`build` 兼容 Build3。
`ACTION_CHASSIS_READY=6` 表示机构仍忙但允许底盘并行，客户端等两者完成后返回。

最后已记录的树莓派同步为 **2026-10-04**：PlanC 使用 Task3-4 和 Task3-2，已有 PlanC 桌面入口；文件核对及远端无硬件检查通过。
本地前臂偏置为 +12.3°，参数见 ACTIONS.md 和源码；上传 Python 不能证明 A 板已烧录同一固件。
本轮只整理文档并上传本地仓库，不改机器人参数，不部署树莓派或执行实机动作。

夺冠、软件测试通过、同步完成、固件一致和某项现场性能达标各自需要相应证据。
历次调参与检查次数保存在 CHANGELOG，本 README 不再维护过期的逐次部署表。
