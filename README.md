# Uniforest - RoboGame 2026

RoboGame 2026 竞技组 Uniforest 队机器人软件仓库，包含 Raspberry Pi 5 上位机程序和 DJI RoboMaster A 板下位机固件。

## 工程结构

- [`RaspberryPi/`](RaspberryPi/)：比赛策略、视觉定位、运动规划、调试工具及上下位机通信。
- [`Uniforest_A/`](Uniforest_A/)：STM32F427 底层控制、传感器采集、执行机构驱动及串口协议。
- [`备份/`](备份/)：本地历史快照目录；GitHub 只保留说明文件，不上传快照内容。

当前实现只以上述两个代码目录为准。上位机通过 UART7 向 A 板发送底盘、舵机和步进电机命令；A 板以 1 kHz 运行底盘速度环，并向上位机回传电机、IMU、遥控器和执行机构遥测。

2026-08-23 上位机迭代集中在比赛策略和视觉鲁棒性：目标锁定增加 X/Z 跳变限制与歧义保护，橙色方块增加精对准阶段，定距动作支持“编码器进度达到 90% 以上时接受超时结果”，并取消部分路线中的额外保持等待。本次未改变 UART7 命令编号、载荷格式、80 字节遥测或 A 板实时控制逻辑；实机验证前仍需按 `RaspberryPi/README.md` 的顺序执行预检。

2026-08-25 调试收尾：完成三种方块的视觉目标整理，并将可现场修改的目标坐标、粗对准范围和末端微调范围集中到 `RaspberryPi/Strategy/vision_targets.py`。三种方块目标均为 `0.0 mm`；橙色粗调 `[-20, 5] mm`、精调 `[-3, 3] mm`；紫色粗调 `[-5, 5] mm`、精调 `[-3, 3] mm`。本轮仅修改上位机策略配置、视觉目标入口和测试文档，UART7 协议、载荷格式、遥测布局及 A 板固件均未改变。

## 上位机

运行环境为 Raspberry Pi 5 8GB 和 Python 3。当前树莓派开发主机为 `192.168.137.50`，用户名为 `uniforest`；建议通过 VSCode Remote-SSH 连接。密码由队内管理员线下提供，不写入项目文档。

首次部署：

```bash
cd RaspberryPi
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

Windows 调试时使用 `RaspberryPi\.venv\Scripts\python.exe`；先执行 `py -3 -m venv RaspberryPi\.venv`，再按 [`RaspberryPi/README.md`](RaspberryPi/README.md) 安装两份 requirements 并运行 `tests\import_smoke.py`。项目虚拟环境已加入 `.gitignore`。

正式比赛入口：

```bash
python main.py                    # Task0、第一轮和第二轮完整流程
python main.py --task round1     # 只运行第一轮 Task1/Task2
python main.py --task round2     # 只运行第二轮 Task1/Task2
python main.py --task task1-r2   # 只运行第二轮 Task1
```

完整的相机角色、任务流程、标定和调试说明见 [`RaspberryPi/README.md`](RaspberryPi/README.md)。

## 下位机

主控为 STM32F427IIHx，使用 STM32 HAL、CMake 和裸机超级循环。团队统一使用 CLion 中配置的 OpenOCD + DAPLink 工具链进行调试和烧录。命令行只用于无硬件编译验证：

```powershell
cd Uniforest_A
cmake --preset Debug
cmake --build build/Debug
```

固件入口为 `Uniforest_A/Core/Src/main.c`。当前架构、通信协议和硬件说明见 [`Uniforest_A/PROJECT.md`](Uniforest_A/PROJECT.md)。

## 开发约定

- 不在仓库中保存密码、密钥、令牌或设备私密信息。
- 修改通信协议时，同步更新上、下位机实现、测试和文档。
- 仅修改 Raspberry Pi 策略、视觉或位置外环时，也要核对协议和遥测契约；若协议未变，应在变更说明中明确记录。
- 每次实机迭代前先运行上位机无硬件测试和下位机命令行构建；烧录只通过 CLion 配置的 OpenOCD + DAPLink 完成。
- 不提交 `build/`、IDE 配置、Python 缓存和视觉调试输出。
- 历史快照只放入本地 `备份/`，不作为当前代码继续开发。

协作工具的完整修改边界见 [`AGENTS.md`](AGENTS.md)。
