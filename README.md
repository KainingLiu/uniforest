# Uniforest - RoboGame 2026

RoboGame 2026 竞技组 Uniforest 队机器人软件仓库，包含 Raspberry Pi 5 上位机程序和 DJI RoboMaster A 板下位机固件。

## 工程结构

- [`RaspberryPi/`](RaspberryPi/)：比赛策略、视觉定位、运动规划、调试工具及上下位机通信。
- [`Uniforest_A/`](Uniforest_A/)：STM32F427 底层控制、传感器采集、执行机构驱动及串口协议。
- [`备份/`](备份/)：本地历史快照目录；GitHub 只保留说明文件，不上传快照内容。

当前实现只以上述两个代码目录为准。上位机通过 UART7 向 A 板发送底盘、舵机和步进电机命令；A 板以 1 kHz 运行底盘速度环，并向上位机回传电机、IMU、遥控器和执行机构遥测。

## 上位机

运行环境为 Raspberry Pi 5 8GB 和 Python 3。首次部署：

```bash
cd RaspberryPi
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

正式比赛入口：

```bash
python main.py                 # 依次运行 Task1 和 Task2
python main.py --task task1   # 只运行 Task1
python main.py --task task2   # 只运行 Task2
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
- 不提交 `build/`、IDE 配置、Python 缓存和视觉调试输出。
- 历史快照只放入本地 `备份/`，不作为当前代码继续开发。

协作工具的完整修改边界见 [`AGENTS.md`](AGENTS.md)。
