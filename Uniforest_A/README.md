# Uniforest A 板下位机程序

当前正式固件运行于 DJI RoboMaster A 板 STM32F427IIHx，使用 STM32 HAL、CMake
和裸机主循环，无 RTOS。本文只维护入口与构建方法，更新于 2026-09-26。

## 入口与文档

| 入口 | 职责 |
| --- | --- |
| `Core/Src/main.c` | 初始化、主循环及 TIM6 底盘速度环 |
| `Core/Src/actions.c` | Grap1/2/3、Build 动作表和非阻塞执行器 |
| `Core/Src/protocol.c` | UART7 命令、遥测及失联处理 |
| `Core/Src/motor3508.c` | CAN 电机反馈、速度闭环与累计编码器 |
| `Core/Src/stepper.c`、`servo.c`、`suction.c` | 步进、舵机及吸盘执行机构 |
| [ACTIONS.md](ACTIONS.md) | 当前机械动作、默认角度、等待和底盘并行节点 |
| [PROJECT.md](PROJECT.md) | 架构、协议、当前步进参数与硬件排障 |
| [前臂调零](../RaspberryPi/tools/arm_zero_adjust.md) | 前臂逻辑角度与 +12° 输出偏置 |
| [变更记录](../RaspberryPi/CHANGELOG.md) | 参数调整、部署与验证记录 |

正式固件由树莓派指挥，不自动切换遥控。SBUS 只作为备用输入初始化，主循环不
调用阻塞式 `Remote_Control()`；USART3 VOFA+ 调试遥测默认关闭。

## 构建与烧录

在本目录进行无硬件编译：

```powershell
cmake --preset Debug
cmake --build build/Debug
```

也可在上位机目录执行 `python tools/check.py --firmware`，同时检查 Python 语法、
导入和协议格式。当前正式工程不包含虚拟机器人或动作轨迹测试。

统一通过 **CLion 的 OpenOCD + DAPLink** 调试、烧录，不使用命令行直接烧录。
产物 `build/Debug/Uniforest_A_0628.elf` 与 `.ioc` 的 `0628` 为历史名称。
固件参数修改必须重新烧录，单独上传 Python 不会改变板上的动作与速度。

## 验证边界

最新步进参数与 Grap3 行程已完成无硬件编译；板上固件版本、负载下失步和整套动作
仍需现场确认。烧录后先核对机构起始位置，再做单动作小范围测试，最后进入完整任务。
通信失联、取消或急停不会自动续跑；协议兼容与超时说明见 [PROJECT.md](PROJECT.md)。
