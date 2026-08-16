# Uniforest A 板下位机程序

RoboGame 2026 竞技组 Uniforest 队的 DJI RoboMaster A 板固件。主控为 STM32F427IIHx，使用 STM32 HAL、CMake 和裸机超级循环，不使用 RTOS。

## 当前运行架构

树莓派负责比赛策略、视觉和位置外环，A 板负责实时执行与硬件安全：

- UART7 以 115200 8N1 接收上位机命令并发送 CRC16 帧；
- TIM6 中断以 1 kHz 运行四轮 M3508 速度 PID；
- CAN1 中断接收 C620 反馈并累计多圈编码器位置；
- TIM7 以 100 kHz 生成双步进电机脉冲；
- 主循环轮询 JY61P IMU、处理上位机命令并按设定频率发送 80 字节遥测；
- 超过 200 ms 未收到有效上位机帧时停止底盘，并以红灯闪烁提示通信丢失。

SBUS 接收仍作为备用输入初始化，但当前正式主循环不调用阻塞式 `Remote_Control()`。USART3 VOFA+ 调试遥测默认关闭，避免与 UART7 共用 DAPLink 时冲突。

## 代码入口

- [`PROJECT.md`](PROJECT.md)：当前架构、模块职责、通信协议和调试边界。
- `Core/Src/main.c`：初始化、主循环和 TIM6 底盘速度环中断。
- `Core/Src/protocol.c`：UART7 帧解析、命令分发和遥测打包。
- `Core/Src/motor3508.c`：CAN 电机反馈、速度闭环与累计编码器。
- `Core/Src/servo.c`、`Core/Src/stepper.c`：执行机构驱动。
- `Uniforest_A_0628.ioc`：STM32CubeMX 工程配置；文件名是历史名称，不代表当前版本。

## 构建与烧录

团队统一使用 CLion 中配置的 OpenOCD + DAPLink 工具链进行编译、调试和烧录，不使用命令行直接烧录。无硬件编译验证：

```powershell
cmake --preset Debug
cmake --build build/Debug
```

CMake 目标名仍为 `Uniforest_A_0628`，因此产物沿用旧名称。这只是构建配置的历史命名。

所有有效迭代均在本目录完成；工作区根目录的 `备份/` 只保存历史快照。
