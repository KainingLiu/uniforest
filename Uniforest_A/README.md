# Uniforest A 板下位机程序

RoboGame 2026 竞技组 Uniforest 队的 DJI RoboMaster A 板固件。主控为 STM32F427IIHx，使用 STM32 HAL、CMake 和裸机超级循环，不使用 RTOS。

## 当前运行架构

树莓派负责比赛策略、视觉和位置外环，A 板负责实时执行与硬件安全：

- UART7 以 115200 8N1 接收上位机命令并发送 CRC16 帧；
- TIM6 中断以 1 kHz 运行四轮 M3508 速度 PID；
- CAN1 中断接收 C620 反馈并累计多圈编码器位置；
- TIM7 以 100 kHz 生成双步进电机脉冲；
- 主循环轮询 JY61P IMU、处理上位机命令并按设定频率发送 80 字节遥测；
- 主循环非阻塞执行 Grap1/2/3 和 Build 整套机械时序，树莓派请求、监视动作并协调后续底盘路线；
- 超过 200 ms 未收到有效上位机帧时停止底盘、双步进和吸盘，取消整套动作，并以红灯闪烁提示通信丢失；重连不会继续原动作。

SBUS 接收仍作为备用输入初始化，但当前正式主循环不调用阻塞式 `Remote_Control()`。USART3 VOFA+ 调试遥测默认关闭，避免与 UART7 共用 DAPLink 时冲突。

## 代码入口

- [`PROJECT.md`](PROJECT.md)：当前架构、模块职责、通信协议和调试边界。
- `Core/Src/main.c`：初始化、主循环和 TIM6 底盘速度环中断。
- `Core/Src/protocol.c`：UART7 帧解析、命令分发和遥测打包。
- `Core/Src/actions.c`：Grap1/2/3、Build 动作表和非阻塞执行器；参数调整后需重新烧录。
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

## 验证与当前参数

2026-09-14 已按用户要求移除原生 C 虚拟硬件和动作轨迹测试；当前使用上面的
CMake 命令进行无硬件编译。上位机 `python tools/check.py --firmware` 可统一执行
Python 语法、导入、6 项协议格式检查和固件编译。这些检查不证明机械动作现场通过。

当前动作表的步进巡航目标半周期延时为 83 μs，启动延时 1000 μs、加速段 400 步；
实际脉冲受 TIM7 10 μs tick 量化。修改固件参数后需用户重新烧录，不能只上传 Python。
Task1/Task2 新增的携带数量检查在上位机编排，使用原单舵机、底盘零速和动作查询命令；
本次数量检查协议未变，没有新增 A 板动作 ID。

当前完整步骤集中在[机械动作流程](ACTIONS.md)：Grap 无动作内固定等待，Build 仅保留取件和分段抬臂等待；步进运动完成后没有额外 100 ms 稳定等待。

动作协议使用 schema v3：Grap2 回程上升到 5 cm、Build 最后一次释放后，状态变为 `ACTION_CHASSIS_READY=6`。该状态仍属于机构忙，继续执行剩余动作；配套上位机可同时运行底盘路线。底盘与机构都结束后才进入下一次机械动作，异常仍整机急停。

旧上位机会将状态 6 误判为故障。2026-09-18 已修复树莓派版本不匹配并同步配套代码；现场日志仅确认固件已能发出状态 6，完整联动效果仍待用户复测。协议布局、超时与兼容性见 [PROJECT.md](PROJECT.md)，部署与验证时间见[变更记录](../RaspberryPi/CHANGELOG.md)。

所有有效迭代均在本目录完成；工作区根目录的 `备份/` 只保存历史快照。
