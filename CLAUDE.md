# Uniforest - RoboGame 2026 机器人开发

本文件与 [AGENTS.md](AGENTS.md) 使用相同的项目边界。完整协作约束以 `AGENTS.md` 为准。

- `RaspberryPi/`：持续迭代的上位机程序。
- `Uniforest_A/`：持续迭代的下位机程序。
- `备份/`：每日成果的历史快照，默认只读，不作为当前实现。

下位机默认入口为 `Uniforest_A/Core/Src/main.c`。烧录统一使用 CLion 配置的 OpenOCD + DAPLink 工具链，不使用命令行直接烧录。
