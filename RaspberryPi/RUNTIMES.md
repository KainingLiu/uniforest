# 主分支与 game 独立运行

整合基线：GitHub main `cefa25a`（2026-10-02 新场地标定）。

## 本次变更范围

| 行为变化 | 受影响模块 | 验证 / 文档 |
| --- | --- | --- |
| 三个原入口默认运行 main，显式 `--runtime game` 选择独立实现 | main.py、task2_main.py、robot.py 的脚本启动段；utils/runtime_launcher.py | 主分支函数 AST、旧指令与流程预览比对、导入隔离、非法参数拒绝 |
| 两套入口自动保存输出和诊断 | utils/run_logs.py、utils/diagnostics.py 及 game 对应文件 | 退出码、异常、线程写入、100 次保留、活跃日志保护 |
| game 同步 GitHub 新场地视觉标定 | game/vision/opencv 的相机设置、几何常量、偏置、标定记录 | 与 main 标定逐文件 / 数值比对；保留 game 视觉扩展 |
| game 代码及扩展固件独立存放 | RaspberryPi/game/、Uniforest_A/game/ | 双套 Python 测试、双套 Debug 构建；旧命令差分及会话接手测试 |

2026-10-03 按用户明确要求，在 main 和 game 同步实现橙块跨区补抓，
main 的 competition/task2、数量检查及 Robot 数量检查适配器允许这一项行为变化；
详见 [主策略约定](Strategy/README.md)。
其余受保护的源码、标定、协议、Agent、桌面入口、依赖和默认下位机工程继续保留基线校验。
三个入口的 CLI 参数解析及分流语义保持，main 不依赖 game 的模块。
协议未变：主分支命令编号、载荷长度、字节序、遥测布局、200 ms 失联处理均保留。
game 原有扩展协议继续只在其独立实现中启用。

## 启动

在 RaspberryPi 目录内，原命令继续使用：

```bash
python main.py --strategy PlanA
python main.py --task task2-1
python robot.py --action build
python task2_main.py --variant 1
```

选择 game 时仅添加新参数；建议先预览：

```bash
python main.py --runtime game --strategy PlanA --show-plan
python main.py --runtime game --strategy PlanB --show-plan
python task2_main.py --runtime game --variant 1 --show-plan
python robot.py --runtime game --help
```

`--runtime=game` 同样有效；省略该参数或指定 `--runtime main` 使用主分支。
game 的优化参数见 [game 运行指南](game/README.md)。主分支不会接受或自动启用 game 优化参数。
现有桌面图标、安装脚本、Agent 与直接导入原模块的代码继续使用 main。

game 以独立 Python 进程映像启动，沿用当前解释器，不在主分支进程里导入 game 的 robot/Strategy/protocol。
两套运行时同属于仓库中的普通目录，Git 拉取即包含全部源码，无需额外的 submodule 初始化。
相对路径参数仍相对于调用时的工作目录；明确指定的配置、日志路径按原参数处理。

停止原任务、退出原进程、核对起始位置后再切换；不提供运行中自动切换或失败后自动回退。
现有桌面启动锁保留，直接调用 Python 的原有锁行为保持；不要同时启动两套控制进程。

## 日志

main：`RaspberryPi/logs/runs/`；game：`RaspberryPi/game/logs/runs/`。
每次运行保存 console.log、diagnostics.jsonl、run.json，保留最近 100 次，活跃记录受锁保护。
run.json 标明 runtime、入口和退出码。主分支日志建立失败时告警后继续原入口，运行中的日志写入失败不会中断控制。显式 `--diagnostics-log` 继续额外写入指定文件。
`UNIFOREST_RUN_LOG_DIR` 可覆盖目录；日志和实拍数据不提交 Git。

## 标定与固件

两套代码包含 main cefa25a 的相机曝光、Task1/Task2 固定几何与橙色零偏置；只同步已有确认参数。
本次同步本地、GitHub 与树莓派，保留现有标定；仅执行无硬件检查，视觉准确率与 game 实机行为需后续现场验证。

`Uniforest_A/` 的原默认工程保持。`Uniforest_A/game/` 是 game 所需的可选增量固件工程；
它保留最新 main 的原动作表，并通过扩展编号提供 game 会话。game 比赛入口遇到旧固件会拒绝启动。
需要切换到该可选固件时，使用 CLion 配置的 OpenOCD + DAPLink 编译、烧录；本次不烧录。
同一增量固件的新旧命令兼容性由主机测试检查，物理行为仍需现场验证。

## 无硬件验证

在 RaspberryPi 下分别运行，避免同名模块共用测试进程：

```bash
python tests/import_smoke.py
python -m unittest discover -s tests -q
cd game
python tests/import_smoke.py
python -m unittest discover -s tests -q
```

两个固件工程分别执行 `cmake --preset Debug` 和 `cmake --build build/Debug`，仅作编译验证。

## 验证限制

主分支兼容性与自动日志单独验证。game 全量测试存在整合前已复现的问题；
详细结果及 ARM 工具链限制见 [game 验证记录](game/INTEGRATION_VALIDATION.md)。
当前同步不包含固件烧录或实机动作。

本地 main 完整 151 项检查通过；跨平台兼容检查调整后 10 项入口测试再次通过。
树莓派 main 完整 151 项检查通过（2 项缺固件源码的检查跳过）。
game 的完整测试结果及既有失败见上述验证记录，不能作为完整实机运行已验证的依据。
