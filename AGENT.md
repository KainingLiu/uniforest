# Uniforest — RoboGame 2026 机器人开发

## 项目概述
RoboGame 2026 竞技组 Uniforest 队的电控与视觉开发工作区。

请你详细阅读规则手册 @RoboGame2026 竞技组规则手册2_1.pdf ，充分了解比赛规则，机器人设计要求等关键约束。

请你仔细阅读本队的计划书 @RoboGame 2026 Uniforest队计划书.pdf ，这里详细描述了本队计划采取的技术路径。

### 电控部分

我们使用大疆官方RoboMaster A板作为主控模块

请你在需要的时候读取开发板使用说明文件 @开发板说明.pdf 和原理图 @RoboMaster 开发板A型 原理图.pdf

这里是我们设计和调试主控模块（开发板）程序的工作区 @STM32 （链接GitHub项目：https://github.com/KainingLiu/uniforest），开发板的主程序在这里编写，调试，编译，烧录。

### 算法和视觉部分

我们使用Raspberry Pi 5 8GB作为上位机，负责算法和视觉的实现

树莓派的用户名：uniforest；密码请向队内管理员获取，不在公共仓库中保存。

代码工作区在 @RaspberryPi （链接GitHub项目：https://github.com/KainingLiu/uniforest）
