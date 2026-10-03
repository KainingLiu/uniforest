# YOLO 方块识别

Uniforest 项目内的 YOLO 视觉目录。全部方案统一维护在 [实施方案](docs/IMPLEMENTATION_PLAN.md)，涵盖任务、模型、损失、数据集、训练优化、hybrid 接入与验证步骤。

已实现抓取前的自动原图采集：仅在橙色/紫色搜索和对准阶段后台保存图片和 SQLite 索引，
其他比赛阶段暂停；显式手动补拍入口仍可独立采集。
用法、保存位置、配置和标注边界见 [数据采集说明](docs/DATA_COLLECTION.md)。
已用第一批 418 张图完成候选标注逐图审阅、分组划分和 YOLO11n-seg 首轮离线训练。
测试结果、权重位置及局限见 [首轮训练报告](docs/FIRST_TRAINING_REPORT.md)。
另保留了一个尚未通过三维线框验收的四参考角点 YOLO11n-pose 实验模型，见
[方块几何辅助三维姿态报告](docs/CUBOID_POSE_REPORT.md)。
针对其隐藏边交叉问题，已重新生成并审阅六个**可见轮廓**关键点标签，
从官方预训练权重微调 YOLO11n-pose；数据划分、测试指标、权重位置和叠图限制见
[六点重标注训练报告](docs/VISIBLE_OUTLINE_V2_REPORT.md)。
新模型仍在本机离线验证，画面边缘的截断方块会偶发多余或缺失边线。
比赛程序中的分割推理与 hybrid 接入仍待实现。
