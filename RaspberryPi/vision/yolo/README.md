# YOLO 方块识别

Uniforest 项目内的 YOLO 视觉目录。全部方案统一维护在 [实施方案](docs/IMPLEMENTATION_PLAN.md)，涵盖任务、模型、损失、数据集、训练优化、hybrid 接入与验证步骤。

已实现抓取前的自动原图采集：仅在橙色/紫色搜索和对准阶段后台保存图片和 SQLite 索引，
其他比赛阶段暂停；显式手动补拍入口仍可独立采集。
用法、保存位置、配置和标注边界见 [数据采集说明](docs/DATA_COLLECTION.md)。
训练、分割推理及 hybrid 接入仍待实现。
