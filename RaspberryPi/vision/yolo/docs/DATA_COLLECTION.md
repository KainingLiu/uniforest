# 方块数据自动采集

比赛和调车程序启用 cube 视觉时，默认同时采集未标注图片。数据保存在运行程序的
电脑或树莓派本地，采用 JPEG 原图、逐图 JSON 信息和 SQLite 索引。数据文件不提交 Git。

## 正常使用

在 `RaspberryPi` 目录按原来的方式启动流程即可，例如：

```bash
python main.py --task task1-r1
python main.py --task task2-r2
python task2_main.py
```

这些入口仍执行各自原有的机器人动作。自动采集复用正在运行的 cube 摄像头，
不另外打开摄像头，不修改路线、机械动作、曝光或控制参数。
`robot.py --vision` 和启用视觉的 Agent Robot 会使用同一采集机制。
未启用视觉的纯底盘/通信调试不会为了采集自动启动相机。

启动时显示一次数据目录，结束时显示保存张数、重复跳过数和队列丢弃数。
采集错误或存储达到上限时提示并停止本次采集；已有控制和视觉流程继续运行。

```bash
# 查看累计数量、最近批次和任务阶段分布；不会打开摄像头或串口
python tools/collect_cube_data.py stats
python tools/collect_cube_data.py stats --json

# 本轮停用采集
python main.py --task task1-r1 --no-collect-data

# 保存到已经挂载且当前用户可写的外部存储
python main.py --task task2-r1 --dataset-dir /media/uniforest/DATA/cube_dataset
python tools/collect_cube_data.py stats --dataset-dir /media/uniforest/DATA/cube_dataset
```

`main.py`、`task2_main.py` 和 `robot.py` 调试入口均支持
`--no-collect-data`、`--dataset-dir`。其他复用 Robot 的入口可以通过环境变量配置：

```bash
export UNIFOREST_DATASET_DIR=/media/uniforest/DATA/cube_dataset
export UNIFOREST_DATASET_BATCH=20260924_roomA_layout01
# 所有入口暂时停用：export UNIFOREST_COLLECT_DATA=0
```

环境变量只影响当前终端及其子进程。长期设置可以修改
[`collection_config.json`](../collection_config.json)。优先级：入口参数 > 环境变量 > JSON。
相对数据路径以 `RaspberryPi` 为基准，不受启动时工作目录影响。

## 补拍独立场景

以下命令只使用相机，不连接串口，也不发送底盘或机械臂命令。先人工摆好方块：

```bash
python tools/collect_cube_data.py capture --duration 60 --scene "三块橙色紧贴，槽位半遮挡"
python tools/collect_cube_data.py capture --duration 60 --profile building --scene "三层混色建筑"
python tools/collect_cube_data.py capture --duration 30 --scene "空槽位和反光"
```

可用 `--camera cube` 指定相机、`--hz 4` 调整抽样频率，`--duration 0` 持续采集至 Ctrl+C。
`capture` 是显式采集命令，会启用本次采集；它仍遵守存储和队列限制。
正常跑流程时使用内置采集，不同时运行第二个 `capture` 进程争用摄像头。

## 保存内容

默认位置：`RaspberryPi/vision/yolo/data/collection/`。

```text
collection/
  catalog.sqlite3
  runs/
    20260924T120000_123456Z_ab12cd34/
      session.json
      images/
        000001.jpg
        000001.json
        000002.jpg
        000002.json
```

- 每个 Robot/独立采集实例创建唯一 session，跨进程、重复运行不覆盖旧图片。
- 每次 `run_tasks()` 创建独立 `flow_id`，区分同一 Agent 进程多次执行流程。
- 图片使用相机返回的完整分辨率、原始 BGR 内容编码为 JPEG，不缩小到 320×320，
  不画检测框，不裁 ROI，不把上半部涂黑。JPEG 有损压缩，默认质量为 95。
- 逐图信息包含 session/flow/batch、任务、阶段、视觉 profile、相机来源、请求设置、
  驱动报告设置、内参快照、图像宽高、UTC Unix 时间及单调时间。
- 时间表示主机成功读完图像的时刻，并非传感器曝光时间。驱动报告 FPS 不作为实测帧率。
- Task0/Task1/Task2 的既有状态会自动记录，如 `ORANGE_SEARCH`、`ORANGE_ALIGN`、
  `PURPLE_SEARCH`、`BUILDING_ALIGN`、`COUNT_CHECK`、`BUILD`。第一轮类名记录为
  `Task1`/`Task2`，第二轮记录为 `Task1-R2`/`Task2-R2`。
- `scene_note` 供人工补拍记录；任务/profile 仅用于筛选，不能证明图片一定包含某种颜色、
  某个数量、紧贴场景或空场景。
- `annotation_status` 一律为 `unlabelled`，不生成训练标签或自动把无检测画面当成负样本。

SQLite 的 `sessions` 表保存批次和状态，`frames` 表保存图片路径、时间、任务、阶段、
profile 及完整元数据 JSON。图像路径相对数据根目录，整个目录可以一起复制迁移。
正常停止采集后复制整个目录；运行中的 SQLite 可能还有 `-wal`/`-shm` 文件，
不要只复制正在写入的数据库主文件。

## 采样及存储限制

| 配置 | 默认值 | 作用 |
| --- | --- | --- |
| `enabled` | `true` | 自动采集开关 |
| `sample_hz` | 2 | 常规抽样最高每秒约 2 帧，实际受新帧频率限制 |
| `static_keep_seconds` | 5 | 近乎不变的画面仍约每 5 秒保留一张 |
| `dedup_mean_abs_diff` | 2 | 低分辨率彩色缩略图相似度阈值；局部明显变化仍保留；设为 0 关闭去重 |
| `jpeg_quality` | 95 | JPEG 编码质量 |
| `queue_size` | 4 | 后台保存队列上限，积压时丢弃本次新样本 |
| `max_images_gb` | 8 | 索引记录的累计 JPEG 字节预算，单位 GiB |
| `min_free_gb` | 2 | 为磁盘保留的空闲空间，单位 GiB |
| `max_session_images` | 5000 | 单次 session 图片上限 |
| `batch_tag` | 空 | 同一拍摄批次/摆放组的人工标记 |

任务阶段、flow、人工场景说明或 profile 改变时，尝试保留下一张新帧，跳过常规抽样
间隔和去重；队列和存储限制仍然生效。采集不以传统检测器是否找到目标作为条件，
因此漏检、空场和遮挡画面都有机会保留。

磁盘检查、编码和 SQLite 写入由后台线程完成，相机线程仅在选中帧时复制图像并尝试
入队。队列满时立即返回。关闭程序时先完成原有机器人停止/断开流程，再等待最多
2 秒刷新采集队列。Pi 上的 CPU、存储开销及控制帧率仍需现场测量。

达到限制会结束本次采集，不自动删除历史数据。清理/归档整个库或更换数据根目录后
重新启动采集；手工删除单张 JPEG 不会同步减少 SQLite 中的预算统计。
断电或强制杀进程可能丢失队列内数据，并留下尚未入索引的图片/JSON，或状态仍为
`running` 的旧 session。保留这些文件供检查，不把该状态直接视为程序仍在运行。

## 标注和后续训练

本次实现负责积累分帧图片和索引，不自动训练模型，也不推断真实物理坐标。
低频抽样图片不能替代原帧率连续视频的跟踪/延迟验收。
毫米定位真值、每块可见轮廓、遮挡和相邻关系需要人工测量或审核后补充。
完整采集范围和实例标注语义沿用 [`IMPLEMENTATION_PLAN.md`](IMPLEMENTATION_PLAN.md)。

自动流程只会采到当时实际出现的光照和摆放。调车之外仍应补拍紫色、弱缝、重遮挡、
混色建筑等不足场景。先人工标注一小批并试训，再按模型错误选择补拍内容。

划分训练/验证/测试时，以 `split_group` 为分组依据起点；它使用 `batch_tag`，
未设置时使用 session ID。同一场景跨多次重启采集应设置相同 batch，或者后期合并
近重复组，避免同一摆放的相邻帧进入不同集合。每个集合的场景和颜色仍需人工检查。

## 验证

```bash
python -m unittest tests.test_cube_collection -v
python tools/check.py
```

测试使用临时目录和模拟相机，不连接机器人。覆盖真实 JPEG/JSON/SQLite 保存、
无目标画面、阶段切换、限流去重、队列满、磁盘不足、保存失败清理、累计容量限制、
只读统计和硬件停止先于磁盘收尾。
