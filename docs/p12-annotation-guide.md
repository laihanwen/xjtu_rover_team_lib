# P12 水下目标打标教程

本教程用于建立 `sea_cucumber`、`turtle`、`starfish` 三类目标的 YOLO 检测数据集。
打标工具是本地运行的 Label Studio；数据库、上传缓存、图片和标签均不会提交到 Git。

## 1. 目录与数据流

```text
相机视频 / 原始照片
        ↓ 筛选、去重、抽帧
datasets/interim/sea_cucumber_v1/images/
        ↓ Label Studio 矩形框标注
datasets/raw/sea_cucumber_v1/{images,labels}/
        ↓ 固定随机种子切分、完整性校验
datasets/processed/sea_cucumber_v1/{images,labels}/{train,val,test}/
        ↓ YOLO11n 训练、评估、ONNX 导出
models/artifacts/sea_cucumber_yolo11n-v1.onnx
```

`raw` 保存已确认的原始标注结果，原则上只追加、不覆盖；`processed` 是某次训练使用的
确定版本。不得直接在 `processed` 目录中手工修标签。

## 2. 安装和启动 Label Studio

项目使用 Python 3.12 和独立 uv 环境，不影响 `/usr/bin/python3`、ROS 2 或
`vision/.venv`。

fish：

```fish
cd /home/hanwen/auv/annotation
set -gx UV_CACHE_DIR /home/hanwen/auv/.cache/uv
uv sync --frozen
./start.sh
```

bash：

```bash
cd /home/hanwen/auv/annotation
export UV_CACHE_DIR=/home/hanwen/auv/.cache/uv
uv sync --frozen
./start.sh
```

浏览器打开 <http://127.0.0.1:8080>。服务只监听本机，首次进入时创建一个本地账户。
运行数据保存在 `annotation/data/`；需要停止时在启动终端按 `Ctrl+C`。

## 3. 准备待标注图片

创建本地目录并放入筛选后的图片：

```fish
mkdir -p /home/hanwen/auv/datasets/interim/sea_cucumber_v1/images
```

图片建议保留原始分辨率，格式使用 JPG 或 PNG。入库前剔除：

- 全黑、严重欠曝、过曝或无法辨认的画面；
- 运动模糊到无法确定目标轮廓的画面；
- 完全重复的图片；
- 大量几乎相同的视频连续帧；
- 包含不应公开的人员、地点或比赛资料的图片。

同一段视频的相邻帧高度相似。应先降低抽帧频率，并记录来源视频或采集批次；同一批次
只能进入 train、val、test 中的一个集合，否则测试指标会虚高。

推荐文件名保留来源信息，例如：

```text
pool01_down_20261003_000123.jpg
pool01_front_20261003_000456.jpg
lake02_front_20261010_000081.jpg
```

## 4. 创建标注项目

1. 点击 **Create Project**，项目名填写 `AUV P12 sea cucumber v1`。
2. 打开 **Labeling Setup**，切换到 **Code**。
3. 复制 `annotation/labeling-config.xml` 的全部内容并保存。
4. 进入 **Data Import**，上传 `datasets/interim/sea_cucumber_v1/images/` 中的图片。
5. 打开一张图片，确认可以选择三个类别并绘制矩形框。

固定类别及 YOLO 编号如下，后续不得改变顺序：

| 编号 | Label Studio 值 | 判定范围 |
|---:|---|---|
| 0 | `sea_cucumber` | 任务目标海参，包括不同姿态、颜色和部分遮挡个体 |
| 1 | `turtle` | 海龟目标或比赛使用的对应模型 |
| 2 | `starfish` | 海星目标或比赛使用的对应模型 |

不要使用中文类别名、同义词或临时类别，否则导出的编号可能与训练配置不一致。

## 5. 画框规则

每张图片按以下规则处理：

1. 对所有清晰可辨的三类目标分别画一个矩形框，不遗漏小目标。
2. 框尽量贴合可见轮廓，允许少量边缘余量，不包含大面积水体或池底背景。
3. 两个目标重叠时分别画框；框可以互相重叠。
4. 部分遮挡目标只框可见范围；如果仍可可靠判断类别，则正常标注。
5. 被画面边缘截断但仍可辨认的目标，框到图像边界。
6. 无法确认类别时不要猜测，将任务标记为待复核。
7. 不要框倒影、阴影、气泡、灯光光斑或印刷图片，除非它们本身就是比赛目标。

### 负样本

没有任何三类目标、但环境有代表性的清晰图片是有价值的负样本。此类图片应提交为空标注；
导出整理时必须为图片保留同名的空 `.txt` 文件，例如：

```text
images/pool01_empty_0001.jpg
labels/pool01_empty_0001.txt  # 文件存在但内容为空
```

当前数据校验器将“缺少标签文件”视为错误，但允许空标签文件。

## 6. 标注质检

首轮标注后至少进行一次独立复核，重点检查：

- 每张图片是否漏框；
- 类别是否误选；
- 框是否过大、过小或越界；
- 同类目标的尺度和遮挡规则是否一致；
- 是否存在重复图片或连续帧泄漏；
- 负样本是否确实不含目标；
- 水下颜色偏移、浑浊、气泡、反光和低照度场景是否都有覆盖。

建议先标注 100–300 张做第一轮小数据训练，检查模型高频误检，再针对性补充困难负样本，
而不是一次性标完全部图片才开始验证。

## 7. 导出 YOLO 数据

在项目页面选择 **Export → YOLO** 并下载压缩包。解压后先打开 `classes.txt`，内容及
顺序必须严格为：

```text
sea_cucumber
turtle
starfish
```

将图片和标签整理为：

```text
datasets/raw/sea_cucumber_v1/
├── images/
│   ├── example_0001.jpg
│   └── example_0002.jpg
└── labels/
    ├── example_0001.txt
    └── example_0002.txt
```

每个标签行必须是 YOLO 检测格式：

```text
class_id center_x center_y width height
```

后四个值必须归一化到 `[0, 1]`。图片和标签必须同名，仅扩展名不同。

## 8. 切分和自动校验

进入训练环境：

```fish
cd /home/hanwen/auv/vision
set -gx UV_CACHE_DIR /home/hanwen/auv/.cache/uv
uv sync --frozen
```

如果所有图片已经按采集批次去重并确认可以进行图片级随机切分，执行：

```fish
uv run python preprocessing/split_dataset.py \
  ../datasets/raw/sea_cucumber_v1 \
  ../datasets/processed/sea_cucumber_v1 \
  --seed 42 --val-fraction 0.2 --test-fraction 0.1
```

如果数据来自多段连续视频，应先按视频或采集批次手工分组，再分别放入 train、val、test；
不要直接使用图片级随机切分。

验证处理后的数据：

```fish
uv run python preprocessing/validate_dataset.py \
  ../datasets/processed/sea_cucumber_v1 \
  --report ../datasets/processed/sea_cucumber_v1-validation.json
```

校验报告中的 `valid` 必须为 `true`，并人工确认三类目标的框数量合理。校验器会检查：

- 图片损坏；
- 标签缺失或孤立；
- 类别编号越界；
- 坐标或框越界；
- 跨 train/val/test 的重复图片内容；
- 各集合图片数、框数和负样本数。

## 9. 打标完成标准

数据进入正式训练前应同时满足：

- [ ] `classes.txt` 的类别和顺序完全正确；
- [ ] 每张图片都有同名标签文件，负样本使用空文件；
- [ ] 已完成至少一次人工复核；
- [ ] train/val/test 按采集来源隔离；
- [ ] 三个集合都包含代表性的水下光照、浑浊度和目标尺度；
- [ ] 自动校验结果为 `valid: true`；
- [ ] 数据版本、来源、相机、分辨率和切分种子已记录；
- [ ] 原始数据、标注导出和处理集均有本地备份。

满足这些条件后，再按 `vision/README.md` 执行 YOLO11n 训练、独立测试集评估、ONNX
导出和树莓派部署验证。

## 10. 常见问题

### 浏览器无法打开 8080

确认启动终端没有退出，再检查：

```fish
curl -I http://127.0.0.1:8080
```

如果端口被占用，可在 `annotation/start.sh` 中临时改为其他本机端口。

### 导出后类别编号错误

不要手工批量替换编号。先修正 Label Studio 项目的标签配置，再重新导出；训练配置中的
类别定义以 `vision/configs/classes.yaml` 为准。

### 校验器报告 missing label

每张图片都必须存在同名 `.txt`。无目标图片创建空标签文件，有目标图片检查文件名和
相对目录是否与图片一致。

### 能否直接用自动模型打标

首版模型尚未达到可信程度时，应以人工框为基准。得到经过验收的模型后，可以将预测
作为预标注，但每个框仍需人工确认，不能把模型预测直接当作真值。

## 参考资料

- [Label Studio 安装文档](https://labelstud.io/guide/install.html)
- [图片矩形框标注模板](https://labelstud.io/templates/image_bbox)
- [Label Studio 数据导出与 YOLO 格式](https://labelstud.io/guide/export)
