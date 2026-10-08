# Vision development

该目录用于不依赖 ROS 系统 Python 的视觉研发流程。P12 使用 uv 管理的
Python 3.13 环境，当前锁定 PyTorch CUDA、Ultralytics、ONNX 和 ONNX Runtime。

```text
vision/
├── configs/       # 训练、数据增强、导出配置
├── training/      # YOLO 训练与评估入口
├── preprocessing/ # 数据清洗、标注转换和切分
├── export/        # ONNX / NCNN 导出与一致性检查
└── notebooks/     # 仅用于探索，不作为部署入口
```

原则：

- 使用 uv 创建独立环境，不向系统 Python 执行 `pip install`。
- OpenCV 九宫格、交通锥算法应支持录制视频离线复现。
- YOLO 第一版使用 YOLO11n，类别为 `sea_cucumber`、`turtle`、`starfish`。
- 导出模型进入 `models/artifacts/`，并更新模型清单；权重文件不提交 Git。
- ROS 推理节点最终位于 `src/legacy/ros2/auv_vision`，只消费发布后的模型，不包含训练环境。

## P12 环境

```fish
cd /home/hanwen/auv/vision
set -gx UV_CACHE_DIR /home/hanwen/auv/.cache/uv
uv sync --frozen
uv run python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

不要激活该虚拟环境后运行 ROS，也不要把 Ultralytics 安装到系统 Python。

## 数据集结构

真实数据放在被 Git 忽略的目录：

```text
datasets/processed/sea_cucumber_v1/
├── images/{train,val,test}/
└── labels/{train,val,test}/
```

标签使用 YOLO 检测格式：`class cx cy width height`。类别顺序固定为：

```text
0 sea_cucumber
1 turtle
2 starfish
```

先验证数据，验证失败时训练脚本拒绝启动：

```fish
uv run python preprocessing/validate_dataset.py \
  ../datasets/processed/sea_cucumber_v1 \
  --report ../datasets/processed/sea_cucumber_v1-validation.json
```

校验包括损坏图片、缺失标签、非法类别、越界框、孤立标签，以及跨
train/val/test 的重复图片内容。

如果原始资料尚未切分，可先整理为 `datasets/raw/sea_cucumber_v1/{images,labels}`，
再用固定随机种子生成不可变的处理集：

```fish
uv run python preprocessing/split_dataset.py \
  ../datasets/raw/sea_cucumber_v1 \
  ../datasets/processed/sea_cucumber_v1 \
  --seed 42 --val-fraction 0.2 --test-fraction 0.1
```

该工具只复制文件，不删除原始资料；目标目录非空时必须显式传入 `--force`。

## 训练、评估与导出

```fish
uv run python training/train.py

uv run python training/evaluate.py \
  runs/sea_cucumber_yolo11n/baseline_v1/weights/best.pt \
  --output runs/sea_cucumber_yolo11n/baseline_v1/test-metrics.json

uv run python export/export_onnx.py \
  runs/sea_cucumber_yolo11n/baseline_v1/weights/best.pt \
  ../models/artifacts/sea_cucumber_yolo11n-v1.onnx

uv run python export/verify_onnx.py \
  runs/sea_cucumber_yolo11n/baseline_v1/weights/best.pt \
  ../models/artifacts/sea_cucumber_yolo11n-v1.onnx \
  --report runs/sea_cucumber_yolo11n/baseline_v1/onnx-comparison.json
```

只有真实训练、独立测试集评估和 ONNX 对照完成后，才能创建
`models/manifests/sea_cucumber_yolo11n-v1.yaml`。不得填写占位哈希或指标。

清单由真实文件和评估 JSON 自动生成：

```fish
uv run python export/create_manifest.py \
  ../models/artifacts/sea_cucumber_yolo11n-v1.onnx \
  runs/sea_cucumber_yolo11n/baseline_v1/onnx-comparison.json \
  ../models/manifests/sea_cucumber_yolo11n-v1.yaml
```

训练工具测试：

```fish
set -gx PYTEST_DISABLE_PLUGIN_AUTOLOAD 1
uv run pytest -q
uv run ruff check .
```
