# Vision development

该目录用于不依赖 ROS 系统 Python 的视觉研发流程：

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
- ROS 推理节点最终位于 `src/auv_vision`，只消费发布后的模型，不包含训练环境。
