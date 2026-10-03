# AUV 打标环境快速入口

此目录使用独立的 Label Studio 环境，不修改 ROS 系统 Python，也不与
`vision/.venv` 混用。任务类型为矩形框目标检测，类别顺序与
`vision/configs/classes.yaml` 一致：

```text
0 sea_cucumber
1 turtle
2 starfish
```

## 安装与启动

```bash
cd /home/hanwen/auv/annotation
export UV_CACHE_DIR=/home/hanwen/auv/.cache/uv
uv sync --frozen
./start.sh
```

浏览器打开 <http://127.0.0.1:8080>。首次使用时在本机创建账户，然后新建项目，
在 **Labeling Setup → Code** 中粘贴 `labeling-config.xml` 的内容。

## 图片与标注

将待标注图片放到 `datasets/interim/sea_cucumber_v1/images/`。标注项目使用
`labeling-config.xml`，完整的项目创建、类别定义、画框规范、负样本处理和质检步骤见：

> [P12 水下目标打标教程](../docs/p12-annotation-guide.md)

## 导出

在 Label Studio 的 **Export** 中选择 **YOLO**。导出后先核对 `classes.txt`
必须严格为上述三个类别及顺序，再整理到：

```text
datasets/raw/sea_cucumber_v1/
├── images/
└── labels/
```

之后运行 `vision/preprocessing/split_dataset.py` 和
`vision/preprocessing/validate_dataset.py`。Label Studio 的运行数据库与上传缓存
位于 `annotation/data/`，已被 Git 忽略。

停止服务时回到运行 `./start.sh` 的终端按 `Ctrl+C`。
