# Ubuntu AI 交接：任务一开发增量

日期：2026-10-12。以包含本文件的最新 Git 提交为本轮源码基线；上一轮 Ubuntu 报告对应 `37acfa2` 之前的源码，不能替代本轮验证。本轮没有部署、烧录或运动授权。

## 最新决定：全部改在 Ubuntu 验证

用户已要求停止在 Pi 上构建，并已将 Pi 断电。本轮 Pi 仅取得配置/资源/服务摘要、上传源码及完成 CMake 配置，构建未取得完成结果，也未执行新增测试、未部署服务、未烧录。不要继续 SSH、恢复 Pi 构建或把它计为编译通过。

本轮最新素材附件见 [Ubuntu 感知验证交接](https://github.com/laihanwen/xjtu_rover_team_lib/releases/tag/ubuntu-perception-validation-20261012)。包含用户已填工作簿、46 帧人工审阅、39 个采用帧的分组无损样本及 C++ 对照、逐文件 SHA256；原尺寸图片仍复用原交接附件。39 帧全部为部分外框；不能要求这些帧通过完整四角链，也不能将颜色分量当作已分类锥。已有旧素材附件继续保留，不覆盖用户填写文件。

已填表只完成关键帧筛选：39 采用、7 跳过，主体掩膜/格位语义/节点关联/回访事件尚未填。Windows 独立复核：ID18 解码 1 帧（F035）、完整网格 0、颜色分量 23 帧、管线候选 30 帧；不是局部度量定位或训练完成。

新审阅包使用单独校验工具，不能套用旧包的 native/labelled 目录约定：

```bash
python3 tools/auv/verify_perception_review_bundle.py /path/to/perception-review
python3 tools/auv/verify_perception_review_bundle.py /path/to/perception-review \
  --replay build-task-one-ubuntu/core/auv_core/auv_task_video_replay \
  --output build-task-one-ubuntu/filled-review-replay
```

将新的逐帧输出通过包内 index.json 的 F 编号对齐，保留 Windows 对照和最新 Ubuntu 二进制版本。样本已经矫正，不再次去畸变；1 FPS 是稀疏样本播放频率，不是运动时间，不能用于光流闭环或重建同步姿态。输出目录须不存在。

实际 Pi 摄像头服务此前配置 `tag36h11`，与已确认的 `tag16h5 / ID18` 不一致。后续部署准备须修正目标字典/ID及选定标签相机，保留已生效双摄640×480参数；不能直接覆盖为低分辨率模板。当前服务串口为空、运动关闭，ROV 桥占 UART，这只是共享摄像头观察入口，不能据此认为 AUV 固件与自主任务已部署。

## 交给 Ubuntu AI 的任务

在保护当前工作区修改后更新远程 `main`，完成下述隔离软件验证。修复真实失败并重跑受影响测试，保存原失败证据；不要删除测试、放宽阈值、伪造标定验证或连接生产 UART。结果形成中文报告，绑定完整 Git 提交号、构建参数、依赖版本、二进制 SHA256、测试数量、跳过项与限制，并提交推送供 Windows 端拉取。

本轮包括：颜色/主体分割双方案及 agreement 拒绝、受门控的管中心线实验、Pi 同钟遥测记录与 boot/epoch 校验、任务一局部动态夹具、上浮前空格移位、四锥后的退出状态/控制、标签与网格相对坐标、主体训练和 ONNX 导出工具。没有真实 shape 权重，默认仍为颜色方案；所有生产导航/退出未验证参数保留门控。

## 1. 更新与环境

```bash
git status --short
# 有未提交修改先保存；不要 reset --hard 或直接覆盖。
git fetch origin
git switch main
git pull --ff-only origin main
git rev-parse HEAD
```

默认 Ubuntu shell 为 fish，下面脚本明确由 bash 运行。保留现有 ROS/CUDA/PyTorch 环境，不升级或卸载。需要 C++17、CMake、Ninja、OpenCV C++ >=4.7（含 dnn、objdetect）、yaml-cpp、cpp-httplib、ffmpeg、Python cv2/NumPy/PyYAML。先检查已安装依赖，不重复安装；缺失时可用 Ubuntu 包 `build-essential cmake ninja-build libopencv-dev libyaml-cpp-dev libcpp-httplib-dev ffmpeg python3-opencv python3-numpy python3-yaml`。Python 工具运行在已有可用隔离环境；不向系统 Python pip 安装。

## 2. 完整 Runtime 与回归

驾驶台新增感知接口后的增量说明见 [AUV 感知检查](auv-perception-console-20261012.md)。本轮还应注册并执行 `perception_http_readonly`：在 INIT/DISARM、空串口条件下检查真实 C++ 标签解码、JPEG 和 unknown 格位，不能只看演示画面。便携新鲜度测试为 `perception_view_freshness`。

在仓库根目录：

```bash
bash tools/auv/validate_task_one_ubuntu.sh build-task-one-ubuntu
```

可在 bash 中设置 `PYTHON=/path/to/venv/bin/python JOBS=3`；fish 用 `env PYTHON=/path/to/python JOBS=3 bash ...`。构建目录必须来自同一系统/生成器；如果已有本轮同目录缓存则增量复用，不删除缓存。脚本不操作服务、SSH 或硬件。

检查 CTest 实际注册清单，至少应含 `local_task_one_closed_loop`、`a2_closed_loop`、`grid_exit_closed_loop`、`tag_grid_binding`、`dual_cone_vision`、`shape_model_inference`、局部地图/关联/里程计及旧门控测试。不要预填旧 27/27。本地 Windows 最新便携 20/20、训练清单 2/2 已通过，但 `runtime.cpp` 和 Linux 局部动态入口未执行。

`local_task_one_closed_loop` 依次运行正常场景及表面地标缺失场景。正常场景应通过水下局部地图累积/冻结、空格移位、上浮、等待新阶段前视位姿、四锥无重复访问、`EXIT_GRID`、真实越界确认到 COMPLETE。缺失场景应 FAULT 且 DISARM；旧水下位姿不得授权表面遍历。合成摄像头/虚拟 MCU 不是实测参数，夹具内 verified=true 不得迁入生产配置。

失败时单独运行：

```bash
ctest --test-dir build-task-one-ubuntu -R local_task_one_closed_loop --output-on-failure -V
```

先查看保存的 config、status snapshots、UART targets、events、末帧与 stderr，区分识别、关联、地图冻结、ARM 门、控制和定位新鲜度失败。不能靠延长所有超时或移除四锥/全机越界约束“通过”。源码已添加出口新 epoch 清理及再定位等待，需要确认没有旧位姿泄漏。

## 3. 真实素材回归

沿用 Release `validation-20261012` 的 `ubuntu-validation-20261012.zip`，SHA256 `c00bc14aebf450006d02bf101ae664fbb60cc36fe2afb2c6427d661da2e8696f`。已有本地素材可复用，不需重新下载。包内清单记录旧源码 `94235ae`：这是素材来源版本，**本轮执行程序必须来自最新 Git**；不能把素材旧清单当成新二进制验证版本。

```bash
python3 tools/auv/verify_ubuntu_materials.py /path/to/materials
python3 tools/auv/verify_ubuntu_materials.py /path/to/materials \
  --replay build-task-one-ubuntu/core/auv_core/auv_task_video_replay \
  --output build-task-one-ubuntu/real-semantic-new
```

输出目录须不存在，保留旧结果。比较标签 ID18、颜色候选、分类/unknown、管线与拒绝原因；1010 样本由 978 原录像无损样本和 32 旧筛选帧组成。旧人工“4 锥”不是每格真值，底座不能判主体类别。管中心线实验在 L 素材上的召回曾下降，生产 `local_pipe_centres` 保持 false，不因合成测试通过启用。

旧素材没有足够同钟传感器输入，仍不能开展真实米制闭环验收。`prepare_sensor_replay.py` 仅接受新同 boot/epoch 记录与完整实测模型；不要用 PC UTC、人工 1 FPS 播放时间、池深常数或重复深度归零补造位姿。

## 4. 人工标注与方案 B

交接附件入口：[task-one-handoff-20261012](https://github.com/laihanwen/xjtu_rover_team_lib/releases/tag/task-one-handoff-20261012)。包含 Excel、原尺寸样图与标注规则；附件发布后以 SHA256 校验清单为准。

配套 Excel/原尺寸图片在本次交接 Release 附件，源码仓库不保存图片数据。Excel 是索引和审核表，不生成掩膜。先补每帧可见格、锥主体类别/轮廓、标签角点、管接头节点及真实回访顺序；画外/遮挡格保持 unknown。

主体掩膜 PNG：0 背景/底座，1 圆锥主体，2 方锥主体。坐标必须对应指定原尺寸图片，Excel 缩略图不能作为标注坐标。训练输入须与 C++ 网格图像空间一致；从原帧到网格变换尚未成立的帧只做原帧轮廓审核，不直接混入训练。训练/验证分组隔离整个录像及重复实例，不随机拆相邻帧。

用既有隔离 PyTorch 环境运行 `vision/training/train_cone_body.py`；`--check-only` 先检查清单。真实标注未提供前不训练或宣称 shape ready。训练后独立核验 ONNX 输入输出、C++ 推理、每类 IoU/错误、条纹变化/遮挡及 Pi 预算；`shape_model_verified` 由验收决定。

## 5. 回传要求与下一步边界

脚本保存到 `build/ubuntu-task-one-evidence/<UTC>/`（Git 忽略）：日志、源码提交/修改补丁、动态运行证据、二进制哈希与校验清单。失败也应保留。将结果摘要写入 `docs/testing/ubuntu-task-one-results-20261012.md`，说明总数、通过/失败/跳过、完整 Runtime 是否编译及真实素材的来源/执行版本；大证据包压缩上传 Release，普通 Git 不放日志、录像、权重或凭据。若修复代码，按中文提交并推送；保留修复前后对应关系。

完整软件通过后仍需现场配置核验、无动力联调与分阶段水池验收。退出需实测机体包络、净空及允许出口；绝对深度偏移和完整航向映射未取到不猜填。不能把这份交接当作烧录、服务重启或 ARM 授权。
