# 双视觉与同钟记录软件开发：2026-10-12

本轮源码基于 `37acfa2`，尚未提交或部署。目标是补齐视觉替代方案和下次同步采样的软件入口；不是声明真实局部地图、导航或全任务一已验收。STM32 固件、PID、混控及 ROV ARM 规则未改动。

## 两套交通锥方案

| 配置 | 方案与依据 | 缺失或冲突行为 |
|---|---|---|
| `cone_vision_mode: colour` | A：现有 CLAHE、HSV、轮廓几何与多帧投票 | 未测到完整支持轮廓仍 unknown；条纹分裂不能当整锥 |
| `cone_vision_mode: shape` | B：灰度锥体语义分割，输出 body 类别及实测像素轮廓 | 无权重、未验证权重、非法输出或裁切证据不会授权类别 |
| `cone_vision_mode: agreement` | 同一格、相同类别、中心接近才采纳 B 的 body 轮廓 | 类别冲突、不同实例或某方案缺证据保持 unknown，不静默回退 |

任务模板保持 A。B 使用 OpenCV DNN CPU 后端，ONNX 输入为 `[1,3,320,320]` 的复制灰度图、像素除以 255；输出为 `[1,3,320,320]` 的归一化 float 概率（背景、圆锥体、方锥体）。body 标签必须排除底座，不能把两类锥都有的方形底座当方锥标签。解码拒绝非法概率、跨格/裁切主体和同格多个实例，前景像素阈值 0.85。模型域是已矫正的地图图像，必须验证真实倾角与透视变换后的锥体。

配置入口：

```yaml
vision:
  cone_vision_mode: agreement
  cone_shape_model: /path/to/body-model.onnx
  cone_shape_model_id: measured-model-version
  cone_shape_model_verified: false
```

目前**没有真实训练和验收权重**，B 尚不可作为比赛识别方案。软件推理链已用明确的合成 ONNX 烟测验证，该模型只是“亮像素”测试夹具，不能用于分类或导航。需要从现有素材标注完整锥体与底座，按实例划分训练/验证集合，并随机化条纹颜色、数量、亮度和遮挡；测试真实两类锥后才确认模型。灰度输入降低颜色依赖，不保证天然对所有条纹变化不敏感。

两套候选数、选用模式和原因写入 `LOCAL_GRID` 事件；现有 map.complete 和定位门未放宽。局部地图采纳已验证 B 的 body 证据时不再受“只能单红色组件”的 A 专用门影响，但整格可见、定位唯一及多帧证据仍必需。

A 新增同格多个显著红色组件拒绝门，避免选择其中一个条纹当完整方锥。局部回放 CSV 同样记录两套候选数及决策原因；缺模型或推理失败时不能把格判 empty。

已指定路径和版本的未验收模型可以输出诊断候选，`cone_shape_model_verified: false` 仍阻止语义采纳和 empty 判定。因此可以先评估模型，不需为了回放而提前翻转验收标记。

B/双方案判空另外要求每个背景像素概率至少 0.995、覆盖格内 ROI 至少 99.5%，并满足已有已验证地板颜色门。低置信主体不会因为没有输出类别就当空格。

## 管中心与瓷砖干扰

局部管网检测新增可选距离变换脊线，`vision.local_pipe_centres` 默认 false；原颜色支持、格距、分支、唯一关联和 PnP 门保留。最大半宽 10 是鸟瞰/图像空间检测像素预算，不是现场管径声明。合成裁剪网格两条路径均通过，但真实 L 接头 138 个样本中，脊线候选出现帧只有 9，原图像边缘候选为 76；减少不能当作误检率改善。因此默认保留原路径，离线 CSV 同时输出 `pipe_segment` 与 `pipe_center_segment`。真实瓷砖、细管、阴影及模糊情况仍需像素真值回归。

## Pi 同钟采样

ROV 桥新增可选独立 `SensorRecorder`，UART 解析只入有界队列，写盘在独立线程。默认关闭，部署时由现有串口桥显式增加：

```sh
# 对现有 trial_server 服务参数追加，不能再启动第二个串口桥：
--sensor-recording-directory /var/log/auv-rov/sensors \
--sensor-model /path/to/measured-sensor-model.json
```

记录 STATUS/PID 的 Pi UART 接收单调时间、序号、MCU tick/重启 epoch、姿态、深度和 UTC。STATUS 不能证明独立传感器新鲜度；PID 中 `imu_fresh` 明确记录，当前权威固件在深度失效时写 NAN，因此有限 PID 深度用于独立有效性。它们是 MCU 生成时的标记，Pi 接收时间不是硬件测量时刻；运输延迟仍需实测。

默认队列 512，单会话 64 MiB，保留 256 MiB 磁盘空间；队列溢出、空间不足、预算耗尽及写盘错误均显示在 ROV 状态 `sensor_recording`，不会 ARM、DISARM 或改运动参数。服务结束生成日志 SHA256；记录自身源码哈希与测量模型。此入口仅在现有 ROV 客户端会话接收 UART 时记录，不声称无岸上控制的 AUV 已使用它；AUV 仍使用自己的任务记录器。

`sensor-model-template.json` 的未知值保持 null。必须从实测证据填写完整 IMU 符号/偏移，以及 `depth_absolute_offset_m`（绝对深度 = 实际记录深度 + 此偏移）；`depth_zero_m` 仅记录 MCU 口径，不再次扣零。未填写模型阻止米制回放。

Runtime 状态、JPEG/MJPEG 响应增加 Pi boot ID；PC 双摄 session 和 frame 侧车保存该 ID。若录制中标识改变或缺失，立即结束此次采集，防止跨 Pi 启动混用。旧服务仍可录制，但没有 boot ID 的旧素材不会被新同步工具当成已确认同钟输入。

## 同钟回放准备

```sh
python3 tools/auv/prepare_sensor_replay.py SENSOR_SESSION down_0001.frames.jsonl frames.json --phase underwater
auv_local_mapping_replay offline.yaml down_0001.avi frames.json output
```

视频侧车应保留在原会话目录旁（工具读取 camera_calibration 和图像空间）；前视文件同样可用。工具要求同一 Pi boot、因果 PID 接收时间差不超过 0.15 秒、两传感器有效、完整测量模型和明确阶段。重复 MCU tick 不刷新证据；重启/回绕混入需先分段；记录丢失保持拒绝。未匹配帧写审计，不补造传感器。Runtime 回放另外检查模型符号/偏移与配置一致，避免数值被二次转换。输出 JSON 是 YAML 的子集。

仍须核验对应 K/D、尺寸和阶段外参，模板的导航验证标记保持 false。新采样入口不会恢复旧日志中从未记录的同钟数据。

## 验证与待办

Windows Release 便携构建最新通过 20/20 CTest，包括两个新测试：双方案冲突/缺权重、主体概率/边界、管脊线，以及实际 OpenCV ONNX 推理（合成夹具）。新记录、同步工具及原 ROV 协议/录像测试另行记录。

Python 定向回归：记录 2/2、同钟准备 3/3、原桥协议/门控 10/10、原最新值导出 2/2、双摄录像 12 项中 11 通过、独立 ffmpeg 解码检查因依赖缺失跳过。新增 boot 变更退出用本地 HTTP 夹具通过。初次受限执行因临时目录/回环网络访问被拒绝失败；在可用环境重跑取得这些结果，没有把环境拒绝当作代码通过。

完整 `runtime.cpp` 的 HTTP boot 字段及事件改动不能在 Windows 完整运行，仍需 Ubuntu 增量编译和 CTest；不能沿用上次 27/27。本轮没有宣称 Pi 帧率改善、模型分类准确率或完成真实米制闭环。


## 任务一退出与上浮前移位

`grid_exit` 默认关闭。启用必须同时明确 `verified`、允许出口方向位掩码（北/东/南/西分别 1/2/4/8）、实测机体水平包络半径和净空。未知值保留 0 并拒绝启用。该配置只用于局部地图 A2；不是全局放宽定位范围。

冻结地图后若机体不在上浮格，先沿已观测 empty 格 BFS 移位，再执行原有中心对准和上浮。四锥访问后进入 `EXIT_GRID`，重新等待本阶段地标帧；禁止复用进入阶段前的位姿。退出路径不能重入已访问锥格；边界外定位范围只在退出阶段显式扩大，并保持窄走廊、姿态、帧龄、跳变和超时保护。机体包络完全越界后还需连续新地标确认，里程计不能单独宣布完成。

新增 `grid_exit_closed_loop` 验证动态移位/退出、阻断路线、旧帧等待和 FSM 结束条件；`tag_grid_binding` 验证同帧标签 PnP 与网格位姿的相对变换。标签绑定写入任务记录目录 `tag-grid-binding.json`，不猜测比赛绝对航向，`competition_heading_verified` 保持 false。文件 IO 在记录线程执行。

## 局部 Runtime 动态入口

Linux 完整构建后运行：

```sh
ctest --test-dir build-native -R local_task_one_closed_loop --output-on-failure
```

夹具以虚拟 UART 指令驱动位置和深度，双摄只呈现局部网格，覆盖建图、上浮、阶段重定位、四锥及退出；不连接真实串口。该入口目前仅完成 Python 语法检查，Windows 无完整 Linux Runtime，**尚未执行、不得记为通过**。原完整外框模拟仍保留为独立回归。

## 方案 B 训练与导出

`vision/training/train_cone_body.py` 接收 JSON `records`：每条包含 `image`、`mask`、`split`（train/val）、`group`、`reviewed:true`。输入应是与检测器一致的矫正网格图像；灰度 PNG 掩膜 0 为背景/底座、1 为圆锥主体、2 为方锥主体，不能用底座形状代替主体类别。不同划分必须隔离源录像及相同实例，脚本检查组重叠、重复图像及人工审核标记；不得把相邻帧拆进两个划分。

```sh
python vision/training/train_cone_body.py labels.json output --check-only
python vision/training/train_cone_body.py labels.json NEW_OUTPUT --device cuda --epochs 50
```

使用既有隔离 PyTorch 环境；脚本不安装依赖。提供主体内条纹扰动、分组验证 IoU、softmax ONNX 导出与源码/数据版本证据；模型报告保持 `shape_model_verified:false`。真实掩膜尚缺，因此没有训练真实权重，也未执行 PyTorch/ONNX 导出。新增清单门控测试 2/2 通过；C++ 合成 ONNX 推理通过只验证接口。
