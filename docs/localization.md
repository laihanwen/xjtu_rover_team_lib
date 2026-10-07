# 轻量二维定位首版

2026-10-07更新：原生Runtime已在Pi完成编译及8项CTest，并部署。ROV配置已启用定位诊断；桥接服务补齐RuntimeDirectory后，`/run/auv-rov/localization-input.json`已验证有新鲜IMU/深度输入。由于水下内参、安装关系和水面零点尚未标定，接口当前返回 `valid=false, reason=calibration_required`，不能生成可信米制轨迹。用户提供池深约0.44m，但来源为同一深度计读数，暂不作为已验证池深填入配置。需独立量池深、记录水面零点、完成下视水下标定后才可解除门限。下方早期未部署记录仅描述首版当时状态。

该版本用于 ROV 轨迹显示、记录和离线验证；AUV 轻量任务可读取相同接口。无 ROS 依赖，不发送运动指令。

## 算法与模块

`auv_mapping/plane_odometry.hpp` 是无 OpenCV、UART、HTTP 依赖的几何核心。它输入两帧归一化特征对应、姿态、深度、池深与安装关系，计算机器人中心在水平池底平面上的位移。机器人坐标 X 前、Y 左、Z 上，姿态采用右手系 roll/pitch/yaw，内部弧度。

runtime 的独立线程复用下视捕获帧，执行角点检测、双向 LK 跟踪、RANSAC 平面筛选和鲁棒位移估计。姿态用于补偿图像旋转，距底高度提供米制尺度。当前航向依赖 IMU，会累计漂移；不是完整 VIO/SLAM。前视 USB 不参与首版定位。既有标签、网格尚未参与绝对定位或回环修正。

ROV 模式通过遥控桥的独立写盘线程导出 `/run/auv-rov/localization-input.json`。文件为最新快照并原子替换；定位线程只读，不另开串口。AUV 模式中 runtime 已拥有串口时，直接传递已有状态缓存。同一树莓派单调时钟配对图像和遥测，遥测年龄不超过 0.3 秒、与图像差不超过 0.15 秒；这些是接收时间近似，不能等同硬件采样同步。

## 启用前填写参数

在使用的 runtime YAML 的 `localization` 段填写：

- 实际下视分辨率、相机内参、畸变；必须在实际防水舱与水下条件验证。
- `camera_to_body`：光学坐标到机器人坐标的 3×3 正交旋转矩阵，按行存储；不能照搬其他机器的矩阵。
- `camera_offset_m`、`depth_offset_m`：相机和深度计相对机器人中心的位置。
- `imu_signs`、`imu_offsets_deg`：实测三轴方向及水平安装偏置；符号仅允许 ±1。
- `pool_depth_m`：池底到水面的实际深度；`depth_zero_m`：传感器水面零点读数，算法使用原始读数减此值。
- 完成验证后设置 `calibration_verified: true` 和 `enabled: true`。

仓库默认关闭，标定值为空；不会用示例数值生成位置。首版要求平底、足够纹理、相机距底至少 0.15 m，俯仰和横滚不超过约 34°。定位默认最高 10 Hz，参数可选 1–20 Hz；帧间隔超过 0.3 秒失效。最多跟踪 200 个角点，筛选后需至少 20 个一致平面点。位移残差阈值 0.04 m、速度筛选 1 m/s，是首版算法默认门限而非硬件保证。

## 接口与驾驶台

- `GET /api/localization`：`frame_id=odom`，位置 `x_m/y_m`、`yaw_rad`、`stamp/age_s`、`session_id`、`valid/continuous/ready`、内点数、残差和失效原因。`uncertainty_m=null`，尚未估计统计位置误差。`input` 保存用于此帧的原始传感器输入。
- `POST /api/localization/reset`：未 ARM、图像和遥测有效、姿态/深度稳定且视觉平移速度小于 0.02 m/s 连续两秒后允许。将当前机器人中心设为原点、机头设为 X 正向，生成新会话。
- PC 驾驶台通过后台线程代理 `/localization`，不更新操作者控制许可。二维轨迹支持失效断线、会话重置和浏览器 JSONL 导出；坐标图上方为 X 前向、左侧为 Y 正向。

所有位置为相对位置，会累计漂移。跟踪失败后停止积分，连续性丢失后必须人工重新设原点；首版不尝试凭标签自动重定位。重新设原点会改变坐标，AUV 后续导航使用方必须消费会话变化并重新确认目标。`ready` 表示输入具备处理条件，原点设置仍可能因尚未静止两秒被拒绝。

Pi 现有旋转事件日志记录 `LOCALIZATION` 和 `LOCALIZATION_RESET`，`detail` 是 JSON 字符串；需解析两次得到结果和输入。PC 五秒日志也保存定位快照。浏览器曲线/导出只是会话缓存，刷新页面会丢失，长期记录以后台文件为准。

## 编译、回放与验证

在树莓派现有原生构建环境执行（无需 ROS）：

```sh
cmake -S . -B build/native -DCMAKE_BUILD_TYPE=Release
cmake --build build/native -j2
ctest --test-dir build/native -R plane_odometry --output-on-failure
```

新增 OpenCV `video` 组件用于 LK 跟踪；复用现有 OpenCV、yaml-cpp 依赖。维护部署脚本会带上新增源码和桥文件；ROV 桥 drop-in 增加 RuntimeDirectory，手动安装时也需复制该配置并 reload systemd。不得同时让 ROV 桥与 AUV runtime 占用串口。

离线回放使用下视 AVI 和一帧一行的 JSONL 输入：包含 `frame_stamp`（图像时间），以及 `stamp`（遥测接收时间）、`roll_deg/pitch_deg/yaw_deg/depth_m/armed/valid`。可从同一次 Pi 开机的定位事件和录制 sidecar 生成：

```sh
python tools/rov/prepare_localization_replay.py down_0001.frames.jsonl events.ndjson inputs.jsonl
build/native/runtime/auv_localization_replay runtime/config/pi-rov.yaml down_0001.avi inputs.jsonl results.jsonl
```

回放程序使用相同算法，在静止条件满足后自动设置原点，仅影响离线结果。视频和遥测行数必须一致；没有配对传感器的帧会标记无效，不以零值补齐。事件日志须来自该视频所属 Pi 开机，不能混合不同开机的单调时间戳。源码默认未标定配置会退出并报告未初始化，这是预期行为。

本地已运行几何测试：前向位移、原地偏航、升降、带安装偏移的倾斜补偿、点数不足、位移不一致、时间异常及标定拒绝；桥导出与回放配对测试通过。定位适配器和回放程序通过 C++ 语法检查。Windows 缺少完整 Linux runtime 编译环境，树莓派 SSH 超时，因此尚未完成整套 native 链接、真实视频回放或部署。实际更新率、录制并行开销和水下精度均待实测。
