# 任务一：AprilTag 与交通锥遍历验收

本流程只覆盖任务一：识别 AprilTag、建立九宫格语义地图、识别四个交通锥、规划路线并
逐格访问。第一次验收必须按“视觉 → dry-run → 拆桨运动 → 固定水槽”顺序推进。

## 实机前配置

1. 在 `cameras.yaml` 填写稳定的下视相机 `/dev/v4l/by-id/...` 路径。
2. 在水下完成相机标定，填写 `down_camera_calibration.yaml` 并设置
   `calibration_configured: true`。
3. 在 `planning.yaml` 填写真实起始格，不能保留 `-1/-1`。
4. 用水下录像调整 `mapping.yaml` 和 `cones.yaml`；默认要求稳定识别四个锥。
5. 保持 `control.yaml` 的 `publish_motion_commands: false`。

## 第一阶段：视觉与规划 dry-run

```fish
source /opt/ros/lyrical/setup.fish
source /home/hanwen/auv/install/setup.fish
ros2 launch auv_bringup system.launch.py \
  start_stm32_bridge:=true \
  start_cameras:=true start_apriltag:=true \
  start_mapping:=true start_cones:=true start_planning:=true \
  start_mission:=true start_route_executor:=true
```

执行器默认不会发布运动命令。检查：

```fish
ros2 topic echo /apriltag/detections
ros2 topic echo /semantic_map
ros2 topic echo /mapping/grid_pose
ros2 topic echo /planning/route
ros2 topic echo /planning/execution_state
```

验收要求：

- 指定 AprilTag 连续可见且没有明显误检；
- `/semantic_map.complete=true` 前已经稳定检测到四个锥；
- 四个锥的 row/col 和圆/方分类正确；
- `/mapping/grid_pose` 连续、有效，移动相机时方向符合实际；
- `/planning/route` 有效、相邻格连续且访问所有目标；
- `/planning/execution_state` 带 `dry-run:` 前缀；
- dry-run 期间不存在 route executor 的 `/cmd_vel` 发布者。

## 第二阶段：拆桨方向与故障测试

断开推进器动力或拆除全部桨叶，固定整机并准备物理断电。先用
`ros2 topic info /cmd_vel -v` 确认只有预期执行器发布运动目标。然后显式启动：

```fish
ros2 launch auv_bringup system.launch.py \
  start_stm32_bridge:=true \
  start_cameras:=true start_apriltag:=true \
  start_mapping:=true start_cones:=true start_planning:=true \
  start_mission:=true start_route_executor:=true \
  enable_route_motion:=true
```

Mission 到达 `VISIT_CONES` 后仍需人工显式 ARM。逐项确认：

1. row/col 误差分别只产生预期 surge/sway 方向；方向错误时只修改 `control.yaml` 的
   四个映射系数，不能交换电机线来掩盖软件坐标错误。
2. 速度绝对值不超过 `maximum_speed`。
3. 深度与航向目标在开始运动时锁定，不随遥测漂移。
4. 遮住九宫格或停止相机后，0.5 秒内输出归零并请求 DISARM。
5. 停止 STM32 状态、注入 leak/error、拔掉网线时均归零并 DISARM。
6. 到达目标中心需连续满足容差，随后发布 `/planning/visited_cell`。
7. 四个目标完成后 Mission 进入 `COMPLETE` 并请求 DISARM。

## 第三阶段：固定水槽与低速系留

只有拆桨测试全部记录通过后才进入固定水槽。先将 `maximum_speed` 降到现场负责人批准的
最小值，验证单轴方向、定位稳定性、失联停止和四目标 visited 顺序。固定水槽通过后，
才允许在安全系留下进行低速自由遍历。

建议记录以下 topics：

```fish
ros2 bag record /camera/down/image_raw /apriltag/detections /cones/detections \
  /semantic_map /mapping/grid_pose /planning/route /planning/execution_state \
  /planning/visited_cell /mission/state /cmd_vel /cmd_depth /cmd_yaw /stm32/status
```
