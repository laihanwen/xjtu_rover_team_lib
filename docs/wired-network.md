# PC 与树莓派有线联调

这条链路同时承载 ROS 2 视频/检测结果回传和运动目标。STM32 仍只通过树莓派的
UART 接入；PC 不直接产生 PWM，且网络命令不会自动 ARM。

## 1. 网络与 ROS 2 环境

建议给专用网口设置同一子网的静态地址，例如：

| 设备 | IPv4 |
|---|---|
| PC | `192.168.50.1/24` |
| 树莓派 | `192.168.50.2/24` |

接口名以 `ip -br link` 的实际输出为准。两端先互相 `ping`，再确保 ROS 2 Lyrical、
工作区版本和以下环境变量一致：

```fish
source /opt/ros/lyrical/setup.fish
source ~/auv/install/setup.fish
set -gx ROS_DOMAIN_ID 42
set -gx ROS_LOCALHOST_ONLY 0
```

如果树莓派工作区位于 `~/auv_ws`，第二条按实际路径改为
`source ~/auv_ws/install/setup.fish`。不要在两端设置不同的 `ROS_DOMAIN_ID`。

先在树莓派执行 `ros2 multicast receive`，再在 PC 执行 `ros2 multicast send`。
若收不到，检查两端防火墙和有线网口的组播策略；SSH 能连通不代表 DDS 发现一定正常。

## 2. 树莓派启动

先把 `src/legacy/ros2/auv_bringup/config/cameras.yaml` 的两个 `source` 改成实际稳定的
`/dev/v4l/by-id/...`，把 `stm32_bridge.yaml` 的 `serial_device` 改成实际 UART 设备。
随后构建并启动：

```fish
cd ~/auv
source /opt/ros/lyrical/setup.fish
colcon build --symlink-install
source install/setup.fish
set -gx ROS_DOMAIN_ID 42
set -gx ROS_LOCALHOST_ONLY 0
ros2 launch auv_bringup system.launch.py \
  start_stm32_bridge:=true start_cameras:=true start_apriltag:=true
```

启动不会自动 ARM。串口、相机路径仍为空时，对应节点会保持安全失败状态。

## 3. PC 接收视频与状态

PC 加载同版本工作区和相同 ROS 环境后：

```fish
ros2 node list
ros2 topic hz /camera/down/image_raw
ros2 topic hz /camera/front/image_raw
ros2 topic echo /stm32/status
rqt_image_view /camera/down/image_raw
```

也可在 `rqt_image_view` 中选择 `/apriltag/debug_image`，但需要先把
`apriltag.yaml` 的 `publish_debug_image` 设为 `true`。原始 640×480 BGR、30 fps 单路
理论数据量约 221 Mbit/s（未计 DDS 开销），双路联调应使用千兆以太网；实际丢帧时先
降低帧率/分辨率，再引入 `image_transport` 压缩，避免在控制联调时同时改变编码链路。

## 4. 拆桨状态验证运动目标

首次验证必须断开推进器动力或拆桨。PC 用三个终端以 20 Hz 连续发布完整目标：

```fish
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

```fish
ros2 topic pub -r 20 /cmd_depth std_msgs/msg/Float32 '{data: 0.0}'
```

```fish
ros2 topic pub -r 20 /cmd_yaw std_msgs/msg/Float32 '{data: 0.0}'
```

确认 `/stm32/status.connected=true`、无漏水/急停/传感器错误后，才显式 ARM：

```fish
ros2 service call /stm32/set_armed auv_interfaces/srv/SetArmed '{armed: true}'
ros2 topic echo /stm32/status
```

验收顺序：零目标时 `thruster_outputs` 全零；短时发送很小的 `linear.x` 后八路输出符合
分配矩阵；停止任意一个命令 topic，250 ms 左右应自动 DISARM 且输出归零。当前深度
传感器驱动尚未接入，`/cmd_depth` 只参加完整性校验，不会产生垂向推力。

## 5. 断网验收

保持拆桨，ARM 后拔掉 PC—树莓派网线。PC 命令停止到达后，bridge 不再生成完整新鲜
运动目标，STM32 应在 250 ms 左右 DISARM。重新插线只恢复 ROS 通信，不会自动 ARM。
