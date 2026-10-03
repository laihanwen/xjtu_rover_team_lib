# auv_stm32_bridge

ROS 2 与 STM32 之间的安全串口桥接边界。当前提供参数化 POSIX 串口传输、自动重连、P3 协议编解码、heartbeat、状态解析、ARM 服务及 P13 单舵机夹爪接口。

## 安全行为

- 节点始终以 DISARMED 状态启动。
- `serial_device` 默认为空，因此不会自动打开任何硬件。
- 即使串口成功打开，在收到 CRC 正确的 v1 协议帧前，`connected` 仍为 `false`。
- 串口打开后按参数化频率发送 HEARTBEAT；heartbeat 不包含运动目标。
- `/stm32/set_armed` 发送带 sequence 的显式请求；只有收到匹配 ACK 才返回成功。
- 本包不控制 PWM，也不包含推进器参数。
- `/gripper/set_state` 只发送 OPEN/CLOSE/STOP；PWM 标定、限幅和缓启动留在 STM32。

当前 `error_flags` 的桥接端临时位定义：

| 位 | 含义 |
|---|---|
| bit 0 | 串口 transport 不可用或未配置 |
| bit 1 | 串口已打开，但合法协议帧已超时 |

STM32 上报的正式 error flags 将在 P4 固件安全接口中冻结。

## 构建和运行

```fish
source /opt/ros/lyrical/setup.fish
colcon build --symlink-install --packages-up-to auv_stm32_bridge
source install/setup.fish

# 默认不打开串口，只发布安全状态
ros2 run auv_stm32_bridge stm32_bridge_node

# 仅在确认实际设备路径后配置；只会自动发送无运动目标的 heartbeat
ros2 run auv_stm32_bridge stm32_bridge_node --ros-args \
  -p serial_device:=/dev/ttyUSB0 -p baud_rate:=115200
```

通过 bringup 启动时必须显式开启：

```fish
ros2 launch auv_bringup system.launch.py start_stm32_bridge:=true
```

## PC 经有线网络发送运动目标

PC 和树莓派必须使用相同 ROS 2 Lyrical、`ROS_DOMAIN_ID` 和 `auv_interfaces`。
bridge 同时要求以下三个 topic 在 250 ms 内持续更新：

- `/cmd_vel` (`geometry_msgs/msg/Twist`)：只使用 `linear.x` 和 `linear.y`，单位 m/s；
- `/cmd_depth` (`std_msgs/msg/Float32`)：目标深度，向下为正，单位 m；
- `/cmd_yaw` (`std_msgs/msg/Float32`)：目标航向，范围 `[-pi, pi]`，单位 rad。

任一 topic 停止更新后，bridge 停止下发运动帧，STM32 也会在 250 ms 后撤销 ARM。
命令不会隐式 ARM。首次联调必须断开电机电源或拆桨，并以 20 Hz 连续发布三项：

```fish
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
ros2 topic pub -r 20 /cmd_depth std_msgs/msg/Float32 '{data: 0.0}'
ros2 topic pub -r 20 /cmd_yaw std_msgs/msg/Float32 '{data: 0.0}'
```

确认 `/stm32/status` 已连接且所有硬件安全输入有效后，才可显式请求 ARM：

```fish
ros2 service call /stm32/set_armed auv_interfaces/srv/SetArmed '{armed: true}'
ros2 topic echo /stm32/status
```

`thruster_outputs` 返回经过安全门控后的 T1–T8 归一化输出；DISARM 时八路均应为零。
当前 `vx/vy` 是没有 DVL 时的开环前馈，最终艇体必须在约束水池内重新标定
`AUV_SURGE_PWM_PER_MPS` 和 `AUV_SWAY_PWM_PER_MPS`。深度驱动尚未接入，因此当前版本不会
根据 `/cmd_depth` 产生垂向推力。

## P13 单舵机夹爪

接口：

```text
/gripper/set_state  auv_interfaces/srv/SetGripper
/gripper/status     auv_interfaces/msg/GripperStatus
```

动作编号：`0=STOP`、`1=CLOSE`、`2=OPEN`。OPEN/CLOSE 要求 STM32 已 ARM 且
`AUV_GRIPPER_CALIBRATED=1`；未标定固件会返回 unsafe，PA8 保持 1500 μs。

拆桨、断开推进器动力并完成舵机端点标定后才可测试：

```fish
ros2 topic echo /gripper/status
ros2 service call /gripper/set_state auv_interfaces/srv/SetGripper '{action: 2}'
ros2 service call /gripper/set_state auv_interfaces/srv/SetGripper '{action: 1}'
ros2 service call /gripper/set_state auv_interfaces/srv/SetGripper '{action: 0}'
```

详细标定与上电验收见
[`docs/testing/p13-gripper.md`](../../docs/testing/p13-gripper.md)。

## 验证

```fish
colcon test --packages-select auv_stm32_bridge
colcon test-result --verbose
ros2 topic echo /stm32/status
ros2 service call /stm32/set_armed auv_interfaces/srv/SetArmed "{armed: true}"
```

ARM 请求在 300 ms 内未收到匹配 ACK 时返回失败。串口写入成功不代表 STM32 已 ARM。

串口单元测试使用 PTY 模拟器验证 8N1 配置和双向原始字节传输，不需要连接 STM32。

协议定义、payload 表和黄金向量见 [`docs/protocol/serial-protocol.md`](../../docs/protocol/serial-protocol.md)。
