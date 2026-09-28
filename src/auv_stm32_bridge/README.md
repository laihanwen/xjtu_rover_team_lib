# auv_stm32_bridge

ROS 2 与 STM32 之间的安全串口桥接边界。当前提供参数化 POSIX 串口传输、自动重连、P3 协议编解码、heartbeat、状态解析和 ARM 服务入口。

## 安全行为

- 节点始终以 DISARMED 状态启动。
- `serial_device` 默认为空，因此不会自动打开任何硬件。
- 即使串口成功打开，在收到 CRC 正确的 v1 协议帧前，`connected` 仍为 `false`。
- 串口打开后按参数化频率发送 HEARTBEAT；heartbeat 不包含运动目标。
- `/stm32/set_armed` 发送带 sequence 的显式请求；只有收到匹配 ACK 才返回成功。
- 本包不控制 PWM，也不包含推进器参数。

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
