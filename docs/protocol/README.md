# 通信协议与接口定义

本目录聚合 ROS / Pi / STM32 之间的串口与消息接口定义，适合在联调、故障排查和协议迭代时查阅。

## 目录内容

- [serial-protocol.md](./serial-protocol.md) — Pi ↔ STM32 串口协议 v1

## 重要约束

- 下行命令优先使用稳定的目标值，如 `vx`、`vy`、`depth_target`、`yaw_target`。
- STM32 侧保留实时闭环控制、PWM 混控与 failsafe。
- 所有传输均采用明确长度和 CRC 校验，避免不受控的裸字节通信。
- 硬件鉴权与故障状态不得被忽略，尤其是 heartbeat、leak、ARM 条件和 output saturation。

## 适用场景

- 诊断串口解码是否一致
- 检查 STM32 端状态上报是否符合协议
- 验证 ROS 节点和底层控制之间的接口边界
