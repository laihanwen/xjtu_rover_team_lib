# Raspberry Pi ↔ STM32 串口协议

状态：草案。固件 MCU 已确认为 STM32F405RGT6；字节序、CRC16 变体、消息 ID 和控制周期仍需在 ROS bridge 接入前确认。

## 帧结构

```text
0xAA 0x55 | protocol_version | message_type | payload_length | payload | CRC16
```

多字节字段的字节序、CRC16 变体和最大 payload 长度尚未由硬件验证，不应在实现前猜测。确定后需要在本文档给出黄金测试向量，并在 ROS 与 STM32 两侧运行相同的编码/解码测试。

## 消息方向

Pi → STM32：

- heartbeat
- arm / disarm
- velocity target (`vx`, `vy`)
- depth target
- yaw target
- actuator command

STM32 → Pi：

- attitude (`roll`, `pitch`, `yaw`)
- depth
- battery voltage
- leak state
- error flags
- thruster outputs

## 安全约束

- 上电和通信建立时保持 DISARM。
- Pi 以 20–50 Hz 发送心跳。
- STM32 超过配置的心跳超时后进入 failsafe，默认目标为 500 ms。
- ARM 必须是显式命令，不能由普通速度指令隐式触发。
- 无效长度、CRC 错误、未知协议版本和过期命令必须拒绝。
