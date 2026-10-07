# Raspberry Pi ↔ STM32 串口协议 v1

2026-10-06 新增 `0x84 PID_DIAGNOSTIC`（初版49字节，定航向/定深候选版61字节，随STATUS以10Hz回传），用于PC每5秒日志。包含MCU控制周期时间、ARM/IMU/闭环标志、三轴角度/误差/混控前纠正量及俯仰/偏航角速度。完整定义见 [ROV日志说明](../../tools/rov/LOGGING.md) 与 [闭环扩展](../../tools/rov/HOLD_MODES.md)。原STATUS布局不变，旧端可忽略新诊断帧。

状态：v1 已冻结，heartbeat、ARM、MOTION_TARGET 和状态遥测均已接入。
MCU 为 STM32F405RGT6；实机接线和台架安全测试仍需人工确认。

## 传输约定

- UART 默认 `115200 8N1`、无硬件流控；波特率保持可配置。
- 多字节整数和 IEEE-754 binary32 浮点数均为小端序。
- `bool` 编码为一个 `uint8`，只允许 0 或 1。
- 不直接传输 C struct，避免 padding 和 ABI 差异。
- 最大 payload 64 bytes，最大完整帧 71 bytes。
- Pi 默认以 20 Hz 发送 heartbeat；允许配置为 20–50 Hz。
- STM32 heartbeat 超时目标为 500 ms，failsafe 在 P4 实现。

## 帧与 CRC

| Offset | 大小 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | sync 0 | `0xAA` |
| 1 | 1 | sync 1 | `0x55` |
| 2 | 1 | version | `0x01` |
| 3 | 1 | message type | 见消息表 |
| 4 | 1 | payload length | `0..64` |
| 5 | N | payload | 消息字段 |
| 5+N | 2 | CRC16 | 小端，低字节在前 |

CRC 使用 **CRC-16/CCITT-FALSE**：poly `0x1021`、init `0xFFFF`、refin/refout false、xorout `0x0000`。覆盖 `version`、`message type`、`payload length` 和 payload，不含 sync 与 CRC。ASCII `123456789` 的检查值为 `0x29B1`。

## 消息 ID

| ID | 名称 | 方向 | Payload bytes |
|---:|---|---|---:|
| `0x01` | HEARTBEAT | Pi → STM32 | 8 |
| `0x02` | SET_ARMED | Pi → STM32 | 5 |
| `0x03` | MOTION_TARGET | Pi → STM32 | 20 |
| `0x04` | ACTUATOR_COMMAND | Pi → STM32 | 9 |
| `0x05` | RC_TARGET | Pi → STM32 | 16 |
| `0x06` | REMOTE_KILL | Pi → STM32 | 1 |
| `0x07` | Commissioning pulse（默认禁用） | Pi → STM32 | 7 |
| `0x08` | CALIBRATE_LEVEL | Pi → STM32 | 5 |
| `0x7F` | ACK | STM32 → Pi | 6 |
| `0x80` | STATUS | STM32 → Pi | 30 + 2N |
| `0x81` | IMU | STM32 → Pi | 40 |
| `0x82` | DEPTH | STM32 → Pi | 9 |
| `0x83` | ACTUATOR_STATUS | STM32 → Pi | 16 |

未知 version、未知 ID、超长 payload、长度不完整或 CRC 错误的帧必须丢弃，不得更新控制目标。解析器从下一个 `AA 55` 重新同步。

## Payload 定义

Offset 均相对 payload 起点。

### HEARTBEAT `0x01`

| Offset | 类型 | 字段 |
|---:|---|---|
| 0 | `uint32` | sequence |
| 4 | `uint32` | Pi monotonic uptime，ms，允许自然回绕 |

### SET_ARMED `0x02`

| Offset | 类型 | 字段 |
|---:|---|---|
| 0 | `uint32` | sequence |
| 4 | `uint8` | 0 DISARM，1 ARM |

ARM 必须显式请求；运动指令不能隐式 ARM。STM32 必须用 ACK 报告结果，主机不能把串口写成功视为 ARM 成功。

### MOTION_TARGET `0x03`

| Offset | 类型 | 单位 | 字段 |
|---:|---|---|---|
| 0 | `uint32` | — | sequence |
| 4 | `float32` | m/s | `vx`，+X 向前 |
| 8 | `float32` | m/s | `vy`，+Y 向左 |
| 12 | `float32` | m | depth target，深度向下为正 |
| 16 | `float32` | rad | yaw target，右手系 |

值必须有限；STM32 必须再次限幅并拒绝 NaN/Inf。

当前平移控制在尚无 DVL 的条件下使用可标定的前馈增益，不能宣称为真实速度闭环；
`yaw target` 使用 H30 IMU 闭环。深度目标会被完整传输和校验，但在真实深度驱动接入前
不会产生垂向推力。DISARM 时合法目标只会被缓存且 ACK 返回 disarmed，绝不会产生输出；
显式 ARM 时锁定当前新鲜控制源。锁定源 250 ms 未收到新目标即撤销 ARM。

### ACTUATOR_COMMAND `0x04`

| Offset | 类型 | 字段 |
|---:|---|---|
| 0 | `uint32` | sequence |
| 4 | `uint8` | actuator ID；`1` 为 T35-L 单舵机夹爪 |
| 5 | `float32` | `-1.0` 闭合，`0.0` 停止并保持，`+1.0` 张开 |

夹爪 OPEN/CLOSE 仅在 ARMED 且标定门开启时接受；STOP 允许在 DISARMED 时执行。
Pi OPEN/CLOSE 还要求本次 ARM 周期已经锁定 Pi 控制源；STM32 拒绝重复或过期
sequence。业务层不得使用中间浮点值直接控制 PWM。

### ACK `0x7F`

| Offset | 类型 | 字段 |
|---:|---|---|
| 0 | `uint8` | 被确认的 message type |
| 1 | `uint8` | result：0 accepted，1 malformed，2 disarmed，3 unsafe，4 unsupported |
| 2 | `uint32` | 被确认请求的 sequence |

### STATUS `0x80`

| Offset | 类型 | 单位 | 字段 |
|---:|---|---|---|
| 0 | `uint32` | — | sequence |
| 4 | `uint8` | bitset | bit0 armed，bit1保留未使用，bit2 failsafe，bit3岸上水平已校准 |
| 5 | `uint32` | bitset | firmware error flags，见下表 |
| 9 | `float32` | V | battery voltage |
| 13 | `float32` | m | depth |
| 17/21/25 | `float32` | rad | roll / pitch / yaw |
| 29 | `uint8` | — | thruster count N，`0..8` |
| 30 | `int16[N]` | 0.001 | normalized output，`-1000..1000` |

长度必须严格等于 `30 + 2N`。

Firmware error flags：

| 位 | 含义 |
|---:|---|
| 0 | heartbeat timeout |
| 1 | required sensor invalid |
| 2 | 保留未使用（当前无漏水传感器） |
| 3 | hardware kill active |

### IMU `0x81`

| Offset | 类型 | 单位 | 字段 |
|---:|---|---|---|
| 0 | `uint32` | — | sequence |
| 4/8/12 | `float32` | rad | roll / pitch / yaw |
| 16/20/24 | `float32` | rad/s | gyro X/Y/Z |
| 28/32/36 | `float32` | m/s² | acceleration X/Y/Z |

当前 H30 接入仅提供欧拉角；未提供的角速度和线加速度字段必须发送 quiet NaN，bridge 同时把对应 covariance 首项设为 `-1`，不得用零伪装测量值。

### DEPTH `0x82`

| Offset | 类型 | 单位 | 字段 |
|---:|---|---|---|
| 0 | `uint32` | — | sequence |
| 4 | `float32` | m | depth |
| 8 | `uint8` | — | valid：0/1 |

深度传感器未接入或读数失效时发送 `valid=0` 和 quiet NaN；只有 `valid=1` 且深度为有限值时才是可用测量。

### ACTUATOR_STATUS `0x83`

| Offset | 类型 | 字段 |
|---:|---|---|
| 0 | `uint32` | telemetry sequence |
| 4 | `uint8` | actuator ID；夹爪为 `1` |
| 5 | `uint8` | 状态：0 未标定、1 停止、2 正在闭合、3 已闭合、4 正在张开、5 已张开、6 故障 |
| 6 | `uint8` | bit0：已标定 |
| 7 | `uint16` | 当前 PWM 脉宽，μs |
| 9 | `uint16` | 目标 PWM 脉宽，μs |
| 11 | `uint8` | bit0 未标定、bit1 配置错误、bit2 sequence 错误 |
| 12 | `uint32` | 最近一次已解析命令的 sequence |

状态中的脉宽仅用于诊断。ROS 任务层仍只使用 OPEN/CLOSE/STOP。

## 黄金测试向量

以下十六进制 bytes 包含完整帧，CRC 低字节在前。

```text
# HEARTBEAT sequence=0x01020304, uptime_ms=0x0A0B0C0D
AA 55 01 01 08 04 03 02 01 0D 0C 0B 0A 55 8D

# SET_ARMED sequence=0x01020304, armed=1
AA 55 01 02 05 04 03 02 01 01 A5 0A

# MOTION_TARGET sequence=0x12345678, vx=1, vy=-0.5, depth=2.5, yaw=0.5
AA 55 01 03 14 78 56 34 12 00 00 80 3F 00 00 00 BF 00 00 20 40 00 00 00 3F C1 01

# ACTUATOR_COMMAND sequence=0x01020304, gripper ID=1, OPEN=+1.0
AA 55 01 04 09 04 03 02 01 01 00 00 80 3F 69 F4
```

STM32 P4 接入时必须逐 byte 复用这些权威向量。

## 安全与时序

- 上电和重新连接保持 DISARM。
- sequence 单调递增并允许 `uint32` 回绕；STM32 应拒绝明显过期的控制命令。
- heartbeat 超时后进入 failsafe，并要求新的显式 ARM。
- 漏水、无效关键传感器或 kill 状态下拒绝 ARM。
- 串口断开或合法状态帧超时后，bridge 发布 `connected=false`、`armed=false`。
- heartbeat 不携带运动目标，不能改变 ARM 状态。
- `/cmd_vel`、`/cmd_depth`、`/cmd_yaw` 任一输入超过 250 ms 未更新时，Pi bridge
  停止发送 MOTION_TARGET；STM32 的独立 250 ms 超时随后撤销 ARM。
- ARM 时若 Pi 和兼容遥控输入都新鲜则锁定 Pi；锁定源超时后不会自动切换来源，
  必须重新显式 ARM。


### CALIBRATE_LEVEL `0x08`

Payload offset0为uint32 sequence，offset4为uint8显式岸上确认（必须1）。仅DISARM、250ms内有效IMU且序号比上次更新时可接受。将最新原始俯仰/横滚写入RAM水平参考，不改变相对偏航原点，不ARM，不写Flash。首次请求允许任意sequence，后续按int32差值判断更新（支持uint32回绕）。返回标准ACK（type0x08，result0成功，其余拒绝，回显sequence）。成功后STATUS bit3置位，STM32重启清除。主机应校验匹配ACK和新STATUS，不根据按钮点击直接显示校准成功。

主机需要人工确认岸上放平，未ARM、回中并静止采样2秒；CRC或协议本身无法证明设备物理位置在岸上。未校准会被固件ARM安全门拒绝。其它消息布局、CRC与八路输出顺序保持不变。

### 2026-10-07 PID diagnostic state extension

Type 0x84 now uses 64 payload bytes; the first 61 bytes remain unchanged. Byte 61 is heading state, byte 62 depth state: 0 disabled, 1 disarmed, 2 RC stale, 3 sensor invalid/stale, 4 manual override, 5 locked. Byte 63 is the priority mixer's motion scale quantized to 0..255 (divide by 255). This measures reduction during priority combination only, not earlier motion normalization, subsequent ESC deadband compensation, or measured thrust. PC decoder accepts 49/61/64-byte variants; old firmware has no authoritative state reason.
