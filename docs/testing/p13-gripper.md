# P13 T35-L 单舵机夹爪标定与验收

## 已确认配置

| 项目 | 配置 |
|---|---|
| 舵机 | T35-L，180°位置舵机 |
| 功能 | 单自由度开合 |
| 信号 | PA8 / TIM1_CH1 |
| PWM | 50 Hz，0.5 μs/count |
| 方向 | 脉宽增大张开，脉宽减小闭合 |
| 上电目标 | 标定后缓慢闭合 |
| 遥控 | SB=1 启用；SA 低位闭合、高位张开、中间停止 |
| ROS | `/gripper/set_state`、`/gripper/status` |

T35-L 必须由独立大电流电源供电并与 STM32 共地，禁止使用控制板 5 V 引脚供电。
厂家给出的最大堵转电流约 5.7 A；电源、线束和连接器均应留出瞬态余量。

## 1. 安全准备

- 断开八个推进器的动力电源或拆桨并固定整机。
- 先断开舵机连杆，确认 PWM 方向，再连接机械机构。
- 手、线缆和工具远离夹爪运动范围。
- 准备可立即断开的舵机独立电源。
- 初次标定从较低工作电压开始。
- 监测电源电流和 STM32 是否复位。

## 2. 未标定固件检查

保持 `firmware/stm32/MDK-ARM/AuvGripperConfig.h` 中：

```c
#define AUV_GRIPPER_CALIBRATED 0U
```

烧录后 PA8 应输出约 1500 μs，OPEN/CLOSE 必须返回 unsafe：

```fish
ros2 topic echo /gripper/status
```

预期：

```text
connected: true
calibrated: false
state: 0
current_pulse_us: 1500
target_pulse_us: 1500
```

## 3. 测量闭合与张开端点

不要直接把脉宽从 500 μs 扫到 2500 μs。使用临时调试构建，从 1500 μs 开始，
每次只变化 20–25 μs，并观察机构和电流：

1. 逐步减小脉宽，找到刚好完全闭合但没有顶住机械硬限位的位置。
2. 向张开方向回退少量余量，记录为 `AUV_GRIPPER_CLOSE_US`。
3. 从 1500 μs 逐步增大，找到安全完全张开位置。
4. 向闭合方向回退少量余量，记录为 `AUV_GRIPPER_OPEN_US`。
5. 确认始终满足 `MIN <= CLOSE < OPEN <= MAX`。
6. 连续往返五次，确认没有撞限位、卡滞和明显电流异常。

将实测值写入 `AuvGripperConfig.h`，再把标定门改为 `1U`。不得只修改标定门而保留
未经测量的示例端点。

## 4. 软件验证

重新编译并运行：

```fish
cmake -S firmware/stm32/test -B firmware/stm32/build/host-tests
cmake --build firmware/stm32/build/host-tests
ctest --test-dir firmware/stm32/build/host-tests --output-on-failure

source /opt/ros/lyrical/setup.fish
colcon build --symlink-install --packages-select auv_interfaces auv_stm32_bridge
source install/setup.fish
```

## 5. 上电闭合测试

首次连接机构时保持夹爪内无物体。上电后舵机应从中位以限速方式移动到闭合位置，
不能突然跳转。确认：

- `current_pulse_us` 逐步接近 `target_pulse_us`；
- 最终状态为 `STATE_CLOSED=3`；
- 闭合后无机械顶死、持续啸叫或异常电流；
- 舵机动作不会导致 STM32、树莓派或传感器复位。

## 6. 遥控验收

在推进器断电条件下显式 ARM 遥控控制源：

- SB=1、SA 低位：夹爪缓慢闭合；
- SB=1、SA 高位：夹爪缓慢张开；
- SB=1、SA 中间：停止并保持当前位置；
- SB≠1：不改变夹爪目标；
- 遥控掉线：推进器 DISARM，夹爪保持最后目标，不继续接受新动作。

## 7. ROS 验收

确认串口连接、安全输入和 ARM 状态后执行：

ROS 控制周期必须先以 20 Hz 持续发布 `/cmd_vel`、`/cmd_depth`、`/cmd_yaw` 的安全零
目标，再显式 ARM，使本次 ARM 周期锁定为 Pi 控制源。若当前锁定的是遥控器，ROS
OPEN/CLOSE 会被拒绝，不能在运行中抢占。

```fish
ros2 service call /gripper/set_state auv_interfaces/srv/SetGripper '{action: 2}'
ros2 topic echo /gripper/status
ros2 service call /gripper/set_state auv_interfaces/srv/SetGripper '{action: 1}'
ros2 service call /gripper/set_state auv_interfaces/srv/SetGripper '{action: 0}'
```

验收要求：

- 每个命令只有收到匹配 sequence 的 STM32 ACK 才返回成功；
- 重复或旧 sequence 被拒绝；
- DISARM 时 OPEN/CLOSE 被拒绝，STOP 可执行；
- 标定门关闭时 OPEN/CLOSE 被拒绝；
- `/gripper/status` 与实际动作方向一致；
- 串口中断或节点停止后无失控动作。

## 8. 抓取验证

软件“已闭合”只代表 PWM 到达标定目标，不等于已经抓住海参。当前没有限位、位置或
电流反馈，因此正式任务必须结合视觉复核或后续增加实体传感器。依次完成：

1. 空载开合 100 次。
2. 干态模型夹取 20 次。
3. 固定水槽夹取 20 次。
4. AUV 固定、推进器断电情况下视觉触发夹取。
5. 低速自主接近与夹取。

任何阶段出现卡滞、掉电复位、过热或持续堵转，都应停止并重新检查机械端点和供电。
