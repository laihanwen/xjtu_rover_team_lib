# STM32F405 八推全矢量固件

该目录保存已经完成的八推全矢量 ROV 固件基线，并作为后续 AUV 底层控制开发入口。

## 已确认基线

| 项目 | 当前值 |
|---|---|
| MCU | STM32F405RGT6，LQFP64 |
| CubeMX 工程 | `Copy_cup.ioc` |
| 部署工程 | Keil MDK-ARM 5，`MDK-ARM/Copy_cup.uvprojx` |
| 控制源码 | `MDK-ARM/Move.c`、`Mate.c`、`PID.c`、`RC.c`、`imu.c` |
| 推进器 | 8 个全矢量推进器 |
| 坐标系 | `+X` 前、`+Y` 左、`+Z` 上，重心为原点 |
| PWM 映射 | T1–T8 映射记录在 `Mate.c` 与 `Move_Manual.md` |

详细的安装坐标、推力方向、混控矩阵、推进器极性及遥控映射见 [Move_Manual.md](MDK-ARM/Move_Manual.md)。原工程的开发说明保存在 [DEVELOPMENT.md](DEVELOPMENT.md)。

> [!IMPORTANT]
> P4 已加入显式 ARM/DISARM、Pi heartbeat timeout、故障状态机和八推进器 PWM 门控。软件测试通过不等于实机安全验证完成；漏水、kill 和传感器有效信号仍需在 P5/硬件接线阶段调用 `AuvLink_SetSafetyInputs()` 接入。

## 哪些文件是有效入口

```text
firmware/stm32/
├── Copy_cup.ioc             # CubeMX 硬件配置源
├── Core/                    # CubeMX 生成代码
├── Drivers/                 # 构建实际需要的 HAL 与 CMSIS 子集
├── MDK-ARM/
│   ├── Copy_cup.uvprojx     # 权威部署工程
│   ├── Move.c               # 六维指令与八推分配
│   ├── Mate.c               # PID、动态去饱和与 PWM 输出
│   ├── PID.c                # 位置式 PID
│   ├── RC.c                 # 遥控数据映射
│   ├── imu.c                # IMU 接收和解析
│   ├── AuvProtocol.c/.h     # 固定缓冲区 P3 协议与 CRC
│   ├── AuvSafety.c/.h       # ARM/DISARM/heartbeat/failsafe 状态机
│   ├── AuvLink.c/.h         # USART、ACK 和 STATUS 集成
│   ├── AuvRcInput.c/.h      # 0xA5 遥控帧接收、快照与掉线超时
│   ├── AuvGripper.c/.h      # P13 T35-L 单舵机夹爪状态机
│   ├── AuvGripperConfig.h   # 标定门、端点、缓启动和遥控阈值
│   ├── AuvCameraServo.c/.h  # 摄像头舵机限位、斜率限制与命令校验
│   ├── AuvCameraServoConfig.h # 摄像头舵机标定门与 CCR 端点
│   ├── AuvDepth.c/.h        # 深度样本校验、缓存与 250 ms 新鲜度门
│   └── Move_Manual.md       # 安装和控制说明
├── CMakeLists.txt           # 编辑器代码模型 / 编译检查
├── COLCON_IGNORE            # 防止 ROS colcon 误构建固件
└── cmake/                   # ARMClang 与 GCC 工具链文件
```

`Core/Src/Move.c`、`Core/Src/Mate.c` 等同名文件是历史副本，不在当前 Keil 自定义控制编译链中。修改控制逻辑时以 `MDK-ARM/` 为准。

## 构建与检查

部署固件以 Keil 工程为准，交付目标是 ARMCC/ARMClang 全量编译 `0 Error(s), 0 Warning(s)`。

Linux 上可用 `arm-none-eabi-gcc` 做编译级检查，但当前 CMake 目标是 object library，不生成可烧录固件：

```fish
cmake -S firmware/stm32 -B firmware/stm32/build/gcc-check \
  -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build firmware/stm32/build/gcc-check --target rov_ui_model
```

导入时该检查成功编译 30 个源文件；GCC 在 `MDK-ARM/imu.c` 报告 1 个 `-Wtype-limits` 警告。为保持原始固件基线，本次导入没有改动其逻辑。

P4 后编译检查覆盖 33 个源文件；新增 P4 文件无警告，`imu.c` 的同一个导入基线警告仍保留。

P4 安全核心的无硬件测试：

```fish
cmake -S firmware/stm32/test -B /tmp/auv-firmware-test -G Ninja
cmake --build /tmp/auv-firmware-test
ctest --test-dir /tmp/auv-firmware-test --output-on-failure
```

## 遥控代码合并状态

已对照“八推全矢量 ROVER”参考工程完成核验：`RC.c`、`Move.c`、`Motor.c`
中的通道转换、六维遥控量和八推进器控制基础原本已存在于当前固件，因此没有用旧工程
覆盖 P4/P5 代码。本次补入的是参考链路中缺少的安全接收层：

- USART2 使用参考通信程序约定的 PA2/PA3、115200 8N1；
- 控制帧固定为 11 字节：`0xA5 + 10 字节通道数据`；
- 中断只组帧并发布一致快照，主循环负责转换和控制；
- 250 ms 没有完整帧即将所有遥控运动通道清零；若当时已 ARM，同时转为 DISARMED；
- DISARM 或遥控超时期间不更新舵机目标，推进器仍由 P4 输出门控保持中位。

参考帧本身没有 CRC，现阶段只能依靠固定长度、帧头和超时保护。正式自主控制将继续走
USART3 上带 CRC16 的 Pi 协议，不应把 0xA5 遥控帧扩展成自主任务主链路。

## P4 串口与安全行为

- Pi link 当前通过 `AUV_LINK_UART_HANDLE` 配置，默认使用未被原基线占用的 `huart3`。
- 遥控兼容链路使用 `huart2`（PA2/PA3）115200 8N1，与 Pi CRC 链路相互独立。
- 当前 CubeMX 映射为 USART3 TX=PC10、RX=PC11；`AuvLink_Init()` 将其配置为 115200 8N1。
- 上电默认 DISARMED；只有外部漏水/急停输入已经上报，而且 IMU、深度及外部传感器状态均有效且新鲜时，ARM 才可能成功。
- 上电后的前 2 秒为强制 ARM 抑制期；期间即使心跳和传感器状态正常也拒绝 ARM。
- 只有 CRC、version、type 和长度均合法的 HEARTBEAT 才更新时间。
- 超过 500 ms 无合法 heartbeat，状态进入 FAILSAFE。
- heartbeat 恢复后只回到 DISARMED，不自动重新 ARM。
- 漏水、kill 或传感器无效立即进入 FAILSAFE。
- `VectorThrusterPwm_Write()` 是 T1–T8 唯一硬件写入口；非 ARMED 状态强制使用原固件已确认的 `midvalue=1488`。
- SET_ARMED、MOTION_TARGET 和夹爪 ACTUATOR_COMMAND 均通过 ACK 返回实际结果。
- 合法 MOTION_TARGET 现已接入：Pi 目标新鲜时优先于遥控输入，250 ms 超时即清零并撤销 ARM。
- `vx/vy` 当前通过 `Mate.h` 中可标定的前馈增益转换为受统一限幅保护的动力指令；
  yaw 使用 IMU 闭环。真实深度驱动接入前，depth target 不产生垂向输出。
- STATUS 现在返回经过 ARM 门控后的 T1–T8 归一化输出，便于拆桨验收。
- STATUS 的姿态字段来自新鲜 H30 IMU；未收到 IMU 时发送 NaN，不再用零伪装有效姿态。
- 深度驱动应通过 `AuvLink_UpdateDepth()` 提交米制读数；超过 250 ms 未更新后 STATUS/DEPTH 自动转为无效并阻止 ARM。当前尚未绑定具体深度传感器和总线。
- T35-L 信号使用 PA8/TIM1_CH1；未标定固件固定保持 1500 μs 并拒绝开合。
- 遥控模式中 SB=1 启用夹爪，SA 低位闭合、高位张开、中间区停止保持。
- 摄像头舵机使用 PC7/TIM8_CH2；SB=0 时由 SA 控制，也可使用执行器 ID 2 的 Pi 命令。
- 摄像头舵机默认 `AUV_CAMERA_SERVO_CALIBRATED=0`，不启动该 PWM 通道并拒绝运动；实测机械端点后才能打开标定门。

## “八推矢量_代码开发_可改_摄像”选择性合并记录

参考工程中可独立验证的摄像头舵机控制和上电等待要求已经合入，但没有覆盖当前安全架构。
摄像头舵机沿用参考工程的 PC7/TIM8_CH2 和候选 CCR 范围 `2250..3000`，同时增加了
标定门、ARM/控制源授权、协议序号校验、边界检查和每周期斜率限制。候选范围只是待实机确认的
初值，当前默认禁用，因此烧录后不会启动该 PWM 通道或接受摄像头转动命令。

以下差异没有合并：

- 参考工程的推进器中位值与代码/说明互相不一致，当前继续使用已建立基线的 `1488`；
- T2/T3 极性与当前安装表冲突，需逐台拆桨低功率验证后才能修改；
- Pitch/Roll PID 符号及参数未经本机水槽数据验证；
- 参考深度模块占用 USART3，与当前 Pi CRC 通信链路冲突；
- 参考工程会移除 heartbeat、failsafe 和控制源锁，不能整体覆盖当前工程。

连接实机前必须核对 PC10/PC11 是否确实接到 Pi/USB-UART、双方为兼容的 3.3 V UART 电平且共地。不得把 RS-232 电平直接接入 STM32。

## 修改规则

- CubeMX 生成文件中的手工修改只放在 `USER CODE` 区域。
- 修改 `MDK-ARM/Move.c` 后同步更新 `Move_Manual.md`。
- 不提交 `MDK-ARM/Copy_cup/`、HEX/AXF、日志或 IDE 用户状态。
- 协议修改必须同步更新 `docs/protocol/serial-protocol.md` 和两侧黄金测试向量。
- 未完成单推进器低功率方向确认前，不执行多自由度水下测试。

> [!CAUTION]
> 当前代码会启动多路 PWM，并包含已有的推进器极性和舵机 CCR 参数。首次运行必须断开电机电源、拆桨或可靠固定推进器；不要仅凭编译通过就连接实机执行。
