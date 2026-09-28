# P4 heartbeat 与 failsafe 验收

## 已自动验证

- 固件协议编码匹配 P3 heartbeat 黄金帧和 CRC 标准检查值。
- 固定 71-byte 缓冲解析器拒绝坏 CRC、未知版本和非法长度并能恢复同步。
- 上电状态为 DISARMED；没有合法 heartbeat 时拒绝 ARM。
- 500 ms 边界仍保持状态，501 ms 进入 FAILSAFE。
- HAL tick 的 `uint32` 回绕不会绕过 timeout。
- heartbeat 自身不能 ARM。
- 漏水、kill、传感器无效均触发 FAILSAFE。
- heartbeat 恢复只进入 DISARMED，必须再次显式 ARM。
- ROS bridge 仅在收到匹配 sequence 的 ACK 后报告 ARM 成功。
- T1–T8 唯一 PWM 写入口在非 ARMED 状态强制中位。

## 无硬件验收命令

```fish
cd /home/hanwen/auv

cmake -S firmware/stm32/test -B /tmp/auv-firmware-test -G Ninja
cmake --build /tmp/auv-firmware-test
ctest --test-dir /tmp/auv-firmware-test --output-on-failure

cmake -S firmware/stm32 -B firmware/stm32/build/gcc-check \
  -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build firmware/stm32/build/gcc-check --target rov_ui_model

source tools/setup_dev.fish
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

部署前还必须在 Keil 对 `MDK-ARM/Copy_cup.uvprojx` 做全量构建。Linux GCC 目标只做编译检查，不生成可烧录镜像。

## 实机台架验收（必须人工执行）

首次测试断开 ESC 主电源；需要观察 PWM 时必须拆桨或可靠固定推进器，并准备物理断电。

1. 核对 USART3 PC10/PC11、3.3 V 电平和共地。
2. 仅给控制板上电，确认八路输出为已验证的中位。
3. 启动 bridge，确认 `/stm32/status` 可见且默认未武装。
4. P5 尚未提供有效传感器输入时，ARM 必须返回 unsafe。
5. 使用测试桩显式标记传感器有效后，发送 ARM 并核对 ACK。
6. 停止 bridge；超过 500 ms 后必须进入 FAILSAFE，八路输出回中位。
7. 恢复 bridge；必须保持 DISARMED，不能自动恢复动力。
8. 分别注入 leak、kill、sensor-invalid，确认均立即回中位。

在上述结果逐项记录前，不允许安装桨叶进行多推进器测试。
