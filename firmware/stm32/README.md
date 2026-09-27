# STM32 firmware

这里是实时底层固件的独立构建边界，职责包括：

- IMU 与深度传感器采样
- 姿态估计
- Roll / Pitch / Yaw / Depth PID
- Thruster mixer 与 ESC PWM
- 机械执行器
- heartbeat、漏水检测和 failsafe

当前尚未确认 STM32 型号、CubeMX 工程、GPIO、定时器、ESC PWM 范围、推进器位置和方向，因此暂不生成芯片工程或控制参数。

引入 CubeMX 工程后应遵守：

- 自动生成文件中的修改仅放在 `USER CODE` 区域。
- 协议编解码和控制算法尽量放在不会被 CubeMX 覆盖的独立源文件。
- 默认 DISARM，启动输出为中性值。
- mixer 参数由真实推进器几何布局生成并进行台架验证。
- host 侧协议单元测试应可在不连接硬件时运行。

任何首次电机测试必须断电、拆桨或可靠固定推进器。
