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

## 修改规则

- CubeMX 生成文件中的手工修改只放在 `USER CODE` 区域。
- 修改 `MDK-ARM/Move.c` 后同步更新 `Move_Manual.md`。
- 不提交 `MDK-ARM/Copy_cup/`、HEX/AXF、日志或 IDE 用户状态。
- ROS 串口接入前，先确定协议字节序、CRC16 变体和黄金测试向量。
- 未完成单推进器低功率方向确认前，不执行多自由度水下测试。

> [!CAUTION]
> 当前代码会启动多路 PWM，并包含已有的推进器极性和舵机 CCR 参数。首次运行必须断开电机电源、拆桨或可靠固定推进器；不要仅凭编译通过就连接实机执行。
