# 自动控制开发转向轻量系统

生效日期：2026-10-08。当前由于运行平台性能不足和 ROS 方案的设计问题，项目暂时弃用 ROS 相关功能，将自动控制系统开发中心转为原生 C++ 轻量系统。该决定由项目负责人提出；目前没有随仓库提交完整的 ROS/轻量方案性能对比数据，本文不作具体 CPU、内存或延迟提升承诺。

## 当前开发与运行边界

高层入口是 Raspberry Pi 上的 `auv_runtime`，核心算法在 `core/`。双摄采集和矫正、AprilTag、网格语义建图、定位、规划、Mission FSM、UART 通信和任务记录在原生系统演进。HTTP 只读监视台、`auvctl`、离线回放和 CTest 是默认调试入口。日志采用现有任务记录器的事件、遥测、地图、轨迹和视频，不要求 rosbag2；坐标变换通过明确的 YAML 标定和 C++ 接口管理，不要求 TF 服务。

ROV 继续使用现有手柄驾驶台与 Pi 串口桥作为稳定基线和数据采集入口。自主模式由 Runtime 独占 UART；ROV 模式由 ROV 桥独占 UART，Runtime 的串口留空且运动关闭。STM32 保留实时姿态/深度 PID、混控和输出保护。默认 DISARM、显式 ARM、超时保护和未标定运动门控继续生效。

ROS 节点、接口和 launch 归档到 `legacy/ros2/`，暂停开发、部署和作为当前验收入口。海参、抓取和转盘历史 ROS 功能的存在不代表这些能力已接入轻量系统；后续逐项迁移、离线验证和实机验收。完整比赛任务尚不能仅因结构迁移视为完成。

## 仓库与构建变化

- `src/auv_core` 与运行时使用的原生源文件、头文件、核心测试迁至 `core/auv_*`，C++ 接口名称保持兼容。
- ROS 剩余文件和 `colcon.defaults.yaml` 迁至 `legacy/ros2/`，以 `COLCON_IGNORE` 排除扫描；归档源码因原生实现迁出不保证直接构建。
- 根 CMake、Pi 安装脚本和 PC 部署打包只引用 `core/` 与 `runtime/`，不依赖归档目录。
- 部署目标和配置路径保持 `runtime/`，STM32/Keil 工程、UART 协议和实测参数保持兼容。

Ubuntu 26.04 / fish 或配置好的树莓派 Linux，从仓库根目录运行（原布局的构建缓存需使用新目录）：

```fish
cmake -S . -B build-native -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build-native -j 3
ctest --test-dir build-native --output-on-failure
./build-native/runtime/auv_runtime runtime/config/runtime.yaml
```

依赖见 [Runtime 手册](../runtime/README.md)。无需 source ROS 环境或执行 colcon。默认配置关闭运动；硬件部署仍需核对设备路径和实测标定。Windows 可用 `-DAUV_PERCEPTION_ONLY=ON` 构建便携核心与测试，但必须安装 OpenCV/yaml-cpp 的 C++ 开发依赖，完整 Runtime 需要 Linux/POSIX。

## 验收与恢复条件

结构迁移后重新在 Ubuntu 构建、执行 CTest，并核对双摄矫正、录像、离线定位、UART 模拟故障与任务状态机。用户已报告迁移前代码在 Ubuntu 验证；该结果不替代本次迁移的编译验收。实机阶段记录双路视频 FPS、延迟、CPU/内存与通信新鲜度，之后再评估性能瓶颈。

若未来恢复 ROS，先明确使用目的和运行资源预算，解决节点间数据复制、生命周期与控制所有权问题，再单独验证适配层。历史完整布局可在独立 checkout 检出 `8064e66`。历史文档中的 ROS 命令只供参考，不作为当前部署步骤。

## 本次本地检查

原生 CMake 的 21 个源码/测试引用及主入口文档链接已检查；核心不含 ROS package manifest、rclcpp 或 ament 构建依赖。标定配置测试通过；ROV 录像测试 8 项通过、1 项因可选解码依赖跳过；维护脚本 Python 语法检查通过。Windows 的 CMake 配置在查找 OpenCV C++ 开发包时停止，因此本次未完成 C++ 编译和 CTest，需在 Ubuntu 重新验证。
