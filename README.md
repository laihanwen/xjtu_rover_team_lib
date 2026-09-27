# AUV

面向水下机器人竞赛的自主 AUV ROS 2 工作区。高层视觉、规划与任务管理运行在 Raspberry Pi / PC，实时姿态、深度、推进器和安全控制运行在 STM32。

## 开发环境

- Ubuntu 26.04 (resolute)
- ROS 2 Lyrical
- fish 4.x
- colcon

## 快速开始

```fish
source tools/setup_dev.fish
colcon build
# 构建后再次加载，以发现新生成的 package 环境
source tools/setup_dev.fish
colcon test
colcon test-result --verbose
```

`tools/setup_dev.fish` 会加载 ROS 2 Lyrical，并在存在时加载当前工作区。仓库级 colcon 配置固定使用 `/usr/bin/python3`，防止用户目录中的 uv Python 与 ROS 系统 Python 混用；它不会创建或修改系统 Python 环境。

当前托管环境将空 `.git` 挂载为只读目录，因此环境脚本会在必要时自动使用 `.git-data` 中的本地 Git 元数据。正常 clone 不受影响，仍使用标准 `.git`。

## 仓库结构

```text
auv/
├── src/                # ROS 2 运行时 packages
├── firmware/stm32/     # STM32 实时控制固件（独立构建边界）
├── vision/             # 数据处理、训练、评估与模型导出
├── models/             # 模型清单、部署配置；权重不直接进 Git
├── datasets/           # 数据集说明和本地目录；数据不直接进 Git
├── hardware/           # BOM、接线、机构与传感器资料
├── docs/               # 架构、协议、任务和测试文档
├── tools/              # 开发、构建和部署工具
└── logs|videos|maps... # 运行输出，不提交 Git
```

当前可构建的 ROS packages 是 `auv_interfaces` 和 `auv_bringup`。其余模块按 [AGENTS.md](AGENTS.md) 的优先级逐步实现，完整落位规则见 [仓库布局](docs/architecture/repository-layout.md)。生成目录 `build/`、`install/`、`log/`、模型权重、数据集以及运行数据不会提交到 Git。

## 安全

默认配置保持 `DISARM`。任何涉及推进器、电调或执行器的测试，必须先断开电机电源、拆桨或可靠固定推进器。
