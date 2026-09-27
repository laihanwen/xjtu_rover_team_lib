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

## 当前结构

```text
src/
├── auv_interfaces  # 项目公共 ROS msg/srv 接口
└── auv_bringup     # 启动入口和安全默认参数
```

后续 package 按 [AGENTS.md](AGENTS.md) 中的优先级逐步加入。生成目录 `build/`、`install/`、`log/` 以及运行数据不会提交到 Git。

## 安全

默认配置保持 `DISARM`。任何涉及推进器、电调或执行器的测试，必须先断开电机电源、拆桨或可靠固定推进器。
