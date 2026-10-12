# Ubuntu 局部定位误差修复：2026-10-12

本轮针对此前局部模拟约 0.108 m 位姿误差修复。源码基于 `784dd0e` 的未提交工作区；没有连接 Pi、部署、烧录或实机运动。历史验证结果仍见 [任务一增量报告](ubuntu-task-one-results-20261012.md)。

## 改动与原因

- 管线方向：投票窗口只用于选择支持集，再对实际线段角度做模 90° 加权平均。不再把首个获胜窗口中心直接当成测量角度，避免窗口选择造成系统偏差。
- 局部位姿：局部检测本已使用水平池底、深度、IMU 姿态和相机外参投影。新 `local_plane_pose` 用相同约束拟合平面刚体位置与航向，不再让自由平面 PnP 将少量像素偏差拟合成倾斜；继续检查原有像素残差、节点基线、倾斜和场地边界。完整外框 PnP 未改。
- 冲突处理：图像拓扑/语义冲突停止传播和输出，但保留同阶段锚点用于连续里程计拒绝跳格。真正阶段切换仍清空锚点；未延长里程计预算或放宽安全门控。
- 建图顺序：语义观测必须先通过融合检查，被拒绝的地标不能写入地图。
- 回归：新增带相机/深度计偏移、IMU 倾斜和像素扰动的确定性测试；新增冲突后禁止传播、长期跳格仍被拒绝的检查。模拟真实位姿只用于测试断言和诊断，绝不输入 Runtime。

## 最终完整回归

完整 Runtime 编译通过，CTest **33/34**，耗时 127.12 秒。Python 测试夹具语法检查与 `git diff --check` 通过。本轮没有重新执行 Python 工具全集或真实素材回放，不能套用旧版本结论。

最终证据：`build/ubuntu-task-one-evidence/repair-continuity/`，包含完整 `LastTest.log`、模拟状态、有效位姿误差、配置、事件、UART 目标和最终帧。

该次局部模拟的 391 个有效位姿采样：最大误差 **0.029338 m**，中位数 **0.009578 m**，P95 **0.018561 m**。这只是合成场景中报告有效的样本，不包含无效位姿，也不是实机导航精度。

同一最终二进制再次单独运行局部闭环：仍因建图不完整失败，耗时 43.93 秒；360 个有效样本最大误差 **0.021687 m**，中位数 **0.009194 m**，P95 **0.016438 m**。证据及该次 `LastTest.log` 在 `build/ubuntu-task-one-evidence/repair-continuity-repeat/`。两次均未触发原有 0.2 格（模拟 0.1 m）误差断言，不能据此宣称所有场景误差已消除。

Runtime SHA256：`b4e6b45d4fb559c67daa347dd2bd34000a41d4992f7f5adb20be42258e6502b9`。

## 未完成范围

局部 `local_task_one_closed_loop` 仍失败：到达 `BUILD_MAP`，只积累下排两个方锥和中间空格；上两排仍未知。走廊扫描结束后报告 `calibrated observation corridor exhausted before map completion` 并进入 `FAULT`。

因此局部正常场景没有完成上浮、表面重定位、四锥或退出，其后的缺失表面负例也没有执行。最终故障快照仍显示模拟 MCU armed，不能把该快照称为已验证 DISARM。

独立 `a2_closed_loop`（完整外框，含缺失表面负例）与 `grid_exit_closed_loop` 通过，不替代局部整链路验收。下一步需解决裁剪管线/重复网格中的稳定节点提取与连续关联，恢复足够真实观测；不填造未知格或增大误差门槛。

## 复现

沿用已有完整 Linux 构建配置与依赖：

```bash
cmake --build build/task-one-ubuntu-20261012 -j 3
env AUV_SIM_EVIDENCE_DIR=/home/hanwen/auv/build/ubuntu-task-one-evidence/repair-reproduce \
  ctest --test-dir build/task-one-ubuntu-20261012 --output-on-failure
```

生产标定、自动 ARM、运动开关及实机配置未修改。验证时为未提交工作区，随后按用户要求与覆盖累计和分段验证源码一并提交上传。
