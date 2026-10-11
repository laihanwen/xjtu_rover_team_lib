# ROV 到 AUV 的姿态与深度基准迁移

用户已确认偏航方向、独立深度新鲜度、深度零点经过 ROV 验证。本记录核对数值在当前工程中的流向，不重复要求现场验证。

## 已迁入

- 三份 AUV YAML 的 `imu_offsets_deg` 为 [0,0,0]：权威 Keil 工程编译 MDK-ARM/imu.c，该文件已在 MCU 扣除手动水平 roll/pitch 参考和首帧 yaw 原点，Runtime 不再重复扣参考。零表示无附加变换，不是传感器原始偏移为零。
- 人工实测机头上抬 pitch 为负、左侧上抬 roll 为正，与 X 前/Y 左/Z 上姿态公式一致，roll/pitch 转换符号 +1 已记录在 `hardware/rov-auv-datum-transfer-20261010.yaml`。
- AUV 固件 `AuvLink_UpdateDepth` 先扣 `AUV_DEPTH_ZERO_M`，后发布 STATUS/DEPTH；Runtime 定位 `depth_zero_m` 保持 0，避免同一零点扣两次。这是 AUV 数据口径约定，不能用来声称 ROV 原始表压已经归零。
- ROV 固件的 `AuvDepth_UpdateGauge` 不在 AuvLink 扣零，桥只传递读数。因此 ROV 数据回放若要计算水面基准深度，需要使用当时已验证的零点，而不是盲套 AUV 已归零数据口径。

## 未猜填的数值

当前仓库中 `AUV_DEPTH_ZERO_CALIBRATED=0`、`AUV_DEPTH_ZERO_M=0.0f` 是未配置默认值；未找到 ROV 验证后的实测零点记录。不能用岸上 -0.25 m 自动替代，也不将 calibrated 置 1。

ROV `AUV_ROV_YAW_FEEDBACK_SIGN=-1` 只作用于控制混控反馈，不改变上报 yaw，不能证明定位 yaw 转换符号为 -1。已验证偏航事实保留，但具体相对当前机体右手坐标的数值记录尚未找到。因此 `imu_signs` 继续空数组（接口要求完整三元组），部分已确认符号单独记录，不用默认 yaw 符号补全。

Pi 只读 SSH 核对本轮连接超时，未拿到设备现存配置；未修改或重启服务。后续设备恢复后优先读取既有零点和符号文件/构建参数；若记录不在设备，再由用户提供原 ROV 的数值或记录路径，无需重做已确认验证。

本轮为源码配置迁移，未部署、未烧录。定位全局 verified 与运动门仍关闭，池深未知继续保留。完整数值迁移尚待 yaw 转换值和水面表压零点取证。

后续用户确认池深 1.4 m，已更新三份 AUV 模板。Pi 恢复连接后，只读检索 `/etc/auv-runtime` 及 `/home/pi/auv`、`auv-current`、`auv-releases`、`auv-test-reports`、旧验证源码目录、`rov` 的 YAML/头文件/JSON；找到的 imu_signs 为空、depth_zero_m 为 0，未找到实测 yaw 映射或归零数值。不能将这些默认字段当验证值；不更改运行服务。
