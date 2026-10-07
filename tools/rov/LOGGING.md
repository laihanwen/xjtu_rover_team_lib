# IMU / PID 运行日志

新版 PC 驾驶台启动即创建 `logs/rov/<UTC会话>/`，每 5 秒采样一次当前状态，写入 `telemetry_0001.jsonl`；10 MiB 后分段。`--log-dir` 可修改根目录。每条立即 flush；正常退出关闭文件。突然断电仍可能丢失最后一条，不承诺磁盘 fsync。日志不依赖网页保持打开，不更新网页许可租约、不发送 ARM、UART 或其他控制命令。

每条包含 UTC、PC monotonic、连接/遥测年龄、IMU 三轴角度、深度、校准标志、ARM/failsafe/错误位、八路实际归一化 PWM 输出、手柄输入及串口 CRC 错误。另记录 STM32 的控制周期时间戳、IMU 新鲜标志、三轴误差、实际参与控制的三轴纠正量、俯仰/偏航角速度，以及 roll/pitch 纠正与 heading hold 状态。DISARM 时纠正量为零。航向目标可以由 yaw + wrapped yaw_error 恢复。

PID 纠正量是混控前的控制力矩指令（含倍率、死区开关及 PD 限幅），不是测得的推力，也不是 ESC PWM；实际八路输出见 imu.outputs。对当前手柄配置，pitch 最大纠正量为 40（PID 输出20 × 倍率2），roll/yaw 为20。未触发死区纠正时零值正常，不能把“启用功能”当作“正在纠正”。日志仅采样当前值，5 秒内的快速振荡或短暂故障可能未被采到；若需分析瞬态，另行增加高速数据记录。

无连接、过期遥测会标记无效；无 PID 诊断或其年龄超过0.3秒时 `pid_valid=false,pid=null`，不会用配置常量冒充实际 PID。缺失角度时 imu_valid=false；原始状态可以保留以分析传感器无效。若日志磁盘写入失败，记录线程停止并通过 `/logs` 与页面显示错误，遥控链路继续运行。不会自动删除旧日志，请定期归档。

新增协议 `0x84 PID_DIAGNOSTIC`，49字节 payload：`uint32 tick_ms`、`uint8 flags`、11个 little-endian float。flags位0armed、1imu_fresh、2roll_active、3pitch_active、4heading_hold_active。float依次为roll/pitch/yaw(deg)，roll/pitch/yaw_error(deg)，roll/pitch/yaw_correction，pitch/yaw_rate(deg/s)。诊断随 STATUS 10Hz发送，PC每5秒落盘；角度/误差/纠正项来自同一控制周期。STATUS与诊断为独立帧，时间不保证完全同步，分别保留其年龄。MCU tick为uint32，约49.7天回绕。

启用真实 PID 日志需要更新固件、Pi trial_server/trial_protocol 与 PC驾驶台。旧固件仍可记录 IMU，但 PID 标记不可用。2026-10-06 已烧录、完整读回校验并更新 Pi/PC，真实PID诊断回传成功；操作者确认岸上放平后重新手动校准成功，全程未 ARM。连续4条校准后日志有效，实测最近采样间隔5.00035秒。HEX SHA256：08c3859297c698b5b070e4f7eab12abdc3997990513b095c2903e67c53a7d261。重新烧录或断电会清除RAM水平基准，须再次岸上校准。

## 2026-10-07 自动保持诊断

改动前远程基线：0784d52，origin/codex/rov-manual-level-video。新诊断源码尚未部署。STM32 回传保持状态原因和优先混控运动保留比例；协议见 serial-protocol.md。网页诊断超过 300ms 显示过期，不将开关状态当作锁定状态。无 ARM 不输出保持推力。

启动 PC 控制服务时增加 `--diagnostic-log`，在 logs/rov/diagnostic 下额外记录 10Hz JSONL，独立线程、每段10MiB轮转，原5秒日志保持不变。GET /logs 同时返回高频日志目录/计数/错误，不更新控制许可。记录的是 PC 最近接收的快照，可能重复同一个 MCU tick；不是 STM32 控制环全量记录，也不会提升0.5–1Hz深度计采样率。磁盘错误停止该日志并显示错误，不阻塞控制；没有自动删除旧日志。
