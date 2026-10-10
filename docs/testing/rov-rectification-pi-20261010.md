# ROV 畸变矫正开关实机验证记录

日期：2026-10-10。状态：重启后已完成增量回归、Pi 部署、四种双摄预览/录像组合验证，以及用户追加要求的独立 ROV STM32 烧录。未发送实机 ARM 或运动请求。

## 已完成与证据

- 已连接 `pi@192.168.137.150`，确认独立 ROV 桥与 Runtime 服务正在运行。
- 现场 Runtime 配置为 CSI 下视、USB 前视、320×240；双摄预览矫正开启。Runtime 串口为空，运动、自动启动和自动 ARM 关闭。前视仍为试验标定，仅用于显示。
- 当前修改已在 Pi 的 GCC 14.2 / OpenCV 4.10 / Python 3.13.5 环境完成完整 Release 编译。
- 首轮 CTest：20/22 通过。失败项为 `tag_docking_simulation` 和 `autonomous_simulation`。
- 单独复查：`autonomous_simulation` 通过；`tag_docking_simulation` 仍因 `telemetry, pressure, recording, camera or attitude safety gate` 进入 FAULT。具体触发分项尚未定位，不能宣称回归全部通过。
- PC ROV Python 检查：44 项通过、1 项可选测试跳过；JavaScript 语法检查通过。

仿真中的 ARM、串口与传感器均为测试进程的虚拟输入，不是实机动作。实机 MCU 的 DISARM 为用户现场确认；Runtime 因串口为空显示的 `armed:false` 不能代替 MCU 遥测验收。

## 重启后保留的进度

Pi 构建目录：`/home/pi/auv-rectification-validation-baeba2e045`。编译产物、CTest 记录位于其 `build/` 内；重启后先确认目录与产物存在，不重新上传源码或创建全新构建。

电脑证据目录：`build/rectification-validation/`。其中 `build.log` 保存首轮完整编译及 CTest 输出，`tag-docking-recheck.log` 保存两项单独复查，`checkpoint.json` 保存恢复检查点。源码归档和验证脚本也保留在该目录；这些生成物与日志不进入普通源码提交。

第一次构建在 `/tmp`，已被前一次重启中断；该次 CTest 未完成，不计作通过。随后已改为上述持久目录完成编译与首轮回归。

## 下一步

1. 连接恢复后核验现有构建产物与现场安全状态。
2. 保留失败证据，定位标签接近仿真的具体失效门；不放宽安全阈值。只重跑该失败项，有源码修改才做增量构建和相关回归。
3. 完成回归后，备份并更新摄像头 Runtime，核验配置保持一致；不烧录 STM32，不更新 ROV 串口桥。
4. 验证四种前视／下视矫正组合、双摄预览与录像并发、录制期间切换锁、图像空间和 K/D 元数据、JPEG 尺寸与标记。
5. 用 FFmpeg 解码测试录像，核对帧数；测量接收 FPS、帧间隔和 Pi 资源占用。

主动停止摄像头服务的断流故障测试脚本准备曾被自动审批审查拒绝，仅返回 `blocked by policy`。该项未执行，不能宣称实机断流恢复验收通过。

以上“已完成”和“下一步”保留重启前的检查点事实；以下结果更新最终状态。

## 重启后完成结果

定位和控制循环在等待锁/获取样本前读取时钟，新样本可能被误判为未来时间。现改为取得输入/状态锁后读取时钟，并增加标签安全门的分项诊断，未放宽安全阈值。增量构建后相关测试 5/5、其余测试 17/17，通过同一新二进制合计 22/22（两次调用）。首轮失败证据保留。

已备份并更新 `/usr/local/bin/auv_runtime`，SHA256 为 `2f406e982310b03463bfa1c8e81969b3d9e15baeb6a557db09347da5fce25c1e`。原二进制与配置位于 `/var/backups/auv-rectification-20261010/`。配置逐字节未改，SHA256 为 `0ed67ca4f08e90d212dcf0a06079bbf37a3853b28cead806f936d960bc9a793f`。仅重启 Runtime；ROV 桥未更新/重启，最终 PID 仍为 1516，Runtime PID 1855，两服务 active。Runtime 串口、运动、自动启动、自动 ARM 均继续关闭。

各组合约 6 秒，同时打开两路预览与两路录像；这是短时接收性能，不能替代长期水下验证。

| 下视 / 前视 | 下视预览 FPS | 前视预览 FPS | 录像帧数（下 / 前） |
|---|---:|---:|---:|
| 矫正 / 矫正 | 30.086 | 48.371 | 183 / 295 |
| 原图 / 原图 | 30.086 | 48.870 | 183 / 297 |
| 原图 / 矫正 | 30.167 | 48.333 | 185 / 296 |
| 矫正 / 原图 | 30.167 | 47.714 | 185 / 291 |

四组均无录像错误，录制期间切换被拒绝；JPEG 320×240、`X-Image-Space`、录像图像空间与 K/D 均匹配。八段 AVI 经 Pi 现有 FFmpeg 完整解码，帧数与帧索引记录相同。前视内参仍仅作试验显示，未验证导航有效性。

35 次并发资源采样均未出现 Runtime fault；下视帧龄 12.8–44.6 ms，前视 13.6–45.8 ms，控制更新龄 2.18–49.90 ms，各采样视觉延迟 p99 为 46.8–143.6 ms。代表性 `ps`：Runtime CPU 209%、RSS 99556 KiB；rpicam CPU 19.3%、RSS 63924 KiB。CPU 为进程存活期平均、按单核计数，不能当作瞬时负载或性能提升证明。

## STM32 烧录

权威 `Copy_cup.uvprojx` 独立 ROV 配置编译 0 错误、0 警告。烧录前备份 32 KiB，烧录后完整读回校验一致并复位。HEX SHA256：`acbf329a412454c478f2e302466e2f9a5a3eb5cbf244ba9fc799675bc5dc3c2d`；备份与校验位于 `build/maintenance/20261010T082200460437Z/`。

既有 ROV 桥收到 20 条新鲜 MCU 遥测，保持 `armed:false`、八路输出零、UART CRC 错误零、IMU 新鲜。未发送 ARM/运动请求，桥连接及关闭执行原有 DISARM 保护。`level_calibrated:false`，后续遥控前需要重新校准水平；同时保留 `failsafe:true`、`error_flags:10`（传感器无效与急停位），不宣称所有硬件故障清除。报警是否消失由现场观察确认。

最新证据为 `matrix.json`、`resource-samples.json`、`decode.json`、`postflash.json`、`incremental-recheck.log`、`remaining-regression.log` 与 `checkpoint.json`，均在上述电脑证据目录。日志、录像与编译产物不提交源码。主动断流测试仍未执行；长期并发、受控运动、水池闭环与前视测距标定仍待验收。

