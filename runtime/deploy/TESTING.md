# 树莓派轻量运行框架测试脚本

在树莓派仓库目录（例如 `/home/pi/auv`）下执行。脚本只读取状态、网页和系统信息，**不会发送 START、ARM 或运动目标**。每次运行将 JSON 报告写到 `~/auv-test-reports/`，可用 `AUV_REPORT_DIR` 改位置。

```sh
runtime/deploy/run_bench.sh preflight
runtime/deploy/run_bench.sh collect
```

`preflight` 验证服务、控制 socket `0660`、DISARM、运动输出关闭和只读网页。摄像头或串口尚未接入时显示 `PENDING`，基础部署仍可通过。要把缺失硬件视作失败，可直接运行 `python3 runtime/deploy/bench_checks.py preflight --strict-hardware`。`collect` 留存状态、service/journal、设备列表、温度和 throttling 信息。

## 摄像头和视频

接入下视 USB 摄像头后，先确认 `ls -l /dev/v4l/by-id/` 有稳定路径。把该路径写入 `/etc/auv-runtime/runtime.yaml` 的 `camera.source`，配置实际标定值，然后 `sudo systemctl restart auv-runtime`。重启后仍应为 DISARM。执行：

```sh
runtime/deploy/run_bench.sh camera 30
```

脚本检查稳定设备路径、持续帧新鲜度、视觉平均速率至少 10 Hz、HLS 播放列表和分片可访问、编码状态正常。网页地址由配置中的 `web.bind` 和 `web.port` 决定。这一步不需要启动 Mission。

## STM32（推进器电源断开）

连接 STM32，确认 `ls -l /dev/serial/by-id/`，把稳定串口路径填到 `serial.device`。保持 `motion_commands_enabled: false` 并重启服务，随后执行：

```sh
runtime/deploy/run_bench.sh serial 30
```

脚本要求 STATUS 有效、无漏水或错误、持续 DISARM，且 heartbeat 平均至少 19 Hz。它不向 STM32 注入假 STATUS，也不发送 ARM。可另用 `auvctl status` 对照电压、姿态和深度的真实读数。

## 无桨台架故障观察

先断开推进器电源或拆桨，并安排人在现场观察。需要 Mission 处于活动阶段时，由操作员手动执行 `auvctl start`。在拔摄像头或串口前运行观察脚本：

```sh
runtime/deploy/run_bench.sh fault-watch camera 15
runtime/deploy/run_bench.sh fault-watch serial 15
runtime/deploy/run_bench.sh fault-watch leak 15
runtime/deploy/run_bench.sh fault-watch error 15
```

脚本每 0.1 秒记录状态，验证进入 FAULT 且显示 DISARM。漏水和错误标志需要真实硬件安全注入；脚本不会主动改变硬件。故障恢复后由操作员手动 `auvctl disarm`、检查设备、`auvctl reset`，是否再次 START 由现场决定。断线时 STATUS 可能是最后一次缓存值，因此应同时核对 STM32 本机 failsafe 和物理输出。

## 30 分钟性能记录

摄像头、HLS 和 STM32 均稳定，推进器电源断开后执行：

```sh
runtime/deploy/run_bench.sh endurance 1800
```

保留原有 `acceptance.py` 的 30 分钟指标：视觉 ≥10 Hz、控制和 heartbeat ≥19 Hz、视觉延迟 p99 <250 ms、RSS <700 MiB、CPU 平均 <300%、温度 <75°C、无 throttling。新增持续 DISARM、相机帧新鲜、STM32 STATUS 有效、无漏水/错误和日志正常要求。报告自动保存；任一指标未达标返回非零退出码。

从 PC 收集报告：

```sh
scp -r pi@192.168.137.150:/home/pi/auv-test-reports ./auv-test-reports-pi
```

正式运动输出测试需要完成标定和独立安全流程；这些只读脚本不会解锁运动。

## 自主模式验收

只有上述相机、串口、故障和耐久测试全部通过后，才可制作自主比赛配置。第一次自主模式测试仍必须断开推进器电源或拆桨。确认启动后无需 `auvctl start`，状态自动从 `INIT` 前进，并验证远程 `start`、`arm`、`pause` 和 `reset` 均被拒绝。随后执行紧急 `auvctl disarm`，应进入 `FAULT` 并保持 DISARM。

在同一次开机中重启服务：

```sh
sudo systemctl restart auv-runtime
auvctl status
```

状态必须报告 `autonomous mission already started since boot`，不得再次自动开始或 ARM。只有整机重启才会清除 `/run` 中的自主启动锁存。正式有桨或下水测试必须另行执行现场安全评审。

自主启动前还应从事件日志确认 `AUTO_START` 仅出现在相机、视觉和STM32状态连续稳定之后，并从状态中的 `control_age_sec` 验证控制循环健康。控制循环超时会停止Linux侧heartbeat；必须同时在真实STM32上验证其约500 ms heartbeat failsafe确实输出中性值并解除武装。
