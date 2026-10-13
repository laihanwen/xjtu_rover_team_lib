# 最短标签往返与证据回传

`runtime/config/pi-auv-tag-docking.yaml` 是本次入口。`approach_only: true`、`observation_sec: 8`、`return_to_origin: true` 表示：建立本次相对原点 → 前视找 ID18 → 下视确认并居中 → 观察 8 秒 → 沿实测出航点返回原点 → DISARM。录像从 Runtime 启动时已经写入双摄原始 MJPG；`OBSERVATION_START/END` 是该观察窗口的事件，不代表录像文件只覆盖这 8 秒。其他比赛阶段没有删除，但该 profile 不执行抓取、转盘或 A2 遍历。

默认仍是 `motion_commands_enabled: false`、`auto_start: false`、`auto_arm: false`、`corridor_verified: false`、`depth_verified: false`；源码更新不会解除门控。部署、烧录、ARM、下水是彼此独立的验收步骤。运动需当前实测安全走廊、深度、方向、定位和固件门全部通过。若不能自主运动，可用 ROV 手动完成相同拍摄路线，PC 驾驶台保存原始录像/时间戳/操作日志；Pi Runtime 必须保持摄像头和 UART 所有权互斥。手动日志及人工九宫格只能标成 `operator_manual`，不能冒充 `tag_docking` 自主决策或机载建图。若 ROV 配置未开启机载 `recording`，下列 Pi 回传工具不会生成完整包，应使用已有 PC 录像回传脚本并离线补充人工事件。

## 一键回传

在 PC 仓库根目录，Windows PowerShell：

```powershell
./tools/auv/Collect-MissionRun.ps1 -HostName pi@192.168.137.201 -IdentityFile C:/Users/21670/.ssh/id_ed25519
```

实际设备若是仓库维护地址，改为 `pi@192.168.137.150`。身份文件可不填，使用 `ssh-agent`/默认密钥。`-RunId` 选择指定机载运行目录；不填时取最新一个。`-ManualGrid` 可给 `minimal-task.html` 导出的 `manual-grid-*.json`。Linux/macOS 使用 `python3 tools/auv/collect_mission_run.py --host pi@192.168.137.201 --identity ~/.ssh/id_ed25519`。已有运行目录离线重算可用 `--local-run /path/to/run_id`。需要账户有运行目录的只读权限；不使用密码或 sudo，不访问控制串口。

本工具选 B：一次性回传机载原图 MJPG 和 JSONL 索引。文件本来就在 Pi 上写盘，避免 PC 网络实时录制再次丢帧和独立 PC/Pi 时钟对齐。输出在忽略的 `logs/mission-runs/run_YYYYMMDD_HHMMSS/`，其中 `raw/` 保留机载目录，包括 `events.ndjson*` 轮转段、双摄 MJPG/索引、`evidence/` 网格/标签图及 JSON、配置快照、manifest、轨迹和规划文件。回传后输出 `source_sha256.csv`、`review_events.ndjson`、`review_summary.json`、`event_video_alignment.csv`、`top_down_map.png`、`keyframes/`、`review.md`。若没有机器地图，PNG 九格全为未知；若提供人工地图，图示锥类别/格位但来源写明 `operator_manual`，不改写原始 `events.ndjson`。标签位置只标出实测悬停轨迹点附近，不冒称全局坐标。

每一行对照表列出事件 Pi `steady_sec`、事件墙钟、两路最近原始帧在各自视频中的实际秒数、帧序号、时间差、段文件及字节偏移。默认最多相差 0.5 秒；超出显示 `NO_FRAME_WITHIN_TOLERANCE`。录像“时长/FPS”由逐帧 Pi 接收时间算出，MJPG 本身没有固定 FPS。Pi 时间戳是收帧/解码时间，不是曝光时间；双摄无硬件同步。帧索引内的重复遥测不是独立测量。视频缺失、索引越界、地图未生成、路线事件不完整等在终端和 `review.md` 明确列错，进程返回非零；已有原始素材仍保存，可供复盘。

## 事件与验收

复用原事件格式 `run_id`、`timestamp_unix_ms`、`steady_sec`、`event`、`detail`。`TIME_ANCHOR` 自身两项时间给出运行时钟对应。`TAG_DOCK_START`、`ARM_REQUEST`、`ARM_CONFIRMED`、`TAG_IDENTIFIED`（ID、相机、帧号、相对机体位姿）、`TAG_ABOVE_CONFIRMED`、`OBSERVATION_START/END`、`RETURN_START/COMPLETE`、`TASK_COMPLETE` 描述最短流程；`TAG_DOCK_TRACE` 给相对轨迹。机载地图若由 A1 实际生成，原有 `MAP_COMPLETE` 对应 `evidence/map_*.jpg.json`，其中每格与四锥轮廓/类别/格位有来源帧。`tag_docking` 不调用 A1 完整九格建图，只有黄边候选和双摄视频；不得把四锥人工标注写成机器识别。抓取数量、碰撞、转盘缺少正向观测时在 `review_summary.json` 保持未确认或零个**已证实**抓取，不等同“没有碰撞”或“任务已完成”。

上机逐项核对：

- [ ] `auv_runtime --check-config` 通过；确认有效配置中的 `motion_commands_enabled`、标定门、一次性启动锁存和固件版本。
- [ ] 在 DISARM 下确认双摄原图、帧索引、事件日志同时持续增长；确认 Pi/STM32 串口只有一位所有者。
- [ ] 受控条件下显式 START/`arm --confirm SAFE_TO_ARM`；记录 MCU ACK 与 armed STATUS，验证超时、急停、断摄像头会停推并 DISARM。不得为了跑通改标定标志。
- [ ] 标签进入下视后确认先进入 `DOWN_CENTER`，稳定居中才出现 `TAG_ABOVE_CONFIRMED`/`OBSERVATION_START`，约 8 秒后 `OBSERVATION_END`/`RETURN_START`；到原点后 `RETURN_COMPLETE`/`TASK_COMPLETE`。
- [ ] 回传命令返回后检查 `review.md` 是否有 ERROR，逐个打开前/下视 `keyframes/`；用 CSV 中 `source` 的 `offset+bytes` 与帧索引核对同一帧，检查事件时间差绝对值不超过所设阈值。
- [ ] 查看 `top_down_map.png` 和 `review_summary.json` 的 `map_source`；四锥若不是机载地图证据，明确标人工或未知。抓取、碰撞和转盘不得仅凭控制命令宣布完成。

本地替代验证：Windows 便携构建的 `tag_docking` 单项 CTest 验证状态与门；`python -m unittest tools/auv/test_collect_mission_run.py` 验证帧对应和损坏索引。Linux/Pi Runtime 编译、设备时序、双摄帧率、下水返航仍只能在目标设备验收。`file:` 视频回放和 `auv_surface_replay` 可另行检查已有录像/索引，但不能替代实机安全门和动力闭环。
