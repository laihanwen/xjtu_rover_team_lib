# 标签测试 Runtime 部署（2026-10-09）

用户明确授权部署树莓派标签任务程序。最新工作区源码在 Pi 的 OpenCV 4.10 环境原生构建，完整 CTest **20/20**、独立固件主机 CTest **16/16** 通过。修正了标签单元测试的两个素材问题：有效姿态投影越出画面，及丢失超时模拟未经过完整的已观测丢失间隔。未修改运行时保护或放宽门控。

编译期间 SSH 中断且临时目录丢失，恢复后重新构建并获得完整通过结果。安装后第一次立即 HTTP 检查因接口尚未监听而失败，随后一键入口、PC HTTP 实机检查均通过。

- 发布目录：`/opt/auv-tag-docking/20261008-aa1d57979003`（原脚本目录前缀为历史日期，实际部署为 2026-10-09）。
- 配置：`/etc/auv-runtime/pi-auv-tag-docking.yaml`；唯一控制服务：`auv-tag-docking.service`。
- 旧配置/服务状态备份：`/var/backups/auv-tag-docking/20261009T130536Z`。
- 源码基于 `a45a454e9ab7f2567eb1cef4953588185c4b8bd2` 加未提交工作区变更，发布元数据明确为 local changes，不宣称等同该提交快照。
- 固件记录匹配已烧录独立 AUV_TAG_DOCK，HEX SHA256 `dc6ef06c8777eb0d63076dbd1e937e9a48d4efb36c80e27673f04133d145dd30`。

原 ROV 及其他 Runtime 控制服务停止并取消自动启动，原程序、配置保留；标签服务开机进入待命。debug、motion_commands_enabled=false、auto_start=false、auto_arm=false。未发送任务 start、ARM 或机构动作，未重新烧录 MCU。

实机验证：INIT、UART 已连接、新鲜状态、DISARM、运动关闭、recording_ready=true；CSI 下视与 USB 前视快照均返回有效 JPEG。`safe_status=false`、`depth_sample_fresh=false`、`origin_ready=false`、`error_flags=2`，深度没有有效值，未标定保护保持。因此当前支持待命监视/记录，**不能进行自主游动测试**。不据错误位猜测硬件根因；深度信号与标定需继续核验。

监视台 `http://192.168.137.150:8080/`。一键启动实机通过；一键检查应报告真实未就绪条件。原始构建/部署/HTTP 证据保存在本地忽略目录 `build/tag-docking/` 与 `build/auv-entry/`，录像在 Pi 任务记录目录，不加入源码提交。
