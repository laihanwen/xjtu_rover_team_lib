# AUV 标签测试一键入口

Windows 双击 `tools/auv/Check-Tag-Test.cmd`：连接已配置的 Pi，核对最近一次本地烧录记录、部署固件哈希、独立标签服务与配置，调用原生配置校验，再读取实时状态。检查不修改 Pi 配置、不启动或停止服务、不发送任务 start 或 ARM。未就绪返回失败并显示缺项。

Windows 双击 `tools/auv/Start-Tag-Test.cmd`：通过静态检查后，启动已部署的标签 Runtime **待命服务**并打开 `http://192.168.137.150:8080/`。debug / auto_start=false / auto_arm=false 是硬性条件；重复点击不会重启已有服务、清除故障或重复请求任务。冲突服务存在时拒绝，不自动停止 ROV 或抢占串口。

待命启动与实机作业开始分开。完成标定、视觉/深度/记录/原点等启动条件后，可在 PowerShell 显式请求任务进入 WAIT_ARM：

```powershell
./tools/auv/Tag-Test.ps1 -Action task
```

此动作再次检查实时安全门，仅发送 `start`，不发送 ARM。当前任务约 5 秒等待 ARM；操作员须在现场安全条件满足时，另行显式执行 `auvctl arm --confirm SAFE_TO_ARM`。脚本不延长等待或绕过启动锁存。停止使用树莓派 `auvctl disarm`。

命令行入口也支持 `-Action check`、`-Action start`、`-NoBrowser` 及 `-Python`（安装了 paramiko 的项目隔离环境）。复用已有 DPAPI 凭据与已保存 SSH 主机密钥，未知主机拒绝连接。日志位于忽略目录 `build/auv-entry/时间/`，不打印或保存密码。

入口同时查找普通用户 LOCALAPPDATA 与 Codex 应用隔离目录中的工具环境及 DPAPI 文件，兼容资源管理器双击与应用内运行。`-LocalOnly` 只验证 Python、paramiko 和凭据解密，不连接设备；Windows PowerShell 5.1 下普通路径与备用路径均已验证。

固件哈希核对是**烧录/部署记录一致性**，并非当前 MCU 固件在线识别。设备被其他工具重新烧录后，必须重新用本项目流程烧录并校验，不能用历史记录证明实机身份。

## 当前实机检查结果（2026-10-09）

SSH 检查成功到达设备，但缺少 `/etc/auv-runtime/pi-auv-tag-docking.yaml`；一键检查正确失败，未启动实机服务。需先完成匹配 Runtime 原生编译、CTest 和 `runtime/deploy/deploy_tag_docking.sh` 独立部署。入口不会自动安装服务、覆盖配置或把标定标记改为已验证。

后续用户授权部署后，已完成独立安装，缺少配置的问题已解决。一键 `-Action start -NoBrowser` 实机通过，服务与双摄、记录器就绪。检查入口仍会因深度、安全状态、定位原点和标定未就绪返回失败，这是实机作业门控而非入口路径错误；不能把待命成功理解为可 ARM。详情见 [部署记录](auv-tag-runtime-deployment-20261009.md)。

Python 门控测试 6/6 通过，涵盖自动 ARM 配置拒绝、相机专用配置拒绝、缺失状态门拒绝、已 ARM/任务未就绪拒绝。真实启动路径尚未验收，因为实机部署前置条件未满足；不代表可进行动力测试。
