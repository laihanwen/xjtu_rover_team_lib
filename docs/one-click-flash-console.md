# 一键烧录独立 ROV 并启动采集驾驶台

更新：2026-10-11。双击 [Flash-And-Start-Rov.cmd](../tools/rov/Flash-And-Start-Rov.cmd)。最新指本地工作区源码，不自动拉取远程仓库。

流程：依赖检查 → SSH/ROV及摄像头服务检查 → ARM/录像/采样检查 → 输入 YES 确认实机安全条件 → 编译独立 Copy_cup → 原固件备份/烧录/完整读回 → 驾驶台连接 → 打开采集助手。检查失败不继续烧录，错误显示具体原因及日志目录。

```powershell
./tools/rov/Flash-And-Start-Rov.ps1 -CheckOnly
./tools/rov/Flash-And-Start-Rov.ps1
```

自动读取项目工具环境及已有 DPAPI 凭据；有密码时优先密码认证，避免无关加密私钥干扰。支持 -Python、-PiHost、-PiUser、-WebPort、-Keil、-Probe、-Pack。需要已配置好的独立 ROV 服务、摄像头服务和本机可识别烧录探针。

默认 Pi 192.168.137.150，控制8888、摄像头8080、驾驶台8767。脚本不自动 ARM、水平校准、录像或部署新 AUV Runtime。烧录后先岸上放平并显式水平校准。摄像头保持实际 Pi 配置，640×480 下视原图不会被烧录入口恢复为320×240。

旧 tools/auv/Flash-And-Console.cmd/ps1 仅转发到同一 ROV 入口，目录名不表示烧录 AUV。旧参数 SafeToFlash 映射到当前一次安全确认，非永久免确认；CheckOnly 现在检查 Pi 连接和服务。AUV 使用独立构建/烧录及任务入口。

固件备份/读回与失败记录：build/maintenance/；驾驶台启动：build/rov-test/；录像及采样：data/rov-recordings/；遥测：logs/rov/。

原双模式说明见 [历史归档](archive/dual-mode/auv-rov-mode-switch.md)，不作为当前操作流程。
