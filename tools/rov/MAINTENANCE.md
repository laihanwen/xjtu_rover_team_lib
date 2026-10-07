# 重复编译、烧录和部署

## 一键启动水下测试

双击本目录 `Start-RovTest.cmd`，或在仓库根目录运行 `./tools/rov/Start-RovTest.ps1`。脚本启动驾驶台并打开浏览器，默认开启10Hz诊断日志；已有驾驶台时复用，不创建第二个控制器。显示手柄、树莓派链路、水平校准和ARM状态。日志保存到 `logs/rov`，启动错误保存到 `build/rov-test/<时间>/`。

需要编译和重新烧录时运行 `./tools/rov/Start-RovTest.ps1 -Flash`，或 `Start-RovTest.cmd -Flash`。脚本拒绝已知ARM状态，停止本机控制进程后调用维护脚本编译、备份烧录并读回，失败不启动控制台。运行前保持未ARM、机器人固定；不要在运动和录像中烧录。烧录不更新Pi服务。

每次MCU重启后先在岸上可靠放平、手柄回中，在驾驶台勾选岸上确认并点击水平校准；下水后保持左肩许可、点击ARM。脚本不自动校准或ARM。水下升降杆回中锁当前深度，保持ARM和左肩许可才会输出自动保持推力；STOP/Esc或撤销许可停止。`-NoBrowser`供自动化使用，`-PiHost`、`-WebPort`可覆盖地址；复用已有控制台时沿用其配置。

在 Windows PowerShell 的仓库根目录执行。脚本从自身路径定位仓库，不依赖当前目录。

```powershell
# 仅编译，失败立即退出
./tools/rov/Maintain-Rov.ps1 build
# 仅检查 Pi 服务和 runtime 状态，不发送运动指令
./tools/rov/Maintain-Rov.ps1 check
# 编译 → 现场备份/烧录/读回 → 部署 → 检查
./tools/rov/Maintain-Rov.ps1 all
```

`flash` 仅烧录现有 HEX；`deploy` 仅更新 Pi，二者也可单独使用。烧录前退出 PC 遥控程序、保持 DISARM，并断开电机电源或可靠固定。脚本不发送 ARM，但复位 MCU 和重启服务会中断正在进行的控制及录制，不能在运动或录制过程中运行。

默认 Keil、PTD02HW 探针、DFP 路径和 Pi 地址来自本次实机配置，可通过 `-Keil`、`-Probe`、`-Pack`、`-PiHost`、`-PiUser` 覆盖。脚本不修改 PID、混控矩阵或电机参数。MCU 重启后仍须岸上手动水平校准，再由操作者显式 ARM。

Pi 使用 SSH 密钥或环境变量 `AUV_DEPLOY_PASSWORD`；密码不写进脚本、命令参数或日志。无密码时需要 SSH 密钥及免密码 sudo。首次连接将主机密钥保存到 `build/maintenance/known_hosts`，后续密钥变化会拒绝连接；首次指纹应由操作者核对。Python 依赖为 pyocd、intelhex、paramiko；启动器兼容现有 `build/debug-deps` 和 `build/deploy-ssh`，不自动安装系统包。

部署针对已配置好的 Pi，要求已有 `auv` 用户、两个服务及 `30-manual-trial.conf`、pyserial 和 CMake/原生编译依赖；新机器先按 `runtime/deploy/install_pi.sh` 完成初始化。远程先构建并运行 CTest，成功后才停服务安装。默认保留实机 runtime.yaml 和服务/串口配置；显式传 `-ApplyPiProfile` 才应用当前双摄模板（CSI 下视、USB 前视），使用前检查 USB 设备路径和绑定地址。不会调用旧安装脚本中的供电检查或 apt 升级。

每次运行日志位于 `build/maintenance/<UTC时间>/`：编译日志、烧录前现场 32 KiB 备份、合并镜像、HEX SHA256、完整读回校验结果、部署和检查日志。Pi 安装前备份配置、桥接代码及 runtime 到 `/var/backups/auv-maintenance/<UTC时间>/`。烧录保留 HEX 未覆盖字节，校验失败保持 MCU halted；不自动重新烧录或回滚。安装中途失败可能留下停止的服务，按日志修复后重新部署。

`check` 验证 SSH、两项服务处于 active、服务启动命令/时间和 runtime `/api/status`；这是只读基础检查，不代表实机喷流、PID、水下稳定性或摄像头持续帧率已经验收。检查 URL 当前使用已验证的 Pi 配置地址 `192.168.137.150:8080`，其他配置需同步修改或另行检查。

后续可以直接请求“运行维护脚本 build/check/all”，无需重新拼装长命令。`all` 在本机编译后按顺序执行，任一步失败即停止，已成功步骤不会自动撤销。
