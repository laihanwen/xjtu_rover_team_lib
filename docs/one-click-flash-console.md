# 一键烧录独立 ROV 固件并打开原驾驶台

**当前流程已回退为独立模式。** `Flash-And-Console.cmd` 编译并烧录 `Copy_cup/Copy_cup.hex`，确认中立后启动 ROV 服务，打开原驾驶台根页面 `/`，双摄复用原连续 MJPEG 与录像代码。AUV 使用 `Build-AuvFirmware.ps1 -Profile AUV_TAG_DOCK` 和独立烧录入口。`AUV_ROV_DUAL` 不再是可选构建项，`switch_mode.py` 命令入口停用；切换 ROV/AUV 必须使用各自固件与匹配服务。

以下双模式记录为回退前历史，不作为当前烧录操作依据。

Windows 连接 Pi 与现有 pyOCD 可识别的烧录探针后，双击 `tools/auv/Flash-And-Console.cmd`。确认已停止任务、断开推进器动力或可靠固定机器人，输入 `SAFE_TO_FLASH`，随后自动执行。脚本不发送 ARM 或机构动作。

2026-10-09 入口修复：默认优先使用当前用户 `%LOCALAPPDATA%/xjtu-auv/console-venv` 独立 Python 环境，兼容 Codex Windows 应用的环境重定向；不存在时才尝试系统 Python，也可显式指定 `-Python`。相关 PowerShell 文件使用带 BOM 的 UTF-8，兼容双击调用的 Windows PowerShell 5.1。远端准备阶段优先加载本轮上传的协议辅助模块，避免 Pi 当前目录中的历史模块遮蔽。

实机联调同时修复模式选择 ACK 被重复 DISARM 应答覆盖的问题：选择已发出后只维持 heartbeat，继续要求匹配 ACK 和选择后的中立遥测，不放宽确认条件。2026-10-09 经用户物理安全确认后，AUV_ROV_DUAL 编译 0 错误/0 警告，固件 SHA256 为 `37b6c634ad866ee9a4f340509e10c1d50b86d51a2a2ac484617a5ad65736357e`，完整 32 KiB 读回校验通过，Pi 交接确认 ROV/DISARM，PC 驾驶台已启动，双摄快照均可用。当前手柄未连接，PC 遥控链尚未就绪；不代表水池运动验收，也未执行 ARM。首次整合执行在确认阶段中止，修复后分别完成交接与控制台启动；不将其描述为一键从头到尾无失败。

也可从仓库根目录执行：

```powershell
./tools/auv/Flash-And-Console.ps1
./tools/auv/Flash-And-Console.ps1 -CheckOnly
```

需要 Keil、STM32F4xx DFP 算法包和同一 Python 环境的 paramiko、pyserial、pyocd、intelhex、pygame。现有忽略目录 `build/debug-deps`、`build/deploy-ssh`、`build/joystick-deps` 可复用；否则使用独立虚拟环境安装并以 `-Python` 指定解释器，不安装到系统 Python。

默认 Pi 为 `pi@192.168.137.150`，串口 `/dev/serial0`，探针 `ATK 20190528`，PC 驾驶台端口 8767。可用 `-PiHost`、`-PiUser`、`-Device`、`-Probe`、`-Keil`、`-Pack`、`-Python`、`-WebPort` 覆盖。默认账户复用现有 DPAPI 凭据或 SSH 密钥；不在脚本保存密码。`-SafeToFlash` 表示调用者已完成物理安全确认，可跳过输入。

“最新”指当前工作区源码强制重新编译，不自动拉取远程提交。固定生成 `AUV_ROV_DUAL`，烧录前确认三条不同且递增的 MCU 遥测均为 DISARM、八路输出中立；烧录使用现有备份与完整 32 KiB 读回校验流程。完成后通过既有模式切换入口确认双模式能力、选择 ROV 并启动现有 Pi 摄像头/桥服务，最后打开 `/dashboard/`。控制台仍等待操作员显式 ARM。

Pi 须已安装并配置 ROV 服务、相机与串口。脚本不覆盖实测配置，不自动部署/编译新 Runtime，不启动 AUV 测试任务；AUV 切换与未验证标定仍受原安全门约束。仅更新用于交接的 Python 辅助文件。Pi 服务停止后失败则保持停止，不自动恢复；烧录成功不代表完成水池验收。跨模式录像仍是独立会话。

日志和固件哈希在 `build/one-click/`，烧录备份/校验日志由现有维护工具保存在 `build/maintenance/`。首次 SSH 连接沿用维护工具的主机密钥记录机制，后续密钥变化拒绝连接。`-CheckOnly` 仅检查本地依赖，不访问硬件；本轮仅语法检查和模拟测试，未实机执行。

当前烧录接口要求探针在 Windows 端被 pyOCD 识别（含透明 USB 转发）。仅有网络地址的独立烧录服务器不属于该接口，需按其真实协议另行适配，不能直接将 IP 当作探针 ID。
