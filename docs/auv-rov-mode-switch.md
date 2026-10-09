# 单次烧录、AUV / ROV 会话切换

2026-10-09。新增独立 `AUV_ROV_DUAL` 固件，包含原 ROV 与自主控制路径。上电默认 ROV / DISARM；ARM 状态禁止直接换模式。原 ROV HEX 保留，不通过模式选择绕过未验证的硬件标定。

## 使用入口

Windows 编译（不烧录）：

```powershell
./tools/auv/Build-AuvFirmware.ps1 -Profile AUV_ROV_DUAL
```

产物 `firmware/stm32/MDK-ARM/AUV_ROV_DUAL/AUV_ROV_DUAL.hex`。另获实机烧录授权后使用 `Flash-AuvFirmware.ps1 -Profile AUV_ROV_DUAL`；后续选择 AUV/ROV 不需要重新烧录。改变 MCU 实测安全参数仍需要重新编译和烧录。

Pi 安装经过验证的 Runtime、ROV 桥和两套服务后，从源码目录显式执行：

```sh
sudo python3 tools/auv/switch_mode.py rov
sudo python3 tools/auv/switch_mode.py auv
```

默认 AUV 服务 `auv-tag-docking`、配置 `/etc/auv-runtime/pi-auv-tag-docking.yaml`；其他独立任务可通过 `--auv-unit`、`--auv-config` 指定。需要 pyserial、PyYAML。脚本检查服务实际配置路径、自动启动/ARM 均关闭。ROV Runtime 串口必须为空、运动关闭；AUV Runtime 串口必须匹配 `/dev/serial0`。

切换顺序：记录旧服务状态 → 停止并禁用竞争服务自动启动 → 独占 UART → 发送 DISARM → 三次新 STATUS 确认 DISARM 和八推输出归零 → SELECT_MODE 与对应 ACK → 确认模式 STATUS 和至少 200 ms 中立间隔 → 释放 UART → 启动对应服务。脚本不发送 ARM 或运动目标。

失败不恢复可能重新控制机器的旧服务；错误记录在 `/var/log/auv-runtime/mode-switches/`，操作员排查后显式重试。配置缺失、旧固件不支持双模式、输出非中立、NACK 或模式状态不匹配都会阻止交接。

ROV 启动 `auv-rov`（UART）和 `auv-runtime`（双摄）；AUV 只启动目标 Runtime（UART 和双摄）。切换会中断任务与录像会话，不是水下无缝接管。ROV 驾驶台应先停止遥控、释放 deadman，切换后重新连接、显式 ARM。AUV 同样需要 start/ARM 及全部安全门通过。

当前为会话级切换：MCU 重启回到 ROV/DISARM，脚本禁用相关服务自启，Pi 重启后需再次显式选择。正式自主开机需另行实现模式握手与一次性启动锁存，不能直接打开旧自动 ARM 配置。

## 固件与兼容性

- ROV 模式只接收 RC 控制，自主目标返回 UNSUPPORTED；AUV 忽略 RC 运动，只接收自主目标。ARM 区间锁定控制源，源失效 DISARM。
- 切换清空运动目标、RC 缓存、深度有效性和控制源。DISARM 控制循环清除保持状态；重新 ARM 至少等待 200 ms，重新取得当前模式的传感器/控制输入。
- AUV commissioning、深度零点、方向和输出增益仍使用实测配置。默认未验证，会拒绝 AUV ARM；双模式功能不等于自主运动已验收。
- 当前双模式自主最大深度 1.2 m、归一化平移目标每轴上限 0.2。保留 ROV 遥控挡位、姿态调平和相对定深参数，不修改混控矩阵或推进器符号。
- 固定 ROV、AUV_A0、AUV_TAG_DOCK 入口保留。固定固件收到模式选择返回 UNSUPPORTED；状态解码兼容旧固件，Runtime 与 ROV 桥对新固件检查模式再允许 ARM。
- 双模式 DEPTH 使用已有的 17 字节扩展包；原 ROV 桥继续读取 STATUS 深度并忽略 DEPTH，不受新增样本字段影响。ROV 深度仍是压力表读数，AUV 使用实测水面零点修正；不能沿用上一模式的读数语义。

## 本轮验证

- 双模式 Keil：0 错误、0 警告，Code 30756、RO 692 字节，仍在当前 32 KiB 烧录工具限制内，后续功能须继续检查容量。
- 固定 ROV 另目录 Keil 编译：0 错误、0 警告，未覆盖原 ROV HEX。
- 固件 CTest 17/17 通过；切换事务模拟 4/4 通过；独立 C++ 模式状态/协议兼容测试 1/1 通过。
- ROV Python 回归 36 项：35 通过、1 跳过（缺少独立视频解码依赖）。完整 Runtime 原生回归待 Pi 恢复连接。
- 未部署、烧录、ARM 或执行实机模式切换。真实控制权交接仍需先无推进器动力验收，再受控水池验证。
