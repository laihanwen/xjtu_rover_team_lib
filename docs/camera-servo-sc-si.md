# 摄像头舵机：SC 三档与 SI 拨轮

按操作者提供的 Servo.c / Servo.h 和已回零的机构移植，复用现有 AuvCameraServo 模块，没有复制附件中的主舵机输出。

| SC 档位 | 摄像头 | 主舵机 |
|---|---|---|
| 下（帧值 0） | 停止角度更新，保持当前位置 PWM | 不输出 |
| 中（帧值 1） | 已确认 SI 拨轮控制角度 | 不输出 |
| 上（帧值 2） | 停止角度更新，保持当前位置 PWM | 占位，不输出 |

摄像头使用工程现有 PC7 / TIM8_CH2，2 MHz 计数、20 ms 周期。上电预置 0°：CCR=3000；范围 −45°：2550 至 +10°：3100。SI 线性绝对位置映射，0→−45°，255→+10°；零度约在 209，拨轮中点不是零度。沿用 10 counts / 10 ms 渐变。未 ARM、遥控过期、SC 退出中档或拨轮未确认会取消剩余渐变并保持当前角度；不会强制舵机回零。主舵机的已有标定门保持 0，PWM 不启动，SC 上档不调用机械爪命令。

## 通道名称与链路

内部通道现在是 SI=MyRCKey[9]（RcData[5]），SC=MyRCKey[11]（RcData[7]）。速度独立为 SPEED_SELECTOR=10（RcData[6]），保持原来的速度逻辑。旧代码把内部 SI=14 用作定深按钮别名；现已移除这个名字复用，原 RcData[9] 按钮仍独立是 DEPTH_HOLD_SWITCH=14。SI 拨轮不会切换定深。

RC 帧仍为 11 字节。最后一个 RcData[10] 原保留字节现在表示摄像头拨轮确认：1 仅允许在 SC 中档、拨轮已配置时转发；0 禁止拨轮控制。PC 编码、trial_server 清洗、STM32 CRC 输入校验和执行端均处理此标记。非摄像头档不转发机械爪角度。必须一起更新 PC/Pi 脚本与固件；旧 Pi 桥会清掉拨轮字段。

ROV 网页的安全许可与 SC 独立：本次测量确认 SC 是 axis6；独立许可使用原有左肩 axis5 上档。网页速度由按钮覆盖帧 RcData[6]，不随 SC 改变。手动 ARM、回中、新鲜遥测与许可超时保护保留，不会自动 ARM。SC 下档只停止舵机控制，车辆停机使用 STOP 或撤销独立安全许可。

## 2026-10-07 映射实测完成

只读测试确认 SI=axis4。取每项结束前 25 个样本的中位数：向下端点 −0.80176，向上端点 +0.21136。SC=axis6，下档 −1，中档 0，上档 +0.99997。摘要保存在 `docs/reviews/camera-servo-mapping-20261007.json`，完整样本保存在 `build/auv/controller-servo-20261007-205731.json`。

radiomaster-pocket.json 已保存 dial_axis、dial_min、dial_max，网页控制端直接加载测量值，不需要额外命令行参数。超出端点的输入会限幅。更换模型时应重新测试，不能复用本次端点。

这次测量仅确认 PC 输入，没有驱动实机舵机，也没有刷写固件。首次实际方向验证应断开推进器电源或拆桨，检查向上对应 +10°、向下对应 −45°。当前摄像头控制仍需显式 ARM 与新鲜遥控；不要仅为测试拨轮就让带桨推进器通电。

## 验证与构建

```powershell
cmake --build build/firmware-host --config Release
ctest --test-dir build/firmware-host -C Release --output-on-failure
python -m unittest discover -s tools/rov -p test_rov.py
python -m unittest discover -s tools/rov -p test_trial.py
powershell -NoProfile -ExecutionPolicy Bypass -File tools/rov/Maintain-Rov.ps1 -Action build
```

通过 16 项固件主机测试，以及 6 项映射、9 项桥接测试；覆盖真实角度端点、零度、取消渐变、三档互斥、未确认拨轮、DISARM、CRC 输入与控制许可。Keil ROV 构建 0 错误、0 警告。代码尚未刷写或在实机验证；附件的 README 作为接口参考，用户指定的 SC/SI 和主舵机占位优先于其中 SB/SA 与主舵机动作。
