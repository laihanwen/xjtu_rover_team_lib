# 岸上水平校准、时间戳修复与视频帧率（2026-10-06）

本次为本地候选实现，未烧录或部署；没有发送ARM、推进器脉冲或采集真实水平基准。推进器混控矩阵、电机极性、PID增益和PWM限制未在本次改变。

## 最终行为

- 上电首帧只确定相对yaw原点；pitch/roll保持原始姿态，等待岸上手动基准。未校准禁止ARM。
- ROV页面确认岸上放平、DISARM、摇杆回中后，Pi观察至少2秒和18个不同STATUS样本。两轴各波动≤0.5°才发送0x08显式校准；STM32再次检查DISARM、序号及250ms内有效IMU，ACK返回后显示完成。离中、STOP、过期和不稳定采样取消。当前基准只保存在RAM，STM32重启必须重新在岸上校准。
- IMU采样、时间戳、心跳上下文、新鲜度判断在一致快照中进行；传输在临界区外完成，恢复原PRIMASK，不无条件开启中断。校准ACK有独立缓存，连续DISARM响应不覆盖它或完成提示。
- 下视CSI回传复用原始JPEG，前视USB只为每个采集帧编码一次。发布图像和时间戳使用同一锁。实时USB读取后去掉额外帧间等待；文件回放仍节流。
- 双路MJPEG长连接支持无叠加原图录像，HTTP整帧一次写入并启用TCP_NODELAY；预览不再等待视觉识别结果。保留JPEG快照和HLS兼容入口。慢客户端读取最新帧，不在应用中积压录像队列。
- 最多4条MJPEG流（双路预览+双路录像）和8个HTTP工作线程，额外流返回429，写超时/无帧超时2秒。不会阻塞实时控制线程。
- `runtime/config/pi-rov.yaml`为可跟踪的待部署Pi配置：CSI下视/USB前视320×240、目标30fps，默认关闭额外HLS编码；需要HLS时可显式启用libx264，仍须测CPU及后续分段解码。PC默认目标30fps，显示实际接收FPS，不复制源时间戳相同的帧。

## 已验证

- Keil完整固件构建0错误、0警告。Code=28612、RO-data=692，符合现有32KiB授权空间。HEX SHA256：`00b201a0658921c3941a3340adbe6ca31ee66c50f0cd9c95f7d4cfec9a073dd4`。
- 15项C主机测试通过，新增真实imu.c解析路径测试：首帧不校零、显式基准、±180°跨界、过期拒绝、uint32时间回绕、PRIMASK恢复；协议新增独立CRC黄金向量。
- 8项桥接测试通过，含2秒静止采样、重复STATUS拒绝、波动拒绝、缺少岸上确认不发校准、匹配ACK完成且始终不ARM。
- 8项录像测试通过，含分包MJPEG、单路单连接、超长/截断拒绝、双路分段、去重、来源验证、收尾和独立FFmpeg解码。Python语法检查通过，Git diff无空白错误。
- 从实际runtime源码提取HTTP处理片段，用官方cpp-httplib v0.11.4在Windows编译并运行模拟相机服务。30fps合成源下，下视/前视各收到37帧，约30.78/30.8fps；同时4条流时状态请求0.74ms，第5条返回429，录像独立解码通过。这是短时模拟测试，不是树莓派帧率或长期性能验收。
- 隔离8778页面检查：未连接时ARM/校准禁用，校准说明和实际录制FPS显示，任务页双摄MJPEG/兼容HLS入口存在；预览服务禁止全部POST，不连接实机。

## 尚未验证

树莓派192.168.137.150 SSH超时；pyOCD未检测到调试探针。没有完成固件烧录、Pi部署或真实双摄测速。Windows缺少Linux/OpenCV原生环境，完整runtime构建及Linux双摄断流CTest未执行；HTTP片段构建不能代替完整运行时构建。

设备恢复连接后应一起更新固件、trial_server.py/trial_protocol.py、PC驾驶台和video_recorder.py，以及runtime。Pi部署先保持serial.device为空、motion_commands_enabled=false、auto_start/auto_arm=false。按现有runtime安装流程构建/测试，再使用pi-rov.yaml作为相机配置；安装脚本会保留现有runtime.yaml，需要显式选择此配置，不能认为仓库配置已作用于机器。

首先保持DISARM在岸上完成手动校准，读取新STATUS bit3和近零pitch/roll；只读观察倾斜反馈。随后测10秒双摄down_hz/front_hz及PC received_fps、记录来源时间戳间隙、CPU和真实分段解码结果，再决定是否提高分辨率或启用HLS。既有矩阵方向及水中稳定性问题未在本次解决。
