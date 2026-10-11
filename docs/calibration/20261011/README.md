# 2026-10-11 新摄像头水下标定

用户确认录像全部为水下原图，棋盘 11×8 内角点、格长 15 mm。来源会话 20261011T094538_111539Z，前视 USB、下视 OV5647，两路实际 640×480；源文件哈希见 source-manifest.json。视频、选帧图和计算环境位于 build/calibration-20261011，不入源码提交。

下视：每 0.5 名义视频秒采样，539 帧中 28 帧检出完整角点并通过姿态去重；22 帧拟合、6 帧留出。5 参数模型 RMS 0.25035 px、留出 RMS 0.19953 px；固定 k3=0 模型 RMS 0.25041 px、留出 RMS 0.20116 px，焦距差约 0.2%，选择较简单的固定 k3 模型。按原始 K 生成映射有效像素约 87.2%，映射雅可比最小值约 0.996，未见折叠。现场预览使用原始 K，边缘会有无效黑边，不能以 alpha=0 裁切后的预览图代替 Runtime 图像空间。

前视：全部五段共 534 个抽样帧常规 SB 检测 0 完整棋盘；第一段经 CLAHE、3 倍放大和 EXHAUSTIVE/ACCURACY 再检测 122 帧仍 0。大多无棋盘，出现的棋盘远、暗、倾角大。未生成新的前视内参，不使用下视参数代替，也不缩放旧 320×240 参数到 640×480。前视保持当前高清原图试验状态。需要在前视下单独补录完整棋盘覆盖中心与四角、不同距离和倾角，避免裁切、拖影及过暗。

下视结果可供显示矫正，不意味着 navigation calibration_verified 可设 true。留出帧来自同次录像，尚无独立录像、水面标定或导航验收。AVI nominal_fps=30，前视实际接收速率较高，报告中的 seconds 为名义帧时间；精确录像时间以 frames.jsonl 为准。

## 实机接入

已备份并更新 /etc/auv-runtime/runtime.yaml 的下视字段，同时更新当前 /run/auv-front-resolution-test/runtime.yaml。备份为 /var/backups/auv-maintenance/calibration-20261011-1791710425978026817。下视矫正写入持久配置，Pi 重启后仍可使用。前视高清原图仍按先前临时方案运行，重启后的前视设置由 /etc 旧配置决定，未在此次改写。

PC 驾驶台 /preview 已设置 down=default、front=raw，录像未启动。实机状态下视 calibration_id=underwater-down-20261011-640x480、vision_image_space=rectified，两路帧数持续增加、帧龄 <0.07 秒、fault 为空。仅重启摄像头 Runtime，不操作 ROV 桥或 MCU；运动保持关闭。首次等待检查遇到服务启动时端口暂未就绪，已自动恢复原配置；修正等待重试后部署成功。

localization 仍关闭、旧定位内参未更新，导航标定未验收。无需通过开启定位或 movement 门控来验证显示矫正。未编译/运行 CTest，此次为离线计算、映射与图像检查，以及实际配置接入验证。
