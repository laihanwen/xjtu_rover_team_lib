# 前视分辨率临时测试

2026-10-11，用户要求先不调色差，并明确选择 640×480 原图、暂时关闭前视矫正。

实机 USB 摄像头枚举支持 MJPG 640×480 等模式，原服务为 320×240 矫正显示，旧 K/D 只适用于该尺寸。未缩放或套用旧内参。保留 /etc/auv-runtime/runtime.yaml，临时配置放在 /run/auv-front-resolution-test/runtime.yaml，由 /run/systemd/system/auv-runtime.service.d/91-front-resolution-test.conf 覆盖 ExecStart。前视 640×480、30fps 请求，preview_rectify=false、K/D 清空并标记 raw_display_only。

执行前检查 Runtime 串口、运动、ARM、自动启动、自动 ARM 和机载记录均关闭，PC 录像未启动。仅重启摄像头 Runtime，未操作控制桥、MCU 或执行运动。前视白平衡、曝光和其他颜色控制未修改；下视先前固定白平衡仍由 90-awb-test.conf 生效。

JPEG 三次实际解码尺寸为 640×480；前视帧数 363→404→444，下视 249→280→311，两路持续更新、帧龄 <0.03 秒、fault 为空。短时采样不代表长期帧率或稳定性验收。未运行编译/CTest（此次仅临时实机配置测试）。

重启 Pi 后 /run 覆盖消失，恢复原前视 320×240 矫正配置。立即恢复时，停止录像后执行（短暂中断双摄）：

```sh
sudo rm -f /run/systemd/system/auv-runtime.service.d/91-front-resolution-test.conf
sudo systemctl daemon-reload
sudo systemctl restart auv-runtime
```

此次恢复命令保留下视 90-awb-test.conf。高清矫正所需标定和用户显示验收未完成。
