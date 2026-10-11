# 下视水下白平衡临时测试

2026-10-11，用户授权在当前水下场景修改测试。目标 pi@192.168.137.150，OV5647，640×480。

执行前确认 Runtime serial/motion_enabled/armed/auto_start/auto_arm 均 false，机载记录关闭，PC 8767 录像 inactive。仅重启 auv-runtime 摄像头服务，未操作 ROV 桥、MCU、ARM 或机构。

独占 CSI 采样 10 秒获得 287 帧自动白平衡元数据，最后 60 帧 ColourGains 中位数为红 1.6023725、蓝 1.3067135。该窗口变化较小，未复现用户描述的颜色大幅跳变，不能据此确认根因。

最终通过 /run/systemd/system/auv-runtime.service.d/90-awb-test.conf 的 PATH 覆盖和 /usr/local/lib/auv-awb-test/rpicam-vid 包装器，向实际采集进程传入 --awbgains 1.602372,1.306713。已核对进程参数，723 帧元数据末 300 帧增益一致。验证后移除持续元数据写入参数，避免无限写盘。双摄恢复，连续五次状态下视帧数 99→222、帧龄 <0.02 秒，运动仍关闭。

测试过程中 /run 包装器未生效，服务私有绑定尝试造成下视无帧；已替换为最终可执行目录方案并验证恢复。这两次尝试不算成功测试。

这是固定当前自动增益的稳定性对照，未使用已知白/灰参照物测量真实色彩；场景和光照发生变化，前后照片不能作为校色对照。尚无用户验收。未改变 YAML、内参或前视色彩参数。重启 Pi 后 /run drop-in 消失，自动白平衡恢复；/usr/local/lib/auv-awb-test 文件不再被服务引用。

备份：Pi /run/auv-awb-test/runtime-before.yaml；本工作区 build/maintenance/awb-runtime-before.yaml、awb-report.json、awb-before.jpg、awb-after-*.jpg（不入提交）。

手动恢复自动白平衡（停止录像后执行；会中断双摄）：

```sh
sudo rm -f /run/systemd/system/auv-runtime.service.d/90-awb-test.conf
sudo systemctl daemon-reload
sudo systemctl restart auv-runtime
```

## 中性色参照微调

用户确认白/灰参照物已放好。中央参照板内部 ROI x=220..529、y=90..299（避开边框）初始 RGB 中位数 [188,220,201]、[186,219,199]、[196,226,206]，明显绿偏。经 sRGB 线性化比值估算和两次实机增益微调，最终固定红 1.930145、蓝 1.458917；中间 2.015489/1.491237 稍偏红，已回调。

最终五次 ROI RGB 中位数 [226,220,219]、[223,217,217]、[219,214,212]、[218,214,213]、[222,215,214]，通道差由约 30 缩小到 4–8。双摄新鲜，下视帧数 464→590，fault 为空。

仅证明该参照板在当前水下光照中更接近中性，不是色卡准确度验收或所有水深/光照的通用校色；仍有轻微红偏。长期跳变复现与用户视觉验收待确认。最终参数通过上述临时 drop-in 生效，重启 Pi 恢复自动；未更新源码和 YAML。采样照片和测量结果保存于 build/maintenance/awb-neutral-*，不入提交。
