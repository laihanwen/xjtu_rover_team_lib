# AUV 网线调试监视台

监视台位于 `runtime/web/index.html`，与机载 Runtime 同源。浏览器不发送 ARM、运动或任务控制命令。运行配置显示 `debug/autonomous` 和 mission profile；目前 STM32 未提供固件身份回传，不能据此证明固件模式。

## 启用与连接

先更新机载 Runtime 二进制和网页资源。仅替换网页无法获得本次增加的遥测字段。以下命令在树莓派仓库目录执行，兼容 fish：

```fish
cmake -S . -B build-lightweight -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-lightweight -j 3
ctest --test-dir build-lightweight --output-on-failure
sudo cmake --install build-lightweight
```

在实际服务 ExecStart 指向的 YAML 中启用网页。建议网线调试通过 SSH 转发，网页绑定回环地址，避免依赖旧的静态 IP。A1/A2 模板原本关闭 web，必须编辑已部署配置：

```yaml
operation:
  mode: debug
  auto_start: false
  auto_arm: false
web:
  enabled: true
  bind: 127.0.0.1
  port: 8080
  assets: /usr/local/share/auv-runtime/web
```

这是修改已有配置项的示例，不是完整 YAML。前期观测保留 `motion_commands_enabled: false`，串口和摄像头填写实机设备；不要将未标定项目改成 verified。默认 A2 模板没有开启完整执行，需要先完成 A1/A2 交付文档中的实机标定。首次调试电机断电或拆桨。

核对选中的 AUV 服务与配置：

```fish
systemctl show auv-task-one.service -p ExecStart
/usr/local/bin/auv_runtime --check-config /etc/auv-runtime/pi-auv-task-one.yaml
```

确认该服务配置为 debug、自动启动/ARM 关闭后，在车辆 DISARM 时重启对应服务加载新配置。A1 服务是 `auv-observation.service`，配置路径以其 ExecStart 为准。仓库未自动安装或启用这两个新服务，不要仅凭服务名字判断部署完成。

在电脑终端使用实际树莓派用户与网线 IP（替换尖括号内容）：

```fish
ssh -N -L 18080:127.0.0.1:8080 <Pi用户>@<Pi网线IP>
```

保持 SSH 窗口运行，浏览器打开 `http://127.0.0.1:18080/`。另开 SSH 终端执行 `auvctl status` 等调试命令。若选择直接网线访问，把 web.bind 改成实机有线地址，浏览器使用 `http://<Pi网线IP>:8080/`；地址必须已存在，否则 HTTP 绑定失败。

## 页面读法

- 顶部区分运行方式、任务配置、STM32 ARM 与运动开关。安全门显示当前后端判断，不能替代硬件标定。
- 默认 2 Hz JPEG 快照，支持实时 MJPEG、已启用的 HLS 和关闭预览。切换只改变预览，不改变机载采集/记录。页面使用本地 hls.js，无外网依赖。
- 下视采集 Hz、视觉处理 Hz 与新鲜度分别显示。前视未启用时隐藏；相机断流画面变暗。
- A2 九宫格使用新鲜表面绝对位置；旧的水下行列不能冒充表面定位。规划路线与机载确认访问分别显示。黄色边始终显示在规范地图底部。
- 深度趋势保留最近 60 秒浏览器采样；断线留空，重新启动的 run_id 会清空曲线。姿态单位为度，速度为 m/s。
- 定位与记录区显示相对定位会话、故障原因、记录目录和 run_id。浏览器状态变化列表上限 100 条，是客户端观察记录，完整事件仍在机载文件中。
- 导出 JSON 包含最后接收时刻、过期标记、状态、定位与浏览器事件；断线后仍可导出历史快照。剪贴板不允许时显示命令供手动复制。
- 断网时整块旧数据变暗，ARM 改为“未知”；车辆本身可能仍在运行。网线连接不构成安全停止机制。

## 验证范围

离线浏览器 smoke test 模拟 A2 状态，覆盖表面位置选择、DISARM、前视隐藏、HLS 未启用、断线、导出与 390 px 移动布局。需要现有 Playwright 环境与浏览器：

```fish
node runtime/test/dashboard_smoke.cjs
# 使用系统 Edge 时：
env AUV_BROWSER_CHANNEL=msedge node runtime/test/dashboard_smoke.cjs
```

本次通过 Windows 系统 Edge 无头验证与截图检查；Runtime 在隔离 Linux 头文件环境做含 HTTP 分支的语法检查。尚未在树莓派部署或验证实机双摄、网线吞吐与浏览器长时间运行。
