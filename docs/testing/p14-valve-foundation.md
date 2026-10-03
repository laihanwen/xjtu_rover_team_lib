# P14 转盘视觉基础环境验收

当前阶段只建立转盘视觉输入和可调试接口，不控制推进器或机械执行器。

## 已实现边界

- 输入：`/camera/front/image_raw`，BGR8 图像。
- 输出：`/valve/detection`，每帧都会发布，包括未检测到目标的负结果。
- 调试：`/valve/debug_image`，显示圆盘候选和把手轴线。
- 参数：`src/auv_bringup/config/valve.yaml`。
- 启动开关：`start_valve:=true`，默认关闭。
- 检测限频：默认 15 Hz，不降低前摄像头话题本身的帧率。

`handle_angle_rad` 是无方向直线角，范围为 `[-pi/2, pi/2)`。当前结果不能用于证明
已经旋转 180°，也没有接入 Mission FSM 或 STM32 执行器。

## 构建与自动测试

```fish
cd /home/hanwen/auv
source /opt/ros/lyrical/setup.fish
colcon build --symlink-install --packages-select auv_interfaces auv_vision auv_bringup
source install/setup.fish
colcon test --packages-select auv_interfaces auv_vision auv_bringup
colcon test-result --verbose
```

自动测试覆盖空图、非法配置、合成圆盘/把手以及 ROS 图像输入到检测和调试图发布。

## 摄像头实时验收

先在 `src/auv_bringup/config/cameras.yaml` 中填写前摄像头的稳定设备路径，然后执行：

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
ros2 launch auv_bringup system.launch.py \
  start_cameras:=true start_valve:=true
```

另开终端：

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
ros2 topic hz /camera/front/image_raw
ros2 topic echo /valve/detection
rqt_image_view /valve/debug_image
```

验收时记录至少三段素材：正视转盘、明显倾斜转盘、无转盘背景。确认无目标时
`detected=false`，目标进入画面后中心点和圆半径随目标移动，并保存 rosbag 供后续调参。

## 下一步进入闭环前的门槛

1. 提供比赛转盘实物照片、尺寸和旋转规则。
2. 完成前摄像头水下标定与去畸变。
3. 使用真实录像统计误检率、漏检率和角度抖动。
4. 增加多帧稳定与转角累计，验证旋转不少于 180°。
5. 明确执行器型号、反馈方式和 STM32 可用通道后，再定义控制协议。
