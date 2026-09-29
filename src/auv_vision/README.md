# auv_vision

P6 provides one reusable `camera_node` executable. Bringup starts it twice as
`auv_camera_down` and `auv_camera_front`, publishing:

- `/camera/down/image_raw`
- `/camera/front/image_raw`

The `source` parameter accepts a V4L2 device index (`"0"`), a stable device path
such as `/dev/v4l/by-id/...`, a video file, or an OpenCV image-sequence pattern.
Live V4L2 sources reconnect after read failures. File and image-sequence inputs
stop at EOF unless `loop=true`.

Hardware device paths intentionally default to empty in
`auv_bringup/config/cameras.yaml`; identify the real cameras before setting them.
Resolution, frame rate and `MJPG` are requested settings and must be checked
against the selected camera's actual V4L2 capabilities.

## Run

```fish
source tools/setup_dev.fish
ros2 launch auv_bringup system.launch.py start_cameras:=true
```

For an offline recording:

```fish
ros2 run auv_vision camera_node --ros-args \
  -r __node:=auv_camera_down \
  -p source:=/absolute/path/to/recording.mp4 \
  -p topic:=/camera/down/image_raw \
  -p frame_id:=camera_down_optical_frame \
  -p loop:=true
```

Validate without a GUI:

```fish
ros2 topic hz /camera/down/image_raw
ros2 topic echo /camera/down/image_raw --field header --once
```
