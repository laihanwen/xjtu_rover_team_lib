# auv_vision

P6 provides one reusable `camera_node` executable. Bringup starts it twice as
`auv_camera_down` and `auv_camera_front`, publishing:

- `/camera/down/image_raw`
- `/camera/front/image_raw`

P7 adds `apriltag_detector_node`. It subscribes to the down-camera image and
publishes `auv_interfaces/msg/AprilTagDetectionArray` on
`/apriltag/detections`. The default family is `tag36h11`; `tag16h5`, `tag25h9`
and `tag36h10` are also supported.

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

## AprilTag detection

Start the down camera and detector together:

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
ros2 launch auv_bringup system.launch.py start_cameras:=true start_apriltag:=true
```

Verify detections and optionally enable the annotated image:

```fish
ros2 topic echo /apriltag/detections
ros2 topic hz /apriltag/debug_image
```

The debug publisher is created at startup, so set `publish_debug_image: true`
in `src/auv_bringup/config/apriltag.yaml` before launching when it is needed.
Without calibrated `camera_matrix` values the message still contains tag ID,
family, pixel center and four ordered corners, while `pose_valid` is false.
After calibration, set the 9 row-major camera-matrix values and distortion
coefficients; pose is then expressed in the image header's optical frame, using
metres and the configured `tag_size`.

For an offline repeatable test, point `camera_node.source` at a recorded video
or image sequence and keep the same detector topic. The detector publishes an
empty array for frames with no tag, which lets rosbag preserve negative results.
