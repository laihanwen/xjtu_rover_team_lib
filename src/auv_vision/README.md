# auv_vision

P6 provides one reusable `camera_node` executable. Bringup starts it twice as
`auv_camera_down` and `auv_camera_front`, publishing:

- `/camera/down/image_raw`
- `/camera/front/image_raw`

P7 adds `apriltag_detector_node`. It subscribes to the down-camera image and
publishes `auv_interfaces/msg/AprilTagDetectionArray` on
`/apriltag/detections`. The default family is `tag36h11`; `tag16h5`, `tag25h9`
and `tag36h10` are also supported.

P9 adds `cone_detector_node`. It consumes the stable 600×600 rectified grid,
segments configured cone colours, classifies circle/square contours, and
publishes temporally stable per-cell results on `/cones/detections`.

The `source` parameter accepts a V4L2 device index (`"0"`), a stable device path
such as `/dev/v4l/by-id/...`, a video file, or an OpenCV image-sequence pattern.
Live V4L2 sources reconnect after read failures. File and image-sequence inputs
stop at EOF unless `loop=true`.

Live capture runs on a dedicated thread, so a blocking V4L2 read cannot stall
the ROS executor. `frame_rate` requests the device/file rate, while
`publish_frame_rate` independently caps the ROS image topic. At startup the node
logs the format, resolution, and frame rate actually negotiated by OpenCV/V4L2.

Hardware device paths intentionally default to empty in
`auv_bringup/config/cameras.yaml`; identify the real cameras before setting them.
Resolution, frame rate and `MJPG` are requested settings and must be checked
against the selected camera's actual V4L2 capabilities.

For high-rate USB cameras, install the standard compressed transport plugin and
select the `compressed` transport in `rqt_image_view`:

```bash
sudo apt install ros-lyrical-image-transport-plugins
```

Publishing 640x480 BGR8 at 60 Hz requires roughly 55 MB/s before DDS overhead;
compressed transport is therefore recommended for display or transmission.
The tested Yahboom HFR mode is 640x480 MJPG at 120.101 Hz, capped to 60 Hz on
the ROS topic with `publish_frame_rate`.

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
Set the detector's `image_transport` parameter to `compressed` to keep
high-resolution input off the raw DDS path after installing the transport
plugin.
Without calibrated `camera_matrix` values the message still contains tag ID,
family, pixel center and four ordered corners, while `pose_valid` is false.
After calibration, set the 9 row-major camera-matrix values and distortion
coefficients; pose is then expressed in the image header's optical frame, using
metres and the configured `tag_size`.

For an offline repeatable test, point `camera_node.source` at a recorded video
or image sequence and keep the same detector topic. The detector publishes an
empty array for frames with no tag, which lets rosbag preserve negative results.

## Traffic-cone detection

Start mapping and cone detection together (and normally the down camera):

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
ros2 launch auv_bringup system.launch.py \
  start_cameras:=true start_mapping:=true start_cones:=true
```

Validate the stable result and optionally the annotated image:

```fish
ros2 topic echo /cones/detections
rqt_image_view /cones/debug_image
```

Set `publish_debug_image: true` in `config/cones.yaml` before launch to create
the debug publisher. The default red/orange HSV ranges are placeholders: record
real underwater footage, tune the YAML thresholds, and keep the C++ algorithm
unchanged. A detection must win three of the latest five frames by default;
small blobs and contours touching a cell margin are rejected.
