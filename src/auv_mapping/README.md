# auv_mapping

P8 structured 3×3 field mapper. The package detects the 3×3 grid frame (three
white edges plus one yellow edge), validates the two internal horizontal and
vertical divisions, orients the rectified view so the yellow edge becomes the
map's bottom edge, and publishes a deterministic row-major semantic map.

P9 consumes `/mapping/rectified_image`. This node remains the sole semantic-map
publisher and fuses fresh stable cone detections into `circle_cone` and
`square_cone` cells; stale or missing detections remain `unknown`.

## Topics

| Direction | Topic | Type |
|---|---|---|
| input | `/camera/down/image_raw` | `sensor_msgs/msg/Image` |
| input | `/cones/detections` | `auv_interfaces/msg/ConeDetectionArray` |
| input | `/planning/visited_cell` | `auv_interfaces/msg/GridCell` |
| output | `/semantic_map` | `auv_interfaces/msg/SemanticMap` |
| output | `/mapping/rectified_image` | `sensor_msgs/msg/Image` |
| output | `/mapping/grid_pose` | `auv_interfaces/msg/GridPose` |
| optional | `/mapping/debug_image` | `sensor_msgs/msg/Image` |

The rectified image is published only after the grid passes the configured
consecutive-frame stability check. Rows increase downward and columns increase
to the right in that image.

Production mode requires down-camera calibration. For synthetic or offline
development only, pass `require_calibration:=false` explicitly. Production
intrinsics and distortion coefficients belong in
`auv_bringup/config/down_camera_calibration.yaml`; that file is shared with
AprilTag pose estimation.

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
ros2 run auv_mapping semantic_mapper_node --ros-args \
  -p require_calibration:=false -p publish_debug_image:=true
```

The default pipeline applies CLAHE luminance enhancement, HSV yellow
segmentation combined with a white mask over low-saturation, high-value pixels
(so the three white edges plus one yellow edge form a closed quadrilateral
against the non-white pool floor), morphological filtering, convex quadrilateral
validation, yellow-edge orientation, homography, and edge support checks around
the four expected internal grid lines. All
thresholds are ROS parameters in `auv_bringup/config/mapping.yaml`. Tune them
from recorded underwater video rather than embedding pool-specific values in
source code.

## Validate

```fish
ros2 topic echo /semantic_map
ros2 topic hz /mapping/rectified_image
rqt_image_view /mapping/debug_image
```

`complete=true` requires three consecutive geometrically consistent frames, a completed temporal
cone scan, and at least `expected_cone_count` targets (competition default: four). A grid frame
without two horizontal and two vertical internal divisions is rejected, and the generated map is
oriented so the single yellow edge sits at the bottom.
