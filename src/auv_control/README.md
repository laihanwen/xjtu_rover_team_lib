# auv_control

`route_executor_node` converts a validated 3×3 `PlannedRoute` into low-speed body-frame
motion targets for task 1. It never sends ARM. An operator must explicitly ARM after Mission
enters `VISIT_CONES`.

## Safety gates

Motion is published only when all conditions are true:

- `enable_route_motion:=true` was explicitly supplied at launch;
- Mission state is `VISIT_CONES`;
- route, grid pose and STM32 status are fresh and valid;
- STM32 is connected, armed, has no leak and reports no error flags;
- depth and yaw telemetry are finite.

Loss of pose/status, a fault, or route completion publishes zero surge/sway and requests DISARM.
Depth and yaw targets are latched when route motion begins. The default is dry-run mode.

## Dry-run

```fish
ros2 launch auv_bringup system.launch.py \
  start_stm32_bridge:=true \
  start_cameras:=true start_apriltag:=true \
  start_mapping:=true start_cones:=true start_planning:=true \
  start_mission:=true start_route_executor:=true
```

Inspect without moving hardware:

```fish
ros2 topic echo /mapping/grid_pose
ros2 topic echo /planning/route
ros2 topic echo /planning/execution_state
ros2 topic echo /planning/visited_cell
```

## Motion enable gate

Only after underwater calibration, fixed-vehicle sign checks, propeller-off failsafe tests and
operator approval, add `enable_route_motion:=true`. Verify the four grid-to-body matrix values in
`auv_bringup/config/control.yaml`; their signs depend on camera mounting and field orientation.
