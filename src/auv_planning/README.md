# auv_planning

P10 deterministic route planning for the structured competition grid. The node
subscribes to `/semantic_map`, extracts unvisited `circle_cone` and
`square_cone` cells, evaluates every target order, and joins four-neighbour A*
segments into the minimum-cost route.

The planner publishes grid coordinates, not motion commands. It never arms the
vehicle and does not publish `/cmd_vel`, `/cmd_depth`, or `/cmd_yaw`. Converting
grid steps into controlled vehicle motion belongs to P11 and later closed-loop
control work.

## Configure and run

Set the known competition start cell in
`auv_bringup/config/planning.yaml`. The defaults are `-1`, so an unknown start
produces an explicit invalid route instead of silently guessing.

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
ros2 launch auv_bringup system.launch.py \
  start_mapping:=true start_cones:=true start_planning:=true
```

Inspect the result:

```fish
ros2 topic echo /semantic_map
ros2 topic echo /planning/route
```

`valid=true` means every unvisited target is reachable. `path` contains every
grid step, including the start and targets. `total_cost` is the number of
four-neighbour moves. Equal-cost routes use row, column, then object type as a
stable tie-break. Unknown cells are traversable; only object types listed in
`blocked_object_types` are obstacles.

## Test

```fish
colcon test --packages-select auv_interfaces auv_planning auv_bringup
colcon test-result --verbose
```
