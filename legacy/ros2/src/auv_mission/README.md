# auv_mission

Safety-gated mission orchestration shared by the ROS and lightweight runtimes.
The default `task_one` sequence is:

```text
INIT → SELF_CHECK → SEARCH_APRILTAG → BUILD_MAP
     → PLAN_CONES → VISIT_CONES → COMPLETE
```

Every transition is published on `/mission/state` and written to the ROS log.
Each active phase has a timeout. Leak detection, STM32 errors, status loss,
disconnect, or an unexpected ARM state sends the mission to `FAULT`.

Mission itself does not publish `/cmd_vel`, `/cmd_depth`, `/cmd_yaw`, or an ARM request. The
separate, default-disabled `auv_control/route_executor_node` executes the frozen route and marks
cone cells visited. ARM is allowed only during `VISIT_CONES` when configured, remains explicit,
and is never requested automatically by the ROS node.

The core FSM also supports the complete competition sequence when
`MissionFsmConfig::full_mission` is enabled:

```text
VISIT_CONES → SEARCH_CUCUMBER → ALIGN_CUCUMBER → GRAB
→ TRANSPORT → RELEASE → SEARCH_VALVE → ALIGN_VALVE → ROTATE_VALVE
→ RETURN_HOME → SURFACE → COMPLETE
```

These transitions require explicit confirmations from perception, gripper,
transport, valve, home and surface inputs. Enabling the core sequence alone does
not synthesize those inputs or make unfinished hardware operational. The
lightweight runtime currently consumes real gripper telemetry and issues guarded
close/open/stop commands; front-camera alignment, transport, valve actuation,
return-home and surface controllers remain integration work.

## Run

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
ros2 launch auv_bringup system.launch.py start_mission:=true
```

Production configuration uses `auto_start: false`. Start explicitly:

```fish
ros2 service call /mission/command auv_interfaces/srv/MissionCommand \
  "{command: 1}"
ros2 topic echo /mission/state
```

Commands are `START=1`, `PAUSE=2`, `RESUME=3`, `ABORT=4`, and `RESET=5`.
`RESET` is accepted only from `COMPLETE`, `FAULT`, or `ABORTED`.

## Safe virtual test

The virtual driver uses isolated `/virtual/*` topics and never starts the STM32
bridge or publishes propulsion commands:

```fish
ros2 run auv_mission mission_virtual_test
```

Success ends with:

```text
VIRTUAL_MISSION_RESULT=PASS
```
