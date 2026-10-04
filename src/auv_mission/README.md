# auv_mission

P11 safety-gated mission orchestration for the capabilities implemented through
P10. The current autonomous sequence is:

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
and is never requested automatically. P12–P14 will extend the FSM with sea cucumber,
grab, valve, return-home, and surface states after those capabilities exist.

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
