# Copyright 2026 hanwen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Top-level AUV launch entry point.

Nodes are added here only after their packages have standalone tests.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bridge_config = os.path.join(
        get_package_share_directory('auv_bringup'), 'config', 'stm32_bridge.yaml'
    )
    camera_config = os.path.join(
        get_package_share_directory('auv_bringup'), 'config', 'cameras.yaml'
    )
    apriltag_config = os.path.join(
        get_package_share_directory('auv_bringup'), 'config', 'apriltag.yaml'
    )
    mapping_config = os.path.join(
        get_package_share_directory('auv_bringup'), 'config', 'mapping.yaml'
    )
    cones_config = os.path.join(
        get_package_share_directory('auv_bringup'), 'config', 'cones.yaml'
    )
    planning_config = os.path.join(
        get_package_share_directory('auv_bringup'), 'config', 'planning.yaml'
    )
    mission_config = os.path.join(
        get_package_share_directory('auv_bringup'), 'config', 'mission.yaml'
    )
    cucumber_config = os.path.join(
        get_package_share_directory('auv_bringup'), 'config', 'cucumber.yaml'
    )
    calibration_config = os.path.join(
        get_package_share_directory('auv_bringup'),
        'config',
        'down_camera_calibration.yaml',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_stm32_bridge',
                default_value='false',
                description='Start the safe STM32 serial transport bridge.',
            ),
            DeclareLaunchArgument(
                'start_cameras',
                default_value='false',
                description='Start the down and front camera acquisition nodes.',
            ),
            DeclareLaunchArgument(
                'start_apriltag',
                default_value='false',
                description='Start AprilTag detection on the down-camera topic.',
            ),
            DeclareLaunchArgument(
                'start_mapping',
                default_value='false',
                description='Start calibrated 3x3 semantic grid mapping.',
            ),
            DeclareLaunchArgument(
                'start_cones',
                default_value='false',
                description='Start 3x3 traffic-cone detection on rectified images.',
            ),
            DeclareLaunchArgument(
                'start_planning',
                default_value='false',
                description='Start deterministic 3x3 grid route planning.',
            ),
            DeclareLaunchArgument(
                'start_mission',
                default_value='false',
                description='Start the safety-gated autonomous mission FSM.',
            ),
            DeclareLaunchArgument(
                'start_cucumber',
                default_value='false',
                description='Start front-camera YOLO sea-cucumber detection.',
            ),
            LogInfo(
                msg=(
                    'AUV base workspace is ready. Only requested nodes are started; '
                    'the propulsion system remains DISARMED.'
                )
            ),
            Node(
                package='auv_stm32_bridge',
                executable='stm32_bridge_node',
                name='stm32_bridge',
                parameters=[bridge_config],
                condition=IfCondition(LaunchConfiguration('start_stm32_bridge')),
                output='screen',
            ),
            Node(
                package='auv_vision',
                executable='camera_node',
                name='auv_camera_down',
                parameters=[camera_config],
                condition=IfCondition(LaunchConfiguration('start_cameras')),
                output='screen',
            ),
            Node(
                package='auv_planning',
                executable='planner_node',
                name='auv_planner',
                parameters=[planning_config],
                condition=IfCondition(LaunchConfiguration('start_planning')),
                output='screen',
            ),
            Node(
                package='auv_mission',
                executable='mission_manager_node',
                name='auv_mission_manager',
                parameters=[mission_config],
                condition=IfCondition(LaunchConfiguration('start_mission')),
                output='screen',
            ),
            Node(
                package='auv_vision',
                executable='cucumber_detector_node',
                name='auv_cucumber_detector',
                parameters=[cucumber_config],
                condition=IfCondition(LaunchConfiguration('start_cucumber')),
                output='screen',
            ),
            Node(
                package='auv_vision',
                executable='camera_node',
                name='auv_camera_front',
                parameters=[camera_config],
                condition=IfCondition(LaunchConfiguration('start_cameras')),
                output='screen',
            ),
            Node(
                package='auv_vision',
                executable='apriltag_detector_node',
                name='auv_apriltag_detector',
                parameters=[apriltag_config, calibration_config],
                condition=IfCondition(LaunchConfiguration('start_apriltag')),
                output='screen',
            ),
            Node(
                package='auv_mapping',
                executable='semantic_mapper_node',
                name='auv_semantic_mapper',
                parameters=[mapping_config, calibration_config],
                condition=IfCondition(LaunchConfiguration('start_mapping')),
                output='screen',
            ),
            Node(
                package='auv_vision',
                executable='cone_detector_node',
                name='auv_cone_detector',
                parameters=[cones_config],
                condition=IfCondition(LaunchConfiguration('start_cones')),
                output='screen',
            ),
        ]
    )
