"""차선 주행만 실행하는 최소 launch (ROS1 lane_drive.launch 대체).

camera -> lane_detection -> main_planner -> control
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('car_planner'), 'config', 'params.yaml'
    )

    use_control = LaunchConfiguration('use_control')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_control', default_value='true',
            description='하드웨어 구동 노드 실행 여부 (개발 PC 에서는 false)',
        ),

        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='lane_detection',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='car_planner',
            executable='main_planner_node',
            name='main_planner',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_control),
        ),
    ])
