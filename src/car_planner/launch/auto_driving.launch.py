"""전체 자율주행 스택 launch (ROS1 main_planner.launch 대체).

camera -> lane_detection / cv_detect -> main_planner -> control
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

    use_cv_detect = LaunchConfiguration('use_cv_detect')
    use_control = LaunchConfiguration('use_control')

    return LaunchDescription([
        DeclareLaunchArgument(
            # 새 대회에 표지판/라바콘 미션 없음 — 기본 OFF (CPU ~0.6코어 절약, 필요시 true)
            'use_cv_detect', default_value='false',
            description='표지판/라바콘 검출 노드 실행 여부',
        ),
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
            package='cv_detect',
            executable='parking_sign_node',
            name='parking_sign_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_cv_detect),
        ),
        Node(
            package='cv_detect',
            executable='rubbercone_node',
            name='rubbercone_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_cv_detect),
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
