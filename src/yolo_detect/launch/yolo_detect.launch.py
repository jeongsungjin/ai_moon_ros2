"""YOLO 인지 노드 단독 launch.

차선 주행 스택과 함께 쓸 때:
  터미널1: ros2 launch car_planner lane_drive.launch.py
  터미널2: ros2 launch yolo_detect yolo_detect.launch.py
(camera 노드는 lane_drive 쪽에서 이미 실행 중이어야 함.
 카메라 없이 단독 테스트하려면 with_camera:=true)
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

    with_camera = LaunchConfiguration('with_camera')
    model_path = LaunchConfiguration('model_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'with_camera', default_value='false',
            description='camera_node 도 함께 실행 (lane_drive 없이 단독 테스트용)',
        ),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='모델 경로 override (기본: params.yaml 값)',
        ),

        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(with_camera),
        ),
        Node(
            package='yolo_detect',
            executable='yolo_detect_node',
            name='yolo_detect_node',
            output='screen',
            parameters=[params_file],
            # model_path 인자를 줬을 때만 override 하고 싶지만 launch 조건 분기가
            # 복잡해지므로, override 는 ros2 run + -p model_path:=... 를 사용 권장
        ),
    ])
