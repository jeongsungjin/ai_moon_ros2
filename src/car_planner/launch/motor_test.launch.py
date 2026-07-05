"""모터/서보 단독 구동 테스트 launch (인지/플래너 없이 하드웨어만).

control_node (PCA9685 구동) + motor_test_node (테스트 패턴 발행)

사용 예:
  # 조향 서보만 좌우 스윕 (바퀴 안 돎 — 조향 방향/범위 확인)
  ros2 launch car_planner motor_test.launch.py

  # 구동 모터 테스트 (차 들어올리고!)
  ros2 launch car_planner motor_test.launch.py mode:=throttle throttle:=0.2

  # 조향 + 구동 동시
  ros2 launch car_planner motor_test.launch.py mode:=both throttle:=0.15 duration:=10.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('car_planner'), 'config', 'params.yaml'
    )

    mode = LaunchConfiguration('mode')
    throttle = LaunchConfiguration('throttle')
    duration = LaunchConfiguration('duration')
    sweep_period = LaunchConfiguration('sweep_period')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='steering',
            description="테스트 모드: 'steering'(서보 스윕) | 'throttle'(구동) | 'both'",
        ),
        DeclareLaunchArgument(
            'throttle', default_value='0.2',
            description='throttle/both 모드에서의 스로틀 percent (max 0.5)',
        ),
        DeclareLaunchArgument(
            'duration', default_value='5.0',
            description='테스트 시간 (초), 이후 중립 유지',
        ),
        DeclareLaunchArgument(
            'sweep_period', default_value='2.0',
            description='조향 스윕 1주기 시간 (초)',
        ),

        # 하드웨어 구동 (i2c_bus, 서보/ESC 캘리브레이션은 params.yaml 공유)
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[params_file],
        ),
        # 테스트 패턴 발행
        Node(
            package='control',
            executable='motor_test_node',
            name='motor_test_node',
            output='screen',
            parameters=[{
                'mode': mode,
                'throttle': ParameterValue(throttle, value_type=float),
                'duration_sec': ParameterValue(duration, value_type=float),
                'sweep_period_sec': ParameterValue(sweep_period, value_type=float),
            }],
        ),
    ])
