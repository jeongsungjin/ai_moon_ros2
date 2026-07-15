"""SEA:ME 아웃코스 레이스 풀스택 launch.

인코스 race.launch.py와 센서/신호등/장애물/플래너/제어 스택은 공유하지만,
roundabout_mission은 실행하지 않는다. 대신 fork_mission이 YOLO left/right
표지판을 받아 lane_detection의 LEFT/RIGHT 추종 기준을 한 번 선택해 래치한다.

사용:
  ros2 launch car_planner outcourse.launch.py
  ros2 launch car_planner outcourse.launch.py use_control:=false
  ros2 launch car_planner outcourse.launch.py use_web_viewer:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('car_planner')
    params_file = os.path.join(share, 'config', 'params.yaml')
    outcourse_params_file = os.path.join(share, 'config', 'outcourse_params.yaml')

    use_control = LaunchConfiguration('use_control')
    use_yolo = LaunchConfiguration('use_yolo')
    use_web_viewer = LaunchConfiguration('use_web_viewer')

    common_args = [
        DeclareLaunchArgument(
            'use_control', default_value='true',
            description='하드웨어 구동 노드 실행 여부',
        ),
        DeclareLaunchArgument(
            'use_yolo', default_value='true',
            description='YOLO와 신호등/갈림길 미션 실행 여부',
        ),
        DeclareLaunchArgument(
            'use_web_viewer', default_value='true',
            description='웹 뷰어(:8080) 실행 여부',
        ),
    ]

    return LaunchDescription(common_args + [
        Node(
            package='camera', executable='camera_node', name='camera_node',
            output='screen', parameters=[params_file, outcourse_params_file],
        ),
        Node(
            package='outcourse_lane_detection', executable='lane_detection_node',
            name='lane_detection_node', output='screen',
            parameters=[params_file, outcourse_params_file],
        ),
        Node(
            package='outcourse_yolo_detect', executable='yolo_detect_node',
            name='yolo_detect_node', output='screen',
            parameters=[params_file, outcourse_params_file],
            condition=IfCondition(use_yolo),
            prefix='taskset -c 2,3 nice -n 5',
        ),
        Node(
            package='cv_detect', executable='aruco_detect_node',
            name='aruco_detect_node', output='screen',
            parameters=[params_file, outcourse_params_file],
        ),
        Node(
            package='cv_detect', executable='red_zone_node',
            name='red_zone_node', output='screen',
            parameters=[params_file, outcourse_params_file],
        ),
        Node(
            package='missions', executable='traffic_light_mission_node',
            name='traffic_light_mission', output='screen',
            parameters=[
                params_file,
                outcourse_params_file,
                {'enabled': ParameterValue(use_yolo, value_type=bool)},
            ],
            prefix='nice -n 5',
        ),
        Node(
            package='outcourse_missions', executable='fork_mission_node',
            name='fork_mission', output='screen',
            parameters=[
                outcourse_params_file,
                {'enabled': ParameterValue(use_yolo, value_type=bool)},
            ],
            prefix='nice -n 5',
        ),
        Node(
            package='missions', executable='dynamic_obs_mission_node',
            name='dynamic_obs_mission', output='screen',
            parameters=[params_file, outcourse_params_file],
            prefix='nice -n 5',
        ),
        Node(
            package='car_planner', executable='main_planner_node',
            name='main_planner', output='screen',
            parameters=[params_file, outcourse_params_file],
        ),
        Node(
            package='car_planner', executable='web_viewer_node',
            name='web_viewer', output='screen',
            parameters=[params_file, outcourse_params_file],
            condition=IfCondition(use_web_viewer),
            prefix='nice -n 10',
        ),
        TimerAction(period=5.0, actions=[
            Node(
                package='control', executable='control_node', name='control_node',
                output='screen', parameters=[params_file, outcourse_params_file],
                condition=IfCondition(use_control),
            ),
        ]),
    ])
