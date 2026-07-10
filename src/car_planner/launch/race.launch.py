"""SEA:ME 인코스 레이스 풀스택 launch.

파이프라인:
  camera ─┬─ lane_detection ──────────────┐
          ├─ yolo_detect (green/red) ─────┤
          ├─ aruco_detect (동적장애물) ────┤→ 미션 노드들 → main_planner → control
          └─ red_zone (/is_red) ──────────┘

미션:
  - traffic_light_mission: green 대기 출발(1회) / 도착 red 정지(1회)
  - roundabout_mission: 진입→1회전→탈출 (⚠️ params 의 enabled 로 켜야 동작)
  - 동적장애물 미션 노드는 장애물 형태 조사 후 추가 예정
    (인지 계층 /aruco/* + /is_red 는 이미 발행 중)

사용:
  ros2 launch car_planner race.launch.py                  # 실차
  ros2 launch car_planner race.launch.py use_control:=false  # 인지/미션만 (모터 없이)
  ros2 launch car_planner race.launch.py use_yolo:=false     # 모델 없을 때
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('car_planner'), 'config', 'params.yaml'
    )

    use_control = LaunchConfiguration('use_control')
    use_yolo = LaunchConfiguration('use_yolo')
    use_web_viewer = LaunchConfiguration('use_web_viewer')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_control', default_value='true',
            description='하드웨어 구동 노드 실행 여부 (개발 PC 에서는 false)',
        ),
        DeclareLaunchArgument(
            'use_yolo', default_value='true',
            description='YOLO 인지 실행 여부. false 면 신호등 미션도 자동 비활성 '
                        '(green 대기 없이 바로 주행 — 테스트용)',
        ),
        DeclareLaunchArgument(
            'use_web_viewer', default_value='true',
            description='웹 뷰어(:8080) 실행 여부',
        ),

        # ---------------- 센서 ----------------
        Node(
            package='camera', executable='camera_node', name='camera_node',
            output='screen', parameters=[params_file],
        ),

        # ---------------- 인지 ----------------
        Node(
            package='lane_detection', executable='lane_detection_node',
            name='lane_detection_node', output='screen', parameters=[params_file],
        ),
        Node(
            package='yolo_detect', executable='yolo_detect_node',
            name='yolo_detect_node', output='screen', parameters=[params_file],
            condition=IfCondition(use_yolo),
        ),
        Node(
            package='cv_detect', executable='aruco_detect_node',
            name='aruco_detect_node', output='screen', parameters=[params_file],
        ),
        Node(
            package='cv_detect', executable='red_zone_node',
            name='red_zone_node', output='screen', parameters=[params_file],
        ),

        # ---------------- 미션 ----------------
        Node(
            package='missions', executable='traffic_light_mission_node',
            name='traffic_light_mission', output='screen',
            parameters=[
                params_file,
                # use_yolo:=false 면 green 대기 없이 바로 주행 (테스트용)
                {'enabled': ParameterValue(use_yolo, value_type=bool)},
            ],
        ),
        Node(
            package='missions', executable='roundabout_mission_node',
            name='roundabout_mission', output='screen', parameters=[params_file],
        ),
        Node(
            package='missions', executable='dynamic_obs_mission_node',
            name='dynamic_obs_mission', output='screen', parameters=[params_file],
        ),

        # ---------------- 판단/제어 ----------------
        Node(
            package='car_planner', executable='main_planner_node',
            name='main_planner', output='screen', parameters=[params_file],
        ),
        Node(
            package='control', executable='control_node', name='control_node',
            output='screen', parameters=[params_file],
            condition=IfCondition(use_control),
        ),

        # ---------------- 모니터링 ----------------
        Node(
            package='car_planner', executable='web_viewer_node',
            name='web_viewer_node', output='screen',
            parameters=[{
                'topics': [
                    '/camera/image/compressed',
                    '/lane_detection/image/debug',
                    '/yolo/image/debug',
                    '/aruco/image/debug',
                    '/red_zone/image/debug',
                ],
            }],
            condition=IfCondition(use_web_viewer),
        ),
    ])
