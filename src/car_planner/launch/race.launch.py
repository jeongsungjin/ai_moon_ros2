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
            # 추론(1T ~250ms) 버스트가 차선 파이프라인을 선점하지 않게 코어 2,3 + 저우선순위로 격리
            # (통합 실측: 차선 23.6→26.1Hz, YOLO 2.5Hz 유지 — 2026-07-13)
            prefix='taskset -c 2,3 nice -n 5',
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
        # 미션 노드는 저부하(10~30Hz 상태머신)라 nice+5 로도 지연 체감 없음 —
        # 인지(camera/lane/aruco)·판단(planner)·구동(control)이 코어 경합에서 이기게 양보
        Node(
            package='missions', executable='traffic_light_mission_node',
            name='traffic_light_mission', output='screen',
            parameters=[
                params_file,
                # use_yolo:=false 면 green 대기 없이 바로 주행 (테스트용)
                {'enabled': ParameterValue(use_yolo, value_type=bool)},
            ],
            prefix='nice -n 5',
        ),
        Node(
            package='missions', executable='roundabout_mission_node',
            name='roundabout_mission', output='screen', parameters=[params_file],
            prefix='nice -n 5',
        ),
        Node(
            package='missions', executable='dynamic_obs_mission_node',
            name='dynamic_obs_mission', output='screen', parameters=[params_file],
            prefix='nice -n 5',
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
    ])
