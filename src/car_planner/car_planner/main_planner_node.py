#!/usr/bin/env python3
"""메인 플래너 (ROS1 main_planner.py 포팅).

각 미션 노드가 발행하는 /motor_* (drive_msgs/DriveCommand) 를 구독하고,
flag 기반 우선순위로 MODE 를 결정한 뒤 해당 미션의 speed/angle 을
최종 차량 명령으로 변환한다.

변경점 (JetRacer / 카메라 온리):
  - LiDAR(/obstacles, /raw_obstacles_rubbercone) 구독 및 감속 로직 제거
  - 출력: AckermannDriveStamped -> control_msgs/Control (/control)
    * steering = -angle * steering_gain  (원본: -angle * 0.002 rad 와 동일 구조)
    * throttle = speed * throttle_gain
  - 우선순위/모드 구조는 원본 그대로 (미션 추가 시 /motor_* 토픽만 늘리면 됨)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, Int32, String

from control_msgs.msg import Control
from drive_msgs.msg import DriveCommand

# 우선순위 순서 (앞일수록 높음). LANE 은 기본 모드.
MISSION_PRIORITY = [
    ('SIGN', '/motor_sign'),
    ('RABACON', '/motor_rabacon'),
    ('STATIC', '/motor_static'),
    ('DYNAMIC', '/motor_dynamic'),
    ('ROUNDABOUT', '/motor_roundabout'),
    ('TUNNEL', '/motor_tunnel'),
    ('PARKING', '/motor_parking'),
]


class MissionCommand:
    __slots__ = ('speed', 'angle', 'flag')

    def __init__(self):
        self.speed = 0.0
        self.angle = 0.0
        self.flag = False


class MainPlannerNode(Node):
    def __init__(self):
        super().__init__('main_planner')

        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('loop_hz', 30.0)
        self.declare_parameter('version', 'safe')
        # 조향 변환: 원본 steering_angle(rad) = -angle * 0.002, 조향 최대각 ±0.34rad
        # -> percent(-1~1) 환산 기본값 0.002/0.34 ≈ 0.006 근처에서 트랙 튜닝
        self.declare_parameter('steering_gain', 0.003)
        self.declare_parameter('invert_steering', False)
        # 스로틀 변환: 원본 speed(0.2~0.5) -> percent
        self.declare_parameter('throttle_gain', 1.0)
        self.declare_parameter('max_throttle', 0.6)
        # 완주 판정용 모터 합산 (원본 white_cnt 로직 유지)
        self.declare_parameter('white_cnt_threshold', 220)

        self.control_topic = str(self.get_parameter('control_topic').value)
        self.steering_gain = float(self.get_parameter('steering_gain').value)
        self.invert_steering = bool(self.get_parameter('invert_steering').value)
        self.throttle_gain = float(self.get_parameter('throttle_gain').value)
        self.max_throttle = float(self.get_parameter('max_throttle').value)
        self.white_cnt_threshold = int(self.get_parameter('white_cnt_threshold').value)

        # 미션 명령 저장소
        self.ctrl_lane = MissionCommand()
        self.missions = {name: MissionCommand() for name, _ in MISSION_PRIORITY}

        # 구독: 차선 (기본 모드)
        self.create_subscription(DriveCommand, '/motor_lane', self.lane_callback, 10)
        # 구독: 미션들 (클로저로 콜백 생성)
        for name, topic in MISSION_PRIORITY:
            self.create_subscription(
                DriveCommand, topic, self.make_mission_callback(name), 10
            )
        self.create_subscription(Int32, '/white_cnt', self.white_cnt_callback, 10)

        # 발행
        self.control_pub = self.create_publisher(Control, self.control_topic, 10)
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        self.motor_sum_pub = self.create_publisher(Float32, '/sum_of_motor', 10)

        self.mode = 'LANE'
        self.motor = 0.0
        self.steer = 0.0
        self.white_cnt = 0
        self.motor_sum = 0.0

        # ros2 param set 으로 게인류 실시간 튜닝
        self.add_on_set_parameters_callback(self.on_param_change)

        loop_hz = float(self.get_parameter('loop_hz').value)
        self.timer = self.create_timer(1.0 / loop_hz, self.loop)

        self.get_logger().info(
            f'main_planner started: control={self.control_topic}, '
            f'steering_gain={self.steering_gain}, throttle_gain={self.throttle_gain}'
        )

    def on_param_change(self, params):
        for p in params:
            if p.name == 'steering_gain':
                self.steering_gain = float(p.value)
            elif p.name == 'throttle_gain':
                self.throttle_gain = float(p.value)
            elif p.name == 'invert_steering':
                self.invert_steering = bool(p.value)
            elif p.name == 'max_throttle':
                self.max_throttle = float(p.value)
            self.get_logger().info(f'param updated: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ---------------- 콜백 ----------------
    def make_mission_callback(self, name):
        def cb(msg: DriveCommand):
            cmd = self.missions[name]
            cmd.speed = msg.speed
            cmd.angle = msg.angle
            cmd.flag = msg.flag
        return cb

    def lane_callback(self, msg: DriveCommand):
        self.ctrl_lane.speed = msg.speed
        self.ctrl_lane.angle = msg.angle
        self.ctrl_lane.flag = msg.flag

    def white_cnt_callback(self, msg: Int32):
        self.white_cnt = msg.data

    # ---------------- 메인 루프 ----------------
    def loop(self):
        # MODE 판별: flag 가 켜진 최고 우선순위 미션
        self.mode = 'LANE'
        selected = self.ctrl_lane
        for name, _ in MISSION_PRIORITY:
            if self.missions[name].flag:
                self.mode = name
                selected = self.missions[name]
                break

        self.mode_pub.publish(String(data=self.mode))

        self.motor = selected.speed
        self.steer = selected.angle

        # 완주(횡단보도 정지) 이후 모터 합산 (원본 로직 유지)
        if self.white_cnt >= self.white_cnt_threshold:
            self.motor_sum += self.motor
            self.motor_sum_pub.publish(Float32(data=float(self.motor_sum)))
        else:
            self.motor_sum_pub.publish(Float32(data=0.0))

        self.get_logger().info(
            f'MODE: {self.mode} | SPEED: {self.motor:.2f} | STEER: {self.steer:.1f}',
            throttle_duration_sec=0.5,
        )

        self.publish_ctrl_cmd(self.motor, self.steer)

    def publish_ctrl_cmd(self, motor_msg, servo_msg):
        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        steering = -float(servo_msg) * self.steering_gain
        if self.invert_steering:
            steering = -steering
        msg.steering = float(np.clip(steering, -1.0, 1.0))

        throttle = float(motor_msg) * self.throttle_gain
        msg.throttle = float(np.clip(throttle, -self.max_throttle, self.max_throttle))

        self.control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MainPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
