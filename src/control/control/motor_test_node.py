#!/usr/bin/env python3
"""모터/서보 단독 구동 테스트 노드 (D-Racer-Kit manual_driving 참고).

인지/플래너 없이 /control (control_msgs/Control) 로 테스트 패턴을 발행한다.
control_node 와 함께 띄우면 실제 하드웨어가 움직인다 (motor_test.launch.py).

테스트 모드:
  - steering : 조향 서보만 사인파 스윕 (스로틀 0 — 바퀴 안 굴러감, 방향 확인용)
  - throttle : 조향 중립 + 일정 스로틀 (구동 모터/ESC 확인용, 차 들어올리고!)
  - both     : 조향 스윕 + 일정 스로틀

duration_sec 경과 후에는 중립(0,0)을 계속 발행한다 (Ctrl+C 로 종료).
"""

import math

import rclpy
from rclpy.node import Node

from control_msgs.msg import Control

# 테스트 노드 자체 안전 상한 (control_node 의 max_throttle 과 별개)
TEST_THROTTLE_LIMIT = 0.5


class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')

        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('mode', 'steering')        # 'steering' | 'throttle' | 'both'
        self.declare_parameter('throttle', 0.2)           # throttle/both 모드에서 사용
        self.declare_parameter('steering', 0.0)           # throttle 모드에서 고정 조향값
        self.declare_parameter('steering_amplitude', 0.8) # 스윕 진폭 (서보 한계 보호)
        self.declare_parameter('sweep_period_sec', 2.0)   # 스윕 1주기 시간
        self.declare_parameter('duration_sec', 5.0)       # 테스트 시간
        self.declare_parameter('publish_hz', 30.0)

        control_topic = str(self.get_parameter('control_topic').value)
        self.mode = str(self.get_parameter('mode').value)
        self.throttle = float(self.get_parameter('throttle').value)
        self.steering = float(self.get_parameter('steering').value)
        self.steering_amplitude = float(self.get_parameter('steering_amplitude').value)
        self.sweep_period_sec = float(self.get_parameter('sweep_period_sec').value)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        if self.mode not in ('steering', 'throttle', 'both'):
            raise ValueError("mode must be one of: 'steering', 'throttle', 'both'")
        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be greater than 0')

        # 안전 클램프
        if abs(self.throttle) > TEST_THROTTLE_LIMIT:
            self.get_logger().warning(
                f'throttle {self.throttle} clamped to ±{TEST_THROTTLE_LIMIT} (test safety limit)'
            )
            self.throttle = math.copysign(TEST_THROTTLE_LIMIT, self.throttle)
        self.steering_amplitude = max(0.0, min(1.0, self.steering_amplitude))

        self.control_pub = self.create_publisher(Control, control_topic, 10)

        self.start_time = self.get_clock().now()
        self.done = False
        self.timer = self.create_timer(1.0 / publish_hz, self.timer_callback)

        self.get_logger().info(
            f'motor_test started: mode={self.mode}, throttle={self.throttle}, '
            f'duration={self.duration_sec}s, sweep_period={self.sweep_period_sec}s'
        )
        if self.mode in ('throttle', 'both'):
            self.get_logger().warning('구동 모터가 돌아갑니다 — 차를 들어올리거나 안전 확보!')

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # 테스트 종료: 중립 유지 (control_node 타임아웃과 이중 안전)
        if elapsed >= self.duration_sec:
            self.publish_control(0.0, 0.0)
            if not self.done:
                self.done = True
                self.get_logger().info('Motor test finished — neutral published. (Ctrl+C to exit)')
            return

        # 조향값 계산
        if self.mode in ('steering', 'both'):
            steering = self.steering_amplitude * math.sin(
                2.0 * math.pi * elapsed / self.sweep_period_sec
            )
        else:
            steering = self.steering

        # 스로틀값 계산
        throttle = self.throttle if self.mode in ('throttle', 'both') else 0.0

        self.publish_control(steering, throttle)
        self.get_logger().info(
            f'[{elapsed:4.1f}s] steering={steering:+.2f}, throttle={throttle:+.2f}',
            throttle_duration_sec=0.5,
        )

    def publish_control(self, steering, throttle):
        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.steering = float(steering)
        msg.throttle = float(throttle)
        self.control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 직전 중립 명령
        try:
            node.publish_control(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
