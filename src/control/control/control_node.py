#!/usr/bin/env python3
"""차량 구동 노드 (D-Racer-Kit control_node 포팅).

/control (control_msgs/Control, steering/throttle percent) 을 구독하여
PCA9685 로 조향 서보 / ESC 를 구동한다.

- /e_stop (std_msgs/Bool) 로 비상 정지 가능
- command_timeout 동안 명령이 없으면 스로틀 0 (안전 정지)
"""

import os
import signal

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool

from control_msgs.msg import Control
from control.racer import Racer, ServoCalib, EscCalib


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('command_hz', 30.0)
        self.declare_parameter('command_timeout', 0.5)   # sec, 0 이면 미사용
        self.declare_parameter('i2c_bus', 1)             # JetRacer(Jetson)=1, D3-G=3
        self.declare_parameter('pca9685_addr', 0x40)
        self.declare_parameter('steering_channel', 0)
        self.declare_parameter('throttle_channel', 1)
        self.declare_parameter('steer_trim', 0.0)
        self.declare_parameter('max_throttle', 0.6)      # 안전 상한
        # 스로틀 증가 슬루 (틱당 최대 증가량, 0=끔). 정지→가속의 돌진 전류가 배터리 전압을
        # 순간 강하시켜 ESC 저전압 컷 깜빡임(모터 1~2초 사망) 유발 — bag 실측 2026-07-15.
        # 0.012@30Hz = 0→0.21 을 약 0.6초에 램프. 감속/정지는 즉시 (안전 우선, 제한 없음)
        self.declare_parameter('throttle_slew_up', 0.012)
        self.declare_parameter('servo_center_us', 1500)
        self.declare_parameter('servo_span_us', 500)
        self.declare_parameter('esc_neutral_us', 1500)
        self.declare_parameter('esc_fwd_us', 2000)
        self.declare_parameter('esc_rev_us', 1000)
        self.declare_parameter('dry_run', False)         # true 면 하드웨어 미접속(로그만)

        control_topic = str(self.get_parameter('control_topic').value)
        command_hz = float(self.get_parameter('command_hz').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.steer_trim = float(self.get_parameter('steer_trim').value)
        self.max_throttle = float(self.get_parameter('max_throttle').value)
        self.throttle_slew_up = float(self.get_parameter('throttle_slew_up').value)
        self._throttle_applied = 0.0
        self.dry_run = bool(self.get_parameter('dry_run').value)

        if command_hz <= 0.0:
            raise ValueError('command_hz must be greater than 0')

        if self.dry_run:
            self.racer = None
            self.get_logger().warning('dry_run=True: hardware is NOT driven')
        else:
            self.racer = Racer(
                i2c_bus=int(self.get_parameter('i2c_bus').value),
                pca9685_addr=int(self.get_parameter('pca9685_addr').value),
                steering_channel=int(self.get_parameter('steering_channel').value),
                throttle_channel=int(self.get_parameter('throttle_channel').value),
                steering=ServoCalib(
                    center_us=int(self.get_parameter('servo_center_us').value),
                    span_us=int(self.get_parameter('servo_span_us').value),
                ),
                esc=EscCalib(
                    neutral_us=int(self.get_parameter('esc_neutral_us').value),
                    fwd_us=int(self.get_parameter('esc_fwd_us').value),
                    rev_us=int(self.get_parameter('esc_rev_us').value),
                ),
            )

        self.steering = self.steer_trim
        self.throttle = 0.0
        self.e_stop_active = False
        self.last_cmd_time = self.get_clock().now()

        # 시작 즉시 중립 — 직전 프로세스가 강제종료로 남긴 폭주 PWM 을 덮어씀
        if self.racer is not None:
            self.racer.stop()

        self.create_subscription(Control, control_topic, self.control_callback, 10)
        self.create_subscription(Bool, '/e_stop', self.e_stop_callback, 10)

        # 수동 조종 우선권: /control_manual 이 신선하면(manual_timeout 내) 자율 명령 무시
        self.declare_parameter('manual_timeout', 0.4)
        self.manual_timeout = float(self.get_parameter('manual_timeout').value)
        self.last_manual_time = None
        self.create_subscription(Control, '/control_manual', self.manual_callback, 10)

        # ros2 param set 으로 트림/스로틀 상한 실시간 튜닝
        self.add_on_set_parameters_callback(self.on_param_change)

        self.timer = self.create_timer(1.0 / command_hz, self.timer_callback)

        self.get_logger().info(
            f'control_node started: topic={control_topic}, hz={command_hz}, '
            f'steer_trim={self.steer_trim}, max_throttle={self.max_throttle}, '
            f'dry_run={self.dry_run}'
        )

    def timer_callback(self):
        if self.e_stop_active:
            self.apply_actuation(self.steering, 0.0)
            return

        # 명령 타임아웃 시 안전 정지
        if self.command_timeout > 0.0:
            elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if elapsed > self.command_timeout:
                self.apply_actuation(self.steering, 0.0)
                return

        self.apply_actuation(self.steering, self.throttle)

    def apply_actuation(self, steering, throttle):
        throttle = max(-self.max_throttle, min(self.max_throttle, float(throttle)))
        # 증가만 슬루 제한 (돌진 전류로 인한 ESC 저전압 컷 방지). 감소/정지는 즉시.
        if self.throttle_slew_up > 0.0:
            if throttle > self._throttle_applied:
                throttle = min(throttle, self._throttle_applied + self.throttle_slew_up)
            self._throttle_applied = throttle
        if self.racer is None:
            return
        self.racer.set_steering_percent(float(steering))
        self.racer.set_throttle_percent(throttle)

    def _manual_active(self):
        if self.last_manual_time is None:
            return False
        age = (self.get_clock().now() - self.last_manual_time).nanoseconds * 1e-9
        return age < self.manual_timeout

    def control_callback(self, msg: Control):
        if self.e_stop_active:
            return
        if self._manual_active():   # 수동 조종 중에는 자율 명령 무시
            return
        self.steering = float(msg.steering) + self.steer_trim
        self.throttle = float(msg.throttle)
        self.last_cmd_time = self.get_clock().now()

    def manual_callback(self, msg: Control):
        if self.e_stop_active:      # E-STOP 은 수동보다도 우선
            return
        self.steering = float(msg.steering) + self.steer_trim
        self.throttle = float(msg.throttle)
        self.last_manual_time = self.get_clock().now()
        self.last_cmd_time = self.last_manual_time

    def on_param_change(self, params):
        for p in params:
            if p.name == 'steer_trim':
                self.steer_trim = float(p.value)
            elif p.name == 'max_throttle':
                self.max_throttle = float(p.value)
            self.get_logger().info(f'param updated: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def e_stop_callback(self, msg: Bool):
        if bool(msg.data):
            self.e_stop_active = True
            self.throttle = 0.0
            self.steering = self.steer_trim   # 바퀴 자동 정렬 (직진 위치)
            self.apply_actuation(self.steering, 0.0)
            self.get_logger().warning('E-STOP engaged. Steering aligned, throttle blocked.')
        else:
            if self.e_stop_active:
                self.get_logger().info('E-STOP released.')
            self.e_stop_active = False

    def destroy_node(self):
        try:
            if self.racer is not None:
                self.racer.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    # SIGHUP(SSH 끊김)/SIGTERM(강제종료 1단계)에도 모터 중립을 보장하고 종료
    # (기본 동작은 정리 코드 없이 즉사 → 마지막 PWM 이 남아 폭주)
    def _neutral_and_exit(signum, frame):
        try:
            if node.racer is not None:
                node.racer.stop()
        finally:
            os._exit(0)   # rclpy 대기 루프에 안 막히는 즉시 종료 (중립은 이미 기록됨)

    signal.signal(signal.SIGHUP, _neutral_and_exit)
    signal.signal(signal.SIGTERM, _neutral_and_exit)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 경로 어디서든 모터 중립 보장 (마지막 PWM 이 하드웨어에 남는 폭주 방지)
        try:
            if node.racer is not None:
                node.racer.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
