#!/usr/bin/env python3
"""동적 장애물 미션 노드 (아루코 마커 = 장애물).

규정: 장애물 등장 시 정지, 퇴거 시 출발. 정지 중에는 스톱워치가 일시정지되므로
보수적으로(빨리 서고, 확실히 사라진 뒤 출발) 대응해도 시간 손해가 없다.
단, 등장 전에 미리 멈추거나 퇴거 전에 출발하면 미션 실패.

동작 (반응형 — 코스 위치 기억 없음):
  - /aruco/visible 이 appear_frames 연속 true  → 즉시 정지 (/motor_dynamic flag=True, speed=0)
  - /aruco/visible 이 clear_frames  연속 false → 정지 해제 (flag=False → 차선 주행 재개)
  - use_red_gate=true 면 /is_red(빨간 구간) 일 때만 정지 발동 — 다른 구간에서
    아루코가 스치듯 보여도 오발동하지 않게 하는 옵션 (기본 false: 무조건 반응)

우선순위: main_planner 에서 DYNAMIC 은 ROUNDABOUT 보다 높음
  → 회전교차로 탈출 조향 중이라도 장애물이 보이면 정지가 우선 (규정에 부합)
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, String

from drive_msgs.msg import DriveCommand


class DynamicObsMissionNode(Node):
    def __init__(self):
        super().__init__('dynamic_obs_mission')

        self.declare_parameter('enabled', True)
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('appear_frames', 2)   # 등장 판정 연속 프레임 (작을수록 즉각 정지)
        self.declare_parameter('clear_frames', 8)    # 퇴거 판정 연속 프레임 (깜빡임에 일찍 출발 방지)
        self.declare_parameter('use_red_gate', False)  # true: 빨간 구간에서만 정지 발동

        self.enabled = bool(self.get_parameter('enabled').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.appear_frames = int(self.get_parameter('appear_frames').value)
        self.clear_frames = int(self.get_parameter('clear_frames').value)
        self.use_red_gate = bool(self.get_parameter('use_red_gate').value)

        self.create_subscription(Bool, '/aruco/visible', self.aruco_callback, 10)
        self.create_subscription(Bool, '/is_red', self.is_red_callback, 10)

        self.cmd_pub = self.create_publisher(DriveCommand, '/motor_dynamic', 10)
        self.state_pub = self.create_publisher(String, '/mission/dynamic_state', 10)

        self.add_on_set_parameters_callback(self.on_param_change)

        self.stopped = False
        self.appear_count = 0
        self.clear_count = 0
        self.is_red = False

        self.timer = self.create_timer(1.0 / publish_hz, self.loop)

        self.get_logger().info(
            f'dynamic_obs_mission started: enabled={self.enabled}, '
            f'appear={self.appear_frames}f, clear={self.clear_frames}f, '
            f'red_gate={self.use_red_gate}'
        )

    # ---------------- 콜백 ----------------
    def aruco_callback(self, msg: Bool):
        if not self.enabled:
            return
        # red_gate 사용 시: 정지 "발동"만 빨간 구간으로 제한.
        # 이미 정지 중이면 게이트 무관하게 퇴거 판정은 계속한다 (멈춘 채 잠기는 것 방지)
        effective_visible = msg.data and (not self.use_red_gate or self.is_red or self.stopped)

        if effective_visible:
            self.appear_count += 1
            self.clear_count = 0
            if not self.stopped and self.appear_count >= self.appear_frames:
                self.stopped = True
                self.get_logger().info('ARUCO OBSTACLE — STOP')
        else:
            self.clear_count += 1
            self.appear_count = 0
            if self.stopped and self.clear_count >= self.clear_frames:
                self.stopped = False
                self.get_logger().info('obstacle cleared — RESUME lane driving')

    def is_red_callback(self, msg: Bool):
        self.is_red = msg.data

    def on_param_change(self, params):
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
                if not self.enabled:
                    self.stopped = False
            elif p.name == 'use_red_gate':
                self.use_red_gate = bool(p.value)
            elif p.name in ('appear_frames', 'clear_frames'):
                setattr(self, p.name, int(p.value))
            self.get_logger().info(f'param updated: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ---------------- 발행 루프 ----------------
    def loop(self):
        cmd = DriveCommand()
        cmd.speed = 0.0
        cmd.angle = 0.0
        cmd.flag = bool(self.enabled and self.stopped)
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(
            String(data='STOPPED' if cmd.flag else 'CLEAR')
        )


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObsMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
