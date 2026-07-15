#!/usr/bin/env python3
"""아웃코스 갈림길 방향 선택 미션.

갈림길 전에는 기존 슬라이딩 윈도우의 BOTH 모드를 유지한다. YOLO가 학습된
left/right 표지판을 안정 검출해 /traffic_sign 으로 발행하면 해당 방향을 한 번
선택하고, 이후에는 /lane_topic 에 LEFT 또는 RIGHT를 계속 발행해 선택한 차선을
끝까지 추종한다.

이 노드는 조향/속도 명령을 직접 만들지 않는다. 따라서 main_planner의 우선순위나
/motor_* 인터페이스를 변경하지 않고 lane_detection의 기존 제어기를 그대로 쓴다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


WAIT_START = 'WAIT_START'
ARMED = 'ARMED'
SELECTED_LEFT = 'SELECTED_LEFT'
SELECTED_RIGHT = 'SELECTED_RIGHT'


class ForkMissionNode(Node):
    def __init__(self):
        super().__init__('outcourse_fork_mission')

        self.declare_parameter('enabled', True)
        self.declare_parameter('publish_hz', 10.0)
        # 출발 직후 카메라에 남아 있는 표지판/화면 오검출을 막는 선택 유예 시간.
        self.declare_parameter('arm_delay_sec', 0.0)
        self.declare_parameter('default_lane_side', 'BOTH')
        self.declare_parameter('left_class_name', 'left')
        self.declare_parameter('right_class_name', 'right')

        self.enabled = bool(self.get_parameter('enabled').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.arm_delay_sec = float(self.get_parameter('arm_delay_sec').value)
        self.default_lane_side = self._valid_side(
            str(self.get_parameter('default_lane_side').value), allow_both=True)
        self.left_class_name = str(self.get_parameter('left_class_name').value).lower()
        self.right_class_name = str(self.get_parameter('right_class_name').value).lower()

        self.create_subscription(String, '/traffic_sign', self.sign_callback, 10)
        self.create_subscription(
            String, '/mission/traffic_state', self.traffic_state_callback, 10)
        self.lane_side_pub = self.create_publisher(String, '/lane_topic', 10)
        self.state_pub = self.create_publisher(String, '/mission/fork_state', 10)
        self.direction_pub = self.create_publisher(String, '/mission/fork_direction', 10)

        self.state = WAIT_START if self.enabled else ARMED
        self.drive_start_time = None
        self.selected_side = None
        self._last_announced_side = None

        self.timer = self.create_timer(1.0 / publish_hz, self.loop)
        self.get_logger().info(
            f'fork_mission started: enabled={self.enabled}, '
            f'default={self.default_lane_side}, arm_delay={self.arm_delay_sec}s'
        )

    @staticmethod
    def _valid_side(side, allow_both=False):
        side = side.upper()
        valid = ('LEFT', 'RIGHT', 'BOTH') if allow_both else ('LEFT', 'RIGHT')
        return side if side in valid else 'BOTH'

    def traffic_state_callback(self, msg: String):
        if not self.enabled:
            return
        if msg.data == 'DRIVING' and self.drive_start_time is None:
            self.drive_start_time = self.get_clock().now()
            self.state = ARMED
            self.get_logger().info('start detected — fork sign detection armed')

    def _selection_allowed(self):
        if not self.enabled or self.selected_side is not None or self.drive_start_time is None:
            return False
        elapsed = (self.get_clock().now() - self.drive_start_time).nanoseconds * 1e-9
        return elapsed >= self.arm_delay_sec

    def sign_callback(self, msg: String):
        if not self._selection_allowed():
            return

        detected = msg.data.strip().lower()
        if detected == self.left_class_name:
            self.select('LEFT')
        elif detected == self.right_class_name:
            self.select('RIGHT')

    def select(self, side):
        """첫 유효 방향만 영구 래치한다. 후속 오검출은 선택을 바꾸지 않는다."""
        if self.selected_side is not None:
            return
        self.selected_side = self._valid_side(side)
        self.state = SELECTED_LEFT if self.selected_side == 'LEFT' else SELECTED_RIGHT
        self.get_logger().warning(
            f'FORK SELECTED: {self.selected_side} — lane selection latched'
        )

    def loop(self):
        side = self.selected_side or self.default_lane_side
        self.lane_side_pub.publish(String(data=side))
        self.state_pub.publish(String(data=self.state))
        self.direction_pub.publish(String(data=self.selected_side or 'NONE'))

        if side != self._last_announced_side:
            self.get_logger().info(f'lane side -> {side}')
            self._last_announced_side = side


def main(args=None):
    rclpy.init(args=args)
    node = ForkMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

