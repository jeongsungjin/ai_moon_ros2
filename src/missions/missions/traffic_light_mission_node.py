#!/usr/bin/env python3
"""신호등 미션 노드 (출발 green / 도착 red — 각 1회).

규정:
  - 출발: 준비 2분 후 신호등에 초록불 점등 → 인식 후 출발 (미인식 = 미션 실패)
  - 도착: 랩타임 종료 후 빨간불 점등 → 정지 (미인식 = +30s)
  - 출발/도착이 같은 신호등이므로 주행 중의 red/green 검출은 무시해야 함

내부 단계 (이 노드 안에만 존재하는 국소 상태):
  WAIT_GREEN : /motor_sign 으로 정지 유지 (speed=0, flag=True).
               green 이 green_stable_frames 연속 → 제어권 반납, DRIVING 으로
  DRIVING    : flag=False 만 발행 (차선 주행이 차를 몬다).
               출발 후 min_drive_time_sec 동안은 red 무시 (같은 신호등 오검출 방지)
  FINISH     : red 가 red_stable_frames 연속 → 정지 latch (speed=0, flag=True 계속)

입력: /yolo/green, /yolo/red (Bool, 프레임 단위 — yolo_detect 의 conf/크기 게이트 통과분)
출력: /motor_sign (drive_msgs/DriveCommand), /mission/traffic_state (String, 디버그)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from drive_msgs.msg import DriveCommand

WAIT_GREEN = 'WAIT_GREEN'
DRIVING = 'DRIVING'
FINISH = 'FINISH'


class TrafficLightMissionNode(Node):
    def __init__(self):
        super().__init__('traffic_light_mission')

        # enabled=false → 신호등 미션 비활성 (YOLO 없이 테스트 주행할 때).
        # 비활성 시 항상 flag=False + 상태 'DRIVING' 발행 (회전교차로 타이머는 launch 시점부터)
        self.declare_parameter('enabled', True)
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('green_stable_frames', 2)   # green 연속 N프레임 → 출발
        self.declare_parameter('red_stable_frames', 1)     # red 연속 N프레임 → 정지
        # 출발 직후 같은 신호등의 red 를 볼 수 있으므로 일정 시간 red 무시.
        # 트랙 1랩 예상 시간보다 확실히 짧게, 출발 신호등을 벗어날 시간보다 길게.
        self.declare_parameter('min_drive_time_sec', 20.0)

        self.enabled = bool(self.get_parameter('enabled').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.green_stable_frames = int(self.get_parameter('green_stable_frames').value)
        self.red_stable_frames = int(self.get_parameter('red_stable_frames').value)
        self.min_drive_time_sec = float(self.get_parameter('min_drive_time_sec').value)

        self.create_subscription(Bool, '/yolo/green', self.green_callback, 10)
        self.create_subscription(Bool, '/yolo/red', self.red_callback, 10)

        self.cmd_pub = self.create_publisher(DriveCommand, '/motor_sign', 10)
        self.state_pub = self.create_publisher(String, '/mission/traffic_state', 10)

        # 비활성 모드면 처음부터 DRIVING (정지 없이 주행 허용)
        self.state = DRIVING if not self.enabled else WAIT_GREEN
        self.green_count = 0
        self.red_count = 0
        self.drive_start_time = None

        self.timer = self.create_timer(1.0 / publish_hz, self.loop)

        self.get_logger().info(
            f'traffic_light_mission started: enabled={self.enabled}, state={self.state}, '
            f'green_stable={self.green_stable_frames}, red_stable={self.red_stable_frames}, '
            f'min_drive_time={self.min_drive_time_sec}s'
        )

    # ---------------- 인지 콜백 (연속 카운트만 갱신) ----------------
    def green_callback(self, msg: Bool):
        if not self.enabled or self.state != WAIT_GREEN:
            return
        self.green_count = self.green_count + 1 if msg.data else 0
        if self.green_count >= self.green_stable_frames:
            self.state = DRIVING
            self.drive_start_time = self.get_clock().now()
            self.get_logger().info('GREEN LIGHT — GO! (control released to lane driving)')

    def red_callback(self, msg: Bool):
        if not self.enabled or self.state != DRIVING:
            return
        # 출발 직후 red 무시 구간
        if self.drive_start_time is not None:
            elapsed = (self.get_clock().now() - self.drive_start_time).nanoseconds * 1e-9
            if elapsed < self.min_drive_time_sec:
                return
        self.red_count = self.red_count + 1 if msg.data else 0
        if self.red_count >= self.red_stable_frames:
            self.state = FINISH
            self.get_logger().info('RED LIGHT — FINISH STOP (latched)')

    # ---------------- 발행 루프 ----------------
    def loop(self):
        cmd = DriveCommand()
        if not self.enabled:
            # 미션 비활성: 절대 제어권을 잡지 않음 (테스트 주행용)
            cmd.flag = False
            self.cmd_pub.publish(cmd)
            self.state_pub.publish(String(data=DRIVING))
            return
        if self.state == WAIT_GREEN or self.state == FINISH:
            cmd.speed = 0.0
            cmd.angle = 0.0
            cmd.flag = True     # 제어권 요청: 정지 유지
        else:  # DRIVING
            cmd.speed = 0.0
            cmd.angle = 0.0
            cmd.flag = False    # 제어권 반납: 차선 주행
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
