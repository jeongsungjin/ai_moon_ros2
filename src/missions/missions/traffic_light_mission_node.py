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

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32, String

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
        # 빨간 구간 통과 전 red 무시의 시간 백업: 구간 인지 실패 시에도 이 시간 뒤엔 red 유효
        self.declare_parameter('red_zone_fallback_sec', 90.0)
        # green 발광 패스트패스 (2026-07-15): YOLO 가 이 램프에 약해(실측 conf 0.25) 카메라
        # 프레임에서 발광 초록 픽셀을 직독 — WAIT_GREEN 에서만 작동, 출발 즉시 비활성.
        # ROI 하단 제한(y>=glow_roi_y0)으로 상단 대형 스크린 오발 차단 (신호등은 바닥 8cm)
        self.declare_parameter('green_glow_fast', True)
        self.declare_parameter('glow_roi_y0', 200)
        self.declare_parameter('glow_min_px', 35)     # 실물 램프 실측 181px @ 근거리
        # 기동 직후 green 무시 시간: control(모터)이 5초 지연 기동이라, 그 전에 green 을 봐서
        # DRIVING 이 시작되면 회전교차로 타이머만 흐르고 차는 제자리 → 진입 창 소진 사고
        # (2026-07-15 bag 실측). 스택이 완전히 뜬 뒤에만 출발 인정.
        self.declare_parameter('startup_ignore_sec', 6.5)
        # 목표 랩 수: red 정지가 이 횟수에 도달해야 최종 FINISH latch.
        # 그 전의 red 는 "랩 완료 정지" — green 재점등 시 다시 출발 (WAIT_GREEN 복귀)
        self.declare_parameter('lap_count', 4)

        self.enabled = bool(self.get_parameter('enabled').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.green_stable_frames = int(self.get_parameter('green_stable_frames').value)
        self.red_stable_frames = int(self.get_parameter('red_stable_frames').value)
        self.min_drive_time_sec = float(self.get_parameter('min_drive_time_sec').value)
        self.red_zone_fallback_sec = float(self.get_parameter('red_zone_fallback_sec').value)
        self.green_glow_fast = bool(self.get_parameter('green_glow_fast').value)
        self.startup_ignore_sec = float(self.get_parameter('startup_ignore_sec').value)
        self._node_start_time = self.get_clock().now()
        self.glow_roi_y0 = int(self.get_parameter('glow_roi_y0').value)
        self.glow_min_px = int(self.get_parameter('glow_min_px').value)
        self._glow_skip = 0
        if self.green_glow_fast:
            self.create_subscription(CompressedImage, '/camera/image/compressed',
                                     self.glow_callback, qos_profile_sensor_data)
        self.lap_count = int(self.get_parameter('lap_count').value)

        self.create_subscription(Bool, '/yolo/green', self.green_callback, 10)
        self.create_subscription(Bool, '/yolo/red', self.red_callback, 10)
        # 빨간 구간 게이트 (2026-07-15, 7/13 사고 예비 카드 투입): 빨간 바닥 위에 있는 동안
        # red 신호 무시 — 바닥/근처 붉은 물체 오인으로 인한 조기 FINISH 차단.
        # 결승 신호등은 빨간 구간 밖이므로 정상 정지에는 영향 없음.
        self.create_subscription(Bool, '/is_red', self.is_red_callback, 10)
        self.in_red_zone = False
        # 결승 red 는 빨간 구간(아루코) 통과 이후에만 유효 (트랙 순서: ...→빨간구간→결승등).
        # 구간 인지가 실패해도 red_zone_fallback_sec 경과 시 무조건 유효 (미정지 벌점 방지 백업)
        self.passed_red_zone = False

        self.cmd_pub = self.create_publisher(DriveCommand, '/motor_sign', 10)
        self.state_pub = self.create_publisher(String, '/mission/traffic_state', 10)
        self.lap_pub = self.create_publisher(Int32, '/mission/lap', 10)

        # 비활성 모드면 처음부터 DRIVING (정지 없이 주행 허용)
        self.state = DRIVING if not self.enabled else WAIT_GREEN
        self.green_count = 0
        self.red_count = 0
        self.drive_start_time = None
        self.laps_done = 0      # red 정지(랩 완료) 누적

        self.timer = self.create_timer(1.0 / publish_hz, self.loop)

        self.get_logger().info(
            f'traffic_light_mission started: enabled={self.enabled}, state={self.state}, '
            f'green_stable={self.green_stable_frames}, red_stable={self.red_stable_frames}, '
            f'min_drive_time={self.min_drive_time_sec}s'
        )

    # ---------------- 인지 콜백 (연속 카운트만 갱신) ----------------
    def _stack_ready(self):
        """기동 직후 유예 — control 지연 기동(5s)보다 길게 기다린 뒤에만 출발 인정."""
        up = (self.get_clock().now() - self._node_start_time).nanoseconds * 1e-9
        return up >= self.startup_ignore_sec

    def green_callback(self, msg: Bool):
        if not self.enabled or self.state != WAIT_GREEN or not self._stack_ready():
            return
        self.green_count = self.green_count + 1 if msg.data else 0
        if self.green_count >= self.green_stable_frames:
            self.state = DRIVING
            self.drive_start_time = self.get_clock().now()
            self.get_logger().info('GREEN LIGHT — GO! (control released to lane driving)')

    def glow_callback(self, msg: CompressedImage):
        """green 발광 직독 — WAIT_GREEN 전용 즉발 출발 (카메라 25Hz 중 1/3 처리 ≈ 8Hz)."""
        if (not self.enabled or self.state != WAIT_GREEN
                or not self.green_glow_fast or not self._stack_ready()):
            return
        self._glow_skip = (self._glow_skip + 1) % 3
        if self._glow_skip:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        roi = img[self.glow_roi_y0:]
        g = roi[:, :, 1].astype(np.int16)
        glow = (g > 150) & (g > roi[:, :, 2].astype(np.int16) + 50) \
            & (g > roi[:, :, 0].astype(np.int16) + 50)
        n_glow = int(np.count_nonzero(glow))
        if n_glow >= self.glow_min_px:
            self.state = DRIVING
            self.drive_start_time = self.get_clock().now()
            self.get_logger().info(f'GREEN GLOW({n_glow}px) — 즉발 출발!')

    def is_red_callback(self, msg: Bool):
        # 구간 진입 후 이탈(True→False) = 아루코 구간 통과 완료 → 결승 red 즉시 유효
        # 단 주행 중(DRIVING)에만 인정 — 출발 대기 중 시야에 걸린 구간으로 조기 개방되는 것 방지
        # (2026-07-15 리허설: 출발선 정지 상태에서 ENTER/EXIT 이 잡혀 게이트가 미리 열렸음)
        if (self.state == DRIVING and self.in_red_zone and not msg.data
                and not self.passed_red_zone):
            self.passed_red_zone = True
            self.get_logger().info('빨간 구간 통과 — 결승 red 감시 활성화')
        self.in_red_zone = bool(msg.data)

    def red_callback(self, msg: Bool):
        if not self.enabled or self.state != DRIVING:
            return
        # 출발 직후 red 무시 구간
        if self.drive_start_time is not None:
            elapsed = (self.get_clock().now() - self.drive_start_time).nanoseconds * 1e-9
            if elapsed < self.min_drive_time_sec:
                return
        # 빨간 구간 위에서는 red 신호 무시 (바닥 오인 차단 — 결승등은 구간 밖)
        if self.in_red_zone:
            self.red_count = 0
            return
        # 빨간 구간(아루코) 통과 전엔 red 무시 — 단 시간 백업(fallback) 경과 시엔 유효
        if not self.passed_red_zone and self.drive_start_time is not None:
            elapsed = (self.get_clock().now() - self.drive_start_time).nanoseconds * 1e-9
            if elapsed < self.red_zone_fallback_sec:
                self.red_count = 0
                return
        self.red_count = self.red_count + 1 if msg.data else 0
        if self.red_count >= self.red_stable_frames:
            self.laps_done += 1
            self.red_count = 0
            self.green_count = 0
            if self.laps_done >= self.lap_count:
                self.state = FINISH
                self.get_logger().info(
                    f'RED LIGHT — lap {self.laps_done}/{self.lap_count} FINAL, FINISH STOP (latched)')
            else:
                # 중간 랩 정지: green 재점등을 기다렸다가 다시 출발
                self.state = WAIT_GREEN
                self.get_logger().info(
                    f'RED LIGHT — lap {self.laps_done}/{self.lap_count} done, waiting green to resume')

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
        self.lap_pub.publish(Int32(data=self.laps_done))


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
