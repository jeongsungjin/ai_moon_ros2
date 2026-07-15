#!/usr/bin/env python3
"""회전교차로 미션 노드 (In 코스: 진입 → 1회전 이상 → 탈출).

인지 포인트: /yellow_pixels 급증이 진입 신호.
  - HSV 모드: 노란(링) HSV 픽셀 수
  - adaptive 모드(2026-07-14 대회장 기본): BGR 산술 노랑 카운트 (min(R,G)-B > 60)
    — bag 실측: 흰선 구간 0 / 링 접근 2633 / 주황(빨간구간) 바닥 0. 임계 1000.
회전 중에는 차선 주행(슬라이딩윈도우)이 원형 차선을 그대로 따라가므로,
이 노드는 (1) 진입 시점 판단 (2) 1회전 완료 판단 (3) 탈출 조향 개입만 담당한다.

단계 (국소 상태):
  IDLE   : 출발 대기. /mission/traffic_state 가 DRIVING 이 되면 타이머 시작
  ARMED  : entry_min_sec 경과 후, yellow 임계 초과 또는 entry_max_sec 도달 → LOOP
  LOOP   : 제어 개입 없음(flag=False). 시간 + |조향| 적분으로 회전량 추정
           완료 조건: mode 'time'  → loop_sec 경과
                     mode 'steer' → 조향 적분 >= steer_integral_target
                     mode 'both'  → 둘 중 먼저 도달하는 쪽
  EXIT   : exit_duration_sec 동안 고정 조향 개입 (flag=True) — 출구로 이탈
  DONE   : 제어권 영구 반납

⚠️ enabled=false 가 기본값 — 현장에서 entry/loop/exit 파라미터 튜닝 후 켤 것.
   (미튜닝 상태로 켜면 잘못된 시점에 조향 개입해 차선 주행을 해칠 수 있음)

angle 단위 주의: DriveCommand.angle 은 차선 픽셀오차 스케일.
   최종 조향 = -angle * steering_gain(≈0.0029) → angle 100 ≈ 조향 -0.29
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Float32, Int32, String

from control_msgs.msg import Control
from drive_msgs.msg import DriveCommand

IDLE = 'IDLE'
ARMED = 'ARMED'
LOOP = 'LOOP'
EXIT = 'EXIT'
DONE = 'DONE'


class RoundaboutMissionNode(Node):
    def __init__(self):
        super().__init__('roundabout_mission')

        self.declare_parameter('enabled', False)          # ⚠️ 현장 튜닝 후 true
        self.declare_parameter('publish_hz', 30.0)
        # ---- 진입 판단 ----
        self.declare_parameter('entry_min_sec', 0.0)      # 출발 후 이 시간 전엔 arm 안 함
        self.declare_parameter('entry_max_sec', 15.0)     # 이 시간 도달 시 yellow 무관 강제 진입
        self.declare_parameter('use_yellow_arm', True)
        self.declare_parameter('yellow_arm_threshold', 1800)   # /yellow_pixels 임계
        # ---- 1회전 완료 판단 ----
        self.declare_parameter('loop_done_mode', 'both')  # 'time' | 'steer' | 'both'
        self.declare_parameter('loop_sec', 8.0)           # 시간 기반: 1회전 소요시간 (실측!)
        self.declare_parameter('steer_integral_target', 3.0)  # |조향%| 적분 목표 (실측!)
        # ---- 회전 중 차선 기준 ----
        # LOOP 진입 시 /lane_topic 으로 발행 — 안쪽 차선만 추종해 원을 안정적으로 돎
        # (정방향 반시계 회전 기준 LEFT. track_reverse=true 면 자동으로 반대로)
        self.declare_parameter('loop_lane_side', 'LEFT')   # 'LEFT'|'RIGHT'|'BOTH'
        # ---- 탈출 방식 ----
        # 'lane'      : 반대쪽 차선 추종으로 전환 (슬라이딩윈도우가 조향)
        # 'lane_steer': 차선 추종 전환 + 차선 조향값에 exit_angle 바이어스 가산 (권장)
        # 'steer'     : exit_duration 동안 고정 조향 개입 (open-loop, fallback)
        self.declare_parameter('exit_mode', 'lane_steer')
        self.declare_parameter('exit_lane_side', 'RIGHT')  # lane/lane_steer 시 추종 차선
        self.declare_parameter('exit_angle', -100.0)       # steer: 고정 조향 / lane_steer: 바이어스
        self.declare_parameter('exit_speed', 0.2)
        self.declare_parameter('exit_duration_sec', 2.0)   # 탈출 유지 시간 (이후 BOTH 복귀)
        # 정/역방향 트랙: 회전 방향 반대 → 차선/조향/적분 부호 자동 반전
        self.declare_parameter('track_reverse', False)

        self.enabled = bool(self.get_parameter('enabled').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.entry_min_sec = float(self.get_parameter('entry_min_sec').value)
        self.entry_max_sec = float(self.get_parameter('entry_max_sec').value)
        self.use_yellow_arm = bool(self.get_parameter('use_yellow_arm').value)
        self.yellow_arm_threshold = int(self.get_parameter('yellow_arm_threshold').value)
        self.loop_done_mode = str(self.get_parameter('loop_done_mode').value)
        self.loop_sec = float(self.get_parameter('loop_sec').value)
        self.steer_integral_target = float(self.get_parameter('steer_integral_target').value)
        self.loop_lane_side = str(self.get_parameter('loop_lane_side').value)
        self.exit_mode = str(self.get_parameter('exit_mode').value)
        self.exit_lane_side = str(self.get_parameter('exit_lane_side').value)
        self.exit_angle = float(self.get_parameter('exit_angle').value)
        self.exit_speed = float(self.get_parameter('exit_speed').value)
        self.exit_duration_sec = float(self.get_parameter('exit_duration_sec').value)
        self.track_reverse = bool(self.get_parameter('track_reverse').value)

        self.create_subscription(String, '/mission/traffic_state', self.traffic_state_callback, 10)
        self.create_subscription(Int32, '/yellow_pixels', self.yellow_callback, 10)
        self.create_subscription(Control, '/control', self.control_callback, 10)
        # lane_steer 탈출용: 차선 노드의 조향값에 바이어스를 가산하기 위해 구독
        self.create_subscription(DriveCommand, '/motor_lane', self.lane_cmd_callback, 10)

        self.cmd_pub = self.create_publisher(DriveCommand, '/motor_roundabout', 10)
        self.state_pub = self.create_publisher(String, '/mission/roundabout_state', 10)
        self.integral_pub = self.create_publisher(Float32, '/roundabout/steer_integral', 10)
        # LOOP 경과 시간 (뷰어 모니터링용 — loop_sec 대비 진행률)
        self.elapsed_pub = self.create_publisher(Float32, '/roundabout/loop_elapsed', 10)
        # 차선 기준 전환 (lane_detection 의 슬라이딩윈도우가 구독)
        self.lane_side_pub = self.create_publisher(String, '/lane_topic', 10)

        # 차선 색 전환: LOOP~DONE 동안 노랑(링) 전용, DONE 에서 원래 색으로 복원
        # (흰 바깥차선이 왼쪽 박스에 오인되는 것을 색 차원에서 차단)
        self._lane_get = self.create_client(GetParameters, '/lane_detection_node/get_parameters')
        self._lane_set = self.create_client(SetParameters, '/lane_detection_node/set_parameters')
        self._saved_colors = None   # LOOP 진입 시점의 (white, orange, yellow) 스냅샷

        # 실시간 튜닝
        self.add_on_set_parameters_callback(self.on_param_change)

        self.state = IDLE
        self.drive_start_time = None    # 출발(DRIVING 전환) 시각
        self.loop_start_time = None
        self.exit_start_time = None
        self.yellow_pixels = 0
        self.steer_integral = 0.0
        self.last_control_time = None
        self.lane_angle = 0.0
        self.lane_speed = 0.0

        self.timer = self.create_timer(1.0 / publish_hz, self.loop)

        self.get_logger().info(
            f'roundabout_mission started: enabled={self.enabled}, '
            f'mode={self.loop_done_mode}, loop_sec={self.loop_sec}, '
            f'integral_target={self.steer_integral_target}, reverse={self.track_reverse}'
        )

    # ---------------- 콜백 ----------------
    def traffic_state_callback(self, msg: String):
        if msg.data == 'DRIVING' and self.drive_start_time is None:
            self.drive_start_time = self.get_clock().now()
            self.get_logger().info('start detected (traffic DRIVING) — roundabout timer begins')
        # 랩 완료 정지(WAIT_GREEN 복귀) 시 재무장: 다음 랩에서 회전교차로를 다시 처리
        elif msg.data == 'WAIT_GREEN' and self.state == DONE:
            self.state = IDLE
            self.drive_start_time = None
            self.steer_integral = 0.0
            self.get_logger().info('lap stop — roundabout re-armed for next lap')

    def yellow_callback(self, msg: Int32):
        self.yellow_pixels = msg.data

    def lane_cmd_callback(self, msg: DriveCommand):
        self.lane_angle = msg.angle
        self.lane_speed = msg.speed

    def control_callback(self, msg: Control):
        # LOOP 중 조향 적분 (회전량 추정). 정방향/역방향은 부호만 다름 → 절대 회전량은
        # 방향 부호를 곱해 누적 (반대 방향 조향은 감산되어 직진 구간 노이즈 상쇄)
        if self.state != LOOP:
            return
        now = self.get_clock().now()
        if self.last_control_time is not None:
            dt = (now - self.last_control_time).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                sign = -1.0 if self.track_reverse else 1.0
                self.steer_integral += sign * float(msg.steering) * dt
        self.last_control_time = now

    def on_param_change(self, params):
        float_params = ('entry_min_sec', 'entry_max_sec', 'loop_sec',
                        'steer_integral_target', 'exit_angle', 'exit_speed',
                        'exit_duration_sec')
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
                # 회전 중 강제 비활성화 시 색/차선 기준 안전 복원
                if not self.enabled and self._saved_colors is not None:
                    self.restore_colors()
                    self.set_lane_side('BOTH')
            elif p.name == 'track_reverse':
                self.track_reverse = bool(p.value)
            elif p.name == 'use_yellow_arm':
                self.use_yellow_arm = bool(p.value)
            elif p.name == 'yellow_arm_threshold':
                self.yellow_arm_threshold = int(p.value)
            elif p.name in ('loop_done_mode', 'loop_lane_side', 'exit_mode', 'exit_lane_side'):
                setattr(self, p.name, str(p.value))
            elif p.name in float_params:
                setattr(self, p.name, float(p.value))
            self.get_logger().info(f'param updated: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ---------------- 상태 진행 ----------------
    def elapsed_since(self, t):
        if t is None:
            return 0.0
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def side_for(self, side):
        """track_reverse 시 LEFT/RIGHT 자동 반전 (거울상 트랙 대응)."""
        if not self.track_reverse:
            return side
        return {'LEFT': 'RIGHT', 'RIGHT': 'LEFT'}.get(side, side)

    def set_lane_side(self, side):
        self.lane_side_pub.publish(String(data=side))
        self.get_logger().info(f'lane side -> {side}')

    # ---------------- 차선 색 전환 ----------------
    def _apply_lane_colors(self, white, orange, yellow):
        if not self._lane_set.service_is_ready():
            self.get_logger().warning('lane 파라미터 서비스 미준비 — 색 전환 실패!')
            return
        params = []
        for name, val in (('lane_use_white', white),
                          ('lane_use_orange', orange),
                          ('lane_use_yellow', yellow)):
            pv = ParameterValue()
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = bool(val)
            params.append(Parameter(name=name, value=pv))
        self._lane_set.call_async(SetParameters.Request(parameters=params))
        self.get_logger().info(f'차선 색 전환: white={white} orange={orange} yellow={yellow}')

    def set_yellow_only(self):
        """LOOP 진입: 현재 색 구성을 스냅샷하고 노랑(링) 전용으로 전환.

        adaptive(그레이스케일) 모드에선 색 플래그가 마스크에 영향 없음(무해한 no-op) —
        링 추종은 set_lane_side(안쪽 차선 박스) 기하가 담당. HSV 원복 시를 위해 유지.
        """
        names = ['lane_use_white', 'lane_use_orange', 'lane_use_yellow']
        if self._lane_get.service_is_ready():
            fut = self._lane_get.call_async(GetParameters.Request(names=names))

            def done(f):
                try:
                    vals = f.result().values
                    self._saved_colors = tuple(v.bool_value for v in vals)
                except Exception:
                    pass
            fut.add_done_callback(done)
        self._apply_lane_colors(False, False, True)

    def restore_colors(self):
        """DONE/비활성화: LOOP 진입 전 색 구성으로 복원 (스냅샷 실패 시 기본값)."""
        w, o, y = self._saved_colors if self._saved_colors else (True, True, True)
        self._apply_lane_colors(w, o, y)
        self._saved_colors = None

    def loop(self):
        if not self.enabled:
            self.publish_cmd(flag=False)
            self.state_pub.publish(String(data=f'{self.state}(disabled)'))
            return

        if self.state == IDLE:
            if self.drive_start_time is not None:
                if self.elapsed_since(self.drive_start_time) >= self.entry_min_sec:
                    self.state = ARMED
                    self.get_logger().info('ARMED — watching for roundabout entry')

        elif self.state == ARMED:
            elapsed = self.elapsed_since(self.drive_start_time)
            yellow_hit = self.use_yellow_arm and self.yellow_pixels > self.yellow_arm_threshold
            timeout_hit = elapsed >= self.entry_max_sec
            if yellow_hit or timeout_hit:
                self.state = LOOP
                self.loop_start_time = self.get_clock().now()
                self.steer_integral = 0.0
                self.last_control_time = None
                # 회전 중: 안쪽 차선 기준 추종 + 노랑(링) 전용 마스크
                self.set_lane_side(self.side_for(self.loop_lane_side))
                self.set_yellow_only()
                reason = 'yellow' if yellow_hit else 'entry_max_sec timeout'
                self.get_logger().info(f'LOOP entered ({reason})')

        elif self.state == LOOP:
            loop_elapsed = self.elapsed_since(self.loop_start_time)
            time_done = loop_elapsed >= self.loop_sec
            steer_done = abs(self.steer_integral) >= self.steer_integral_target
            done = (
                time_done if self.loop_done_mode == 'time'
                else steer_done if self.loop_done_mode == 'steer'
                else (time_done or steer_done)
            )
            self.integral_pub.publish(Float32(data=float(self.steer_integral)))
            self.elapsed_pub.publish(Float32(data=float(loop_elapsed)))
            if done:
                self.state = EXIT
                self.exit_start_time = self.get_clock().now()
                if self.exit_mode in ('lane', 'lane_steer'):
                    # 탈출: 바깥쪽 차선 기준으로 전환 → 슬라이딩윈도우가 출구로 유도
                    self.set_lane_side(self.side_for(self.exit_lane_side))
                self.get_logger().info(
                    f'LOOP done (t={loop_elapsed:.1f}s, integral={self.steer_integral:.2f}) '
                    f'— EXIT ({self.exit_mode})'
                )

        elif self.state == EXIT:
            if self.elapsed_since(self.exit_start_time) >= self.exit_duration_sec:
                self.state = DONE
                self.set_lane_side('BOTH')   # 일반 주행 복귀
                self.restore_colors()        # 차선 색도 원래대로 (노랑+흰 등)
                self.get_logger().info('roundabout DONE — lane BOTH, control released')

        # ---- 발행 ----
        if self.state == EXIT and self.exit_mode == 'steer':
            # 고정 조향 개입 (open-loop)
            angle = -self.exit_angle if self.track_reverse else self.exit_angle
            self.publish_cmd(flag=True, speed=self.exit_speed, angle=angle)
        elif self.state == EXIT and self.exit_mode == 'lane_steer':
            # 차선(바깥쪽) 조향값 + 탈출 바이어스 가산 — 차선 추종을 유지하면서
            # 출구 방향으로 지속적으로 밀어줌 (closed-loop + bias)
            bias = -self.exit_angle if self.track_reverse else self.exit_angle
            speed = self.exit_speed if self.exit_speed > 0.0 else self.lane_speed
            self.publish_cmd(flag=True, speed=speed, angle=self.lane_angle + bias)
        else:
            # IDLE/ARMED/LOOP/DONE, 또는 lane 모드 EXIT: 차선 주행이 조향 (flag=False)
            self.publish_cmd(flag=False)
        self.state_pub.publish(String(data=self.state))

    def publish_cmd(self, flag, speed=0.0, angle=0.0):
        cmd = DriveCommand()
        cmd.speed = float(speed)
        cmd.angle = float(angle)
        cmd.flag = bool(flag)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RoundaboutMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
