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
  LOOP   : 진입 부스트 후 차선 조향+작은 곡률 bias를 유지. 시간+방향성 조향 적분으로 회전량 추정
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
from std_msgs.msg import Bool, Float32, Int32, String

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

        self.declare_parameter('enabled', True)          # ⚠️ 현장 튜닝 후 true
        self.declare_parameter('publish_hz', 30.0)
        # ---- 진입 판단 ----
        self.declare_parameter('entry_min_sec', 0.0)      # 출발 후 이 시간 전엔 arm 안 함
        # 출발(green) 직후 이 시간 동안 왼쪽 차선 우선 인지 (회전교차로 진입 유도, 0=끔)
        # — LOOP 진입하면 어차피 LEFT 유지, 이 시간 내 진입 실패 시 BOTH 복귀 (2026-07-15)
        self.declare_parameter('entry_lane_side_sec', 15.0)
        self.declare_parameter('entry_max_sec', 15.0)     # 이 시간 도달 시 yellow 무관 강제 진입
        self.declare_parameter('use_yellow_arm', True)
        # 노랑 트리거 후 진입까지 고정 지연 (2026-07-15): 트리거 지점 = 진입 점선이 발밑에
        # 닿는 곳(트랙 고정 기하) — 실제 입구는 반 발짝 뒤라 지연으로 변환. 0=즉시
        self.declare_parameter('entry_trigger_delay_sec', 0.5)
        self.declare_parameter('yellow_arm_threshold', 1800)   # /yellow_pixels 임계
        # ---- 1회전 완료 판단 ----
        self.declare_parameter('loop_done_mode', 'both')  # 'time' | 'steer' | 'both'
        self.declare_parameter('loop_sec', 9.0)           # 시간 기반: 1회전 소요시간 (실측!)
        self.declare_parameter('steer_integral_target', 3.0)  # |조향%| 적분 목표 (실측!)
        # ---- 회전 중 차선 기준 ----
        # LOOP 진입 시 /lane_topic 으로 발행 — 안쪽 차선만 추종해 원을 안정적으로 돎
        # (정방향 반시계 회전 기준 LEFT. track_reverse=true 면 자동으로 반대로)
        self.declare_parameter('loop_lane_side', 'LEFT')   # 'LEFT'|'RIGHT'|'BOTH'
        # ---- 진입 부스트 (2026-07-15: 진입구가 최급 곡률 — P제어 지연으로 입구를 지나침) ----
        # LOOP 시작 직후 이 시간 동안 차선 조향 + entry_angle 바이어스로 안쪽으로 밀어넣음
        # (exit_mode lane_steer 와 동일 메커니즘의 거울상). entry_boost_sec 0 = 끔
        self.declare_parameter('entry_boost_sec', 1.5)
        self.declare_parameter('entry_angle', -120.0)   # 음수 = 좌 (planner: 조향 = -angle×gain)
        self.declare_parameter('entry_speed', 0.2)      # 부스트 중 속도 (0 이하 = 차선 속도)
        # 정방향 고정 트랙 진입 합산 조향 하한. lane_angle과 bias가 같은 방향으로 겹쳐
        # 최신 bag에서 angle≈-286/steering≈+0.86까지 폭증한 것을 제한한다.
        self.declare_parameter('entry_angle_limit', 180.0)
        # 부스트 종료 전 이 시간 동안 바이어스 선형 감쇠 (0=스텝 종료 = 구동작).
        # 핸드오프 불연속 제거: 종료 시점 각도가 lane 각도로 수렴한 뒤 제어권 반납
        self.declare_parameter('entry_boost_fade_sec', 1.0)
        # 부스트 종료 후에도 원형 곡률을 유지하는 작은 좌조향 feedforward.
        # 위치 P제어만 쓰면 오차가 작을 때 조향이 0으로 풀려 링 바깥으로 반복 이탈했다.
        self.declare_parameter('loop_angle_bias', -40.0)
        self.declare_parameter('loop_angle_limit', 180.0)
        # 링은 지속 곡률이므로 차선 오차가 bias를 상쇄해도 최소 좌조향을 유지해야 한다.
        # angle 픽셀단위 70 × steering_gain 0.003 ≈ 최종 조향 0.21.
        self.declare_parameter('loop_min_curve_angle', 70.0)
        self.declare_parameter('tracking_degraded_speed', 0.18)
        self.declare_parameter('tracking_stop_sec', 0.30)
        self.declare_parameter('tracking_recover_sec', 0.15)
        self.declare_parameter('seed_crawl_speed', 0.18)
        self.declare_parameter('seed_crawl_angle_limit', 60.0)
        self.declare_parameter('seed_crawl_max_sec', 1.20)
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
        # LOOP 전용 x_location EMA (0=끔): 링 안 인지 지터(실측 ±40~70px)가 조향 스윙으로
        # 증폭돼 휘청임 — LOOP 동안만 스무딩 강화, DONE 에서 원복. 일반 주행 무영향.
        # 라이브 원복: ros2 param set /roundabout_mission loop_x_ema 0.0
        self.declare_parameter('loop_x_ema', 0.35)

        self.enabled = bool(self.get_parameter('enabled').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.entry_min_sec = float(self.get_parameter('entry_min_sec').value)
        self.entry_lane_side_sec = float(self.get_parameter('entry_lane_side_sec').value)
        self._entry_side_active = False
        self._entry_side_start_time = None
        self.entry_max_sec = float(self.get_parameter('entry_max_sec').value)
        self.use_yellow_arm = bool(self.get_parameter('use_yellow_arm').value)
        self.entry_trigger_delay_sec = float(self.get_parameter('entry_trigger_delay_sec').value)
        self._yellow_hit_time = None
        self.yellow_arm_threshold = int(self.get_parameter('yellow_arm_threshold').value)
        self.loop_done_mode = str(self.get_parameter('loop_done_mode').value)
        self.loop_sec = float(self.get_parameter('loop_sec').value)
        self.steer_integral_target = float(self.get_parameter('steer_integral_target').value)
        self.loop_lane_side = str(self.get_parameter('loop_lane_side').value)
        self.entry_boost_sec = float(self.get_parameter('entry_boost_sec').value)
        self.entry_boost_fade_sec = float(self.get_parameter('entry_boost_fade_sec').value)
        self.loop_angle_bias = float(self.get_parameter('loop_angle_bias').value)
        self.loop_angle_limit = float(self.get_parameter('loop_angle_limit').value)
        self.loop_min_curve_angle = float(
            self.get_parameter('loop_min_curve_angle').value)
        self.tracking_degraded_speed = float(
            self.get_parameter('tracking_degraded_speed').value)
        self.tracking_stop_sec = float(self.get_parameter('tracking_stop_sec').value)
        self.tracking_recover_sec = float(self.get_parameter('tracking_recover_sec').value)
        self.seed_crawl_speed = float(self.get_parameter('seed_crawl_speed').value)
        self.seed_crawl_angle_limit = float(
            self.get_parameter('seed_crawl_angle_limit').value)
        self.seed_crawl_max_sec = float(self.get_parameter('seed_crawl_max_sec').value)
        self.entry_angle = float(self.get_parameter('entry_angle').value)
        self.entry_speed = float(self.get_parameter('entry_speed').value)
        self.entry_angle_limit = float(self.get_parameter('entry_angle_limit').value)
        self.exit_mode = str(self.get_parameter('exit_mode').value)
        self.exit_lane_side = str(self.get_parameter('exit_lane_side').value)
        self.exit_angle = float(self.get_parameter('exit_angle').value)
        self.exit_speed = float(self.get_parameter('exit_speed').value)
        self.exit_duration_sec = float(self.get_parameter('exit_duration_sec').value)
        self.track_reverse = bool(self.get_parameter('track_reverse').value)
        self.loop_x_ema = float(self.get_parameter('loop_x_ema').value)

        self.create_subscription(String, '/mission/traffic_state', self.traffic_state_callback, 10)
        self.create_subscription(Int32, '/yellow_pixels', self.yellow_callback, 10)
        self.create_subscription(Control, '/control', self.control_callback, 10)
        self.create_subscription(Bool, '/ring/tracking_valid', self.ring_valid_callback, 10)
        # lane_steer 탈출용: 차선 노드의 조향값에 바이어스를 가산하기 위해 구독
        self.create_subscription(DriveCommand, '/motor_lane', self.lane_cmd_callback, 10)

        self.cmd_pub = self.create_publisher(DriveCommand, '/motor_roundabout', 10)
        self.state_pub = self.create_publisher(String, '/mission/roundabout_state', 10)
        self.integral_pub = self.create_publisher(Float32, '/roundabout/steer_integral', 10)
        # LOOP 경과 시간 (뷰어 모니터링용 — loop_sec 대비 진행률)
        self.elapsed_pub = self.create_publisher(Float32, '/roundabout/loop_elapsed', 10)
        # 차선 기준 전환 (lane_detection 의 슬라이딩윈도우가 구독)
        self.lane_side_pub = self.create_publisher(String, '/lane_topic', 10)

        # LOOP 추종 모드 전환: 대회장 조명에서는 HSV 색 분리가 성립하지 않으므로
        # 회전교차로도 그레이스케일 adaptive 경로를 명시적으로 사용한다. 진입 시점의
        # lane_use_adaptive 값을 저장하고 DONE 에서 복원한다.
        self._lane_get = self.create_client(GetParameters, '/lane_detection_node/get_parameters')
        self._lane_set = self.create_client(SetParameters, '/lane_detection_node/set_parameters')
        self._saved_adaptive = None  # LOOP 진입 시점의 lane_use_adaptive 스냅샷
        self._saved_x_ema = None    # LOOP 진입 시점의 x_ema_alpha 스냅샷

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
        self.last_valid_lane_angle = 0.0
        self.ring_tracking_valid = False
        self._ring_invalid_since = self.get_clock().now()
        self._ring_valid_since = None
        self.loop_tracking_acquired = False
        self.loop_valid_elapsed = 0.0
        self._last_loop_tick = None

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
            self._entry_side_active = False   # 진입 창 상태도 함께 초기화 (desync 방지)
            self._entry_side_start_time = None
            self.get_logger().info('lap stop — roundabout re-armed for next lap')

    def yellow_callback(self, msg: Int32):
        self.yellow_pixels = msg.data

    def lane_cmd_callback(self, msg: DriveCommand):
        self.lane_angle = msg.angle
        self.lane_speed = msg.speed
        if self.ring_tracking_valid:
            self.last_valid_lane_angle = msg.angle

    def ring_valid_callback(self, msg: Bool):
        valid = bool(msg.data)
        if valid != self.ring_tracking_valid:
            now = self.get_clock().now()
            if valid:
                self._ring_valid_since = now
                self._ring_invalid_since = None
            else:
                self._ring_invalid_since = now
                self._ring_valid_since = None
        self.ring_tracking_valid = valid

    def tracking_safe_speed(self, normal_speed):
        """LOOP 전용: 순간 손실은 저속, 지속 손실은 정지, 재확인은 안정 후 복귀."""
        now = self.get_clock().now()
        degraded = min(float(normal_speed), self.tracking_degraded_speed)
        if not self.ring_tracking_valid:
            lost = ((now - self._ring_invalid_since).nanoseconds * 1e-9
                    if self._ring_invalid_since is not None else self.tracking_stop_sec)
            return 0.0 if lost >= self.tracking_stop_sec else degraded
        stable = ((now - self._ring_valid_since).nanoseconds * 1e-9
                  if self._ring_valid_since is not None else 0.0)
        return float(normal_speed) if stable >= self.tracking_recover_sec else degraded

    def control_callback(self, msg: Control):
        # LOOP 중 예상 회전 방향 조향만 누적한다. 기존 signed 적분은 순간 반대 보정이
        # 이미 누적한 회전량을 취소해 한 바퀴를 돌고도 타임아웃되는 문제가 반복됐다.
        if self.state != LOOP or not self.loop_tracking_acquired:
            return
        # 잘못 잡은 아웃코스 위에서 적분 목표만 채워 EXIT하는 것을 막는다.
        if not self.ring_tracking_valid:
            self.last_control_time = None
            return
        now = self.get_clock().now()
        if self.last_control_time is not None:
            dt = (now - self.last_control_time).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                sign = -1.0 if self.track_reverse else 1.0
                directional = sign * float(msg.steering)
                self.steer_integral += max(0.0, directional) * dt
        self.last_control_time = now

    def on_param_change(self, params):
        float_params = ('entry_min_sec', 'entry_max_sec', 'loop_sec',
                        'steer_integral_target', 'exit_angle', 'exit_speed',
                        'exit_duration_sec', 'loop_x_ema', 'entry_angle_limit',
                        'loop_angle_bias', 'loop_angle_limit')
        float_params = float_params + (
            'loop_min_curve_angle', 'tracking_degraded_speed',
            'tracking_stop_sec', 'tracking_recover_sec', 'seed_crawl_speed',
            'seed_crawl_angle_limit', 'seed_crawl_max_sec')
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
                # 회전 중 강제 비활성화 시 인지/차선 기준 안전 복원
                if not self.enabled and self._saved_adaptive is not None:
                    self.restore_lane_mode()
                    self.restore_x_ema()
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

    # ---------------- LOOP 그레이스케일 인지 전환 ----------------
    def _apply_adaptive_mode(self, enabled):
        if not self._lane_set.service_is_ready():
            self.get_logger().warning('lane 파라미터 서비스 미준비 — adaptive 전환 실패!')
            return
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = bool(enabled)
        self._lane_set.call_async(SetParameters.Request(
            parameters=[Parameter(name='lane_use_adaptive', value=pv)]))
        self.get_logger().info(f'lane grayscale adaptive -> {bool(enabled)}')

    def set_roundabout_lane_mode(self):
        """LOOP 진입: 색 플래그 대신 그레이스케일 adaptive 추종을 강제한다.

        노란 링 진입 감지는 /yellow_pixels(BGR chroma)를 계속 사용한다. 그레이스케일만으로
        노란 링과 일반 흰선을 구분할 수 없기 때문이다. LOOP 안에서는 색을 구분하지 않고
        밝은 선 전체를 이진화한 뒤 LEFT/RIGHT 기하 선택으로 안쪽 링을 추종한다.
        """
        if self._lane_get.service_is_ready():
            fut = self._lane_get.call_async(GetParameters.Request(names=['lane_use_adaptive']))

            def done(f):
                try:
                    self._saved_adaptive = bool(f.result().values[0].bool_value)
                except Exception:
                    self._saved_adaptive = True
                self._apply_adaptive_mode(True)
            fut.add_done_callback(done)
        else:
            # 서비스가 아직 준비되지 않았더라도 다음 LOOP 틱의 차선 선택은 계속된다.
            # 대회 기본 모드가 adaptive=true이므로 상태만 안전 기본값으로 보존한다.
            self._saved_adaptive = True
            self._apply_adaptive_mode(True)

    def restore_lane_mode(self):
        """DONE/비활성화: LOOP 진입 전 인지 모드로 복원한다."""
        # 대회 기본값은 adaptive=true. 비동기 스냅샷이 늦은 경우에도 HSV로 잘못
        # 떨어지지 않도록 true를 안전 기본값으로 사용한다.
        previous = self._saved_adaptive if self._saved_adaptive is not None else True
        self._apply_adaptive_mode(previous)
        self._saved_adaptive = None

    # ---------------- LOOP 전용 EMA 강화 ----------------
    def _apply_x_ema(self, val):
        if not self._lane_set.service_is_ready():
            self.get_logger().warning('lane 파라미터 서비스 미준비 — x_ema 전환 실패!')
            return
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(val)
        self._lane_set.call_async(SetParameters.Request(
            parameters=[Parameter(name='x_ema_alpha', value=pv)]))
        self.get_logger().info(f'x_ema_alpha -> {val}')

    def set_loop_x_ema(self):
        """LOOP 진입: 현재 EMA 를 스냅샷하고 링 전용 강화값으로 전환 (0=기능 끔)."""
        if self.loop_x_ema <= 0.0:
            return
        if self._lane_get.service_is_ready():
            fut = self._lane_get.call_async(GetParameters.Request(names=['x_ema_alpha']))

            def done(f):
                try:
                    self._saved_x_ema = float(f.result().values[0].double_value)
                except Exception:
                    pass
            fut.add_done_callback(done)
        self._apply_x_ema(self.loop_x_ema)

    def restore_x_ema(self):
        """DONE: LOOP 진입 전 EMA 로 복원 (스냅샷 실패 시 params.yaml 기본 0.5)."""
        if self.loop_x_ema <= 0.0:
            return
        self._apply_x_ema(self._saved_x_ema if self._saved_x_ema else 0.5)
        self._saved_x_ema = None

    def loop(self):
        if not self.enabled:
            self.publish_cmd(flag=False)
            self.state_pub.publish(String(data=f'{self.state}(disabled)'))
            return

        # ARMED 이후 왼쪽 우선 인지의 시간 만료 처리. 출발 직후부터 LEFT를 강제하면
        # 회전교차로 도착 전 일반주행까지 한쪽 선만 보게 되므로 ARMED에서만 시작한다.
        if (self._entry_side_active and self.state in (IDLE, ARMED)
                and self.elapsed_since(self._entry_side_start_time) >= self.entry_lane_side_sec):
            self.set_lane_side('BOTH')
            self._entry_side_active = False
            self._entry_side_start_time = None
            self.get_logger().info(f'entry lane-side window({self.entry_lane_side_sec}s) 만료 — BOTH 복귀')

        if self.state == IDLE:
            if self.drive_start_time is not None:
                if self.elapsed_since(self.drive_start_time) >= self.entry_min_sec:
                    self.state = ARMED
                    if self.entry_lane_side_sec > 0.0:
                        self.set_lane_side(self.side_for(self.loop_lane_side))
                        self._entry_side_active = True
                        self._entry_side_start_time = self.get_clock().now()
                    self.get_logger().info('ARMED — watching for roundabout entry')

        elif self.state == ARMED:
            elapsed = self.elapsed_since(self.drive_start_time)
            # 노랑 트리거 = "진입 점선이 발밑" — 실제 입구는 entry_trigger_delay_sec 뒤
            if (self.use_yellow_arm and self._yellow_hit_time is None
                    and self.yellow_pixels > self.yellow_arm_threshold):
                self._yellow_hit_time = self.get_clock().now()
                self.get_logger().info(
                    f'노랑 트리거({self.yellow_pixels}px) — {self.entry_trigger_delay_sec}s 뒤 진입')
            yellow_hit = (self._yellow_hit_time is not None
                          and self.elapsed_since(self._yellow_hit_time)
                          >= self.entry_trigger_delay_sec)
            timeout_hit = elapsed >= self.entry_max_sec
            if yellow_hit or timeout_hit:
                self.state = LOOP
                self.loop_start_time = self.get_clock().now()
                self.steer_integral = 0.0
                self.loop_valid_elapsed = 0.0
                self._last_loop_tick = self.loop_start_time
                self.last_control_time = None
                # LOOP 밖의 ring_valid=True를 seed 안정 판정에 재사용하지 않는다.
                self.ring_tracking_valid = False
                self._ring_valid_since = None
                self._ring_invalid_since = self.loop_start_time
                # 진입은 마지막으로 실증된 동작을 그대로 수행한다. seed 안정화는 진입의
                # 선행조건이 아니며, 인지 실패가 이미 되던 진입을 막지 않게 한다.
                self.loop_tracking_acquired = True
                # 회전 중: 그레이스케일 adaptive + 안쪽 차선 기하 추종
                self.set_lane_side(self.side_for(self.loop_lane_side))
                self.set_roundabout_lane_mode()
                self.set_loop_x_ema()   # 링 전용 스무딩 강화 (DONE 에서 원복)
                self._entry_side_active = False   # LOOP 이 LEFT 를 이어받음 — 진입 창 종료
                self._entry_side_start_time = None
                reason = 'yellow' if yellow_hit else 'entry_max_sec timeout'
                self.get_logger().info(f'LOOP entered ({reason})')

        elif self.state == LOOP:
            loop_elapsed = self.elapsed_since(self.loop_start_time)
            now = self.get_clock().now()
            if self._last_loop_tick is not None and self.ring_tracking_valid:
                dt = (now - self._last_loop_tick).nanoseconds * 1e-9
                if 0.0 < dt < 0.5:
                    self.loop_valid_elapsed += dt
            self._last_loop_tick = now
            # 추적을 놓친 시간이 15초 타임아웃을 소모해 강제 EXIT시키지 않게 한다.
            time_done = self.loop_valid_elapsed >= self.loop_sec
            steer_done = self.steer_integral >= self.steer_integral_target
            done = (
                time_done if self.loop_done_mode == 'time'
                else steer_done if self.loop_done_mode == 'steer'
                else (time_done or steer_done)
            )
            self.integral_pub.publish(Float32(data=float(self.steer_integral)))
            self.elapsed_pub.publish(Float32(data=float(self.loop_valid_elapsed)))
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
                self.restore_lane_mode()     # LOOP 전 인지 모드로 복원
                self.restore_x_ema()         # EMA 도 일반 주행값으로 복원
                self.get_logger().info('roundabout DONE — lane BOTH, control released')

        # ---- 발행 ----
        if (self.state == LOOP and self.entry_boost_sec > 0.0
                and self.elapsed_since(self.loop_start_time) < self.entry_boost_sec):
            # 진입 부스트: 차선 조향 + 좌측 바이어스 (최급 곡률 진입구를 P 지연 없이 통과)
            bias = -self.entry_angle if self.track_reverse else self.entry_angle
            # 종료 전 fade 구간에서 바이어스 선형 감쇠 → 스텝 핸드오프 불연속 제거
            remain = self.entry_boost_sec - self.elapsed_since(self.loop_start_time)
            if self.entry_boost_fade_sec > 0.0 and remain < self.entry_boost_fade_sec:
                bias *= remain / self.entry_boost_fade_sec
            speed = self.entry_speed if self.entry_speed > 0.0 else self.lane_speed
            lane_angle = (self.lane_angle if self.ring_tracking_valid
                          else self.last_valid_lane_angle)
            angle = lane_angle + bias
            # 대회 맵은 정방향 고정: 음수 angle이 좌회전 요구다. 진입 차선 오차와
            # 고정 bias의 이중 가산만 제한하고 반대 방향 복구 조향은 막지 않는다.
            if self.entry_angle_limit > 0.0:
                angle = max(angle, -self.entry_angle_limit)
            # 진입 중에는 seed가 순간 끊겨도 계속 제한 동작한다. 차량이 들어가야 링
            # 형상이 보이므로 tracking_safe_speed로 즉시 정지시키지 않는다.
            self.publish_cmd(flag=True, speed=speed, angle=angle)
        elif self.state == LOOP:
            # 링은 지속 좌곡률이라 위치 오차가 작아도 기본 좌조향이 필요하다. 일반 P제어에
            # 작은 feedforward만 더하고, 진입과 같은 합산 상한으로 과조향을 방지한다.
            lane_angle = (self.lane_angle if self.ring_tracking_valid
                          else self.last_valid_lane_angle)
            angle = lane_angle + self.loop_angle_bias
            if self.loop_angle_limit > 0.0:
                angle = (min(angle, self.loop_angle_limit) if self.track_reverse
                         else max(angle, -self.loop_angle_limit))
            if self.loop_min_curve_angle > 0.0:
                angle = (max(angle, self.loop_min_curve_angle) if self.track_reverse
                         else min(angle, -self.loop_min_curve_angle))
            self.publish_cmd(
                flag=True, speed=self.tracking_safe_speed(self.lane_speed), angle=angle)
        elif self.state == EXIT and self.exit_mode == 'steer':
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
