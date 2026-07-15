#!/usr/bin/env python3
"""차선 인식 노드 (ROS1 lane_detection_hsv.py 포팅).

변경점 (JetRacer / 카메라 온리 플랫폼):
  - LiDAR(/raw_obstacles), IMU(/heading) 구독 제거
  - 입력: /camera/image/compressed (sensor_msgs/CompressedImage)
  - 출력: /motor_lane (drive_msgs/DriveCommand) — 로직/토픽 구조는 원본 유지
  - HSV 범위, 속도, 미션 임계값을 전부 ROS2 파라미터화 (트랙바 대체)
"""

import array

import cv2

# 통합 스택(노드 5개×워커 4스레드)이 4코어에서 서로 선점하며 병렬 동기화 비용만 내는 것 방지
# (실측: 경합 시 차선 파이프라인 4T 65ms vs 1T 31ms — 격리 시엔 4T 18ms vs 1T 22ms 로 손해 미미)
cv2.setNumThreads(1)

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32, String

from drive_msgs.msg import DriveCommand
from outcourse_lane_detection.slidewindow import SlideWindow

HALF_FRAME_W = 320.0    # 화면 반폭 (640/2) — 오차 정규화 분모 (조향 목표와는 별개)
MORPH_BAR_W = 3         # 세로 막대 커널 가로폭 — "세로 구조만 생존" 판정 기준


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte
        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('outcourse_lane_detection_node')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('process_hz', 30.0)
        self.declare_parameter('version', 'safe')            # 'safe' | 'fast'
        self.declare_parameter('speed_safe', 0.45)
        self.declare_parameter('speed_fast', 0.5)
        self.declare_parameter('use_pid', False)              # 원본은 raw 오차 사용
        self.declare_parameter('publish_debug_image', True)
        # 디버그 영상을 N프레임마다 1장만 발행 (인코딩+발행 비용 1/N) — 뷰어는 10fps 면 충분
        self.declare_parameter('debug_every', 3)
        # 영상이 이 시간 이상 끊기면 정지 명령 (카메라 사망 시 정지화면 주행 방지)
        self.declare_parameter('image_stale_sec', 0.5)
        # 아웃코스 안전장치: 차선을 연속으로 잃은 채 마지막 조향값으로 주행하지 않는다.
        self.declare_parameter('detection_stop_sec', 0.5)
        # 모니터/VNC 연결 시 imshow + HSV 트랙바 표시 (헤드리스 SSH 에서는 false 유지)
        self.declare_parameter('show_gui', False)

        # 차선 마스크에 포함할 색 선택 (새 트랙: 흰색 + 주황 차선)
        self.declare_parameter('lane_use_white', True)
        self.declare_parameter('lane_use_orange', True)
        self.declare_parameter('lane_use_yellow', True)   # 구 트랙(노란 차선) 호환용

        # HSV 범위
        # (빨간 바닥 검출은 cv_detect/red_zone_node 로 분리 — 여기는 차선 색만)
        self.declare_parameter('hsv_yellow_lower', [10, 108, 125])
        self.declare_parameter('hsv_yellow_upper', [35, 255, 255])
        self.declare_parameter('hsv_orange_lower', [5, 80, 80])
        self.declare_parameter('hsv_orange_upper', [25, 255, 255])
        self.declare_parameter('hsv_white_lower', [30, 0, 151])
        self.declare_parameter('hsv_white_upper', [122, 67, 207])

        # Adaptive Threshold 차선 이진화 (2026-07-14 대회장: 글레어↔암부 편차가 HSV 절대값
        # 튜닝 범위를 벗어남 → 국소 평균 대비 밝은 픽셀 = 차선, 조명 불변. HSV 연산 완전 미사용)
        # false 로 내리면 기존 HSV 경로로 즉시 원복 (ros2 param set 런타임 스위치).
        self.declare_parameter('lane_use_adaptive', True)
        self.declare_parameter('adaptive_block', 75)   # 국소 평균 윈도우 (홀수, 차선 폭 2배쯤)
        self.declare_parameter('adaptive_c', -18)      # 임계 = 국소평균 − C → 음수 절대값이 클수록 엄격
        # adaptive 모드의 /yellow_pixels = BGR 산술 노랑 카운트: min(R,G)-B > 임계 (ROI y>=300).
        # HSV 없이 색만 선별 — 대회장 bag 실측(run_0714_163731, T=60): 흰선 구간 0 /
        # 링 접근 중앙값 2633 / 링 위 10428 / 빨간(주황) 구간 0 → 회전교차로 arm 전용 신호.
        # (라인 픽셀 총량 방식은 링/직선 구분 실패로 폐기 — 링 접근 11.6k vs 직선 12k)
        self.declare_parameter('yellow_chroma_thresh', 60)
        # 노랑 카운트 ROI 시작 y (2026-07-15): 300이면 원거리 링도 잡혀 "보임"에 조기 발동 —
        # 400(화면 최하단 80px)이면 선이 차 바로 앞에 왔을 때만 카운트 = 도착 신호
        # (bag 실측: 접근 중 y>400 은 0 유지 → 입구 도달 순간 0→2924 계단)
        self.declare_parameter('yellow_roi_y0', 400)
        # 회전교차로 LOOP에서는 그레이스케일이 옆 아웃코스 흰선을 같은 차선으로
        # 받아들이므로 BGR 색차로 노란 안쪽 링의 정체성을 유지한다 (HSV 미사용).
        self.declare_parameter('ring_use_chroma', True)
        self.declare_parameter('ring_chroma_min_pixels', 120)
        self.declare_parameter('ring_temporal_margin', 60)
        self.declare_parameter('ring_temporal_min_windows', 3)
        self.declare_parameter('ring_adaptive_c', -40)
        self.declare_parameter('ring_chroma_gate_dilate', 21)
        self.declare_parameter('ring_x_valid_min', 40.0)
        self.declare_parameter('ring_x_valid_max', 600.0)
        self.declare_parameter('ring_outer_anchor_tolerance', 80.0)
        # BEV 워핑 후 좌우 가장자리 N px 컬럼 제거 — 트랙 밖 배경 침입 차단 (0=끔)
        self.declare_parameter('bev_edge_mask', 60)
        # 모폴로지 열기 커널 (0=끔, 홀수 3+) — 커널보다 얇은 노이즈 제거.
        # ⚠️ bag 실측(2026-07-14): 이 트랙 노이즈는 굵어서 효과 미미 (커널7: 노이즈 -11% vs 차선 -8.6%)
        self.declare_parameter('morph_open', 0)
        # 면적 기반 블롭 제거 (0=끔): contourArea < 임계 인 성분을 통째로 삭제.
        # 모폴로지와 달리 차선 실선(큰 성분)은 전혀 안 깎음 — bag 실측(500): 노이즈 -16% vs
        # 차선픽셀 -8% (손실분은 원거리 파편/점선 조각). 비용 0.3ms.
        self.declare_parameter('min_blob_area', 500)
        # 높이 기반 블롭 제거 (0=끔): 세로 높이가 이 미만인 성분 삭제 — 점선(20~60px) 제거,
        # 실선(ROI 관통 170px+)은 보존. 급커브에서 실선이 끊겨 짧아지면 낮출 것
        self.declare_parameter('min_blob_height', 80)
        # x_location EMA 스무딩: 새 샘플 가중치 (1.0=끔). bag 실측(2026-07-15): 인지 지터가
        # 프레임당 평균 40px(단선 추정 전환 점프) → 게인만으로 지그재그 해결 불가.
        # 0.5 = 지터 절반, 지연 ~1프레임(40ms). 커브 반응이 둔해지면 0.7 로.
        self.declare_parameter('x_ema_alpha', 0.5)
        # x_location 프레임당 최대 변화량 (0=끔). bag 실측(2026-07-15 t=45.6s): 트랙 옆
        # 사람이 커브에서 오른쪽 차선으로 오인 → 1프레임 247→67 급락 → 급꺾임.
        # 실제 차량 동역학상 60px/frame(25Hz 기준 1500px/s) 이상의 참 변화는 불가능.
        self.declare_parameter('x_max_step', 60)
        # 급점프 완전 홀드 프레임 수 (25Hz 기준 4=160ms) — 그 이상 지속되면 실제 변화로 수용
        self.declare_parameter('x_hold_frames', 4)
        # 커브 감속 (0=끔): speed ×= 1 - gain×|err|/320 (하한 60%). P제어 커브 정상상태
        # 오차(조향수요÷게인, 0.0035 에서 ~100px)는 게인 인상 없인 못 줄이는데 게인 0.0040 은
        # 과조향 기각(2026-07-15) → 저속으로 같은 조향에 더 작게 돌게 하는 최후의 보루.
        self.declare_parameter('curve_slow_gain', 0.6)
        # 감속 하한 (speed_safe 대비 비율). 0.6×0.18=0.108 은 모터 정지마찰 미만 → 사실상
        # 정지했다가 인지 정리 시 자동 재출발 (오차 비례라 상태 없음). ⚠️ 진짜 이탈로 오차가
        # 유지되면 교착(영구 정지) — 경기에선 0.8(기어가기 유지) 권장, 연습은 0.6 허용.
        self.declare_parameter('curve_slow_floor', 0.6)
        # 조향 목표 x (기본 320=화면 중앙). 좌회전 전용 트랙 공략(2026-07-15): 340~355 로
        # 올리면 차가 상시 차선 왼쪽(안쪽)에 붙어 달림 — "왼쪽에 붙으면 돈다" 실증의 자동화.
        # 부수 효과: 카메라 장착 틀어짐으로 인한 중심 오프셋(실측 시 x≈285 편향)도 이 값으로 교정.
        self.declare_parameter('lane_center_x', 320.0)

        # 슬라이딩윈도우 튜닝값 (slidewindow.py 의 동명 속성과 1:1, 실시간 반영)
        self.declare_parameter('sw_margin', 60)
        self.declare_parameter('sw_win_h1', 380)
        self.declare_parameter('sw_win_half', 140)
        self.declare_parameter('sw_circle_height', 280)
        self.declare_parameter('sw_road_width', 0.51)
        self.declare_parameter('sw_nwindows', 20)
        self.declare_parameter('sw_minpix', 0)

        # PID 게인 (use_pid: true 일 때 사용 — 직선 지그재그/커브 출렁임 억제용)
        self.declare_parameter('pid_kp', 0.7)
        self.declare_parameter('pid_ki', 0.0008)
        self.declare_parameter('pid_kd', 0.15)

        self.version = str(self.get_parameter('version').value)
        self.speed_safe = float(self.get_parameter('speed_safe').value)
        self.speed_fast = float(self.get_parameter('speed_fast').value)
        self.use_pid = bool(self.get_parameter('use_pid').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.debug_every = max(1, int(self.get_parameter('debug_every').value))
        self._debug_skip = 0
        self.image_stale_sec = float(self.get_parameter('image_stale_sec').value)
        self.detection_stop_sec = float(self.get_parameter('detection_stop_sec').value)
        self.last_image_time = self.get_clock().now()
        self._stale_logged = False
        self._detection_invalid_since = None
        self._detection_lost_sec = 0.0
        self.show_gui = bool(self.get_parameter('show_gui').value)

        self.lane_use_white = bool(self.get_parameter('lane_use_white').value)
        self.lane_use_orange = bool(self.get_parameter('lane_use_orange').value)
        self.lane_use_yellow = bool(self.get_parameter('lane_use_yellow').value)

        self.lower_yellow = np.array(self.get_parameter('hsv_yellow_lower').value)
        self.upper_yellow = np.array(self.get_parameter('hsv_yellow_upper').value)
        self.lower_orange = np.array(self.get_parameter('hsv_orange_lower').value)
        self.upper_orange = np.array(self.get_parameter('hsv_orange_upper').value)
        self.lower_white = np.array(self.get_parameter('hsv_white_lower').value)
        self.upper_white = np.array(self.get_parameter('hsv_white_upper').value)

        self.lane_use_adaptive = bool(self.get_parameter('lane_use_adaptive').value)
        self.adaptive_block = int(self.get_parameter('adaptive_block').value)
        self.adaptive_c = int(self.get_parameter('adaptive_c').value)
        self.yellow_chroma_thresh = int(self.get_parameter('yellow_chroma_thresh').value)
        self.yellow_roi_y0 = int(self.get_parameter('yellow_roi_y0').value)
        self.ring_use_chroma = bool(self.get_parameter('ring_use_chroma').value)
        self.ring_chroma_min_pixels = int(
            self.get_parameter('ring_chroma_min_pixels').value)
        self.ring_adaptive_c = int(self.get_parameter('ring_adaptive_c').value)
        self.ring_chroma_gate_dilate = int(
            self.get_parameter('ring_chroma_gate_dilate').value)
        self.ring_x_valid_min = float(self.get_parameter('ring_x_valid_min').value)
        self.ring_x_valid_max = float(self.get_parameter('ring_x_valid_max').value)
        self.ring_outer_anchor_tolerance = float(
            self.get_parameter('ring_outer_anchor_tolerance').value)
        self.bev_edge_mask = int(self.get_parameter('bev_edge_mask').value)
        self.morph_open = int(self.get_parameter('morph_open').value)
        self.min_blob_area = int(self.get_parameter('min_blob_area').value)
        self.min_blob_height = int(self.get_parameter('min_blob_height').value)
        self.x_ema_alpha = float(self.get_parameter('x_ema_alpha').value)
        self.x_max_step = int(self.get_parameter('x_max_step').value)
        self.x_hold_frames = int(self.get_parameter('x_hold_frames').value)
        self._x_jump_count = 0
        self.curve_slow_gain = float(self.get_parameter('curve_slow_gain').value)
        self.curve_slow_floor = float(self.get_parameter('curve_slow_floor').value)
        self.lane_center_x = float(self.get_parameter('lane_center_x').value)
        # BEV 워핑이 y>340 만 샘플 → 그 아래 ROI 만 이진화 (여유 40px, CPU ~1/3)
        self.adapt_y0 = 300

        # Perspective Transform (원본 좌표 유지)
        # 파이프라인 입구에서 640x480 강제 리사이즈하므로 폭은 상수 (process 의 x 와 동일)
        x = 640
        left_margin = 200
        top_margin = 340
        src_points = np.float32([
            [128, 400],
            [left_margin, top_margin],
            [x - left_margin, top_margin],
            [520, 400],
        ])
        dst_points = np.float32([
            [x // 4, 460],
            [x // 4, 0],
            [x // 4 * 3, 0],
            [x // 4 * 3, 460],
        ])
        self.matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        # ---------------- 통신 ----------------
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,   # 최신 프레임만 사용 — 처리 지연 시 스테일 프레임 역직렬화 낭비 방지 (10→1, 통합 25Hz)
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_topic = str(self.get_parameter('image_topic').value)
        self.create_subscription(CompressedImage, image_topic, self.image_callback, image_qos)
        self.create_subscription(String, '/lane_topic', self.lane_topic_callback, 10)
        self.create_subscription(
            String, '/mission/roundabout_state', self.roundabout_state_callback, 10)
        # E-STOP 전환 시 PID 누적 리셋 (정지 중 쌓인 적분이 해제 순간 튀는 것 방지)
        self.create_subscription(Bool, '/e_stop', self.e_stop_reset_callback, 10)

        self.ctrl_cmd_pub = self.create_publisher(DriveCommand, '/motor_lane', 1)
        self.white_pixel_pub = self.create_publisher(Int32, '/white_pixels', 1)
        self.orange_pixel_pub = self.create_publisher(Int32, '/orange_pixels', 1)
        # 노란 픽셀 수 — 회전교차로(노란 링) 진입(arm) 신호로 사용
        self.yellow_pixel_pub = self.create_publisher(Int32, '/yellow_pixels', 1)
        # 제어 디버깅 단계별 x: raw(슬라이딩윈도우) → guarded(폭주가드) →
        # limited(rate limiter) → /lane_x_location(EMA 최종). Float32 4개라 bag 비용은 미미하다.
        self.x_raw_pub = self.create_publisher(Float32, '/lane_x_raw', 1)
        self.x_guarded_pub = self.create_publisher(Float32, '/lane_x_guarded', 1)
        self.x_limited_pub = self.create_publisher(Float32, '/lane_x_limited', 1)
        self.x_location_pub = self.create_publisher(Float32, '/lane_x_location', 1)
        self.lane_error_pub = self.create_publisher(Float32, '/lane_error', 1)
        # 아웃코스 기본 제어 벡 진단: 이진화→필터→BEV→경계선 선택 단계별 관측값.
        self.mask_raw_pixel_pub = self.create_publisher(Int32, '/lane/mask_pixels_raw', 1)
        self.mask_filtered_pixel_pub = self.create_publisher(
            Int32, '/lane/mask_pixels_filtered', 1)
        self.bev_pixel_pub = self.create_publisher(Int32, '/lane/bev_pixels', 1)
        self.detect_valid_pub = self.create_publisher(Bool, '/lane/detection_valid', 1)
        self.detected_line_pub = self.create_publisher(String, '/lane/detected_line', 1)
        self.left_seed_pixel_pub = self.create_publisher(Int32, '/lane/left_seed_pixels', 1)
        self.right_seed_pixel_pub = self.create_publisher(Int32, '/lane/right_seed_pixels', 1)
        self.tracked_windows_pub = self.create_publisher(Int32, '/lane/tracked_windows', 1)
        self.center_target_pub = self.create_publisher(Float32, '/lane/center_target', 1)
        self.speed_target_pub = self.create_publisher(Float32, '/lane/speed_target', 1)
        self.ring_valid_pub = self.create_publisher(Bool, '/ring/tracking_valid', 1)
        self.ring_source_pub = self.create_publisher(String, '/ring/tracking_source', 1)
        self.ring_pixel_pub = self.create_publisher(Int32, '/ring/chroma_pixels', 1)
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                CompressedImage, '/lane_detection/image/debug', image_qos
            )

        # ---------------- 상태 ----------------
        self.slidewindow = SlideWindow()
        self.slidewindow.margin = int(self.get_parameter('sw_margin').value)
        self.slidewindow.win_h1 = int(self.get_parameter('sw_win_h1').value)
        self.slidewindow.win_half = int(self.get_parameter('sw_win_half').value)
        self.slidewindow.circle_height = int(self.get_parameter('sw_circle_height').value)
        self.slidewindow.road_width = float(self.get_parameter('sw_road_width').value)
        self.slidewindow.nwindows = int(self.get_parameter('sw_nwindows').value)
        self.slidewindow.minpix = int(self.get_parameter('sw_minpix').value)
        self.slidewindow.temporal_margin = int(
            self.get_parameter('ring_temporal_margin').value)
        self.slidewindow.temporal_min_windows = int(
            self.get_parameter('ring_temporal_min_windows').value)
        self.pid = PID(
            float(self.get_parameter('pid_kp').value),
            float(self.get_parameter('pid_ki').value),
            float(self.get_parameter('pid_kd').value),
        )

        self.cv_image = None
        self.raw_image = None
        self._last_x_location = 320.0   # 폭주 가드용 직전 유효값
        self._x_filtered = 320.0        # EMA 스무딩 상태
        self.steer = 0.0
        self.motor = self.speed_safe
        self.lane_state = None
        self.roundabout_loop = False
        self.ring_seeded = False
        self.ring_outer_tracking = False
        self._ring_outer_anchor_x = None
        self._ring_last_valid_inner_x = None
        self._ring_chroma_lost_frames = 0
        self._ring_loop_frames = 0
        self._ring_invalid_frames = 0
        self._ring_reseed_used = False

        if self.show_gui:
            self.setup_gui()

        # ros2 param set 실시간 반영 (HSV / 차선 색 선택)
        self.add_on_set_parameters_callback(self.on_param_change)

        process_hz = float(self.get_parameter('process_hz').value)
        self.timer = self.create_timer(1.0 / process_hz, self.process)

        self.get_logger().info(
            f'lane_detection_node started: image={image_topic}, '
            f'version={self.version}, show_gui={self.show_gui}'
        )

    # ---------------- 파라미터 실시간 반영 ----------------
    def on_param_change(self, params):
        """ros2 param set 으로 HSV/색 선택/임계값을 재시작 없이 튜닝."""
        hsv_map = {
            'hsv_yellow_lower': 'lower_yellow', 'hsv_yellow_upper': 'upper_yellow',
            'hsv_orange_lower': 'lower_orange', 'hsv_orange_upper': 'upper_orange',
            'hsv_white_lower': 'lower_white', 'hsv_white_upper': 'upper_white',
        }
        for p in params:
            if p.name in hsv_map:
                setattr(self, hsv_map[p.name], np.array(p.value))
            elif p.name in ('lane_use_white', 'lane_use_orange', 'lane_use_yellow',
                            'lane_use_adaptive'):
                setattr(self, p.name, bool(p.value))
            elif p.name in ('adaptive_block', 'adaptive_c', 'yellow_chroma_thresh',
                            'bev_edge_mask', 'morph_open', 'min_blob_area',
                            'min_blob_height', 'yellow_roi_y0',
                            'ring_chroma_min_pixels', 'ring_adaptive_c',
                            'ring_chroma_gate_dilate'):
                setattr(self, p.name, int(p.value))
            elif p.name in ('ring_temporal_margin', 'ring_temporal_min_windows'):
                setattr(self.slidewindow, p.name.removeprefix('ring_'), int(p.value))
            elif p.name == 'ring_use_chroma':
                self.ring_use_chroma = bool(p.value)
            elif p.name == 'x_ema_alpha':
                self.x_ema_alpha = float(p.value)
            elif p.name in ('x_max_step', 'x_hold_frames'):
                setattr(self, p.name, int(p.value))
            elif p.name in ('curve_slow_gain', 'curve_slow_floor', 'lane_center_x'):
                setattr(self, p.name, float(p.value))
            elif p.name in ('ring_x_valid_min', 'ring_x_valid_max'):
                setattr(self, p.name, float(p.value))
            elif p.name in ('speed_safe', 'speed_fast'):
                setattr(self, p.name, float(p.value))
            elif p.name in ('sw_margin', 'sw_win_h1', 'sw_win_half', 'sw_circle_height',
                            'sw_nwindows', 'sw_minpix'):
                setattr(self.slidewindow, p.name[3:], int(p.value))
            elif p.name == 'sw_road_width':
                self.slidewindow.road_width = float(p.value)
            elif p.name == 'use_pid':
                self.use_pid = bool(p.value)
            elif p.name == 'debug_every':
                self.debug_every = max(1, int(p.value))
            elif p.name in ('pid_kp', 'pid_ki', 'pid_kd'):
                setattr(self.pid, p.name[4:], float(p.value))
            self.get_logger().info(f'param updated: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ---------------- GUI (트랙바/imshow, 원본 워크플로우) ----------------
    def setup_gui(self):
        """원본 lane_detection_hsv.py 의 트랙바 구성 복원. 디스플레이 없으면 자동 비활성화."""
        try:
            cv2.namedWindow('Trackbars')
            bars = [
                ('Yellow Lower H', self.lower_yellow[0], 179), ('Yellow Lower S', self.lower_yellow[1], 255), ('Yellow Lower V', self.lower_yellow[2], 255),
                ('Yellow Upper H', self.upper_yellow[0], 179), ('Yellow Upper S', self.upper_yellow[1], 255), ('Yellow Upper V', self.upper_yellow[2], 255),
                ('Orange Lower H', self.lower_orange[0], 179), ('Orange Lower S', self.lower_orange[1], 255), ('Orange Lower V', self.lower_orange[2], 255),
                ('Orange Upper H', self.upper_orange[0], 179), ('Orange Upper S', self.upper_orange[1], 255), ('Orange Upper V', self.upper_orange[2], 255),
                ('White Lower H', self.lower_white[0], 179), ('White Lower S', self.lower_white[1], 255), ('White Lower V', self.lower_white[2], 255),
                ('White Upper H', self.upper_white[0], 179), ('White Upper S', self.upper_white[1], 255), ('White Upper V', self.upper_white[2], 255),
            ]
            for name, init, maxval in bars:
                cv2.createTrackbar(name, 'Trackbars', int(init), maxval, lambda x: None)
        except cv2.error as e:
            self.get_logger().warning(f'No display available, disabling GUI: {e}')
            self.show_gui = False

    def read_trackbars(self):
        """트랙바 값으로 HSV 범위 실시간 갱신 (원본 run() 루프와 동일)."""
        g = lambda name: cv2.getTrackbarPos(name, 'Trackbars')
        self.lower_yellow = np.array([g('Yellow Lower H'), g('Yellow Lower S'), g('Yellow Lower V')])
        self.upper_yellow = np.array([g('Yellow Upper H'), g('Yellow Upper S'), g('Yellow Upper V')])
        self.lower_orange = np.array([g('Orange Lower H'), g('Orange Lower S'), g('Orange Lower V')])
        self.upper_orange = np.array([g('Orange Upper H'), g('Orange Upper S'), g('Orange Upper V')])
        self.lower_white = np.array([g('White Lower H'), g('White Lower S'), g('White Lower V')])
        self.upper_white = np.array([g('White Upper H'), g('White Upper S'), g('White Upper V')])

    # ---------------- 콜백 ----------------
    def image_callback(self, msg: CompressedImage):
        # 바이트만 저장 — 디코딩은 process() 에서 최신 프레임 1장만 수행
        # (30Hz 전수 디코딩이 단일 실행 스레드를 포화시켜 process 가 10Hz 에 묶였음)
        self.raw_image = msg.data
        self.last_image_time = self.get_clock().now()

    def lane_topic_callback(self, msg: String):
        if msg.data in ('LEFT', 'RIGHT'):
            self.lane_state = msg.data
        self.slidewindow.set_lane_side(msg.data)
        self.get_logger().info(f'Current lane state: {self.lane_state}')

    def roundabout_state_callback(self, msg: String):
        was_loop = self.roundabout_loop
        self.roundabout_loop = msg.data == 'LOOP'
        if self.roundabout_loop and not was_loop:
            self.ring_seeded = False
            self.ring_outer_tracking = False
            self._ring_outer_anchor_x = None
            self._ring_last_valid_inner_x = None
            self._ring_chroma_lost_frames = 0
            self._ring_loop_frames = 0
            self._ring_invalid_frames = 0
            self._ring_reseed_used = False
            self.slidewindow.reset_temporal_track()
            self.slidewindow.set_temporal_tracking(True)
        elif was_loop and not self.roundabout_loop:
            self.ring_seeded = False
            self.ring_outer_tracking = False
            self._ring_outer_anchor_x = None
            self._ring_last_valid_inner_x = None
            self.slidewindow.set_temporal_tracking(False)
        self.slidewindow.set_force_line_identity(
            self.lane_state if self.roundabout_loop else None)

    def e_stop_reset_callback(self, msg: Bool):
        # 걸릴 때/풀릴 때 모두 리셋 — 어느 쪽이든 누적은 무효
        self.pid.p_error = 0.0
        self.pid.i_error = 0.0
        self.pid.d_error = 0.0

    # ---------------- 메인 처리 (원본 run() 루프) ----------------
    def process(self):
        if self.raw_image is None:
            return
        # 안전장치: 카메라가 죽으면 raw_image 가 마지막 프레임에 얼어붙어
        # 정지 화면으로 계속 조향/주행하게 됨 → 영상이 끊기면 정지 명령
        age = (self.get_clock().now() - self.last_image_time).nanoseconds * 1e-9
        if age > self.image_stale_sec:
            self.publish_ctrl_cmd(0.0, 0.0)
            if not self._stale_logged:   # 진입 시 1회만 (30Hz 반복 로그는 그 자체가 CPU 비용)
                self._stale_logged = True
                self.get_logger().error(f'카메라 영상 {age:.1f}s 끊김 — 안전 정지 명령 발행')
            return
        self._stale_logged = False
        try:
            raw = np.frombuffer(self.raw_image, dtype=np.uint8)
            self.cv_image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Error decoding image: {e}')
            return
        if self.cv_image is None:
            return

        frame_resized = cv2.resize(self.cv_image, (640, 480))
        y, x = frame_resized.shape[0:2]

        # GUI 모드: 트랙바 값으로 HSV 범위 실시간 갱신
        if self.show_gui:
            self.read_trackbars()

        if self.lane_use_adaptive:
            # 그레이스케일 전용 경로 — HSV 연산(cvtColor+inRange) 완전 미사용.
            # 조명 불변 이진화: 국소 평균보다 (−adaptive_c 만큼) 밝은 픽셀 = 차선.
            # 색 무관이라 흰선·노란 링을 모두 잡고, 회전교차로에서는 미션 노드가
            # /lane_topic LEFT/RIGHT 를 지정해 안쪽 링을 기하적으로 선택한다.
            block = max(3, self.adaptive_block) | 1   # 홀수 강제
            gray_roi = cv2.cvtColor(frame_resized[self.adapt_y0:], cv2.COLOR_BGR2GRAY)
            mask_lane = np.zeros((y, x), dtype=np.uint8)
            adaptive_c = (self.ring_adaptive_c
                          if self.roundabout_loop and self.ring_seeded
                          else self.adaptive_c)
            mask_lane[self.adapt_y0:] = cv2.adaptiveThreshold(
                gray_roi, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
                block, adaptive_c)
            # /yellow_pixels ← BGR 산술 노랑 카운트 (min(R,G)-B > yellow_chroma_thresh).
            # HSV 없이 노란 링만 선별 — bag 실측(T=60): 흰선 0 / 링 접근 2.6k / 주황바닥 0.
            # 흰선은 R≈G≈B 라 0, 주황 바닥은 G-B 차가 작아 T=60 에서 소거됨.
            b_ch, g_ch, r_ch = cv2.split(frame_resized[self.yellow_roi_y0:])
            yellowness = cv2.subtract(cv2.min(r_ch, g_ch), b_ch)
            yellow_count = int(cv2.countNonZero(
                cv2.compare(yellowness, self.yellow_chroma_thresh, cv2.CMP_GT)))
            white_count = orange_count = 0

            # LOOP 전용 실제 추적 마스크. arm용 하단 ROI 카운트와 달리 BEV가 사용하는
            # y>=adapt_y0 전체에서 노란 링을 만든다. 노랑이 사라졌다고 흰선 마스크로
            # 즉시 폴백하면 바로 옆 아웃코스로 갈아타므로 빈 마스크를 유지해 invalid로 둔다.
            ring_chroma_pixels = 0
            ring_source = 'grayscale'
            if self.roundabout_loop and self.ring_use_chroma:
                b_ring, g_ring, r_ring = cv2.split(frame_resized[self.adapt_y0:])
                ring_yellowness = cv2.subtract(cv2.min(r_ring, g_ring), b_ring)
                ring_mask_roi = cv2.compare(
                    ring_yellowness, self.yellow_chroma_thresh, cv2.CMP_GT)
                ring_chroma_pixels = int(cv2.countNonZero(ring_mask_roi))
                mask_lane[:] = 0
                if self.ring_outer_tracking:
                    # seed에서 도로 폭으로 짝지은 바깥 흰 경계만 grayscale로 추적한다.
                    # temporal 창 밖의 다른 아웃코스 선은 전역 재획득하지 않는다.
                    mask_lane[self.adapt_y0:] = cv2.adaptiveThreshold(
                        gray_roi, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                        cv2.THRESH_BINARY, block, self.ring_adaptive_c)
                    ring_source = 'paired_outer_track'
                elif ring_chroma_pixels >= self.ring_chroma_min_pixels:
                    self._ring_chroma_lost_frames = 0
                    if not self.ring_seeded:
                        mask_lane[self.adapt_y0:] = ring_mask_roi
                        ring_source = 'yellow_seed'
                    else:
                        # seed 이후에도 색차는 정체성 게이트로만 유지한다. adaptive 형상 중
                        # 노란 링 주변만 통과시켜 바로 옆 굵은 흰 외곽선으로 이동하지 못하게 한다.
                        k = max(1, self.ring_chroma_gate_dilate) | 1
                        gate = cv2.dilate(ring_mask_roi, np.ones((k, k), np.uint8))
                        shaped = cv2.bitwise_and(
                            mask_lane[self.adapt_y0:], gate)
                        mask_lane[self.adapt_y0:] = cv2.bitwise_or(shaped, ring_mask_roi)
                        ring_source = 'yellow_identity_gate'
                else:
                    self._ring_chroma_lost_frames += 1
                    ring_source = 'lost'
                    mask_lane[:] = 0
                    # 노란 안쪽 선이 차량 아래로 3프레임 사라지면, seed 곡선을 도로 폭만큼
                    # 오른쪽으로 평행 이동해 이미 짝지은 흰 외곽선으로 이관한다.
                    if (self.ring_seeded and self._ring_chroma_lost_frames >= 3
                            and self.slidewindow.track_centers):
                        shift = int(round(640 * self.slidewindow.road_width))
                        # 선 경계는 도로 폭만큼 옮기지만, 계산되는 차량 중심은 같은
                        # 도로 중심이어야 한다. 이 값을 paired 추적의 공간 앵커로 둔다.
                        self._ring_outer_anchor_x = self._ring_last_valid_inner_x
                        if self._ring_outer_anchor_x is not None:
                            self.slidewindow.x_previous = int(
                                self._ring_outer_anchor_x)
                        self.slidewindow.track_centers = [
                            int(np.clip(v + shift, 0, 639))
                            for v in self.slidewindow.track_centers]
                        self.slidewindow.set_force_line_identity('RIGHT')
                        self.ring_outer_tracking = True
                        mask_lane[self.adapt_y0:] = cv2.adaptiveThreshold(
                            gray_roi, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                            cv2.THRESH_BINARY, block, self.ring_adaptive_c)
                        ring_source = 'paired_outer_track'
        else:
            # 기존 HSV 경로 (lane_use_adaptive=false 원복용)
            img_hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(img_hsv, self.lower_yellow, self.upper_yellow)
            mask_orange = cv2.inRange(img_hsv, self.lower_orange, self.upper_orange)
            mask_white = cv2.inRange(img_hsv, self.lower_white, self.upper_white)

            # 차선 마스크 합성: 활성화된 색상들의 OR
            mask_lane = np.zeros(mask_white.shape, dtype=np.uint8)
            if self.lane_use_white:
                mask_lane = cv2.bitwise_or(mask_lane, mask_white)
            if self.lane_use_orange:
                mask_lane = cv2.bitwise_or(mask_lane, mask_orange)
            if self.lane_use_yellow:
                mask_lane = cv2.bitwise_or(mask_lane, mask_yellow)

            yellow_count = int(np.count_nonzero(mask_yellow))
            orange_count = int(np.count_nonzero(mask_orange))
            white_count = int(np.count_nonzero(mask_white))
            ring_chroma_pixels = 0
            ring_source = 'hsv'

        # 컬러 필터 이미지는 GUI 표시용으로만 필요 (와핑 입력으로는 미사용)
        mask_raw_pixels = int(cv2.countNonZero(mask_lane))
        filtered_img = (cv2.bitwise_and(frame_resized, frame_resized, mask=mask_lane)
                        if self.show_gui else None)

        # 픽셀 모니터링: 와핑 전 마스크 기준
        # /yellow_pixels = 회전교차로 진입(arm) 신호 (adaptive 모드: BGR 산술 노랑 카운트)
        self.orange_pixel_pub.publish(Int32(data=orange_count))
        self.white_pixel_pub.publish(Int32(data=white_count))
        self.yellow_pixel_pub.publish(Int32(data=yellow_count))

        # 모폴로지 열기 (옵션): 세로 막대 커널(3×k) — "세로로 k px 이상 연속인 것만 생존"
        # → 정지선/가로 줄무늬 등 가로 구조물 제거, 차선(세로)은 보존 (2026-07-15:
        # 코너에서 정지선을 차선으로 오인하는 문제 대응). k 는 정지선 두께보다 크게 (21 권장).
        # ⚠️ 급커브에서 차선이 눕는 상단부가 깎일 수 있음 — 커브 추적 확인하며 사용
        ring_tracking_active = self.roundabout_loop and self.ring_use_chroma
        if self.morph_open >= 3 and not ring_tracking_active:
            k = self.morph_open | 1
            ker = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_BAR_W, k))
            mask_lane[self.adapt_y0:] = cv2.morphologyEx(
                mask_lane[self.adapt_y0:], cv2.MORPH_OPEN, ker)

        # 블롭 제거: 면적(작은 조각) + 높이(점선 조각) 기준 — 차선 실선은 무손상, 0.3ms
        # 높이 필터 근거(2026-07-15 bag 실측): 흰 점선 조각 세로 20~60px vs 실선 170px+(ROI 관통)
        # — 면적으로는 점선(~2770px)과 실선(~3600px) 갭이 좁아 높이가 정확한 분리축
        # 원형 링은 원근영상에서 짧고 누운 조각으로 보이므로 일반 차선용 세로높이
        # 필터를 적용하면 최신 bag 기준 유효 검출률이 90.9%→69.7%로 하락한다.
        if ((self.min_blob_area > 0 or self.min_blob_height > 0)
                and not ring_tracking_active):
            roi_m = mask_lane[self.adapt_y0:]
            cnts, _ = cv2.findContours(roi_m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            small = [c for c in cnts
                     if cv2.contourArea(c) < self.min_blob_area
                     or cv2.boundingRect(c)[3] < self.min_blob_height]
            if small:
                cv2.drawContours(roi_m, small, -1, 0, -1)

        self.mask_raw_pixel_pub.publish(Int32(data=mask_raw_pixels))
        self.mask_filtered_pixel_pub.publish(
            Int32(data=int(cv2.countNonZero(mask_lane))))

        # 슬라이딩윈도우 입력: 1채널 차선 마스크만 와핑
        # (기존: 3채널 컬러 와핑 + cvtColor + 미사용 yellow 와핑 → 10fps CPU 병목이라 제거)
        warped_lane = cv2.warpPerspective(mask_lane, self.matrix, (640, 480))

        # BEV 좌우 가장자리 마스킹 (2026-07-14): 그레이스케일 전환 후 트랙 밖 배경(바닥
        # 격자선 등)이 BEV 가장자리로 침입 — 대비가 차선급이라 C 로는 분리 불가(bag 실측),
        # 위치로 차단. 정상 주행 시 차선은 x 90~550 안 (bag 실측). 0 이면 비활성.
        m = self.bev_edge_mask
        if m > 0:
            warped_lane[:, :m] = 0
            warped_lane[:, -m:] = 0
        self.bev_pixel_pub.publish(Int32(data=int(cv2.countNonZero(warped_lane))))

        # 이진화 + 슬라이딩 윈도우
        bin_img = np.zeros_like(warped_lane)
        bin_img[warped_lane > 20] = 1

        prior_track_centers = (list(self.slidewindow.track_centers)
                               if self.ring_outer_tracking
                               and self.slidewindow.track_centers else None)
        out_img, x_location, _ = self.slidewindow.slidewindow(bin_img)
        self.detect_valid_pub.publish(
            Bool(data=bool(self.slidewindow.detection_valid)))
        self.detected_line_pub.publish(String(data=self.slidewindow.current_line))
        self.left_seed_pixel_pub.publish(
            Int32(data=int(self.slidewindow.left_initial_pixel_count)))
        self.right_seed_pixel_pub.publish(
            Int32(data=int(self.slidewindow.right_initial_pixel_count)))
        self.tracked_windows_pub.publish(
            Int32(data=int(self.slidewindow.tracked_window_count)))
        if (self.roundabout_loop and self.ring_seeded
                and not self.ring_outer_tracking
                and self.slidewindow.detection_valid):
            self._ring_last_valid_inner_x = float(x_location)
        if (self.ring_outer_tracking and self._ring_outer_anchor_x is not None
                and self.slidewindow.detection_valid
                and abs(x_location - self._ring_outer_anchor_x)
                > self.ring_outer_anchor_tolerance):
            # 인접 아웃코스 선을 향해 프레임마다 조금씩 이동하는 identity creep 차단.
            # 측정뿐 아니라 갱신된 창도 되돌려 다음 프레임의 기준이 오염되지 않게 한다.
            if prior_track_centers is not None:
                self.slidewindow.track_centers = prior_track_centers
            self.slidewindow.detection_valid = False
            x_location = int(self._ring_outer_anchor_x)
            ring_source = 'paired_outer_rejected'
        if self.roundabout_loop:
            self._ring_loop_frames += 1
            if self.ring_seeded and not self.slidewindow.detection_valid:
                self._ring_invalid_frames += 1
            else:
                self._ring_invalid_frames = 0
            # 진입 준비 중 마스크 정착으로 최초 seed와 실제 선이 벌어진 경우에만 한 번
            # 재-seed한다. 25프레임 이후/주행 중에는 다른 노란 선 전역 재획득을 금지한다.
            if (self.ring_seeded and not self.ring_outer_tracking
                    and not self._ring_reseed_used
                    and self._ring_loop_frames <= 25
                    and self._ring_invalid_frames >= 3):
                self.ring_seeded = False
                self._ring_reseed_used = True
                self._ring_invalid_frames = 0
                self.slidewindow.reset_temporal_track()
                ring_source = 'entry_reseed'
        if (self.roundabout_loop and self.ring_use_chroma and not self.ring_seeded
                and ring_source == 'yellow_seed' and self.slidewindow.detection_valid):
            self.ring_seeded = True
        # 토픽 의미를 LOOP 링 신뢰도로 한정한다. 기존에는 LOOP 밖에서 항상 True라
        # 미션이 오래된 값을 새 seed 안정 신호로 오인하고 즉시 부스트를 시작했다.
        ring_valid = (self.roundabout_loop
                      and (not self.ring_use_chroma
                           or (self.slidewindow.detection_valid
                               and self.ring_x_valid_min <= x_location
                               <= self.ring_x_valid_max)))
        self.ring_valid_pub.publish(Bool(data=bool(ring_valid)))
        self.ring_source_pub.publish(String(data=ring_source))
        self.ring_pixel_pub.publish(Int32(data=ring_chroma_pixels))
        # slidewindow는 미검출 시 마지막 x를 반환하므로 x 값만으로는 차선 손실을 알 수 없다.
        # 실제 detection_valid를 시간으로 누적해 장기 손실 때만 정지한다.
        now = self.get_clock().now()
        if self.slidewindow.detection_valid:
            self._detection_invalid_since = None
            self._detection_lost_sec = 0.0
        else:
            if self._detection_invalid_since is None:
                self._detection_invalid_since = now
            self._detection_lost_sec = (
                now - self._detection_invalid_since).nanoseconds * 1e-9
        self.x_raw_pub.publish(Float32(data=float(x_location)))

        # 폭주 가드 (2026-07-15 bag 실측: polyfit 실패 순간 x_location 이 -8124 ~ +17928 로
        # 튐 → 게인 곱해 1~2프레임 풀락 조향 = 커브에서 차가 튕기는 원인). 화면 범위를
        # 벗어난 값은 인지 실패로 보고 직전 유효값을 유지한다.
        if 0.0 <= x_location <= 640.0:
            self._last_x_location = x_location
        else:
            self.get_logger().warning(
                f'x_location 폭주 {x_location:.0f} → 직전값 {self._last_x_location:.0f} 유지',
                throttle_duration_sec=2.0)
            x_location = self._last_x_location
        self.x_guarded_pub.publish(Float32(data=float(x_location)))

        # 점프 기각+홀드 (2026-07-15): 물리 한계(x_max_step/frame) 초과 변화 = 오인 판정
        # → x_hold_frames 동안 직전값 완전 유지 (끌려가지 않음 — 플립/배경 오인 시 차가
        # 원래 궤적을 계속 탐). 그 이상 지속되면 실제 변화로 보고 제한 걸고 수용.
        if self.x_max_step > 0:
            if abs(x_location - self._x_filtered) > self.x_max_step:
                self._x_jump_count += 1
                if self._x_jump_count <= self.x_hold_frames:
                    x_location = self._x_filtered
                else:
                    lo = self._x_filtered - self.x_max_step
                    hi = self._x_filtered + self.x_max_step
                    x_location = min(max(x_location, lo), hi)
            else:
                self._x_jump_count = 0
        self.x_limited_pub.publish(Float32(data=float(x_location)))

        # EMA 스무딩: 인지 지터(±40px/프레임, 단선 추정 전환 점프) 완화 — alpha=새 샘플 가중치
        if 0.0 < self.x_ema_alpha < 1.0:
            self._x_filtered = (self.x_ema_alpha * x_location
                                + (1.0 - self.x_ema_alpha) * self._x_filtered)
            x_location = self._x_filtered
        else:
            self._x_filtered = x_location
        self.x_location_pub.publish(Float32(data=float(x_location)))

        # 조향: 원본은 raw 픽셀 오차 (PID 는 옵션). 목표 x 는 파라미터 (좌회전 트랙 공략:
        # lane_center_x > 320 = 상시 좌측 붙어 달리기)
        error = x_location - self.lane_center_x
        self.center_target_pub.publish(Float32(data=float(self.lane_center_x)))
        self.lane_error_pub.publish(Float32(data=float(error)))
        self.steer = self.pid.pid_control(error) if self.use_pid else float(error)

        self.motor = self.speed_fast if self.version == 'fast' else self.speed_safe
        # 커브 감속: 오차 비례로 속도를 줄여 커브 유지력 확보. 하한이 모터 정지마찰
        # 미만이면(0.6×0.18) "멈춤→인지 정리→자동 재출발" 동작 — 오차가 줄면 즉시 복귀
        if self.curve_slow_gain > 0.0:
            self.motor *= max(self.curve_slow_floor,
                              1.0 - self.curve_slow_gain * abs(error) / HALF_FRAME_W)
        if (self.detection_stop_sec > 0.0
                and self._detection_lost_sec >= self.detection_stop_sec):
            self.motor = 0.0
        self.speed_target_pub.publish(Float32(data=float(self.motor)))

        self.publish_ctrl_cmd(self.motor, self.steer)

        # GUI 모드: 원본에서 주석 처리돼 있던 imshow 복원
        if self.show_gui:
            cv2.imshow('Original Image', frame_resized)
            cv2.imshow('Lane Mask (combined)', filtered_img)
            if not self.lane_use_adaptive:   # adaptive 모드에선 색 마스크 없음
                cv2.imshow('Orange Mask', cv2.bitwise_and(frame_resized, frame_resized, mask=mask_orange))
                cv2.imshow('White Mask', cv2.bitwise_and(frame_resized, frame_resized, mask=mask_white))
            cv2.imshow('Warped Image', warped_lane)
            cv2.imshow('Output Image', out_img)
            cv2.waitKey(1)

        # N프레임당 1장만 인코딩/발행 (return 으로 빠지지 않게 블록 가드 — 아래에 코드가 추가돼도 안전)
        self._debug_skip = (self._debug_skip + 1) % self.debug_every
        if self.publish_debug_image and self._debug_skip == 0:
            ok, encoded = cv2.imencode('.jpg', out_img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                dbg = CompressedImage()
                dbg.header.stamp = self.get_clock().now().to_msg()
                dbg.header.frame_id = 'lane_debug'
                dbg.format = 'jpeg'
                dbg.data = array.array('B', encoded.tobytes())   # fast-path (바이트 단위 검증 회피)
                self.debug_image_pub.publish(dbg)

    def publish_ctrl_cmd(self, motor_msg, servo_msg):
        msg = DriveCommand()
        msg.speed = float(motor_msg)
        msg.angle = float(servo_msg)
        msg.flag = True
        self.ctrl_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
