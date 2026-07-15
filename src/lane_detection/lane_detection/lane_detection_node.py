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
from lane_detection.slidewindow import SlideWindow


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
        super().__init__('lane_detection_node')

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
        # BEV 워핑 후 좌우 가장자리 N px 컬럼 제거 — 트랙 밖 배경 침입 차단 (0=끔)
        self.declare_parameter('bev_edge_mask', 60)
        # 모폴로지 열기 커널 (0=끔, 홀수 3+) — 커널보다 얇은 노이즈 제거.
        # ⚠️ bag 실측(2026-07-14): 이 트랙 노이즈는 굵어서 효과 미미 (커널7: 노이즈 -11% vs 차선 -8.6%)
        self.declare_parameter('morph_open', 0)
        # 면적 기반 블롭 제거 (0=끔): contourArea < 임계 인 성분을 통째로 삭제.
        # 모폴로지와 달리 차선 실선(큰 성분)은 전혀 안 깎음 — bag 실측(500): 노이즈 -16% vs
        # 차선픽셀 -8% (손실분은 원거리 파편/점선 조각). 비용 0.3ms.
        self.declare_parameter('min_blob_area', 500)
        # x_location EMA 스무딩: 새 샘플 가중치 (1.0=끔). bag 실측(2026-07-15): 인지 지터가
        # 프레임당 평균 40px(단선 추정 전환 점프) → 게인만으로 지그재그 해결 불가.
        # 0.5 = 지터 절반, 지연 ~1프레임(40ms). 커브 반응이 둔해지면 0.7 로.
        self.declare_parameter('x_ema_alpha', 0.5)
        # x_location 프레임당 최대 변화량 (0=끔). bag 실측(2026-07-15 t=45.6s): 트랙 옆
        # 사람이 커브에서 오른쪽 차선으로 오인 → 1프레임 247→67 급락 → 급꺾임.
        # 실제 차량 동역학상 60px/frame(25Hz 기준 1500px/s) 이상의 참 변화는 불가능.
        self.declare_parameter('x_max_step', 60)
        # 커브 감속 (0=끔): speed ×= 1 - gain×|err|/320 (하한 60%). P제어 커브 정상상태
        # 오차(조향수요÷게인, 0.0035 에서 ~100px)는 게인 인상 없인 못 줄이는데 게인 0.0040 은
        # 과조향 기각(2026-07-15) → 저속으로 같은 조향에 더 작게 돌게 하는 최후의 보루.
        self.declare_parameter('curve_slow_gain', 0.6)

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
        self.last_image_time = self.get_clock().now()
        self._stale_logged = False
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
        self.bev_edge_mask = int(self.get_parameter('bev_edge_mask').value)
        self.morph_open = int(self.get_parameter('morph_open').value)
        self.min_blob_area = int(self.get_parameter('min_blob_area').value)
        self.x_ema_alpha = float(self.get_parameter('x_ema_alpha').value)
        self.x_max_step = int(self.get_parameter('x_max_step').value)
        self.curve_slow_gain = float(self.get_parameter('curve_slow_gain').value)
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
        # E-STOP 전환 시 PID 누적 리셋 (정지 중 쌓인 적분이 해제 순간 튀는 것 방지)
        self.create_subscription(Bool, '/e_stop', self.e_stop_reset_callback, 10)

        self.ctrl_cmd_pub = self.create_publisher(DriveCommand, '/motor_lane', 1)
        self.white_pixel_pub = self.create_publisher(Int32, '/white_pixels', 1)
        self.orange_pixel_pub = self.create_publisher(Int32, '/orange_pixels', 1)
        # 노란 픽셀 수 — 회전교차로(노란 링) 진입(arm) 신호로 사용
        self.yellow_pixel_pub = self.create_publisher(Int32, '/yellow_pixels', 1)
        self.x_location_pub = self.create_publisher(Float32, '/lane_x_location', 1)
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
                            'bev_edge_mask', 'morph_open', 'min_blob_area'):
                setattr(self, p.name, int(p.value))
            elif p.name == 'x_ema_alpha':
                self.x_ema_alpha = float(p.value)
            elif p.name == 'x_max_step':
                self.x_max_step = int(p.value)
            elif p.name == 'curve_slow_gain':
                self.curve_slow_gain = float(p.value)
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
            # 색 무관이라 흰선·노란(링) 선 모두 잡힘 — 기하 추종용으로는 그게 맞음.
            block = max(3, self.adaptive_block) | 1   # 홀수 강제
            gray_roi = cv2.cvtColor(frame_resized[self.adapt_y0:], cv2.COLOR_BGR2GRAY)
            mask_lane = np.zeros((y, x), dtype=np.uint8)
            mask_lane[self.adapt_y0:] = cv2.adaptiveThreshold(
                gray_roi, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
                block, self.adaptive_c)
            # /yellow_pixels ← BGR 산술 노랑 카운트 (min(R,G)-B > yellow_chroma_thresh).
            # HSV 없이 노란 링만 선별 — bag 실측(T=60): 흰선 0 / 링 접근 2.6k / 주황바닥 0.
            # 흰선은 R≈G≈B 라 0, 주황 바닥은 G-B 차가 작아 T=60 에서 소거됨.
            b_ch, g_ch, r_ch = cv2.split(frame_resized[self.adapt_y0:])
            yellowness = cv2.subtract(cv2.min(r_ch, g_ch), b_ch)
            yellow_count = int(cv2.countNonZero(
                cv2.compare(yellowness, self.yellow_chroma_thresh, cv2.CMP_GT)))
            white_count = orange_count = 0
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

        # 컬러 필터 이미지는 GUI 표시용으로만 필요 (와핑 입력으로는 미사용)
        filtered_img = (cv2.bitwise_and(frame_resized, frame_resized, mask=mask_lane)
                        if self.show_gui else None)

        # 픽셀 모니터링: 와핑 전 마스크 기준
        # /yellow_pixels = 회전교차로 진입(arm) 신호 (adaptive 모드: BGR 산술 노랑 카운트)
        self.orange_pixel_pub.publish(Int32(data=orange_count))
        self.white_pixel_pub.publish(Int32(data=white_count))
        self.yellow_pixel_pub.publish(Int32(data=yellow_count))

        # 모폴로지 열기 (옵션): 커널보다 얇은 노이즈 제거 — ROI 만 (0.2~0.8ms)
        if self.morph_open >= 3:
            k = self.morph_open | 1
            ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask_lane[self.adapt_y0:] = cv2.morphologyEx(
                mask_lane[self.adapt_y0:], cv2.MORPH_OPEN, ker)

        # 면적 기반 블롭 제거: 작은 성분만 통째로 삭제 (차선 실선은 무손상, 0.3ms)
        # CHAIN_APPROX_SIMPLE + drawContours 일괄 1회 (프레임별 파이썬 루프 갱신 회피)
        if self.min_blob_area > 0:
            roi_m = mask_lane[self.adapt_y0:]
            cnts, _ = cv2.findContours(roi_m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            small = [c for c in cnts if cv2.contourArea(c) < self.min_blob_area]
            if small:
                cv2.drawContours(roi_m, small, -1, 0, -1)

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

        # 이진화 + 슬라이딩 윈도우
        bin_img = np.zeros_like(warped_lane)
        bin_img[warped_lane > 20] = 1

        out_img, x_location, _ = self.slidewindow.slidewindow(bin_img)

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

        # 변화율 제한: 1프레임 급점프(트랙 옆 사람 오인 등)는 물리적으로 불가능한 신호
        # — 프레임당 x_max_step 이내로 클램프 (지속적인 참 변화는 몇 프레임에 걸쳐 따라감)
        if self.x_max_step > 0:
            lo = self._x_filtered - self.x_max_step
            hi = self._x_filtered + self.x_max_step
            x_location = min(max(x_location, lo), hi)

        # EMA 스무딩: 인지 지터(±40px/프레임, 단선 추정 전환 점프) 완화 — alpha=새 샘플 가중치
        if 0.0 < self.x_ema_alpha < 1.0:
            self._x_filtered = (self.x_ema_alpha * x_location
                                + (1.0 - self.x_ema_alpha) * self._x_filtered)
            x_location = self._x_filtered
        else:
            self._x_filtered = x_location
        self.x_location_pub.publish(Float32(data=float(x_location)))

        # 조향: 원본은 raw 픽셀 오차 (PID 는 옵션)
        error = x_location - 320
        self.steer = self.pid.pid_control(error) if self.use_pid else float(error)

        self.motor = self.speed_fast if self.version == 'fast' else self.speed_safe
        # 커브 감속: 오차 비례로 속도를 줄여 커브 유지력 확보 (하한 60%)
        if self.curve_slow_gain > 0.0:
            self.motor *= max(0.6, 1.0 - self.curve_slow_gain * abs(error) / 320.0)

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
