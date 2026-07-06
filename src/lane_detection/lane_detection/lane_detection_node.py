#!/usr/bin/env python3
"""차선 인식 노드 (ROS1 lane_detection_hsv.py 포팅).

변경점 (JetRacer / 카메라 온리 플랫폼):
  - LiDAR(/raw_obstacles), IMU(/heading) 구독 제거
  - 입력: /camera/image/compressed (sensor_msgs/CompressedImage)
  - 출력: /motor_lane (drive_msgs/DriveCommand) — 로직/토픽 구조는 원본 유지
  - HSV 범위, 속도, 미션 임계값을 전부 ROS2 파라미터화 (트랙바 대체)
"""

import cv2
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
        self.declare_parameter('speed_red_zone', 0.2)         # 빨간 구간 감속
        self.declare_parameter('use_pid', False)              # 원본은 raw 오차 사용
        self.declare_parameter('publish_debug_image', True)
        # 모니터/VNC 연결 시 imshow + HSV 트랙바 표시 (헤드리스 SSH 에서는 false 유지)
        self.declare_parameter('show_gui', False)

        # 차선 마스크에 포함할 색 선택 (새 트랙: 흰색 + 주황 차선)
        self.declare_parameter('lane_use_white', True)
        self.declare_parameter('lane_use_orange', True)
        self.declare_parameter('lane_use_yellow', False)   # 구 트랙(노란 차선) 호환용

        # HSV 범위 (원본 튜닝값 그대로 기본값)
        self.declare_parameter('hsv_yellow_lower', [10, 108, 125])
        self.declare_parameter('hsv_yellow_upper', [35, 255, 255])
        self.declare_parameter('hsv_left_yellow_lower', [15, 80, 90])
        self.declare_parameter('hsv_left_yellow_upper', [30, 255, 235])
        self.declare_parameter('hsv_orange_lower', [5, 80, 80])
        self.declare_parameter('hsv_orange_upper', [25, 255, 255])
        self.declare_parameter('hsv_white_lower', [30, 0, 151])
        self.declare_parameter('hsv_white_upper', [122, 67, 207])
        self.declare_parameter('hsv_red_lower', [145, 35, 35])
        self.declare_parameter('hsv_red_upper', [179, 255, 255])

        # 미션 임계값
        self.declare_parameter('red_pixel_threshold', 30000)
        self.declare_parameter('white_pixel_threshold', 35000)
        self.declare_parameter('white_stop_max_count', 190)

        self.version = str(self.get_parameter('version').value)
        self.speed_safe = float(self.get_parameter('speed_safe').value)
        self.speed_fast = float(self.get_parameter('speed_fast').value)
        self.speed_red_zone = float(self.get_parameter('speed_red_zone').value)
        self.use_pid = bool(self.get_parameter('use_pid').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)

        self.lane_use_white = bool(self.get_parameter('lane_use_white').value)
        self.lane_use_orange = bool(self.get_parameter('lane_use_orange').value)
        self.lane_use_yellow = bool(self.get_parameter('lane_use_yellow').value)

        self.lower_yellow = np.array(self.get_parameter('hsv_yellow_lower').value)
        self.upper_yellow = np.array(self.get_parameter('hsv_yellow_upper').value)
        self.lower_left_yellow = np.array(self.get_parameter('hsv_left_yellow_lower').value)
        self.upper_left_yellow = np.array(self.get_parameter('hsv_left_yellow_upper').value)
        self.lower_orange = np.array(self.get_parameter('hsv_orange_lower').value)
        self.upper_orange = np.array(self.get_parameter('hsv_orange_upper').value)
        self.lower_white = np.array(self.get_parameter('hsv_white_lower').value)
        self.upper_white = np.array(self.get_parameter('hsv_white_upper').value)
        self.lower_red = np.array(self.get_parameter('hsv_red_lower').value)
        self.upper_red = np.array(self.get_parameter('hsv_red_upper').value)

        self.red_pixel_threshold = int(self.get_parameter('red_pixel_threshold').value)
        self.white_pixel_threshold = int(self.get_parameter('white_pixel_threshold').value)
        self.white_stop_max_count = int(self.get_parameter('white_stop_max_count').value)

        # ---------------- 통신 ----------------
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_topic = str(self.get_parameter('image_topic').value)
        self.create_subscription(CompressedImage, image_topic, self.image_callback, image_qos)
        self.create_subscription(String, '/lane_topic', self.lane_topic_callback, 10)
        self.create_subscription(Bool, '/tunnel_done', self.tunnel_done_callback, 10)

        self.ctrl_cmd_pub = self.create_publisher(DriveCommand, '/motor_lane', 1)
        self.white_cnt_pub = self.create_publisher(Int32, '/white_cnt', 1)
        self.yellow_pixel_pub = self.create_publisher(Int32, '/yellow_pixel', 1)
        self.white_pixel_pub = self.create_publisher(Int32, '/white_pixels', 1)
        self.red_pixel_pub = self.create_publisher(Int32, '/red_pixels', 1)
        self.x_location_pub = self.create_publisher(Float32, '/lane_x_location', 1)
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                CompressedImage, '/lane_detection/image/debug', image_qos
            )

        # ---------------- 상태 ----------------
        self.slidewindow = SlideWindow()
        if self.version == 'fast':
            self.pid = PID(0.78, 0.0005, 0.405)
        else:
            self.pid = PID(0.7, 0.0008, 0.15)

        self.cv_image = None
        self.steer = 0.0
        self.motor = self.speed_safe
        self.white_count = 0
        self.stop_count = 0
        self.after_white = False
        self.tunnel_done_flag = False
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
            'hsv_left_yellow_lower': 'lower_left_yellow', 'hsv_left_yellow_upper': 'upper_left_yellow',
            'hsv_orange_lower': 'lower_orange', 'hsv_orange_upper': 'upper_orange',
            'hsv_white_lower': 'lower_white', 'hsv_white_upper': 'upper_white',
            'hsv_red_lower': 'lower_red', 'hsv_red_upper': 'upper_red',
        }
        for p in params:
            if p.name in hsv_map:
                setattr(self, hsv_map[p.name], np.array(p.value))
            elif p.name in ('lane_use_white', 'lane_use_orange', 'lane_use_yellow'):
                setattr(self, p.name, bool(p.value))
            elif p.name in ('red_pixel_threshold', 'white_pixel_threshold'):
                setattr(self, p.name, int(p.value))
            elif p.name in ('speed_safe', 'speed_fast', 'speed_red_zone'):
                setattr(self, p.name, float(p.value))
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
                ('Red Lower H', self.lower_red[0], 179), ('Red Lower S', self.lower_red[1], 255), ('Red Lower V', self.lower_red[2], 255),
                ('Red Upper H', self.upper_red[0], 179), ('Red Upper S', self.upper_red[1], 255), ('Red Upper V', self.upper_red[2], 255),
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
        self.lower_red = np.array([g('Red Lower H'), g('Red Lower S'), g('Red Lower V')])
        self.upper_red = np.array([g('Red Upper H'), g('Red Upper S'), g('Red Upper V')])

    # ---------------- 콜백 ----------------
    def image_callback(self, msg: CompressedImage):
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            self.cv_image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Error decoding image: {e}')

    def lane_topic_callback(self, msg: String):
        if msg.data in ('LEFT', 'RIGHT'):
            self.lane_state = msg.data
        self.slidewindow.set_lane_side(msg.data)
        self.get_logger().info(f'Current lane state: {self.lane_state}')

    def tunnel_done_callback(self, msg: Bool):
        self.tunnel_done_flag = msg.data

    # ---------------- 메인 처리 (원본 run() 루프) ----------------
    def process(self):
        if self.cv_image is None:
            return

        frame_resized = cv2.resize(self.cv_image, (640, 480))
        y, x = frame_resized.shape[0:2]

        # GUI 모드: 트랙바 값으로 HSV 범위 실시간 갱신
        if self.show_gui:
            self.read_trackbars()

        img_hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)

        # 색상별 마스크
        if not self.tunnel_done_flag:
            mask_yellow = cv2.inRange(img_hsv, self.lower_yellow, self.upper_yellow)
        else:
            mask_yellow = cv2.inRange(img_hsv, self.lower_left_yellow, self.upper_left_yellow)

        mask_orange = cv2.inRange(img_hsv, self.lower_orange, self.upper_orange)
        mask_white = cv2.inRange(img_hsv, self.lower_white, self.upper_white)
        mask_red = cv2.inRange(img_hsv, self.lower_red, self.upper_red)

        # 차선 마스크 합성: 활성화된 색상들의 OR (새 트랙 = 흰색 + 주황)
        # (기존 코드는 노란색만 슬라이딩윈도우에 들어갔음 — 흰색은 정지선 카운트 전용이었음)
        mask_lane = np.zeros(mask_white.shape, dtype=np.uint8)
        if self.lane_use_white:
            mask_lane = cv2.bitwise_or(mask_lane, mask_white)
        if self.lane_use_orange:
            mask_lane = cv2.bitwise_or(mask_lane, mask_orange)
        if self.lane_use_yellow:
            mask_lane = cv2.bitwise_or(mask_lane, mask_yellow)

        filtered_img = cv2.bitwise_and(frame_resized, frame_resized, mask=mask_lane)

        # Perspective Transform (원본 좌표 유지)
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
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        warped_img = cv2.warpPerspective(filtered_img, matrix, (640, 480))
        warped_img_white = cv2.warpPerspective(mask_white, matrix, (640, 480))
        warped_img_yellow = cv2.warpPerspective(mask_yellow, matrix, (640, 480))
        warped_img_red = cv2.warpPerspective(mask_red, matrix, (640, 480))

        self.yellow_pixel_pub.publish(Int32(data=int(np.count_nonzero(warped_img_yellow))))
        self.red_pixel_pub.publish(Int32(data=int(np.count_nonzero(mask_red))))

        # 이진화 + 슬라이딩 윈도우
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img > 20] = 1

        out_img, x_location, _ = self.slidewindow.slidewindow(bin_img)
        self.x_location_pub.publish(Float32(data=float(x_location)))

        # 조향: 원본은 raw 픽셀 오차 (PID 는 옵션)
        error = x_location - 320
        self.steer = self.pid.pid_control(error) if self.use_pid else float(error)

        self.motor = self.speed_fast if self.version == 'fast' else self.speed_safe

        # 미션: 빨간색 차로 구간 감속
        if np.count_nonzero(warped_img_red) > self.red_pixel_threshold:
            self.motor = self.speed_red_zone

        # 미션: 흰색 횡단보도 구간 정지
        elif (self.stop_count < self.white_stop_max_count
              and np.count_nonzero(warped_img_white) > self.white_pixel_threshold):
            self.motor = 0.0
            self.stop_count += 1
            self.after_white = True

        if self.after_white:
            self.white_count += 1
            self.white_cnt_pub.publish(Int32(data=self.white_count))

        self.white_pixel_pub.publish(Int32(data=int(np.count_nonzero(warped_img_white))))

        self.publish_ctrl_cmd(self.motor, self.steer)

        # GUI 모드: 원본에서 주석 처리돼 있던 imshow 복원
        if self.show_gui:
            cv2.imshow('Original Image', frame_resized)
            cv2.imshow('Lane Mask (combined)', filtered_img)
            cv2.imshow('Orange Mask', cv2.bitwise_and(frame_resized, frame_resized, mask=mask_orange))
            cv2.imshow('White Mask', cv2.bitwise_and(frame_resized, frame_resized, mask=mask_white))
            cv2.imshow('Red Mask', cv2.bitwise_and(frame_resized, frame_resized, mask=mask_red))
            cv2.imshow('Warped Image', warped_img)
            cv2.imshow('Output Image', out_img)
            cv2.imshow('Warped White Stop Line', warped_img_white)
            cv2.waitKey(1)

        if self.publish_debug_image:
            ok, encoded = cv2.imencode('.jpg', out_img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                dbg = CompressedImage()
                dbg.header.stamp = self.get_clock().now().to_msg()
                dbg.header.frame_id = 'lane_debug'
                dbg.format = 'jpeg'
                dbg.data = encoded.tobytes()
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
