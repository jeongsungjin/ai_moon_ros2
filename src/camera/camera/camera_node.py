#!/usr/bin/env python3
"""D-Racer-Kit 규약 카메라 노드.

USB(V4L2) 또는 CSI(GStreamer) 카메라에서 프레임을 읽어
/camera/image/compressed (sensor_msgs/CompressedImage, jpeg) 로 발행한다.
"""

import array

import cv2

# 통합 스택(노드 5개×워커 4스레드)이 4코어에서 서로 선점하며 병렬 동기화 비용만 내는 것 방지
# (실측: 경합 시 차선 파이프라인 4T 65ms vs 1T 31ms — 격리 시엔 4T 18ms vs 1T 22ms 로 손해 미미)
cv2.setNumThreads(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def gstreamer_pipeline(width, height, fps, flip_method=0):
    """Jetson CSI 카메라(nvarguscamerasrc)용 GStreamer 파이프라인."""
    return (
        'nvarguscamerasrc ! '
        f'video/x-raw(memory:NVMM), width={width}, height={height}, '
        f'framerate={int(fps)}/1 ! '
        f'nvvidconv flip-method={flip_method} ! '
        f'video/x-raw, width={width}, height={height}, format=BGRx ! '
        'videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1'
    )


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('publish_topic', '/camera/image/compressed')
        self.declare_parameter('camera_type', 'usb')       # 'usb' | 'csi'
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('flip_180', False)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('debug_log', False)
        # USB 카메라의 하드웨어 MJPG(JPEG)를 디코딩/재인코딩 없이 그대로 발행 (CPU ~90%→~5%).
        # flip_180 이거나 협상/프레임 검증 실패 시 자동으로 기존(재인코딩) 경로 폴백
        self.declare_parameter('passthrough_mjpg', False)

        publish_topic = str(self.get_parameter('publish_topic').value)
        self.camera_type = camera_type = str(self.get_parameter('camera_type').value)
        camera_device = str(self.get_parameter('camera_device').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.flip_180 = bool(self.get_parameter('flip_180').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)
        self.passthrough = bool(self.get_parameter('passthrough_mjpg').value)
        self._passthrough_checked = False   # 첫 프레임 JPEG 유효성 검증 여부

        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be greater than 0')
        if not 0 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in range [0, 100]')

        # D-Racer-Kit 과 동일한 QoS (RELIABLE / VOLATILE / KEEP_LAST 10)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(CompressedImage, publish_topic, image_qos)

        if camera_type == 'csi':
            pipeline = gstreamer_pipeline(
                self.image_width, self.image_height, publish_hz,
                flip_method=2 if self.flip_180 else 0,
            )
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, publish_hz)
            if self.passthrough and self.flip_180:
                self.get_logger().warning('flip_180 은 디코딩이 필요 — passthrough 비활성')
                self.passthrough = False
            if self.passthrough:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
                fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC)) & 0xFFFFFFFF
                name = ''.join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))
                if name != 'MJPG':
                    self.get_logger().warning(f'MJPG 협상 실패 (현재 {name!r}) — passthrough 비활성')
                    self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
                    self.passthrough = False
        if self.passthrough and camera_type == 'csi':
            self.passthrough = False   # CSI 는 BGR 파이프라인

        if not self.cap.isOpened():
            raise RuntimeError(
                f'Failed to open camera (type={camera_type}, device={camera_device})'
            )

        self.get_logger().info(
            f'camera_node started: type={camera_type}, device={camera_device}, '
            f'{self.image_width}x{self.image_height}@{publish_hz}Hz -> {publish_topic}'
        )

        self.timer = self.create_timer(1.0 / publish_hz, self.timer_callback)

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning('Failed to read camera frame')
            return

        if self.passthrough:
            jpeg = frame.tobytes()   # CONVERT_RGB=0 → 카메라의 MJPG 비트스트림 그대로
            if not self._passthrough_checked:
                # 첫 프레임만 실제 JPEG 인지 검증 — 아니면 재인코딩 경로로 영구 폴백
                import numpy as np
                img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is None or img.shape[1] != self.image_width:
                    self.get_logger().warning('passthrough 프레임 검증 실패 — 재인코딩 경로 폴백')
                    self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
                    self.passthrough = False
                    return
                self._passthrough_checked = True
                self.get_logger().info(
                    f'MJPG passthrough 활성: {img.shape[1]}x{img.shape[0]}, {len(jpeg)/1024:.0f}KB/f')
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.format = 'jpeg'
            # array('B') 는 setter fast-path — bytes 로 넣으면 rclpy 가 바이트 단위 검증(~30ms/f)
            msg.data = array.array('B', jpeg)
            self.image_pub.publish(msg)
            return

        # CSI 는 파이프라인의 flip-method=2 로 이미 처리됨 -> USB 만 여기서 회전
        # (둘 다 적용하면 이중 회전으로 원상복구되는 버그)
        if self.flip_180 and self.camera_type != 'csi':
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        ok, encoded = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            self.get_logger().warning('Failed to encode camera frame')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = array.array('B', encoded.tobytes())   # fast-path (바이트 단위 검증 회피)
        self.image_pub.publish(msg)

        if self.debug_log:
            self.get_logger().info('Published camera frame')

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
