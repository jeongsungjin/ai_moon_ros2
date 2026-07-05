#!/usr/bin/env python3
"""D-Racer-Kit 규약 카메라 노드.

USB(V4L2) 또는 CSI(GStreamer) 카메라에서 프레임을 읽어
/camera/image/compressed (sensor_msgs/CompressedImage, jpeg) 로 발행한다.
"""

import cv2
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

        publish_topic = str(self.get_parameter('publish_topic').value)
        self.camera_type = camera_type = str(self.get_parameter('camera_type').value)
        camera_device = str(self.get_parameter('camera_device').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.flip_180 = bool(self.get_parameter('flip_180').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

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
        msg.data = encoded.tobytes()
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
