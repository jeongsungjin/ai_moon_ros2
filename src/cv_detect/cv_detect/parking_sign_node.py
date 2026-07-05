#!/usr/bin/env python3
"""주차 표지판(파란색) 검출 노드 (ROS1 parking_sign.py 포팅).

변경점: 트랙바 -> ROS2 파라미터 (헤드리스 Jetson 환경 대응),
입력은 CompressedImage. 터널 통과 후 일정 프레임 뒤부터 유효 판정하는
원본 로직 유지 (해당 미션이 없으면 require_tunnel_done=False 로 비활성화).
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


class ParkingSignNode(Node):
    def __init__(self):
        super().__init__('parking_sign_node')

        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('process_hz', 30.0)
        self.declare_parameter('hsv_blue_lower', [100, 50, 50])
        self.declare_parameter('hsv_blue_upper', [130, 255, 255])
        self.declare_parameter('blue_pixel_threshold', 1000)
        # ROI: resize(640x480) 기준 [y1, y2, x1, x2]
        self.declare_parameter('roi', [270, 331, 325, 640])
        self.declare_parameter('require_tunnel_done', True)
        self.declare_parameter('tunnel_pass_frames', 240)

        self.lower_blue = np.array(self.get_parameter('hsv_blue_lower').value)
        self.upper_blue = np.array(self.get_parameter('hsv_blue_upper').value)
        self.blue_pixel_threshold = int(self.get_parameter('blue_pixel_threshold').value)
        self.roi = list(self.get_parameter('roi').value)
        self.require_tunnel_done = bool(self.get_parameter('require_tunnel_done').value)
        self.tunnel_pass_frames = int(self.get_parameter('tunnel_pass_frames').value)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_topic = str(self.get_parameter('image_topic').value)
        self.create_subscription(CompressedImage, image_topic, self.image_callback, image_qos)
        self.create_subscription(Bool, '/tunnel_done', self.tunnel_done_callback, 10)

        self.is_blue_pub = self.create_publisher(Bool, '/is_blue', 1)
        self.blue_pixel_pub = self.create_publisher(Int32, '/blue_pixels', 1)

        self.cv_image = None
        self.tunnel_done_flag = False
        self.aruco_cnt = 0
        self.aruco_pass = False

        process_hz = float(self.get_parameter('process_hz').value)
        self.timer = self.create_timer(1.0 / process_hz, self.process)

        self.get_logger().info(f'parking_sign_node started: image={image_topic}')

    def image_callback(self, msg: CompressedImage):
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            self.cv_image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warning(f'Error decoding image: {e}')

    def tunnel_done_callback(self, msg: Bool):
        self.tunnel_done_flag = msg.data

    def process(self):
        if self.cv_image is None:
            return

        frame_resized = cv2.resize(self.cv_image, (640, 480))
        y1, y2, x1, x2 = self.roi
        frame_cropped = frame_resized[y1:y2, x1:x2]

        img_hsv = cv2.cvtColor(frame_cropped, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(img_hsv, self.lower_blue, self.upper_blue)
        blue_pixel_counts = int(np.count_nonzero(mask_blue))

        if self.require_tunnel_done:
            if self.tunnel_done_flag:
                self.aruco_cnt += 1
                if self.aruco_cnt >= self.tunnel_pass_frames and not self.aruco_pass:
                    self.aruco_pass = True
                    self.get_logger().info('Now accepting real parking sign detections')
            gate_open = self.tunnel_done_flag and self.aruco_pass
        else:
            gate_open = True

        is_blue = blue_pixel_counts > self.blue_pixel_threshold and gate_open

        self.is_blue_pub.publish(Bool(data=bool(is_blue)))
        self.blue_pixel_pub.publish(Int32(data=blue_pixel_counts))


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
