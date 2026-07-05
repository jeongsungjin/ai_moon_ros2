#!/usr/bin/env python3
"""라바콘(주황색) 검출 노드 (ROS1 rubbercone_orange_detection.py 포팅).

변경점: 트랙바 -> ROS2 파라미터, 입력은 CompressedImage.
주황 픽셀 수가 임계값을 넘으면 /is_orange = True.
(기존 LiDAR 기반 rabacon_drive 를 대체할 카메라 기반 미션의 트리거로 사용)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


class RubberconeNode(Node):
    def __init__(self):
        super().__init__('rubbercone_node')

        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('process_hz', 30.0)
        self.declare_parameter('hsv_orange_lower', [0, 41, 223])
        self.declare_parameter('hsv_orange_upper', [66, 255, 255])
        self.declare_parameter('orange_pixel_threshold', 10000)

        self.lower_orange = np.array(self.get_parameter('hsv_orange_lower').value)
        self.upper_orange = np.array(self.get_parameter('hsv_orange_upper').value)
        self.orange_pixel_threshold = int(self.get_parameter('orange_pixel_threshold').value)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_topic = str(self.get_parameter('image_topic').value)
        self.create_subscription(CompressedImage, image_topic, self.image_callback, image_qos)

        self.is_orange_pub = self.create_publisher(Bool, '/is_orange', 1)
        self.orange_pixels_pub = self.create_publisher(Int32, '/orange_pixels', 1)

        self.cv_image = None

        process_hz = float(self.get_parameter('process_hz').value)
        self.timer = self.create_timer(1.0 / process_hz, self.process)

        self.get_logger().info(f'rubbercone_node started: image={image_topic}')

    def image_callback(self, msg: CompressedImage):
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            self.cv_image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warning(f'Error decoding image: {e}')

    def process(self):
        if self.cv_image is None:
            return

        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv_image, self.lower_orange, self.upper_orange)
        orange_pixel_counts = int(np.count_nonzero(orange_mask))

        self.is_orange_pub.publish(Bool(data=orange_pixel_counts > self.orange_pixel_threshold))
        self.orange_pixels_pub.publish(Int32(data=orange_pixel_counts))


def main(args=None):
    rclpy.init(args=args)
    node = RubberconeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
