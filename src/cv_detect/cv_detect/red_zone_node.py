#!/usr/bin/env python3
"""빨간 바닥(동적 장애물 구간) 인지 노드.

트랙의 빨간 페인트 구간을 HSV 로 검출해 /is_red 를 발행한다.
미션 노드는 is_red == true 일 때 동적 장애물(아루코) 대응을 준비(arm)한다.

- 빨강은 HSV Hue 가 0 근처에서 감기는(wrap) 색이라 두 구간(0~低, 高~179)의
  마스크를 합쳐서 판정한다.
- on/off 이중 임계값(히스테리시스)으로 경계에서 토픽이 깜빡이는 것을 방지.

발행:
  /is_red          (Bool)
  /red_zone_pixels (Int32)  ROI 내 빨간 픽셀 수 (임계값 튜닝용)
  /red_zone/image/debug (CompressedImage) 마스크 시각화
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


class RedZoneNode(Node):
    def __init__(self):
        super().__init__('red_zone_node')

        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('process_hz', 15.0)
        # ROI: resize(640x480) 기준 [y1, y2, x1, x2] — 바닥이 보이는 하단부
        self.declare_parameter('roi', [240, 480, 0, 640])
        # 빨강 hue wrap: 저구간 + 고구간
        self.declare_parameter('hsv_red1_lower', [0, 80, 60])
        self.declare_parameter('hsv_red1_upper', [10, 255, 255])
        self.declare_parameter('hsv_red2_lower', [160, 80, 60])
        self.declare_parameter('hsv_red2_upper', [179, 255, 255])
        # 히스테리시스: on 은 넘어야 켜지고, off 아래로 내려가야 꺼짐
        self.declare_parameter('on_pixel_threshold', 4000)
        self.declare_parameter('off_pixel_threshold', 1500)
        self.declare_parameter('publish_debug_image', True)

        image_topic = str(self.get_parameter('image_topic').value)
        process_hz = float(self.get_parameter('process_hz').value)
        self.roi = list(self.get_parameter('roi').value)
        self.lower_red1 = np.array(self.get_parameter('hsv_red1_lower').value)
        self.upper_red1 = np.array(self.get_parameter('hsv_red1_upper').value)
        self.lower_red2 = np.array(self.get_parameter('hsv_red2_lower').value)
        self.upper_red2 = np.array(self.get_parameter('hsv_red2_upper').value)
        self.on_pixel_threshold = int(self.get_parameter('on_pixel_threshold').value)
        self.off_pixel_threshold = int(self.get_parameter('off_pixel_threshold').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, image_topic, self.image_callback, image_qos)

        self.is_red_pub = self.create_publisher(Bool, '/is_red', 10)
        self.pixels_pub = self.create_publisher(Int32, '/red_zone_pixels', 10)
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(
                CompressedImage, '/red_zone/image/debug', image_qos
            )

        # 실시간 튜닝 (HSV/임계값)
        self.add_on_set_parameters_callback(self.on_param_change)

        self.raw_image = None
        self.is_red = False
        self.timer = self.create_timer(1.0 / process_hz, self.process)

        self.get_logger().info(
            f'red_zone_node started: roi={self.roi}, '
            f'on>{self.on_pixel_threshold}, off<{self.off_pixel_threshold}'
        )

    def image_callback(self, msg: CompressedImage):
        self.raw_image = msg.data

    def on_param_change(self, params):
        hsv_map = {
            'hsv_red1_lower': 'lower_red1', 'hsv_red1_upper': 'upper_red1',
            'hsv_red2_lower': 'lower_red2', 'hsv_red2_upper': 'upper_red2',
        }
        for p in params:
            if p.name in hsv_map:
                setattr(self, hsv_map[p.name], np.array(p.value))
            elif p.name in ('on_pixel_threshold', 'off_pixel_threshold'):
                setattr(self, p.name, int(p.value))
            elif p.name == 'roi':
                self.roi = list(p.value)
            self.get_logger().info(f'param updated: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def process(self):
        if self.raw_image is None:
            return
        frame = cv2.imdecode(np.frombuffer(self.raw_image, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        frame_resized = cv2.resize(frame, (640, 480))
        y1, y2, x1, x2 = self.roi
        cropped = frame_resized[y1:y2, x1:x2]

        img_hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(img_hsv, self.lower_red1, self.upper_red1),
            cv2.inRange(img_hsv, self.lower_red2, self.upper_red2),
        )
        red_pixels = int(np.count_nonzero(mask))

        # 히스테리시스 판정
        if self.is_red:
            if red_pixels < self.off_pixel_threshold:
                self.is_red = False
                self.get_logger().info(f'RED ZONE EXIT (pixels={red_pixels})')
        else:
            if red_pixels > self.on_pixel_threshold:
                self.is_red = True
                self.get_logger().info(f'RED ZONE ENTER (pixels={red_pixels})')

        self.is_red_pub.publish(Bool(data=self.is_red))
        self.pixels_pub.publish(Int32(data=red_pixels))

        if self.publish_debug_image:
            dbg_img = cv2.bitwise_and(cropped, cropped, mask=mask)
            ok, encoded = cv2.imencode('.jpg', dbg_img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                dbg = CompressedImage()
                dbg.header.stamp = self.get_clock().now().to_msg()
                dbg.header.frame_id = 'red_zone_debug'
                dbg.format = 'jpeg'
                dbg.data = encoded.tobytes()
                self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = RedZoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
