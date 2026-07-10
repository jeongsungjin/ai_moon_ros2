#!/usr/bin/env python3
"""아루코 마커 인지 노드 (동적 장애물 = 아루코 마커 부착 물체).

cv2.aruco 로 마커를 검출해 발행한다. 미션 판단은 하지 않는다 (인지 전용).

발행:
  /aruco/visible   (Bool)   이번 프레임에 크기 게이트를 통과한 마커 존재 여부
  /aruco/id        (Int32)  가장 큰 마커의 ID (없으면 -1)
  /aruco/height_px (Int32)  가장 큰 마커의 픽셀 높이 (없으면 0) — 근접도 지표
  /aruco/image/debug (CompressedImage) 검출 시각화

OpenCV 4.6 이하(cv2.aruco.detectMarkers)와 4.7+(ArucoDetector) 모두 지원.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


def make_detector(dict_name):
    """OpenCV 버전에 관계없이 (detect_fn, dictionary) 를 돌려준다."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    if hasattr(aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        return lambda gray: detector.detectMarkers(gray)[:2]
    params = aruco.DetectorParameters_create()   # OpenCV <= 4.6
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_node')

        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('process_hz', 15.0)
        # 대회 마커 사전은 사전 제공 정보 확인 후 확정 (기본 4x4_50)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('min_marker_height_px', 20)   # 이보다 작으면(멀면) 무시
        self.declare_parameter('publish_debug_image', True)

        image_topic = str(self.get_parameter('image_topic').value)
        process_hz = float(self.get_parameter('process_hz').value)
        dict_name = str(self.get_parameter('aruco_dict').value)
        self.min_marker_height_px = int(self.get_parameter('min_marker_height_px').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        self.detect = make_detector(dict_name)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, image_topic, self.image_callback, image_qos)

        self.visible_pub = self.create_publisher(Bool, '/aruco/visible', 10)
        self.id_pub = self.create_publisher(Int32, '/aruco/id', 10)
        self.height_pub = self.create_publisher(Int32, '/aruco/height_px', 10)
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(
                CompressedImage, '/aruco/image/debug', image_qos
            )

        self.raw_image = None
        self.timer = self.create_timer(1.0 / process_hz, self.process)

        self.get_logger().info(
            f'aruco_detect_node started: dict={dict_name}, '
            f'min_h={self.min_marker_height_px}px @{process_hz}Hz'
        )

    def image_callback(self, msg: CompressedImage):
        self.raw_image = msg.data   # 디코딩은 process() 에서 최신 1장만

    def process(self):
        if self.raw_image is None:
            return
        frame = cv2.imdecode(np.frombuffer(self.raw_image, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect(gray)

        best_id = -1
        best_h = 0
        if ids is not None and len(ids) > 0:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                pts = marker_corners.reshape(-1, 2)
                h = int(pts[:, 1].max() - pts[:, 1].min())
                if h > best_h:
                    best_h = h
                    best_id = int(marker_id)

        visible = best_h >= self.min_marker_height_px

        self.visible_pub.publish(Bool(data=bool(visible)))
        self.id_pub.publish(Int32(data=best_id if visible else -1))
        self.height_pub.publish(Int32(data=best_h))

        if visible:
            self.get_logger().info(
                f'ARUCO: id={best_id}, h={best_h}px', throttle_duration_sec=0.5
            )

        if self.publish_debug_image:
            annotated = frame
            if ids is not None and len(ids) > 0:
                annotated = frame.copy()
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            ok, encoded = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                dbg = CompressedImage()
                dbg.header.stamp = self.get_clock().now().to_msg()
                dbg.header.frame_id = 'aruco_debug'
                dbg.format = 'jpeg'
                dbg.data = encoded.tobytes()
                self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
