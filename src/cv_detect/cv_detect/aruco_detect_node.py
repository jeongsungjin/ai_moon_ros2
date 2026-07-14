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

import array

import cv2

# 통합 스택(노드 5개×워커 4스레드)이 4코어에서 서로 선점하며 병렬 동기화 비용만 내는 것 방지
# (실측: 경합 시 차선 파이프라인 4T 65ms vs 1T 31ms — 격리 시엔 4T 18ms vs 1T 22ms 로 손해 미미)
cv2.setNumThreads(1)

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


def make_detector(dict_name, min_perimeter_rate=0.08, thresh_win_min=23, thresh_win_max=23):
    """OpenCV 버전에 관계없이 (detect_fn, dictionary) 를 돌려준다.

    검출 비용 튜닝 (실측: 기본값 21.9ms → 2.1ms/f, 640x480 의 1/2 해상도 기준):
      - min_perimeter_rate 0.08: 둘레가 이미지 폭의 8% 미만인 후보(≈변 6px 미만) 제거.
        6x6 사전은 ~24px 미만이면 어차피 디코딩 불가 + min_marker_height_px 게이트(기본 10px)보다
        작으므로 실검출 손실 없음 (40px 합성 마커 검출 유지 검증).
      - thresh_win 23 고정: 적응 임계 멀티스케일(3/13/23) → 단일 스케일. 조명이 극단적으로
        불균일하면 min=3, max=23 으로 원복 (ros2 param set 실시간 반영).
    """
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    if hasattr(aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        params = aruco.DetectorParameters()
    else:                                        # OpenCV <= 4.6
        params = aruco.DetectorParameters_create()
    params.minMarkerPerimeterRate = float(min_perimeter_rate)
    params.adaptiveThreshWinSizeMin = int(thresh_win_min)
    params.adaptiveThreshWinSizeMax = int(thresh_win_max)
    if hasattr(aruco, 'ArucoDetector'):
        detector = aruco.ArucoDetector(dictionary, params)
        return lambda gray: detector.detectMarkers(gray)[:2]
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_node')

        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('process_hz', 15.0)
        # 대회 마커 사전은 사전 제공 정보 확인 후 확정 (기본 4x4_50)
        self.declare_parameter('aruco_dict', 'DICT_6X6_50')
        self.declare_parameter('min_marker_height_px', 20)   # 이보다 작으면(멀면) 무시
        # 반응할 마커 ID 목록. [-1] = 전체 허용. 특정 ID만: 예 [3] 또는 [3, 7]
        # (현장에서 ros2 topic echo /aruco/id 로 장애물 마커 ID 확인 후 설정)
        self.declare_parameter('target_ids', [3])
        self.declare_parameter('publish_debug_image', True)
        # 검출용 축소 배율 (2=반해상도: 이진화/윤곽 비용 ~1/5 실측, 좌표·크기는 원해상도 환산).
        # 마커가 32px 미만으로 보이는 원거리 반응이 필요하면 1로 (기존 동작과 동일)
        self.declare_parameter('detect_downscale', 2)
        # 검출기 비용 튜닝 (make_detector docstring 참고 — 기본값 원복: rate 0.03, win 3/23)
        self.declare_parameter('min_perimeter_rate', 0.08)
        self.declare_parameter('thresh_win_min', 23)
        self.declare_parameter('thresh_win_max', 23)

        image_topic = str(self.get_parameter('image_topic').value)
        process_hz = float(self.get_parameter('process_hz').value)
        dict_name = str(self.get_parameter('aruco_dict').value)
        self.min_marker_height_px = int(self.get_parameter('min_marker_height_px').value)
        self.target_ids = [int(v) for v in self.get_parameter('target_ids').value]
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.detect_downscale = max(1, int(self.get_parameter('detect_downscale').value))

        self._dict_name = dict_name
        self.detect = self._build_detector()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,   # 최신 프레임만 사용 — 처리 지연 시 스테일 프레임 역직렬화 낭비 방지 (10→1, 통합 25Hz)
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

        # target_ids / min_marker_height_px 실시간 변경
        from rcl_interfaces.msg import SetParametersResult

        def on_param_change(params):
            rebuild = False
            for p in params:
                if p.name == 'target_ids':
                    self.target_ids = [int(v) for v in p.value]
                elif p.name == 'min_marker_height_px':
                    self.min_marker_height_px = int(p.value)
                elif p.name == 'detect_downscale':
                    self.detect_downscale = max(1, int(p.value))
                elif p.name in ('min_perimeter_rate', 'thresh_win_min', 'thresh_win_max'):
                    rebuild = True   # 검출기 파라미터는 재생성 필요 (아래에서 최신 값으로 일괄 반영)
                self.get_logger().info(f'param updated: {p.name} = {p.value}')
            if rebuild:
                # set_parameters 는 이 콜백 승인 후 반영되므로, 콜백 인자 값을 우선 사용
                override = {p.name: p.value for p in params}
                self.detect = self._build_detector(override)
            return SetParametersResult(successful=True)

        self.add_on_set_parameters_callback(on_param_change)

        self.raw_image = None
        self.timer = self.create_timer(1.0 / process_hz, self.process)

        self.get_logger().info(
            f'aruco_detect_node started: dict={dict_name}, '
            f'min_h={self.min_marker_height_px}px, target_ids={self.target_ids} '
            f'@{process_hz}Hz, downscale=1/{self.detect_downscale}'
        )

    def _build_detector(self, override=None):
        """현재 파라미터 값으로 검출기 (재)생성."""
        override = override or {}

        def val(name):
            return override.get(name, self.get_parameter(name).value)

        return make_detector(
            self._dict_name,
            min_perimeter_rate=float(val('min_perimeter_rate')),
            thresh_win_min=int(val('thresh_win_min')),
            thresh_win_max=int(val('thresh_win_max')),
        )

    def image_callback(self, msg: CompressedImage):
        self.raw_image = msg.data   # 디코딩은 process() 에서 최신 1장만

    def process(self):
        if self.raw_image is None:
            return
        buf = np.frombuffer(self.raw_image, dtype=np.uint8)
        # 축소 해상도에서 검출 (비용은 픽셀 수 비례) 후 좌표를 원해상도로 환산
        # → min_marker_height_px, 디버그 표시, /aruco/height_px 의미는 기존 그대로
        ds = self.detect_downscale
        frame = None
        if ds == 2 and not self.publish_debug_image:
            # JPEG 을 반해상도 흑백으로 직접 디코딩 (5.5ms→1.6ms 실측, 픽셀차 무시 수준)
            gray = cv2.imdecode(buf, cv2.IMREAD_REDUCED_GRAYSCALE_2)
        else:
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if frame is None:
                return
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if ds > 1:
                gray = cv2.resize(gray, (gray.shape[1] // ds, gray.shape[0] // ds))
        if gray is None:
            return
        corners, ids = self.detect(gray)
        if ds > 1:
            corners = [c * float(ds) for c in corners]

        accept_all = (len(self.target_ids) == 0) or (self.target_ids == [-1])

        best_id = -1
        best_h = 0
        if ids is not None and len(ids) > 0:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                # ID 필터: target_ids 에 없는 마커는 무시 ([-1] = 전체 허용)
                if not accept_all and int(marker_id) not in self.target_ids:
                    continue
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
                dbg.data = array.array('B', encoded.tobytes())   # fast-path (바이트 단위 검증 회피)
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
