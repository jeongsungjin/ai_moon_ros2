#!/usr/bin/env python3
"""YOLO26 신호등/표지판 인지 노드.

학습된 yolo26n 모델을 로드해 카메라 영상에서 4개 클래스
(green / left / red / right) 를 추론하고, 검출된 클래스 이름을
/traffic_sign (std_msgs/String) 토픽으로 발행한다.

설계:
  - 이미지 콜백은 최신 프레임만 저장, 추론은 별도 타이머(infer_hz)로 실행
    (보드 성능에 맞게 추론 주기를 카메라 fps 와 독립적으로 조절)
  - 오검출 방지 2중 장치:
      * min_box_height_px : 멀리 있는(작은) 표지판 무시 — 가까워졌을 때만 반응
      * stable_frames     : 같은 클래스가 N회 연속 검출되어야 발행 (깜빡임 억제)
  - 검출 없으면 발행하지 않음 (구독측은 마지막 메시지 시각으로 판단 가능)
  - /yolo/image/debug 로 박스 그린 디버그 영상 발행 (웹 뷰어로 확인)

미션 연동 (추후):
  /traffic_sign 을 구독하는 미션 노드가 red→정지, green→출발,
  left/right→회전 미션을 수행하고 /motor_sign (DriveCommand) 발행
  → main_planner 가 SIGN 모드로 중재 (구조 이미 준비됨)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        self.declare_parameter('model_path', 'models/yolo26n_traffic.pt')
        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('detect_topic', '/traffic_sign')
        self.declare_parameter('infer_hz', 10.0)          # 보드 성능에 맞게 (D3-G/Jetson: 5~15)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('min_box_height_px', 30)   # 원본(640x480) 기준 최소 박스 높이
        self.declare_parameter('stable_frames', 3)        # N회 연속 검출 시 발행
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('device', '')              # ''=자동, 'cpu', '0'(cuda) 등

        model_path = str(self.get_parameter('model_path').value)
        image_topic = str(self.get_parameter('image_topic').value)
        detect_topic = str(self.get_parameter('detect_topic').value)
        infer_hz = float(self.get_parameter('infer_hz').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.min_box_height_px = int(self.get_parameter('min_box_height_px').value)
        self.stable_frames = int(self.get_parameter('stable_frames').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        device = str(self.get_parameter('device').value)

        if infer_hz <= 0.0:
            raise ValueError('infer_hz must be greater than 0')

        # ---------------- 모델 로드 ----------------
        try:
            from ultralytics import YOLO
        except ImportError:
            raise SystemExit(
                'ultralytics 미설치. 보드에서:  pip3 install "ultralytics>=8.3.200"'
            )

        import os
        if not os.path.exists(model_path):
            raise SystemExit(
                f'모델 파일 없음: {model_path}\n'
                '학습 후 tools/train_yolo26.py 가 models/yolo26n_traffic.pt 로 복사합니다.\n'
                '(또는 model_path 파라미터로 경로 지정)'
            )

        self.model = YOLO(model_path)
        self.device = device if device else None
        self.class_names = self.model.names  # {0:'green',1:'left',2:'red',3:'right'}
        self.get_logger().info(
            f'model loaded: {model_path}, classes={list(self.class_names.values())}'
        )

        # ---------------- 통신 ----------------
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, image_topic, self.image_callback, image_qos)
        self.detect_pub = self.create_publisher(String, detect_topic, 10)
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(
                CompressedImage, '/yolo/image/debug', image_qos
            )

        # ---------------- 상태 ----------------
        self.cv_image = None
        self.candidate_class = None   # 연속 검출 추적 중인 클래스
        self.candidate_count = 0

        self.timer = self.create_timer(1.0 / infer_hz, self.infer)

        self.get_logger().info(
            f'yolo_detect_node started: {image_topic} -> {detect_topic} '
            f'@{infer_hz}Hz, conf>={self.conf_threshold}, '
            f'min_h={self.min_box_height_px}px, stable={self.stable_frames}frames'
        )

    def image_callback(self, msg: CompressedImage):
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            self.cv_image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warning(f'Error decoding image: {e}')

    def infer(self):
        if self.cv_image is None:
            return
        frame = self.cv_image

        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )
        result = results[0]

        # 크기 조건을 통과한 검출 중 최고 confidence 클래스 선택
        best_cls = None
        best_conf = 0.0
        boxes = result.boxes
        if boxes is not None and len(boxes) > 0:
            for i in range(len(boxes)):
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                conf = float(boxes.conf[i])
                cls_id = int(boxes.cls[i])
                if (y2 - y1) < self.min_box_height_px:
                    continue  # 너무 멀리 있음 (작은 박스) → 무시
                if conf > best_conf:
                    best_conf = conf
                    best_cls = self.class_names[cls_id]

        # 연속 검출 debounce 후 발행
        if best_cls is not None:
            if best_cls == self.candidate_class:
                self.candidate_count += 1
            else:
                self.candidate_class = best_cls
                self.candidate_count = 1

            if self.candidate_count >= self.stable_frames:
                self.detect_pub.publish(String(data=best_cls))
                self.get_logger().info(
                    f'DETECT: {best_cls} (conf={best_conf:.2f})',
                    throttle_duration_sec=0.5,
                )
        else:
            self.candidate_class = None
            self.candidate_count = 0

        # 디버그 영상 (박스 렌더링)
        if self.publish_debug_image:
            annotated = result.plot()  # ultralytics 기본 시각화 (BGR)
            ok, encoded = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                dbg = CompressedImage()
                dbg.header.stamp = self.get_clock().now().to_msg()
                dbg.header.frame_id = 'yolo_debug'
                dbg.format = 'jpeg'
                dbg.data = encoded.tobytes()
                self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
