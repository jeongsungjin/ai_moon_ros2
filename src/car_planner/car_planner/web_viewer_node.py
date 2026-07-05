"""헤드리스 보드용 MJPEG 웹 뷰어.

보드에 디스플레이가 없어도 브라우저로 카메라/차선 디버그 영상을 볼 수 있다.
CompressedImage(jpeg) 토픽을 그대로 MJPEG 스트림으로 중계하므로 재인코딩 없음.

사용:
  ros2 run car_planner web_viewer_node
  → 브라우저에서 http://<보드IP>:8080  (인덱스에 토픽별 링크)

파라미터:
  port (int, 8080)
  topics (string[], ['/camera/image/compressed', '/lane_detection/image/debug'])
"""

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage

BOUNDARY = 'frame'


class WebViewerNode(Node):
    def __init__(self):
        super().__init__('web_viewer_node')

        self.declare_parameter('port', 8080)
        self.declare_parameter(
            'topics',
            ['/camera/image/compressed', '/lane_detection/image/debug'],
        )

        self.port = int(self.get_parameter('port').value)
        topics = list(self.get_parameter('topics').value)

        # 토픽별 최신 jpeg 바이트 + 프레임 갱신 알림
        self.latest = {t: None for t in topics}
        self.cond = threading.Condition()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        for t in topics:
            self.create_subscription(
                CompressedImage, t,
                lambda msg, topic=t: self._on_image(topic, msg), qos,
            )

        self.get_logger().info(
            f'web_viewer started: http://0.0.0.0:{self.port}  topics={topics}'
        )

    def _on_image(self, topic, msg):
        with self.cond:
            self.latest[topic] = bytes(msg.data)
            self.cond.notify_all()


def make_handler(node: WebViewerNode):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # 요청 로그 소음 제거
            pass

        def do_GET(self):
            path = self.path.split('?')[0].rstrip('/')
            # /stream<topic>  예: /stream/lane_detection/image/debug
            if path.startswith('/stream/'):
                topic = path[len('/stream'):]
                if topic in node.latest:
                    return self._serve_mjpeg(topic)
            if path in ('', '/index.html'):
                return self._serve_index()
            self.send_error(404)

        def _serve_index(self):
            rows = ''.join(
                f'<h3>{t}</h3><img src="/stream{t}" style="max-width:100%">'
                for t in node.latest
            )
            body = (
                '<html><head><title>AI_moon viewer</title></head>'
                f'<body style="background:#222;color:#eee">{rows}</body></html>'
            ).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_mjpeg(self, topic):
            self.send_response(200)
            self.send_header(
                'Content-Type',
                f'multipart/x-mixed-replace; boundary={BOUNDARY}',
            )
            self.end_headers()
            try:
                while rclpy.ok():
                    with node.cond:
                        node.cond.wait(timeout=1.0)
                        jpeg = node.latest.get(topic)
                    if jpeg is None:
                        continue
                    self.wfile.write(
                        f'--{BOUNDARY}\r\n'
                        f'Content-Type: image/jpeg\r\n'
                        f'Content-Length: {len(jpeg)}\r\n\r\n'.encode()
                    )
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass  # 브라우저 탭 닫힘

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = WebViewerNode()

    server = ThreadingHTTPServer(('0.0.0.0', node.port), make_handler(node))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
