#!/usr/bin/env python3
"""웹 기반 BEV/ROI 튜너 (헤드리스 보드용).

lane_detection_node 의 perspective transform 에 쓰는 src 사다리꼴을
브라우저 슬라이더로 움직이면서 (1) 원본 위 ROI 오버레이,
(2) 와핑된 BEV 결과를 실시간으로 확인한다.

판정 기준: BEV 화면에서 좌우 차선이 파란 세로 가이드선(x=160, 480) 위에
나란히 수직으로 서면 성공. 차선이 벌어지거나 기울면 꼭짓점을 조정한다.

사용:
  source ~/ai_moon_ros2/install/setup.bash
  python3 ~/ai_moon_ros2/tools/roi_tuner.py
  → 브라우저에서 http://<보드IP>:8082

확정값은 화면 하단 코드 스니펫을 lane_detection_node.py 의
src_points 블록에 반영 (로직 코드라 성진과 협의).
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

PORT = 8082
BOUNDARY = 'frame'

# lane_detection_node.py 의 현재 하드코딩 값
DEFAULTS = {
    'bot_lx': 128, 'bot_y': 400,
    'top_lx': 200, 'top_y': 340,
    'top_rx': 440,
    'bot_rx': 520,
}
DST = np.float32([[160, 460], [160, 0], [480, 0], [480, 460]])


class RoiTunerNode(Node):
    def __init__(self):
        super().__init__('roi_tuner')
        self.declare_parameter('image_topic', '/camera/image/compressed')
        topic = str(self.get_parameter('image_topic').value)

        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.frame = None
        self.roi = dict(DEFAULTS)

        self.create_subscription(CompressedImage, topic, self._on_image, 1)
        self.get_logger().info(
            f'roi_tuner started: http://0.0.0.0:{PORT}  (camera: {topic})'
        )

    def _on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        frame = cv2.resize(frame, (640, 480))
        with self.cond:
            self.frame = frame
            self.cond.notify_all()

    def src_points(self):
        with self.lock:
            r = dict(self.roi)
        return np.float32([
            [r['bot_lx'], r['bot_y']],
            [r['top_lx'], r['top_y']],
            [r['top_rx'], r['top_y']],
            [r['bot_rx'], r['bot_y']],
        ])

    def make_overlay(self, frame):
        pts = self.src_points().astype(int)
        out = frame.copy()
        cv2.polylines(out, [pts.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        for i, p in enumerate(pts):
            cv2.circle(out, tuple(p), 6, (0, 0, 255), -1)
            cv2.putText(out, str(i), (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return out

    def make_bev(self, frame):
        matrix = cv2.getPerspectiveTransform(self.src_points(), DST)
        bev = cv2.warpPerspective(frame, matrix, (640, 480))
        # dst 차선 기준선: 여기에 좌우 차선이 수직으로 서면 OK
        for gx in (160, 480):
            cv2.line(bev, (gx, 0), (gx, 480), (255, 128, 0), 1)
        return bev

    def snippet(self):
        with self.lock:
            r = dict(self.roi)
        return (
            'src_points = np.float32([\n'
            f"    [{r['bot_lx']}, {r['bot_y']}],\n"
            f"    [{r['top_lx']}, {r['top_y']}],\n"
            f"    [{r['top_rx']}, {r['top_y']}],\n"
            f"    [{r['bot_rx']}, {r['bot_y']}],\n"
            '])'
        )


PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ROI / BEV tuner</title>
<style>
body { background:#1b1b1b; color:#eee; font-family:sans-serif; margin:12px; }
img { width:100%; max-width:640px; display:block; margin-bottom:8px; }
.slider { display:flex; align-items:center; gap:8px; margin:6px 0; }
.slider label { width:130px; font-size:14px; }
.slider input { flex:1; }
.slider span { width:44px; text-align:right; }
pre { background:#2a2a2a; padding:10px; border-radius:6px; overflow-x:auto; }
button { padding:8px 14px; border:0; border-radius:6px; background:#444; color:#eee; }
small { color:#999; }
</style></head><body>
<h3>원본 + ROI (초록 사다리꼴)</h3>
<img src="/stream/overlay">
<h3>BEV 와핑 결과 <small>차선이 파란 세로선 위에 수직으로 서면 OK</small></h3>
<img src="/stream/bev">
<div id="sliders"></div>
<button onclick="reset()">기본값으로 리셋</button>
<h3>lane_detection_node.py 에 반영할 코드</h3>
<pre id="code"></pre>
<script>
const KEYS = [
  ['top_y',   '윗변 y',        479],
  ['top_lx',  '윗변 왼쪽 x',   639],
  ['top_rx',  '윗변 오른쪽 x', 639],
  ['bot_y',   '아랫변 y',      479],
  ['bot_lx',  '아랫변 왼쪽 x', 639],
  ['bot_rx',  '아랫변 오른쪽 x', 639],
];
let timer = null;

function send() {
  const q = KEYS.map(([k]) => `${k}=${document.getElementById(k).value}`).join('&');
  fetch('/set?' + q).then(r => r.json()).then(s =>
    document.getElementById('code').textContent = s.snippet);
}
function onSlide() {
  KEYS.forEach(([k]) => document.getElementById(k+'_v').textContent
                        = document.getElementById(k).value);
  clearTimeout(timer); timer = setTimeout(send, 80);
}
function reset() { fetch('/reset').then(refresh); }
function refresh() {
  fetch('/state').then(r => r.json()).then(s => {
    KEYS.forEach(([k]) => {
      document.getElementById(k).value = s.roi[k];
      document.getElementById(k+'_v').textContent = s.roi[k];
    });
    document.getElementById('code').textContent = s.snippet;
  });
}
document.getElementById('sliders').innerHTML = KEYS.map(([k,label,max]) =>
  `<div class="slider"><label>${label}</label>` +
  `<input type="range" id="${k}" min="0" max="${max}" oninput="onSlide()">` +
  `<span id="${k}_v"></span></div>`).join('');
refresh();
</script></body></html>
"""


def make_handler(node: RoiTunerNode):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, obj):
            body = json.dumps(obj).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _state(self):
            with node.lock:
                roi = dict(node.roi)
            return {'roi': roi, 'snippet': node.snippet()}

        def do_GET(self):
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            path = url.path.rstrip('/')

            if path in ('', '/index.html'):
                body = PAGE.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == '/state':
                self._json(self._state())
            elif path == '/reset':
                with node.lock:
                    node.roi = dict(DEFAULTS)
                self._json(self._state())
            elif path == '/set':
                try:
                    new = {k: max(0, int(q[k])) for k in DEFAULTS if k in q}
                    with node.lock:
                        node.roi.update(new)
                except ValueError:
                    pass
                self._json(self._state())
            elif path == '/stream/overlay':
                self._serve_mjpeg(node.make_overlay)
            elif path == '/stream/bev':
                self._serve_mjpeg(node.make_bev)
            else:
                self.send_error(404)

        def _serve_mjpeg(self, transform):
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
                        frame = node.frame
                    if frame is None:
                        continue
                    ok, jpeg = cv2.imencode(
                        '.jpg', transform(frame),
                        [cv2.IMWRITE_JPEG_QUALITY, 80],
                    )
                    if not ok:
                        continue
                    data = jpeg.tobytes()
                    self.wfile.write(
                        f'--{BOUNDARY}\r\n'
                        f'Content-Type: image/jpeg\r\n'
                        f'Content-Length: {len(data)}\r\n\r\n'.encode()
                    )
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    rclpy.init()
    node = RoiTunerNode()

    server = ThreadingHTTPServer(('0.0.0.0', PORT), make_handler(node))
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
