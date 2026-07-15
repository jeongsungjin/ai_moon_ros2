#!/usr/bin/env python3
"""웹 기반 HSV 튜너 (헤드리스 보드용).

lane_detection_node 와 독립적으로 카메라 영상을 직접 구독해서
브라우저 슬라이더로 HSV 범위를 실시간 조정하고 마스크를 확인한다.
확정된 값은 화면의 YAML 스니펫을 params.yaml 에 복사하면 된다.

'adaptive' 버튼 = 그레이스케일 국소 임계(adaptiveThreshold) 미리보기 —
lane_detection_node 의 lane_use_adaptive 경로와 동일 (블록/C 슬라이더).

사용:
  source ~/ai_moon_ros2/install/setup.bash
  python3 ~/ai_moon_ros2/tools/hsv_tuner.py
  → 폰/노트북 브라우저에서 http://<보드IP>:8081

카메라 노드(/camera/image/compressed)가 켜져 있어야 영상이 나온다:
  ros2 launch car_planner lane_drive.launch.py use_control:=false
"""

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

PORT = 8081
BOUNDARY = 'frame'
PARAMS_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'src', 'car_planner', 'config', 'params.yaml',
)

# params.yaml 의 hsv_<이름>_lower/upper 와 1:1 대응
COLOR_NAMES = ['yellow', 'left_yellow', 'white', 'red']
# 'adaptive' = 그레이스케일 국소 임계 미리보기 — lane_detection_node 의
# lane_use_adaptive 경로와 동일 로직 (블록/C 슬라이더로 튜닝)
MODES = COLOR_NAMES + ['adaptive']
ADAPT_Y0 = 300   # lane_detection_node.adapt_y0 와 반드시 일치 (BEV 는 y>340 만 샘플)

DEFAULTS = {
    'yellow': ([10, 108, 125], [35, 255, 255]),
    'left_yellow': ([15, 80, 90], [30, 255, 235]),
    'white': ([30, 0, 151], [122, 67, 207]),
    'red': ([145, 35, 35], [179, 255, 255]),
}


def load_initial_values():
    """params.yaml 의 현재 튜닝값에서 시작한다 (없으면 기본값)."""
    values = {c: {'lower': list(lo), 'upper': list(up)}
              for c, (lo, up) in DEFAULTS.items()}
    values['adaptive'] = {'block': 75, 'c': -18}
    try:
        with open(PARAMS_YAML) as f:
            params = yaml.safe_load(f)['lane_detection_node']['ros__parameters']
        for c in COLOR_NAMES:
            if f'hsv_{c}_lower' in params:
                values[c]['lower'] = list(params[f'hsv_{c}_lower'])
                values[c]['upper'] = list(params[f'hsv_{c}_upper'])
        values['adaptive'] = {'block': int(params.get('adaptive_block', 75)),
                              'c': int(params.get('adaptive_c', -18))}
    except Exception as e:
        print(f'params.yaml 읽기 실패, 기본값 사용: {e}')
    return values


class HsvTunerNode(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.declare_parameter('image_topic', '/camera/image/compressed')
        topic = str(self.get_parameter('image_topic').value)

        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.frame = None          # 최신 BGR 프레임
        self.values = load_initial_values()
        self.current = 'yellow'    # 지금 슬라이더로 만지는 색
        self.pixel_count = 0       # 현재 색 마스크 픽셀 수

        self.create_subscription(CompressedImage, topic, self._on_image, 1)
        self.get_logger().info(
            f'hsv_tuner started: http://0.0.0.0:{PORT}  (camera: {topic})'
        )

    def _on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self.cond:
            self.frame = frame
            self.cond.notify_all()

    def make_mask(self, frame):
        with self.lock:
            current = self.current
            if current == 'adaptive':
                block = max(3, int(self.values['adaptive']['block'])) | 1
                c = int(self.values['adaptive']['c'])
            else:
                lo = np.array(self.values[current]['lower'])
                up = np.array(self.values[current]['upper'])
        if current == 'adaptive':
            # lane_detection_node 의 lane_use_adaptive 경로와 동일 (ROI y>=ADAPT_Y0)
            gray_roi = cv2.cvtColor(frame[ADAPT_Y0:], cv2.COLOR_BGR2GRAY)
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            mask[ADAPT_Y0:] = cv2.adaptiveThreshold(
                gray_roi, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
                block, c)
            with self.lock:
                self.pixel_count = int(cv2.countNonZero(mask))
            return mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if lo[0] <= up[0]:
            mask = cv2.inRange(hsv, lo, up)
        else:  # H 범위가 0/179 를 감아 도는 경우 (빨강 계열)
            m1 = cv2.inRange(hsv, np.array([lo[0], lo[1], lo[2]]),
                             np.array([179, up[1], up[2]]))
            m2 = cv2.inRange(hsv, np.array([0, lo[1], lo[2]]),
                             np.array([up[0], up[1], up[2]]))
            mask = cv2.bitwise_or(m1, m2)
        with self.lock:
            self.pixel_count = int(cv2.countNonZero(mask))
        return mask

    def yaml_snippet(self):
        with self.lock:
            lines = []
            for c in COLOR_NAMES:
                lines.append(f"hsv_{c}_lower: {self.values[c]['lower']}")
                lines.append(f"hsv_{c}_upper: {self.values[c]['upper']}")
            lines.append(f"adaptive_block: {self.values['adaptive']['block']}")
            lines.append(f"adaptive_c: {self.values['adaptive']['c']}")
        return '\n'.join(lines)


PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HSV tuner</title>
<style>
body { background:#1b1b1b; color:#eee; font-family:sans-serif; margin:12px; }
img { width:100%; max-width:640px; display:block; margin-bottom:8px; }
.colors button { padding:8px 14px; margin:2px; border:0; border-radius:6px;
  background:#444; color:#eee; font-size:15px; }
.colors button.on { background:#2d7; color:#000; font-weight:bold; }
.slider { display:flex; align-items:center; gap:8px; margin:6px 0; }
.slider label { width:64px; }
.slider input { flex:1; }
.slider span { width:40px; text-align:right; }
pre { background:#2a2a2a; padding:10px; border-radius:6px; overflow-x:auto; }
#count { color:#2d7; }
</style></head><body>
<h3>원본 / 마스크 <small id="count"></small></h3>
<img src="/stream/original"><img src="/stream/mask">
<div class="colors" id="colors"></div>
<div id="sliders"></div>
<h3>params.yaml 에 붙여넣기</h3>
<pre id="yaml"></pre>
<script>
const CH_HSV = [['hl','H 하한',0,179],['sl','S 하한',0,255],['vl','V 하한',0,255],
                ['hh','H 상한',0,179],['sh','S 상한',0,255],['vh','V 상한',0,255]];
// adaptive: lane_detection_node 의 adaptive_block / adaptive_c 와 1:1
const CH_AD  = [['block','블록(홀수)',3,201],['c','C(음수=엄격)',-60,10]];
let state = null, timer = null;

function chans() { return state && state.current === 'adaptive' ? CH_AD : CH_HSV; }
function send() {
  const qs = chans().map(([k]) => `${k}=${document.getElementById(k).value}`).join('&');
  fetch(`/set?color=${state.current}&` + qs);
}
function onSlide() {
  chans().forEach(([k]) => document.getElementById(k+'_v').textContent
                           = document.getElementById(k).value);
  clearTimeout(timer); timer = setTimeout(send, 80);
}
function build() {
  document.getElementById('sliders').innerHTML = chans().map(([k,label,min,max]) =>
    `<div class="slider"><label>${label}</label>` +
    `<input type="range" id="${k}" min="${min}" max="${max}" oninput="onSlide()">` +
    `<span id="${k}_v"></span></div>`).join('');
}
function refresh() {
  fetch('/state').then(r => r.json()).then(s => {
    state = s;
    document.getElementById('colors').innerHTML = s.colors.map(c =>
      `<button class="${c===s.current?'on':''}" onclick="fetch('/select?color=${c}').then(refresh)">${c}</button>`).join('');
    build();
    let pairs;
    if (s.current === 'adaptive') {
      pairs = [['block', s.values.adaptive.block], ['c', s.values.adaptive.c]];
    } else {
      const [lo, up] = [s.values[s.current].lower, s.values[s.current].upper];
      pairs = [['hl',lo[0]],['sl',lo[1]],['vl',lo[2]],['hh',up[0]],['sh',up[1]],['vh',up[2]]];
    }
    pairs.forEach(([k,v]) => { document.getElementById(k).value = v;
                               document.getElementById(k+'_v').textContent = v; });
    document.getElementById('yaml').textContent = s.yaml;
  });
}
setInterval(() => fetch('/state').then(r => r.json()).then(s => {
  document.getElementById('count').textContent = `픽셀 ${s.pixel_count}`;
  document.getElementById('yaml').textContent = s.yaml;
}), 700);
build(); refresh();
</script></body></html>
"""


def make_handler(node: HsvTunerNode):
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
                with node.lock:
                    state = {
                        'colors': MODES,
                        'current': node.current,
                        'values': node.values,
                        'pixel_count': node.pixel_count,
                    }
                state['yaml'] = node.yaml_snippet()
                self._json(state)
            elif path == '/select':
                c = q.get('color')
                if c in MODES:
                    with node.lock:
                        node.current = c
                self._json({'ok': True})
            elif path == '/set':
                c = q.get('color')
                if c == 'adaptive':
                    try:
                        block = max(3, int(q['block'])) | 1
                        cval = int(q['c'])
                        with node.lock:
                            node.current = c
                            node.values['adaptive'] = {'block': block, 'c': cval}
                    except (KeyError, ValueError):
                        pass
                elif c in COLOR_NAMES:
                    try:
                        lo = [int(q['hl']), int(q['sl']), int(q['vl'])]
                        up = [int(q['hh']), int(q['sh']), int(q['vh'])]
                        with node.lock:
                            node.current = c
                            node.values[c] = {'lower': lo, 'upper': up}
                    except (KeyError, ValueError):
                        pass
                self._json({'ok': True})
            elif path == '/stream/original':
                self._serve_mjpeg(lambda f: f)
            elif path == '/stream/mask':
                self._serve_mjpeg(node.make_mask)
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
    node = HsvTunerNode()

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
