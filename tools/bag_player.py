#!/usr/bin/env python3
"""웹 기반 rosbag 플레이어 (:8084) — ROS 노드 없이 bag 을 브라우저로 재생.

- ~/ai_moon_ros2/bags/ 의 bag 을 골라 재생/일시정지/스크럽(탐색)/배속
- 모드: 원본 / adaptive 마스크(블록·C 슬라이더) / 좌우 비교
  → 대회장 bag 하나면 차 없이 이진화 파라미터를 무한 재검증 가능
- bag(.db3, sqlite) 을 직접 읽으므로 ros2 bag play / 카메라 / 노드 전부 불필요

사용:
  ms && python3 ~/ai_moon_ros2/tools/bag_player.py      # → http://<보드IP>:8084
  python3 ~/ai_moon_ros2/tools/bag_player.py ~/path/to/bags   # bag 디렉토리 지정
"""

import glob
import json
import os
import sqlite3
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

PORT = 8084
TOPIC = '/camera/image/compressed'
BAGS_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/ai_moon_ros2/bags')
PARAMS_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', 'src', 'car_planner', 'config', 'params.yaml')
ADAPT_Y0 = 300   # lane_detection_node.adapt_y0 와 동일 (BEV 는 y>340 만 샘플)


def load_adaptive_defaults():
    try:
        with open(PARAMS_YAML) as f:
            p = yaml.safe_load(f)['lane_detection_node']['ros__parameters']
        return int(p.get('adaptive_block', 75)), int(p.get('adaptive_c', -18))
    except Exception:
        return 75, -18


class Bag:
    """bag 디렉토리의 .db3(sqlite) 들을 직접 인덱싱 — 임의 접근(스크럽) 지원."""

    def __init__(self, bag_dir):
        self.name = os.path.basename(bag_dir.rstrip('/'))
        self.index = []   # (db3경로, message rowid, timestamp_ns)
        self._conns = {}
        for db3 in sorted(glob.glob(os.path.join(bag_dir, '*.db3'))):
            conn = sqlite3.connect(f'file:{db3}?mode=ro', uri=True,
                                   check_same_thread=False)
            row = conn.execute('SELECT id FROM topics WHERE name=?', (TOPIC,)).fetchone()
            if row is None:
                conn.close()
                continue
            self._conns[db3] = conn
            for mid, ts in conn.execute(
                    'SELECT id, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp',
                    (row[0],)):
                self.index.append((db3, mid, ts))
        self.index.sort(key=lambda r: r[2])
        if len(self.index) >= 2:
            span = (self.index[-1][2] - self.index[0][2]) * 1e-9
            self.fps = max(1.0, (len(self.index) - 1) / max(span, 0.001))
        else:
            self.fps = 30.0

    def frame(self, i):
        db3, mid, _ = self.index[max(0, min(i, len(self.index) - 1))]
        data = self._conns[db3].execute(
            'SELECT data FROM messages WHERE id=?', (mid,)).fetchone()[0]
        msg = deserialize_message(bytes(data), CompressedImage)
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)

    def close(self):
        for c in self._conns.values():
            c.close()


def list_bags():
    out = []
    for d in sorted(glob.glob(os.path.join(BAGS_DIR, '*'))):
        if os.path.isdir(d) and glob.glob(os.path.join(d, '*.db3')):
            out.append(os.path.basename(d))
    return out


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.bag = None
        block, c = load_adaptive_defaults()
        self.block = block
        self.c = c

    def open(self, name):
        with self.lock:
            if self.bag is not None:
                self.bag.close()
            self.bag = Bag(os.path.join(BAGS_DIR, name))
            return {'name': self.bag.name, 'frames': len(self.bag.index),
                    'fps': round(self.bag.fps, 1)}

    def render(self, i, mode):
        with self.lock:
            bag, block, c = self.bag, max(3, self.block) | 1, self.c
        if bag is None or not bag.index:
            return None
        frame = bag.frame(i)
        if frame is None:
            return None
        if mode == 'raw':
            out = frame
        else:
            gray_roi = cv2.cvtColor(frame[ADAPT_Y0:], cv2.COLOR_BGR2GRAY)
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            mask[ADAPT_Y0:] = cv2.adaptiveThreshold(
                gray_roi, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
                block, c)
            cv2.line(mask, (0, ADAPT_Y0), (mask.shape[1], ADAPT_Y0), 60, 1)
            if mode == 'mask':
                out = mask
            else:   # side: 원본 | 마스크 좌우 비교
                out = np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        ok, jpeg = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpeg.tobytes() if ok else None


PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>bag player</title>
<style>
body { background:#1b1b1b; color:#eee; font-family:sans-serif; margin:12px; }
img { width:100%; max-width:960px; display:block; background:#000; min-height:240px; }
button, select { padding:8px 14px; margin:2px; border:0; border-radius:6px;
  background:#444; color:#eee; font-size:15px; }
button.on { background:#2d7; color:#000; font-weight:bold; }
.row { display:flex; align-items:center; gap:8px; margin:8px 0; flex-wrap:wrap; }
input[type=range] { flex:1; min-width:200px; }
#pos { min-width:110px; text-align:right; font-variant-numeric:tabular-nums; }
.dim { color:#999; font-size:13px; }
</style></head><body>
<div class="row">
  <select id="bags"></select><button onclick="openBag()">열기</button>
  <span id="info" class="dim"></span>
</div>
<img id="view">
<div class="row">
  <button id="play" onclick="toggle()">▶</button>
  <button onclick="step(-1)">◀ 1</button><button onclick="step(1)">1 ▶</button>
  <select id="speed"><option value="0.25">0.25x</option><option value="0.5">0.5x</option>
    <option value="1" selected>1x</option><option value="2">2x</option><option value="4">4x</option></select>
  <input type="range" id="seek" min="0" max="0" value="0"
         oninput="i=+this.value; showPos(); if(!playing) draw();">
  <span id="pos">- / -</span>
</div>
<div class="row">
  모드: <button id="m_raw" onclick="setMode('raw')">원본</button>
  <button id="m_mask" onclick="setMode('mask')">adaptive</button>
  <button id="m_side" onclick="setMode('side')">비교</button>
</div>
<div class="row" id="adrow">
  블록 <input type="range" id="block" min="3" max="201" oninput="setParam()">
  <span id="block_v"></span>
  C <input type="range" id="c" min="-60" max="10" oninput="setParam()">
  <span id="c_v"></span>
</div>
<div class="dim">스페이스=재생/정지, ←→=1프레임. adaptive 값은 lane_detection_node 와 동일 로직 (ROI 선 = y=300)</div>
<script>
let meta=null, i=0, playing=false, mode='side', timer=null, ptimer=null;
const $ = id => document.getElementById(id);
function showPos(){ $('pos').textContent = meta ? `${i} / ${meta.frames-1}` : '- / -';
                    $('seek').value = i; }
function draw(){
  if(!meta) return;
  $('view').src = `/frame.jpg?i=${i}&mode=${mode}&t=${Date.now()}`;
  showPos();
}
function tick(){
  if(!playing || !meta) return;
  const sp = +$('speed').value;
  i += Math.max(1, Math.round(sp));
  if(i >= meta.frames){ i = 0; }
  const img = $('view');
  img.onload = () => { timer = setTimeout(tick, 1000/(meta.fps*Math.min(sp,1))); };
  img.onerror = img.onload;
  draw();
}
function toggle(){ playing = !playing; $('play').textContent = playing?'❚❚':'▶';
                   clearTimeout(timer); if(playing) tick(); }
function step(d){ if(!meta) return; playing=false; $('play').textContent='▶';
                  i = Math.max(0, Math.min(meta.frames-1, i+d)); draw(); }
function setMode(m){ mode=m; ['raw','mask','side'].forEach(x =>
    $('m_'+x).className = x===m?'on':''); draw(); }
function setParam(){
  $('block_v').textContent = $('block').value; $('c_v').textContent = $('c').value;
  clearTimeout(ptimer);
  ptimer = setTimeout(()=>fetch(`/set?block=${$('block').value}&c=${$('c').value}`)
                      .then(()=>{ if(!playing) draw(); }), 120);
}
function openBag(){
  fetch(`/open?name=${encodeURIComponent($('bags').value)}`).then(r=>r.json()).then(m=>{
    meta=m; i=0; $('seek').max = m.frames-1;
    $('info').textContent = `${m.frames}프레임 @ ${m.fps}fps`;
    draw();
  });
}
document.addEventListener('keydown', e=>{
  if(e.code==='Space'){ e.preventDefault(); toggle(); }
  if(e.code==='ArrowLeft') step(-1);
  if(e.code==='ArrowRight') step(1);
});
fetch('/state').then(r=>r.json()).then(s=>{
  $('bags').innerHTML = s.bags.map(b=>`<option>${b}</option>`).join('');
  $('block').value=s.block; $('c').value=s.c;
  $('block_v').textContent=s.block; $('c_v').textContent=s.c;
  setMode('side');
  if(s.bags.length){ $('bags').value = s.bags[s.bags.length-1]; openBag(); }
});
</script></body></html>
"""


def make_handler(st: State):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            path = url.path.rstrip('/')
            try:
                if path in ('', '/index.html'):
                    self._send(PAGE.encode(), 'text/html; charset=utf-8')
                elif path == '/state':
                    self._send(json.dumps({
                        'bags': list_bags(), 'block': st.block, 'c': st.c,
                    }).encode(), 'application/json')
                elif path == '/open':
                    meta = st.open(q.get('name', ''))
                    self._send(json.dumps(meta).encode(), 'application/json')
                elif path == '/set':
                    with st.lock:
                        st.block = int(q.get('block', st.block))
                        st.c = int(q.get('c', st.c))
                    self._send(b'{"ok":true}', 'application/json')
                elif path == '/frame.jpg':
                    jpeg = st.render(int(q.get('i', 0)), q.get('mode', 'side'))
                    if jpeg is None:
                        self.send_error(404)
                    else:
                        self._send(jpeg, 'image/jpeg')
                else:
                    self.send_error(404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:
                try:
                    self.send_error(500, str(e))
                except Exception:
                    pass

    return Handler


def main():
    if not os.path.isdir(BAGS_DIR):
        print(f'bag 디렉토리 없음: {BAGS_DIR} (mbag 으로 먼저 녹화)')
        sys.exit(1)
    bags = list_bags()
    print(f'bag {len(bags)}개: {", ".join(bags) if bags else "(없음 — mbag 으로 녹화)"}')
    server = ThreadingHTTPServer(('0.0.0.0', PORT), make_handler(State()))
    print(f'bag player: http://0.0.0.0:{PORT}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
