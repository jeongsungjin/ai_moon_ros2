#!/usr/bin/env python3
"""웹 키보드 파라미터 튜너 — 조종기(2.4GHz) 혼선 대비 백업 (:8083).

mtune(param_tuner.py)과 동일 기능을 노트북 키보드로 조작한다. 대회장처럼
같은 2.4GHz 조종기가 많아 무선이 불안할 때 사용. 노트북↔보드 통신은
SSH/포트포워딩(TCP) 경유라 재전송이 되므로 패드 무선 링크보다 혼잡에 강하다.
params.yaml 은 절대 수정하지 않는다 — mtune 과 동일하게 메모리 값만 바꾸고
종료 시 붙여넣기용 YAML 을 출력한다.

사용:
  1) 주행 스택 실행 (mlane / mauto — 대상 노드들이 떠 있어야 함)
  2) ms && python3 ~/ai_moon_ros2/tools/web_tuner.py
  3) 브라우저에서 http://localhost:8083 열고 페이지 클릭(키 포커스)

키 (튜닝 모드 — 기본):
  ↑ / ↓        튜닝할 파라미터 선택
  → / ←        +큰 스텝 / -큰 스텝
  Shift+→/←    +미세 / -미세 (큰 스텝의 1/5)
  M            🕹 수동 조종 모드 전환
  Space        ⛔ E-STOP 토글
  V            현재 값 전체 재조회/출력

키 (수동 조종 모드 — M 으로 전환):
  W / S        전진 / 후진 (누르는 동안만)
  A / D        좌 / 우 조향 (누르는 동안만)
  ] / [        수동 속도 배율 + / -
  M      w      튜닝 모드 복귀 (수동 정지, 자율 재개)
  Space        ⛔ E-STOP (수동보다 우선)

안전 (데드맨):
  - 수동 모드에서 브라우저가 0.15초마다 입력 상태를 전송한다. 0.5초 이상
    끊기면(탭 닫힘·WiFi 끊김·절전) 수동 출력을 즉시 중립으로 만든다.
    수동 모드는 유지되므로 자율로 넘어가지 않고 차가 멈춘다.
  - 탭이 백그라운드로 가거나 포커스를 잃으면 브라우저가 즉시 중립을 보낸다.
"""

import json
import os
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402

from param_tuner import (  # noqa: E402
    MANUAL_INVERT_STEER, MANUAL_INVERT_THROTTLE, PARAM_TABLE, ParamTuner)

PORT = 8083
KEY_STEER_MAG = 0.7   # 키보드 조향 크기 — 스틱과 달리 on/off 라 고정값 (현장 조절)
DEADMAN_SEC = 0.5     # 이 시간 이상 브라우저 입력 없으면 수동 중립


class WebTuner(ParamTuner):
    def __init__(self):
        super().__init__()
        self.loglines = deque(maxlen=14)
        self.last_input_time = 0.0
        self._deadman_thread = threading.Thread(target=self._deadman_loop, daemon=True)
        self._deadman_thread.start()

    def log(self, msg):
        super().log(msg)
        if hasattr(self, 'loglines'):
            self.loglines.append(f'[{time.strftime("%H:%M:%S")}] {msg}')

    def _deadman_loop(self):
        while rclpy.ok():
            stale = (time.time() - self.last_input_time) > DEADMAN_SEC
            if self.manual_mode and stale and (self.manual_steer or self.manual_throttle):
                self.manual_steer = 0.0
                self.manual_throttle = 0.0
                self.log('⚠️ 브라우저 입력 끊김(데드맨) → 수동 중립')
            time.sleep(0.1)

    # ---------- 웹 API ----------
    def state_dict(self):
        params = []
        for i, (label, node_name, param, *_rest) in enumerate(PARAM_TABLE):
            params.append({'label': label, 'param': param,
                           'value': self.values.get((node_name, param), '?'),
                           'sel': i == self.selected})
        return {'params': params, 'estop': self.estop_active,
                'manual': self.manual_mode, 'scale': round(self.manual_scale, 2),
                'log': list(self.loglines)}

    def handle_action(self, k):
        if k == 'estop':
            self.estop_toggle()
        elif k == 'manual':
            self.manual_toggle()
        elif k == 'refresh':
            self.fetch_initial_values()
            self.show_all()
        elif k == 'scale+':
            self.manual_scale_adjust(+1)
        elif k == 'scale-':
            self.manual_scale_adjust(-1)
        elif self.manual_mode:
            pass   # 수동 모드에선 파라미터 조절 잠금 (mtune 과 동일)
        elif k == 'up':
            self.selected = (self.selected - 1) % len(PARAM_TABLE)
            self.show_selected()
        elif k == 'down':
            self.selected = (self.selected + 1) % len(PARAM_TABLE)
            self.show_selected()
        elif k == 'inc':
            self.adjust(+1)
        elif k == 'dec':
            self.adjust(-1)
        elif k == 'incf':
            self.adjust(+1, fine=True)
        elif k == 'decf':
            self.adjust(-1, fine=True)

    def handle_drive(self, data):
        self.last_input_time = time.time()
        if not self.manual_mode or self.estop_active:
            return
        s = max(-1.0, min(1.0, float(data.get('s', 0))))
        t = max(-1.0, min(1.0, float(data.get('t', 0))))
        if MANUAL_INVERT_STEER:
            s = -s
        if MANUAL_INVERT_THROTTLE:
            t = -t
        self.manual_steer = s * KEY_STEER_MAG
        self.manual_throttle = t * self.manual_scale


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<title>web_tuner :8083</title><style>
body{background:#14161a;color:#dde;font:14px/1.5 monospace;margin:0;padding:14px}
h3{margin:0 0 8px} .bar{display:flex;gap:8px;margin-bottom:10px;align-items:center}
.badge{padding:3px 10px;border-radius:4px;background:#2a2e36}
.estop{background:#c0392b;color:#fff;font-weight:bold}
.manual{background:#d68910;color:#fff;font-weight:bold}
table{border-collapse:collapse;width:100%;max-width:560px}
td{padding:2px 8px;border-bottom:1px solid #23262c}
tr.sel{background:#1b3a5c} tr.sel td:first-child::before{content:"▶ "}
#log{background:#0d0f12;padding:8px;margin-top:10px;white-space:pre-wrap;
     max-width:560px;min-height:150px;font-size:12px;color:#9ab}
.help{color:#778;font-size:12px;margin-top:8px;max-width:560px}
</style></head><body>
<h3>🎛 web_tuner <span style="color:#778">— 키보드 파라미터 튜너</span></h3>
<div class="bar">
 <span id="bEstop" class="badge">E-STOP: -</span>
 <span id="bMode" class="badge">모드: -</span>
 <span id="bScale" class="badge">배율: -</span>
</div>
<table id="tbl"></table>
<div id="log"></div>
<div class="help">
튜닝: ↑↓ 선택 · →← ±큰스텝 · Shift+→← ±미세 · V 재조회 |
공통: M 수동전환 · Space E-STOP |
수동: W/S 전후진 · A/D 조향 · ]/[ 배율 —
<b>페이지를 클릭해 키 포커스를 잡은 뒤 사용</b>. 영상은 :8080 mview 창을 옆에 두기.
</div>
<script>
let manual=false, held={w:0,s:0,a:0,d:0};
const post=(u,b)=>fetch(u,{method:'POST',body:JSON.stringify(b)});
async function act(k){try{const r=await post('/act',{k});render(await r.json());}catch(e){}}
function drive(){post('/drive',{s:(held.d?1:0)-(held.a?1:0),t:(held.w?1:0)-(held.s?1:0)});}
function render(st){
 manual=st.manual;
 const be=document.getElementById('bEstop');
 be.textContent='E-STOP: '+(st.estop?'⛔ 정지':'해제');
 be.className='badge'+(st.estop?' estop':'');
 const bm=document.getElementById('bMode');
 bm.textContent='모드: '+(st.manual?'🕹 수동조종':'🎛 튜닝');
 bm.className='badge'+(st.manual?' manual':'');
 document.getElementById('bScale').textContent='배율: '+st.scale;
 document.getElementById('tbl').innerHTML=st.params.map(p=>
  `<tr class="${p.sel?'sel':''}"><td>${p.label}</td><td>${p.param}</td><td><b>${p.value}</b></td></tr>`).join('');
 document.getElementById('log').textContent=st.log.join('\\n');
}
async function poll(){try{const r=await fetch('/state');render(await r.json());}catch(e){}}
document.addEventListener('keydown',e=>{
 if(e.repeat)return;
 const k=e.code;
 if(k==='Space'){e.preventDefault();act('estop');return;}
 if(k==='KeyM'){act('manual');return;}
 if(k==='KeyV'){act('refresh');return;}
 if(k==='BracketRight'){act('scale+');return;}
 if(k==='BracketLeft'){act('scale-');return;}
 if(manual){
  if(k==='KeyW'){held.w=1;drive();}
  else if(k==='KeyS'){held.s=1;drive();}
  else if(k==='KeyA'){held.a=1;drive();}
  else if(k==='KeyD'){held.d=1;drive();}
 }else{
  if(k==='ArrowUp'){e.preventDefault();act('up');}
  else if(k==='ArrowDown'){e.preventDefault();act('down');}
  else if(k==='ArrowRight'){e.preventDefault();act(e.shiftKey?'incf':'inc');}
  else if(k==='ArrowLeft'){e.preventDefault();act(e.shiftKey?'decf':'dec');}
 }
});
document.addEventListener('keyup',e=>{
 const k=e.code;
 if(k==='KeyW')held.w=0; else if(k==='KeyS')held.s=0;
 else if(k==='KeyA')held.a=0; else if(k==='KeyD')held.d=0;
 else return;
 if(manual)drive();
});
function neutral(){held={w:0,s:0,a:0,d:0};if(manual)drive();}
window.addEventListener('blur',neutral);
document.addEventListener('visibilitychange',()=>{if(document.hidden)neutral();});
setInterval(()=>{if(manual)drive();},150);
setInterval(poll,300);
poll();
</script></body></html>"""


def make_handler(tuner):
    lock = threading.Lock()   # HTTP 스레드들이 동시에 노드를 spin 하지 않게

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/':
                body = PAGE.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == '/state':
                self._json(tuner.state_dict())
            else:
                self.send_error(404)

        def _body(self):
            n = int(self.headers.get('Content-Length', 0) or 0)
            raw = self.rfile.read(n) if n else b''
            return json.loads(raw or b'{}')

        def do_POST(self):
            try:
                data = self._body()
            except Exception:
                self.send_error(400)
                return
            if self.path == '/act':
                with lock:
                    tuner.handle_action(str(data.get('k', '')))
                self._json(tuner.state_dict())
            elif self.path == '/drive':
                tuner.handle_drive(data)
                self._json({'ok': True})
            else:
                self.send_error(404)

    return Handler


def main():
    rclpy.init()
    tuner = WebTuner()
    tuner.log(f'변경 로그 파일: {tuner.log_path}')
    if not tuner.fetch_initial_values():
        tuner.log('일부 노드가 응답하지 않았습니다. 주행 스택(mlane 등)을 먼저 켜세요.')
    tuner.show_all()
    tuner.show_selected()

    server = ThreadingHTTPServer(('0.0.0.0', PORT), make_handler(tuner))
    tuner.log(f'web_tuner started: http://0.0.0.0:{PORT} (브라우저에서 열고 클릭)')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        tuner.final_summary()
        tuner.log_file.close()
        try:
            tuner.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
