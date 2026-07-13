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

import json
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
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32, String

BOUNDARY = 'frame'

# 우측 가이드 패널 (파라미터별 ↑/↓ 효과 + 추천 조합) — 정적 HTML 이라 부하 없음
GUIDE_HTML = """
<h3>🧭 튜닝 순서 (십자키 순서와 동일)</h3>
<p style="color:#bbb;margin:4px 0">STEP0 기초 → STEP1 추적(계단이 선을 무는가)
→ STEP2 중심(빨간점) → STEP3 제어(거동) → STEP4 PID(예비).
한 번에 하나만 바꾸고 같은 커브에서 재현 확인.</p>
<table>
<tr><th>파라미터</th><th class="up">올리면 ↑</th><th class="dn">내리면 ↓</th></tr>
<tr><th colspan="3">STEP 0. 기초 — 직진부터</th></tr>
<tr data-k="트림"><td><b>①트림</b><br>steer_trim</td>
  <td colspan="2">직진 실측으로 결정 — 쏠리는 반대방향으로. 좌우 코너 비대칭이면 재점검</td></tr>
<tr data-k="속도"><td><b>②속도</b><br>speed_safe</td>
  <td class="up">랩타임 단축</td>
  <td class="dn">커브 안정, 판단 여유 (⚠️ ~0.3 미만 출발불가)</td></tr>
<tr><th colspan="3">STEP 1. 추적 — 윈도우 계단이 선을 무는가</th></tr>
<tr data-k="윈도우폭"><td><b>③윈도우폭</b><br>margin</td>
  <td class="up">급커브 추적력 ↑, 끊긴 선 연결</td>
  <td class="dn">노이즈 오인 방지, 연산량 ↓</td></tr>
<tr data-k="초기반폭"><td><b>④초기반폭</b><br>win_half</td>
  <td class="up">커브 진입 검출력 ↑</td>
  <td class="dn">반대차선 오인 방지 (중앙 완충 ↑)</td></tr>
<tr data-k="초기높이"><td><b>⑤초기높이</b><br>win_h1</td>
  <td class="up">값↑=창 낮음: 오검출 ↓</td>
  <td class="dn">값↓=창 큼: 검출력 ↑</td></tr>
<tr data-k="윈도수"><td><b>⑥윈도수</b><br>nwindows</td>
  <td class="up">멀리까지 추적</td>
  <td class="dn">연산 절약 (⚠️ 개수×20 ≥ 480−ld 유지)</td></tr>
<tr data-k="minpix"><td><b>⑦minpix</b></td>
  <td class="up">몇 픽셀 노이즈에 끌려가는 것 방지</td>
  <td class="dn">0 = 희미한/끊긴 선도 인정</td></tr>
<tr><th colspan="3">STEP 2. 중심 추정 — 빨간점이 맞는 곳에 있는가</th></tr>
<tr data-k="차선폭"><td><b>⑧차선폭</b><br>road_width</td>
  <td colspan="2">BEV 실측과 일치가 정답 — 차 중앙일 때 빨간점 320 오도록</td></tr>
<tr data-k="ld"><td><b>⑨ld</b><br>circle_height</td>
  <td class="up">값↑=가까이: 차분, 출렁임 제거</td>
  <td class="dn">값↓=멀리: 코너 선반응 (⚠️ 윈도수 커버)</td></tr>
<tr><th colspan="3">STEP 3. 제어 — 주행 거동</th></tr>
<tr data-k="조향게인"><td><b>⑩조향게인</b><br>steering_gain</td>
  <td class="up">코너 추종력 ↑</td>
  <td class="dn">직선 지그재그 제거</td></tr>
<tr data-k="스로틀게인"><td><b>⑪스로틀게인</b><br>throttle_gain</td>
  <td class="up">전체 속도 스케일 ↑</td>
  <td class="dn">전 구간 일괄 감속</td></tr>
<tr data-k="최대스로틀"><td><b>⑫최대스로틀</b><br>max_throttle</td>
  <td class="up">상한 해제</td>
  <td class="dn">안전 상한 (사고 시 피해 제한)</td></tr>
<tr data-k="fast속도"><td><b>⑬fast속도</b><br>speed_fast</td>
  <td colspan="2">version: fast 모드에서만 사용</td></tr>
<tr><th colspan="3">STEP 4. PID — P제어로 안 잡힐 때만 (use_pid: true)</th></tr>
<tr data-k="PIDkp"><td><b>⑭PID kp</b></td>
  <td class="up">반응 세기 ↑</td><td class="dn">과반응 억제</td></tr>
<tr data-k="PIDkd"><td><b>⑮PID kd</b></td>
  <td class="up">급변 브레이크 — 지그재그 감쇠 (제일 먼저 만질 것)</td>
  <td class="dn">노이즈 민감도 ↓</td></tr>
<tr data-k="PIDki"><td><b>⑯PID ki</b></td>
  <td class="up">잔류 쏠림 제거 (아주 조금만)</td>
  <td class="dn">누적 출렁임 방지</td></tr>
</table>
<h3>🤝 같이 조정하면 좋은 조합</h3>
<table>
<tr><th>상황</th><th>조합</th></tr>
<tr><td>급커브 이탈</td><td>③윈도우폭↑ + ⑨ld값↑(가까이) + ②속도↓</td></tr>
<tr><td>코너 늦게 꺾음</td><td>⑨ld값↓(멀리) + ⑩게인↑ — 지그재그 시작되면 게인 한 스텝 back</td></tr>
<tr><td>직선 출렁임</td><td>⑩게인↓ + ⑨ld값↑(가까이) — 그래도 휘청이면 STEP4 PID (kd 부터)</td></tr>
<tr><td>속도 올리기</td><td>②속도↑ 하면 ⑩게인 살짝↓ + ⑨ld 멀리</td></tr>
<tr><td>ld 멀리 볼 때</td><td>⑥윈도수 같이↑ (개수×20 ≥ 480−ld)</td></tr>
<tr><td>커브 진입 미검출</td><td>④초기반폭↑ + ⑤초기높이↓ — 오인 생기면 반대로</td></tr>
</table>
<p style="color:#999">💡 PID 는 최후의 카드: use_pid 를 params.yaml 또는
<code>ros2 param set /lane_detection_node use_pid true</code> 로 켠 뒤 kd → kp → ki 순.
확정값은 mtune 종료 YAML → params.yaml 반영.</p>
"""

# 레이스 모니터 바: 미션/인지 상태 토픽 실시간 표시 (label, topic, 타입)
RACE_TOPICS = [
    ('MODE', '/mode', String),
    ('신호등', '/mission/traffic_state', String),
    ('교차로', '/mission/roundabout_state', String),
    ('장애물', '/mission/dynamic_state', String),
    ('YOLO', '/traffic_sign', String),
    ('아루코', '/aruco/visible', Bool),
    ('아루코ID', '/aruco/id', Int32),
    ('빨간구간', '/is_red', Bool),
    ('노랑px', '/yellow_pixels', Int32),
    ('조향적분', '/roundabout/steer_integral', Float32),
    ('차선중심', '/lane_x_location', Float32),
    ('차선기준', '/lane_topic', String),   # 슬라이딩윈도우 추종 기준 (LEFT/RIGHT/BOTH)
    ('회전t', '/roundabout/loop_elapsed', Float32),   # LOOP 경과 시간(초)
]

# 상태 바에 표시할 주요 파라미터 — 순서 = param_tuner 십자키 순서 = 가이드 순서
STATUS_PARAMS = [
    ('트림',     '/control_node',        'steer_trim'),
    ('속도',     '/lane_detection_node', 'speed_safe'),
    ('윈도우폭', '/lane_detection_node', 'sw_margin'),
    ('초기반폭', '/lane_detection_node', 'sw_win_half'),
    ('초기높이', '/lane_detection_node', 'sw_win_h1'),
    ('윈도수',   '/lane_detection_node', 'sw_nwindows'),
    ('minpix',   '/lane_detection_node', 'sw_minpix'),
    ('차선폭',   '/lane_detection_node', 'sw_road_width'),
    ('ld',       '/lane_detection_node', 'sw_circle_height'),
    ('조향게인', '/main_planner',        'steering_gain'),
    ('스로틀게인', '/main_planner',      'throttle_gain'),
    ('최대스로틀', '/control_node',      'max_throttle'),
    ('fast속도', '/lane_detection_node', 'speed_fast'),
    ('PIDkp',    '/lane_detection_node', 'pid_kp'),
    ('PIDkd',    '/lane_detection_node', 'pid_kd'),
    ('PIDki',    '/lane_detection_node', 'pid_ki'),
]


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

        # ---- 상태 바용: 주요 파라미터 폴링(1Hz) + e_stop/배터리 구독 ----
        self.status = {label: '?' for label, _, _ in STATUS_PARAMS}
        self.estop = None
        self.battery = None
        self._param_clients = {}
        for _, node_name, _ in STATUS_PARAMS:
            if node_name not in self._param_clients:
                self._param_clients[node_name] = self.create_client(
                    GetParameters, f'{node_name}/get_parameters')
        self.create_subscription(Bool, '/e_stop', self._on_estop, 10)
        self.create_subscription(Float32, '/battery/percent', self._on_battery, 10)

        # ---- 레이스 모니터: 미션/인지 토픽 실시간 수집 ----
        self.race = {}          # label -> (value, 수신시각 sec)
        for label, topic, msg_type in RACE_TOPICS:
            self.create_subscription(
                msg_type, topic,
                lambda msg, lb=label: self._on_race(lb, msg), 10,
            )
        # 튜너의 현재 선택 파라미터 (latched 발행에 맞춘 QoS)
        self.selected_label = None
        self._label_by_param = {p: label for label, _, p in STATUS_PARAMS}
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '/param_tuner/selected', self._on_selected, latched)

        # 회전교차로 판정 기준값 폴링 (진행률 게이지용)
        self.rb = {}
        self._rb_client = self.create_client(
            GetParameters, '/roundabout_mission/get_parameters')

        self.status_timer = self.create_timer(1.0, self._refresh_status)

        self.get_logger().info(
            f'web_viewer started: http://0.0.0.0:{self.port}  topics={topics}'
        )

    def _on_image(self, topic, msg):
        with self.cond:
            self.latest[topic] = bytes(msg.data)
            self.cond.notify_all()

    def _on_race(self, label, msg):
        v = msg.data
        if isinstance(v, bool):
            v = '●' if v else '—'
        elif isinstance(v, float):
            v = round(v, 1)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.race[label] = (v, now)

    def _on_estop(self, msg):
        self.estop = bool(msg.data)

    def _on_battery(self, msg):
        self.battery = float(msg.data)

    def _on_selected(self, msg):
        self.selected_label = self._label_by_param.get(msg.data)

    def _refresh_status(self):
        """노드별 GetParameters 비동기 호출 — 1Hz 라 부하 무시 수준."""
        by_node = {}
        for label, node_name, param in STATUS_PARAMS:
            by_node.setdefault(node_name, []).append((label, param))
        for node_name, items in by_node.items():
            client = self._param_clients[node_name]
            if not client.service_is_ready():
                for label, _ in items:
                    self.status[label] = '?'
                continue
            req = GetParameters.Request(names=[p for _, p in items])
            future = client.call_async(req)

            def done(fut, items=items):
                try:
                    values = fut.result().values
                except Exception:
                    return
                for (label, _), pv in zip(items, values):
                    if pv.type == ParameterType.PARAMETER_DOUBLE:
                        self.status[label] = round(pv.double_value, 4)
                    elif pv.type == ParameterType.PARAMETER_INTEGER:
                        self.status[label] = pv.integer_value
            future.add_done_callback(done)

        # 회전교차로 기준값 (임계/목표/1회전시간)
        if self._rb_client.service_is_ready():
            rb_names = ['yellow_arm_threshold', 'steer_integral_target', 'loop_sec']
            rb_future = self._rb_client.call_async(
                GetParameters.Request(names=rb_names))

            def rb_done(fut):
                try:
                    values = fut.result().values
                except Exception:
                    return
                for name, pv in zip(rb_names, values):
                    self.rb[name] = (round(pv.double_value, 2)
                                     if pv.type == ParameterType.PARAMETER_DOUBLE
                                     else pv.integer_value)
            rb_future.add_done_callback(rb_done)


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
            if path == '/status':
                return self._serve_status()
            if path in ('', '/index.html'):
                return self._serve_index()
            self.send_error(404)

        def _serve_status(self):
            # 레이스 토픽: 3초 이상 미수신이면 stale 표시
            now = node.get_clock().now().nanoseconds * 1e-9
            race = {}
            for label, _, _ in RACE_TOPICS:
                if label in node.race:
                    v, t = node.race[label]
                    race[label] = {'v': v, 'stale': (now - t) > 3.0}
                else:
                    race[label] = {'v': '?', 'stale': True}
            body = json.dumps({
                'params': node.status,
                'estop': node.estop,
                'battery': node.battery,
                'selected': node.selected_label,
                'race': race,
                'rb': node.rb,
            }).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_index(self):
            rows = ''.join(
                f'<h3>{t}</h3><img src="/stream{t}" style="max-width:100%">'
                for t in node.latest
            )
            status_bar = (
                '<div id="race" style="position:sticky;top:0;background:#001a00;'
                'padding:8px;font-family:monospace;font-size:15px;'
                'border-bottom:1px solid #464;z-index:10">레이스 상태 로딩중...</div>'
                '<div id="bar" style="position:sticky;top:60px;background:#111;'
                'padding:8px;font-family:monospace;font-size:14px;'
                'border-bottom:1px solid #444;z-index:9">파라미터 로딩중...</div>'
                '<script>'
                'const MODE_COLOR={LANE:"#5f5",SIGN:"#f55",DYNAMIC:"#fa0",'
                'ROUNDABOUT:"#5cf",RABACON:"#c8f",STATIC:"#c8f",TUNNEL:"#c8f",PARKING:"#c8f"};'
                'setInterval(()=>fetch("/status").then(r=>r.json()).then(s=>{'
                # 레이스 모니터 바 (500ms 갱신)
                'if(s.race){'
                'let items=Object.entries(s.race).map(([k,o])=>{'
                'let st=o.stale?"opacity:.35":"";'
                'if(k==="MODE"){let c=MODE_COLOR[o.v]||"#eee";'
                'return `<span style="${st};background:#222;border:1px solid ${c};color:${c};'
                'padding:1px 8px;border-radius:4px;font-weight:bold">${o.v}</span>`;}'
                'return `<span style="${st}">${k} <b style="color:#ff6">${o.v}</b></span>`;'
                '}).join(" · ");'
                # 회전교차로 해석 라인: 상태별 한글 문장 + 진행 게이지
                'let rb=s.rb||{},rc=s.race||{};'
                'let rst=String((rc["교차로"]||{}).v||"?");'
                'let lane=(rc["차선기준"]||{}).v||"BOTH";'
                'let yp=Number((rc["노랑px"]||{}).v)||0;'
                'let ig=Math.abs(Number((rc["조향적분"]||{}).v)||0);'
                'let thr=rb.yellow_arm_threshold||0,tgt=rb.steer_integral_target||0;'
                'let rl="",rcol="#888";'
                'if(rst.includes("disabled")){rl=`🔒 봉인됨 (enabled:false)`;}'
                'else if(rst==="IDLE"){rl=`⏸ 출발 대기 (DRIVING 신호 기다림)`;rcol="#aaa";}'
                'else if(rst==="ARMED"){let pct=thr?Math.min(100,Math.round(yp/thr*100)):0;'
                'rl=`👀 진입 감시중 — 노란링 ${yp}/${thr} (${pct}%)`;rcol="#fd5";}'
                'else if(rst==="LOOP"){let pct=tgt?Math.min(100,Math.round(ig/tgt*100)):0;'
                'let lt=Number((rc["회전t"]||{}).v)||0;let ls=rb.loop_sec||0;'
                'rl=`🌀 회전중 — <b>${lane}</b> 차선 추종 · 회전량 ${ig.toFixed(1)}/${tgt} (${pct}%)'
                ' · ⏱ ${lt.toFixed(1)}/${ls}s`;rcol="#5cf";}'
                'else if(rst==="EXIT"){rl=`↗ 탈출중 — <b>${lane}</b> 차선 + 바이어스 조향`;rcol="#fa0";}'
                'else if(rst==="DONE"){rl=`✅ 완료 — 일반 주행 복귀 (${lane})`;rcol="#5f5";}'
                'let rbline=`<span style="color:${rcol}">🌀 회전교차로: ${rl}</span>`;'
                'document.getElementById("race").innerHTML=`🏁 ${items}<br>${rbline}`;}'
                # 기존 파라미터 바
                'let e=s.estop===true?"⛔E-STOP ":"";'
                'let b=s.battery!==null?`🔋${s.battery.toFixed(0)}% `:"";'
                'let p=Object.entries(s.params).map(([k,v])=>k===s.selected'
                '?`<span style="background:#274;color:#ff6;padding:1px 5px;border-radius:4px">🎮${k} <b>${v}</b></span>`'
                ':`${k} <b>${v}</b>`).join(" · ");'
                'document.getElementById("bar").innerHTML='
                '`<span style="color:#f55">${e}</span><span style="color:#5f5">${b}</span>${p}`;'
                'document.querySelectorAll("#guide tr[data-k]").forEach(tr=>{'
                'tr.style.background=tr.dataset.k===s.selected?"#264":"";});'
                '}).catch(()=>{}),500);'
                '</script>'
            )
            guide = GUIDE_HTML
            body = (
                '<html><head><title>AI_moon viewer</title>'
                '<meta name="viewport" content="width=device-width, initial-scale=1">'
                '<style>'
                '#wrap{display:flex;flex-wrap:wrap;gap:12px;padding:8px}'
                '#video{flex:1 1 640px;max-width:820px}'
                '#guide{flex:1 1 340px;min-width:320px;font-size:13px;line-height:1.55}'
                '#guide table{border-collapse:collapse;width:100%}'
                '#guide td,#guide th{border:1px solid #444;padding:4px 6px;vertical-align:top}'
                '#guide th{background:#333}'
                '#guide h3{margin:14px 0 6px;color:#8cf}'
                '#guide .up{color:#7f7}#guide .dn{color:#f97}'
                '</style></head>'
                f'<body style="background:#222;color:#eee;margin:0;font-family:sans-serif">{status_bar}'
                f'<div id="wrap"><div id="video">{rows}</div>'
                f'<div id="guide">{guide}</div></div></body></html>'
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
