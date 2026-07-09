#!/usr/bin/env python3
"""조종기 실시간 파라미터 튜너 (tuning 브랜치 전용 도구).

주행 중 게임패드로 주요 파라미터를 실시간 조절한다. params.yaml 은 절대
수정하지 않는다 — 돌고 있는 노드의 메모리 값만 바꾸고, 모든 변경을 로그로
남기며, 종료 시 최종 값을 params.yaml 에 붙여넣을 YAML 로 출력한다.
(노드를 재시작하면 params.yaml 의 원래 값으로 돌아간다.)

사용:
  1) 주행 스택 실행 (mlane / mauto — 대상 노드들이 떠 있어야 함)
  2) python3 ~/ai_moon_ros2/tools/param_tuner.py

조작법 (튜닝 모드 — 기본):
  십자키 ↑/↓   : 튜닝할 파라미터 선택
  R1 / L1      : +큰 스텝 / -큰 스텝   (십자키 →/← 도 동일)
  R2 / L2      : +미세 스텝 / -미세 스텝  (트리거, 큰 스텝의 1/5)
  A            : 🕹 수동 조종 모드 전환
  B            : ⛔ E-STOP 토글 (정지+바퀴 정렬 ↔ 다시 누르면 재개)
  SELECT       : 현재 값 전체 출력
  Ctrl+C       : 종료 → 최종 YAML 요약 출력

조작법 (수동 조종 모드 — A 로 전환, 자율주행 명령보다 우선):
  왼쪽 스틱 ↑↓ : 전진 / 후진
  오른쪽 스틱 ←→: 조향
  R1 / L1      : 수동 속도 배율 +/- (파라미터 조절은 잠김)
  A            : 튜닝 모드로 복귀 (수동 정지, 자율 재개)
  B            : ⛔ E-STOP (수동보다도 우선)
"""

import datetime
import os
import sys
import time

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Bool, String

from control_msgs.msg import Control

# 수동 조종 방향이 반대면 True 로 (현장에서 한 번 확인)
MANUAL_INVERT_STEER = False
MANUAL_INVERT_THROTTLE = False
MANUAL_DEADZONE = 0.08
MANUAL_SCALE_DEFAULT = 0.30   # 수동 스로틀 배율 시작값 (R1/L1 로 조절)
MANUAL_SCALE_STEP = 0.02

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gamepads import ShanWanGamepad  # noqa: E402  (tools/gamepads.py)

# (표시이름, 노드, 파라미터, 스텝, 최소, 최대, 정수여부, params.yaml 섹션)
# 순서 = 권장 튜닝/디버깅 순서 (mview 가이드 패널과 동일):
#   STEP0 기초 → STEP1 추적 → STEP2 중심(빨간점) → STEP3 제어 → STEP4 PID(예비)
PARAM_TABLE = [
    # -- STEP 0. 기초 (직진부터) --
    ('직진 트림',   '/control_node',        'steer_trim',       0.01,  -0.5,   0.5,   False, 'control_node'),
    ('주행 속도',   '/lane_detection_node', 'speed_safe',       0.01,   0.0,   0.5,   False, 'lane_detection_node'),
    # -- STEP 1. 추적 (윈도우 계단이 선을 무는가) --
    ('윈도우 폭',   '/lane_detection_node', 'sw_margin',        5,      20,    150,   True,  'lane_detection_node'),
    ('초기창 반폭', '/lane_detection_node', 'sw_win_half',      10,     60,    320,   True,  'lane_detection_node'),
    ('초기창 높이', '/lane_detection_node', 'sw_win_h1',        20,     150,   460,   True,  'lane_detection_node'),
    ('윈도우 개수', '/lane_detection_node', 'sw_nwindows',      1,      8,     24,    True,  'lane_detection_node'),
    ('minpix',      '/lane_detection_node', 'sw_minpix',        5,      0,     50,    True,  'lane_detection_node'),
    # -- STEP 2. 중심 추정 (빨간점 위치) --
    ('차선 폭',     '/lane_detection_node', 'sw_road_width',    0.01,   0.30,  0.70,  False, 'lane_detection_node'),
    ('ld(빨간점y)', '/lane_detection_node', 'sw_circle_height', 10,     80,    459,   True,  'lane_detection_node'),
    # -- STEP 3. 제어 (주행 거동) --
    ('조향 게인',   '/main_planner',        'steering_gain',    0.0005, 0.001, 0.008, False, 'main_planner'),
    ('스로틀 게인', '/main_planner',        'throttle_gain',    0.05,   0.5,   1.5,   False, 'main_planner'),
    ('최대 스로틀', '/control_node',        'max_throttle',     0.05,   0.2,   0.8,   False, 'control_node'),
    ('fast 속도',   '/lane_detection_node', 'speed_fast',       0.01,   0.0,   0.6,   False, 'lane_detection_node'),
    # -- STEP 4. PID (P제어로 안 잡힐 때만 — use_pid: true 필요) --
    ('PID kp',      '/lane_detection_node', 'pid_kp',           0.05,   0.1,   2.0,   False, 'lane_detection_node'),
    ('PID kd',      '/lane_detection_node', 'pid_kd',           0.05,   0.0,   1.0,   False, 'lane_detection_node'),
    ('PID ki',      '/lane_detection_node', 'pid_ki',           0.0005, 0.0,   0.01,  False, 'lane_detection_node'),
]

LOG_DIR = os.path.expanduser('~/ai_moon_ros2/tune_logs')


class ParamTuner(Node):
    def __init__(self):
        super().__init__('param_tuner')
        self.estop_pub = self.create_publisher(Bool, '/e_stop', 10)
        # 현재 선택 파라미터를 웹 뷰어에 알림 (latched — 늦게 접속해도 마지막 값 수신)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sel_pub = self.create_publisher(String, '/param_tuner/selected', latched)
        self.set_clients = {}
        self.get_clients = {}
        self.values = {}          # (node, param) -> 현재값
        self.selected = 0
        self.estop_active = False

        # 수동 조종 모드 상태 (A 버튼 토글)
        self.manual_mode = False
        self.manual_steer = 0.0
        self.manual_throttle = 0.0
        self.manual_scale = MANUAL_SCALE_DEFAULT
        self.manual_pub = self.create_publisher(Control, '/control_manual', 10)
        self._manual_thread = threading.Thread(target=self._manual_sender, daemon=True)
        self._manual_thread.start()

        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(LOG_DIR, f'tune_{stamp}.log')
        self.log_file = open(self.log_path, 'w')

        for _, node_name, *_ in PARAM_TABLE:
            if node_name not in self.set_clients:
                self.set_clients[node_name] = self.create_client(
                    SetParameters, f'{node_name}/set_parameters')
                self.get_clients[node_name] = self.create_client(
                    GetParameters, f'{node_name}/get_parameters')

    def log(self, msg):
        line = f'[{datetime.datetime.now().strftime("%H:%M:%S")}] {msg}'
        print(line, flush=True)
        self.log_file.write(line + '\n')
        self.log_file.flush()

    # ---------- 파라미터 서비스 ----------
    def call(self, client, request, timeout=2.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def fetch_initial_values(self):
        by_node = {}
        for _, node_name, param, *_ in PARAM_TABLE:
            by_node.setdefault(node_name, []).append(param)
        ok = True
        for node_name, names in by_node.items():
            req = GetParameters.Request(names=names)
            res = self.call(self.get_clients[node_name], req)
            if res is None:
                self.log(f'⚠️  {node_name} 응답 없음 — 노드가 켜져 있나요?')
                ok = False
                continue
            for name, pv in zip(names, res.values):
                val = pv.double_value if pv.type == ParameterType.PARAMETER_DOUBLE \
                    else pv.integer_value
                self.values[(node_name, name)] = val
        return ok

    def apply(self, node_name, param, value, is_int):
        pv = ParameterValue()
        if is_int:
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        req = SetParameters.Request(parameters=[Parameter(name=param, value=pv)])
        res = self.call(self.set_clients[node_name], req)
        return res is not None and res.results and res.results[0].successful

    # ---------- 조작 ----------
    def step_of(self, fine):
        step = PARAM_TABLE[self.selected][3]
        is_int = PARAM_TABLE[self.selected][6]
        if not fine:
            return step
        return max(1, round(step / 5)) if is_int else step / 5

    def show_selected(self):
        label, node_name, param, *_ = PARAM_TABLE[self.selected]
        val = self.values.get((node_name, param), '?')
        self.log(f'▶ 선택: {label} ({param}) = {val}  '
                 f'[R1/L1 ±{self.step_of(False)}, R2/L2 ±{self.step_of(True)}]')
        self.sel_pub.publish(String(data=param))

    def adjust(self, direction, fine=False):
        label, node_name, param, _, lo, hi, is_int, _ = PARAM_TABLE[self.selected]
        step = self.step_of(fine)
        cur = self.values.get((node_name, param))
        if cur is None:
            self.log(f'⚠️  {param} 현재값 없음 — 노드 연결 확인')
            return
        new = max(lo, min(hi, cur + direction * step))
        new = int(round(new)) if is_int else round(new, 5)
        if self.apply(node_name, param, new, is_int):
            self.values[(node_name, param)] = new
            arrow = '+' if direction > 0 else '-'
            self.log(f'{label} ({param}): {cur} → {new}  ({arrow}{step})')
        else:
            self.log(f'⚠️  {param} 적용 실패 — {node_name} 상태 확인')

    # ---------- 수동 조종 ----------
    def _manual_sender(self):
        """수동 모드일 때만 30Hz 로 /control_manual 발행 (평소엔 sleep 뿐)."""
        import time as _t
        while rclpy.ok():
            if self.manual_mode and not self.estop_active:
                msg = Control()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'manual'
                msg.steering = float(self.manual_steer)
                msg.throttle = float(self.manual_throttle)
                self.manual_pub.publish(msg)
            _t.sleep(1.0 / 30.0)

    def manual_toggle(self):
        self.manual_mode = not self.manual_mode
        self.manual_steer = 0.0
        self.manual_throttle = 0.0
        if self.manual_mode:
            self.log(f'🕹 수동 조종 모드 ON (속도배율 {self.manual_scale:.2f} — R1/L1 조절, A 로 복귀)')
        else:
            # 자율에 인계하기 전 중립 1회 발행 — 마지막 수동 명령이 남아 있지 않게
            msg = Control()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'manual'
            self.manual_pub.publish(msg)   # steering=0, throttle=0
            self.log('🎛 튜닝 모드 복귀 — 수동 중립 → 자율 명령 재개')

    def manual_scale_adjust(self, direction):
        self.manual_scale = max(0.05, min(0.5, self.manual_scale + direction * MANUAL_SCALE_STEP))
        self.log(f'🕹 수동 속도 배율 → {self.manual_scale:.2f}')

    def estop_toggle(self):
        # 디바운스: 버튼 튐/눌림+뗌 중복으로 정지→즉시해제 되는 것 방지
        now = time.time()
        if now - getattr(self, '_last_estop', 0.0) < 0.5:
            return
        self._last_estop = now
        self.estop_active = not self.estop_active
        self.manual_steer = 0.0     # 해제 직후 스틱 잔류값으로 튀지 않게 리셋
        self.manual_throttle = 0.0
        self.estop_pub.publish(Bool(data=self.estop_active))
        self.log('⛔ E-STOP!  (정지 + 바퀴 정렬 — B 다시 누르면 재개)'
                 if self.estop_active else '✅ E-STOP 해제 — 주행 재개')

    def show_all(self):
        self.log('--- 현재 값 ---')
        for label, node_name, param, *_ in PARAM_TABLE:
            self.log(f'  {label:10s} {param} = {self.values.get((node_name, param), "?")}')

    def final_summary(self):
        lines = ['', '=' * 46, ' 최종 튜닝 요약 — params.yaml 에 붙여넣기', '=' * 46]
        by_section = {}
        for label, node_name, param, _, _, _, _, section in PARAM_TABLE:
            val = self.values.get((node_name, param))
            if val is not None:
                by_section.setdefault(section, []).append((param, val))
        for section, items in by_section.items():
            lines.append(f'{section}:')
            lines.append('  ros__parameters:')
            for param, val in items:
                lines.append(f'    {param}: {val}')
        lines.append('=' * 46)
        lines.append(f'전체 변경 이력: {self.log_path}')
        for line in lines:
            print(line)
            self.log_file.write(line + '\n')
        self.log_file.flush()


def main():
    rclpy.init()
    tuner = ParamTuner()

    print(__doc__)
    tuner.log(f'변경 로그 파일: {tuner.log_path}')
    if not tuner.fetch_initial_values():
        tuner.log('일부 노드가 응답하지 않았습니다. 주행 스택(mlane 등)을 먼저 켜세요.')
    tuner.show_all()

    try:
        gamepad = ShanWanGamepad()
    except Exception as e:
        tuner.log(f'조종기 열기 실패: {e} (조종기 연결/전원 확인)')
        return

    tuner.show_selected()
    prev = {'up': None, 'down': None, 'left': None, 'right': None,
            'r1': None, 'l1': None, 'r2': None, 'l2': None,
            'a': None, 'b': None, 'select': None}

    def dz(v):
        v = float(v or 0.0)
        return 0.0 if abs(v) < MANUAL_DEADZONE else v

    try:
        while rclpy.ok():
            data = gamepad.read_data()   # 이벤트 1건 블로킹 수신

            def pressed(key, cur):
                hit = bool(cur) and not bool(prev[key])
                prev[key] = cur
                return hit

            # 모드 공통: A(모드 전환) / B(E-STOP) / SELECT
            if pressed('a', data.button_a):
                tuner.manual_toggle()
                continue
            if pressed('b', data.button_b):
                tuner.estop_toggle()
                continue
            if pressed('select', data.button_select):
                tuner.show_all()
                continue

            if tuner.manual_mode:
                # ---- 수동 조종 모드: 스틱 = 주행, R1/L1 = 속도 배율 전용 ----
                if pressed('r1', data.button_R1):
                    tuner.manual_scale_adjust(+1)
                elif pressed('l1', data.button_L1):
                    tuner.manual_scale_adjust(-1)
                s = dz(data.analog_stick_right.x)
                t = dz(data.analog_stick_left.y)
                if MANUAL_INVERT_STEER:
                    s = -s
                if MANUAL_INVERT_THROTTLE:
                    t = -t
                tuner.manual_steer = s
                tuner.manual_throttle = t * tuner.manual_scale
            else:
                # ---- 튜닝 모드: 파라미터 선택/조절 ----
                if pressed('up', data.dpad_up):
                    tuner.selected = (tuner.selected - 1) % len(PARAM_TABLE)
                    tuner.show_selected()
                elif pressed('down', data.dpad_down):
                    tuner.selected = (tuner.selected + 1) % len(PARAM_TABLE)
                    tuner.show_selected()
                elif pressed('right', data.dpad_right) or pressed('r1', data.button_R1):
                    tuner.adjust(+1)
                elif pressed('left', data.dpad_left) or pressed('l1', data.button_L1):
                    tuner.adjust(-1)
                elif pressed('r2', data.button_R2):
                    tuner.adjust(+1, fine=True)
                elif pressed('l2', data.button_L2):
                    tuner.adjust(-1, fine=True)
    except KeyboardInterrupt:
        pass
    finally:
        tuner.final_summary()
        tuner.log_file.close()
        try:
            tuner.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ctrl+C 시 rclpy 가 먼저 종료된 경우


if __name__ == '__main__':
    main()
