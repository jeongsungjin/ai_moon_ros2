#!/usr/bin/env python3
"""통합 스택 원샷 진단: 전 토픽 fps + 노드 CPU + 핵심 파라미터 + 프레임 크기.

사용: mauto(또는 mracenc) 켜둔 상태에서
  python3 ~/ai_moon_ros2/tools/stack_diag.py
"""
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32

MEASURE_SEC = 10.0

TOPICS = [
    ('카메라', '/camera/image/compressed', CompressedImage, 30),
    ('차선', '/lane_x_location', Float32, 30),
    ('아루코', '/aruco/visible', Bool, 15),
    ('YOLO', '/yolo/green', Bool, 2.5),
    ('빨간구간', '/is_red', Bool, 3),
]

NODE_PATTERNS = ['camera_', 'lane_de', 'yolo_de', 'aruco_d', 'red_zon',
                 'main_pl', 'roundab', 'dynamic', 'traffic_l', 'control_n', 'web_vie']


def main():
    rclpy.init()
    n = Node('stack_diag')
    counts = {name: 0 for name, *_ in TOPICS}
    sizes = []

    def make_cb(name, is_img):
        def cb(m):
            counts[name] += 1
            if is_img and len(sizes) < 5:
                sizes.append(len(m.data))
        return cb

    for name, topic, typ, _ in TOPICS:
        n.create_subscription(typ, topic, make_cb(name, typ is CompressedImage), 10)

    # DDS 매칭 대기: 고부하에선 새 구독자 연결에 수 초 걸림 — 전 토픽 수신 시작 후 측정
    print('발행자 연결 대기 (최대 15초)...')
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.05)
        if all(counts[name] > 0 for name, *_ in TOPICS):
            break
    for name, *_ in TOPICS:
        counts[name] = 0

    print(f'{MEASURE_SEC:.0f}초 측정 중...')
    t0 = time.time()
    while time.time() - t0 < MEASURE_SEC:
        rclpy.spin_once(n, timeout_sec=0.05)
    dt = time.time() - t0

    print('\n===== 토픽 처리율 =====')
    for name, topic, _, expect in TOPICS:
        hz = counts[name] / dt
        if hz == 0:
            npub = n.count_publishers(topic)
            mark = '❌ 발행자 없음 (노드 문제)' if npub == 0 else f'❌ 발행자 {npub}개 있음 — DDS 연결 실패'
        else:
            mark = '✅' if hz >= expect * 0.8 else '⚠️ 낮음'
        print(f'{name:6s} {hz:5.1f} Hz (기대 {expect})  {mark}')
    if sizes:
        print(f'카메라 프레임: {sum(sizes)//len(sizes)//1024}KB '
              f'({ "패스스루(원본)" if sizes[0] > 100_000 else "재인코딩(정상)"})')

    print('\n===== 노드 CPU (top 2회 평균) =====')
    out = subprocess.run(['top', '-bn2', '-d', '2'], capture_output=True, text=True).stdout
    lines = [l for l in out.splitlines() if any(p in l for p in NODE_PATTERNS)]
    seen = {}
    for l in lines:
        f = l.split()
        seen.setdefault(f[-1], []).append(float(f[8]))
    total = 0.0
    for name, vals in sorted(seen.items(), key=lambda kv: -max(kv[1])):
        avg = sum(vals[-1:]) / 1
        total += avg
        print(f'{name:16s} {avg:6.1f}%')
    print(f'{"합계":16s} {total:6.1f}% / 400%')

    print('\n===== 중복 노드 검사 (유령 스택 감지) =====')
    ps = subprocess.run(['ps', '-eo', 'pid,etime,comm'], capture_output=True, text=True).stdout
    procs = {}
    for line in ps.splitlines():
        f = line.split()
        if len(f) == 3 and any(p in f[2] for p in NODE_PATTERNS):
            procs.setdefault(f[2], []).append((f[0], f[1]))
    dup_found = False
    for comm, lst in procs.items():
        if len(lst) > 1:
            dup_found = True
            detail = ', '.join(f'PID {p}(가동 {e})' for p, e in lst)
            print(f'⚠️ {comm} × {len(lst)}개 — 유령 프로세스! 조향/토픽 오염 원인: {detail}')
    if dup_found:
        print('   → 오래된 PID 를 kill 하고 스택 재기동 필요')
    else:
        print('✅ 중복 없음')

    print('\n===== 핵심 파라미터 (실행 중 값) =====')
    for node, param in [('/camera_node', 'passthrough_mjpg'),
                        ('/aruco_detect_node', 'detect_downscale'),
                        ('/lane_detection_node', 'publish_debug_image'),
                        ('/yolo_detect_node', 'ncnn_threads')]:
        try:
            r = subprocess.run(['ros2', 'param', 'get', node, param],
                               capture_output=True, text=True, timeout=15)
            val = r.stdout.strip().split(': ')[-1] if r.returncode == 0 else '조회 실패'
        except subprocess.TimeoutExpired:
            val = '타임아웃 (부하 중 서비스 디스커버리 지연)'
        print(f'{node} {param} = {val}')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
