#!/usr/bin/env python3
"""slidewindow 최적화(2026-07-10)의 출력 동일성 검증 — 누구든 재현 가능.

기준(oracle) = slidewindow_ref.py (최적화 직전, 실차를 몰던 버전의 박제).
현행 = src/lane_detection/lane_detection/slidewindow.py

검증 3종:
  1) 퍼징: 무작위/경계 입력에서 x_location·상태·디버그이미지 비트 비교
  2) 실주행 bag: bags/run_* 전 프레임 파이프라인 통과 후 비교
  3) 변이 테스트: 채점기(이 스크립트)가 심어놓은 미세 결함을 잡는지 자가 검증

사용:
  cd ~/ai_moon_ros2 && python3 tools/verify_slidewindow/verify.py           # 퍼징+변이 (빠름)
  python3 tools/verify_slidewindow/verify.py --bags                          # bag 전체까지 (수 분)
"""

import argparse
import glob
import importlib.util
import os
import random
import sys
import types
import warnings

import cv2
import numpy as np

warnings.filterwarnings('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
CUR_PATH = os.path.join(HERE, '..', '..', 'src', 'lane_detection', 'lane_detection', 'slidewindow.py')
REF_PATH = os.path.join(HERE, 'slidewindow_ref.py')


def load_module(name, path, mutate=None):
    src = open(path).read()
    if mutate:
        old, new = mutate
        assert old in src, f'변이 대상을 찾지 못함: {old}'
        src = src.replace(old, new, 1)
    m = types.ModuleType(name)
    exec(compile(src, path, 'exec'), m.__dict__)
    return m


def random_mask(rng):
    img = np.zeros((480, 640), np.uint8)
    k = random.choice(['empty', 'dots', 'line', 'curve', 'two', 'edge'])
    if k == 'dots':
        n = rng.integers(1, 2500)
        img[rng.integers(0, 480, n), rng.integers(0, 640, n)] = 1
    elif k == 'line':
        cv2.line(img, (int(rng.integers(0, 640)), 479), (int(rng.integers(0, 640)), 0), 1, int(rng.integers(2, 28)))
    elif k == 'curve':
        pts = np.array([[rng.integers(0, 640), y] for y in range(479, -1, -40)], np.int32)
        cv2.polylines(img, [pts], False, 1, int(rng.integers(2, 22)))
    elif k == 'two':
        for cx in (rng.integers(30, 300), rng.integers(340, 610)):
            cv2.line(img, (int(cx), 479), (int(cx + rng.integers(-150, 150)), 0), 1, int(rng.integers(3, 18)))
    elif k == 'edge':
        img[:, 0] = img[:, 639] = 1
        img[0, :] = img[479, :] = 1
    return img


def compare(ref_mod, cur_mod, n_frames, seed):
    rng = np.random.default_rng(seed)
    random.seed(seed)
    param_sets = [dict(
        margin=int(rng.integers(20, 151)), win_h1=int(rng.integers(150, 461)),
        win_half=int(rng.integers(60, 321)), circle_height=int(rng.integers(80, 460)),
        road_width=float(rng.uniform(0.3, 0.7)), nwindows=int(rng.integers(8, 25)),
        minpix=int(rng.choice([0, 1, 5, 20, 50])),
    ) for _ in range(30)]

    fails = 0
    for side in ('BOTH', 'LEFT', 'RIGHT'):
        a, b = ref_mod.SlideWindow(), cur_mod.SlideWindow()
        a.set_lane_side(side)
        b.set_lane_side(side)
        for _ in range(n_frames):
            ps = random.choice(param_sets)
            for k, v in ps.items():
                setattr(a, k, v)
                setattr(b, k, v)
            img = random_mask(rng)
            oa, xa, la = a.slidewindow(img.copy())
            ob, xb, lb = b.slidewindow(img.copy())
            if xa != xb or la != lb or not np.array_equal(oa, ob):
                fails += 1
    return fails


def run_bags(ref_mod, cur_mod):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    w_lo, w_up = np.array([0, 0, 180]), np.array([179, 60, 255])
    o_lo, o_up = np.array([5, 80, 80]), np.array([25, 255, 255])
    mat = cv2.getPerspectiveTransform(
        np.float32([[128, 400], [200, 340], [440, 340], [520, 400]]),
        np.float32([[160, 460], [160, 0], [480, 0], [480, 460]]))

    a, b = ref_mod.SlideWindow(), cur_mod.SlideWindow()
    total = fails = 0
    for bag in sorted(glob.glob(os.path.expanduser('~/ai_moon_ros2/bags/run_*'))):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic != '/camera/image/compressed':
                continue
            msg = deserialize_message(data, CompressedImage)
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            hsv = cv2.cvtColor(cv2.resize(img, (640, 480)), cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, w_lo, w_up) | cv2.inRange(hsv, o_lo, o_up)
            warped = cv2.warpPerspective(mask, mat, (640, 480))
            bin_img = np.zeros_like(warped)
            bin_img[warped > 20] = 1
            oa, xa, la = a.slidewindow(bin_img.copy())
            ob, xb, lb = b.slidewindow(bin_img.copy())
            total += 1
            fails += (xa != xb or la != lb or not np.array_equal(oa, ob))
        print(f'  {os.path.basename(bag)}: 누적 {total} 프레임', flush=True)
    return total, fails


MUTANTS = [
    ('경계 1px 오프바이원', ('r1 = min(h, y_hi)', 'r1 = min(h, y_hi + 1)')),
    ('x 경계 포함/제외 뒤집기', ('(bx >= win_x_low) & (bx < win_x_high)     # x 반개구간 (기준과 동일)',
                                  '(bx >= win_x_low) & (bx <= win_x_high)     # 변이')),
    ('polyfit 차수 변경', ('np.polyfit(left_y, left_x, 2)', 'np.polyfit(left_y, left_x, 1)')),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bags', action='store_true', help='실주행 bag 전체 검증까지 (수 분)')
    args = ap.parse_args()

    ref_mod = load_module('ref', REF_PATH)
    cur_mod = load_module('cur', CUR_PATH)
    ok = True

    print('[1/3] 퍼징 (3모드 × 700프레임 × 파라미터 30조합)...')
    f = compare(ref_mod, cur_mod, 700, seed=42)
    print(f'  → 불일치 {f}건', '✅' if f == 0 else '❌')
    ok &= (f == 0)

    print('[2/3] 변이 테스트 (채점기 자가 검증)...')
    for name, mut in MUTANTS:
        mf = compare(ref_mod, load_module('mut', CUR_PATH, mutate=mut), 150, seed=7)
        caught = mf > 0
        print(f'  → [{name}] {"✅ 검출" if caught else "❌ 놓침 — 채점기 결함!"} ({mf}건)')
        ok &= caught

    if args.bags:
        print('[3/3] 실주행 bag 전체...')
        total, f = run_bags(load_module('ref2', REF_PATH), load_module('cur2', CUR_PATH))
        print(f'  → {total} 프레임 중 불일치 {f}건', '✅' if f == 0 else '❌')
        ok &= (f == 0)
    else:
        print('[3/3] bag 검증 생략 (--bags 로 실행 가능)')

    print('\n최종:', '🟢 통과 — 현행 = 기준과 출력 동일' if ok else '🔴 실패 — 채택 불가')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
