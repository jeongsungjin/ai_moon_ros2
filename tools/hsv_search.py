#!/usr/bin/env python3
"""HSV 임계값 탐색: 후보 HSV로 재이진화 → 고정 GT/유효마스크로 채점.

사용법: python3 tools/hsv_search.py <bag_dir_or_db3> [--jobs=N]
"""
import itertools
import multiprocessing as mp
import os
import pickle
import sqlite3
import sys
import warnings

import cv2
import numpy as np

warnings.filterwarnings('ignore')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from param_search2 import SW

SRC = np.float32([[128, 400], [200, 340], [440, 340], [520, 400]])
DST = np.float32([[160, 460], [160, 0], [480, 0], [480, 460]])
M = cv2.getPerspectiveTransform(SRC, DST)

SW_PARAMS = dict(circle_height=380, road_width=0.53, init_half=160,
                 win_h1=340, margin=60, minpix=0)
CH = 380

_G = {}


def _init(jpegs, gt, valid, step):
    _G.update(jpegs=jpegs, gt=gt, valid=valid, step=step)


def evaluate(hsv_params):
    jpegs, gt, valid, step = _G['jpegs'], _G['gt'], _G['valid'], _G['step']
    wl_v, wu_s, ol_s = hsv_params
    lower_w = np.array([0, 0, wl_v])
    upper_w = np.array([179, wu_s, 255])
    lower_o = np.array([5, ol_s, 80])
    upper_o = np.array([25, 255, 255])
    sw = SW(**SW_PARAMS)
    errs, dxs, mids, gtn = [], [], 0, 0
    prev_x, prev_valid = None, False
    for i in range(0, len(jpegs), step):
        frame = cv2.imdecode(np.frombuffer(jpegs[i], np.uint8), cv2.IMREAD_COLOR)
        frame = cv2.resize(frame, (640, 480))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv, lower_w, upper_w),
                              cv2.inRange(hsv, lower_o, upper_o))
        warped = cv2.warpPerspective(mask, M, (640, 480))
        ys, xs = (warped > 20).nonzero()
        x, cur = sw.slidewindow(ys.astype(np.int32), xs.astype(np.int32))
        g = gt[i]
        if g is not None and valid[i]:
            gtn += 1
            errs.append(abs(x - g))
            if cur == 'MID':
                mids += 1
        if prev_x is not None and valid[i] and prev_valid:
            dxs.append(abs(x - prev_x))
        prev_x, prev_valid = x, valid[i]
    if gtn == 0:
        return 0.0, hsv_params, {}
    acc = float(np.mean([max(0.0, 1 - e / 100) for e in errs]))
    cov = 1 - mids / gtn
    smooth = max(0.0, 1 - float(np.percentile(dxs, 95)) / 150) if dxs else 0.0
    score = 100 * (0.65 * acc + 0.20 * cov + 0.15 * smooth)
    detail = dict(acc=acc, cov=cov, smooth=smooth,
                  mae=float(np.mean(errs)), p95dx=float(np.percentile(dxs, 95)))
    return score, hsv_params, detail


def main():
    import glob
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    db = sys.argv[1]
    if os.path.isdir(db):
        db = glob.glob(os.path.join(db, '*.db3'))[0]
    jobs = 4
    for a in sys.argv[2:]:
        if a.startswith('--jobs='):
            jobs = int(a[len('--jobs='):])

    with open(db + '.pre.pkl', 'rb') as f:
        data = pickle.load(f)
    gt = data['gts'][CH]
    valid = list(np.load(db + '.valid_mask.npy'))

    conn = sqlite3.connect(db)
    cam_tid = next(tid for tid, name, _ in conn.execute(
        'SELECT id, name, type FROM topics') if name == '/camera/image/compressed')
    jpegs = [bytes(deserialize_message(bytes(d), CompressedImage).data)
             for _, d in conn.execute(
                 'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                 (cam_tid,))]
    print(f'{len(jpegs)} 프레임 로드 (JPEG)', flush=True)

    # (white V lower, white S upper, orange S lower) — 현재값 (180, 60, 80)
    grid = list(itertools.product([150, 165, 180, 195, 210],
                                  [40, 60, 80],
                                  [80, 120]))
    print(f'{len(grid)}개 HSV 조합 탐색 (step=3, jobs={jobs})...', flush=True)
    results = []
    with mp.Pool(jobs, initializer=_init, initargs=(jpegs, gt, valid, 3)) as pool:
        for n, res in enumerate(pool.imap_unordered(evaluate, grid)):
            results.append(res)
            if (n + 1) % 5 == 0:
                print(f'  {n+1}/{len(grid)}  현재 최고 {max(r[0] for r in results):.1f}점', flush=True)

    results.sort(key=lambda r: -r[0])
    print('\n=== 상위 10개 (white_V_lower, white_S_upper, orange_S_lower) ===')
    for score, p, d in results[:10]:
        print(f'{score:5.1f}점  V>{p[0]}, S<{p[1]}, oS>{p[2]}')
        print(f'        정확도 {d["acc"]:.2f} (MAE {d["mae"]:.0f}px) | '
              f'검출율 {d["cov"]:.2f} | 부드러움 {d["smooth"]:.2f} (p95Δ {d["p95dx"]:.0f}px)')


if __name__ == '__main__':
    main()
