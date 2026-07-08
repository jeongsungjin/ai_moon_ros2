#!/usr/bin/env python3
"""bag 리플레이 기반 파라미터 자동 탐색 (main 로직 그대로, 숫자만 인자화).

사용법:
  python3 tools/param_search2.py <bag_dir_or_db3> [--grid='{json}'] [--step=N] [--jobs=N]

방식:
  1) bag 카메라 프레임을 노드와 동일하게 이진화 (white|orange mask -> warp -> gray>20)
     nonzero 좌표로 변환해 캐시(pre.pkl) — 2회차부터 즉시 시작
  2) 프레임마다 GT: y 밴드에서 클러스터 2개가 차로폭(280~420px)이면 중점
  3) 파라미터 조합마다 SlideWindow 리플레이, 점수 = 0.65*정확도 + 0.20*검출율 + 0.15*부드러움
"""
import itertools
import json
import multiprocessing as mp
import os
import pickle
import sqlite3
import sys
import warnings

import numpy as np

warnings.filterwarnings('ignore')

HEIGHT, WIDTH = 480, 640


# ---- main 브랜치 slidewindow.py 와 동일한 로직 (숫자만 인자, 입력은 nonzero 좌표) ----
class SW:
    def __init__(self, margin=40, minpix=0, nwindows=22, window_height=20,
                 win_l_center=145, win_r_center=495, init_half=80,
                 win_l_half=None, win_r_half=None,
                 win_h1=380, win_h2=480, circle_height=200, road_width=0.5, alpha=0.9):
        self.p = dict(margin=margin, minpix=minpix, nwindows=nwindows,
                      window_height=window_height, win_l_center=win_l_center,
                      win_r_center=win_r_center, init_half=init_half,
                      win_l_half=win_l_half or init_half, win_r_half=win_r_half or init_half,
                      win_h1=win_h1, win_h2=win_h2, circle_height=circle_height,
                      road_width=road_width, alpha=alpha)
        self.current_line = "DEFAULT"
        self.x_previous = 320

    def slidewindow(self, nonzeroy, nonzerox):
        p = self.p
        x_location = 320
        height, width = HEIGHT, WIDTH
        window_height = p['window_height']
        nwindows = p['nwindows']
        margin = p['margin']
        minpix = p['minpix']

        win_h1 = p['win_h1']
        win_h2 = p['win_h2']
        win_l_w_l = p['win_l_center'] - p['win_l_half']
        win_l_w_r = p['win_l_center'] + p['win_l_half']
        win_r_w_l = p['win_r_center'] - p['win_r_half']
        win_r_w_r = p['win_r_center'] + p['win_r_half']
        circle_height = p['circle_height']
        road_width = p['road_width']
        half_road_width = 0.5 * road_width

        good_left_inds = ((nonzerox >= win_l_w_l) & (nonzeroy <= win_h2)
                          & (nonzeroy > win_h1) & (nonzerox <= win_l_w_r)).nonzero()[0]
        left_found = len(good_left_inds) > 0
        left_lane_inds = good_left_inds

        good_right_inds = ((nonzerox >= win_r_w_l) & (nonzeroy <= win_h2)
                           & (nonzeroy > win_h1) & (nonzerox <= win_r_w_r)).nonzero()[0]
        right_found = len(good_right_inds) > 0
        right_lane_inds = good_right_inds

        if right_found:
            line_flag = 2
        elif left_found:
            line_flag = 1
        else:
            line_flag = 3

        y_current = height - 1

        if line_flag == 1 and len(left_lane_inds) > 0:
            x_current = int(np.mean(nonzerox[left_lane_inds]))
        elif line_flag == 2 and len(right_lane_inds) > 0:
            x_current = int(np.mean(nonzerox[right_lane_inds]))
        else:
            self.current_line = "MID"
            alpha = p['alpha']
            self.x_previous = int(alpha * self.x_previous + (1 - alpha) * x_location)
            x_location = self.x_previous
            return x_location, self.current_line

        self.current_line = "LEFT" if line_flag == 1 else "RIGHT"
        track_inds = left_lane_inds if line_flag == 1 else right_lane_inds
        p_fit = None

        for window in range(nwindows):
            win_y_low = y_current - (window + 1) * window_height
            win_y_high = y_current - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                         & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            if len(good_inds) > minpix:
                x_current = int(np.mean(nonzerox[good_inds]))
            elif len(track_inds) > 0:
                if p_fit is None:
                    p_fit = np.polyfit(nonzeroy[track_inds], nonzerox[track_inds], 2)
                x_current = int(np.polyval(p_fit, win_y_high))

            if circle_height - 10 <= win_y_low < circle_height + 10:
                if line_flag == 1:
                    x_location = int(x_current + width * half_road_width)
                else:
                    x_location = int(x_current - width * half_road_width)

        return x_location, self.current_line


LOWER_WHITE = np.array([0, 0, 180])
UPPER_WHITE = np.array([179, 60, 255])
LOWER_ORANGE = np.array([5, 80, 80])
UPPER_ORANGE = np.array([25, 255, 255])


def preprocess(db, cache):
    if os.path.exists(cache):
        with open(cache, 'rb') as f:
            return pickle.load(f)
    import cv2
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    conn = sqlite3.connect(db)
    cam_tid = next(tid for tid, name, _ in conn.execute(
        'SELECT id, name, type FROM topics') if name == '/camera/image/compressed')
    src = np.float32([[128, 400], [200, 340], [440, 340], [520, 400]])
    dst = np.float32([[160, 460], [160, 0], [480, 0], [480, 460]])
    m = cv2.getPerspectiveTransform(src, dst)
    nzs, bins = [], []
    for ts, data in conn.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (cam_tid,)):
        msg = deserialize_message(bytes(data), CompressedImage)
        frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        frame = cv2.resize(frame, (640, 480))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE),
                              cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE))
        warped = cv2.warpPerspective(mask, m, (640, 480))
        binary = (warped > 20).astype(np.uint8)
        ys, xs = binary.nonzero()
        nzs.append((ys.astype(np.int32), xs.astype(np.int32)))
        bins.append(binary)
    # GT 는 모든 후보 y 에 대해 미리 계산해 캐시에 포함
    gts = {}
    for y in range(100, 421, 20):
        gts[y] = [ground_truth(b, y) for b in bins]
    data = dict(nzs=nzs, gts=gts)
    with open(cache, 'wb') as f:
        pickle.dump(data, f)
    return data


def ground_truth(binary, y):
    band = binary[max(0, y - 10):y + 10]
    xs = np.where(band.any(axis=0))[0]
    return _gt_from_cols(xs)


def ground_truth_nz(ys, xs, y):
    sel = (ys >= max(0, y - 10)) & (ys < y + 10)
    return _gt_from_cols(np.unique(xs[sel]))


def _gt_from_cols(xs):
    if len(xs) < 6:
        return None
    splits = np.where(np.diff(xs) > 50)[0]
    clusters = np.split(xs, splits + 1)
    centers = [c.mean() for c in clusters if len(c) >= 3]
    if len(centers) < 2:
        return None
    lo, hi = min(centers), max(centers)
    if not (280 <= hi - lo <= 420):
        return None
    return (lo + hi) / 2


_G = {}


def _init(nzs, gts, step, valid):
    _G['nzs'] = nzs
    _G['gts'] = gts
    _G['step'] = step
    _G['valid'] = valid


def evaluate(params):
    nzs, gts, step = _G['nzs'], _G['gts'], _G['step']
    valid = _G['valid']  # 픽업(들어올림) 프레임 제외 마스크: 하단 픽셀 500개 미만
    sw = SW(**params)
    y = params.get('circle_height', 200)
    gt_list = gts[y]
    errs, dxs, mids, gtn = [], [], 0, 0
    prev_x, prev_valid = None, False
    for i in range(0, len(nzs), step):
        ys, xs = nzs[i]
        x, cur = sw.slidewindow(ys, xs)
        gt = gt_list[i]
        if gt is not None:
            gtn += 1
            errs.append(abs(x - gt))
            if cur == 'MID':
                mids += 1
        if prev_x is not None and valid[i] and prev_valid:
            dxs.append(abs(x - prev_x))
        prev_x, prev_valid = x, valid[i]
    if gtn == 0:
        return 0.0, params, {}
    acc = float(np.mean([max(0.0, 1 - e / 100) for e in errs]))
    cov = 1 - mids / gtn
    smooth = max(0.0, 1 - float(np.percentile(dxs, 95)) / 150) if dxs else 0.0
    score = 100 * (0.65 * acc + 0.20 * cov + 0.15 * smooth)
    detail = dict(acc=acc, cov=cov, smooth=smooth,
                  mae=float(np.mean(errs)), p95dx=float(np.percentile(dxs, 95)))
    return score, params, detail


def main():
    import glob
    db = sys.argv[1]
    if os.path.isdir(db):
        db = glob.glob(os.path.join(db, '*.db3'))[0]
    grid_json, step, jobs = None, 2, 4
    for a in sys.argv[2:]:
        if a.startswith('--grid='):
            grid_json = json.loads(a[len('--grid='):])
        if a.startswith('--step='):
            step = int(a[len('--step='):])
        if a.startswith('--jobs='):
            jobs = int(a[len('--jobs='):])

    cache = db + '.pre.pkl'
    print('전처리/캐시 로드 중...', flush=True)
    data = preprocess(db, cache)
    nzs, gts = data['nzs'], data['gts']
    mask_file = db + '.valid_mask.npy'
    if os.path.exists(mask_file):
        valid = list(np.load(mask_file))
    else:
        valid = [(ys > 380).sum() >= 500 for ys, _ in nzs]
    print(f'{len(nzs)} 프레임 (주행 유효 {sum(valid)}, 픽업 제외 {len(nzs)-sum(valid)})', flush=True)
    if grid_json:
        for y in set(grid_json.get('circle_height', [])) - set(gts):
            gts[y] = [ground_truth_nz(ys, xs, y) for ys, xs in nzs]
    for y in sorted(gts):
        n = sum(g is not None for g in gts[y])
        if n:
            print(f'  y={y}: GT {n}', flush=True)

    if grid_json:
        grid = grid_json
    else:
        grid = dict(
            circle_height=[120, 160, 200, 240, 280, 320, 360],
            road_width=[0.50, 0.53, 0.56],
            margin=[40, 60, 80],
            minpix=[0, 20],
        )

    keys = list(grid)
    combos = [dict(zip(keys, v)) for v in itertools.product(*grid.values())]
    print(f'{len(combos)}개 조합 탐색 (step={step}, jobs={jobs})...', flush=True)
    results = []
    with mp.Pool(jobs, initializer=_init, initargs=(nzs, gts, step, valid)) as pool:
        for n, res in enumerate(pool.imap_unordered(evaluate, combos)):
            results.append(res)
            if (n + 1) % 10 == 0:
                best = max(r[0] for r in results)
                print(f'  {n+1}/{len(combos)}  현재 최고 {best:.1f}점', flush=True)

    results.sort(key=lambda r: -r[0])
    print('\n=== 상위 10개 ===')
    for score, params, d in results[:10]:
        print(f'{score:5.1f}점  {params}')
        print(f'        정확도 {d["acc"]:.2f} (MAE {d["mae"]:.0f}px) | '
              f'검출율 {d["cov"]:.2f} | 부드러움 {d["smooth"]:.2f} (p95Δ {d["p95dx"]:.0f}px)')


if __name__ == '__main__':
    main()
