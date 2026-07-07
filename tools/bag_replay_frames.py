#!/usr/bin/env python3
"""bag 의 카메라 프레임을 lane_detection 파이프라인에 오프라인 통과.

사용법:
  python3 tools/bag_replay_frames.py <bag.db3> <출력폴더> <초1> [초2 ...]

각 시각의 원본 + 슬라이딩윈도우 디버그 이미지를 저장하고 x_location 을 출력.
노드와 동일한 HSV/warp 파라미터 사용 (params.yaml 의 기본 흰색/주황 범위).
"""
import sqlite3
import sys

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

sys.path.insert(0, '/home/topst/ai_moon_ros2/src/lane_detection')
from lane_detection.slidewindow import SlideWindow

DB, OUT = sys.argv[1], sys.argv[2]
WANT = [float(s) for s in sys.argv[3:]]

# params.yaml 과 동일한 값
LOWER_WHITE = np.array([0, 0, 180])
UPPER_WHITE = np.array([179, 60, 255])
LOWER_ORANGE = np.array([5, 80, 80])
UPPER_ORANGE = np.array([25, 255, 255])


def process(frame, sw):
    frame = cv2.resize(frame, (640, 480))
    y, x = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE),
        cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE),
    )
    filtered = cv2.bitwise_and(frame, frame, mask=mask)
    left_margin, top_margin = 200, 340
    src = np.float32([[128, 400], [left_margin, top_margin],
                      [x - left_margin, top_margin], [520, 400]])
    dst = np.float32([[x // 4, 460], [x // 4, 0],
                      [x // 4 * 3, 0], [x // 4 * 3, 460]])
    m = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(filtered, m, (640, 480))
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    binary = np.zeros_like(gray)
    binary[gray > 60] = 1
    out_img, x_loc, cur = sw.slidewindow(binary)
    return out_img, x_loc, cur


conn = sqlite3.connect(DB)
cam_tid = next(tid for tid, name, _ in conn.execute('SELECT id, name, type FROM topics')
               if name == '/camera/image/compressed')
rows = list(conn.execute(
    'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (cam_tid,)))
t0 = rows[0][0] * 1e-9

sw = SlideWindow()
# x_previous 연속성을 위해 요청 시각까지 전 프레임을 순서대로 통과시킨다
targets = sorted(WANT)
ti = 0
for ts, data in rows:
    t = ts * 1e-9 - t0
    if ti >= len(targets):
        break
    msg = deserialize_message(bytes(data), CompressedImage)
    frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    out_img, x_loc, cur = process(frame, sw)
    if t >= targets[ti]:
        cv2.imwrite(f'{OUT}/replay_{t:.1f}s_raw.png', cv2.resize(frame, (640, 480)))
        cv2.imwrite(f'{OUT}/replay_{t:.1f}s_debug.png',
                    (out_img * 255 if out_img.max() <= 1 else out_img).astype(np.uint8))
        print(f'{t:6.1f}s  x_location={x_loc}  current_line={cur}')
        ti += 1
