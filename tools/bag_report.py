#!/usr/bin/env python3
"""주행 rosbag 분석 리포트.

사용법:
  python3 tools/bag_report.py <bag.db3> [프레임저장폴더]

x_location/조향 시계열 요약 + 1초 단위 표 + x_location 극단 시점의
카메라 프레임을 저장한다.
"""
import sqlite3
import sys

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32, String
from sensor_msgs.msg import CompressedImage
from control_msgs.msg import Control
from drive_msgs.msg import DriveCommand

DB = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else '.'

TYPE_MAP = {
    'std_msgs/msg/Float32': Float32,
    'std_msgs/msg/String': String,
    'sensor_msgs/msg/CompressedImage': CompressedImage,
    'control_msgs/msg/Control': Control,
    'drive_msgs/msg/DriveCommand': DriveCommand,
}

conn = sqlite3.connect(DB)
topics = {tid: (name, typ) for tid, name, typ in
          conn.execute('SELECT id, name, type FROM topics')}
series = {name: [] for name, _ in topics.values()}
for tid, ts, data in conn.execute(
        'SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp'):
    name, typ = topics[tid]
    if typ not in TYPE_MAP:
        continue
    series[name].append((ts * 1e-9, deserialize_message(bytes(data), TYPE_MAP[typ])))

t0 = min(s[0][0] for s in series.values() if s)
xloc = np.array([(t - t0, m.data) for t, m in series['/lane_x_location']])
ctrl = np.array([(t - t0, m.steering, m.throttle) for t, m in series['/control']])

dur = xloc[-1, 0]
print(f'주행 {dur:.1f}s | x_location {len(xloc)/dur:.1f}Hz | control {len(ctrl)/ctrl[-1,0]:.1f}Hz')
print(f'x_location: mean {xloc[:,1].mean():.0f}  범위 {xloc[:,1].min():.0f}~{xloc[:,1].max():.0f}  std {xloc[:,1].std():.0f}')
print(f'steering  : mean {ctrl[:,1].mean():+.2f}  범위 {ctrl[:,1].min():+.2f}~{ctrl[:,1].max():+.2f}')
print()
print('  t | x_loc (min~max) | steer |  320 기준 위치')
for sec in range(int(dur) + 1):
    xs = xloc[(xloc[:, 0] >= sec) & (xloc[:, 0] < sec + 1), 1]
    ss = ctrl[(ctrl[:, 0] >= sec) & (ctrl[:, 0] < sec + 1), 1]
    if len(xs) == 0:
        continue
    pos = int(np.clip((xs.mean() - 100) / 10, 0, 45))
    print(f'{sec:3d} | {xs.mean():4.0f} ({xs.min():4.0f}~{xs.max():4.0f}) | {ss.mean():+.2f} |' + ' ' * pos + '#')

# x_location 극단 시점 프레임 저장 (최소/최대 각 2, 시작/끝)
cam = series['/camera/image/compressed']


def save_frame(t_want, tag):
    t_msg, msg = min(cam, key=lambda x: abs(x[0] - t0 - t_want))
    img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    path = f'{OUT}/f_{tag}_{t_msg - t0:.1f}s.png'
    cv2.imwrite(path, img)
    print(f'frame saved: {path}')


order = np.argsort(xloc[:, 1])
save_frame(xloc[order[0], 0], 'xmin')
save_frame(xloc[order[-1], 0], 'xmax')
save_frame(0.5, 'start')
save_frame(dur - 0.5, 'end')
