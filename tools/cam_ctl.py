#!/usr/bin/env python3
"""카메라 v4l2 컨트롤 조절 (v4l-utils 미설치 환경용 — ioctl 직접 호출).

camera_node 가 스트리밍 중이어도 라이브로 반영된다 (재시작 불필요).
대회장 강조명으로 노란 선 채도가 뭉개질 때 노출을 낮추는 용도.

사용:
  python3 ~/ai_moon_ros2/tools/cam_ctl.py                      # 현재 값 전체 출력
  python3 ~/ai_moon_ros2/tools/cam_ctl.py exposure_auto 1 exposure_absolute 200
  python3 ~/ai_moon_ros2/tools/cam_ctl.py saturation 160
  python3 ~/ai_moon_ros2/tools/cam_ctl.py --auto               # 자동노출 복귀

주요 컨트롤:
  exposure_auto      1=수동, 3=자동(기본)  ← 수동으로 바꿔야 exposure_absolute 가 먹음
  exposure_absolute  노출 시간 (C920: 3~2047, 낮을수록 어두움/블로우아웃 감소)
  saturation         채도 (0~255, 기본 128 — 올리면 노랑-흰색 분리 폭 증가)
  brightness / contrast / gain / backlight_comp
"""

import fcntl
import struct
import sys

DEV = '/dev/video1'
VIDIOC_G_CTRL = 0xc008561b
VIDIOC_S_CTRL = 0xc008561c

CIDS = {
    'brightness':        0x00980900,
    'contrast':          0x00980901,
    'saturation':        0x00980902,
    'wb_auto':           0x0098090c,
    'gain':              0x00980913,
    'wb_temp':           0x0098091a,
    'backlight_comp':    0x0098091c,
    'exposure_auto':     0x009a0901,
    'exposure_absolute': 0x009a0902,
}


def get_ctrl(fd, cid):
    buf = bytearray(struct.pack('=Ii', cid, 0))
    fcntl.ioctl(fd, VIDIOC_G_CTRL, buf)
    return struct.unpack('=Ii', buf)[1]


def set_ctrl(fd, cid, val):
    buf = bytearray(struct.pack('=Ii', cid, val))
    fcntl.ioctl(fd, VIDIOC_S_CTRL, buf)


def show_all(fd):
    for name, cid in CIDS.items():
        try:
            print(f'  {name:18s} = {get_ctrl(fd, cid)}')
        except OSError:
            print(f'  {name:18s} = (지원 안 함)')


def main():
    args = sys.argv[1:]
    fd = open(DEV, 'rb', buffering=0)
    try:
        if not args:
            print(f'[{DEV} 현재 값]')
            show_all(fd)
            return
        if args == ['--auto']:
            set_ctrl(fd, CIDS['exposure_auto'], 3)
            print('자동 노출로 복귀')
            return
        if len(args) % 2 != 0:
            print('사용: cam_ctl.py [이름 값]...  (인자 없으면 전체 출력)')
            sys.exit(1)
        # 수동 노출값을 주면 exposure_auto 1 을 먼저 자동 적용
        names = args[0::2]
        if 'exposure_absolute' in names and 'exposure_auto' not in names:
            set_ctrl(fd, CIDS['exposure_auto'], 1)
            print('exposure_auto = 1 (수동) 자동 적용')
        for name, val in zip(args[0::2], args[1::2]):
            if name not in CIDS:
                print(f'모르는 컨트롤: {name} (가능: {", ".join(CIDS)})')
                sys.exit(1)
            set_ctrl(fd, CIDS[name], int(val))
            print(f'{name} = {get_ctrl(fd, CIDS[name])}')
    finally:
        fd.close()


if __name__ == '__main__':
    main()
