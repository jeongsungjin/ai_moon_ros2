#!/usr/bin/env python3
"""조종기 연결 진단 (JetRacer 2.4GHz 동글 + ShanWan 계열 패드).

동글(045e:028e)은 조종기가 꺼져 있어도 /dev/input/js0 으로 항상 잡히므로,
"진짜 붙었는가"는 버튼/스틱 이벤트가 실제로 수신되는지로 판정한다.

사용:
  python3 ~/ai_moon_ros2/tools/gamepad_test.py          # 10초 안에 아무 버튼이나 누르기
  python3 ~/ai_moon_ros2/tools/gamepad_test.py 30       # 대기 시간 30초
  python3 ~/ai_moon_ros2/tools/gamepad_test.py --live   # 무한 모니터 (Ctrl+C 종료)

판정:
  [1/3] 동글 USB 인식  → 안 되면: 동글 재삽입 / 다른 포트
  [2/3] js0 디바이스   → 안 되면: xpad 드라이버 문제 (재부팅)
  [3/3] 입력 이벤트    → 안 되면: 조종기 전원/페어링 문제 (동글은 정상)
"""

import os
import select
import struct
import sys
import time

JS_DEV = '/dev/input/js0'
DONGLE_ID = '045e'   # Vendor ID (Xbox360 클론 동글)

AXIS_NAMES = {0: 'L스틱X', 1: 'L스틱Y', 2: 'L2', 3: 'R스틱X', 4: 'R스틱Y',
              5: 'R2', 6: 'DPAD_X', 7: 'DPAD_Y'}
BTN_NAMES = {0: 'A', 1: 'B', 2: 'X', 3: 'Y', 4: 'L1', 5: 'R1',
             6: 'SELECT', 7: 'START', 8: 'HOME'}


def check_dongle():
    try:
        with open('/proc/bus/input/devices') as f:
            txt = f.read()
        for block in txt.split('\n\n'):
            if f'Vendor={DONGLE_ID}' in block:
                name = [l for l in block.splitlines() if l.startswith('N:')]
                return name[0][8:].strip('"') if name else '(이름 미확인)'
    except OSError:
        pass
    return None


def main():
    live = '--live' in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    wait_sec = float(args[0]) if args else 10.0

    # [1/3] 동글
    dongle = check_dongle()
    if not dongle:
        print('❌ [1/3] 동글 USB 미인식 — 동글을 뽑았다 다시 꽂거나 다른 포트로')
        sys.exit(1)
    print(f'✅ [1/3] 동글 인식: {dongle}')

    # [2/3] js0
    if not os.path.exists(JS_DEV):
        print(f'❌ [2/3] {JS_DEV} 없음 — 드라이버 미로드 (동글 재삽입 → 안 되면 재부팅)')
        sys.exit(1)
    print(f'✅ [2/3] {JS_DEV} 존재')

    # [3/3] 이벤트 수신 (init 이벤트 0x80 은 조종기 없이도 오므로 제외)
    fd = os.open(JS_DEV, os.O_RDONLY | os.O_NONBLOCK)
    if live:
        print('👀 라이브 모니터 — 조종기를 조작해봐 (Ctrl+C 종료)')
    else:
        print(f'👉 [3/3] {wait_sec:.0f}초 안에 조종기의 아무 버튼/스틱이나 움직여봐...')
    t0 = time.time()
    got = 0
    try:
        while live or time.time() - t0 < wait_sec:
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            data = os.read(fd, 8 * 64)
            for i in range(0, len(data) - 7, 8):
                _, value, typev, number = struct.unpack('IhBB', data[i:i + 8])
                if typev & 0x80:      # 초기화 이벤트 — 연결 증거 아님
                    continue
                if typev & 0x01:
                    print(f'   버튼 {BTN_NAMES.get(number, number)} = {"눌림" if value else "뗌"}')
                    got += 1
                elif typev & 0x02:
                    if abs(value) > 6000:   # 스틱 드리프트 잡음 무시
                        print(f'   축 {AXIS_NAMES.get(number, number)} = {value / 32767.0:+.2f}')
                        got += 1
                if got and not live and got >= 5:
                    break
            if got and not live and got >= 5:
                break
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)

    if got:
        print(f'✅ [3/3] 입력 이벤트 {got}건 수신 — 조종기 연결 정상! mtune 사용 가능')
    else:
        print('❌ [3/3] 이벤트 없음 — 동글은 정상, 조종기가 안 붙음.')
        print('   → 조종기 전원(HOME 버튼 3초) / 배터리 / 페어링 재시도 (아래 절차)')
        sys.exit(1)


if __name__ == '__main__':
    main()
