#!/usr/bin/env python3
"""조종기 강제 재바인딩 도구 (2.4GHz 클론 동글 USB 리셋 반복).

원리: 이 동글(045e:028e 클론)은 바인딩을 펌웨어가 알아서 하고 호스트는 관여 못 함.
단, USB 재열거(authorized 0→1) 를 하면 동글이 바인딩 탐색을 처음부터 다시 하므로,
"조종기를 켜서 동글 옆 10cm 에 둔 순간" 탐색을 열어주면 가장 신호가 센(=가까운)
우리 조종기가 경쟁에서 이길 확률이 최대가 된다. 이 도구는 그 리셋→감지 사이클을
자동 반복한다.

사용 (root 필요 — 알아서 sudo 로 재실행):
  python3 ~/ai_moon_ros2/tools/gamepad_pair.py        # 기본 6회 시도
  python3 ~/ai_moon_ros2/tools/gamepad_pair.py 10     # 10회

절차:
  1. 조종기 배터리 확인 → HOME 3초로 켜기 → 동글 옆 10cm 에 둔다
  2. 이 도구 실행 → 각 사이클마다 스틱을 계속 살살 움직인다 (이벤트 감지용)
  3. ✅ 뜨면 성공. 6회 다 실패하면 사람 없는 구석으로 차를 옮겨 재시도

⚠️ 같은 USB 버스에 WiFi NIC 이 있음 — idProduct=028e 확인된 포트만 리셋한다.
"""

import glob
import os
import select
import struct
import sys
import time

VENDOR, PRODUCT = '045e', '028e'
JS_DEV = '/dev/input/js0'


def find_dongle():
    for vf in glob.glob('/sys/bus/usb/devices/*/idVendor'):
        dev = os.path.dirname(vf)
        try:
            with open(vf) as f:
                v = f.read().strip()
            with open(os.path.join(dev, 'idProduct')) as f:
                p = f.read().strip()
        except OSError:
            continue
        if (v, p) == (VENDOR, PRODUCT):
            return dev
    return None


def reset_dongle(dev):
    auth = os.path.join(dev, 'authorized')
    with open(auth, 'w') as f:
        f.write('0')
    time.sleep(1.5)
    with open(auth, 'w') as f:
        f.write('1')


def wait_js(timeout=6.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(JS_DEV):
            time.sleep(0.5)   # 드라이버 초기화 여유
            return True
        time.sleep(0.2)
    return False


def watch_events(sec):
    """init(0x80) 아닌 실제 입력 이벤트 수를 반환."""
    try:
        fd = os.open(JS_DEV, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return 0
    got = 0
    t0 = time.time()
    try:
        while time.time() - t0 < sec:
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            try:
                data = os.read(fd, 8 * 64)
            except OSError:
                break
            for i in range(0, len(data) - 7, 8):
                _, value, typev, number = struct.unpack('IhBB', data[i:i + 8])
                if typev & 0x80:
                    continue
                if (typev & 0x01) or (typev & 0x02 and abs(value) > 6000):
                    got += 1
            if got >= 3:
                break
    finally:
        os.close(fd)
    return got


def main():
    if os.geteuid() != 0:
        print('USB 리셋에 root 필요 — sudo 로 재실행 (비번 입력)')
        os.execvp('sudo', ['sudo', sys.executable] + sys.argv)

    tries = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    dev = find_dongle()
    if not dev:
        print('❌ 동글(045e:028e) USB 미인식 — 동글부터 꽂아')
        sys.exit(1)
    print(f'동글 위치: {dev}')
    print('👉 준비: 조종기 켜고(HOME 3초, LED 깜빡임) 동글 옆 10cm 에 둘 것.')
    print('   각 사이클 동안 스틱을 계속 살살 움직여줘 (연결 감지용).\n')
    time.sleep(2)

    for n in range(1, tries + 1):
        print(f'[{n}/{tries}] 동글 리셋 → 바인딩 탐색 재개...')
        try:
            reset_dongle(dev)
        except OSError as e:
            print(f'❌ 리셋 실패: {e}')
            sys.exit(1)
        if not wait_js():
            print('   js0 재등장 안 함 — 동글 물리 재삽입 필요할 수 있음')
            continue
        got = watch_events(8.0)
        if got:
            print(f'\n✅ 연결 성공! 입력 이벤트 {got}건 수신 — mpad --live 로 최종 확인')
            sys.exit(0)
        print('   아직 안 붙음 (8초 무입력)')

    print('\n❌ 전부 실패. 다음 순서로:')
    print('  1. 조종기 배터리 교체 (저전압이 최다 원인)')
    print('  2. 차 들고 사람/다른 조종기 없는 구석으로 이동 후 mpair 재실행')
    print('  3. 동글 물리 재삽입 → 10초 뒤 조종기 켜기 → mpair')
    print('  4. 계속 실패면 오늘은 mwtune(:8083 키보드) 로 진행')
    sys.exit(1)


if __name__ == '__main__':
    main()
