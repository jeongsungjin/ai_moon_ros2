#!/usr/bin/env python3
"""모터 폭주 방지 워치독 (control_node 와 독립 실행).

control_node 가 어떤 이유로든(크래시, SIGKILL, 방전 직전 오동작) 죽으면
PCA9685 에 마지막 스로틀 PWM 이 남아 차가 폭주한다. 이 워치독은 ROS 와
무관하게 프로세스 존재만 감시하다가, control_node 가 사라지는 순간
ESC 채널에 중립(1500us)을 직접 써서 모터를 세운다.

사용 (별도 터미널에 항상 켜두기):
  python3 ~/ai_moon_ros2/tools/motor_watchdog.py

동작:
  - 0.2초마다 control_node 프로세스 확인
  - 살아있다가 사라짐 → 즉시 ESC 중립 연속 기록 (2초간) + 경고 출력
  - 시작 시점에 control_node 가 없으면 예방적으로 중립 1회 기록
  - PCA9685 가 sleep(출력 정지) 상태면 건드리지 않음 (펄스 없음 = 안전)
"""

import os
import time

from smbus2 import SMBus

I2C_BUS = 3
PCA_ADDR = 0x40
THROTTLE_CH = 1          # params.yaml control_node: throttle_channel
NEUTRAL_US = 1500        # ESC 중립 펄스

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

CONTROL_PATTERN = 'lib/control/control_node'

_cached_pid = None


def _find_pid():
    """CONTROL_PATTERN 이 cmdline 에 포함된 프로세스 검색 (/proc 직접 스캔)."""
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                if CONTROL_PATTERN.encode() in f.read():
                    return int(pid)
        except OSError:
            continue
    return None


def control_alive():
    """PID 캐시로 평상시 확인 비용을 0 에 가깝게 유지."""
    global _cached_pid
    if _cached_pid is not None:
        if os.path.exists(f'/proc/{_cached_pid}'):
            return True
        _cached_pid = None          # 죽음 — 재시작 대비 재탐색은 아래에서
    _cached_pid = _find_pid()
    return _cached_pid is not None


def write_neutral(bus):
    """ESC 채널에 중립 펄스 기록. sleep 상태면 출력이 없으므로 패스."""
    mode1 = bus.read_byte_data(PCA_ADDR, MODE1)
    if mode1 & 0x10:                       # SLEEP 비트: 펄스 자체가 안 나감
        return 'sleep(안전)'
    prescale = bus.read_byte_data(PCA_ADDR, PRESCALE)
    freq = 25_000_000 / (4096 * (prescale + 1))
    ticks = int(round(NEUTRAL_US * freq * 4096 / 1_000_000))
    reg = LED0_ON_L + 4 * THROTTLE_CH
    bus.write_i2c_block_data(PCA_ADDR, reg,
                             [0, 0, ticks & 0xFF, (ticks >> 8) & 0x0F])
    return f'중립 {NEUTRAL_US}us 기록 (freq {freq:.0f}Hz)'


def main():
    print('🐕 모터 워치독 시작 — control_node 급사 시 즉시 모터 중립', flush=True)
    with SMBus(I2C_BUS) as bus:
        was_alive = control_alive()
        if not was_alive:
            print(f'control_node 없음 → 예방적 {write_neutral(bus)}', flush=True)
        while True:
            alive = control_alive()
            if was_alive and not alive:
                print('🚨 control_node 소멸 감지! 모터 중립 기록 중...', flush=True)
                end = time.time() + 2.0
                while time.time() < end:      # ESC 가 확실히 받도록 2초간 반복
                    result = write_neutral(bus)
                    time.sleep(0.05)
                print(f'   → {result}. 감시 계속.', flush=True)
            was_alive = alive
            time.sleep(0.2)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('워치독 종료')
