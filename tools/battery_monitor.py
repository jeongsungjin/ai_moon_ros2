#!/usr/bin/env python3
"""배터리 전압/잔량 모니터 (TOPST D3-G — INA219 @ i2c-3, 0x42).

JetRacer 키트의 INA219 센서를 smbus2 로 직접 읽는다 (Jetson 라이브러리 불필요).

사용:
  python3 ~/ai_moon_ros2/tools/battery_monitor.py           # 1초마다 출력
  python3 ~/ai_moon_ros2/tools/battery_monitor.py --once    # 한 번만
"""

import argparse
import time

from smbus2 import SMBus

I2C_BUS = 3
INA219_ADDR = 0x42
REG_BUS_VOLTAGE = 0x02

# 2S 리포 셀당 전압 → 잔량(%) 보간 테이블 (jetracer 배터리 모니터와 동일)
SOC_TABLE = [(4.20, 100), (4.00, 85), (3.85, 60), (3.70, 40), (3.50, 20), (3.30, 10), (3.00, 0)]
CELLS = 2

# 방전 경고 기준 (셀당)
WARN_VPC = 3.50
CRITICAL_VPC = 3.30


def read_pack_voltage(bus):
    raw = bus.read_word_data(INA219_ADDR, REG_BUS_VOLTAGE)
    raw = ((raw & 0xFF) << 8) | (raw >> 8)   # INA219 는 빅엔디안
    return (raw >> 3) * 0.004                # LSB = 4mV


def soc_from_voltage(pack_v):
    vpc = pack_v / CELLS
    if vpc >= SOC_TABLE[0][0]:
        return 100
    if vpc <= SOC_TABLE[-1][0]:
        return 0
    for (v_hi, s_hi), (v_lo, s_lo) in zip(SOC_TABLE, SOC_TABLE[1:]):
        if v_lo <= vpc <= v_hi:
            return s_lo + (s_hi - s_lo) * (vpc - v_lo) / (v_hi - v_lo)
    return 0


def status_label(vpc):
    if vpc <= CRITICAL_VPC:
        return '🔴 위험 — 즉시 충전!'
    if vpc <= WARN_VPC:
        return '🟡 경고 — 곧 충전 필요'
    return '🟢 양호'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--once', action='store_true', help='한 번만 출력하고 종료')
    ap.add_argument('--interval', type=float, default=1.0, help='출력 주기 (초)')
    args = ap.parse_args()

    with SMBus(I2C_BUS) as bus:
        while True:
            v = read_pack_voltage(bus)
            vpc = v / CELLS
            soc = soc_from_voltage(v)
            print(f'배터리 {v:5.2f}V (셀당 {vpc:4.2f}V)  잔량 ~{soc:3.0f}%  {status_label(vpc)}',
                  flush=True)
            if args.once:
                break
            time.sleep(args.interval)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
