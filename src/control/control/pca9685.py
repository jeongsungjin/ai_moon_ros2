"""PCA9685 I2C PWM 드라이버 (D-Racer-Kit topst_utils 에서 포팅, Apache-2.0).

board/blinka 없이 smbus 로 직접 제어한다.
"""

import time

try:
    from smbus2 import SMBus
except ImportError:  # 환경에 따라 smbus 만 있을 수 있음
    from smbus import SMBus  # type: ignore


# PCA9685 레지스터
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

# MODE1 비트
RESTART = 0x80
SLEEP = 0x10
ALLCALL = 0x01

# MODE2 비트
OUTDRV = 0x04


class PCA9685:
    def __init__(self, bus=1, address=0x40, freq_hz=50.0, osc_hz=25_000_000.0):
        self.busnum = bus
        self.addr = address
        self.osc_hz = osc_hz
        self.freq_hz = freq_hz
        self.i2c = SMBus(bus)

        self.write8(MODE1, ALLCALL)
        self.write8(MODE2, OUTDRV)
        time.sleep(0.01)

        self.set_pwm_freq(freq_hz)

    def write8(self, reg, val):
        self.i2c.write_byte_data(self.addr, reg, val & 0xFF)

    def read8(self, reg):
        return self.i2c.read_byte_data(self.addr, reg)

    def set_pwm_freq(self, freq_hz):
        freq_hz = float(freq_hz)
        prescale = int(round(self.osc_hz / (4096.0 * freq_hz) - 1.0))

        oldmode = self.read8(MODE1)
        self.write8(MODE1, (oldmode & 0x7F) | SLEEP)
        self.write8(PRESCALE, prescale)
        self.write8(MODE1, oldmode)
        time.sleep(0.005)
        self.write8(MODE1, oldmode | RESTART)

        self.freq_hz = freq_hz

    def us_to_ticks(self, us):
        period_us = 1_000_000.0 / self.freq_hz
        ticks = int(round((us / period_us) * 4096.0))
        return max(0, min(4095, ticks))

    def set_pwm(self, channel, on, off):
        base = LED0_ON_L + 4 * channel
        self.write8(base + 0, on & 0xFF)
        self.write8(base + 1, (on >> 8) & 0xFF)
        self.write8(base + 2, off & 0xFF)
        self.write8(base + 3, (off >> 8) & 0xFF)

    def set_pulse_us(self, channel, pulse_us):
        self.set_pwm(channel, 0, self.us_to_ticks(pulse_us))

    def close(self):
        try:
            self.i2c.close()
        except Exception:
            pass
