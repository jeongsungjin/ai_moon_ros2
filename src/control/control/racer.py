"""조향 서보 + ESC 하드웨어 추상화 (D-Racer-Kit D3Racer 포팅).

JetRacer / D3-G 공통으로 PCA9685 한 장에
  - steering_channel: 조향 서보
  - throttle_channel: ESC (또는 모터드라이버 PWM 입력)
를 연결한 구성을 가정한다. 채널/버스/펄스 범위는 파라미터로 조정.

set_steering_percent / set_throttle_percent: -1.0 ~ +1.0
"""

from dataclasses import dataclass

from control.pca9685 import PCA9685


@dataclass
class ServoCalib:
    center_us: int = 1500
    span_us: int = 500      # 1500±500 => 1000~2000us
    min_us: int = 1000
    max_us: int = 2000


@dataclass
class EscCalib:
    neutral_us: int = 1500
    fwd_us: int = 2000      # +1.0
    rev_us: int = 1000      # -1.0
    min_us: int = 1000
    max_us: int = 2000


def clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Racer:
    def __init__(
        self,
        i2c_bus=1,
        pca9685_addr=0x40,
        freq_hz=50.0,
        steering_channel=0,
        throttle_channel=1,
        steering=None,
        esc=None,
    ):
        self.pwm = PCA9685(bus=i2c_bus, address=pca9685_addr, freq_hz=freq_hz)
        self.st_ch = steering_channel
        self.th_ch = throttle_channel
        self.st = steering or ServoCalib()
        self.esc = esc or EscCalib()

        # 안전: 초기 중립
        self.set_steering_percent(0.0)
        self.set_throttle_percent(0.0)

    def set_steering_percent(self, p):
        p = clip(float(p), -1.0, 1.0)
        pulse = self.st.center_us + p * self.st.span_us
        pulse = clip(pulse, self.st.min_us, self.st.max_us)
        self.pwm.set_pulse_us(self.st_ch, pulse)

    def set_throttle_percent(self, p):
        p = clip(float(p), -1.0, 1.0)
        if p > 0:
            pulse = self.esc.neutral_us + p * (self.esc.fwd_us - self.esc.neutral_us)
        elif p < 0:
            pulse = self.esc.neutral_us + p * (self.esc.neutral_us - self.esc.rev_us)
        else:
            pulse = self.esc.neutral_us
        pulse = clip(pulse, self.esc.min_us, self.esc.max_us)
        self.pwm.set_pulse_us(self.th_ch, pulse)

    def stop(self):
        self.set_throttle_percent(0.0)

    def close(self):
        self.stop()
        self.pwm.close()
