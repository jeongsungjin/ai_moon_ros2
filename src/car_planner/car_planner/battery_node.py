#!/usr/bin/env python3
"""배터리 모니터 노드 (INA219 @ i2c-3, 0x42 — D-Racer-Kit 의존 제거 자체 구현).

표준 메시지(Float32)만 사용하므로 어떤 터미널에서든 타입 문제 없이 echo 가능.

발행:
  /battery/voltage (Float32)  팩 전압 [V]
  /battery/percent (Float32)  잔량 추정 [%] (2S 리포 셀전압 보간)

잔량이 경고/위험 기준 아래로 내려가면 로그로 경고한다 (방전 사고 예방).

사용:
  ros2 run car_planner battery_node
  ros2 topic echo /battery/percent
"""

import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import Float32

REG_BUS_VOLTAGE = 0x02

# 2S 리포 셀당 전압 → 잔량(%) 보간 테이블
SOC_TABLE = [(4.20, 100), (4.00, 85), (3.85, 60), (3.70, 40), (3.50, 20), (3.30, 10), (3.00, 0)]


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('ina_addr', 0x42)
        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('cells', 2)
        self.declare_parameter('warn_vpc', 3.5)      # 셀당 경고 전압
        self.declare_parameter('critical_vpc', 3.3)  # 셀당 위험 전압

        self.addr = int(self.get_parameter('ina_addr').value)
        self.cells = int(self.get_parameter('cells').value)
        self.warn_vpc = float(self.get_parameter('warn_vpc').value)
        self.critical_vpc = float(self.get_parameter('critical_vpc').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        self.bus = SMBus(int(self.get_parameter('i2c_bus').value))
        self.voltage_pub = self.create_publisher(Float32, '/battery/voltage', 10)
        self.percent_pub = self.create_publisher(Float32, '/battery/percent', 10)
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_status)

        self._warned = False
        self.get_logger().info(
            f'battery_node started: i2c-{self.get_parameter("i2c_bus").value} '
            f'0x{self.addr:02X}, {publish_hz}Hz, 경고 {self.warn_vpc}V/셀'
        )

    def read_pack_voltage(self):
        raw = self.bus.read_word_data(self.addr, REG_BUS_VOLTAGE)
        raw = ((raw & 0xFF) << 8) | (raw >> 8)   # INA219 는 빅엔디안
        return (raw >> 3) * 0.004                # LSB = 4mV

    def soc_from_voltage(self, pack_v):
        vpc = pack_v / self.cells
        if vpc >= SOC_TABLE[0][0]:
            return 100.0
        if vpc <= SOC_TABLE[-1][0]:
            return 0.0
        for (v_hi, s_hi), (v_lo, s_lo) in zip(SOC_TABLE, SOC_TABLE[1:]):
            if v_lo <= vpc <= v_hi:
                return s_lo + (s_hi - s_lo) * (vpc - v_lo) / (v_hi - v_lo)
        return 0.0

    def publish_status(self):
        try:
            v = self.read_pack_voltage()
        except OSError as e:
            self.get_logger().error(f'INA219 읽기 실패: {e}')
            return
        vpc = v / self.cells
        pct = self.soc_from_voltage(v)
        self.voltage_pub.publish(Float32(data=float(v)))
        self.percent_pub.publish(Float32(data=float(pct)))

        if vpc <= self.critical_vpc:
            self.get_logger().error(
                f'🔴 배터리 위험! {v:.2f}V (셀당 {vpc:.2f}V, ~{pct:.0f}%) — 즉시 충전!')
        elif vpc <= self.warn_vpc:
            if not self._warned:
                self.get_logger().warning(
                    f'🟡 배터리 경고: {v:.2f}V (셀당 {vpc:.2f}V, ~{pct:.0f}%) — 곧 충전 필요')
                self._warned = True
        else:
            self._warned = False

    def destroy_node(self):
        try:
            self.bus.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
