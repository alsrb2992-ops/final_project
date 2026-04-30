#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
import serial, struct, math

class YDLIDARBridge(Node):
    def __init__(self):
        super().__init__('ydlidar_bridge')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(LaserScan, '/scan', qos)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 460800, timeout=0.1)
            self.get_logger().info('Serial connected: /dev/ttyACM0 @ 460800')
        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
            return

        self.buf = bytearray()
        self.current_ranges = [0.0] * 360
        self.scan_count = 0

        self.create_timer(0.01, self.receive_loop)

    def receive_loop(self):
        if not self.ser.in_waiting:
            return
        chunk = self.ser.read(4096)
        self.buf.extend(chunk)
        self.parse_buf()

    def parse_buf(self):
        buf = self.buf
        while len(buf) >= 10:
            if buf[0] != 0xAA or buf[1] != 0x55:
                buf.pop(0)
                continue
            lsn = buf[3]
            pkt_len = 10 + 2 * lsn
            if len(buf) < pkt_len:
                break
            pkt = bytes(buf[:pkt_len])
            del buf[:pkt_len]
            self.process_packet(pkt)

    def process_packet(self, pkt):
        ct      = pkt[2]
        lsn     = pkt[3]
        fsa_raw = struct.unpack_from('<H', pkt, 4)[0]
        lsa_raw = struct.unpack_from('<H', pkt, 6)[0]
        is_start = bool(ct & 0x01)

        angle_fsa = (fsa_raw >> 1) / 64.0
        angle_lsa = (lsa_raw >> 1) / 64.0
        diff = (angle_lsa - angle_fsa + 360.0) % 360.0

        if is_start and self.scan_count > 0:
            self.publish_scan()
            self.current_ranges = [0.0] * 360

        for idx in range(lsn):
            si_raw  = struct.unpack_from('<H', pkt, 10 + 2 * idx)[0]
            dist_mm = si_raw >> 2
            interf  = si_raw & 0x03
            if dist_mm > 0 and interf == 0:
                angle = angle_fsa + diff * idx / (lsn - 1) if lsn > 1 else angle_fsa
                corr = math.degrees(math.atan(
                    21.8 * (155.3 - dist_mm) / (155.3 * dist_mm)
                ))
                angle = (angle + corr) % 360.0
                slot = int(angle) % 360
                self.current_ranges[slot] = dist_mm / 1000.0

        self.scan_count += 1

    def publish_scan(self):
        valid = sum(1 for r in self.current_ranges if r > 0.12)
        if valid < 10:
            return

        msg = LaserScan()
        # scan을 0.15초 과거로 발행 → rf2o TF가 이미 존재하는 시간
        stamp = self.get_clock().now()
        msg.header.stamp    = stamp.to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min       = 0.0
        msg.angle_max       = math.radians(359.0)
        msg.angle_increment = math.radians(1.0)
        msg.scan_time       = 0.1
        msg.time_increment  = 0.0
        msg.range_min       = 0.12
        msg.range_max       = 10.0
        msg.ranges          = [float(r) for r in self.current_ranges]
        self.pub.publish(msg)
        self.get_logger().info(f'Scan published: {valid} valid points', once=True)

def main():
    rclpy.init()
    node = YDLIDARBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
