#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import rclpy.duration

class AlertNode(Node):
    def __init__(self):
        super().__init__('alert_node')
        self.create_subscription(Bool, '/alert/rpi', self.rpi_cb, 10)
        self.create_subscription(Bool, '/alert/cnn', self.cnn_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/alert/markers', 10)
        self.rpi_active = False
        self.cnn_active = False
        self.blink_state = False
        self.rpi_timer_count = 0
        self.cnn_timer_count = 0
        self.create_timer(0.3, self.update_markers)
        self.get_logger().info('Alert node started')

    def rpi_cb(self, msg):
        if msg.data:
            self.rpi_active = True
            self.rpi_timer_count = 20

    def cnn_cb(self, msg):
        if msg.data:
            self.cnn_active = True
            self.cnn_timer_count = 20

    def make_bg_marker(self, r, g, b, alpha):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'alert_bg'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = -0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 100.0
        m.scale.y = 100.0
        m.scale.z = 0.01
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = alpha
        m.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        return m

    def make_text_markers(self, lines, r, g, b):
        markers = []
        # 맵 오른쪽 위 바깥에 표시 (x=15, y=10 부근)
        for i, line in enumerate(lines):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'alert_text'
            m.id = 10 + i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = 0.0
            m.pose.position.y = 14.0 - float(i) * 1.5
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.z = 3.0
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0
            m.text = line
            m.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
            markers.append(m)
        return markers

    def update_markers(self):
        if self.rpi_timer_count > 0:
            self.rpi_timer_count -= 1
        else:
            self.rpi_active = False

        if self.cnn_timer_count > 0:
            self.cnn_timer_count -= 1
        else:
            self.cnn_active = False

        self.blink_state = not self.blink_state
        markers = MarkerArray()

        if self.rpi_active:
            alpha = 0.7 if self.blink_state else 0.1
            markers.markers.append(self.make_bg_marker(1.0, 0.0, 0.0, alpha))
            for m in self.make_text_markers(
                ['Sound Detected'],
                1.0, 1.0, 1.0  # 흰색
            ):
                markers.markers.append(m)

        elif self.cnn_active:
            alpha = 0.7 if self.blink_state else 0.1
            markers.markers.append(self.make_bg_marker(1.0, 0.3, 0.0, alpha))
            for m in self.make_text_markers(
                ['Person Detected'],
                1.0, 1.0, 1.0  # 흰색
            ):
                markers.markers.append(m)

        else:
            for ns in ['alert_bg', 'alert_text']:
                for mid in range(12):
                    d = Marker()
                    d.header.frame_id = 'map'
                    d.header.stamp = self.get_clock().now().to_msg()
                    d.ns = ns
                    d.id = mid
                    d.action = Marker.DELETE
                    markers.markers.append(d)

        self.marker_pub.publish(markers)

def main():
    rclpy.init()
    node = AlertNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
