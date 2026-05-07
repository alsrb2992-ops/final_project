#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy.duration
import math

class AlertNode(Node):
    def __init__(self):
        super().__init__('alert_node')
        self.create_subscription(Bool, '/alert/rpi', self.rpi_cb, 10)
        self.create_subscription(Bool, '/alert/cnn', self.cnn_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/alert/markers', 10)

        self.rpi_active = False
        self.cnn_active = False
        self.blink_state = False
        self.rpi_timer_count = 0
        self.cnn_timer_count = 0

        # 로봇 현재 위치/방향
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.create_timer(0.3, self.update_markers)
        self.get_logger().info('Alert node started')

    def pose_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # quaternion → yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

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
        m.pose.position.x = 18.0
        m.pose.position.y = 0.0
        m.pose.position.z = -0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 14.0
        m.scale.y = 8.0
        m.scale.z = 0.01
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = alpha
        m.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        return m

    def make_text_marker(self, lines, r, g, b):
        markers = []
        for i, text in enumerate(lines):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'alert_text'
            m.id = 10 + i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = 18.0
            m.pose.position.y = 1.0 - float(i) * 2.5
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.z = 3.0
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0
            m.text = text
            m.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
            markers.append(m)
        return markers

    def make_cone_markers(self, alpha):
        # 전방 원뿔: 로봇 위치에서 yaw 방향으로 삼각형 점들 생성
        markers = []
        cone_length = 3.0   # 원뿔 길이 (m)
        cone_angle  = 45.0  # 원뿔 각도 (도)
        num_points  = 20    # 삼각형 분할 수

        for i in range(num_points):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'alert_cone'
            m.id = 100 + i
            m.type = Marker.TRIANGLE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = alpha
            m.lifetime = rclpy.duration.Duration(seconds=1).to_msg()

            # 원뿔 삼각형 분할
            angle_step = (cone_angle * 2) / num_points
            a1 = math.radians(-cone_angle + i * angle_step) + self.robot_yaw
            a2 = math.radians(-cone_angle + (i+1) * angle_step) + self.robot_yaw

            from geometry_msgs.msg import Point
            # 꼭짓점 (로봇 위치)
            p0 = Point()
            p0.x = self.robot_x + 0.2 * math.cos(self.robot_yaw)
            p0.y = self.robot_y + 0.2 * math.sin(self.robot_yaw)
            p0.z = 0.0

            # 원뿔 왼쪽 점
            p1 = Point()
            p1.x = self.robot_x + cone_length * math.cos(a1)
            p1.y = self.robot_y + cone_length * math.sin(a1)
            p1.z = 0.0

            # 원뿔 오른쪽 점
            p2 = Point()
            p2.x = self.robot_x + cone_length * math.cos(a2)
            p2.y = self.robot_y + cone_length * math.sin(a2)
            p2.z = 0.0

            m.points = [p0, p1, p2]
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
            markers.markers.append(self.make_bg_marker(1.0, 1.0, 0.3, alpha))
            for m in self.make_text_marker(['Sound', 'Detected'], 1.0, 1.0, 1.0):
                markers.markers.append(m)

        elif self.cnn_active:
            alpha = 0.7 if self.blink_state else 0.1
            markers.markers.append(self.make_bg_marker(1.0, 0.0, 0.0, alpha))
            for m in self.make_text_marker(['Person', 'Detected'], 1.0, 1.0, 1.0):
                markers.markers.append(m)
            # 전방 원뿔 추가
            for m in self.make_cone_markers(0.6 if self.blink_state else 0.2):
                markers.markers.append(m)

        else:
            for ns in ['alert_bg', 'alert_text', 'alert_cone']:
                for mid in range(120):
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
