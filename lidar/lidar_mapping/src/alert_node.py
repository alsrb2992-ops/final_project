#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
import rclpy.duration
import subprocess
import threading
import math
import numpy as np
import os

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
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.create_timer(0.3, self.update_markers)
        self.create_timer(1.0, self.publish_title)
        self.get_logger().info('Alert node started')

    def pose_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def rpi_cb(self, msg):
        if msg.data:
            self.rpi_active = True
            self.rpi_timer_count = 20
            self.play_sound()

    def cnn_cb(self, msg):
        if msg.data:
            self.cnn_active = True
            self.cnn_timer_count = 20
            self.play_sound()

    def play_sound(self):
        if hasattr(self, "_sound_playing") and self._sound_playing:
            return
        self._sound_playing = True
        def _play():
            import sounddevice as sd
            os.environ["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
            rate = 44100
            duration = 0.05
            frames = []
            while self.rpi_active or self.cnn_active:
                frames = []
                for freq in list(range(600, 1200, 50)) + list(range(1200, 600, -50)):
                    t = np.linspace(0, duration, int(rate * duration))
                    wave = np.sin(2 * np.pi * freq * t).astype(np.float32)
                    frames.append(wave)
                siren = np.concatenate(frames)
                sd.play(siren, rate)
                sd.wait()
            self._sound_playing = False
        threading.Thread(target=_play, daemon=True).start()

    def publish_title(self):
        markers = MarkerArray()
        lines = ['Multi-Sensor & AI-Based', 'Industrial Accident Victim Search System']
        for i, text in enumerate(lines):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'title'
            m.id = 200 + i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = 0.0
            m.pose.position.y = 13.0 - float(i) * 1.8
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.z = 1.2
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.text = text
            m.lifetime = rclpy.duration.Duration(seconds=2).to_msg()
            markers.markers.append(m)
        self.marker_pub.publish(markers)

    def make_bg_marker(self, mid, r, g, b, alpha, x, y):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'alert_bg'
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = -0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 10.0
        m.scale.y = 3.0
        m.scale.z = 0.01
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = alpha
        m.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        return m

    def make_text_marker(self, mid, text, r, g, b, x, y):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'alert_text'
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.z = 1.5
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        m.text = text
        m.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        return m

    def make_cone_markers(self, alpha):
        markers = []
        cone_length = 3.0
        cone_angle = 45.0
        num_points = 20
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
            angle_step = (cone_angle * 2) / num_points
            a1 = math.radians(-cone_angle + i * angle_step) + self.robot_yaw
            a2 = math.radians(-cone_angle + (i+1) * angle_step) + self.robot_yaw
            p0 = Point()
            p0.x = self.robot_x + 0.2 * math.cos(self.robot_yaw)
            p0.y = self.robot_y + 0.2 * math.sin(self.robot_yaw)
            p0.z = 0.0
            p1 = Point()
            p1.x = self.robot_x + cone_length * math.cos(a1)
            p1.y = self.robot_y + cone_length * math.sin(a1)
            p1.z = 0.0
            p2 = Point()
            p2.x = self.robot_x + cone_length * math.cos(a2)
            p2.y = self.robot_y + cone_length * math.sin(a2)
            p2.z = 0.0
            m.points = [p0, p1, p2]
            markers.append(m)
        return markers

    def make_person_markers(self, x, y):
        markers = []
        parts = [
            (300, Marker.CYLINDER, 0.0,  0.85, 0.5, 0.5, 0.05),
            (301, Marker.CUBE,     0.0,  0.3,  0.5, 0.8, 0.1),
            (302, Marker.CUBE,    -0.15,-0.4,  0.2, 0.6, 0.1),
            (303, Marker.CUBE,     0.15,-0.4,  0.2, 0.6, 0.1),
            (304, Marker.CUBE,    -0.35, 0.3,  0.2, 0.6, 0.1),
            (305, Marker.CUBE,     0.35, 0.3,  0.2, 0.6, 0.1),
        ]
        for pid, ptype, dx, dy, sx, sy, sz in parts:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'person_icon'
            m.id = pid
            m.type = ptype
            m.action = Marker.ADD
            m.pose.position.x = x + dx
            m.pose.position.y = y + dy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = sz
            m.color.r = 1.0
            m.color.g = 0.8
            m.color.b = 0.6
            m.color.a = 1.0
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

        # RPi: 오른쪽 위 (x=18, y=4)
        if self.rpi_active:
            alpha = 0.7 if self.blink_state else 0.1
            markers.markers.append(self.make_bg_marker(0, 1.0, 1.0, 0.3, alpha, 18.0, 4.0))
            markers.markers.append(self.make_text_marker(10, 'Sound Detected', 1.0, 1.0, 1.0, 18.0, 4.0))

        # CNN: 오른쪽 아래 (x=18, y=0), 사람 아이콘은 더 오른쪽 (x=24)
        if self.cnn_active:
            alpha = 0.7 if self.blink_state else 0.1
            markers.markers.append(self.make_bg_marker(1, 1.0, 0.0, 0.0, alpha, 18.0, 0.0))
            markers.markers.append(self.make_text_marker(11, 'Person Detected', 1.0, 1.0, 1.0, 18.0, 0.0))
            for m in self.make_cone_markers(0.6 if self.blink_state else 0.2):
                markers.markers.append(m)
            for m in self.make_person_markers(24.0, 0.0):
                markers.markers.append(m)

        if not self.rpi_active and not self.cnn_active:
            for ns in ['alert_bg', 'alert_text', 'alert_cone', 'person_icon']:
                for mid in range(310):
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
