from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # fpga_lidar_parser launch 파일 경로
    fpga_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('fpga_lidar_parser'),
            '/launch/lidar.launch.py'
        ])
    )
    
    return LaunchDescription([
        # FPGA LiDAR 파서
        fpga_launch,
        
        # TF: base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),
        
        # TF: odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
    ])