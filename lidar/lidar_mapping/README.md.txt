터미널 1 :

python3 ~/ros2_ws/src/ydlidar_bridge.py

터미널 2:

ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args   -p laser_scan_topic:=/scan   -p odom_topic:=/odom   -p publish_tf:=true   -p base_frame_id:=base_link   -p odom_frame_id:=odom   -p use_sim_time:=false

터미널 3:

 ros2 run tf2_ros static_transform_publisher   --x 0 --y 0 --z 0   --roll 0 --pitch 0 --yaw 0   --frame-id base_link   --child-frame-id laser_frame

터미널 4:

source /opt/ros/humble/setup.bash
ros2 run slam_toolbox sync_slam_toolbox_node \
  --ros-args \
  --params-file /home/$USER/slam_params.yaml \
  -p use_sim_time:=false


터미널 5:

ros2 run rviz2 rviz2