#!/bin/bash
source ~/ros2_ws/install/setup.bash
sudo chmod 666 /dev/ttyUSB0

ros2 launch rplidar_ros rplidar_a1_launch.py &
RPLIDAR_PID=$!

sleep 3

ros2 launch ~/ros2_ws/launch/slam_launch.py
kill $RPLIDAR_PID


