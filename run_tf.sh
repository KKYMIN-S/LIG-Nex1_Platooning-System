#!/bin/bash
mosquitto -v &
BROKER_PID=$!

# static TF publishers 실행 (각각 백그라운드에서)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
TF1_PID=$!

ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_footprint base_link &
TF2_PID=$!

ros2 run tf2_ros static_transform_publisher 0.1 0.0 0.1 0 0 0 base_link laser &
TF3_PID=$!

#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 3.14159 laser scan &
#TF4_PID=$!

# Ctrl+C 시 종료 처리
trap "kill $BROKER_PID $TF1_PID $TF2_PID $TF3_PID" SIGINT

echo "[TF] Static TF 퍼블리셔 3개 & mosquitto 실행 중... 종료하려면 Ctrl+C"

# 무한 대기
wait
