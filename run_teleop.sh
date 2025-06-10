#!/bin/bash

echo "[] USB 권한 설정 중..."
sudo chmod 666 /dev/ttyUSB0

echo "[] ROS2 환경 세팅 중..."
source ~/ros2_ws/install/setup.bash

echo "[] odom_publisher 실행!"
#ros2 run teleop_drive odom_and_scan_publisher &  ###for navigate
ros2 run teleop_drive odom_publisher &   ###for slam

echo "[] teleop_listener 실행!"
ros2 run teleop_drive teleop_listener &

echo "[⌨️] 키보드 조작 창을 새 터미널로 엽니다..."
gnome-terminal -- bash -c "source ~/ros2_ws/install/setup.bash; ros2 run teleop_twist_keyboard teleop_twist_keyboard"

