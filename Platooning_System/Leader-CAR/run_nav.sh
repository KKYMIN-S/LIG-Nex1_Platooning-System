#!/bin/bash
source /opt/ros/eloquent/setup.bash
source ~/ros2_ws/install/setup.bash
#mqtt nofity start sign
python3 ~/ros2_ws/mqtt/send_start_sign.py &

python3 ~/ros2_ws/mqtt/send_waypoint.py &

# 2. RPLIDAR 실행
ros2 launch rplidar_ros rplidar_a1_launch.py use_sim_time:=false &
RPLIDAR_PID=$!

# 3. ROS 2 환경 설정 (순서 중요)
source /opt/ros/eloquent/setup.bash
source ~/ros2_ws/install/setup.bash

# 4. 종료 시 라이다, TF 퍼블리셔 같이 죽이기
trap "kill $RPLIDAR_PID" SIGINT

# 5. 메인 네비게이션 launch 실행
ros2 launch teleop_drive run_nav_launch.py &
NAV_PID=$!

# 6. lifecycle 자동 전환
sleep 4
ros2 lifecycle set /map_server configure
sleep 1
ros2 lifecycle set /map_server activate

# 7. 종료 처리
trap "kill $RPLIDAR_PID $NAV_PID" SIGINT
wait

