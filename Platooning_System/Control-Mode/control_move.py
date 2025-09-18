#!/usr/bin/env python
# coding: utf-8
import cv2
import time
from YB_Pcb_Car import YB_Pcb_Car

car = YB_Pcb_Car()
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

SPEED = 60                  # 모터 속도 (0~255)
DISTANCE_TIME = 1.0         # 키 한 번 누름으로 이동할 시간 (초)
ANGLE_STEP = 5              # 서보 한 번 조작 시 각도 변화량
move_end_time = 0           # 이동 종료 시각 저장

# 서보 초기 각도 (1번: 상하, 2번: 좌우)
angle_vert = 90 + 10
angle_horiz = 90 - 20
car.Ctrl_Servo(1, angle_horiz)
car.Ctrl_Servo(2, angle_vert)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow('Raspbot Camera', frame)
        now = time.time()

        # 이동 시간이 끝나면 자동 정지
        if move_end_time and now >= move_end_time:
            car.Car_Stop()
            move_end_time = 0

        key = cv2.waitKey(1) & 0xFF

        # 주행 제어
        if key == ord('w'):
            car.Car_Run(SPEED, SPEED)
            move_end_time = now + DISTANCE_TIME
        elif key == ord('x'):
            car.Car_Back(SPEED, SPEED)
            move_end_time = now + DISTANCE_TIME
        elif key == ord('a'):
            car.Car_Left(SPEED, SPEED)
            move_end_time = now + DISTANCE_TIME
        elif key == ord('d'):
            car.Car_Right(SPEED, SPEED)
            move_end_time = now + DISTANCE_TIME
        elif key == ord('z'):
            car.Car_Spin_Left(SPEED, SPEED)
            move_end_time = now + DISTANCE_TIME
        elif key == ord('c'):
            car.Car_Spin_Right(SPEED, SPEED)
            move_end_time = now + DISTANCE_TIME
        elif key == ord('s'):
            car.Car_Stop(); move_end_time = 0

        # 서보 제어 (방향키 )
        elif key == 82:  # ↑ 
            angle_vert = max(0, angle_vert - ANGLE_STEP)
            car.Ctrl_Servo(2, angle_vert)
        elif key == 84:  # ↓ 
            angle_vert = min(180, angle_vert + ANGLE_STEP)
            car.Ctrl_Servo(2, angle_vert)
        elif key == 81:  # ← 
            angle_horiz = min(180, angle_horiz + ANGLE_STEP)
            car.Ctrl_Servo(1, angle_horiz)
        elif key == 83:  # → 
            angle_horiz = max(0, angle_horiz - ANGLE_STEP)
            car.Ctrl_Servo(1, angle_horiz)

        elif key == ord('q'):
            break

finally:
    car.Car_Stop()
    cap.release()
    cv2.destroyAllWindows()
