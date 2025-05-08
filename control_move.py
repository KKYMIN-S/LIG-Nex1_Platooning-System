#!/usr/bin/env python
# coding: utf-8
import cv2
from YB_Pcb_Car import YB_Pcb_Car

car = YB_Pcb_Car()
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

SPEED = 60

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow('Raspbot Camera', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('w'):            # 전진
            car.Car_Run(SPEED, SPEED)
        elif key == ord('s'):          # 정지
            car.Car_Stop()
        elif key == ord('x'):          # 후진
            car.Car_Back(SPEED, SPEED)
        elif key == ord('a'):          # 좌회전 (스핀)
            car.Car_Spin_Left(SPEED, SPEED)
        elif key == ord('d'):          # 우회전 (스핀)
            car.Car_Spin_Right(SPEED, SPEED)
        elif key == ord('c'):          # 카메라 화면만 종료
            cv2.destroyWindow('Raspbot Camera')
        elif key == ord('q'):          # 전체 종료
            break

finally:
    car.Car_Stop()
    if cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()
