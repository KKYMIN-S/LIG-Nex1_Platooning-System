#!/usr/bin/env python3
# coding: utf-8

import cv2
import time
from YB_Pcb_Car import YB_Pcb_Car
from KCF_Tracker import KCFTracker
from DistanceSensor import DistanceController

# ───────── 시스템 초기화 ─────────
car = YB_Pcb_Car()
tracker = KCFTracker()
distance_sensor = DistanceController()

# ───────── 주행 파라미터 ─────────
BASE_SPEED = 40           # 기본 전진 속도(PWM)
LEFT_TRIM = -7             # 좌측 휠 트림 보정값
RIGHT_TRIM = 0             # 우측 휠 트림 보정값

# ───────── 서보(Pan/Tilt) 설정 ─────────
CENTER_YAW = 70            # 서보 1 초기 수평 위치
CENTER_PITCH = 110         # 서보 2 초기 수직 위치
servo_yaw = CENTER_YAW     # 현재 수평 서보 각도
SERVO_STEP = 2             # 서보 이동 단계 (도)
DEAD_ZONE = 15             # 중앙 오차 허용 범위 (픽셀)

# 서보를 초기 위치로 이동
car.Ctrl_Servo(1, servo_yaw)
car.Ctrl_Servo(2, CENTER_PITCH)

# ───────── 상태 관리 ─────────
current_state = "moving"  # 차량 상태: "moving" 또는 "stopped"
FAIL_TIMEOUT = 1.0          # 트래커 실패 후 허용 대기 시간 (초)

try:
    while True:
        # 트래커 업데이트 및 화면 획득
        bbox, success, frame = tracker.update()

        # 화면 중앙 계산
        frame_center = frame.shape[1] // 2

        if success:
            # 추적 성공: 바운딩 박스 표시
            x, y, w, h, cx, _ = map(int, bbox)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 카메라(Pan) 자동 조정
            err = cx - frame_center
            if abs(err) > DEAD_ZONE:
                if err > 0:
                    servo_yaw = max(0, servo_yaw - SERVO_STEP)
                else:
                    servo_yaw = min(180, servo_yaw + SERVO_STEP)
                car.Ctrl_Servo(1, servo_yaw)

            # 거리 센서로 간격 유지
            dist = distance_sensor.get_distance()

            if dist == -1:
                # 거리 측정 실패 시 정지
                car.Car_Stop()
                current_state = "stopped"
            else:
                # 정상 거리 측정
                if current_state == "moving":
                    if dist < 10:
                        # 너무 가까우면 정지
                        car.Car_Stop()
                        current_state = "stopped"
                    else:
                        # 충분한 거리, 직진 유지
                        car.Car_Run(BASE_SPEED + LEFT_TRIM,
                                    BASE_SPEED + RIGHT_TRIM)
                else:
                    if dist > 10:
                        # 안전 거리 확보 시 재이동
                        car.Car_Run(BASE_SPEED + LEFT_TRIM,
                                    BASE_SPEED + RIGHT_TRIM)
                        current_state = "moving"
                    else:
                        # 아직 가까우면 정지
                        car.Car_Stop()
        else:
            # 추적 실패 처리
            elapsed = time.time() - tracker.get_last_success_time()
            if elapsed > FAIL_TIMEOUT:
                # 1초 이상 실패 시 정지 상태 유지
                car.Car_Stop()
                current_state = "stopped"
            # 그 외의 경우에는 일시적 실패로 간주하여 명령 유지

        # 결과 화면 출력 및 종료 키 대기
        cv2.imshow("Object Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # 루프 속도 제어
        time.sleep(0.05)

except KeyboardInterrupt:
    # Ctrl+C 입력 시 안전하게 종료
    pass

finally:
    # 자원 해제 및 차량 정지
    car.Car_Stop()
    distance_sensor.cleanup()
    tracker.release()
    cv2.destroyAllWindows()
