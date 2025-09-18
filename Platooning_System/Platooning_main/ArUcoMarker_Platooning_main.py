import cv2
import time
from YB_Pcb_Car import YB_Pcb_Car
from Aruco_Detection import ArucoDetector
from DistanceSensor import DistanceController

# ───────── 객체 초기화 ─────────
car = YB_Pcb_Car()
distance_sensor = DistanceController()
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH , 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ───────── ArUco 탐지기 초기화 ─────────
aruco_tracker = ArucoDetector(target_id=0, timeout=1.0)

# ───────── 주행 설정 ─────────
BASE_SPEED = 40
LEFT_TRIM = 0
RIGHT_TRIM = -5

# ───────── 상태 변수 ─────────
current_state = "stopped"

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라 프레임 오류")
            break

        # 거리 측정
        distance = distance_sensor.get_kalman_distance()

        # 마커 인식 처리
        frame, is_detected, is_timeout = aruco_tracker.detect(frame)

        # 거리 기준 로직
        if distance == -1:
            print("거리 측정 실패")
            if current_state != "stopped":
                car.Car_Stop()
                current_state = "stopped"

        elif distance < 10:
            if current_state != "stopped":
                print(f"{distance:.1f}cm → 너무 가까움 → 정지")
                car.Car_Stop()
                current_state = "stopped"

        elif 10 <= distance <= 12:
            print(f"{distance:.1f}cm → 적정 거리 유지")

        elif distance > 12:
            if is_detected:
                print(f"{distance:.1f}cm → Marker 인식됨 → 전진")
                if current_state != "moving":
                    car.Car_Run(BASE_SPEED + LEFT_TRIM, BASE_SPEED + RIGHT_TRIM)
                    current_state = "moving"
            elif is_timeout:
                print("Marker 1초 이상 미탐지 → 정지")
                if current_state != "stopped":
                    car.Car_Stop()
                    current_state = "stopped"
            else:
                print("Marker 일시적 미탐지 중...")

        cv2.imshow("Aruco Platooning", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("사용자 종료 (q)")
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    print("종료 (Ctrl+C)")

finally:
    car.Car_Stop()
    cap.release()
    distance_sensor.cleanup()
    cv2.destroyAllWindows()
