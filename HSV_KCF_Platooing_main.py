from YB_Pcb_Car import YB_Pcb_Car
from KCF_Tracker import KCFTracker
from DistanceSensor import DistanceController
import cv2
import time

# ───────── 객체 초기화 ─────────
car = YB_Pcb_Car()
tracker = KCFTracker()
distance_sensor = DistanceController()

# ───────── 주행 설정 ─────────
BASE_SPEED = 40
LEFT_TRIM = 0
RIGHT_TRIM = -5

# ───────── 상태 변수 ─────────
current_state = "stopped"
FAIL_TIMEOUT = 1.0

try:
    while True:
        bbox, success, frame = tracker.update()
        distance = distance_sensor.get_kalman_distance()

        if distance == -1:
            print("거리 측정 실패")
            if current_state != "stopped":
                car.Car_Stop()
                current_state = "stopped"

        elif distance < 10:
            if current_state != "stopped":
                print(f"{distance:.1f}cm → 너무 가까움! 정지")
                car.Car_Stop()
                current_state = "stopped"

        elif distance >= 10 and distance <= 12:
            print(f"{distance:.1f}cm → 중간 거리 유지 중 → 현재 상태 유지")

        elif distance > 12:
            if success:
                x, y, w, h, cx, cy = map(int, bbox)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                print(f"{distance:.1f}cm → 거리 안전 + 색상 추적 성공")

                if current_state != "moving":
                    print("→ 차량 전진 시작")
                    car.Car_Run(BASE_SPEED + LEFT_TRIM, BASE_SPEED + RIGHT_TRIM)
                    current_state = "moving"
            else:
                elapsed = time.time() - tracker.get_last_success_time()
                print(f"트래커 실패. 마지막 성공 후 {elapsed:.2f}초 경과")

                if elapsed > FAIL_TIMEOUT:
                    if current_state != "stopped":
                        print("1초 이상 추적 실패 → 차량 정지")
                        car.Car_Stop()
                        current_state = "stopped"
                else:
                    print("일시적 추적 실패 → 상태 유지")

        # ───────── 영상 출력 및 종료 처리 ─────────
        cv2.imshow("Object Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("사용자 종료 (q)")
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    print("종료 (Ctrl+C)")

finally:
    print("차량 정지 및 자원 해제")
    car.Car_Stop()
    distance_sensor.cleanup()
    tracker.release()
    cv2.destroyAllWindows()
