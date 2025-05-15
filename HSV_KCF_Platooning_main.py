from YB_Pcb_Car import YB_Pcb_Car
from HSV_KCF_Tracker import KCFTracker
from Camera_Tracker import CameraTracker
import cv2
import time

# ───────── 객체 초기화 ─────────
car = YB_Pcb_Car()
tracker = KCFTracker()
camera_tracker = CameraTracker(
    car,
    tracker,
    pan_channel=1,
    tilt_channel=2,
    center_yaw=110,
    center_pitch=80,
    dead_zone=30
)

# ───────── 주행 설정 ─────────
BASE_SPEED = 40
LEFT_TRIM = 0
RIGHT_TRIM = -5
STEER_GAIN = 0.6

# ──────── 상태 변수 ────────
current_state = "stopped"
FAIL_TIMEOUT = 1.0
FAIL_TOLERANCE = 3
consecutive_failures = 0

try:
    while True:
        # 카메라 트래킹 및 서보 조절
        bbox, success, frame = camera_tracker.track_and_adjust()

        if success:
            consecutive_failures = 0
            x, y, w, h, cx, cy = map(int, bbox)
            print("색상 추적 성공 → 차량 주행 유지")

            # ✅ 카메라 yaw 오차 계산
            yaw_error = camera_tracker.servo_yaw - camera_tracker.center_yaw

            if abs(yaw_error) < 5:
                # 정면이면 직진
                left_speed = BASE_SPEED + LEFT_TRIM
                right_speed = BASE_SPEED + RIGHT_TRIM
            else:
                # 각도에 따라 좌/우 속도 차이 적용 (비례 제어)
                turn = int(STEER_GAIN * yaw_error)
                left_speed = BASE_SPEED - turn
                right_speed = BASE_SPEED + turn

                # 속도 제한 (0~100)
                left_speed = max(0, min(100, left_speed))
                right_speed = max(0, min(100, right_speed))

            # 차량 전진 명령
            car.Car_Run(left_speed, right_speed)
            current_state = "moving"

        else:
            consecutive_failures += 1
            print(f"추적 실패 ({consecutive_failures}/{FAIL_TOLERANCE})")

            if consecutive_failures >= FAIL_TOLERANCE:
                elapsed = time.time() - tracker.get_last_success_time()
                print(f"연속 실패 지속: {elapsed:.2f}초")

                if elapsed > FAIL_TIMEOUT:
                    if current_state != "stopped":
                        print("1초 이상 연속 실패 → 차량 정지")
                        car.Car_Stop()
                        current_state = "stopped"
                else:
                    print("실패 지속 짧음 → 차량 유지")
            else:
                print("일시적 실패 → 차량 유지")

        # ───────── 영상 출력 및 종료 처리 ─────────
        if frame is not None:
            # 현재 카메라 각도 디버깅 출력
            cv2.putText(frame, f"Yaw: {int(camera_tracker.servo_yaw)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
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
    camera_tracker.reset_servo()
    tracker.release()
    cv2.destroyAllWindows()
