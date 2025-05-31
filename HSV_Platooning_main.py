from YB_Pcb_Car import YB_Pcb_Car
from HSV_Tracker import HSVTracker
from Camera_Tracker import CameraTracker
from DistanceSensor import DistanceController
from MQTT_Client import MqttHandler
import cv2
import time
import json

# 차량 및 트래커 초기화
car = YB_Pcb_Car()
tracker = HSVTracker()
camera_tracker = CameraTracker(
    car,
    tracker,
    pan_channel=1,
    tilt_channel=2,
    center_yaw=110,       # 초기 Yaw 각도
    center_pitch=80,      # 초기 Pitch 각도
    dead_zone=30
)
ultrasonic = DistanceController(trig=16, echo=18)

# 주행 파라미터
BASE_SPEED = 65
LEFT_TRIM = 5
RIGHT_TRIM = 0
STEER_GAIN = 0.5

current_state = "stopped"
FAIL_TIMEOUT = 1.0
FAIL_TOLERANCE = 3
consecutive_failures = 0

last_distance_time = 0
last_distance = -1

# 카메라 스캔 변수
scan_direction = 1
scan_step = 3
scan_limit = 60
scan_reset_done = False

# 주행 상태 변수
is_driving_enabled = False

# ✅ MQTT 콜백 함수 정의
def mqtt_message_handler(client, userdata, msg):
    global is_driving_enabled, current_state
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)
        action = data.get("action", "")
        if action == "run":
            print("[MQTT] 주행 활성화 명령 수신")
            is_driving_enabled = True
        elif action == "stop":
            print("[MQTT] 주행 중지 명령 수신")
            is_driving_enabled = False
            if current_state != "stopped":
                car.Car_Stop()
                current_state = "stopped"
    except Exception as e:
        print(f"[MQTT] 메시지 처리 오류: {e}")

# ✅ MQTT 핸들러 초기화
mqtt = MqttHandler(broker_ip="192.168.0.45", on_msg_callback=mqtt_message_handler)
mqtt.start_subscriber()

try:
    while True:
        now = time.time()

        # ✅ [1] 거리 측정 (0.5초마다)
        if now - last_distance_time >= 0.5:
            new_distance = ultrasonic.get_kalman_distance()
            if new_distance > 0:
                last_distance = new_distance
                print(f"[거리] {last_distance:.2f} cm")
            last_distance_time = now

        # ✅ [2] HSV 추적
        bbox, success, frame = camera_tracker.track_and_adjust()

        # ✅ [3] 바운딩 박스와 정보 시각화
        if frame is not None:
            if success:
                x, y, w, h, cx, cy = map(int, bbox)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            cv2.putText(frame, f"Yaw: {int(camera_tracker.servo_yaw)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Dist: {last_distance:.1f} cm", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("Object Tracking", frame)

        # ✅ [4] 주행 로직
        if 0 < last_distance <= 15:
            if current_state != "stopped":
                print("[정지] 15cm 이내 장애물")
                car.Car_Stop()
                mqtt.publish({"action": "stop"})
                current_state = "stopped"

        elif success and is_driving_enabled:
            consecutive_failures = 0
            scan_reset_done = False
            yaw_error = camera_tracker.servo_yaw - camera_tracker.center_yaw

            if abs(yaw_error) < 5:
                left_speed = BASE_SPEED + LEFT_TRIM
                right_speed = BASE_SPEED + RIGHT_TRIM
            else:
                turn = int(STEER_GAIN * yaw_error)
                left_speed = BASE_SPEED - turn
                right_speed = BASE_SPEED + turn
                left_speed = max(0, min(100, left_speed))
                right_speed = max(0, min(100, right_speed))

            car.Car_Run(left_speed, right_speed)

            if current_state != "moving":
                mqtt.publish({"action": "run"})
                current_state = "moving"

        elif success and not is_driving_enabled:
            if current_state != "stopped":
                car.Car_Stop()
                current_state = "stopped"

        elif not success:
            consecutive_failures += 1
            print(f"[추적 실패] {consecutive_failures}/{FAIL_TOLERANCE}")

            if consecutive_failures >= FAIL_TOLERANCE:
                elapsed = time.time() - tracker.get_last_success_time()
                if elapsed > FAIL_TIMEOUT:
                    if current_state != "stopped":
                        print("[정지] 추적 실패")
                        car.Car_Stop()
                        mqtt.publish({"action": "stop"})
                        current_state = "stopped"

                    if not scan_reset_done:
                        camera_tracker.reset_servo()
                        scan_reset_done = True
                        print("[서보 초기화] 카메라 초기 위치로 복귀")

                    scan_direction = camera_tracker.scan_servo_yaw(scan_direction, scan_step, scan_limit)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    print("종료 (Ctrl+C)")

finally:
    print("차량 정지 및 자원 해제")
    car.Car_Stop()
    camera_tracker.reset_servo()
    tracker.release()
    ultrasonic.cleanup()
    cv2.destroyAllWindows()
