#!/usr/bin/env python3

from YB_Pcb_Car import YB_Pcb_Car
from HSV_Tracker import HSVTracker
from Camera_Tracker import CameraTracker
from MQTT_Client import MqttHandler
from DistanceSensor import DistanceController
import cv2
import time

# 차량 및 트래커 초기화
car = YB_Pcb_Car()
tracker = HSVTracker()
camera_tracker = CameraTracker(
    car, tracker,
    pan_channel=1,
    tilt_channel=2,
    center_yaw=70,
    center_pitch=100,
    dead_zone=30
)

# 초음파 거리 센서
distance_sensor = DistanceController(trig=16, echo=18)

# MQTT 설정
mqtt = MqttHandler(
    broker_ip="172.20.10.2",
    client_id="hsv_platooning_subscriber"
)
mqtt.start_subscriber()

# ───────── MQTT 'Start_Nav!' 수신 핸들러 ─────────
def on_start_nav(client, userdata, message):
    global is_driving_enabled
    payload = message.payload.decode()
    print(f"[MQTT] 수신: {payload} on topic {message.topic}")
    if payload.strip() == "Start_Nav!":
        print("[MQTT] 주행 활성화 신호 수신: 55초 후 주행 시작")
        time.sleep(40)
        is_driving_enabled = True
        print("[MQTT] 55초 경과, 주행 활성화됨")

# 토픽 구독 및 콜백 등록
mqtt.subscribe("ros2/start_sign")
mqtt.add_callback("ros2/start_sign", on_start_nav)

# 주행 파라미터
BASE_SPEED = 85
LEFT_TRIM = 0
RIGHT_TRIM = 5
STEER_GAIN = 0.5
FAIL_TIMEOUT = 1.0
FAIL_TOLERANCE = 3

last_distance = -1
scan_direction = 1
scan_step = 3
scan_limit = 60

# 상태 변수
is_driving_enabled = False
current_state = "stopped"
consecutive_failures = 0
too_close = False
is_scanning = False

try:
    while True:
        now = time.time()
        last_distance = distance_sensor.get_distance()

        # HSV 추적
        bbox, success, frame = camera_tracker.track_and_adjust()

        # 시각화
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

        # 거리 기반 정지
        if 0 < last_distance <= 30:
            if not too_close:
                print("[정지] 30cm 이하 장애물 감지")
                car.Car_Stop()
                mqtt.publish({"action": "stop"})
                current_state = "stopped"
                too_close = True

        elif last_distance > 45:
            if too_close:
                print("[재개] 장애물 멀어짐 (45cm 이상)")
                too_close = False

        # 객체 탐지 성공 시 스캔 모드 해제
        if success:
            consecutive_failures = 0
            if is_scanning:
                print("[스캔 중단] 객체 재탐지 성공")
                is_scanning = False

            if is_driving_enabled and not too_close:
                yaw_error = camera_tracker.servo_yaw - camera_tracker.center_yaw

                if current_state != "moving":
                    print("[주행 시작]")
                    mqtt.publish({"action": "run"})
                    current_state = "moving"

                left_speed = BASE_SPEED + LEFT_TRIM
                right_speed = BASE_SPEED + RIGHT_TRIM

                if abs(yaw_error) >= 5:
                    turn = int(STEER_GAIN * yaw_error)
                    left_speed = max(0, min(100, BASE_SPEED - turn))
                    right_speed = max(0, min(100, BASE_SPEED + turn))

                car.Car_Run(left_speed, right_speed)
            elif current_state == "moving":
                car.Car_Stop()
                current_state = "stopped"

        # 객체 탐지 실패
        elif not success:
            consecutive_failures += 1
            print(f"[추적 실패] {consecutive_failures}/{FAIL_TOLERANCE}")

            if consecutive_failures >= FAIL_TOLERANCE:
                elapsed = time.time() - tracker.get_last_success_time()
                if elapsed > FAIL_TIMEOUT:
                    if not is_scanning:
                        print("[정지] 객체 탐지 실패 → 초기위치 복귀 및 스캔모드 진입")
                        if current_state != "stopped":
                            car.Car_Stop()
                            mqtt.publish({"action": "stop"})
                            current_state = "stopped"

                        camera_tracker.reset_servo()
                        time.sleep(0.2)
                        is_scanning = True

            if is_scanning:
                scan_direction = camera_tracker.scan_servo_yaw(scan_direction, scan_step, scan_limit=60)

        # 키 입력
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('w'):
            print("[입력] 주행 활성화")
            is_driving_enabled = True
            if not too_close and current_state != "moving":
                mqtt.publish({"action": "run"})

        elif key == ord('s'):
            print("[입력] 주행 중지")
            car.Car_Stop()
            mqtt.publish({"action": "stop"})
            current_state = "stopped"
            is_driving_enabled = False

        time.sleep(0.05)

except KeyboardInterrupt:
    print("종료 (Ctrl+C)")

finally:
    print("차량 정지 및 자원 해제")
    car.Car_Stop()
    camera_tracker.reset_servo()
    tracker.release()
    distance_sensor.cleanup()
    cv2.destroyAllWindows()
