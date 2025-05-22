import cv2
import numpy as np
import time
from YB_Pcb_Car import YB_Pcb_Car

LOWER_BLUE = np.array([90, 138, 90])
UPPER_BLUE = np.array([132, 255, 255])

class HSVTracker:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH , 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS,          30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        self.last_success_time = time.time()
        print("[HSVTracker] HSV 기반 추적 시작")

    def track_object(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            if w * h > 1000:
                cx = x + w // 2
                cy = y + h // 2
                self.last_success_time = time.time()
                print(f"[HSV 추적 성공] 중심 좌표: ({cx}, {cy})")
                return (x, y, w, h, cx, cy), True
        return None, False

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            print("[오류] 카메라 프레임 획득 실패")
            return None, False, None

        bbox, success = self.track_object(frame)
        return bbox, success, frame

    def get_last_success_time(self):
        return self.last_success_time

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

class CameraTracker:
    def __init__(self, car, tracker=None,
                 pan_channel=1, tilt_channel=2,
                 center_yaw=110, center_pitch=70,
                 dead_zone=30,
                 yaw_limit=(0, 180), pitch_limit=(0, 180)):
        self.car = car
        self.tracker = tracker

        self.pan_channel = pan_channel
        self.tilt_channel = tilt_channel

        self.servo_yaw = center_yaw
        self.servo_pitch = center_pitch
        self.center_yaw = center_yaw
        self.center_pitch = center_pitch

        self.dead_zone = dead_zone
        self.yaw_min, self.yaw_max = yaw_limit
        self.pitch_min, self.pitch_max = pitch_limit

        self.car.Ctrl_Servo(self.pan_channel, self.servo_yaw)
        self.car.Ctrl_Servo(self.tilt_channel, self.servo_pitch)

    def track_and_adjust(self):
        if self.tracker is None:
            return None, False, None

        bbox, success, frame = self.tracker.update()
        if frame is None:
            return None, False, None

        if success:
            x, y, w, h, cx, cy = map(int, bbox)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            self.adjust_with_coords(cx, cy, frame)

        return bbox, success, frame

    def adjust_with_coords(self, cx, cy, frame):
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        err_x = cx - frame_center_x
        err_y = cy - frame_center_y

        if abs(err_x) > self.dead_zone:
            proportional_step = 0.05 * abs(err_x)
            step = max(0.1, min(proportional_step, 7))
            if err_x > 0:
                self.servo_yaw = max(self.yaw_min, self.servo_yaw - step)
            else:
                self.servo_yaw = min(self.yaw_max, self.servo_yaw + step)
            self.car.Ctrl_Servo(self.pan_channel, int(self.servo_yaw))

        if abs(err_y) > self.dead_zone:
            proportional_step = 0.05 * abs(err_y)
            step = max(0.1, min(proportional_step, 7))
            if err_y > 0:
                self.servo_pitch = min(self.pitch_max, self.servo_pitch + step)
            else:
                self.servo_pitch = max(self.pitch_min, self.servo_pitch - step)
            self.car.Ctrl_Servo(self.tilt_channel, int(self.servo_pitch))

    def reset_servo(self):
        self.servo_yaw = self.center_yaw
        self.servo_pitch = self.center_pitch
        self.car.Ctrl_Servo(self.pan_channel, self.servo_yaw)
        self.car.Ctrl_Servo(self.tilt_channel, self.servo_pitch)

if __name__ == "__main__":
    car = YB_Pcb_Car()
    tracker = HSVTracker()
    camera_tracker = CameraTracker(car, tracker)

    try:
        while True:
            bbox, success, frame = camera_tracker.track_and_adjust()
            if frame is not None:
                cv2.putText(frame, f"Yaw: {int(camera_tracker.servo_yaw)}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, f"Pitch: {int(camera_tracker.servo_pitch)}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.imshow("HSV Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[종료] 사용자 인터럽트")

    finally:
        tracker.release()
        camera_tracker.reset_servo()
        car.Car_Stop()
        print("[정리] 서보 초기화 및 차량 정지 완료")
