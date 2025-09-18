import cv2
import time

class CameraTracker:
    def __init__(self, car, tracker=None,
                 pan_channel=1, tilt_channel=2,
                 center_yaw=110, center_pitch=80,
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

        # ▶ 부드러운 Pan 조정 (비례 제어)
        if abs(err_x) > self.dead_zone:
            proportional_step = 0.05 * abs(err_x)
            step = max(0.1, min(proportional_step, 7))
            if err_x > 0:
                self.servo_yaw = max(self.yaw_min, self.servo_yaw - step)
            else:
                self.servo_yaw = min(self.yaw_max, self.servo_yaw + step)
            self.car.Ctrl_Servo(self.pan_channel, int(self.servo_yaw))

        # ▶ 부드러운 Pitch 조정 (비례 제어)
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

    def scan_servo_yaw(self, scan_direction, scan_step, scan_limit):
        self.servo_yaw += scan_direction * scan_step

        if abs(self.servo_yaw - self.center_yaw) >= scan_limit:
            scan_direction *= -1  # 방향 반전

        # yaw_min, yaw_max 범위 체크
        self.servo_yaw = max(self.yaw_min, min(self.yaw_max, self.servo_yaw))

        # yaw만 업데이트
        self.car.Ctrl_Servo(self.pan_channel, int(self.servo_yaw))

        # pitch를 강제로 고정 (안전장치)
        self.car.Ctrl_Servo(self.tilt_channel, int(self.servo_pitch))

        return scan_direction
