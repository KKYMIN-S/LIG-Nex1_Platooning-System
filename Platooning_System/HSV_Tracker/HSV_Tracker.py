import cv2
import numpy as np
import time

LOWER_BLUE = np.array([95, 150, 70])
UPPER_BLUE = np.array([115, 255, 255])

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
                # print(f"[HSV 추적 성공] 중심 좌표: ({cx}, {cy})")
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
