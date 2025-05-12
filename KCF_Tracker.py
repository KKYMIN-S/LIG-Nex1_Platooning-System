# KCF_Tracker.py
import cv2
import numpy as np
import time

LOWER_BLUE = np.array([95, 117, 62])
UPPER_BLUE = np.array([115, 230, 149])


class KCFTracker:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH , 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS,          30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        self.tracker = None
        self.tracking = False
        self.object_detected = False
        self.last_success_time = time.time()

        print("자동 탐지 및 추적 시작")

    def detect_blue_object(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
        mask = cv2.erode(mask, None, 2)
        mask = cv2.dilate(mask, None, 2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            largest = max(cnts, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            if w * h > 500:
                print(f"탐지된 바운딩 박스 크기: w = {w}, h = {h}")
                return (x, y, w, h)
        return None

    def create_tracker(self):
        if hasattr(cv2, 'legacy'):
            return cv2.legacy.TrackerKCF_create()
        else:
            return cv2.TrackerKCF_create()

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            print("카메라 프레임 오류")
            return None, False, frame

        if not self.tracking:
            roi = self.detect_blue_object(frame)
            if roi:
                self.tracker = self.create_tracker()
                self.tracker.init(frame, roi)
                self.tracking = True
                self.object_detected = True
                self.last_success_time = time.time()
                print("탐지 성공! 직진 시작!")
                x, y, w, h = roi
                cx = x + w // 2
                cy = y + h // 2
                return (x, y, w, h, cx, cy), True, frame
            else:
                if self.object_detected:
                    print("물체 사라짐! 정지 후 재탐색...")
                    self.object_detected = False
                return None, False, frame
        else:
            success, bbox = self.tracker.update(frame)
            if success:
                x, y, w, h = map(int, bbox)
                cx = x + w // 2
                cy = y + h // 2
                self.last_success_time = time.time()
                print(f"추적 성공: 중심 좌표 ({cx}, {cy})")
                return (x, y, w, h, cx, cy), True, frame
            else:
                print("트래커 추적 실패")
                elapsed = time.time() - self.last_success_time

                if elapsed > 1.0:
                    print("1초 이상 추적 실패 → 재탐지 모드 전환")
                    self.tracking = False
                    self.object_detected = False
                return None, False, frame

    def get_last_success_time(self):
        return self.last_success_time

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
