import cv2
import cv2.aruco as aruco
import time

class ArucoDetector:
    def __init__(self, target_id=0, timeout=1.0, dictionary=aruco.DICT_5X5_100):
        self.aruco_dict = aruco.Dictionary_get(dictionary)
        self.parameters = aruco.DetectorParameters_create()
        self.target_id = target_id
        self.timeout = timeout
        self.last_seen_time = time.time()

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        current_time = time.time()

        is_target_detected = False
        if ids is not None and self.target_id in ids:
            self.last_seen_time = current_time
            is_target_detected = True
            aruco.drawDetectedMarkers(frame, corners, ids)

        timeout_exceeded = (current_time - self.last_seen_time > self.timeout)
        return frame, is_target_detected, timeout_exceeded
