# -------
import cv2
import time
import numpy as np
from YB_Pcb_Car import YB_Pcb_Car
from KCF_Tracker import KCFTracker
from DistanceSensor import DistanceController
from Hough_Detector import HoughDetector

# 시스템 초기화
car             = YB_Pcb_Car()
tracker         = KCFTracker()
distance_sensor = DistanceController()

# 주행 파라미터
BASE_SPEED    = 40
LEFT_TRIM     = -6
RIGHT_TRIM    = 0
MIN_DIST_CM   = 10

# HSV 파란색 마스크
LOWER_BLUE = np.array([95,117,62])
UPPER_BLUE = np.array([115,230,149])

# 서보 세팅
CENTER_YAW = 70
servo_yaw  = CENTER_YAW
SERVO_STEP = 2
DEAD_ZONE  = 15
car.Ctrl_Servo(1, servo_yaw)
car.Ctrl_Servo(2, 110)

# HoughDetector 모듈 생성
hough = HoughDetector(dp=1.2, minDist=20, param1=50, param2=20, minRadius=5, maxRadius=30)

# 상태 변수
current_state = "moving"
FAIL_TIMEOUT  = 1.0
last_success  = time.time()

cap = cv2.VideoCapture(0)
while True:
    bbox, success, frame = tracker.update()
    h, w = frame.shape[:2]
    center_x = w//2

    if success:
        x, y, bw, bh, cx, cy = map(int, bbox)
        cv2.rectangle(frame, (x,y), (x+bw, y+bh), (0,255,0), 2)
        last_success = time.time()

        # HSV 마스크 ROI
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
        roi  = cv2.bitwise_and(frame, frame, mask=mask)[y:y+bh, x:x+bw]

        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur_roi = cv2.GaussianBlur(gray_roi, (5,5), 0)

        # 원 검출
        circles = hough.detect(blur_roi)
        for crx, cry, r in circles:
            px, py = x+crx, y+cry
            cv2.circle(frame, (px,py), r, (255,0,0), 2)
            cv2.circle(frame, (px,py), 2, (0,0,255), 3)

        # Pan 보정
        err = cx - center_x
        if abs(err) > DEAD_ZONE:
            old = servo_yaw
            servo_yaw = servo_yaw - SERVO_STEP if err>0 else servo_yaw + SERVO_STEP
            servo_yaw = max(0, min(180, servo_yaw))
            car.Ctrl_Servo(1, servo_yaw)

        # 거리 측정
        dist = distance_sensor.get_distance()
        if dist<0 or dist<MIN_DIST_CM:
            car.Car_Stop()
            current_state = "stopped"
        else:
            if current_state=="moving":
                car.Car_Run(BASE_SPEED+LEFT_TRIM, BASE_SPEED+RIGHT_TRIM)
            elif dist>MIN_DIST_CM:
                car.Car_Run(BASE_SPEED+LEFT_TRIM, BASE_SPEED+RIGHT_TRIM)
                current_state = "moving"

    else:
        if time.time()-last_success>FAIL_TIMEOUT:
            car.Car_Stop()
            current_state = "stopped"

    cv2.imshow("Tracker+HSV+Hough", frame)
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
    time.sleep(0.05)

car.Car_Stop()
distance_sensor.cleanup()
tracker.release()
cv2.destroyAllWindows()
