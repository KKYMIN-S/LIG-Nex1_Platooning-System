#!/usr/bin/env python3
# coding: utf-8

import cv2
import time
import numpy as np
from YB_Pcb_Car import YB_Pcb_Car
from KCF_Tracker import KCFTracker
from DistanceSensor import DistanceController
from Hough_Detector import HoughDetector

def order_rect_pts(x,y,w,h):
    # KCF 박스 (x,y,w,h) → 4개 꼭짓점 src_pts
    return np.array([
        [x,     y    ],  # tl
        [x+w,   y    ],  # tr
        [x+w,   y+h  ],  # br
        [x,     y+h  ]   # bl
    ], dtype="float32")

# ───── 시스템 초기화 ─────
car             = YB_Pcb_Car()
tracker         = KCFTracker()
distance_sensor = DistanceController()

# ───── 주행 파라미터 ─────
BASE_SPEED  = 40
LEFT_TRIM   = -6
RIGHT_TRIM  = 0
MIN_DIST_CM = 40

# ───── HSV 마스크 ─────
LOWER_BLUE = np.array([95,117,62])
UPPER_BLUE = np.array([115,230,149])

# ───── 서보(Pan/Tilt) 설정 ─────
CENTER_YAW   = 70    # 팬
CENTER_PITCH = 100   # 틸트
servo_yaw    = CENTER_YAW
servo_pitch  = CENTER_PITCH
SERVO_STEP   = 2
DEAD_ZONE    = 15    # 수평 허용(px)
DEAD_ZONE_Y  = 15    # 수직 허용(px)

car.Ctrl_Servo(1, servo_yaw)
car.Ctrl_Servo(2, servo_pitch)

# ───── Hough 원 검출기 ─────
hough = HoughDetector()

# ───── 상태 변수 ─────
current_state = "moving"
FAIL_TIMEOUT  = 1.0
last_success  = time.time()

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH , 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS         , 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE  , 1)

print("[INFO] Tracker+HSV+Hough+Pitch+Perspective 모드 시작")

# 초기화 대기
time.sleep(1.0)
for _ in range(5):
    cap.read()

try:
    while True:
        bbox, success, frame = tracker.update()
        h, w = frame.shape[:2]
        cx_frame = w // 2
        cy_frame = h // 2

        if success:
            x, y, bw, bh, cx, cy = map(int, bbox)
            last_success = time.time()

            # 1) KCF 박스 표시
            cv2.rectangle(frame, (x,y), (x+bw,y+bh), (0,255,0), 2)

            # 2) 원근 보정: 박스 네 꼭짓점 → 정사각형(dst_w x dst_h)
            src_pts = order_rect_pts(x,y,bw,bh)
            dst_w, dst_h = bw, bh
            dst_pts = np.array([
                [0,     0    ],
                [dst_w, 0    ],
                [dst_w, dst_h],
                [0,     dst_h]
            ], dtype="float32")
            M      = cv2.getPerspectiveTransform(src_pts, dst_pts)
            warped = cv2.warpPerspective(frame, M, (dst_w, dst_h))
            cv2.imshow("Warped ROI", warped)

            # 3) HSV 마스크 ROI & Hough (보정된 warped 위에서)
            hsv_w  = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
            mask_w = cv2.inRange(hsv_w, LOWER_BLUE, UPPER_BLUE)
            gray_w = cv2.cvtColor(cv2.bitwise_and(warped, warped, mask=mask_w),
                                  cv2.COLOR_BGR2GRAY)
            blur_w = cv2.GaussianBlur(gray_w, (5,5), 0)
            circles = hough.detect(blur_w)

            # 4) 검출된 원 역투영 → 원본 프레임
            if circles:
                Minv = np.linalg.inv(M)
                for (wx, wy, r) in circles:
                    # warped 좌표 → 원본
                    pt_src = np.array([[[wx, wy]]], dtype="float32")
                    pt_dst = cv2.perspectiveTransform(pt_src, Minv)
                    px, py = pt_dst[0,0].astype(int)
                    cv2.circle(frame, (px,py), int(r* (bw/dst_w)), (255,0,0), 2)
                    cv2.circle(frame, (px,py), 3, (0,0,255), -1)
            else:
                print("[WARN] Perspective→Hough: 원 검출 0개")

            # 5) Pan 보정 (KCF 중심 기준)
            err_x = cx - cx_frame
            if abs(err_x) > DEAD_ZONE:
                old = servo_yaw
                servo_yaw += -SERVO_STEP if err_x>0 else SERVO_STEP
                servo_yaw = np.clip(servo_yaw, 0, 180)
                car.Ctrl_Servo(1, servo_yaw)
                print(f"[PAN] err_x={err_x}, yaw {old}->{servo_yaw}")

            # 6) Tilt 보정 (KCF 중심 기준)
            err_y = cy - cy_frame
            if abs(err_y) > DEAD_ZONE_Y:
                oldp = servo_pitch
                servo_pitch += SERVO_STEP if err_y>0 else -SERVO_STEP
                servo_pitch = np.clip(servo_pitch, 0, 180)
                car.Ctrl_Servo(2, servo_pitch)
                print(f"[TILT] err_y={err_y}, pitch {oldp}->{servo_pitch}")

        else:
            # KCF 실패 1초 이상 시 정지
            if time.time() - last_success > FAIL_TIMEOUT:
                car.Car_Stop()
                current_state = "stopped"

        # 7) 결과 화면 및 종료
        cv2.imshow("Tracker+HSV+Hough+Pitch+Perspective", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.05)

finally:
    print("[INFO] 정리 중: 차량 정지 및 자원 해제")
    car.Car_Stop()
    distance_sensor.cleanup()
    tracker.release()
    cap.release()
    cv2.destroyAllWindows()
