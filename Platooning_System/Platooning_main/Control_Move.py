#!/usr/bin/env python3
# coding: utf-8
import cv2, time
from YB_Pcb_Car import YB_Pcb_Car
from IMU_Data import IMUDataCollector

car = YB_Pcb_Car()

# ───── I2C 전송 로깅 ─────
car._orig_Ctrl_Car = car.Ctrl_Car
def Ctrl_Car_dbg(l_dir, l_spd, r_dir, r_spd):
    print(f"[I2C] L({l_dir},{l_spd}) R({r_dir},{r_spd})")
    car._orig_Ctrl_Car(l_dir, l_spd, r_dir, r_spd)
car.Ctrl_Car = Ctrl_Car_dbg
# ──────────────────────────

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH , 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS,          30)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

# ───── 주행 파라미터 (가변) ─────
BASE_SPEED  = 70
LEFT_TRIM   = 0
RIGHT_TRIM  = 5
LEFT_GAIN   = 0.99  # ← 왼쪽 PWM 보정w
RIGHT_GAIN  = 1.00   # ← 오른쪽 PWM 보정
RUN_TIME    = 5.0
ANG_STEP    = 5
until       = 0

angle_v = 100
angle_h = 70
car.Ctrl_Servo(1, angle_h)
car.Ctrl_Servo(2, angle_v)

def clipped(v):          # 0~100 클램프
    return max(0, min(100, v))

def apply_trim(base, l_trim, r_trim):
    l = clipped(base + l_trim) * LEFT_GAIN
    r = clipped(base + r_trim) * RIGHT_GAIN
    return int(l), int(r)

def show_params():
    print(f"\nBASE_SPEED   = {BASE_SPEED}")
    print(f"LEFT_TRIM    = {LEFT_TRIM}")
    print(f"RIGHT_TRIM   = {RIGHT_TRIM}")
    print(f"LEFT_GAIN    = {LEFT_GAIN:.2f}")
    print(f"RIGHT_GAIN   = {RIGHT_GAIN:.2f}\n")

imu = IMUDataCollector()
imu.start_collecting()

try:
    show_params()
    while True:
        ok, frame = cap.read(); assert ok
        cv2.imshow('Raspbot Camera', frame)

        print(f"[IMU] Yaw (heading): {imu.heading:.2f}°", end='\r')

        if until and time.time() >= until:
            car.Car_Stop(); until = 0

        key = cv2.waitKey(1) & 0xFF

        # ───── 주행 키 ─────
        if   key == ord('w'):
            l, r = apply_trim(BASE_SPEED, LEFT_TRIM, RIGHT_TRIM)
            car.Car_Run(l, r); until = time.time() + RUN_TIME
        elif key == ord('x'):
            l, r = apply_trim(BASE_SPEED, LEFT_TRIM, RIGHT_TRIM)
            car.Car_Back(l, r); until = time.time() + RUN_TIME
        elif key == ord('a'):
            car.Car_Left(BASE_SPEED, BASE_SPEED); until = time.time() + RUN_TIME
        elif key == ord('d'):
            car.Car_Right(BASE_SPEED, BASE_SPEED); until = time.time() + RUN_TIME
        elif key == ord('z'):
            car.Car_Spin_Left(BASE_SPEED, BASE_SPEED); until = time.time() + RUN_TIME
        elif key == ord('c'):
            car.Car_Spin_Right(BASE_SPEED, BASE_SPEED); until = time.time() + RUN_TIME
        elif key == ord('s'):
            car.Car_Stop(); until = 0

        # ───── 파라미터 실시간 조정 ─────
        elif key == ord('u'):   BASE_SPEED  += 1; show_params()
        elif key == ord('j'):   BASE_SPEED  -= 1; show_params()
        elif key == ord('h'):   LEFT_TRIM   += 1; show_params()
        elif key == ord('k'):   LEFT_TRIM   -= 1; show_params()
        elif key == ord('n'):   RIGHT_TRIM  += 1; show_params()
        elif key == ord('m'):   RIGHT_TRIM  -= 1; show_params()
        elif key == ord('y'):   LEFT_GAIN   = round(LEFT_GAIN + 0.01, 2); show_params()
        elif key == ord('t'):   LEFT_GAIN   = round(max(0, LEFT_GAIN - 0.01), 2); show_params()
        elif key == ord('o'):   RIGHT_GAIN  = round(RIGHT_GAIN + 0.01, 2); show_params()
        elif key == ord('i'):   RIGHT_GAIN  = round(max(0, RIGHT_GAIN - 0.01), 2); show_params()
        elif key == ord('p'):   show_params()

        # ───── 서보 카메라 ─────
        elif key == 82:  # ↑
            angle_v = max(0, angle_v - ANG_STEP); car.Ctrl_Servo(2, angle_v)
        elif key == 84:  # ↓
            angle_v = min(180, angle_v + ANG_STEP); car.Ctrl_Servo(2, angle_v)
        elif key == 81:  # ←
            angle_h = min(180, angle_h + ANG_STEP); car.Ctrl_Servo(1, angle_h)
        elif key == 83:  # → 
            angle_h = max(0, angle_h - ANG_STEP); car.Ctrl_Servo(1, angle_h)

        elif key == ord('q'):
            break

finally:
    car.Car_Stop()
    cap.release()
    cv2.destroyAllWindows()
