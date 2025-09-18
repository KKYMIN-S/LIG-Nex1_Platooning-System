#!/usr/bin/env python
# coding: utf-8
import cv2, time, threading, ctypes, inspect, numpy as np
import YB_Pcb_Car
import PID
from Ultrasonic_IR_Avoid import is_ir_obstacle, Distance_test, cleanup

# 모터 제어 객체
car = YB_Pcb_Car.YB_Pcb_Car()

# 카메라 설정
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS,          30)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_BRIGHTNESS,   62)
cap.set(cv2.CAP_PROP_CONTRAST,     63)
cap.set(cv2.CAP_PROP_EXPOSURE,    4800)

# 초기 서보 위치
car.Ctrl_Servo(1, 90)
car.Ctrl_Servo(2, 90)

# 공유 프레임
picture = [np.zeros((240,320,3), dtype=np.uint8)]

# --- 여기에 색상 설정 추가 ---
# 색상별 HSV 범위 정의
color_hsv = {
    "red":    ((0, 70, 72),   (7, 255, 255)),
    "green":  ((54,109, 78),  (77,255,255)),
    "blue":   ((92,100, 62),  (121,251,255)),
    "yellow": ((26,100, 91),  (32,255,255))
}
# 추적할 색상 선택 (나중에 이 값만 바꾸면 됩니다)
color = "blue"
color_lower = np.array(color_hsv[color][0])
color_upper = np.array(color_hsv[color][1])
# --------------------------------------

# 스레드 종료 헬퍼
def _async_raise(tid, exctype):
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype): exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res != 1: ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)

def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)

# 색상 추적 및 주행 제어
def Color_Recognize():
    # PID 초기화
    dir_pid   = PID.PositionalPID(0.8, 0,   0.8)
    servo_pid = PID.PositionalPID(0.8, 0.2, 0.8)
    spd_pid   = PID.PositionalPID(2.1, 0,   0.8)

    t0, cnt = time.time(), 0
    time.sleep(1)
    while True:
        ret, frame = cap.read()
        if not ret: break

        frame = cv2.GaussianBlur(frame, (5,5), 0)
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask  = cv2.inRange(hsv, color_lower, color_upper)
        mask  = cv2.erode(mask,  None, iterations=2)
        mask  = cv2.dilate(mask, None, iterations=2)
        cnts  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        # 팔로잉 & 장애물 회피
        if cnts and not is_ir_obstacle():
            c = max(cnts, key=cv2.contourArea)
            (cx,cy), r = cv2.minEnclosingCircle(c)
            if r > 5:
                cv2.circle(frame, (int(cx),int(cy)), int(r), (255,0,255), 2)
                # 방향 PID
                dir_pid.SystemOutput     = cx
                dir_pid.SetStepSignal(160)
                dir_pid.SetInertiaTime(0.01, 0.1)
                offset = int(dir_pid.SystemOutput)
                # 서보 틸트 PID
                servo_pid.SystemOutput   = cy
                servo_pid.SetStepSignal(120)
                servo_pid.SetInertiaTime(0.01, 0.1)
                tilt = int((1500 - servo_pid.SystemOutput - 500) / 10)
                tilt = max(0, min(180, tilt))
                # 속도 PID
                spd_pid.SystemOutput     = int(r)
                spd_pid.SetStepSignal(50)
                spd_pid.SetInertiaTime(0.01, 0.1)
                speed = max(0, min(200, int(spd_pid.SystemOutput)))
                # 과근접 시 정지
                if Distance_test() < 12:
                    speed = 0
                ls = max(-250, min(250, speed - offset))
                rs = max(-250, min(250, speed + offset))
                car.Control_Car(ls, rs)
                car.Ctrl_Servo(2, tilt)
            else:
                car.Car_Stop()
        else:
            car.Car_Stop()

        cnt += 1
        fps = int(cnt / (time.time() - t0))
        cv2.putText(frame, f"FPS {fps}", (40,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        picture[0] = frame

if __name__=='__main__':
    running = False
    worker  = None

    try:
        while True:
            cv2.imshow('Follow', picture[0])
            key = cv2.waitKey(1) & 0xFF

            if key==ord('w') and not running:
                # w 누르면 팔로잉+회피 시작
                worker = threading.Thread(target=Color_Recognize, daemon=True)
                worker.start()
                running = True

            elif key==ord('s') and running:
                # s 누르면 중단
                stop_thread(worker)
                car.Car_Stop()
                running = False

            elif key==ord('q'):
                # q 누르면 종료
                break

    except KeyboardInterrupt:
        pass

    # 정리
    if running:
        stop_thread(worker)
    cap.release()
    cv2.destroyAllWindows()
    car.Car_Stop()
    cleanup()