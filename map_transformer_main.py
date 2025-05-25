#!/usr/bin/env python3
# coding: utf-8
"""
마우스로 지도(PGM)를 클릭해 Way-point를 지정하면
팔로우카가 순차적으로 해당 지점까지 주행한다.
※ 사용 라이브러리: cv2, numpy, time, math (파이썬 표준), map_transformer,
   YB_Pcb_Car, DistanceController
"""

import cv2, math, time, numpy as np
from collections import deque
from pathlib import Path
from map_transformer import MapTransformer          # 픽셀↔월드 변환
from YB_Pcb_Car import YB_Pcb_Car                   # 모터 제어
from DistanceSensor import DistanceController       # 초음파 센서

# ───── 환경 설정 ────────────────────────────────────────────────────────────
MAP_DIR   = "/home/pi/Yahboom_project/Raspbot/raspbot/Slam_png"
YAML_PATH = f"{MAP_DIR}/3F_lab.yaml"

BASE_SPEED       = 40      # 직진 기본 PWM
STEER_GAIN       = 1.2     # 각도 오차 → 속도차 비례계수
WAY_RADIUS       = 0.30    # Way-point 도달 반경 [m]
LOOK_AHEAD       = 0.20    # 속도 감속 시작 거리 [m]
DIST_STOP_CM     = 15      # 장애물 정지 임계 [cm]
DIST_GO_CM       = 20      # 재출발 임계 [cm]
LIN_PER_PWM      = 0.002   # [m/s] = PWM×계수  (차량 실측 후 조정)
ANG_PER_DIFF     = 0.03    # [rad/s] = (R-L)×계수

# ───── 클래스: 간단 Dead-reckon 오도메트리 ────────────────────────────────────
class DeadReckon:
    def __init__(self):
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0      # 월드[m], rad
    def update(self, dt, l_pwm, r_pwm):
        v  = (l_pwm + r_pwm) * 0.5 * LIN_PER_PWM
        w  = (r_pwm - l_pwm) * ANG_PER_DIFF
        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        self.yaw  = (self.yaw + math.pi) % (2*math.pi) - math.pi  # -π~π

# ───── 초기화 ───────────────────────────────────────────────────────────────
tfm  = MapTransformer(YAML_PATH)                       # 지도 메타
car  = YB_Pcb_Car()
odom = DeadReckon()
dist = DistanceController(trig=16, echo=18)

map_img = cv2.imread(f"{MAP_DIR}/3F_lab.pgm", cv2.IMREAD_GRAYSCALE)
map_bgr = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
H, W    = map_img.shape

queue   = deque()          # Way-point 목록
l_cmd = r_cmd = 0          # 현재 PWM 명령 값

# ───── 마우스 콜백 ──────────────────────────────────────────────────────────
def on_click(event, u, v, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        x, y = tfm.px_to_world(u, v)
        queue.append({"x":x, "y":y})
        print(f"[CLICK] 픽셀({u},{v}) → 월드({x:.2f},{y:.2f})m  큐={len(queue)}")

cv2.namedWindow("MAP")
cv2.setMouseCallback("MAP", on_click)
print("[INFO] 지도 클릭으로 Way-point 지정 시작 (ESC 종료)")

# ───── 메인 루프 ────────────────────────────────────────────────────────────
prev_t = time.time()
running = False            # 장애물 상황에 따른 주행 플래그

try:
    while True:
        # ① 시간·오도메트리 갱신
        now = time.time()
        dt  = now - prev_t
        prev_t = now
        odom.update(dt, l_cmd, r_cmd)

        # ② 장애물 거리 확인
        d_cm = dist.get_distance()
        if d_cm != -1:
            if d_cm < DIST_STOP_CM:
                running = False
            elif d_cm > DIST_GO_CM:
                running = True   # 재출발 허용

        # ③ Way-point 처리
        if queue and running:
            wp  = queue[0]
            dx  = wp["x"] - odom.x
            dy  = wp["y"] - odom.y
            dist_wp = math.hypot(dx, dy)
            ang_wp  = math.atan2(dy, dx)
            yaw_err = (ang_wp - odom.yaw + math.pi) % (2*math.pi) - math.pi

            # 속도 프로파일
            base = BASE_SPEED * (0.4 if dist_wp < LOOK_AHEAD else 1.0)
            steer = int(STEER_GAIN * math.degrees(yaw_err))
            l_cmd = int(max(0, min(100, base - steer)))
            r_cmd = int(max(0, min(100, base + steer)))
            car.Car_Run(l_cmd, r_cmd)

            if dist_wp < WAY_RADIUS:
                print(f"[ARRIVE] Way-point 도달 ({wp['x']:.2f},{wp['y']:.2f})")
                queue.popleft()
        else:
            l_cmd = r_cmd = 0
            car.Car_Stop()

        # ④ 시각화
        vis = map_bgr.copy()
        # 현재 위치
        u,v = tfm.world_to_px(odom.x, odom.y)
        cv2.circle(vis, (u,v), 3, (0,0,255), -1)
        # 큐 표시
        for w in queue:
            uu,vv = tfm.world_to_px(w["x"], w["y"])
            cv2.circle(vis, (uu,vv), 4, (255,0,0), 2)
        # 거리 텍스트
        txt = f"D:{d_cm:.0f}cm" if d_cm!=-1 else "D:--"
        cv2.putText(vis, txt, (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0),2)
        cv2.imshow("MAP", vis)

        if cv2.waitKey(1) == 27:     # ESC 종료
            break
        time.sleep(0.03)

finally:
    print("[INFO] 종료: 차량·자원 정리")
    car.Car_Stop()
    dist.cleanup()
    cv2.destroyAllWindows()
