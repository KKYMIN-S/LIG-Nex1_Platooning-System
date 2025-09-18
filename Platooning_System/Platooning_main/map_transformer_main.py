#!/usr/bin/env python3
# coding: utf-8
"""
지도(PGM) 창을 클릭해 Way-point를 지정하면 팔로우카가 순차 주행한다.
p 키로 속도·조향·안전·오도메트리·PWM 비율(LEFT_GAIN/RIGHT_GAIN) 등을
실시간으로 조정할 수 있는 버전.
"""

import cv2, math, time, numpy as np
from collections import deque
from Map_Transformer import MapTransformer
from YB_Pcb_Car import YB_Pcb_Car
from DistanceSensor import DistanceController

# ─────────── 지도 경로
MAP_DIR   = "/home/pi/Yahboom_project/Raspbot/raspbot/Slam_Data"
YAML_PATH = f"{MAP_DIR}/3F_lab.yaml"
PGM_PATH  = f"{MAP_DIR}/3F_lab.pgm"

# ─────────── 주행 파라미터 (기본값)
params = {
    # ── 속도 & 조향 ───────────────────────────
    "BASE_SPEED"   : 50,
    "LEFT_TRIM"    : -5,
    "RIGHT_TRIM"   : 0,
    "STEER_GAIN"   : 0.5,
    "LEFT_GAIN"    : 0.99,   # ← PWM 좌측 보정비
    "RIGHT_GAIN"   : 1.00,   # ← PWM 우측 보정비

    # ── Way-point & 안전 거리 ─────────────────
    "WAY_RADIUS"   : 0.30,
    "LOOK_AHEAD"   : 0.20,
    "DIST_STOP_CM" : 15,
    "DIST_GO_CM"   : 20,

    # ── 오도메트리 계수 ───────────────────────
    "LIN_PER_PWM"  : 0.002,
    "ANG_PER_DIFF" : 0.03,
    "ODOM_SCALE"   : 1.7
}

# ─────────── Dead-reckon
class DeadReckon:
    def __init__(self):
        self.x = self.y = self.yaw = 0.0
    def update(self, dt, l_pwm, r_pwm):
        v = (l_pwm + r_pwm) * 0.5 * params["LIN_PER_PWM"] * params["ODOM_SCALE"]
        w = (r_pwm - l_pwm) * params["ANG_PER_DIFF"]
        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        self.yaw  = (self.yaw + math.pi) % (2*math.pi) - math.pi

# ─────────── 초기화
tfm  = MapTransformer(YAML_PATH)
car  = YB_Pcb_Car()
odom = DeadReckon()
dist = DistanceController(trig=16, echo=18)

map_img = cv2.imread(PGM_PATH, cv2.IMREAD_GRAYSCALE)
map_bgr = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
queue   = deque()

l_cmd = r_cmd = 0
running, prev_t = False, time.time()

# ─────────── 파라미터 편집
def edit_params():
    print("\n── 파라미터 수정 ──")
    for i, k in enumerate(params, 1):
        print(f"{i:2d}. {k:<12}= {params[k]}")
    try:
        sel = int(input("번호(0=취소): "))
        if 1 <= sel <= len(params):
            key = list(params)[sel-1]
            val = float(input(f"{key} 새 값: "))
            params[key] = int(val) if key.endswith("_CM") else val
            print(f"[OK] {key} → {params[key]}")
    except Exception as e:
        print(f"[ERR] {e}")
    print("──────────────────\n")

# ─────────── 마우스 클릭 → Way-point
def on_click(evt, u, v, *_):
    if evt == cv2.EVENT_LBUTTONDOWN:
        x, y = tfm.px_to_world(u, v)
        queue.append({"x": x, "y": y})
        print(f"[CLICK] ({u},{v}) → ({x:.2f},{y:.2f})  큐{len(queue)}")

cv2.namedWindow("MAP")
cv2.setMouseCallback("MAP", on_click)
print("[INFO] 지도 클릭 (ESC=종료, p=파라미터 수정)")

# ─────────── 메인 루프
try:
    while True:
        # ① 오도메트리
        now, dt = time.time(), time.time() - prev_t
        prev_t  = now
        odom.update(dt, l_cmd, r_cmd)

        # ② 초음파
        d_cm = dist.get_distance()
        if d_cm != -1:
            running = d_cm > params["DIST_GO_CM"] or (running and d_cm > params["DIST_STOP_CM"])

        # ③ Way-point 추종
        if queue and running:
            wp = queue[0]
            dx, dy = wp["x"]-odom.x, wp["y"]-odom.y
            d_wp   = math.hypot(dx, dy)
            ang    = math.atan2(dy, dx)
            yaw_e  = (ang - odom.yaw + math.pi) % (2*math.pi) - math.pi

            base = params["BASE_SPEED"] * (0.4 if d_wp < params["LOOK_AHEAD"] else 1.0)
            base += params["LEFT_TRIM"] if base > 0 else params["RIGHT_TRIM"]

            steer = params["STEER_GAIN"] * math.degrees(yaw_e)
            l_raw = base - steer
            r_raw = base + steer

            l_cmd = int(max(0, min(100, l_raw * params["LEFT_GAIN"])))
            r_cmd = int(max(0, min(100, r_raw * params["RIGHT_GAIN"])))
            car.Car_Run(l_cmd, r_cmd)

            if d_wp < params["WAY_RADIUS"]:
                print(f"[ARRIVE] ({wp['x']:.2f},{wp['y']:.2f})")
                queue.popleft()
        else:
            l_cmd = r_cmd = 0
            car.Car_Stop()

        # ④ 시각화
        vis = map_bgr.copy()
        u, v = tfm.world_to_px(odom.x, odom.y)
        cv2.circle(vis, (u, v), 3, (0, 0, 255), -1)
        for w in queue:
            uu, vv = tfm.world_to_px(w["x"], w["y"])
            cv2.circle(vis, (uu, vv), 4, (255, 0, 0), 2)
        txt = f"D:{d_cm:.0f}cm" if d_cm != -1 else "D:--"
        cv2.putText(vis, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("MAP", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord('p'):
            car.Car_Stop()
            edit_params()

        time.sleep(0.03)

finally:
    print("[INFO] 종료: 차량 정지 및 자원 해제")
    car.Car_Stop()
    dist.cleanup()
    cv2.destroyAllWindows()
