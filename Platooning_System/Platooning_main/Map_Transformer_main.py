#!/usr/bin/env python3
# coding: utf-8

import cv2, math, time, numpy as np
from collections import deque
from Map_Transformer import MapTransformer
from YB_Pcb_Car import YB_Pcb_Car
from DistanceSensor import DistanceController

# ─────── 설정
MAP_DIR   = "/home/pi/Yahboom_project/Raspbot/raspbot/Slam_Data"
YAML_PATH = f"{MAP_DIR}/3F_lab.yaml"
PGM_PATH  = f"{MAP_DIR}/3F_lab.pgm"
SCALE     = 2

params = {
    "BASE_SPEED"   : 50,
    "LEFT_TRIM"    : 0,
    "RIGHT_TRIM"   : 5,
    "STEER_GAIN"   : 0.3,
    "LEFT_GAIN"    : 1.00,
    "RIGHT_GAIN"   : 1.02,
    "WAY_RADIUS"   : 0.1,
    "LOOK_AHEAD"   : 0.20,
    "DIST_STOP_CM" : 15,
    "DIST_GO_CM"   : 20,
    "LIN_PER_PWM"  : 0.0032,
    "ANG_PER_DIFF" : 0.05,
    "ODOM_SCALE"   : 1.0
}

class DeadReckon:
    def __init__(self):
        self.x = self.y = self.yaw = 0.0
    def update(self, dt, l_pwm, r_pwm):
        v = (l_pwm + r_pwm) * 0.5 * params["LIN_PER_PWM"] * params["ODOM_SCALE"]
        w = (l_pwm - r_pwm) * params["ANG_PER_DIFF"]  # ← 방향 반전
        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        self.yaw  = (self.yaw + math.pi) % (2*math.pi) - math.pi

car  = YB_Pcb_Car()
dist = DistanceController(trig=16, echo=18)
tfm  = MapTransformer(YAML_PATH)
odom = DeadReckon()

map_img = cv2.imread(PGM_PATH, cv2.IMREAD_GRAYSCALE)
map_bgr = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
queue   = deque()
mouse_pos = None

l_cmd = r_cmd = 0
running, prev_t = False, time.time()

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

def on_click(evt, mx, my, flags, param):
    global mouse_pos
    u, v = int(mx / SCALE), int(my / SCALE)
    mouse_pos = (u, v)
    if evt == cv2.EVENT_LBUTTONDOWN:
        x, y = tfm.px_to_world(u, v)
        dx, dy = x - odom.x, y - odom.y  # 차량 위치 기준으로 쎄타 계산
        dist_wp = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if dist_wp > 0.2 else None
        queue.append({"x": x, "y": y, "yaw": yaw})
        print(f"[CLICK] → World({x:.2f}, {y:.2f}, yaw={math.degrees(yaw) if yaw else 'None'})  Queue{len(queue)}")

cv2.namedWindow("MAP")
cv2.setMouseCallback("MAP", on_click)
print("[INFO] 지도 클릭 (ESC=종료, p=파라미터 수정)")

try:
    while True:
        now, dt = time.time(), time.time() - prev_t
        prev_t  = now
        odom.update(dt, l_cmd, r_cmd)

        d_cm = dist.get_distance()
        if d_cm != -1:
            running = d_cm > params["DIST_GO_CM"] or (running and d_cm > params["DIST_STOP_CM"])

        if queue and running:
            wp = queue[0]
            dx, dy = wp["x"] - odom.x, wp["y"] - odom.y
            d_wp   = math.hypot(dx, dy)
            ang    = math.atan2(dy, dx)
            yaw_e  = (ang - odom.yaw + math.pi) % (2*math.pi) - math.pi

            if d_wp < params["WAY_RADIUS"]:
                desired_yaw = wp.get("yaw")
                if desired_yaw is not None:
                    yaw_err = (desired_yaw - odom.yaw + math.pi) % (2*math.pi) - math.pi
                    if abs(math.degrees(yaw_err)) > 5:
                        steer = params["STEER_GAIN"] * math.degrees(yaw_err)
                        l_raw = steer
                        r_raw = -steer
                        l_cmd = int(max(0, min(100, l_raw + params["LEFT_TRIM"])))
                        r_cmd = int(max(0, min(100, r_raw + params["RIGHT_TRIM"])))
                        print(f"[ALIGN] l:{l_cmd}, r:{r_cmd}, yaw_err:{math.degrees(yaw_err):.2f}°")
                        car.Car_Run(l_cmd, r_cmd)
                        continue
                print(f"[ARRIVE] ({wp['x']:.2f}, {wp['y']:.2f})")
                queue.popleft()
                car.Car_Stop()
                l_cmd = r_cmd = 0
                continue

            base = params["BASE_SPEED"] * (0.4 if d_wp < params["LOOK_AHEAD"] else 1.0)
            base += params["LEFT_TRIM"] if base > 0 else params["RIGHT_TRIM"]

            if abs(math.degrees(yaw_e)) > 20:
                l_raw = -params["BASE_SPEED"]
                r_raw =  params["BASE_SPEED"]
            else:
                steer = params["STEER_GAIN"] * math.degrees(yaw_e)
                l_raw = base - steer
                r_raw = base + steer

            l_cmd = int(max(0, min(100, l_raw * params["LEFT_GAIN"])))
            r_cmd = int(max(0, min(100, r_raw * params["RIGHT_GAIN"])))
            car.Car_Run(l_cmd, r_cmd)
        else:
            l_cmd = r_cmd = 0
            car.Car_Stop()

        vis = cv2.resize(map_bgr.copy(), (0, 0), fx=SCALE, fy=SCALE)

        grid_spacing_m = 0.5
        grid_spacing_px = int(grid_spacing_m / tfm.res * SCALE)
        H, W = vis.shape[:2]
        for x in range(0, W, grid_spacing_px):
            cv2.line(vis, (x, 0), (x, H), (60, 60, 60), 1)
        for y in range(0, H, grid_spacing_px):
            cv2.line(vis, (0, y), (W, y), (60, 60, 60), 1)

        u, v = tfm.world_to_px(odom.x, odom.y)
        u2, v2 = int(u * SCALE), int(v * SCALE)
        cv2.circle(vis, (u2, v2), 3, (0, 0, 255), -1)
        coord_txt = f"({odom.x:.2f}, {odom.y:.2f})"
        cv2.putText(vis, coord_txt, (u2 + 10, v2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        if mouse_pos is not None:
            mx, my = mouse_pos
            mx_w, my_w = tfm.px_to_world(mx, my)
            mx2, my2 = int(mx * SCALE), int(my * SCALE)
            msg = f"({mx_w:.2f}, {my_w:.2f})"
            cv2.putText(vis, msg, (mx2 + 10, my2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)
            cv2.circle(vis, (mx2, my2), 4, (255, 100, 100), 2)

        for i, w in enumerate(queue):
            uu, vv = tfm.world_to_px(w["x"], w["y"])
            uu2, vv2 = int(uu * SCALE), int(vv * SCALE)
            cv2.circle(vis, (uu2, vv2), 4, (255, 0, 0), 2)
            cv2.putText(vis, str(i+1), (uu2 + 6, vv2 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        scale_len_m = 1.0
        scale_px = int(scale_len_m / tfm.res * SCALE)
        bar_x, bar_y = 50, vis.shape[0] - 30
        cv2.line(vis, (bar_x, bar_y), (bar_x + scale_px, bar_y), (255, 255, 255), 2)
        cv2.putText(vis, "1 m", (bar_x, bar_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if queue:
            wp = queue[0]
            dist_m = math.hypot(wp["x"] - odom.x, wp["y"] - odom.y)
            cv2.putText(vis, f"Dist to WP: {dist_m:.2f} m", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        txt = f"D:{d_cm:.0f}cm" if d_cm != -1 else "D:--"
        cv2.putText(vis, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

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
