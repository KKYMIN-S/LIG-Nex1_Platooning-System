#!/usr/bin/env python3
# coding: utf-8
"""
지도 창을 클릭해 목적지를 지정하면
팔로우카가 해당 좌표까지 주행하는 네비게이터.
"""

import cv2, math, time, yaml, json
from pathlib import Path
from collections import deque
from map_transformer import MapTransformer          # 앞서 만든 변환기
from pose_estimator import DeadReckon               # 엔코더·IMU 로컬 포즈
from car_driver import Car                          # Yahboom Car 래퍼

### --- 설정 -----------------------------------------------------------------
MAP_DIR   = "/home/pi/Yahboom_project/Raspbot/raspbot/Slam_png"
YAML_PATH = f"{MAP_DIR}/3F_lab.yaml"
WAY_RADIUS  = 0.30          # 도달 반경 [m]
BASE_SPEED  = 40            # 직진 기본 속도
STEER_GAIN  = 1.0           # 비례 조향 계수
LOOK_AHEAD  = 0.20          # 직선 구간에서 멈출 거리 [m]
SLEEP_DT    = 0.03

### --- 초기화 ---------------------------------------------------------------
tfm   = MapTransformer(YAML_PATH)   # 픽셀↔월드 변환기
pose  = DeadReckon()                # 현재 위치 추정
car   = Car()
queue = deque()                     # Way-point 큐 (FIFO)

map_img  = cv2.imread(f"{MAP_DIR}/3F_lab.pgm", cv2.IMREAD_GRAYSCALE)
map_bgr  = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
H, W     = map_img.shape

def mouse_cb(event, u, v, flags, param):
    """마우스 클릭 → (x,y) 변환 후 큐에 추가"""
    if event == cv2.EVENT_LBUTTONDOWN:
        x, y = tfm.px_to_world(u, v)
        wp   = {"x":x, "y":y, "radius":WAY_RADIUS}
        queue.append(wp)
        print(f"[CLICK] 픽셀({u},{v}) → 월드({x:.2f},{y:.2f})m  큐 사이즈={len(queue)}")

cv2.namedWindow("MapClick")
cv2.setMouseCallback("MapClick", mouse_cb)

print("[INFO] 지도 클릭으로 Way-point 지정 시작")

### --- 메인 루프 ------------------------------------------------------------
try:
    while True:
        pose.update()                             # ① 내 위치 갱신
        cur = {"x":pose.x, "y":pose.y}

        # ② Way-point 처리
        if queue:
            wp = queue[0]
            dist = math.hypot(cur["x"]-wp["x"], cur["y"]-wp["y"])

            # 조향 제어 (단순 비례 제어)
            ang = math.atan2(wp["y"]-cur["y"], wp["x"]-cur["x"])
            yaw = pose.yaw                       # DeadReckon 안에 보관돼 있다고 가정
            yaw_err = (ang - yaw + math.pi) % (2*math.pi) - math.pi
            steer   = int(STEER_GAIN * math.degrees(yaw_err))

            if dist > LOOK_AHEAD:
                l_spd = max(0, min(100, BASE_SPEED - steer))
                r_spd = max(0, min(100, BASE_SPEED + steer))
                car.Car_Run(l_spd, r_spd)
            elif dist > wp["radius"]:
                # 미세 조정 속도
                slow = int(BASE_SPEED*0.5)
                l_spd = max(0, min(100, slow - steer))
                r_spd = max(0, min(100, slow + steer))
                car.Car_Run(l_spd, r_spd)
            else:
                print(f"[ARRIVE] Way-point 도달 → 정지  (남은 큐 {len(queue)-1})")
                car.Car_Stop()
                queue.popleft()

        else:
            car.Car_Stop()

        # ③ 지도 시각화
        vis = map_bgr.copy()
        # 현재 위치 점
        u,v = tfm.world_to_px(cur["x"], cur["y"])
        cv2.circle(vis, (u,v), 3, (0,0,255), -1)
        # Way-point 표시
        for wp in queue:
            uu,vv = tfm.world_to_px(wp["x"], wp["y"])
            cv2.circle(vis, (uu,vv), 4, (255,0,0), 2)
        cv2.imshow("MapClick", vis)
        if cv2.waitKey(1) == 27:  # ESC 종료
            break

        time.sleep(SLEEP_DT)

finally:
    print("[INFO] 종료: 차량 정지 및 자원 해제")
    car.Car_Stop()
    cv2.destroyAllWindows()
