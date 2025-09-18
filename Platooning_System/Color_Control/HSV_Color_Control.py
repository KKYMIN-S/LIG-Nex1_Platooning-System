#!/usr/bin/env python
# coding: utf-8
import cv2
import numpy as np

# 미리 정의된 색상별 기본 HSV 범위
color_hsv = {
    "red":    ((0,  70,  72), (7, 255, 255)),
    "green":  ((54,109,  78), (77,255,255)),
    "blue":   ((92,100,  62), (121,251,255)),
    "yellow": ((26,100,  91), (32,255,255))
}
color_names = list(color_hsv.keys())

def nothing(x):
    pass

# 카메라 열기
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# 트랙바 윈도우 생성
cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)

# 색상 선택용 슬라이더 (0..len-1)
cv2.createTrackbar('Color', 'HSV Tuner', 0, len(color_names)-1, nothing)
# H/S/V min/max 슬라이더
for name, init, mn, mx in [
    ('H Min', 0,   0,   179),
    ('H Max', 179, 0,   179),
    ('S Min', 0,   0,   255),
    ('S Max', 255, 0,   255),
    ('V Min', 0,   0,   255),
    ('V Max', 255, 0,   255),
]:
    cv2.createTrackbar(name, 'HSV Tuner', init, mx, nothing)

# 초기값을 미리 정의된 color_hsv['blue']로 설정
initial = 'blue'
ci = color_names.index(initial)
cv2.setTrackbarPos('Color', 'HSV Tuner', ci)
hmin,smin,vmin = color_hsv[initial][0]
hmax,smax,vmax = color_hsv[initial][1]
cv2.setTrackbarPos('H Min','HSV Tuner',hmin)
cv2.setTrackbarPos('S Min','HSV Tuner',smin)
cv2.setTrackbarPos('V Min','HSV Tuner',vmin)
cv2.setTrackbarPos('H Max','HSV Tuner',hmax)
cv2.setTrackbarPos('S Max','HSV Tuner',smax)
cv2.setTrackbarPos('V Max','HSV Tuner',vmax)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 트랙바 값 읽기
    ci   = cv2.getTrackbarPos('Color','HSV Tuner')
    color_name = color_names[ci]
    h_min = cv2.getTrackbarPos('H Min','HSV Tuner')
    s_min = cv2.getTrackbarPos('S Min','HSV Tuner')
    v_min = cv2.getTrackbarPos('V Min','HSV Tuner')
    h_max = cv2.getTrackbarPos('H Max','HSV Tuner')
    s_max = cv2.getTrackbarPos('S Max','HSV Tuner')
    v_max = cv2.getTrackbarPos('V Max','HSV Tuner')

    # 마스크 생성
    lower = np.array((h_min,s_min,v_min))
    upper = np.array((h_max,s_max,v_max))
    hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask  = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # 색상 이름 오버레이
    cv2.putText(frame, f"Color: {color_name}", (10,20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # 화면 배치
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    # 종료: q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
