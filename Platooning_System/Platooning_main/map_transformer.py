#!/usr/bin/env python3
# coding: utf-8
"""
PNG/PGM 지도와 YAML 메타데이터를 이용해
픽셀 <-> 월드 좌표를 상호 변환하는 유틸리티.
"""

import math
import yaml
import cv2
from pathlib import Path

class MapTransformer:
    """픽셀·월드 좌표 변환기"""
    def __init__(self, yaml_path: str):
        # YAML 로드
        yml = yaml.safe_load(Path(yaml_path).read_text())
        self.res      = float(yml["resolution"])         # m/px
        self.origin_x = float(yml["origin"][0])
        self.origin_y = float(yml["origin"][1])
        self.theta    = float(yml["origin"][2])          # rad
        self.cos_t    = math.cos(self.theta)
        self.sin_t    = math.sin(self.theta)

        # 이미지 높이(H)만 필요
        img_path = (Path(yaml_path).parent / yml["image"]).resolve()
        self.H   = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE).shape[0]

    # -------- 변환 함수 --------
    def px_to_world(self, u: int, v: int):
        """(u,v) 픽셀 → (x,y) 월드 [m]"""
        dx =  u * self.res
        dy = (self.H - 1 - v) * self.res          # y축 뒤집기
        x  = self.origin_x +  self.cos_t*dx - self.sin_t*dy
        y  = self.origin_y +  self.sin_t*dx + self.cos_t*dy
        return x, y

    def world_to_px(self, x: float, y: float):
        """(x,y) 월드 [m] → (u,v) 픽셀"""
        dx =  self.cos_t*(x - self.origin_x) + self.sin_t*(y - self.origin_y)
        dy = -self.sin_t*(x - self.origin_x) + self.cos_t*(y - self.origin_y)
        u  = int(round(dx / self.res))
        v  = int(round((self.H - 1) - dy / self.res))
        return u, v
