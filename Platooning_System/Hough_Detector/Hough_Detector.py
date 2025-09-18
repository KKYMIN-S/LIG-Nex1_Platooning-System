import cv2
import numpy as np

class HoughDetector:
    def __init__(self, dp=1.2, minDist=20, param1=50, param2=20, minRadius=5, maxRadius=30):
        """
        HoughCircles 기반 원 검출 모듈
        dp: 해상도 축소 비율
        minDist: 원 간 최소 거리
        param1: Canny 엣지 상한값
        param2: 원 검출 임계값
        minRadius, maxRadius: 검출할 반지름 범위
        """
        self.dp = dp
        self.minDist = minDist
        self.param1 = param1
        self.param2 = param2
        self.minRadius = minRadius
        self.maxRadius = maxRadius

    def detect(self, gray_roi):
        """
        ROI 내에서 HoughCircles로 원을 검출하여
        (x, y, r) 리스트를 반환
        gray_roi: 회색조 이미지
        """
        circles = cv2.HoughCircles(
            gray_roi,
            cv2.HOUGH_GRADIENT,
            dp=self.dp,
            minDist=self.minDist,
            param1=self.param1,
            param2=self.param2,
            minRadius=self.minRadius,
            maxRadius=self.maxRadius
        )
        if circles is None:
            return []
        # 정수형으로 변환
        return np.uint16(np.around(circles[0]))
