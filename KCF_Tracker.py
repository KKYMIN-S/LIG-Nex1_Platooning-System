import cv2
import numpy as np
import time

# 파란색 검출 범위 (HSV)
LOWER_BLUE = np.array([95, 117, 62])
UPPER_BLUE = np.array([115, 230, 149])

class KCFTracker:
    def __init__(self,
                 sigma=0.5,
                 lambda_val=0.0001,
                 interp_factor=0.075,
                 output_sigma_factor=0.1,
                 cell_size=4,
                 detect_thresh=0.5):
        """
        KCF 트래커의 주요 파라미터를 외부에서 조정할 수 있도록 생성자에 인자로 추가.

        - sigma: 커널 분포 폭
        - lambda_val: 정규화 상수
        - interp_factor: 온라인 업데이트 비율
        - output_sigma_factor: 목표 크기 대비 출력 분포 비율
        - cell_size: HOG 셀 크기 (픽셀)
        - detect_thresh: 초기 검출 임계값
        """
        # 카메라 초기화
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH , 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS,          30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        # 사용자 지정 가능한 KCF 파라미터 저장
        self.params = {
            'sigma': sigma,
            'lambda_': lambda_val,
            'interp_factor': interp_factor,
            'output_sigma_factor': output_sigma_factor,
            'cell_size': cell_size,
            'detect_thresh': detect_thresh
        }

        # 내부 상태 변수
        self.tracker = None
        self.tracking = False
        self.object_detected = False
        self.last_success_time = time.time()

    def create_tracker(self):
        # OpenCV 4.x 이상: legacy 모듈의 TrackerKCF_Params 사용
        if hasattr(cv2.legacy, 'TrackerKCF_Params'):
            p = cv2.legacy.TrackerKCF_Params()
            p.sigma               = self.params['sigma']
            p.lambda_             = self.params['lambda_']
            p.interp_factor       = self.params['interp_factor']
            p.output_sigma_factor = self.params['output_sigma_factor']
            p.cell_size           = self.params['cell_size']
            p.detect_thresh       = self.params['detect_thresh']
            return cv2.legacy.TrackerKCF_create(p)
        else:
            # 하위 호환: 파라미터 없이 기본 트래커 생성
            return cv2.TrackerKCF_create()

    def detect_blue_object(self, frame):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            x, y, w, h = cv2.boundingRect(max(cnts, key=cv2.contourArea))
            if w*h > 500:
                return (x, y, w, h)
        return None

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, False, None

        if not self.tracking:
            roi = self.detect_blue_object(frame)
            if roi:
                self.tracker = self.create_tracker()
                self.tracker.init(frame, roi)
                self.tracking = True
                self.object_detected = True
                self.last_success_time = time.time()
                x, y, w, h = roi
                cx, cy = x + w//2, y + h//2
                return (x, y, w, h, cx, cy), True, frame
            return None, False, frame

        success, bbox = self.tracker.update(frame)
        if success:
            x, y, w, h = map(int, bbox)
            cx, cy = x + w//2, y + h//2
            self.last_success_time = time.time()
            return (x, y, w, h, cx, cy), True, frame
        else:
            # 1초 이상 추적 실패 시 재탐지
            if time.time() - self.last_success_time > 1.0:
                self.tracking = False
                self.object_detected = False
            return None, False, frame

    def get_last_success_time(self):
        return self.last_success_time

    def release(self):
        self.cap.release()
