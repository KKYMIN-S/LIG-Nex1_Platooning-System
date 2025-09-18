# DistanceSensor.py
import RPi.GPIO as GPIO
import time

# ───── 칼만 필터 클래스 정의 ─────
class SimpleKalmanFilter:
    def __init__(self, process_noise=4.0, measurement_noise=1.0, estimate_error=10.0):
        self.q = process_noise      # 예측(프로세스) 노이즈
        self.r = measurement_noise # 측정 노이즈
        self.p = estimate_error     # 초기 추정 오차
        self.x = 0.0                # 초기 추정값

    def update(self, measurement):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

# ───── 초음파 거리 센서 클래스 ─────
class DistanceController:
    def __init__(self, trig=16, echo=18, timeout=0.05):
        self.TrigPin = trig
        self.EchoPin = echo
        self.timeout = timeout  # 센서 타임아웃 기본값 50ms

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.TrigPin, GPIO.OUT)
        GPIO.setup(self.EchoPin, GPIO.IN)

        # 칼만 필터 인스턴스 생성 (성능 향상용 파라미터)
        self.kalman = SimpleKalmanFilter(process_noise=4.0, measurement_noise=1.0, estimate_error=10.0)

        # 초기 측정 평균값으로 칼만 필터 초기화
        init = self.get_average_distance(count=5, delay=0.02)
        if init > 0:
            self.kalman.x = init

    def get_distance(self):
        GPIO.output(self.TrigPin, GPIO.LOW)
        time.sleep(0.00001)
        GPIO.output(self.TrigPin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.TrigPin, GPIO.LOW)

        t_start = time.time()
        while not GPIO.input(self.EchoPin):
            if time.time() - t_start > self.timeout:
                return -1
        t1 = time.time()

        while GPIO.input(self.EchoPin):
            if time.time() - t1 > self.timeout:
                return -1
        t2 = time.time()

        distance = ((t2 - t1) * 340 / 2) * 100  # cm
        return distance

    def get_average_distance(self, count=3, delay=0.01):
        distances = []
        for _ in range(count):
            d = self.get_distance()
            if d > 0:
                distances.append(d)
            time.sleep(delay)

        if not distances:
            return -1
        return sum(distances) / len(distances)

    def get_kalman_distance(self):
        raw = self.get_distance()
        if raw <= 0:
            return self.kalman.x if self.kalman.x > 0 else -1
        return self.kalman.update(raw)

    def cleanup(self):
        GPIO.cleanup()
