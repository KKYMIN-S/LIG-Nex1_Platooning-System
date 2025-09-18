# Ultrasonic_IR_Avoid.py
"""
avoid.py
초음파 및 IR 센서를 이용한 거리 측정 및 장애물 회피 기능을 제공하는 모듈
- Distance(): 단일 초음파 거리 측정 (cm) 또는 -1
- Distance_test(): 여러 측정 후 평균 거리 반환
- is_obstacle(): 초음파+IR 센서로 장애물 여부 판단
- avoid(): 장애물 감지 시 회피 동작 수행
- cleanup(): GPIO 및 모터 정리
"""
import RPi.GPIO as GPIO
import time
from YB_Pcb_Car import YB_Pcb_Car

# 모터/서보 컨트롤 객체 생성
car = YB_Pcb_Car()

# GPIO 설정 (BOARD 모드)
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# IR 센서 핀 정의
IR_LEFT_PIN  = 21
IR_RIGHT_PIN = 19
# 초음파 센서 핀 정의
TRIG_PIN = 16
ECHO_PIN = 18

gpio_pins = [IR_LEFT_PIN, IR_RIGHT_PIN, TRIG_PIN, ECHO_PIN]
GPIO.setup(IR_LEFT_PIN,  GPIO.IN)
GPIO.setup(IR_RIGHT_PIN, GPIO.IN)
GPIO.setup(TRIG_PIN,     GPIO.OUT)
GPIO.setup(ECHO_PIN,     GPIO.IN)

# 상수
SPEED_OF_SOUND = 340.0  # m/s
TIMEOUT = 0.03          # seconds
MIN_DIST_CM = 10.0      # cm


def Distance():
    """단일 초음파 거리 측정 (cm). 실패 시 -1 반환"""
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    start = time.time()
    while not GPIO.input(ECHO_PIN):
        if time.time() - start > TIMEOUT:
            return -1
    t1 = time.time()
    while GPIO.input(ECHO_PIN):
        if time.time() - t1 > TIMEOUT:
            return -1
    t2 = time.time()
    return ((t2 - t1) * SPEED_OF_SOUND / 2) * 100


def Distance_test(samples=3):
    """유효값(samples횟수)만 평균 내어 반환"""
    vals = []
    while len(vals) < samples:
        d = Distance()
        if 0 < d < 500:
            vals.append(d)
    return sum(vals) / len(vals)


# def is_obstacle():
#    """초음파 거리와 IR 센서로 장애물 여부 판단"""
#    d = Distance_test()
#    left  = not GPIO.input(IR_LEFT_PIN)
#    right = not GPIO.input(IR_RIGHT_PIN)
#    return (0 < d < MIN_DIST_CM) or left or right 

# IR 센서를 통해서 물체 회피 함수로 교체
def is_ir_obstacle():
    """
    IR 센서가 LOW일 때만 장애물로 판단
    (왼쪽/오른쪽 IR 둘 중 하나라도)
    """
    left  = not GPIO.input(IR_LEFT_PIN)
    right = not GPIO.input(IR_RIGHT_PIN)
    return left or right



def avoid():
    """장애물 감지 시 회피 동작 수행"""
    dist     = Distance_test()
    ir_left  = not GPIO.input(IR_LEFT_PIN)
    ir_right = not GPIO.input(IR_RIGHT_PIN)

    if 0 < dist < MIN_DIST_CM:
        car.Car_Stop(); time.sleep(0.1)
        if ir_left and ir_right:
            car.Car_Spin_Right(80,80); time.sleep(0.5)
        elif ir_left:
            car.Car_Spin_Right(80,80); time.sleep(0.7)
        elif ir_right:
            car.Car_Spin_Left(80,80); time.sleep(0.7)
        else:
            car.Car_Back(100,100); time.sleep(0.7)
    else:
        car.Car_Run(100,100)


def cleanup():
    """모터 정지 및 GPIO 해제"""
    car.Car_Stop()
    GPIO.cleanup()
