# DistanceSensor.py
import RPi.GPIO as GPIO
import time

class DistanceController:
    def __init__(self, trig=16, echo=18):
        self.TrigPin = trig
        self.EchoPin = echo

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.TrigPin, GPIO.OUT)
        GPIO.setup(self.EchoPin, GPIO.IN)

    def get_distance(self):
        GPIO.output(self.TrigPin, GPIO.LOW)
        time.sleep(0.00001)
        GPIO.output(self.TrigPin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.TrigPin, GPIO.LOW)

        t_start = time.time()
        while not GPIO.input(self.EchoPin):
            if time.time() - t_start > 0.03:
                return -1
        t1 = time.time()
        while GPIO.input(self.EchoPin):
            if time.time() - t1 > 0.03:
                return -1
        t2 = time.time()

        distance = ((t2 - t1) * 340 / 2) * 100  # cm
        return distance

    def cleanup(self):
        GPIO.cleanup()
