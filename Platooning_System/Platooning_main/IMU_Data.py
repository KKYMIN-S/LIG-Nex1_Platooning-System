import time
import math
import threading
from mpu6050.mpu6050 import mpu6050
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class IMUDataCollector:
    def __init__(self, address=0x68, drift_correction=0.35, correction_interval=1.0):
        self.sensor = mpu6050(address)
        self.heading = 0.0
        self.heading_data = []
        self.time_data = []
        self.prev_time = time.time()
        self.last_log_time = self.prev_time
        self.drift_correction = drift_correction
        self.correction_interval = correction_interval
        self.running = False

    def collect_data(self):
        self.running = True
        while self.running:
            now = time.time()
            dt = now - self.prev_time
            self.prev_time = now

            gyro = self.sensor.get_gyro_data()
            GyZ = gyro['z']

            # heading 누적
            self.heading += GyZ * dt

            # 드리프트 보정
            if now - self.last_log_time >= self.correction_interval:
                self.heading -= self.drift_correction
                print(f"[{time.strftime('%H:%M:%S')}] {self.correction_interval:.0f}초 경과 - heading 보정 후: {self.heading:.2f}도")
                self.last_log_time = now

            # 데이터 저장
            self.time_data.append(now)
            self.heading_data.append(self.heading)

            # 리스트 길이 제한
            if len(self.time_data) > 200:
                self.time_data.pop(0)
                self.heading_data.pop(0)

            time.sleep(0.01)

    def start_collecting(self):
        threading.Thread(target=self.collect_data, daemon=True).start()

    def update_plot(self, frame):
        plt.cla()
        plt.plot(self.time_data, self.heading_data, label='Heading (Yaw)')
        plt.xlabel('Time')
        plt.ylabel('Angle (degrees)')
        plt.title('IMU Yaw (Heading) Only')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

    def start_plot(self):
        self.start_collecting()
        ani = FuncAnimation(plt.gcf(), self.update_plot, interval=100)
        plt.show()

# 실행부
if __name__ == "__main__":
    imu_collector = IMUDataCollector()
    imu_collector.start_plot()
