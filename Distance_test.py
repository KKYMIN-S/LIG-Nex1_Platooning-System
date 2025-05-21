import serial
import time
import threading
import re

# === 시리얼 포트 설정 ===
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
exit_flag = False

def listen_keyboard():
    global exit_flag
    while True:
        key = input()
        if key.lower() == 'q':
            exit_flag = True
            break

def read_distance():
    global exit_flag
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # 아두이노 리셋 기다림

        print("거리(cm) 수신 시작... (q를 누르면 종료)")

        while not exit_flag:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                # "xx cm" 형식만 필터링
                match = re.search(r'([0-9.]+)\s*cm', line)
                if match:
                    distance = float(match.group(1))
                    print(f"{distance:.1f} cm")  # 소수점 1자리 출력

        print("종료 명령(q) 감지됨. 루프 탈출.")

    except serial.SerialException as e:
        print(f"시리얼 통신 오류: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    input_thread = threading.Thread(target=listen_keyboard, daemon=True)
    input_thread.start()
    read_distance()
