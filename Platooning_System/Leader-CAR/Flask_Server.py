from flask import Flask, send_from_directory
import os

app = Flask(__name__)

# 공유할 맵 파일이 있는 디렉토리
MAP_DIR = '/home/pi/Yahboom_project/Raspbot/raspbot/Map_Data'  # 실제 경로로 수정

@app.route('/map/<filename>')
def get_map_file(filename):
    return send_from_directory(MAP_DIR, filename)

if __name__ == '__main__':
    print(f"[리드카] 맵 서버 실행 중: http://0.0.0.0:5001/map/...")
    app.run(host='0.0.0.0', port=5001)
