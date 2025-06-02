#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json

# 브로커 설정
BROKER_ADDRESS = "192.168.0.51"
BROKER_PORT = 1883
TOPIC = "ros2/start_and_goal_pose"

# 메시지 수신 콜백 함수
def on_message(client, userdata, message):
    try:
        payload = message.payload.decode()
        data = json.loads(payload)

        # start_pose만 출력
        if 'start_pose' in data:
            start_pose = data['start_pose']
            print("[Received Start Pose]")
            print(f"  x: {start_pose.get('x')}")
            print(f"  y: {start_pose.get('y')}")
            print(f"  z: {start_pose.get('z')}")
            print(f"  yaw: {start_pose.get('yaw')}")
        else:
            print("[MQTT] Received message does not contain start_pose.")

    except Exception as e:
        print(f"[MQTT] Error parsing message: {e}")

def main():
    client = mqtt.Client("start_pose_subscriber")
    client.on_message = on_message

    try:
        client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
        print(f"[MQTT] Connected to broker at {BROKER_ADDRESS}:{BROKER_PORT}")
    except Exception as e:
        print(f"[MQTT] Connection failed: {e}")
        return

    client.subscribe(TOPIC)
    print(f"[MQTT] Subscribed to topic '{TOPIC}'")

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("[MQTT] Subscriber shutting down...")
    finally:
        client.disconnect()

if __name__ == "__main__":
    main()
