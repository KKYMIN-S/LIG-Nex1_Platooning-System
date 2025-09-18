#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json

# 브로커 설정
BROKER_ADDRESS = "192.168.0.51"
BROKER_PORT = 1883
TOPIC = "ros2/start_and_goal_pose"

def on_message(client, userdata, message):
    try:
        payload = message.payload.decode()
        data = json.loads(payload)

        # goal_pose 메시지만 처리
        if 'goal_pose' in data:
            goal_pose = data['goal_pose']
            print("[Received Goal Pose]")
            print(f"  x: {goal_pose.get('x')}")
            print(f"  y: {goal_pose.get('y')}")
            print(f"  z: {goal_pose.get('z')}")
            print(f"  yaw: {goal_pose.get('yaw')}")
    except Exception as e:
        print(f"[MQTT] Error parsing message: {e}")

def main():
    client = mqtt.Client("goal_pose_subscriber")
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
