import paho.mqtt.client as mqtt
import json

class MqttHandler:
    def __init__(self, broker_ip, port=1883, default_topic="leader/motion", on_msg_callback=None, client_id="mqtt_subscriber"):
        self.default_topic = default_topic
        self.client = mqtt.Client(client_id=client_id)  # ✅ client_id 매개변수 추가
        if on_msg_callback:
            self.client.on_message = on_msg_callback
        self.client.connect(broker_ip, port, 60)

    def start_subscriber(self):
        self.client.subscribe(self.default_topic)
        self.client.loop_start()

    def subscribe(self, topic):
        self.client.subscribe(topic)

    def add_callback(self, topic, callback):
        self.client.message_callback_add(topic, callback)

    def publish(self, data: dict, topic=None):
        payload = json.dumps(data)
        target_topic = topic if topic else self.default_topic
        self.client.publish(target_topic, payload)
