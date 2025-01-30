import os
import requests
import paho.mqtt.client as mqtt
import json
import uuid
from datetime import datetime

class MQTTUploader:
    def __init__(self,
                 ip="220.90.239.142",
                 port=1883,
                 username="robot",
                 password="robot123",
                 topic="mrm/geon-1/eventAI/result",
                 api_url="http://220.90.239.142:7080/col/v1/image/saveImageFiles.do",
                 header_key="X-GEO-ROADAI-API",
                 header_value="Geon ce79e749650fb0c8595801d94c222bbc"):
        self.ip = ip
        self.port = port
        self.username = username
        self.password = password
        self.topic = topic
        self.api_url = api_url
        self.header_key = header_key
        self.header_value = header_value

        self.client = mqtt.Client()
        self.client.username_pw_set(username, password)
        self.client.on_connect = self.on_connect

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected successfully")
        else:
            print(f"Connection failed with code {rc}")

    def connect(self):
        self.client.connect(self.ip, self.port, 60)

    def publish_message(self, message):
        try:
            payload = json.dumps(message)
            self.client.publish(self.topic, payload)
            print("Message published successfully")
        except Exception as e:
            print(f"Failed to publish message: {e}")

    def upload_image(self, image_path, event_id):
        headers = {
            self.header_key: self.header_value
        }

        files = {
            "images": (f"{event_id}.png", open(image_path, "rb"), "image/png")
        }

        try:
            print(f"Uploading file: {image_path} → {self.api_url}")
            print(f"Event ID: {event_id}")
            response = requests.post(self.api_url, headers=headers, files=files)
            print(f"Response status code: {response.status_code}")
            print(f"Response content: {response.text}")

            if response.status_code == 200:
                return {"status": "success", "response": response.text}
            else:
                return {"status": "failure", "code": response.status_code, "message": response.text}
        except Exception as e:
            return {"status": "error", "message": str(e)}

# Example Usage
if __name__ == "__main__":
    uploader = MQTTUploader()

    # Example message
    transaction_id = str(uuid.uuid4())
    message_id = str(uuid.uuid4())
    event_id = str(uuid.uuid4())

    current_timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

    message = {
        "transactionId": transaction_id,
        "messageId": message_id,
        "robotId": "geon-1",
        "timestamp": current_timestamp,
        "events": [{
            "eventID": event_id,
            "eventType": "SOC",
            "timestamp": current_timestamp,
            "location": {
                "x": 126.975623,
                "y": 37.713742
            },
            "eventContent": {
                "track_id": "8109",
                "classified": "101",
                "score": "0.29"
            },
        }]
    }

    # Image path
    image_path = "./data/2024-11-19_09-31-18_frame_0509_deep_portholl.png"

    # MQTT message publishing
    uploader.connect()
    uploader.client.loop_start()
    uploader.publish_message(message)
    uploader.client.loop_stop()

    # Image upload
    result = uploader.upload_image(image_path, event_id)
    if result["status"] == "success":
        print(f"[SUCCESS] Response: {result['response']}")
    else:
        print(f"[FAILURE] Status Code: {result.get('code')} | Message: {result.get('message')}")
