import os
import requests
import paho.mqtt.client as mqtt
import json
import uuid
from datetime import datetime
import rospy
from std_msgs.msg import String

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
            rospy.loginfo("Connected successfully")
        else:
            rospy.logerr(f"Connection failed with code {rc}")

    def connect(self):
        self.client.connect(self.ip, self.port, 60)

    def publish_message(self, message):
        try:
            payload = json.dumps(message)
            self.client.publish(self.topic, payload)
            rospy.loginfo("Message published successfully")
        except Exception as e:
            rospy.logerr(f"Failed to publish message: {e}")

    def upload_image(self, image_path, event_id):
        headers = {
            self.header_key: self.header_value
        }

        files = {
            "images": (f"{event_id}.png", open(image_path, "rb"), "image/png")
        }

        try:
            rospy.loginfo(f"Uploading file: {image_path} → {self.api_url}")
            response = requests.post(self.api_url, headers=headers, files=files)
            rospy.loginfo(f"Response status code: {response.status_code}")

            if response.status_code == 200:
                return {"status": "success", "response": response.text}
            else:
                return {"status": "failure", "code": response.status_code, "message": response.text}
        except Exception as e:
            return {"status": "error", "message": str(e)}

# Global variables to hold the latest data
latest_location = {"x": 0, "y": 0}
latest_detection = {"classified": "unknown", "score": 0.0}

# ROS callback for location updates
def location_callback(msg):
    global latest_location
    try:
        latest_location = json.loads(msg.data)
        rospy.loginfo(f"Updated location: {latest_location}")
    except Exception as e:
        rospy.logerr(f"Failed to parse location data: {e}")

# ROS callback for YOLO detection results
def detection_result_callback(msg):
    global latest_detection
    try:
        latest_detection = json.loads(msg.data)
        rospy.loginfo(f"Updated detection data: {latest_detection}")
    except Exception as e:
        rospy.logerr(f"Failed to parse detection data: {e}")

# ROS callback for YOLO detection image trigger
def yolo_detect_callback(msg):
    global latest_location, latest_detection
    uploader = rospy.get_param("/mqtt_uploader_instance")

    transaction_id = str(uuid.uuid4())
    message_id = str(uuid.uuid4())
    event_id = str(uuid.uuid4())
    current_timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

    # Fetch track_id from environment variable
    track_id = os.getenv("TRACK_ID", "unknown")
    robotId = os.getenv("ROBOT_ID", "unknown")

    message = {
        "transactionId": transaction_id,
        "messageId": message_id,
        "robotId": robotId,
        "timestamp": current_timestamp,
        "events": [{
            "eventID": event_id,
            "eventType": "SOC",
            "timestamp": current_timestamp,
            "location": latest_location,
            "eventContent": {
                "track_id": track_id,
                "classified": latest_detection.get("classified", "unknown"),
                "score": latest_detection.get("score", 0.0)
            }
        }]
    }

    rospy.loginfo(f"Publishing MQTT message: {message}")
    uploader.publish_message(message)

    # Assuming the image is saved with a standard name for YOLO detection
    image_path = "/tmp/yolo_detected_image.png"
    result = uploader.upload_image(image_path, event_id)
    if result["status"] == "success":
        rospy.loginfo(f"[SUCCESS] Uploaded image for Event ID {event_id}")
    else:
        rospy.logerr(f"[FAILURE] Image upload failed for Event ID {event_id}: {result.get('message')}")

if __name__ == "__main__":
    rospy.init_node("mqtt_uploader_node")
    uploader = MQTTUploader()

    rospy.set_param("/mqtt_uploader_instance", uploader)
    uploader.connect()
    uploader.client.loop_start()

    rospy.Subscriber("/yolo_detect_img", String, yolo_detect_callback)
    rospy.Subscriber("/estimation_result", String, location_callback)
    rospy.Subscriber("/yolo_detect_result", String, detection_result_callback)

    rospy.loginfo("MQTTUploader ROS Node Running...")
    rospy.spin()

    uploader.client.loop_stop()
