import os
import requests
import paho.mqtt.client as mqtt
import json
import uuid
from datetime import datetime


def configure_mqtt(broker, port, username, password):
    """
    MQTT 클라이언트를 구성합니다.

    Args:
        broker (str): MQTT 브로커 주소.
        port (int): MQTT 포트 번호.
        username (str): MQTT 사용자 이름.
        password (str): MQTT 사용자 비밀번호.

    Returns:
        mqtt.Client: 구성된 MQTT 클라이언트.
    """
    client = mqtt.Client()
    client.username_pw_set(username, password)
    return client


def publish_message(client, topic, message):
    """
    MQTT 메시지를 발행합니다.

    Args:
        client (mqtt.Client): MQTT 클라이언트.
        topic (str): 메시지를 발행할 토픽.
        message (dict): 발행할 메시지 내용.
    """
    try:
        payload = json.dumps(message)
        client.publish(topic, payload)
        print("Message published successfully")
    except Exception as e:
        print(f"Failed to publish message: {e}")


def upload_image(api_url, image_path, event_id, header_key, header_value, box_result, timestamp, image_name):
    """
    하나의 이미지 파일을 지정된 API URL로 업로드합니다.

    Args:
        api_url (str): API URL.
        image_path (str): 업로드할 이미지 파일의 경로.
        event_id (str): MQTT 메시지에서 추출한 eventID.
        header_key (str): 헤더의 키.
        header_value (str): 헤더의 값.

    Returns:
        dict: 서버 응답 또는 오류 세부 정보.
    """
    headers = {
        header_key: header_value
    }

    # form-data에 파일 추가 (KEY: images, VALUE: 파일 데이터)
    files = {
        "images": (f"{image_name}.png", open(image_path, "rb"), "image/png"),
        
    }

    data = {
        "box_result": box_result,
        "timestamp": f"{timestamp}"
    }

    try:
        print(f"파일 업로드 중: {image_path} → {api_url}")
        print(f"Event ID: {image_name}")
        response = requests.post(api_url, headers=headers, files=files, data=data)
        print(f"응답 상태 코드: {response.status_code}")
        print(f"응답 내용: {response.text}")

        if response.status_code == 200:
            return {"status": "success", "response": response.text}
        else:
            return {"status": "failure", "code": response.status_code, "message": response.text}
    except Exception as e:
        return {"status": "error", "message": str(e)}


# 사용 예제
if __name__ == "__main__":
    # 사용자 설정값
    ip = "220.90.239.142"
    port = 1883
    username = "robot"
    password = "robot123"
    topic = "mrm/geon-2/eventAI/result"
    api_url = f"http://{ip}:7080/col/v1/image/saveImageFiles.do"
    header_key = "X-GEO-ROADAI-API"
    header_value = "Geon ce79e749650fb0c8595801d94c222bbc"
    image_path = "./data/2024-12-13_09-12-21_frame_0785_resized.png"  # 업로드할 이미지 파일 경로
    file_name_with_extension = image_path.split('/')[-1]
    image_name = file_name_with_extension.split('.')[0]
    transactionId = uuid.uuid4()
    messageId = uuid.uuid4()
    eventId = uuid.uuid4()
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    box_result = "4,0.81,1082,831,1171,852"

    print(f"transactionId : {transactionId}, \n messageId : {messageId}, \n evendId : {eventId}, \n now : {timestamp}")
    # MQTT 메시지 내용
    message = {
        "transactionId": f"{transactionId}",
        "messageId": f"{messageId}",
        "robotId": "geon-2",
        "timestamp": "20250114142058",
        "events": [{
            "eventID": f"{eventId}",
            "eventType": "SOC",
            "imageID": "2024-12-13_09-12-21_frame_0785_resized.png",
            "timestamp": timestamp,
            "location": {
                "x": 126.815175,
                "y": 37.624831
            },
            "eventContent": {
                "track_id": "9707",
                "classified": "4",
                "score": "0.81"
            },
        }]
    }

    # MQTT 클라이언트 구성
    client = configure_mqtt(ip, port, username, password)


    # MQTT 연결 콜백 함수
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected successfully")
        else:
            print(f"Connection failed with code {rc}")


    client.on_connect = on_connect

    # MQTT 연결 및 메시지 발행
    client.connect(ip, port, 60)
    client.loop_start()
    publish_message(client, topic, message)
    client.loop_stop()

    # MQTT 메시지에서 eventID를 추출
    event_id = message["events"][0]["eventID"]
    print(f"Extracted Event ID: {event_id}")

    # 이미지 업로드
    result = upload_image(api_url, image_path, event_id, header_key, header_value, box_result, timestamp, image_name)
    if result["status"] == "success":
        print(f"[성공] 응답: {result['response']}")
    else:
        print(f"[실패] 상태 코드: {result.get('code')} | 메시지: {result.get('message')}")