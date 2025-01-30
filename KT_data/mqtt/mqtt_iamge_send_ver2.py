import os
import requests
import paho.mqtt.client as mqtt
import json


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


def upload_image(api_url, image_path, event_id, header_key, header_value):
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
        "images": (f"{event_id}.png", open(image_path, "rb"), "image/png")
    }

    try:
        print(f"파일 업로드 중: {image_path} → {api_url}")
        print(f"Event ID: {event_id}")
        response = requests.post(api_url, headers=headers, files=files)
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
    topic = "mrm/geon-1/eventAI/result"
    api_url = f"http://{ip}:7080/col/v1/image/saveImageFiles.do"
    image_path = "./data/2024-11-19_09-31-18_frame_0509_deep_portholl.png"  # 업로드할 이미지 파일 경로
    header_key = "X-GEO-ROADAI-API"
    header_value = "Geon ce79e749650fb0c8595801d94c222bbc"

    # MQTT 메시지 (여러 events 포함)
    message = {
        "transactionId": "17df4a3b-97ea-4b69-a2ea-92b6a9d9a53c",
        "messageId": "6a2bf3a6-bd7d-439c-81f4-e8c7cb15f4b2",
        "robotId": "geon-1",
        "timestamp": "20250114130259",
        "events": [
            {
                "eventID": "dt0001",
                "eventType": "SOC",
                "timestamp": "20250114130259",
                "location": {
                    "x": 126.978405,
                    "y": 37.715816
                },
                "eventContent": {
                    "track_id": "8109",
                    "classified": "101",
                    "score": "0.29"
                },
            }
        ]
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

    # MQTT 연결 시작
    client.connect(ip, port, 60)
    client.loop_start()

    # 각 이벤트 처리
    for event in message["events"]:
        # 개별 이벤트 ID 추출
        event_id = event["eventID"]
        print(f"Processing Event ID: {event_id}")

        # MQTT 메시지 발행 (이벤트 별로 메시지를 발행할 경우)
        single_event_message = {
            "transactionId": message["transactionId"],
            "messageId": message["messageId"],
            "robotId": message["robotId"],
            "timestamp": message["timestamp"],
            "events": [event]  # 개별 이벤트만 포함
        }
        publish_message(client, topic, single_event_message)

        # 이미지 업로드
        result = upload_image(api_url, image_path, event_id, header_key, header_value)
        if result["status"] == "success":
            print(f"[성공] Event ID {event_id} 응답: {result['response']}")
        else:
            print(f"[실패] Event ID {event_id} 상태 코드: {result.get('code')} | 메시지: {result.get('message')}")

    client.loop_stop()


