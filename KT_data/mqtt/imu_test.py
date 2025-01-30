import paho.mqtt.client as mqtt
import json

# MQTT 브로커 정보
BROKER = "220.90.239.142"  # 브로커 주소 (예: localhost, IP 주소, 또는 클라우드 브로커)
PORT = 1883  # 기본 MQTT 포트
TOPIC = "mrm/geon-1/manager/imuData"

# 인증 정보
AUTH_ID = "robot"  # 인증 ID
AUTH_PW = "robot123"  # 인증 PW

# 발행할 메시지 데이터
data = {
    "transactionId": "a61e7fac-9819-4a5b-a095-e0e04c7a5cda",
    "robotId": "geon-1",
    "timestamp": "20241108140435",
    "imu": {
        "accelerometer": {"x": 0.0, "y": 0.73549875, "z": 9.914523149999999},
        "gyroscope": {"x": 0.0041887902047863905, "y": 0.0041887902047863905, "z": 0.008552113334772213},
        "magnetometer": {"x": 0.0364, "y": 0.0182, "z": 0.4201, "w": 0.9066},
    },
}

# MQTT 클라이언트 초기화
client = mqtt.Client(client_id="", clean_session=True, userdata=None, protocol=mqtt.MQTTv311, transport="tcp")

# 인증 설정
client.username_pw_set(username=AUTH_ID, password=AUTH_PW)

# 연결 이벤트 콜백
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        # 연결 성공 시 데이터 발행
        payload = json.dumps(data)  # JSON 포맷으로 직렬화
        client.publish(TOPIC, payload)
        print(f"Data published to topic {TOPIC}: {payload}")
    else:
        print("Failed to connect to MQTT broker.")

# MQTT 브로커 연결
client.on_connect = on_connect
client.connect(BROKER, PORT, 60)

# 네트워크 루프 시작
client.loop_forever()
