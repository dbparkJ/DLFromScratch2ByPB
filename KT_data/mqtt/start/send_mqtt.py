#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix, Imu
import paho.mqtt.client as mqtt
import json
import uuid
import time
import datetime
import threading
import os

# === MQTT 및 로봇 관련 설정 ===
MQTT_HOST = "220.90.239.142"
MQTT_PORT = 1883
MQTT_USER = "robot"
MQTT_PASS = "robot123"

ROBOT_ID = "geon-2"

MQTT_TOPIC_GPS = f"mrm/{ROBOT_ID}/manager/gpsData"
MQTT_TOPIC_IMU = f"mrm/{ROBOT_ID}/manager/imuData"
MQTT_TOPIC_SYS = f"mrm/{ROBOT_ID}/manager/systemStatus"

# 시스템 상태 파일 (예: ~/.sys.json)
SYSINFO_FILE = os.path.expanduser("~/.sys.json")

# MQTT 발행 주기 (초)
PUB_INTERVAL_GPS = 1.0
PUB_INTERVAL_IMU = 1.0
PUB_INTERVAL_SYS = 3.0

# 마지막 발행 시각 (초 단위, 초기값 0)
last_pub_gps = 0
last_pub_imu = 0
last_pub_sys = 0

# === MQTT 클라이언트 설정 ===
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
mqtt_client.connect(MQTT_HOST, MQTT_PORT)
mqtt_client.loop_start()  # 백그라운드에서 네트워크 루프 실행

def current_timestamp():
    """현재 시각을 YYYYMMDDHHMMSS 포맷 문자열로 반환"""
    return datetime.datetime.now().strftime("%Y%m%d%H%M%S")

def publish_mqtt(topic, message_dict):
    """
    전달받은 딕셔너리에 transactionId, robotId, timestamp 필드를 추가한 후
    JSON 문자열로 변환하여 MQTT 브로커로 발행한다.
    """
    message_dict.setdefault("transactionId", str(uuid.uuid4()))
    message_dict.setdefault("robotId", ROBOT_ID)
    message_dict.setdefault("timestamp", current_timestamp())
    message_json = json.dumps(message_dict)
    mqtt_client.publish(topic, message_json)
    #rospy.loginfo(f"Published to {topic}: {message_json}")

# === ROS 콜백 함수 ===
def gps_callback(msg: NavSatFix):
    global last_pub_gps
    now = time.time()
    if now - last_pub_gps >= PUB_INTERVAL_GPS:
        last_pub_gps = now
        # GPS 데이터 JSON 구성
        gps_data = {
            "gps": {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude
            }
        }
        publish_mqtt(MQTT_TOPIC_GPS, gps_data)

def imu_callback(msg: Imu):
    global last_pub_imu
    now = time.time()
    if now - last_pub_imu >= PUB_INTERVAL_IMU:
        last_pub_imu = now
        # IMU 데이터 JSON 구성
        imu_data = {
            "imu": {
                "accelerometer": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z
                },
                "gyroscope": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z
                },
                # 원래 코드에서는 orientation 값을 "magnetometer"라고 출력함
                "magnetometer": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w
                }
            }
        }
        publish_mqtt(MQTT_TOPIC_IMU, imu_data)

# === 시스템 상태 발행 함수 (별도 스레드에서 실행) ===
def system_status_loop():
    global last_pub_sys
    rate = 0.5  # 체크 주기 (0.5초)
    while not rospy.is_shutdown():
        now = time.time()
        if now - last_pub_sys >= PUB_INTERVAL_SYS:
            last_pub_sys = now
            try:
                with open(SYSINFO_FILE, 'r') as f:
                    sys_status = json.load(f)
            except Exception as e:
                rospy.logerr("Failed to read system info file: %s", e)
                sys_status = {}
            sys_data = {
                "systemStatus": sys_status
            }
            publish_mqtt(MQTT_TOPIC_SYS, sys_data)
        time.sleep(rate)

# === 토픽 대기 함수 ===
def wait_for_required_topics():
    """ /ublox/fix와 /imu 토픽이 발행되기 전까지 대기한다."""
    rospy.loginfo("Waiting for /ublox/fix topic...")
    rospy.wait_for_message("/ublox/fix", NavSatFix)
    rospy.loginfo("/ublox/fix topic detected.")

    rospy.loginfo("Waiting for /imu topic...")
    rospy.wait_for_message("/imu", Imu)
    rospy.loginfo("/imu topic detected.")

def main():
    rospy.init_node('mqtt_bridge', anonymous=True)
    rospy.loginfo("MQTT bridge node started.")

    # 필수 토픽이 활성화될 때까지 대기
    wait_for_required_topics()

    # ROS 구독자 등록 (토픽명을 변경)
    rospy.Subscriber("/ublox/fix", NavSatFix, gps_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)

    # 시스템 상태 발행 스레드 시작
    sys_thread = threading.Thread(target=system_status_loop)
    sys_thread.daemon = True
    sys_thread.start()

    # ROS 이벤트 루프 시작
    rospy.spin()

    # 노드 종료 시 MQTT 루프 종료
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
