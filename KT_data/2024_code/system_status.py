#!/usr/bin/env python3
import os
import re
import glob
import json
import time
import subprocess
import rospy
from sensor_msgs.msg import Image, NavSatFix, Imu

# 기본 변수 설정
HOST = "220.90.239.142"
SYSINFO = os.path.expanduser("~/.sys.json")
DEFAULT_NETWORK_TYPE = ""  # 필요시 "Ethernet" 또는 "WiFi" 등으로 지정 가능

# ------------------------------------------------------------------
# 시스템 정보 관련 함수들
# ------------------------------------------------------------------

def sys_network():
    """
    ip route를 이용해 HOST로 가는 경로의 인터페이스를 찾고,
    ifconfig 명령어로 TX/RX 데이터(괄호 안의 문자열)를 추출한다.
    """
    interface = ""
    try:
        out = subprocess.check_output(
            ["ip", "route", "get", HOST],
            universal_newlines=True,
            stderr=subprocess.DEVNULL
        )
        m = re.search(r'(?<=dev )\S+', out)
        if m:
            interface = m.group(0)
    except Exception:
        interface = ""
    
    # 네트워크 타입 결정
    if DEFAULT_NETWORK_TYPE:
        net_type = DEFAULT_NETWORK_TYPE
    else:
        if interface.startswith("e"):
            net_type = "Ethernet"
        elif interface.startswith("w"):
            net_type = "WiFi"
        else:
            net_type = interface

    data_sent = ""
    data_recv = ""
    try:
        ifconfig_out = subprocess.check_output(
            ["ifconfig", interface],
            universal_newlines=True,
            stderr=subprocess.DEVNULL
        )
        # TX 데이터: "TX ... bytes ... (XXX)" 형식에서 괄호 안의 내용을 추출
        tx_match = re.search(r'TX.*bytes.*\(([^)]+)\)', ifconfig_out)
        if tx_match:
            data_sent = tx_match.group(1).replace(" ", "")
        rx_match = re.search(r'RX.*bytes.*\(([^)]+)\)', ifconfig_out)
        if rx_match:
            data_recv = rx_match.group(1).replace(" ", "")
    except Exception:
        data_sent = ""
        data_recv = ""
    
    return {
        "type": net_type,
        "dataSent": data_sent,
        "dataReceived": data_recv
    }

def sys_uptime():
    """ /proc/uptime의 첫번째 숫자를 정수(초)로 반환 """
    try:
        with open("/proc/uptime", "r") as f:
            contents = f.read().split()
            return int(float(contents[0]))
    except Exception:
        return 0

def sys_memory():
    """
    /proc/meminfo에서 MemTotal과 MemAvailable을 읽어
    각각 GB 단위(대문자 "GB" 포함, 소수점 둘째자리까지의 문자열)로 변환하여 반환한다.
    (1 GB = 1024 * 1024 kB)
    """
    mem_total = 0
    mem_available = 0
    try:
        with open("/proc/meminfo", "r") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    parts = line.split()
                    mem_total = int(parts[1])  # 단위: kB
                elif line.startswith("MemAvailable:"):
                    parts = line.split()
                    mem_available = int(parts[1])
    except Exception:
        pass

    total_gb = mem_total / (1024 * 1024)
    available_gb = mem_available / (1024 * 1024)
    used_gb = total_gb - available_gb  # 사용된 메모리
    return {
        "total": round(total_gb, 2),
        "used": round(used_gb, 2)
    }

def sys_disk():
    """
    df -BK / 명령어를 통해 루트 파티션의 총 용량과 사용 용량(킬로바이트 단위)을 읽어
    GB 단위(대문자 "GB" 포함, 소수점 둘째자리까지의 문자열)로 변환하여 반환한다.
    (1 GB = 1024 * 1024 KB)
    """
    disk_total = 0
    disk_used = 0
    try:
        output = subprocess.check_output(
            ["df", "-BK", "/"],
            universal_newlines=True,
            stderr=subprocess.DEVNULL
        )
        lines = output.strip().split("\n")
        if len(lines) >= 2:
            parts = lines[1].split()
            disk_total = int(re.search(r'\d+', parts[1]).group(0))
            disk_used = int(re.search(r'\d+', parts[2]).group(0))
    except Exception:
        pass
    total_gb = disk_total / (1024 * 1024)
    used_gb = disk_used / (1024 * 1024)
    return {
        "total": round(total_gb, 2),
        "used": round(used_gb, 2)
    }

def sys_cpu():
    """
    /proc/stat의 첫번째 "cpu " 라인을 읽은 후 1초 뒤에 다시 읽어,
    CPU 사용률을 계산하여 반환한다.
    """
    def read_cpu():
        with open("/proc/stat", "r") as f:
            for line in f:
                if line.startswith("cpu "):
                    parts = line.split()
                    user = float(parts[1])
                    system = float(parts[3])
                    idle = float(parts[4])
                    return user, system, idle
        return (0, 0, 0)
    
    user1, system1, idle1 = read_cpu()
    time.sleep(1)
    user2, system2, idle2 = read_cpu()
    
    u1 = user1 + system1
    t1 = user1 + system1 + idle1
    u2 = user2 + system2
    t2 = user2 + system2 + idle2
    if (t2 - t1) > 0:
        usage = (u2 - u1) * 100.0 / (t2 - t1)
    else:
        usage = 0.0
    return round(usage, 4)

def sys_gpu():
    """
    /sys/devices/gpu*/load 파일들을 읽어 각 GPU의 로드를 취합한 후
    평균값을 10.0으로 나눈 값을 반환한다.
    """
    loads = []
    for path in glob.glob("/sys/devices/gpu*/load"):
        try:
            with open(path, "r") as f:
                load_value = int(f.read().strip())
                loads.append(load_value)
        except Exception:
            continue
    if loads:
        avg = sum(loads) / len(loads)
        return avg / 10.0
    else:
        return 0.0

def sys_camera():
    """
    카메라 상태 확인:
      - USB 장치와 관련 프로세스를 검사하고,
      - rospy.wait_for_message를 사용하여 /oak/rgb/image_raw와 /oak/stereo/image_raw 토픽으로부터 메시지를 수신하는지 확인함.
    """
    status = "ACTIVE"
    error = "0"
    fps = get_rosparam("/oak/rgb_i_fps")
    try:
        # USB 장치 검사
        lsusb_out = subprocess.check_output(
            ["lsusb"],
            universal_newlines=True,
            stderr=subprocess.DEVNULL
        )
        if not (("intel" in lsusb_out.lower()) and ("movidius" in lsusb_out.lower())):
            status = "ERROR"
            error = "-1"  # Device disconnect
        else:
            pass
        
        # ROS 토픽 검사: rospy.wait_for_message로 메시지 수신 여부 확인 (timeout: 3초)
        if status == "ACTIVE":
            try:
                rgb_msg = rospy.wait_for_message("/oak/rgb/image_raw", Image, timeout=3.0)
            except rospy.ROSException:
                rgb_msg = None
            try:
                stereo_msg = rospy.wait_for_message("/oak/stereo/image_raw", Image, timeout=3.0)
            except rospy.ROSException:
                stereo_msg = None
            
            if rgb_msg is None or stereo_msg is None:
                status = "ERROR"
                error = "-2"  # ROS 토픽에서 메시지를 받지 못함
    except Exception as e:
        status = "ERROR"
        error = "-1"
        rospy.logerr("Exception in sys_camera: %s", e)
    
    return {
        "status": status,
        "errorCode": error,
        "param": {
            "type": "OAK-D PRO W IMX378",
            "fps": f"{fps}",
            "resolution": "1920x1080"
        }
    }

def get_rosparam(param):
    """
    rosparam get 명령어를 타임아웃 2초 내에 실행하여 파라미터 값을 문자열로 반환.
    실패 시 빈 문자열 반환.
    """
    try:
        result = subprocess.check_output(
            ["rosparam", "get", param],
            timeout=2,
            universal_newlines=True,
            stderr=subprocess.DEVNULL
        ).strip()
        return result
    except Exception:
        return ""

def get_gps_fix():
    """
    rospy.wait_for_message를 사용하여 /ublox/fix 토픽으로부터 GPS 데이터를 가져옴.
    """
    try:
        gps_msg = rospy.wait_for_message("/fix", NavSatFix, timeout=2.0)
        # gps_msg.status는 NavSatStatus 객체이며, gps_msg.status.status에 정수 값이 들어 있음
        return gps_msg.latitude, gps_msg.longitude, gps_msg.status.status
    except rospy.ROSException:
        return None, None, None

def is_gps_in_korea(latitude, longitude):
    """
    위도와 경도를 기준으로 GPS 값이 한국 내에 있는지 확인 (위도: 33~44, 경도: 124~132)
    """
    if latitude is None or longitude is None:
        return False
    return 33.0 <= latitude <= 44.0 and 124.0 <= longitude <= 132.0

def sys_gps():
    """
    GPS 상태 확인:
      - 장치 파일 존재 여부, fuser 확인, rosparam으로 baudrate 확인 후,
      - rospy.wait_for_message로 /ublox/fix 토픽에서 GPS 데이터를 가져와 위경도 범위 검사
    """
    dev = get_rosparam("/ublox/device")
    baud = get_rosparam("/ublox/uart1/baudrate")
    
    ntrip_server = "gnssdata.or.kr:2101"
    ntrip_user = "pjmsm0319@gmail.com"
    ntrip_stream = "GANS-RTCM32"
    
    status = "ACTIVE"
    error = "0"
    
    if not os.path.exists(dev):
        status = "ERROR"
        error = "-1"  # Device disconnect
    else:
        try:
            fuser_out = subprocess.check_output(
                ["fuser", dev],
                universal_newlines=True,
                stderr=subprocess.DEVNULL
            ).strip()
            if not fuser_out:
                status = "IDLE"
                error = "0"
        except subprocess.CalledProcessError:
            status = "IDLE"
            error = "0"
    
    if baud == "":
        status = "ERROR"
        error = "-2"  # ROS node is not running
    
    # GPS 데이터 확인
    latitude, longitude, gps_status = get_gps_fix()
    if gps_status == -1 or latitude is None or longitude is None or not is_gps_in_korea(latitude, longitude):
        status = "ERROR"
        error = "-3"  # GPS 상태 오류 또는 한국 외 지역
    
    return {
        "status": status,
        "errorCode": error,
        "param": {
            "type": "zed_f9p",
            "port": dev,
            "baudrate": baud,
            "ntrip_server": ntrip_server,
            "ntrip_user": ntrip_user,
            "ntrip_stream": ntrip_stream,
        }
    }

def get_imu_data():
    """
    rospy.wait_for_message를 사용하여 /imu 토픽으로부터 IMU 데이터를 가져옴.
    (timeout: 2초, 수신 실패 시 None 반환)
    """
    try:
        imu_msg = rospy.wait_for_message("/imu/data", Imu, timeout=2.0)
        return imu_msg
    except rospy.ROSException:
        return None

def is_imu_data_zero(imu_msg):
    """
    IMU 메시지의 모든 필드(orientation, angular_velocity, linear_acceleration)가 0인지 검사.
    """
    if imu_msg is None:
        return True
    if (imu_msg.orientation.x == 0.0 and imu_msg.orientation.y == 0.0 and
        imu_msg.orientation.z == 0.0 and imu_msg.orientation.w == 0.0 and
        imu_msg.angular_velocity.x == 0.0 and imu_msg.angular_velocity.y == 0.0 and
        imu_msg.angular_velocity.z == 0.0 and
        imu_msg.linear_acceleration.x == 0.0 and imu_msg.linear_acceleration.y == 0.0 and
        imu_msg.linear_acceleration.z == 0.0):
        return True
    return False

def sys_imu():
    """
    IMU 상태 확인:
      - /e2box 장치의 포트와 baudrate를 rosparam으로 읽고,
      - 장치 파일 및 fuser로 디바이스 상태를 확인한 후,
      - rospy.wait_for_message를 사용해 /imu 토픽에서 IMU 데이터를 수신하여 데이터가 0인지 검사함.
    """
    dev = get_rosparam("/ebimu_9dofv5_node/port")
    baud = get_rosparam("/ebimu_9dofv5_node/baudrate")
    
    status = "ACTIVE"
    error = "0"
    
    if not os.path.exists(dev):
        status = "ERROR"
        error = "-1"  # Device disconnect
    else:
        try:
            fuser_out = subprocess.check_output(
                ["fuser", dev],
                universal_newlines=True,
                stderr=subprocess.DEVNULL
            ).strip()
            if not fuser_out:
                status = "IDLE"
                error = "0"
        except subprocess.CalledProcessError:
            status = "IDLE"
            error = "0"
    
    if baud == "":
        status = "ERROR"
        error = "-2"  # ROS node is not running
    
    imu_msg = get_imu_data()
    if imu_msg is None or is_imu_data_zero(imu_msg):
        status = "ERROR"
        error = "-3"  # IMU 데이터가 모두 0이거나 수신되지 않음
    
    return {
        "status": status,
        "errorCode": error,
        "param": {
            "type": "e2box",
            "port": dev,
            "baudrate": baud,
            "sof": 2,
            "sog": 1,
            "soa": 1
        }
    }

def get_system_status():
    """
    pc와 sensors 정보를 포함하는 최종 JSON 객체를 구성한다.
    memory와 disk의 값은 GB 단위(소수점 둘째자리까지, "GB" 포함)로 고정되어 있다.
    """
    return {
        "pc": {
            "network": sys_network(),
            "uptime": sys_uptime(),
            "memory": sys_memory(),
            "disk": sys_disk(),
            "cpu": sys_cpu(),
            "gpu": sys_gpu()
        },
        "sensors": {
            "camera": sys_camera(),
            "gps": sys_gps(),
            "imu": sys_imu()
        }
    }

# ------------------------------------------------------------------
# 메인 루프: 1초마다 시스템 상태 정보를 파일에 기록
# ------------------------------------------------------------------

def main():
    # ROS 노드 초기화
    rospy.init_node("system_status_monitor", anonymous=True)
    rate = rospy.Rate(1)  # 1Hz (1초에 한번)

    # 파일 초기화
    with open(SYSINFO, "w") as f:
        f.write("")

    while not rospy.is_shutdown():
        try:
            status = get_system_status()
            with open(SYSINFO, "w") as f:
                json.dump(status, f)
        except Exception as e:
            rospy.logerr("Error in main loop: %s", e)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
