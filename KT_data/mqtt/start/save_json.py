#!/usr/bin/env python3
import os
import re
import glob
import json
import time
import subprocess

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

# ----- 센서 상태 관련 함수 -----

def sys_camera():
    """
    lsusb 명령어로 'Intel'과 'Movidius' 문자열이 있는지 확인하여
    카메라 하드웨어가 연결되어 있는지, ps 명령어로 depthai 프로세스가 실행 중인지 확인한 후,
    rostopic echo 명령어로 /oak/rgb/image_raw와 /oak/stereo/image_raw 두 토픽에서
    메시지가 정상적으로 수신되고 있는지를 검사한다.
    만약 둘 중 한 토픽이라도 메시지가 수신되지 않으면 상태를 ERROR로, 오류 코드를 -1로 설정한다.
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
            # 프로세스 검사
            ps_out = subprocess.check_output(
                ["ps", "ef"],
                universal_newlines=True,
                stderr=subprocess.DEVNULL
            )
            if "depthai" not in ps_out:
                status = "IDLE"
        
        # ROS 토픽 검사: rostopic echo 명령어 사용 (각 토픽에 대해 평균 메시지 속도 확인)
        if status == "ACTIVE":
            try:
                rgb_hz_output = subprocess.check_output(
                    ["rostopic", "echo", "-n", "2", "/oak/rgb/image_raw"],
                    universal_newlines=True,
                    timeout=3,
                    stderr=subprocess.DEVNULL
                )
            except Exception:
                rgb_hz_output = ""
            try:
                stereo_hz_output = subprocess.check_output(
                    ["rostopic", "echo", "-n", "2", "/oak/stereo/image_raw"],
                    universal_newlines=True,
                    timeout=3,
                    stderr=subprocess.DEVNULL
                )
            except Exception:
                stereo_hz_output = ""
            # 둘 중 하나라도 "header" 관련 문자열이 없으면 메시지가 수신되지 않은 것으로 판단
            if ("header" not in rgb_hz_output) or ("header:" not in stereo_hz_output):
                status = "ERROR"
                error = "-2"  # ROS 토픽에서 메시지를 받지 못함
    except Exception as e:
        status = "ERROR"
        error = "-1"
        print("DEBUG: Exception in sys_camera:", e)
    
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
    rosparam get 명령어를 타임아웃 3초 내에 실행하여 파라미터 값을 문자열로 반환한다.
    실패 시 빈 문자열을 반환한다.
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
    rostopic echo /ublox/fix를 실행하여 현재 GPS 위경도 정보를 가져온다.
    """
    try:
        gps_output = subprocess.check_output(
            ["rostopic", "echo", "-n", "1", "/ublox/fix"],
            universal_newlines=True,
            timeout=2,
            stderr=subprocess.DEVNULL
        )
        return parse_gps_data(gps_output)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None, None, None

def parse_gps_data(gps_raw):
    """
    rostopic echo /ublox/fix의 출력을 분석하여 위도(latitude)와 경도(longitude)를 추출한다.
    """
    latitude, longitude, status = None, None, None
    for line in gps_raw.split("\n"):
        line = line.strip()
        if line.startswith("latitude:"):
            try:
                latitude = float(line.split(":")[1].strip())
            except ValueError:
                latitude = None
        elif line.startswith("longitude:"):
            try:
                longitude = float(line.split(":")[1].strip())
            except ValueError:
                longitude = None
        elif line.startswith("status:"):
            try:
                status = int(line.split(":")[1].strip())
            except ValueError:
                status = None
    return latitude, longitude, status

def is_gps_in_korea(latitude, longitude):
    """
    위도와 경도를 기준으로 GPS 값이 한국에 속하는지 확인한다.
    한국의 범위: 위도(33~44), 경도(124~132)
    """
    if latitude is None or longitude is None:
        return False
    return 33.0 <= latitude <= 44.0 and 124.0 <= longitude <= 132.0

def sys_gps():
    """
    기존 GPS 상태 확인 로직에 추가로 위경도 값을 검사하여
    GPS가 한국 외 지역이라면 오류를 반환하도록 수정.
    """
    dev = get_rosparam("/ublox/device")
    baud = get_rosparam("/ublox/uart1/baudrate")
    #ntrip_server = get_rosparam("/ntrip_ros/ntrip_server")
    #ntrip_user = get_rosparam("/ntrip_ros/ntrip_user")
    #ntrip_stream = get_rosparam("/ntrip_ros/ntrip_stream")
    
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
        error = "-3"  # GPS status error or location is outside Korea
    
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
    rostopic echo /imu를 실행하여 IMU 데이터를 가져온다.
    토픽이 publish되지 않아도 3초 후 타임아웃되도록 한다.
    """
    try:
        imu_data = subprocess.check_output(
            ["rostopic", "echo", "-n", "1", "/imu"],
            universal_newlines=True,
            timeout=2,
            stderr=subprocess.DEVNULL
        )
        return parse_imu_data(imu_data)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None

def parse_imu_data(imu_raw):
    """
    rostopic echo /imu의 출력을 분석하여 IMU 데이터를 파싱한다.
    """
    imu_values = {"orientation": {}, "angular_velocity": {}, "linear_acceleration": {}}
    current_section = None
    
    for line in imu_raw.split("\n"):
        line = line.strip()
        if not line or ":" not in line:
            continue  # 빈 줄 또는 ':'이 없는 줄 무시
        
        if line.startswith("orientation:"):
            current_section = "orientation"
        elif line.startswith("angular_velocity:"):
            current_section = "angular_velocity"
        elif line.startswith("linear_acceleration:"):
            current_section = "linear_acceleration"
        elif current_section:
            parts = line.split(":", 1)  # 최대 한 번만 분할
            key = parts[0].strip()
            value = parts[1].strip()
            try:
                if "[" in value and "]" in value:
                    imu_values[current_section][key] = json.loads(value)
                else:
                    imu_values[current_section][key] = float(value)
            except ValueError:
                imu_values[current_section][key] = None
    
    return imu_values

def is_imu_data_zero(imu_values):
    if not imu_values:
        return True  # 데이터가 없으면 모두 0으로 간주
    for section in imu_values.values():
        for value in section.values():
            if isinstance(value, list):
                if any(v != 0.0 for v in value):
                    return False
            elif value != 0.0:
                return False
    return True

def sys_imu():
    """
    /dev/e2box 장치의 존재 여부와 fuser, rosparam 값을 확인하여
    IMU의 상태를 판단한다.
    """
    dev = get_rosparam("/e2box_imu_node/port")
    baud = get_rosparam("/e2box_imu_node/baudrate")
    
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
    
    imu_values = get_imu_data()
    if imu_values is None or is_imu_data_zero(imu_values):
        status = "ERROR"
        error = "-3"  # IMU data is all zero or not received
    
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
    # 파일 초기화
    with open(SYSINFO, "w") as f:
        f.write("")
    
    while True:
        try:
            status = get_system_status()
            with open(SYSINFO, "w") as f:
                json.dump(status, f)
        except Exception as e:
            print("Error in main loop:", e)
        time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
