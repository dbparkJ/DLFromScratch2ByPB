#!/usr/bin/env python3
# sudo apt-get install wmctrl
import subprocess
import time
import json
import os
import signal

# JSON 파일 경로
JSON_FILE_PATH = "/home/dogu/.sys.json"

# 실행할 ROS 노드와 관련 명령어 (환경변수 로드를 포함)
ROS_NODES = {
    "roscore": [
        "source /opt/ros/noetic/setup.bash; roscore"
    ],
    "imu": [
        "source /home/dogu/catkin_ws/devel_isolated/setup.bash; roslaunch ebimu_9dofv5 e2box_imu.launch",
        #"sudo -n stty -F /dev/ttyUSB0 921600; exit"
    ],
    "gps": [
        #"sudo -n /home/jm/add_ftdi_device.sh; exit",
        "source /home/dogu/catkin_ws/devel_isolated/setup.bash; roslaunch ublox_gps ublox_device.launch",
        #"sudo -n stty -F /dev/ttyUSB5 921600; exit"
    ],
    "camera": [
        "source /opt/ros/noetic/setup.bash; source ~/.bashrc; python3 /home/dogu/KT_device_code/start/pub_dai_topic.py"
        #"source /opt/ros/noetic/setup.bash; roslaunch depthai_ros_driver rgbd_pcl.launch params_file:=/home/dogu/oak_d.yaml"
    ],
    "system_status": [
        "source ~/.bashrc; python3 /home/dogu/system_status.py"
    ],
    "send_mqtt": [
        "source /opt/ros/noetic/setup.bash; source ~/.bashrc; python3 /home/dogu/send_mqtt.py"
    ],
    "heading" : [
        "source /home/dogu/catkin_ws/devel/setup.bash; source ~/.bashrc; rosrun geon_heading run_heading_value.py"
    ]
}

# 각 노드의 터미널(프로세스 그룹) 정보를 저장할 딕셔너리
ros_processes = {}

def read_json():
    """
    JSON 파일을 읽어 sensors 항목 내 각 센서의 errorCode를 반환합니다.
    파일이 없거나 오류 발생 시 빈 딕셔너리를 반환합니다.
    """
    if not os.path.exists(JSON_FILE_PATH):
        print(f"파일을 찾을 수 없음: {JSON_FILE_PATH}")
        return {}
    try:
        with open(JSON_FILE_PATH, "r") as f:
            data = json.load(f)
        return {k: v.get("errorCode") for k, v in data.get("sensors", {}).items()}
    except Exception as e:
        print(f"JSON 읽기 오류: {e}")
        return {}

def run_ros_node(node_name):
    """
    각 노드를 실행할 때 gnome-terminal을 열면서 해당 터미널에 제목(title)을 부여합니다.
    - system_status 노드는 인터랙티브 셸(-i)로 실행하여 환경 설정을 온전히 반영합니다.
    - 그 외 노드는 명령 실행 후 'exit'하여 터미널 창이 자동으로 닫히도록 합니다.
    """
    if node_name not in ROS_NODES:
        print(f"알 수 없는 노드: {node_name}")
        return

    print(f"[START] {node_name} 노드를 실행합니다...")
    processes = []
    for cmd in ROS_NODES[node_name]:
        if node_name == "system_status":
            # system_status는 인터랙티브 모드로 실행 (명령 실행 후 창 유지)
            terminal_cmd = f"gnome-terminal --title='{node_name}' -- bash -i -c '{cmd}; exec bash'"
        else:
            # 나머지 노드는 명령 실행 후 exit되어 창이 닫힘
            terminal_cmd = f"gnome-terminal --title='{node_name}' -- bash -c '{cmd}; exit'"
        print(f"실행 명령: {terminal_cmd}")
        process = subprocess.Popen(terminal_cmd, shell=True, preexec_fn=os.setsid)
        processes.append(process)
        time.sleep(15)  # 각 명령어 실행 간 잠시 대기 (필요에 따라 조절)
    ros_processes[node_name] = processes

def restart_ros_node(node_name):
    """
    error code를 감지하면 해당 노드의 터미널 창(프로세스 그룹)에 SIGINT를 보내 정상 종료를 시도합니다.
    5초 동안 종료되지 않으면 SIGKILL으로 강제 종료하고, 추가로 wmctrl을 사용해 해당 창을 닫습니다.
    이후 완전히 종료된 후 새 roslaunch 창을 실행합니다.
    """
    print(f"[RESTART] {node_name} 노드에서 오류 발생! 기존 창을 종료하고 재실행합니다.")
    if node_name in ros_processes:
        # 1) SIGINT 전송 (정상 종료 유도)
        for proc in ros_processes[node_name]:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    print(f"SIGINT 전송: {node_name} 프로세스 그룹 (PID: {proc.pid})")
                except Exception as e:
                    print(f"{node_name} SIGINT 전송 실패: {e}")
        time.sleep(5)  # 종료되길 기다림

        # 2) 그래도 종료되지 않은 프로세스에 대해 SIGKILL 전송
        for proc in ros_processes[node_name]:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    print(f"SIGKILL 전송: {node_name} 프로세스 그룹 (PID: {proc.pid})")
                except Exception as e:
                    print(f"{node_name} SIGKILL 전송 실패: {e}")
        time.sleep(5)

        # 3) wmctrl을 사용하여 해당 제목을 가진 터미널 창을 강제로 닫음
        try:
            subprocess.run(["wmctrl", "-c", node_name], check=True)
            print(f"wmctrl: '{node_name}' 제목의 터미널 창 종료")
        except Exception as e:
            print(f"wmctrl으로 '{node_name}' 창 종료 실패: {e}")
        time.sleep(5)
    # 새롭게 노드 실행
    run_ros_node(node_name)

if __name__ == "__main__":
    time.sleep(3)
    # 초기 실행: 각 노드를 순차적으로 실행합니다.
    for node in ROS_NODES.keys():
        run_ros_node(node)

    print("1분 대기 후 시스템 상태 확인 시작...")
    time.sleep(60)

    # 주기적으로 JSON 파일을 확인하여 errorCode가 "0"이 아닌 경우 해당 노드를 재실행합니다.
    while True:
        error_status = read_json()
        for node, error_code in error_status.items():
            if error_code != "0":
                print(f"[ERROR] {node} 노드에서 오류 발생 (errorCode: {error_code})")
                restart_ros_node(node)
        time.sleep(30)

