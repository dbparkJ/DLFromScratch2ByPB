#!/usr/bin/env python3

import cv2
import numpy as np
import csv
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

# 저역 필터 (Low-pass filter) 설정
alpha = 0.0001  # 필터 민감도 (작을수록 필터가 강하게 작동)


# Low-pass 필터 함수
def low_pass_filter(prev_value, current_value, alpha=0.1):
    return alpha * current_value + (1 - alpha) * prev_value


# IMU 데이터 로드 함수
def load_imu_data(imu_csv_file):
    imu_data = []
    with open(imu_csv_file, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # 헤더 스킵

        accel_data = None
        for row in reader:
            if row[1] != '':  # 가속도계 데이터가 있는 경우
                accel_data = [float(row[1]), float(row[2]), float(row[3])]
            elif row[4] != '' and accel_data:  # 자이로스코프 데이터가 있고, 가속도계 데이터가 존재할 때
                gyro_data = [float(row[4]), float(row[5]), float(row[6])]
                # 가속도계와 자이로스코프 데이터를 결합하여 imu_data에 추가
                imu_data.append(accel_data + gyro_data)
    return imu_data


# 각 프레임을 처리하는 함수
def process_frame(i, frame, imu_data, prev_gyro, alpha):
    if i < len(imu_data):
        gyro_data = imu_data[i][3:]  # 자이로스코프 데이터 (x, y, z)

        # prev_gyro가 없으면 초기화
        if prev_gyro is None:
            prev_gyro = np.array(gyro_data)

        # 저역 필터를 적용하여 자이로 데이터 필터링
        filtered_gyro = low_pass_filter(prev_gyro, np.array(gyro_data), alpha)
        prev_gyro = filtered_gyro  # 이전 값 업데이트

        # 회전 각도를 계산하여 보정
        angle = filtered_gyro[2]  # Z축 회전 (yaw)

        # Affine 변환 매트릭스를 이용해 프레임 보정
        center = (frame.shape[1] // 2, frame.shape[0] // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, -math.degrees(angle), 1)

        # 회전 변환 적용
        stabilized_frame = cv2.warpAffine(frame, rotation_matrix, (frame.shape[1], frame.shape[0]))

        # 메모리 해제
        del rotation_matrix
        return stabilized_frame, prev_gyro
    else:
        # IMU 데이터가 없는 경우 원본 프레임 반환
        return frame, prev_gyro


# 비디오 보정 함수 (멀티스레딩 사용 + 진행 상태 출력)
def stabilize_video(input_file, imu_csv_file, output_file):
    print(f"Processing {input_file}...")

    # IMU 데이터 로드
    imu_data = load_imu_data(imu_csv_file)

    # 비디오 파일 열기
    cap = cv2.VideoCapture(input_file)

    # 비디오 속성 가져오기
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # 비디오 파일 저장 설정
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

    prev_gyro = None  # 초기 prev_gyro 설정

    # 전체 처리 시간 측정 시작
    total_start_time = time.time()

    for i in range(total_frames):
        ret, frame = cap.read()
        if not ret:
            break

        # 각 프레임을 순차적으로 처리
        stabilized_frame, prev_gyro = process_frame(i, frame, imu_data, prev_gyro, alpha)
        out.write(stabilized_frame)

        # 메모리 관리: 이미 처리한 프레임 메모리 해제
        del frame
        del stabilized_frame

    # 전체 처리 시간 측정 종료
    total_end_time = time.time()
    total_processing_time = total_end_time - total_start_time
    print(f"Completed {input_file} in {total_processing_time:.4f} seconds")

    # 리소스 정리
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(f"Stabilization complete. Saved as {output_file}")


# 멀티스레드로 여러 비디오 파일을 동시에 처리하는 함수
def stabilize_multiple_videos(input_folder):
    video_files = [f for f in os.listdir(input_folder) if f.endswith(".mp4")]

    # 스레드 풀 생성
    with ThreadPoolExecutor(max_workers=32) as executor:
        futures = []

        # 각 비디오 파일에 대해 작업을 제출
        for filename in video_files:
            base_name = os.path.splitext(filename)[0]
            imu_csv_file = os.path.join(input_folder, f"{base_name}.csv")
            input_video = os.path.join(input_folder, filename)
            output_folder = os.path.join(input_folder, 'output')

            # output 폴더가 없으면 생성
            if not os.path.exists(output_folder):
                os.makedirs(output_folder)

            output_video = os.path.join(output_folder, f"{base_name}_stabilized.mp4")

            if os.path.exists(imu_csv_file):
                # 비디오 처리 작업을 비동기적으로 제출
                future = executor.submit(stabilize_video, input_video, imu_csv_file, output_video)
                futures.append(future)
            else:
                print(f"CSV file for {filename} not found.")

        # 작업이 완료될 때까지 기다림
        for future in as_completed(futures):
            future.result()


if __name__ == "__main__":
    # 폴더 경로 입력받기
    input_folder = input(r"Enter the folder path containing the videos and CSVs: ").strip()
    stabilize_multiple_videos(input_folder)
