import cv2
import os
import concurrent.futures

def save_frames(video_path, output_folder, frame_rate, compression=3):
    # 비디오 파일 열기
    cap = cv2.VideoCapture(video_path)

    # 비디오의 총 프레임 수와 초당 프레임 수 가져오기
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    # 저장할 간격 설정 (초당 frame_rate 장 저장)
    save_interval = fps // frame_rate

    # 출력 폴더가 없으면 생성
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    frame_id = 0
    saved_frame_count = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 지정된 간격에 따라 프레임 저장
        if frame_id % save_interval == 0:
            output_path = os.path.join(output_folder, f"frame_{saved_frame_count:04d}.png")
            # PNG 압축 설정
            cv2.imwrite(output_path, frame, [cv2.IMWRITE_PNG_COMPRESSION, compression])
            saved_frame_count += 1

        frame_id += 1

    cap.release()

def process_video(video_file, input_folder, output_folder, frame_rate, compression):
    video_path = os.path.join(input_folder, video_file)
    video_output_folder = os.path.join(output_folder, os.path.splitext(video_file)[0])
    save_frames(video_path, video_output_folder, frame_rate, compression)

def save_frames_from_videos(input_folder, output_folder, frame_rate, compression=3):
    # 입력 폴더의 모든 파일을 확인
    video_files = [f for f in os.listdir(input_folder) if f.endswith(('.mp4', '.avi', '.mkv'))]

    # 멀티쓰레딩으로 각 비디오 파일을 처리
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = [executor.submit(process_video, video_file, input_folder, output_folder, frame_rate, compression) for video_file in video_files]
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as exc:
                print(f"An error occurred: {exc}")

    print(f"Frames extracted and saved in '{output_folder}'.")

# 입력 폴더와 출력 폴더 경로, 초당 저장할 프레임 수와 압축률을 사용자로부터 입력받음
input_folder = r"C:\Users\JM\Desktop\2024.10.14\data"
output_folder = r"C:\Users\JM\Desktop\2024.10.14\data\output"
frame_rate = 3
compression = 3  # 압축률 설정 (0-9, 숫자가 클수록 압축률이 높아짐)

# 여러 동영상을 멀티쓰레딩으로 처리
save_frames_from_videos(input_folder, output_folder, frame_rate, compression)
