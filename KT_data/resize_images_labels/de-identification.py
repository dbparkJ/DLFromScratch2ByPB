import os
from ultralytics import YOLO

# 재귀적으로 폴더를 검색하고, 조건에 맞는 폴더 경로를 리스트로 반환하는 함수
def collect_folders_with_condition(input_folder):
    matching_folders = []

    # os.walk()를 사용해 폴더 탐색
    for root, dirs, files in os.walk(input_folder):
        # Camera01 ~ Camera04 폴더만 조건에 맞게 처리
        if any(cam in root for cam in ['Camera01', 'Camera02', 'Camera03', 'Camera04']):
            matching_folders.append(root)  # 조건에 맞는 폴더 경로를 리스트에 추가
            print(f"조건에 맞는 폴더 발견: {root}")

    return matching_folders

# YOLO 모델에 폴더 경로를 넘기는 함수
def process_folder_with_model(model, folder):
    # YOLOv8 모델에 폴더 경로 전달 및 추론
    print(f"폴더 {folder} 경로를 모델에 넣습니다.")
    results = model(folder, stream=True, device=0)  # 폴더 경로를 YOLO 모델에 전달

    # 모델 결과 처리
    for result in results:
        print(f"모델 결과: {result}")

# 메인 실행 함수
def main(input_folder, model):
    # 조건에 맞는 폴더들을 수집
    folders = collect_folders_with_condition(input_folder)

    # 각 폴더를 YOLO 모델에 한 번씩 넣기
    for folder in folders:
        process_folder_with_model(model, folder)

# YOLOv8 모델 불러오기
model = YOLO('best.pt')

# 최상단 입력 폴더 경로
input_folder = r'D:\down'  # 최상단 폴더 경로로 변경

# 메인 함수 실행
main(input_folder, model)
