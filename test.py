import os
import shutil
from concurrent.futures import ThreadPoolExecutor

# 지원할 이미지 확장자 목록
IMAGE_EXTENSIONS = [".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".gif", ".webp"]


def copy_file(image_folder, label_filename, output_folder):
    # 레이블 파일 이름에서 확장자 제거
    base_filename = os.path.splitext(label_filename)[0]

    # 여러 확장자에 대해 이미지 파일 검색
    for ext in IMAGE_EXTENSIONS:
        image_path = os.path.join(image_folder, base_filename + ext)
        if os.path.exists(image_path):
            # 이미지가 존재하면 복사
            shutil.copy(image_path, output_folder)
            print(f"Copied: {label_filename} -> {image_path}")
            return  # 복사 성공하면 루프 종료

    print(f"No matching image found for: {label_filename}")


def copy_images_based_on_labels(image_folder, label_folder, output_folder, max_workers=8):
    # output 폴더가 없으면 생성
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 레이블 폴더에서 파일 리스트 불러오기
    label_files = os.listdir(label_folder)

    # ThreadPoolExecutor를 이용해 멀티스레딩 처리
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        for label_file in label_files:
            executor.submit(copy_file, image_folder, label_file, output_folder)


if __name__ == "__main__":
    # 경로 설정
    image_folder = r"C:\Users\JM\Desktop\class_0"  # 이미지 폴더 경로
    label_folder = r"C:\Users\JM\Downloads\job_179-2024_10_16_02_34_57-yolo 1.1\obj_train_data"  # 레이블 폴더 경로
    output_folder = r"C:\Users\JM\Desktop\output_class0"  # 복사할 output 폴더 경로

    # 복사 작업 실행
    copy_images_based_on_labels(image_folder, label_folder, output_folder)
