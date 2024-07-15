import os
import random
import shutil
from pathlib import Path


def copy_random_files(input_folder, output_image_folder, output_txt_folder, train_file_path, num_files=461):
    # 입력 폴더의 경로 설정
    input_folder = Path(input_folder)
    output_image_folder = Path(output_image_folder)
    output_txt_folder = Path(output_txt_folder)
    train_file_path = Path(train_file_path)

    # 출력 폴더가 없으면 생성
    output_image_folder.mkdir(parents=True, exist_ok=True)
    output_txt_folder.mkdir(parents=True, exist_ok=True)

    # 폴더 내 파일 목록 가져오기
    all_files = list(input_folder.glob('*'))

    # 이미지 파일과 텍스트 파일 분리
    image_files = [f for f in all_files if f.suffix.lower() in ['.jpg', '.jpeg', '.png', '.bmp', '.gif', '.png']]
    txt_files = {f.stem: f for f in all_files if f.suffix.lower() == '.txt'}

    # 이미지 파일이 부족하면 예외 발생
    if len(image_files) < num_files:
        raise ValueError(f"Not enough image files in the directory. Required: {num_files}, Found: {len(image_files)}")

    # 랜덤으로 이미지 파일 선택
    selected_images = random.sample(image_files, num_files)

    # 이미지 파일 이름을 저장할 리스트 초기화
    image_filenames = []

    # 이미지 파일과 같은 이름의 텍스트 파일 복사
    for img_file in selected_images:
        # 이미지 파일 복사
        shutil.copy(img_file, output_image_folder / img_file.name)
        image_filenames.append(img_file.name)

        # 같은 이름의 텍스트 파일이 존재하면 복사
        if img_file.stem in txt_files:
            txt_file = txt_files[img_file.stem]
            shutil.copy(txt_file, output_txt_folder / txt_file.name)

    # train.txt 파일에 이미지 파일 이름 저장
    with open(train_file_path, 'w') as train_file:
        for filename in image_filenames:
            train_file.write(f"{filename}\n")


# 사용 예시
input_folder = r'D:\val'
output_image_folder = r'D:\val\images'
output_txt_folder = r'D:\val\labels'
train_file_path = r'D:\val\train.txt'

copy_random_files(input_folder, output_image_folder, output_txt_folder, train_file_path)
