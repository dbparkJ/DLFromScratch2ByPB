import os
import shutil
from PIL import Image


def get_image_info(image_path):
    with Image.open(image_path) as img:
        return img.size, os.path.getsize(image_path)


def copy_images(source_folder, destination_folder):
    # 지원하는 이미지 확장자 목록
    image_extensions = ('.jpg', '.jpeg', '.png', '.bmp', '.tiff')

    # 대상 폴더가 존재하지 않으면 생성
    if not os.path.exists(destination_folder):
        os.makedirs(destination_folder)

    # 이미지 정보 저장을 위한 딕셔너리
    copied_images = {}
    file_counter = 1

    # 모든 파일과 폴더 순회
    for root, dirs, files in os.walk(source_folder):
        for file in files:
            if file.lower().endswith(image_extensions):
                source_path = os.path.join(root, file)
                image_info = get_image_info(source_path)

                if image_info not in copied_images.values():
                    # 중복되지 않는 경우에만 복사
                    destination_filename = f"{file_counter:04d}{os.path.splitext(file)[1]}"
                    destination_path = os.path.join(destination_folder, destination_filename)
                    shutil.copy2(source_path, destination_path)
                    copied_images[destination_filename] = image_info
                    print(f"Copied {source_path} to {destination_path}")
                    file_counter += 1
                else:
                    print(f"Skipped {source_path} (duplicate)")


source_folder = r'D:\1)낙하물'
destination_folder = r'D:\02_finish'

copy_images(source_folder, destination_folder)
