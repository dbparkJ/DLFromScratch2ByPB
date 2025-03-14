import os
import shutil


def move_and_rename_files(input_folder, output_folder):
    # 입력 폴더 재귀 탐색
    for root, dirs, files in os.walk(input_folder):
        # 현재 폴더명 추출
        current_folder_name = os.path.basename(root)

        # 파일 이동 및 이름 변경
        for file_name in files:
            # 새 파일명 설정 (폴더명_파일이름)
            new_file_name = f"{current_folder_name}_{file_name}"

            # 파일 경로 설정
            source_path = os.path.join(root, file_name)
            destination_path = os.path.join(output_folder, new_file_name)

            # 파일 복사
            shutil.copy2(source_path, destination_path)
            print(f"Copied: {source_path} -> {destination_path}")


# 사용 예시
input_folder = r'C:\Users\JMP\Desktop\data\4.1'
output_folder = r'C:\Users\JMP\Desktop\data\data'

move_and_rename_files(input_folder, output_folder)
