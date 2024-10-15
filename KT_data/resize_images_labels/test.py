# import os
#
# # Directory containing the files
# directory = r"C:\Users\JM\Downloads\traffic_regulation_1\obj_train_data\obj_train_data\traffic_regulation_01\output"
#
# # Loop through each file in the directory
# for filename in os.listdir(directory):
#     # Check if the file has the '_resized' suffix
#     if "_resized" in filename:
#         # Create the new filename by removing the '_resized' suffix
#         new_filename = filename.replace("_resized", "")
#         # Get the full paths for the current and new filenames
#         old_file = os.path.join(directory, filename)
#         new_file = os.path.join(directory, new_filename)
#         # Rename the file
#         os.rename(old_file, new_file)
#
# print("File renaming completed.")

import os
import shutil

def rename_and_copy_files(input_folder, output_folder):
    # 입력 폴더를 재귀적으로 탐색
    for root, dirs, files in os.walk(input_folder):
        # 현재 폴더의 이름을 가져옴
        current_folder_name = os.path.basename(root)

        # 모든 파일에 대해 처리
        for file_name in files:
            # 새로운 파일 이름 생성 (폴더명_파일명)
            new_file_name = f"{current_folder_name}_{file_name}"

            # 원본 파일의 전체 경로
            original_file_path = os.path.join(root, file_name)

            # 출력 폴더에서 저장될 파일의 전체 경로
            output_file_path = os.path.join(output_folder, new_file_name)

            # 출력 폴더가 없으면 생성
            if not os.path.exists(output_folder):
                os.makedirs(output_folder)

            # 파일 복사 및 이름 변경
            shutil.copy2(original_file_path, output_file_path)

            print(f"Copied: {original_file_path} -> {output_file_path}")

# 사용자로부터 입력 폴더와 출력 폴더 경로를 입력받음
input_folder = r"C:\Users\JM\Desktop\extract_images"
output_folder = r"C:\Users\JM\Desktop\extract_images\output"

# 파일 이름 변경 및 복사
rename_and_copy_files(input_folder, output_folder)

print("All files have been renamed and copied successfully.")
