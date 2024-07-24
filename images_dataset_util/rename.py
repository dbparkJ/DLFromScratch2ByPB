import os
import shutil
import zipfile


def rename_files_and_log(source_directory, target_directory, zip_filename):
    # Initialize a counter for numbering
    number = 1

    # Create the target directory if it does not exist
    if not os.path.exists(target_directory):
        os.makedirs(target_directory)

    # Create or open the train.txt file for logging in the target directory
    log_file_path = os.path.join(target_directory, 'train.txt')
    with open(log_file_path, 'w') as log_file:
        # Create a list of file names without extensions
        base_names = set()
        for filename in os.listdir(source_directory):
            name, ext = os.path.splitext(filename)
            if ext in ['.png', '.txt']:
                base_names.add(name)

        # Process each base name
        for base_name in base_names:
            png_file = base_name + '.png'
            txt_file = base_name + '.txt'
            new_base_name = f"AI_Hub_pothole_4_{number}"

            # Copy and rename .png file if it exists
            if os.path.exists(os.path.join(source_directory, png_file)):
                new_png_name = new_base_name + '.png'
                shutil.copy(os.path.join(source_directory, png_file), os.path.join(target_directory, new_png_name))
                log_file.write(new_png_name + '\n')

            # Copy and rename .txt file if it exists
            if os.path.exists(os.path.join(source_directory, txt_file)):
                new_txt_name = new_base_name + '.txt'
                shutil.copy(os.path.join(source_directory, txt_file), os.path.join(target_directory, new_txt_name))

            # Increment the counter
            number += 1

    # Create a zip file with the specified name and add all .png files
    with zipfile.ZipFile(os.path.join(target_directory, zip_filename), 'w') as zipf:
        for filename in os.listdir(target_directory):
            if filename.endswith('.png'):
                zipf.write(os.path.join(target_directory, filename), filename)


# Example usage
source_directory_path = r'D:\포트홀2'  # 원본 폴더 경로를 실제 경로로 변경하세요
target_directory_path = r'D:\포트홀2\ouput'  # 대상 폴더 경로를 실제 경로로 변경하세요
zip_filename = 'portholl_AI_Hub_4.zip'  # 압축 파일명을 원하는 대로 변경하세요
rename_files_and_log(source_directory_path, target_directory_path, zip_filename)
