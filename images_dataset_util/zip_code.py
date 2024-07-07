import os
import zipfile
import glob


def zip_images(input_folder, output_folder, batch_size=500):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    image_files = glob.glob(os.path.join(input_folder, '*.*'))  # 모든 파일을 가져옴
    total_images = len(image_files)

    for i in range(0, total_images, batch_size):
        batch_files = image_files[i:i + batch_size]
        batch_number = i // batch_size + 1
        zip_file_name = os.path.join(output_folder, f'batch_{batch_number}.zip')

        with zipfile.ZipFile(zip_file_name, 'w') as zipf:
            for file in batch_files:
                zipf.write(file, os.path.basename(file))

        print(f'Batch {batch_number} zipped with {len(batch_files)} images.')


# 사용 예시
input_folder = r'D:\roadkill2'
output_folder = r'D:\03_finish'
zip_images(input_folder, output_folder)
