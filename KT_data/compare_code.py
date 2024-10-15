import os


def get_file_list(directory, extensions):
    """
    주어진 디렉토리 내에서 특정 확장자를 가진 파일들을 재귀적으로 검색하여 리스트로 반환
    """
    file_list = []
    for root, _, files in os.walk(directory):
        for file in files:
            if any(file.lower().endswith(ext) for ext in extensions):  # 확장자를 소문자로 비교
                file_list.append(os.path.join(root, file))
    return file_list


def get_label_files(base_folder):
    """
    base_folder 내에서 'obj_train_data' 폴더를 찾아 그 하위의 모든 레이블 파일(.txt)을 리스트로 반환
    """
    label_files = []

    # 재귀적으로 폴더 탐색
    for root, dirs, files in os.walk(base_folder):
        # 각 디렉토리 내에 'obj_train_data' 폴더가 있는지 확인
        if 'obj_train_data' in dirs:
            obj_train_data_path = os.path.join(root, 'obj_train_data')
            for sub_root, _, sub_files in os.walk(obj_train_data_path):
                for file in sub_files:
                    if file.endswith(".txt"):
                        file_path = os.path.join(sub_root, file)
                        label_files.append(file_path)

    return label_files


def get_file_name_without_extension(file_path):
    """
    파일 경로에서 확장자를 제외한 파일명만 반환 (소문자로 변환하여 반환)
    """
    return os.path.splitext(os.path.basename(file_path))[0].lower()


def delete_images_without_labels(image_folder, label_folder, image_extensions):
    """
    이미지 파일과 레이블 파일을 비교하여 레이블이 없는 이미지 파일 삭제
    """
    # 이미지 파일 목록 가져오기
    image_files = get_file_list(image_folder, image_extensions)

    # 레이블 파일 목록 가져오기
    label_files = get_label_files(label_folder)

    # 레이블 파일의 이름만 가져오기 (확장자 제외, 소문자)
    label_names = {get_file_name_without_extension(label) for label in label_files}

    # 레이블이 없는 이미지 파일 삭제
    for image in image_files:
        image_name = get_file_name_without_extension(image)
        if image_name in label_names:
            print(f"Label found for: {image_name}")
        else:
            print(f"No label found for: {image_name} - Deleting {image}")
            os.remove(image)


# 사용 예시
image_folder = r'D:\images\images'  # 이미지 폴더 경로
label_folder = r'D:\images\labels'  # 레이블 폴더 경로 (obj_train_data 상위 폴더)

image_extensions = [
    '.jpg', '.jpeg', '.png', '.bmp', '.gif', '.tiff',
    '.webp', '.svg', '.heic', '.raw', '.dng', '.cr2',
    '.nef', '.orf', '.sr2'
]  # 다양한 이미지 파일 확장자 리스트

delete_images_without_labels(image_folder, label_folder, image_extensions)

