import os
import random
import shutil

# 데이터셋의 최상위 경로 설정
root_path = r"/KT_data/seperate_code/original_data"  # 전체 폴더가 있는 경로로 변경

# train/valid/test 폴더 경로 설정
train_img_dir = os.path.join(root_path, 'images/train')
valid_img_dir = os.path.join(root_path, 'images/valid')
test_img_dir = os.path.join(root_path, 'images/test')
train_lbl_dir = os.path.join(root_path, 'labels/train')
valid_lbl_dir = os.path.join(root_path, 'labels/valid')
test_lbl_dir = os.path.join(root_path, 'labels/test')

# log 파일 경로 설정
log_file_path = os.path.join(root_path, 'log.txt')

# train/valid/test 폴더 생성
os.makedirs(train_img_dir, exist_ok=True)
os.makedirs(valid_img_dir, exist_ok=True)
os.makedirs(test_img_dir, exist_ok=True)
os.makedirs(train_lbl_dir, exist_ok=True)
os.makedirs(valid_lbl_dir, exist_ok=True)
os.makedirs(test_lbl_dir, exist_ok=True)

# 지원하는 이미지 확장자 리스트
supported_image_formats = ('.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.gif')


# 고유 파일 이름을 생성하는 함수
def get_new_filename(index):
    return f"{index:06d}"


# 새로운 파일명 부여를 위한 인덱스
file_index = 1

# 각 클래스별로 개수를 저장할 딕셔너리
train_class_count = {}
valid_class_count = {}
test_class_count = {}


# 각 폴더에서 클래스 개수 세는 함수
def count_classes_in_labels(label_dir, class_count_dict):
    for label_file in os.listdir(label_dir):
        label_path = os.path.join(label_dir, label_file)

        # 라벨 파일 읽기
        with open(label_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                # 클래스 ID 가져오기 (첫 번째 항목)
                class_id = line.split()[0]

                # 클래스 개수 업데이트
                if class_id in class_count_dict:
                    class_count_dict[class_id] += 1
                else:
                    class_count_dict[class_id] = 1


# 각 하위 폴더에 대해 처리
for sub_folder in os.listdir(root_path):
    sub_folder_path = os.path.join(root_path, sub_folder)

    if os.path.isdir(sub_folder_path):
        # 이미지와 라벨 파일 목록 가져오기
        images = [f for f in os.listdir(sub_folder_path) if f.lower().endswith(supported_image_formats)]
        labels = [f for f in os.listdir(sub_folder_path) if f.endswith('.txt')]

        # 이미지와 라벨 파일이 존재하는지 확인하고 페어로 리스트 생성
        image_label_pairs = [(img, img.rsplit('.', 1)[0] + '.txt') for img in images if
                             img.rsplit('.', 1)[0] + '.txt' in labels]

        # 리스트가 비어있지 않은 경우에만 처리
        if len(image_label_pairs) > 0:
            # 데이터셋을 랜덤하게 섞기
            random.shuffle(image_label_pairs)

            # split_ratio에 따라 train/valid/test로 분할
            train_ratio = 0.7
            valid_ratio = 0.2
            train_split_index = int(len(image_label_pairs) * train_ratio)
            valid_split_index = train_split_index + int(len(image_label_pairs) * valid_ratio)

            train_pairs = image_label_pairs[:train_split_index]
            valid_pairs = image_label_pairs[train_split_index:valid_split_index]
            test_pairs = image_label_pairs[valid_split_index:]

            # train 데이터 이동
            for img_name, lbl_name in train_pairs:
                img_src = os.path.join(sub_folder_path, img_name)
                lbl_src = os.path.join(sub_folder_path, lbl_name)

                # 새로운 파일명 생성
                new_filename = get_new_filename(file_index)
                file_index += 1

                # 새로운 파일 경로 설정
                img_ext = os.path.splitext(img_name)[1]
                new_img_path = os.path.join(train_img_dir, new_filename + img_ext)
                new_lbl_path = os.path.join(train_lbl_dir, new_filename + '.txt')

                # 파일 이동
                shutil.move(img_src, new_img_path)
                shutil.move(lbl_src, new_lbl_path)

            # valid 데이터 이동
            for img_name, lbl_name in valid_pairs:
                img_src = os.path.join(sub_folder_path, img_name)
                lbl_src = os.path.join(sub_folder_path, lbl_name)

                # 새로운 파일명 생성
                new_filename = get_new_filename(file_index)
                file_index += 1

                # 새로운 파일 경로 설정
                img_ext = os.path.splitext(img_name)[1]
                new_img_path = os.path.join(valid_img_dir, new_filename + img_ext)
                new_lbl_path = os.path.join(valid_lbl_dir, new_filename + '.txt')

                # 파일 이동
                shutil.move(img_src, new_img_path)
                shutil.move(lbl_src, new_lbl_path)

            # test 데이터 이동
            for img_name, lbl_name in test_pairs:
                img_src = os.path.join(sub_folder_path, img_name)
                lbl_src = os.path.join(sub_folder_path, lbl_name)

                # 새로운 파일명 생성
                new_filename = get_new_filename(file_index)
                file_index += 1

                # 새로운 파일 경로 설정
                img_ext = os.path.splitext(img_name)[1]
                new_img_path = os.path.join(test_img_dir, new_filename + img_ext)
                new_lbl_path = os.path.join(test_lbl_dir, new_filename + '.txt')

                # 파일 이동
                shutil.move(img_src, new_img_path)
                shutil.move(lbl_src, new_lbl_path)

# 각 폴더에서 클래스 개수 세기
count_classes_in_labels(train_lbl_dir, train_class_count)
count_classes_in_labels(valid_lbl_dir, valid_class_count)
count_classes_in_labels(test_lbl_dir, test_class_count)

# 클래스 개수를 log.txt 파일에 기록 (클래스 ID 순서로 정렬하여 기록)
with open(log_file_path, 'w') as log_file:
    log_file.write("Train 폴더 클래스 개수 (클래스 ID 기준 정렬):\n")
    for class_id, count in sorted(train_class_count.items()):
        log_file.write(f"Class {class_id}: {count} 개\n")

    log_file.write("\nValid 폴더 클래스 개수 (클래스 ID 기준 정렬):\n")
    for class_id, count in sorted(valid_class_count.items()):
        log_file.write(f"Class {class_id}: {count} 개\n")

    log_file.write("\nTest 폴더 클래스 개수 (클래스 ID 기준 정렬):\n")
    for class_id, count in sorted(test_class_count.items()):
        log_file.write(f"Class {class_id}: {count} 개\n")

print(f"각 폴더별로 파일이 train/valid/test로 7:2:1 비율로 분할되고, 새로운 이름으로 변경되었습니다! 로그 파일이 {log_file_path}에 생성되었습니다.")
