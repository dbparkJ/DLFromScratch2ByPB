import os
from PIL import Image, ImageOps

# 입력 폴더와 출력 폴더 경로 설정
input_folder_path = r"C:\Users\JM\Downloads\roadkill_1\obj_train_data\01_finish"  # 입력 폴더 경로를 지정하세요.
output_folder_path = r"C:\Users\JM\Downloads\roadkill_1\obj_train_data\01_finish\output"  # 출력 폴더 경로를 지정하세요.

# 출력 폴더가 없으면 생성
os.makedirs(output_folder_path, exist_ok=True)

# 기준 크기 설정
target_width = 1920
target_height = 1080

# 지원하는 이미지 파일 확장자 목록
image_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".gif")

# 입력 폴더 내의 모든 파일을 순회하며 처리
for file_name in os.listdir(input_folder_path):
    if file_name.lower().endswith(image_extensions):  # 지원하는 이미지 파일만 선택
        image_path = os.path.join(input_folder_path, file_name)
        label_path = os.path.join(input_folder_path, file_name.rsplit(".", 1)[0] + ".txt")

        if not os.path.exists(label_path):
            print(f"레이블 파일이 없습니다: {label_path}")
            continue

        # 이미지 열기
        image = Image.open(image_path)
        original_width, original_height = image.size

        # 비율 계산
        ratio_w = target_width / original_width
        ratio_h = target_height / original_height

        # 큰 쪽을 기준으로 리사이즈
        if ratio_w < ratio_h:
            new_width = target_width
            new_height = int(original_height * ratio_w)
        else:
            new_width = int(original_width * ratio_h)
            new_height = target_height

        # 리사이즈
        resized_image = image.resize((new_width, new_height), Image.LANCZOS)

        # 검정색 배경으로 이미지 패딩 추가
        padded_image = ImageOps.pad(resized_image, (target_width, target_height), color=(0, 0, 0))

        # 모든 라벨 읽기
        with open(label_path, 'r') as file:
            labels = file.readlines()

        new_labels = []
        for label in labels:
            label_data = label.strip().split()
            class_name = int(label_data[0])
            x_center = float(label_data[1]) * original_width
            y_center = float(label_data[2]) * original_height
            box_width = float(label_data[3]) * original_width
            box_height = float(label_data[4]) * original_height

            # 새로운 좌표 계산
            x_center = (x_center * (new_width / original_width) + (target_width - new_width) / 2) / target_width
            y_center = (y_center * (new_height / original_height) + (target_height - new_height) / 2) / target_height
            box_width = (box_width * new_width / original_width) / target_width
            box_height = (box_height * new_height / original_height) / target_height

            # 새로운 라벨 리스트에 추가
            new_labels.append(f"{class_name} {x_center:.6f} {y_center:.6f} {box_width:.6f} {box_height:.6f}\n")

        # 새로운 라벨 저장
        output_label_path = os.path.join(output_folder_path, file_name.rsplit(".", 1)[0] + "_resized.txt")
        with open(output_label_path, 'w') as file:
            file.writelines(new_labels)

        # 새로운 이미지 저장
        output_image_path = os.path.join(output_folder_path, file_name.rsplit(".", 1)[0] + "_resized." + file_name.rsplit(".", 1)[1])
        padded_image.save(output_image_path)

        print(f"이미지와 레이블이 성공적으로 리사이즈 및 저장되었습니다: {file_name}")
