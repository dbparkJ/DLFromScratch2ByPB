import os
from PIL import Image, ImageOps
from concurrent.futures import ThreadPoolExecutor, as_completed

# 입력 폴더와 출력 폴더 경로 설정
input_folder_path = r"D:\2024.10.02"  # 입력 폴더 경로를 지정하세요.
output_folder_path = r"D:\2024.10.02\output"  # 출력 폴더 경로를 지정하세요.

# 출력 폴더가 없으면 생성
os.makedirs(output_folder_path, exist_ok=True)

# 기준 크기 설정
target_width = 1920
target_height = 1080

# 지원하는 이미지 파일 확장자 목록
image_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".gif")

# CPU 코어의 절반을 스레드 수로 사용
max_threads = os.cpu_count() // 2


def process_image(file_path, output_dir):
    """이미지 리사이즈 및 패딩 처리"""
    image = Image.open(file_path)
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

    # 파일 이름과 확장자 분리
    file_name = os.path.basename(file_path)
    output_image_path = os.path.join(output_dir, file_name.rsplit(".", 1)[0] + "_resized." + file_name.rsplit(".", 1)[1])

    # 새로운 이미지 저장
    padded_image.save(output_image_path)
    print(f"이미지가 성공적으로 리사이즈 및 저장되었습니다: {file_name}")


def process_labels(file_path, output_dir):
    """레이블 좌표 리사이즈 처리 및 필터링"""
    label_path = file_path.rsplit(".", 1)[0] + ".txt"
    output_image_path = os.path.join(output_dir, os.path.basename(file_path).rsplit(".", 1)[0] + "_resized." + file_path.rsplit(".", 1)[1])
    output_label_path = os.path.join(output_dir, os.path.basename(file_path).rsplit(".", 1)[0] + "_resized.txt")

    if not os.path.exists(label_path):
        print(f"레이블 파일이 없습니다: {label_path}")
        return

    try:
        # 이미지 열기 및 처리
        image = Image.open(file_path)
        original_width, original_height = image.size

        # 새로운 이미지 크기 계산
        ratio_w = target_width / original_width
        ratio_h = target_height / original_height

        if ratio_w < ratio_h:
            new_width = target_width
            new_height = int(original_height * ratio_w)
        else:
            new_width = int(original_width * ratio_h)
            new_height = target_height

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

            # 박스 크기가 15픽셀 미만이면 제외
            if box_width * target_width >= 15 and box_height * target_height >= 15:
                new_labels.append(f"{class_name} {x_center:.6f} {y_center:.6f} {box_width:.6f} {box_height:.6f}\n")

        # 유효한 레이블이 없으면 삭제
        if new_labels:
            # 새로운 라벨 저장
            with open(output_label_path, 'w') as file:
                file.writelines(new_labels)

            print(f"레이블이 성공적으로 리사이즈 및 저장되었습니다: {file_path}")
        else:
            os.remove(file_path)
            if os.path.exists(label_path):
                os.remove(label_path)
            if os.path.exists(output_image_path):
                os.remove(output_image_path)
            if os.path.exists(output_label_path):
                os.remove(output_label_path)
            print(f"유효한 레이블이 없어 파일을 삭제했습니다: {file_path}")

    except Exception as e:
        print(f"이미지 처리 중 오류가 발생했습니다: {file_path}, 오류: {e}")
    finally:
        image.close()


def process_file(file_path, output_dir, choice):
    """사용자가 선택한 처리(이미지, 레이블 또는 둘 다)를 수행"""
    if choice == "1":
        process_image(file_path, output_dir)
    elif choice == "2":
        process_labels(file_path, output_dir)
    elif choice == "3":
        process_image(file_path, output_dir)
        process_labels(file_path, output_dir)


def main():
    print("1: 이미지 처리")
    print("2: 레이블 처리")
    print("3: 이미지와 레이블 모두 처리")
    choice = input("실행할 작업을 선택하세요 (1/2/3): ")

    # 재귀적으로 하위 폴더 내 파일을 처리
    with ThreadPoolExecutor(max_threads) as executor:
        futures = []
        for root, dirs, files in os.walk(input_folder_path):
            relative_path = os.path.relpath(root, input_folder_path)
            output_subfolder = os.path.join(output_folder_path, relative_path)

            # 출력 폴더 생성
            os.makedirs(output_subfolder, exist_ok=True)

            for file_name in files:
                if file_name.lower().endswith(image_extensions):
                    file_path = os.path.join(root, file_name)
                    futures.append(executor.submit(process_file, file_path, output_subfolder, choice))

        # 작업 완료를 기다리고 결과 처리
        for future in as_completed(futures):
            future.result()


if __name__ == "__main__":
    main()
