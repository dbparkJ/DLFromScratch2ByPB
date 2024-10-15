import os
import cv2
import numpy as np
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor

# 클래스 및 색상 정의
class_names = ["pothole", "normal_traffic regulation", "unnomal_traffic regulation", "roadkill", "falling_object"]
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]


def find_image_path(label_path, image_folder):
    """
    레이블 파일과 일치하는 이미지 파일을 검색하고 경로를 반환합니다.
    """
    # 확장자 제거한 파일 이름
    base_filename = os.path.splitext(os.path.basename(label_path))[0]

    # 이미지 파일 경로를 탐색하면서 이름이 같은 파일 찾기
    for root, _, files in os.walk(image_folder):
        for file in files:
            # 이미지 파일 확장자 확인
            image_name, ext = os.path.splitext(file)
            if image_name == base_filename:
                return os.path.join(root, file)

    return None


def process_image(label_path, image_folder):
    # 이미지 파일 경로 찾기
    image_path = find_image_path(label_path, image_folder)

    if image_path is None:
        print(f"Image file not found for label: {label_path}")
        return

    print(f"Processing image: {image_path}")
    image = cv2.imread(image_path)
    height, width, _ = image.shape

    with open(label_path, 'r') as f:
        for line in f:
            try:
                data = line.strip().split()
                if len(data) < 5:
                    print(f"Skipping malformed line in {label_path}: {line}")
                    continue  # 데이터가 부족할 경우 스킵

                # 데이터 형식: class_id, center_x, center_y, width, height
                class_id = int(data[0])
                x_center, y_center, box_width, box_height = map(float, data[1:])

                # 이미지 크기 기준으로 절대 좌표로 변환
                x_center = int(x_center * width)
                y_center = int(y_center * height)
                box_width = int(box_width * width)
                box_height = int(box_height * height)

                # 바운딩 박스의 왼쪽 위와 오른쪽 아래 좌표 계산
                x1 = int(x_center - (box_width / 2))
                y1 = int(y_center - (box_height / 2))
                x2 = int(x_center + (box_width / 2))
                y2 = int(y_center + (box_height / 2))

                # 박스 그리기
                color = colors[class_id]
                class_name = class_names[class_id]
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(image, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            except ValueError as e:
                print(f"Skipping malformed line in {label_path}: {line} (Error: {e})")
            except IndexError as e:
                print(f"Skipping line due to invalid class index in {label_path}: {line} (Error: {e})")

    # 결과 저장을 위한 폴더 생성
    result_folder = os.path.join(os.path.dirname(image_path), 'result')
    os.makedirs(result_folder, exist_ok=True)

    # 결과 이미지 저장
    result_image_path = os.path.join(result_folder, os.path.basename(image_path))
    cv2.imwrite(result_image_path, image)
    print(f"Saved: {result_image_path}")


def draw_bounding_boxes(image_folder, label_folder, num_threads):
    # 모든 레이블 파일 수집을 위한 리스트
    label_files = []

    # 재귀적으로 label_folder 내의 모든 .txt 파일 수집
    for root, _, files in os.walk(label_folder):
        for file in files:
            if file.endswith('.txt'):
                label_files.append(os.path.join(root, file))

    # 수집된 레이블 파일 경로 출력 (디버그용)
    print(f"Found {len(label_files)} label files in '{label_folder}'")
    for label_file in label_files:
        print(f"Label file: {label_file}")

    # 멀티쓰레드로 이미지 처리
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = [executor.submit(process_image, label_file, image_folder) for label_file in label_files]
        for future in futures:
            try:
                future.result()  # 결과를 기다림으로써 예외 처리
            except Exception as e:
                print(f"Error processing file: {e}")


def count_classes_and_files(folder_path):
    txt_count = 0
    class_count = defaultdict(int)

    # 재귀적으로 폴더 탐색
    for root, _, files in os.walk(folder_path):
        for file in files:
            if file.endswith(".txt"):
                txt_count += 1
                file_path = os.path.join(root, file)
                with open(file_path, 'r') as f:
                    lines = f.readlines()
                    for line in lines:
                        try:
                            class_index = int(line.split()[0])  # 첫 번째 숫자가 class를 나타냄
                            class_count[class_names[class_index]] += 1  # 해당 class의 count 증가
                        except (ValueError, IndexError) as e:
                            print(f"Skipping malformed line in {file_path}: {line} (Error: {e})")

    return txt_count, class_count


def main():
    image_folder = r"D:\images\images"  # 이미지 폴더 경로
    label_folder = r"D:\images\labels"  # 레이블 폴더 경로
    num_threads = int(input("Enter the number of threads: "))  # 사용자로부터 스레드 수 입력받기

    # 클래스와 파일 개수 세기
    txt_count, class_count = count_classes_and_files(label_folder)
    print(f"Total .txt files: {txt_count}")
    for class_name, count in class_count.items():
        print(f"Class '{class_name}': {count} instances")

    # 바운딩 박스 그리기
    draw_bounding_boxes(image_folder, label_folder, num_threads)


if __name__ == "__main__":
    main()
