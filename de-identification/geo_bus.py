import os
import cv2
import queue
import threading
import gc
import time
from ultralytics import YOLO
from concurrent.futures import ThreadPoolExecutor


# 블러 처리를 위한 함수
def blur_area(image, x_min, y_min, x_max, y_max, blur_strength=15):
    h, w, _ = image.shape
    x_min = max(0, x_min)
    y_min = max(0, y_min)
    x_max = min(w, x_max)
    y_max = min(h, y_max)
    region = image[y_min:y_max, x_min:x_max]
    blur_strength = max(1, blur_strength // 2 * 2 + 1)
    blurred = cv2.GaussianBlur(region, (blur_strength, blur_strength), 0)
    image[y_min:y_max, x_min:x_max] = blurred
    return image


# 이미지 저장을 위한 함수
def save_image(image, output_path):
    file_ext = os.path.splitext(output_path)[1].lower()
    if file_ext == '.jpg' or file_ext == '.jpeg':
        cv2.imwrite(output_path, image, [cv2.IMWRITE_JPEG_QUALITY, 85])
    elif file_ext == '.png':
        cv2.imwrite(output_path, image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
    else:
        cv2.imwrite(output_path, image)
    print(f"이미지가 {output_path}에 저장되었습니다.")


# 멀티스레드로 블러 처리 함수 (큐에서 값을 가져와 처리)
def blur_processing_worker(q, model1, model2, blur_strength):
    while True:
        item = q.get()
        if item is None:
            break  # None이 들어오면 작업 종료

        result, output_folder = item
        image_file = result.path

        # 큐에서 꺼낸 값을 출력
        print(f"큐에서 꺼낸 값: {image_file} (폴더: {output_folder})")

        # 이미지 파일 읽기
        image = cv2.imread(image_file)
        if image is None:
            print(f"이미지 {image_file}를 불러올 수 없습니다.")
            continue

        # 첫 번째 모델로 바운딩 박스를 이용한 블러 처리
        print(f"첫 번째 모델 결과로 블러 처리 중: {image_file}")
        for box in result.boxes:
            x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
            image = blur_area(image, x_min, y_min, x_max, y_max, blur_strength)

        # 첫 번째 모델로 블러 처리 후 두 번째 모델에 입력
        print(f"두 번째 모델 처리 시작: {image_file}")
        results2 = model2(image, conf=0.7)

        # 두 번째 모델로 바운딩 박스를 이용한 블러 처리
        print(f"두 번째 모델 결과로 블러 처리 중: {image_file}")
        for box in results2.boxes:
            x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
            image = blur_area(image, x_min, y_min, x_max, y_max, blur_strength)

        # 출력 경로 설정 및 저장
        output_path = os.path.join(output_folder, os.path.basename(image_file))
        save_image(image, output_path)
        print(f"블러 처리된 이미지 저장 완료: {output_path}")

        # 처리 후 메모리 해제
        del image  # 이미지 객체 삭제
        gc.collect()  # 명시적으로 가비지 컬렉터 호출

        q.task_done()


# YOLO 모델의 결과를 큐에 넣는 함수
def process_folder_with_model(model, folder, q):
    print(f"폴더 {folder} 경로를 모델에 넣습니다.")

    # 폴더 내의 이미지 파일들을 찾음
    image_files = [os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(('.png', '.jpg', '.jpeg'))]

    if not image_files:
        print(f"폴더 {folder} 안에 처리할 이미지가 없습니다.")
        return

    # YOLO 모델에 각 이미지 파일 전달
    for image_file in image_files:
        print(f"이미지 처리 중: {image_file}")

        # 이미지 로딩 확인
        image = cv2.imread(image_file)
        if image is None:
            print(f"이미지 로드 실패: {image_file}")
            continue

        start_time = time.time()  # 처리 시작 시간 기록

        # 모델로 이미지 처리
        try:
            results = model(image_file)
            print(f"이미지 처리 완료: {image_file}")
        except Exception as e:
            print(f"모델 예측 중 오류 발생: {e}")
            continue

        end_time = time.time()  # 처리 종료 시간 기록
        print(f"처리 시간: {end_time - start_time:.2f}초")

        # 결과를 큐에 저장
        output_folder = os.path.join(folder, 'output3')
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
        print(f"큐에 넣기: {image_file} (폴더: {output_folder})")
        q.put((results[0], output_folder))

    # 처리 후 결과 객체 삭제 및 메모리 해제
    del results
    gc.collect()


# 재귀적으로 폴더를 검색하고, 조건에 맞는 폴더 경로를 리스트로 반환하는 함수
def collect_folders_with_condition(input_folder):
    matching_folders = []
    for root, dirs, files in os.walk(input_folder):
        if 'output' not in root and any(cam in root for cam in ['Camera01', 'Camera02', 'Camera03', 'Camera04']):
            matching_folders.append(root)
            print(f"조건에 맞는 폴더 발견: {root}")
    print(matching_folders)
    return matching_folders


# 메인 함수
def main(input_folder, model1, model2, blur_strength=25, num_threads=None):
    if num_threads is None:
        num_threads = os.cpu_count()  # 사용 가능한 최대 스레드 수

    # 전체 소요 시간 측정을 위한 시작 시간
    total_start_time = time.time()

    # 큐 생성 (버퍼 역할)
    q = queue.Queue()

    # 멀티스레드로 블러 처리 작업 시작
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        for _ in range(num_threads):
            executor.submit(blur_processing_worker, q, model1, model2, blur_strength)

        # 조건에 맞는 폴더들을 싱글 스레드에서 수집
        folders = collect_folders_with_condition(input_folder)

        # YOLO 모델로 각 폴더를 처리하고, 결과를 큐에 넣음 (싱글 스레드)
        for folder in folders:
            process_folder_with_model(model1, folder, q)

    # 작업이 모두 끝난 후, None을 넣어 스레드 종료
    for _ in range(num_threads):
        q.put(None)

    # 모든 작업이 끝날 때까지 대기
    q.join()

    # 전체 소요 시간 계산
    total_end_time = time.time()
    print(f"전체 소요 시간: {total_end_time - total_start_time:.2f}초")

    # 모든 작업 후 가비지 컬렉터 호출
    gc.collect()


# YOLOv8 모델 불러오기
model1 = YOLO('/home/geon_lab/AI_PARK/de-identifcation/model/de-identification_best.pt')
model2 = YOLO('/home/geon_lab/AI_PARK/de-identifcation/model/wild_best.pt')

# 최상단 입력 폴더 경로
input_folder = '/home/geon_lab/AI_PARK/de-identifcation/test'  # 최상단 폴더 경로로 변경

# 메인 함수 실행
main(input_folder, model1, model2, blur_strength=200, num_threads=8)
