import os
import zipfile
from concurrent.futures import ThreadPoolExecutor
import queue

# 폴더 내 이미지 파일과 대응하는 레이블 파일(txt)을 확장자에 따라 필터링
def get_image_and_label_files(folder_path, extensions=('jpg', 'jpeg', 'png', 'gif', 'bmp')):
    image_files = []
    label_files = []

    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.lower().endswith(extensions):
                image_files.append(os.path.join(root, file))
                # 같은 이름의 txt 파일을 찾음
                label_file = os.path.join(root, os.path.splitext(file)[0] + '.txt')
                if os.path.exists(label_file):
                    label_files.append(label_file)
                else:
                    label_files.append(None)  # 없는 경우 None

    return image_files, label_files


# 이미지와 레이블 파일을 ZIP 파일로 묶는 함수
def zip_batch(image_files, label_files, output_folder, output_name, num):
    # 이미지 압축
    zip_image_file_name = os.path.join(output_folder, f"{output_name}_{num}.zip")
    with zipfile.ZipFile(zip_image_file_name, 'w') as zipf:
        for file in image_files:
            zipf.write(file, os.path.basename(file))  # 파일명만 ZIP에 저장

    print(f"{zip_image_file_name}에 {len(image_files)}개의 이미지 파일을 압축했습니다.")

    # 레이블 압축
    zip_label_file_name = os.path.join(output_folder, f"{output_name}_{num}_label.zip")
    with zipfile.ZipFile(zip_label_file_name, 'w') as zipf:
        for label_file in label_files:
            if label_file:  # 레이블 파일이 존재하는 경우
                zipf.write(label_file, os.path.basename(label_file))

    print(f"{zip_label_file_name}에 {len([f for f in label_files if f])}개의 레이블 파일을 압축했습니다.")


# 이미지 파일과 대응하는 레이블 파일을 1000개 단위로 묶어 ZIP 파일 생성
def zip_images_and_labels_in_batches(folder_path, output_folder, output_name, batch_size=1000, max_workers=4):
    num = 1  # 초기 값 설정
    image_files, label_files = get_image_and_label_files(folder_path)

    if not image_files:
        print("이미지 파일이 없습니다.")
        return

    # 출력 폴더가 존재하지 않으면 생성
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    total_files = len(image_files)
    print(f"{total_files}개의 이미지 파일을 찾았습니다.")

    # 작업을 저장할 큐 생성
    task_queue = queue.Queue()

    # 큐에 배치 작업을 추가
    for i in range(0, total_files, batch_size):
        batch_image_files = image_files[i:i + batch_size]
        batch_label_files = label_files[i:i + batch_size]
        current_num = num + 1
        task_queue.put((batch_image_files, batch_label_files, current_num))
        num += 1

    # 스레드 함수 정의
    def worker():
        while not task_queue.empty():
            batch_image_files, batch_label_files, current_num = task_queue.get()
            zip_batch(batch_image_files, batch_label_files, output_folder, output_name, current_num)
            task_queue.task_done()

    # 멀티쓰레딩 사용
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        # 각 스레드가 큐에서 작업을 가져가서 처리
        for _ in range(max_workers):
            executor.submit(worker)

    task_queue.join()  # 모든 작업이 완료될 때까지 대기


# 사용 예시
folder_path = r"E:\규제봉\output"  # 이미지 폴더 경로
output_folder = r"E:\규제봉"  # ZIP 파일을 저장할 폴더 경로
output_name = "traffic_regulation_AI-Hub"  # ZIP 파일명에 사용할 변수명
zip_images_and_labels_in_batches(folder_path, output_folder, output_name, max_workers=32)
