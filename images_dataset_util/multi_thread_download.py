import requests
import os
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
import multiprocessing

# 파일 경로와 저장할 폴더 경로를 설정합니다.
file_path = 'multimedia.txt'  # 파일 경로를 입력하세요
output_dir = r'D:\roadkill2'  # 이미지를 저장할 폴더 경로를 입력하세요
os.makedirs(output_dir, exist_ok=True)

# 파일에서 URL을 읽습니다.
with open(file_path, 'r') as file:
    lines = file.readlines()

# URL을 추출하여 리스트에 저장합니다.
urls = []
for line in lines:
    parts = line.split('\t')
    if len(parts) > 3:
        url = parts[3]
        if url.startswith('http'):
            urls.append(url)

# 다운로드 함수
def download_image(url, index):
    try:
        response = requests.get(url)
        response.raise_for_status()
        file_name = os.path.join(output_dir, f"{index:05d}.jpg")
        with open(file_name, 'wb') as img_file:
            img_file.write(response.content)
        print(f"Downloaded {file_name}")
    except requests.exceptions.RequestException as e:
        print(f"Failed to download {url}: {e}")

# 시스템의 코어 수에 맞게 스레드 설정
num_cores = multiprocessing.cpu_count()
print(f"Number of cores: {num_cores}")

# 멀티스레딩을 이용하여 이미지 다운로드
with ThreadPoolExecutor(max_workers=num_cores) as executor:
    futures = [executor.submit(download_image, url, idx) for idx, url in enumerate(urls)]
    for future in as_completed(futures):
        future.result()

print("All downloads completed.")
