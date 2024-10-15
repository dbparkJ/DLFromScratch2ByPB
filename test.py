import os
from PIL import Image

def convert_jpeg_to_png(input_folder, output_folder):
    # 입력 폴더 내 파일 목록 가져오기
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for filename in os.listdir(input_folder):
        if filename.lower().endswith(".jpeg") or filename.lower().endswith(".jpg"):
            # 파일 경로 설정
            input_path = os.path.join(input_folder, filename)
            output_filename = os.path.splitext(filename)[0] + ".png"
            output_path = os.path.join(output_folder, output_filename)

            # 이미지 열기 및 PNG로 변환하여 저장
            try:
                img = Image.open(input_path)
                img.save(output_path, "PNG")
                print(f"변환 완료: {filename} -> {output_filename}")
            except Exception as e:
                print(f"에러 발생: {filename} 변환 실패. {e}")

# 폴더 경로 입력
input_folder = r"C:\Users\JM\Downloads\민정이"
output_folder = r"C:\Users\JM\Downloads\민정이\output"

# JPEG -> PNG 변환 함수 실행
convert_jpeg_to_png(input_folder, output_folder)
