import os
import cv2
from ultralytics import YOLO

def process_image(model, image_path):
    # 이미지 로드 및 크기 정보 획득
    img = cv2.imread(image_path)
    if img is None:
        print(f"이미지를 읽어올 수 없습니다: {image_path}")
        return
    height, width = img.shape[:2]

    # YOLOv8 모델로 객체 검출
    results = model(image_path, classes=[6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

    # 이미지 파일명과 동일한 라벨 파일명 (예: image1.jpg -> image1.txt)
    base_name = os.path.splitext(os.path.basename(image_path))[0]
    label_path = os.path.join(os.path.dirname(image_path), base_name + ".txt")

    # 검출 결과를 기존 라벨 파일에 추가 ('a' 모드)
    with open(label_path, 'a') as f:
        for result in results:
            boxes = result.boxes  # bounding box 정보 객체
            for i in range(len(boxes)):
                # 기존 클래스 번호에 +6
                original_cls = int(boxes.cls[i].item())
                # new_cls = original_cls + 6

                # 박스 좌표 추출 (x_center, y_center, width, height)
                x_center, y_center, box_w, box_h = boxes.xywh[i].cpu().numpy()

                # 정규화 (이미지 가로/세로 크기로 나눔)
                x_center_norm = x_center / width
                y_center_norm = y_center / height
                box_w_norm = box_w / width
                box_h_norm = box_h / height

                # 한 줄에 하나의 박스 정보 (클래스 x_center y_center w h)
                f.write(f"{original_cls} {x_center_norm:.6f} {y_center_norm:.6f} {box_w_norm:.6f} {box_h_norm:.6f}\n")
    print(f"라벨 파일 업데이트 완료: {label_path}")

def main():
    # 상위 폴더 경로와 YOLO 모델 파일 경로를 변수로 지정
    folder = r'C:\Users\JMP\Desktop\data\4.1'  # 예: 'data/images'
    model_path = r'D:\DLFromScratch2ByPB\KT_data\2025.02.02\best.pt'              # 사용하고자 하는 모델 경로

    # YOLOv8 모델 로드
    model = YOLO(model_path)

    # 처리할 이미지 파일 확장자 (필요에 따라 추가 가능)
    image_extensions = [".jpg", ".jpeg", ".png", ".bmp"]

    # 상위 폴더부터 하위 폴더까지 재귀적으로 이미지 파일 탐색
    for root, dirs, files in os.walk(folder):
        for file in files:
            ext = os.path.splitext(file)[1].lower()
            if ext in image_extensions:
                image_path = os.path.join(root, file)
                print(f"처리 중: {image_path}")
                process_image(model, image_path)

if __name__ == "__main__":
    main()
