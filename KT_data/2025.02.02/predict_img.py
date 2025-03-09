import os
import cv2
from ultralytics import YOLO

def load_model(model_path='yolov8n.pt'):
    """YOLOv8 모델을 로드합니다."""
    return YOLO(model_path)

def process_image(model, image_path, output_folder):
    """이미지를 YOLOv8 모델로 검출하고 결과를 저장합니다."""
    image = cv2.imread(image_path)
    if image is None:
        print(f"이미지를 불러올 수 없습니다: {image_path}")
        return
    
    results = model(image)  # 모델 추론
    
    img_height, img_width = image.shape[:2]
    txt_filename = os.path.splitext(os.path.basename(image_path))[0] + ".txt"
    txt_path = os.path.join(output_folder, txt_filename)
    
    with open(txt_path, 'w') as f:
        for result in results:
            for box in result.boxes:
                cls = int(box.cls.item())  # 클래스
                x_center = box.xywhn[0][0].item()  # 정규화된 x 중심
                y_center = box.xywhn[0][1].item()  # 정규화된 y 중심
                width = box.xywhn[0][2].item()  # 정규화된 너비
                height = box.xywhn[0][3].item()  # 정규화된 높이
                
                f.write(f"{cls} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")
    print(f"검출 결과 저장 완료: {txt_path}")

def process_folder(model, input_folder, output_folder):
    """폴더를 재귀적으로 탐색하여 모든 이미지를 처리합니다."""
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    for root, _, files in os.walk(input_folder):
        for file in files:
            if file.lower().endswith(('jpg', 'jpeg', 'png', 'bmp', 'tiff', 'tif')):
                image_path = os.path.join(root, file)
                process_image(model, image_path, output_folder)

if __name__ == "__main__":
    input_folder = r"C:\Users\JMP\Desktop\20250213"  # 입력 폴더 경로
    output_folder = r"C:\Users\JMP\Desktop\20250213_output"  # 출력 폴더 경로
    
    model = load_model(r"C:\Users\JMP\Desktop\best.pt")
    process_folder(model, input_folder, output_folder)
