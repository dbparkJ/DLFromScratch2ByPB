import cv2
import numpy as np
import os

import time
def preprocess_color_image(image_path, output_folder, clahe_clip=2.0, clahe_tile=(2, 2), 
                           sobel_kernel=1, blend_ratio=(0.95, 0.05)):
    start_time = time.time()
    # 이미지 읽기 (BGR 형식)
    image = cv2.imread(image_path)
    if image is None:
        print(f"⚠️ 이미지를 불러올 수 없습니다: {image_path}")
        return  # 이미지가 없으면 바로 종료
    
    # BGR 이미지를 LAB 색공간으로 변환
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    L, A, B = cv2.split(lab)
    
    # L 채널에 CLAHE 적용
    clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=clahe_tile)
    L_clahe = clahe.apply(L)
    
    # L 채널에 Sobel Edge Detection 적용
    grad_x = cv2.Sobel(L, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    grad_y = cv2.Sobel(L, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    sobel_L = cv2.magnitude(grad_x, grad_y)
    sobel_L = cv2.convertScaleAbs(sobel_L)
    
    # CLAHE 결과와 Sobel 결과를 지정된 비율로 합성
    L_combined = cv2.addWeighted(L_clahe, blend_ratio[0], sobel_L, blend_ratio[1], 0)
    
    # 합성된 L 채널과 원래의 A, B 채널을 병합하여 LAB 이미지 재구성
    lab_combined = cv2.merge([L_combined, A, B])
    # LAB 이미지를 다시 BGR로 변환
    result = cv2.cvtColor(lab_combined, cv2.COLOR_LAB2BGR)
    
    # 전처리 후 결과에 gamma correction 적용
    result = adjust_gamma(result, gamma=1.2)
    
    # 원본 파일명에서 확장자 분리 후 새로운 파일명 생성
    base_name = os.path.basename(image_path)
    name, ext = os.path.splitext(base_name)
    output_path = os.path.join(output_folder, f"{name}_pre{ext}")
    
    # 전처리된 이미지 저장
    cv2.imwrite(output_path, result)
    print(f"✅ 전처리 완료: {output_path}")
    print(f"🕒 소요 시간: {time.time() - start_time:.2f}초")
def adjust_gamma(image, gamma=1.2):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

def process_images_in_folder(input_folder, output_folder, clahe_clip=2.0, clahe_tile=(8, 8), 
                              sobel_kernel=3, blend_ratio=(0.8, 0.2)):
    # 입력 폴더 확인
    if not os.path.exists(input_folder):
        print(f"❌ 입력 폴더가 존재하지 않습니다: {input_folder}")
        return
    
    # 출력 폴더 생성 (존재하지 않으면 자동 생성)
    os.makedirs(output_folder, exist_ok=True)
    
    # 폴더 내 이미지 파일 리스트 가져오기
    image_files = [f for f in os.listdir(input_folder) if f.lower().endswith(('png', 'jpg', 'jpeg', 'bmp', 'tiff'))]
    
    if not image_files:
        print(f"⚠️ 폴더에 처리할 이미지가 없습니다: {input_folder}")
        return
    
    # 모든 이미지 처리
    for file_name in image_files:
        image_path = os.path.join(input_folder, file_name)
        print(f"🔄 처리 중: {image_path}")
        preprocess_color_image(image_path, output_folder, clahe_clip, clahe_tile, sobel_kernel, blend_ratio)

# 예제 실행
if __name__ == "__main__":
    image_path = r"C:\Users\JMP\Downloads\2025.02.13\obj_train_data"
    output_folder = r"C:\Users\JMP\Downloads\output"

    process_images_in_folder(image_path, output_folder)






def preprocess_color_image(image, clahe_clip=2.0, clahe_tile=(2, 2), 
                           sobel_kernel=1, blend_ratio=(0.95, 0.05)):
    start_time = time.time()
    # BGR 이미지를 LAB 색공간으로 변환
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    L, A, B = cv2.split(lab)
    
    # L 채널에 CLAHE 적용
    clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=clahe_tile)
    L_clahe = clahe.apply(L)
    
    # L 채널에 Sobel Edge Detection 적용
    grad_x = cv2.Sobel(L, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    grad_y = cv2.Sobel(L, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    sobel_L = cv2.magnitude(grad_x, grad_y)
    sobel_L = cv2.convertScaleAbs(sobel_L)
    
    # CLAHE 결과와 Sobel 결과를 지정된 비율로 합성
    L_combined = cv2.addWeighted(L_clahe, blend_ratio[0], sobel_L, blend_ratio[1], 0)
    
    # 합성된 L 채널과 원래의 A, B 채널을 병합하여 LAB 이미지 재구성
    lab_combined = cv2.merge([L_combined, A, B])
    # LAB 이미지를 다시 BGR로 변환
    result = cv2.cvtColor(lab_combined, cv2.COLOR_LAB2BGR)
    
    # 전처리 후 결과에 gamma correction 적용
    result = adjust_gamma(result, gamma=1.2)
    
    print(f"🕒 소요 시간: {time.time() - start_time:.2f}초")
    return result