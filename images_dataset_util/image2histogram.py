import cv2
import numpy as np
import matplotlib.pyplot as plt
import mplcursors

# 이미지 파일 경로
image_path = r'443_7.9m.png'
output_path_gray = r'grayscale_image.jpg'
output_path_colormap = r'colormap_image.jpg'
output_path_categorized = r'categorized_image.jpg'

# 이미지를 그레이스케일로 읽기
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# 이미지가 제대로 읽혔는지 확인
if image is None:
    print("이미지를 불러올 수 없습니다. 파일 경로를 확인하세요.")
else:
    # 히스토그램 계산
    hist = cv2.calcHist([image], [0], None, [256], [0, 256])

    # 히스토그램 그리기
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_title("Grayscale Histogram")
    ax.set_xlabel("Pixel Value")
    ax.set_ylabel("Frequency")
    line, = ax.plot(hist)
    ax.set_xlim([0, 256])

    # mplcursors를 사용하여 상호작용 추가
    cursor = mplcursors.cursor(line, hover=True)


    @cursor.connect("add")
    def on_add(sel):
        x, y = sel.target
        sel.annotation.set_text(f"Pixel Value: {int(x)}, Frequency: {int(y)}")


    plt.show(block=True)  # block=True를 설정하여 창이 닫히지 않도록 함

    # 컬러맵을 적용한 이미지 생성
    colormap_image = cv2.applyColorMap(image, cv2.COLORMAP_JET)

    # 원본 이미지와 컬러맵 이미지를 나란히 표시
    plt.figure(figsize=(15, 5))

    plt.subplot(1, 2, 1)
    plt.title("Original Grayscale Image")
    plt.imshow(image, cmap='gray')
    plt.axis('off')

    plt.subplot(1, 2, 2)
    plt.title("Colormap Image")
    plt.imshow(colormap_image)
    plt.axis('off')

    plt.show()

    # 그레이스케일 이미지 저장
    cv2.imwrite(output_path_gray, image)

    # 컬러맵 이미지 저장
    cv2.imwrite(output_path_colormap, colormap_image)

    # 명도 값을 여러 범주로 나누기
    thresholds = [50, 100, 150, 200, 255]
    categorized_image = np.digitize(image, bins=thresholds, right=True)

    # 각 범주의 픽셀 수 계산
    unique, counts = np.unique(categorized_image, return_counts=True)
    pixel_counts = dict(zip(unique, counts))

    # 픽셀 수 출력
    for category, count in pixel_counts.items():
        print(f"Category {category}: {count} pixels")

    # 각 범주에 색상을 부여
    color_map = {
        0: [0, 0, 255],  # Blue
        1: [0, 255, 255],  # Cyan
        2: [0, 255, 0],  # Green
        3: [255, 255, 0],  # Yellow
        4: [255, 0, 0],  # Red
        5: [255, 255, 255]  # White
    }

    # 카테고리 이미지 생성
    categorized_image_color = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
    for category, color in color_map.items():
        categorized_image_color[categorized_image == category] = color

    # 범주화된 이미지 표시 및 저장
    plt.figure(figsize=(10, 5))
    plt.title("Categorized Image")
    plt.imshow(categorized_image_color)
    plt.axis('off')
    plt.show()

    # 범주화된 이미지 저장
    cv2.imwrite(output_path_categorized, categorized_image_color)
    print("이미지가 성공적으로 저장되었습니다.")