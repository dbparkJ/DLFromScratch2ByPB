import cv2
import numpy as np

# 두 이미지를 로드합니다.
image_with_flash = cv2.imread('2.JPG')
image_without_flash = cv2.imread('5.JPG')

# 이미지를 회색조로 변환합니다.
gray_flash = cv2.cvtColor(image_with_flash, cv2.COLOR_BGR2GRAY)
gray_normal = cv2.cvtColor(image_without_flash, cv2.COLOR_BGR2GRAY)

# 두 이미지 간의 차이를 계산합니다.
diff = cv2.absdiff(gray_flash, gray_normal)

# 차이 이미지를 이진화하여 플래시가 터진 부분을 마스크로 만듭니다.
_, mask = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)

# 마스크를 색상 이미지로 변환합니다.
mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

# 원본 이미지를 마스크와 합성하여 플래시 부분만 남기고 나머지는 검정색으로 만듭니다.
result = cv2.bitwise_and(image_with_flash, mask_color)

# 결과 이미지를 저장합니다.
cv2.imwrite('result_image.jpg', result)

# 결과 이미지를 화면에 표시합니다.
cv2.imshow('Result', result)
cv2.waitKey(0)
cv2.destroyAllWindows()
