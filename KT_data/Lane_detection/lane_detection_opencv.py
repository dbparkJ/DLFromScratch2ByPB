import cv2
import numpy as np


# 관심 영역 설정 함수
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


# 검출된 선을 확장하여 차선 그리기
def draw_extended_lines(img, lines, width):
    if lines is None:
        return img
    img = np.copy(img)
    blank_image = np.zeros_like(img)

    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else np.inf
            if 0.5 < np.abs(slope) < 2.0:  # 차선에 해당하는 기울기만 필터링
                if slope < 0:  # 왼쪽 차선
                    x1, x2 = int(x1 - width), int(x2 - width)  # 차선을 왼쪽으로 확장
                elif slope > 0:  # 오른쪽 차선
                    x1, x2 = int(x1 + width), int(x2 + width)  # 차선을 오른쪽으로 확장
                cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), 10)

    img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    return img


# 프레임 처리 함수
def process(image):
    height, width = image.shape[:2]

    # HSV로 변환 후 흰색과 노란색 차선 필터링
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))  # 흰색 차선 필터링
    yellow_mask = cv2.inRange(hsv, (15, 100, 100), (35, 255, 255))  # 노란색 차선 필터링
    combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
    filtered_image = cv2.bitwise_and(image, image, mask=combined_mask)

    # 그레이스케일 변환
    gray_image = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)

    # 가우시안 블러 적용
    blur_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

    # Canny 엣지 검출
    canny_image = cv2.Canny(blur_image, 50, 150)

    # 관심 영역 정의
    region_of_interest_vertices = [
        (width * 0.02, height),  # 왼쪽 아래 모서리
        (width * 0.45, height * 0.6),  # 왼쪽 상단
        (width * 0.55, height * 0.6),  # 오른쪽 상단
        (width * 0.5, height)  # 오른쪽 아래 모서리
    ]
    cropped_image = region_of_interest(canny_image, np.array([region_of_interest_vertices], np.int32))

    # HoughLinesP를 사용하여 직선 검출
    lines = cv2.HoughLinesP(
        cropped_image,
        rho=2,
        theta=np.pi / 180,
        threshold=50,
        minLineLength=100,
        maxLineGap=50
    )

    # 확장된 직선을 그린 이미지 생성
    image_with_lines = draw_extended_lines(image, lines, width=50)

    return image_with_lines


# 비디오 캡처
cap = cv2.VideoCapture("2024-10-24_09-06-56.mp4")

# 창 크기 조절 가능하게 설정
cv2.namedWindow("Lane Detection", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("Lane Detection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        processed_frame = process(frame)
        cv2.imshow("Lane Detection", processed_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()