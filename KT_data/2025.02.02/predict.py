import cv2
import torch
from ultralytics import YOLO
from tqdm import tqdm  # 🔥 진행률 표시 라이브러리 추가

# 모델 로드 (YOLOv8n, YOLOv8s, YOLOv8m 등 선택 가능)
model = YOLO(r"C:\Users\JMP\Desktop\v8_version4.2_model_m_param_imgsz_1280\weights\best.pt")  # 사용자 모델

# 입력 비디오 파일 경로
video_path = r"C:\Users\JMP\Desktop\2025-02-03_09-25-40.mp4"
# 출력 비디오 파일 경로
output_path = r"C:\Users\JMP\Desktop\2025-02-03_09-25-40_output.mp4"

# 비디오 캡처 객체 생성
cap = cv2.VideoCapture(video_path)

# 비디오 속성 가져오기
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
fps = int(cap.get(cv2.CAP_PROP_FPS))
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))  # ✅ 총 프레임 개수 가져오기

# 비디오 저장 객체 설정 (코덱: MP4V)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

# 🔥 진행률 바 설정 (tqdm 사용)
progress_bar = tqdm(total=total_frames, desc="🎬 비디오 처리 중", unit="frame")

# 프레임 단위로 추론 수행
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break  # 비디오가 끝나면 종료

    # YOLOv8 추론 수행
    results = model.predict(source=frame, verbose=False)

    # 결과를 OpenCV 프레임에 렌더링 (font_size와 line_width 적용)
    annotated_frame = list(results)[0].plot(font_size=0.5, line_width=2)

    # 비디오 프레임 저장
    out.write(annotated_frame)

    # 🔥 진행률 업데이트
    progress_bar.update(1)

# 자원 해제
cap.release()
out.release()
cv2.destroyAllWindows()
progress_bar.close()  # ✅ tqdm 종료

print(f"✅ 추론 완료! 결과 저장: {output_path}")
