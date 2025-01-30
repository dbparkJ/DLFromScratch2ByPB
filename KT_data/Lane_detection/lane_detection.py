import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from unet_model import UNet  # U-Net 모델 정의가 필요
from torch.utils.data import DataLoader, Dataset
import torch.optim as optim
import torch.nn as nn
import os
from PIL import Image


# 데이터셋 정의
class LaneDataset(Dataset):
    def __init__(self, image_paths, mask_paths, transform=None):
        self.image_paths = image_paths
        self.mask_paths = mask_paths
        self.transform = transform

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        image = Image.open(self.image_paths[idx]).convert('RGB')
        mask = Image.open(self.mask_paths[idx]).convert('L')

        if self.transform:
            image = self.transform(image)
            mask = self.transform(mask)

        return image, mask


# 데이터셋 경로 설정
train_image_paths = ["path/to/train/images/"]  # 훈련 이미지 경로 리스트
train_mask_paths = ["path/to/train/masks/"]  # 훈련 마스크 경로 리스트

# 데이터 변환 정의
transform = transforms.Compose([
    transforms.Resize((256, 256)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
])

# 데이터셋 및 데이터로더 생성
train_dataset = LaneDataset(train_image_paths, train_mask_paths, transform=transform)
train_loader = DataLoader(train_dataset, batch_size=4, shuffle=True)

# 모델 정의 및 설정
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = UNet(n_channels=3, n_classes=1).to(device)
optimizer = optim.Adam(model.parameters(), lr=0.001)
criterion = nn.BCEWithLogitsLoss()  # 이진 분류를 위한 손실 함수

# 모델 학습 루프
epochs = 10
for epoch in range(epochs):
    model.train()
    running_loss = 0.0
    for images, masks in train_loader:
        images, masks = images.to(device), masks.to(device)

        # 옵티마이저 초기화
        optimizer.zero_grad()

        # 예측 및 손실 계산
        outputs = model(images)
        loss = criterion(outputs, masks)

        # 역전파 및 옵티마이저 스텝
        loss.backward()
        optimizer.step()

        running_loss += loss.item()

    # 에포크당 손실 출력
    print(f"Epoch [{epoch + 1}/{epochs}], Loss: {running_loss / len(train_loader):.4f}")

# 모델 저장
torch.save(model.state_dict(), 'lane_detection_unet.pth')

# 사전 학습된 U-Net 모델 로드 (모델 파일 경로 필요)
model.load_state_dict(torch.load('lane_detection_unet.pth', map_location=device))
model.to(device)
model.eval()


# 프레임 전처리 함수
def preprocess_frame(frame):
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((256, 256)),  # 모델 입력 크기에 맞게 조정
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])  # 정규화
    ])
    frame_tensor = transform(frame).unsqueeze(0)  # 배치 차원 추가
    return frame_tensor.to(device)


# 차선 마스크 후처리 함수
def postprocess_mask(mask, original_shape):
    mask = mask.squeeze().cpu().detach().numpy()  # 텐서를 넘파이 배열로 변환
    mask_resized = cv2.resize(mask, (original_shape[1], original_shape[0]))
    mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255  # 임계값을 적용하여 이진화
    return mask_binary


# 프레임 처리 함수
def process_frame_dl(frame):
    preprocessed_frame = preprocess_frame(frame)
    with torch.no_grad():
        prediction = model(preprocessed_frame)[0]  # 예측 수행
    mask = prediction[0]  # 첫 번째 채널 사용 (흑백 마스크)
    mask_binary = postprocess_mask(mask, frame.shape)

    # 원본 프레임 위에 마스크 적용
    mask_colored = cv2.cvtColor(mask_binary, cv2.COLOR_GRAY2BGR)
    result = cv2.addWeighted(frame, 1, mask_colored, 0.3, 0)
    return result


# 비디오 캡처
cap = cv2.VideoCapture('2024-10-24_09-06-56.mp4')
cv2.namedWindow('Lane Detection (Deep Learning)', cv2.WINDOW_NORMAL)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 딥러닝 기반 차선 검출 적용
    result = process_frame_dl(frame)

    # 결과 표시
    cv2.imshow('Lane Detection (Deep Learning)', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
