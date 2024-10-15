import cv2
import numpy as np

# LDR 이미지 파일 경로
ldr_images = ["4.jpg", "5.jpg", "6.jpg"]

# LDR 이미지 읽기
ldr_list = [cv2.imread(img) for img in ldr_images]

# 이미지가 제대로 읽혔는지 확인
for i, ldr in enumerate(ldr_list):
    if ldr is None:
        print(f"이미지 {ldr_images[i]}을(를) 읽는 데 실패했습니다.")
        exit()

# 노출 시간 (주어진 값 사용)
exposure_times = np.array([50.0, 25.0, 13.0], dtype=np.float32)

# 노출 시간에 대한 로그 계산
log_exposure_times = np.log(exposure_times)

# AlignMTB를 사용하여 이미지를 정렬
alignMTB = cv2.createAlignMTB()
alignMTB.process(ldr_list, ldr_list)

# 정렬된 이미지를 저장하여 확인
for i, aligned_img in enumerate(ldr_list):
    cv2.imwrite(f"aligned_{i}.jpg", aligned_img)

# CalibrateDebevec를 사용하여 카메라 응답 함수 추정
calibrateDebevec = cv2.createCalibrateDebevec()
try:
    responseDebevec = calibrateDebevec.process(ldr_list, log_exposure_times)
except cv2.error as e:
    print(f"카메라 응답 함수 추정 중 오류 발생: {e}")
    exit()

# MergeDebevec를 사용하여 HDR 이미지 생성
mergeDebevec = cv2.createMergeDebevec()
try:
    hdrDebevec = mergeDebevec.process(ldr_list, log_exposure_times, responseDebevec)
except cv2.error as e:
    print(f"HDR 이미지 생성 중 오류 발생: {e}")
    exit()

# HDR 이미지가 생성되었는지 확인
if hdrDebevec is None or np.isnan(hdrDebevec).any() or np.isinf(hdrDebevec).any():
    print("HDR 이미지가 올바르게 생성되지 않았습니다.")
    exit()

# HDR 이미지 값 확인을 위한 디버깅
print("HDR 이미지의 크기:", hdrDebevec.shape)
print("HDR 이미지의 데이터 타입:", hdrDebevec.dtype)
print("HDR 이미지의 일부 값:", hdrDebevec[0, 0])

# HDR 이미지 값의 범위를 각 채널별로 확인하고 정규화
channels = cv2.split(hdrDebevec)
min_vals = [cv2.minMaxLoc(channel)[0] for channel in channels]
max_vals = [cv2.minMaxLoc(channel)[1] for channel in channels]

print(f"HDR 이미지의 채널별 최소값: {min_vals}, 최대값: {max_vals}")

# 각 채널을 정규화
normalized_channels = [channel / max_vals[i] for i, channel in enumerate(channels)]

hdrDebevec = cv2.merge(normalized_channels)

# HDR 이미지를 저장 (32비트 float 형식)
cv2.imwrite("hdr_image.hdr", hdrDebevec)

# TonemapDrago를 사용하여 톤 매핑 (디스플레이용으로 밝기 조정)
tonemapDrago = cv2.createTonemapDrago(1.0, 0.7)
ldrDrago = tonemapDrago.process(hdrDebevec)

# NaN 및 Inf 값을 0으로 대체
ldrDrago = np.nan_to_num(ldrDrago, nan=0.0, posinf=0.0, neginf=0.0)

# 8비트 이미지로 변환 (24비트 수준)
ldrDrago_8bit = np.clip(ldrDrago * 255, 0, 255).astype('uint8')

# 결과 이미지 저장 (24비트 이미지로 저장)
cv2.imwrite("ldr_result.jpg", ldrDrago_8bit)

print("HDR 이미지와 톤 매핑된 LDR 이미지를 생성하였습니다.")
