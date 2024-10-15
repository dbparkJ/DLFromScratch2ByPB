import cv2
import numpy as np
from PIL import Image
import piexif


def extract_exif_data(image_path):
    img = Image.open(image_path)
    try:
        exif_dict = piexif.load(img.info['exif'])

        exposure_time = exif_dict['Exif'][piexif.ExifIFD.ExposureTime]
        aperture = exif_dict['Exif'][piexif.ExifIFD.FNumber]
        ISO = exif_dict['Exif'][piexif.ExifIFD.ISOSpeedRatings]

        # Exposure time and aperture are returned as tuples (numerator, denominator)
        exposure_time = exposure_time[0] / exposure_time[1]
        aperture = aperture[0] / aperture[1]

    except KeyError:
        # 기본값 설정 (예시 값 사용)
        exposure_time = 1 / 50
        aperture = 1.6953125  # 최대 개방 값을 사용
        ISO = 100

    return exposure_time, aperture, ISO


def calculate_luminance(pixel_value, response_curve, exposure_time):
    return response_curve[pixel_value] / exposure_time


def calculate_illuminance(pixel_value, response_curve, exposure_time, aperture, ISO):
    return (aperture ** 2 / (ISO * exposure_time)) * response_curve[pixel_value]


def calculate_retro_reflectivity(luminance, illuminance):
    # illuminance 값이 0인 경우 처리
    with np.errstate(divide='ignore', invalid='ignore'):
        retro_reflectivity = np.true_divide(luminance, illuminance)
        retro_reflectivity[illuminance == 0] = 0  # 조도가 0인 경우 반사율을 0으로 설정
    return retro_reflectivity


# 플래시 마스크 생성 함수
def create_flash_mask(img_flash, img_no_flash, threshold=50):
    diff = cv2.absdiff(img_flash, img_no_flash)
    _, mask = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
    return mask


# 저장된 이미지 로드
aligned_img_path = 'aligned_image.jpg'
night_img_path = 'night_image.jpg'
no_flash_img_path = 'no_flash_image.jpg'
aligned_img = cv2.imread(aligned_img_path, cv2.IMREAD_GRAYSCALE)
night_img = cv2.imread(night_img_path, cv2.IMREAD_GRAYSCALE)
no_flash_img = cv2.imread(no_flash_img_path, cv2.IMREAD_GRAYSCALE)

# 이미지 메타데이터에서 카메라 설정 추출
exposure_time, aperture, ISO = extract_exif_data(aligned_img_path)

# 예시 반응 곡선 (로그 함수)
response_curve = np.log(np.arange(1, 257))  # 크기를 256으로 변경

# 플래시 마스크 생성
flash_mask = create_flash_mask(night_img, no_flash_img)

# 휘도 및 조도 계산
luminance_img_flash = calculate_luminance(night_img, response_curve, exposure_time)
luminance_img_no_flash = calculate_luminance(no_flash_img, response_curve, exposure_time)
illuminance_img_flash = calculate_illuminance(aligned_img, response_curve, exposure_time, aperture, ISO)
illuminance_img_no_flash = calculate_illuminance(aligned_img, response_curve, exposure_time, aperture, ISO)

# 재귀반사율 계산
retro_reflectivity_img_flash = calculate_retro_reflectivity(luminance_img_flash, illuminance_img_flash)
retro_reflectivity_img_no_flash = calculate_retro_reflectivity(luminance_img_no_flash, illuminance_img_no_flash)

# 결과 이미지 저장
cv2.imwrite('luminance_image_flash.jpg', luminance_img_flash)
cv2.imwrite('illuminance_image_flash.jpg', illuminance_img_flash)
cv2.imwrite('retro_reflectivity_image_flash.jpg', retro_reflectivity_img_flash)
cv2.imwrite('luminance_image_no_flash.jpg', luminance_img_no_flash)
cv2.imwrite('illuminance_image_no_flash.jpg', illuminance_img_no_flash)
cv2.imwrite('retro_reflectivity_image_no_flash.jpg', retro_reflectivity_img_no_flash)
