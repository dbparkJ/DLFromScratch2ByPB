from PIL import Image
from PIL.ExifTags import TAGS

def get_exif_data(image_path):
    image = Image.open(image_path)
    exif_data = image._getexif()
    if not exif_data:
        print("No EXIF data found.")
    return exif_data

def get_exif_tag_value(exif_data, tag_name):
    if exif_data:
        for tag, value in exif_data.items():
            decoded_tag_name = TAGS.get(tag, tag)
            if decoded_tag_name == tag_name:
                return value
    return None

# 이미지 파일 경로
image_path = '4.jpg'

# EXIF 데이터를 가져옵니다.
exif_data = get_exif_data(image_path)

# 모든 EXIF 데이터를 출력해봅니다.
if exif_data:
    for tag, value in exif_data.items():
        decoded_tag_name = TAGS.get(tag, tag)
        print(f"Tag: {decoded_tag_name}, Value: {value}")

# 셔터 스피드 값을 저장할 변수
shutter_speed_value = get_exif_tag_value(exif_data, 'ExposureTime')

# 셔터 스피드의 n 값을 계산 및 출력
if shutter_speed_value:
    if isinstance(shutter_speed_value, tuple):
        numerator, denominator = shutter_speed_value
        n_value = int(denominator / numerator) if numerator != 0 else 'undefined'
        print(f"Shutter Speed (ExposureTime) n value: {n_value}")
    else:
        print(f"Shutter Speed (ExposureTime): {shutter_speed_value} seconds")
else:
    print("Shutter speed not found in EXIF data.")
