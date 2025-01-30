import requests
import uuid
from datetime import datetime

# 사용자 설정값
api_url = "http://127.0.0.1:5000/col/v1/image/saveImageFiles.do"  # 로컬 테스트를 위해 URL 수정
header_key = "X-GEO-ROADAI-API"
header_value = "Geon ce79e749650fb0c8595801d94c222bbc"
image_path = "./data/2024-12-18_10-56-31_frame_0391_resized.png"
event_id = str(uuid.uuid4())  # 고유한 Event ID 생성
timestamp = datetime.now().strftime("%Y%m%d%H%M%S")  # 타임스탬프 생성
box_result = "2, 1173, 981, 1310, 1024"  # 박스 결과 데이터

# 헤더 설정
headers = {
    header_key: header_value
}

# URL 파라미터 추가
params = {
    "box_result": box_result,
    "timestamp": timestamp
}

try:
    # 파일 핸들링은 with 문을 사용하여 리소스 관리를 자동화
    with open(image_path, "rb") as image_file:
        files = {
            "images": (f"{event_id}.png", image_file, "image/png")
        }

        print(f"파일 업로드 중: {image_path} → {api_url}")
        print(f"Event ID: {event_id}")

        # POST 요청 보내기
        response = requests.post(api_url, headers=headers, files=files, params=params)
        print(f"응답 상태 코드: {response.status_code}")
        print(f"응답 내용: {response.text}")

        if response.status_code == 200:
            print("[성공] 업로드 완료")
        else:
            print(f"[실패] 응답 코드: {response.status_code} | 내용: {response.text}")

except FileNotFoundError:
    print(f"[오류] 파일을 찾을 수 없습니다: {image_path}")
except requests.exceptions.RequestException as e:
    print(f"[오류] 요청 실패: {e}")
except Exception as e:
    print(f"[오류] 예상치 못한 오류 발생: {e}")
