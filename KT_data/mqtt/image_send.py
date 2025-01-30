import os
import requests


def upload_image(api_url, image_path, header_key, header_value):
    """
    하나의 이미지 파일을 지정된 API URL로 업로드합니다.

    Args:
        api_url (str): API URL.
        image_path (str): 업로드할 이미지 파일의 경로.
        header_key (str): 헤더의 키.
        header_value (str): 헤더의 값.

    Returns:
        dict: 서버 응답 또는 오류 세부 정보.
    """
    headers = {
        header_key: header_value
    }

    # form-data에 파일 추가 (KEY: images, VALUE: 파일 데이터)
    files = {
        "images": (os.path.basename(image_path), open(image_path, "rb"), "image/png")
    }

    try:
        print(f"파일 업로드 중: {image_path} → {api_url}")
        response = requests.post(api_url, headers=headers, files=files)
        print(f"응답 상태 코드: {response.status_code}")
        print(f"응답 내용: {response.text}")

        if response.status_code == 200:
            return {"status": "success", "response": response.text}
        else:
            return {"status": "failure", "code": response.status_code, "message": response.text}
    except Exception as e:
        return {"status": "error", "message": str(e)}


def upload_images_from_folder(api_url, folder_path, header_key, header_value):
    """
    폴더 내 모든 이미지 파일을 지정된 API URL로 업로드합니다.

    Args:
        api_url (str): API URL.
        folder_path (str): 업로드할 이미지 파일이 위치한 폴더 경로.
        header_key (str): 헤더의 키.
        header_value (str): 헤더의 값.
    """
    if not os.path.isdir(folder_path):
        print(f"잘못된 폴더 경로: {folder_path}")
        return

    for file_name in os.listdir(folder_path):
        file_path = os.path.join(folder_path, file_name)

        # 이미지 파일만 처리 (확장자 필터링)
        if os.path.isfile(file_path) and file_name.lower().endswith((".png", ".jpg", ".jpeg", ".gif", ".bmp")):
            print(f"업로드 중: {file_name}")
            result = upload_image(api_url, file_path, header_key, header_value)
            if result["status"] == "success":
                print(f"[성공] 파일: {file_name} | 응답: {result['response']}")
            else:
                print(f"[실패] 파일: {file_name} | 상태 코드: {result.get('code')} | 메시지: {result.get('message')}")


# 사용 예제
if __name__ == "__main__":
    # API URL과 폴더 경로 설정
    ip = ""
    api_url = f"http://{ip}/col/v1/image/saveImageFiles.do"
    folder_path = "./data"  # 업로드할 이미지가 위치한 폴더 경로
    header_key = "X-GEO-ROADAI-API"
    header_value = "Geon ce79e749650fb0c8595801d94c222bbc"

    # 폴더 내 이미지 파일 업로드
    upload_images_from_folder(api_url, folder_path, header_key, header_value)
