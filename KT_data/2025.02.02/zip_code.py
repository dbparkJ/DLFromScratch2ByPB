import os
import zipfile

def compress_images_in_folder(root_folder, outputfolder):
    # outputfolder가 존재하지 않으면 생성
    if not os.path.exists(outputfolder):
        os.makedirs(outputfolder)
    
    # os.walk로 재귀적으로 폴더 탐색 (root_folder부터 시작)
    for dirpath, dirnames, filenames in os.walk(root_folder):
        # 확장자를 소문자로 처리하여 이미지(.png, .jpg) 파일 목록과 텍스트(.txt) 파일 목록 생성
        image_files = [f for f in filenames if f.lower().endswith(('.png', '.jpg'))]
        text_files = [f for f in filenames if f.lower().endswith('.txt')]
        
        # 이미지와 텍스트 파일이 모두 존재하는 폴더인 경우에만 처리
        if image_files and text_files:
            # 폴더 이름을 추출하여 zip파일의 이름으로 사용 (예: folder_name.zip)
            folder_name = os.path.basename(dirpath)
            zip_file_name = f"{folder_name}.zip"
            zip_file_path = os.path.join(outputfolder, zip_file_name)
            
            # zip파일 생성 (압축 모드: ZIP_DEFLATED)
            with zipfile.ZipFile(zip_file_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                for image_file in image_files:
                    image_path = os.path.join(dirpath, image_file)
                    # zip 내부에 저장할 때는 파일명만 사용하거나, 상대경로를 사용할 수 있음.
                    zipf.write(image_path, arcname=image_file)
            
            # 이미지 압축이 완료되었으므로, 원본 이미지 파일 삭제
            for image_file in image_files:
                image_path = os.path.join(dirpath, image_file)
                try:
                    os.remove(image_path)
                except Exception as e:
                    print(f"Error deleting {image_path}: {e}")
            
            print(f"폴더 '{dirpath}'의 이미지들을 '{zip_file_path}'로 압축 후 삭제하였습니다.")

if __name__ == "__main__":
    # 사용자로부터 상위 폴더와 출력 폴더 경로 입력 받기
    root_folder = r'C:\Users\JMP\Desktop\data\4.1'
    outputfolder = r'C:\Users\JMP\Desktop\data'
    
    compress_images_in_folder(root_folder, outputfolder)
