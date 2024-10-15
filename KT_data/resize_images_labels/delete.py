import os


def delete_matching_files(input_folder, input2_folder):
    # 입력 폴더에 있는 모든 파일 리스트 가져오기
    for filename in os.listdir(input_folder):
        # .txt 파일만 처리
        if filename.endswith('.txt'):
            # _resized를 제외한 파일 이름 얻기
            base_filename = filename.replace('_resized', '')

            txt_file_path_input2 = os.path.join(input2_folder, base_filename)
            jpg_file_path_input2 = os.path.join(input2_folder, base_filename.replace('.txt', '.jpg'))

            # 입력2 폴더에 동일한 이름의 .txt 파일이 있는지 확인
            if os.path.exists(txt_file_path_input2):
                print(f"삭제 중: {txt_file_path_input2}")
                os.remove(txt_file_path_input2)  # .txt 파일 삭제

                # 동일한 이름의 .jpg 파일이 있으면 삭제
                if os.path.exists(jpg_file_path_input2):
                    print(f"삭제 중: {jpg_file_path_input2}")
                    os.remove(jpg_file_path_input2)  # .jpg 파일 삭제
            else:
                print(f"{base_filename} 파일이 입력2 폴더에 없습니다.")


# 사용 예시
input_folder = r'C:\Users\JM\Downloads\task_traffic_regulation_ai-hub_1-2024_09_12_01_36_25-yolo 1.1\obj_train_data'  # 입력 폴더 경로
input2_folder = r'E:\규제봉'  # 입력2 폴더 경로
delete_matching_files(input_folder, input2_folder)
