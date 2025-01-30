import os
import pandas as pd


def process_csv_files(input_folder, output_folder):
    # 재귀적으로 입력 폴더를 탐색
    for root, dirs, files in os.walk(input_folder):
        for file in files:
            # CSV 파일인 경우만 처리
            if file.endswith('.csv'):
                file_path = os.path.join(root, file)
                print(f"Processing file: {file_path}")

                # CSV 파일 읽어오기
                df = pd.read_csv(file_path, sep=';')

                # 'severity' 열 삭제
                if 'severity' in df.columns:
                    df = df.drop(columns=['severity'])

                # 'stamp' 열을 첫 번째로 이동하고 소수점 5째 자리로 변환
                if 'stamp' in df.columns:
                    df['stamp'] = df['stamp'].astype(float).round(5)  # 소수점 5째 자리까지 반올림
                    cols = ['stamp'] + [col for col in df.columns if col != 'stamp']
                    df = df[cols]

                # 행 순서를 거꾸로 변경
                df = df.iloc[::-1].reset_index(drop=True)

                # 출력 폴더 내에서 입력 폴더의 하위 경로 유지
                relative_path = os.path.relpath(root, input_folder)
                target_folder = os.path.join(output_folder, relative_path)

                # 하위 폴더가 없다면 생성
                if not os.path.exists(target_folder):
                    os.makedirs(target_folder)

                # 수정된 파일 이름 생성 (엑셀 파일로 저장)
                modified_file_name = file.replace('.csv', '_modified.xlsx')
                modified_file_path = os.path.join(target_folder, modified_file_name)

                # 수정된 데이터프레임을 엑셀 파일로 저장
                df.to_excel(modified_file_path, index=False)
                print(f"Saved modified file: {modified_file_path}")


# 사용자가 입력한 최상위 폴더 및 출력 폴더 경로
input_folder = input(r"최상위 폴더 경로를 입력하세요: ")
output_folder = input(r"출력 폴더 경로를 입력하세요: ")
process_csv_files(input_folder, output_folder)
