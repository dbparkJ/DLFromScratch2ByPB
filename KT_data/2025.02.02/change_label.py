import os

# 수정할 상위 폴더 경로 (필요에 맞게 변경하세요)
ROOT_FOLDER = r"D:\de_identification"

# 클래스 번호에 더할 값 (예: 6)
ADDITION_VALUE = 6

def process_file(file_path, addition_value):
    """
    파일 내 각 줄의 첫 번째 숫자(클래스)를 찾아 addition_value를 더하고
    변경된 내용을 다시 파일에 저장합니다.
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    modified_lines = []
    for line in lines:
        parts = line.strip().split()
        # 빈 줄인 경우 그대로 추가
        if not parts:
            modified_lines.append(line)
            continue
        
        try:
            # 첫 번째 요소를 정수형으로 변환 후 addition_value만큼 더함
            class_num = int(parts[0])
            new_class_num = class_num + addition_value
            # 변경된 클래스 번호와 나머지 데이터를 합쳐 새 줄 생성
            new_line = " ".join([str(new_class_num)] + parts[1:]) + "\n"
            modified_lines.append(new_line)
        except ValueError:
            # 만약 첫 번째 요소가 숫자가 아니라면 원본 줄을 그대로 사용
            modified_lines.append(line)

    # 변경된 내용을 파일에 다시 기록 (덮어쓰기)
    with open(file_path, 'w', encoding='utf-8') as f:
        f.writelines(modified_lines)
    print(f"수정 완료: {file_path}")

def process_folder(folder_path, addition_value):
    """
    지정한 폴더 내의 모든 txt 파일을 재귀적으로 찾아 process_file 함수를 실행합니다.
    """
    for dirpath, dirnames, filenames in os.walk(folder_path):
        for filename in filenames:
            if filename.lower().endswith('.txt'):
                file_path = os.path.join(dirpath, filename)
                process_file(file_path, addition_value)

if __name__ == '__main__':
    process_folder(ROOT_FOLDER, ADDITION_VALUE)
