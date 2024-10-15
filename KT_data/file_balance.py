import os

# 클래스 이름 정의 및 unnomal_traffic_regulation 클래스 인덱스 설정
class_names = ["pothole", "normal_traffic_regulation", "unnomal_traffic_regulation", "roadkill", "falling_object"]
unnomal_class_index = class_names.index("unnomal_traffic_regulation")
target_line_count = 2000  # 유지할 클래스 라인의 수


def balance_unnomal_files_by_lines(base_folder, target_line_count):
    unnomal_files = []
    total_unnomal_lines = 0

    # 재귀적으로 폴더 탐색
    for root, _, files in os.walk(base_folder):
        for file in files:
            if file.endswith(".txt"):
                file_path = os.path.join(root, file)
                with open(file_path, 'r') as f:
                    lines = f.readlines()
                    # 각 줄이 숫자로 시작하는지 확인하고, 그렇지 않으면 무시
                    valid_lines = [line for line in lines if line.strip() and line.split()[0].isdigit()]

                    # 파일 내에 여러 클래스가 혼합된 경우를 제외
                    class_indices = {int(line.split()[0]) for line in valid_lines}
                    if len(class_indices) > 1:
                        continue  # 혼합된 클래스가 있으면 파일 유지

                    # 해당 파일이 unnomal_traffic_regulation 클래스만 포함하는지 확인
                    if all(int(line.split()[0]) == unnomal_class_index for line in valid_lines):
                        unnomal_files.append((file_path, len(valid_lines)))  # 파일 경로와 라인 수 저장
                        total_unnomal_lines += len(valid_lines)

    # 삭제할 필요가 있는지 확인
    if total_unnomal_lines > target_line_count:
        # 파일을 라인 수 기준으로 내림차순 정렬하여 많은 라인을 가진 파일부터 삭제
        unnomal_files.sort(key=lambda x: x[1], reverse=True)
        lines_to_delete = total_unnomal_lines - target_line_count
        deleted_files = 0

        for file_path, line_count in unnomal_files:
            if lines_to_delete <= 0:
                break
            # 해당 파일 삭제
            os.remove(file_path)
            lines_to_delete -= line_count
            deleted_files += 1
            print(f"Deleted: {file_path} ({line_count} lines)")

        print(f"Deleted {deleted_files} files. Total unnomal_traffic_regulation lines reduced to {target_line_count}.")
    else:
        print(f"Total unnomal_traffic_regulation lines: {total_unnomal_lines}. No files were deleted.")


# 사용자 입력을 통해 폴더 경로 설정
base_folder = r"D:\images\labels"

# 파일 밸런스 맞추기
balance_unnomal_files_by_lines(base_folder, target_line_count)
