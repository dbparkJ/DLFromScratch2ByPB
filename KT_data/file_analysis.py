import os
import matplotlib.pyplot as plt
from collections import defaultdict

# 클래스 이름 정의
class_names = ["pothole", "normal_traffic_regulation", "unnomal_traffic_regulation", "roadkill", "falling_object"]
colors = ["red", "orange", "yellow", "green", "blue"]  # 클래스별 색상 정의


def count_classes_and_files(base_folder):
    txt_count = 0
    class_count = defaultdict(int)

    # 재귀적으로 폴더 탐색
    for root, dirs, files in os.walk(base_folder):
        # 각 디렉토리 내에 'obj_train_data' 폴더가 있는지 확인
        if 'obj_train_data' in dirs:
            obj_train_data_path = os.path.join(root, 'obj_train_data')
            for sub_root, _, sub_files in os.walk(obj_train_data_path):
                for file in sub_files:
                    if file.endswith(".txt"):
                        txt_count += 1
                        file_path = os.path.join(sub_root, file)
                        with open(file_path, 'r') as f:
                            lines = f.readlines()
                            for line in lines:
                                class_index = int(line.split()[0])  # 첫 번째 숫자가 class를 나타냄
                                class_count[class_names[class_index]] += 1  # 해당 class의 count 증가

    return txt_count, class_count


def plot_class_distribution(class_count, save_path):
    # 클래스별 데이터 추출
    classes = list(class_count.keys())
    counts = list(class_count.values())

    # 막대 그래프 그리기
    plt.figure(figsize=(10, 6))
    bars = plt.bar(classes, counts, color=colors)
    plt.xlabel('Class')
    plt.ylabel('Count')
    plt.title('Class Distribution')
    plt.xticks(rotation=45)

    # 각 막대 위에 데이터 라벨 추가
    for bar in bars:
        yval = bar.get_height()
        plt.text(bar.get_x() + bar.get_width() / 2, yval + 1, int(yval), ha='center', va='bottom')

    # 그래프 저장
    plt.savefig(save_path)
    plt.show()


# 사용자 입력을 통해 폴더 경로 설정
base_folder = r"D:\balance_data"

# 파일 수와 클래스 수 파악
txt_file_count, class_counts = count_classes_and_files(base_folder)

# 결과 출력
print(f"Total number of txt files: {txt_file_count}")
print("Class counts:")
for class_name, count in class_counts.items():
    print(f"{class_name}: {count}")

# 클래스 분포 그래프 저장 경로
save_path = os.path.join(base_folder, 'class_distribution.png')

# 클래스 분포 그래프 및 저장
plot_class_distribution(class_counts, save_path)
print(f"Class distribution plot saved to: {save_path}")
