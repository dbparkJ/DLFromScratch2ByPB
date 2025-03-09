import os
import shutil

def move_files_with_filtered_labels(input_root, output_root, min_pixel=15):
    """
    input_root: 원본 데이터가 있는 상위 폴더 경로
    output_root: 기준에 적합한 데이터를 저장할 output 폴더 경로
    min_pixel: 바운딩 박스의 가로와 세로 길이가 이 값 이상이어야 유효함 (기본값 15)
    
    동작:
      - input_root 아래의 모든 폴더를 재귀적으로 탐색
      - 이미지 파일(확장자 png, jpg, jpeg)을 찾고, 동일한 이름의 .txt 파일(레이블 파일)을 함께 처리
      - 레이블 파일의 각 줄은 "class x y w h" (정규화된 값, 0~1) 형태라고 가정
      - 실제 크기는 고정된 이미지 크기 (1920×1080)를 곱해 계산
      - 한 줄의 가로 혹은 세로가 15px 미만이면 해당 줄은 필터링(제거)
      - 필터링 후 유효한(15px 이상) 라벨 데이터가 한 개 이상 있어야 이동 진행
      - 레이블 파일이 아예 비어있거나, 유효한 라벨이 하나도 없다면 이동하지 않음.
      - 이동 시, 전체 폴더 구조를 복제하지 않고, 
           input_root 바로 아래의 폴더(즉, 가장 상위 폴더)만 output 폴더 하위에 생성하여 이동
    """
    # 고정된 이미지 크기
    img_width, img_height = 1920, 1080

    # input_root의 모든 하위 폴더 및 파일을 재귀적으로 탐색
    for root, dirs, files in os.walk(input_root):
        for file in files:
            if file.lower().endswith(('.png', '.jpg', '.jpeg')):
                image_path = os.path.join(root, file)
                # 이미지와 동일한 이름의 레이블 파일
                label_filename = os.path.splitext(file)[0] + '.txt'
                label_path = os.path.join(root, label_filename)

                if not os.path.exists(label_path):
                    print(f"[Skip] 레이블 파일이 없습니다: {image_path}")
                    continue

                # 레이블 파일 읽기
                with open(label_path, 'r') as f:
                    lines = f.readlines()

                # 레이블 파일이 아예 비어있으면 이동하지 않음.
                if not lines:
                    print(f"[Not Moved] 레이블 파일이 비어있으므로 이동하지 않음: {image_path}")
                    continue

                # 각 라인별로 필터링 진행 (정규화된 값에 고정 이미지 크기를 곱해 계산)
                filtered_lines = []
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) != 5:
                        print(f"[Warning] 예상과 다른 형식 ({label_path}): {line}")
                        continue
                    cls, x, y, w, h = parts
                    try:
                        w_val = float(w)
                        h_val = float(h)
                    except Exception as e:
                        print(f"[Error] 레이블 파싱 실패 ({label_path}): {line} - {e}")
                        continue

                    actual_w = w_val * img_width
                    actual_h = h_val * img_height

                    # 실제 크기가 모두 min_pixel 이상인 경우에만 남김
                    if actual_w >= min_pixel and actual_h >= min_pixel:
                        filtered_lines.append(line)
                    else:
                        print(f"[Filtered Out] {label_path}의 라인 제거됨: {line}")

                # 유효한 라벨 데이터가 하나도 없으면 이동하지 않음.
                if not filtered_lines:
                    print(f"[Not Moved] 유효한 라벨이 없으므로 이동하지 않음: {image_path}")
                    continue

                # 이동할 때 전체 폴더 구조 대신 input_root 바로 아래의 1단계 폴더만 사용
                relative_path = os.path.relpath(root, input_root)
                if relative_path == '.' or relative_path == os.curdir:
                    # input_root에 바로 있는 파일인 경우
                    dest_dir = output_root
                else:
                    # 상대경로의 첫 번째 폴더명만 사용
                    top_level_folder = relative_path.split(os.sep)[0]
                    dest_dir = os.path.join(output_root, top_level_folder)

                os.makedirs(dest_dir, exist_ok=True)

                dest_image_path = os.path.join(dest_dir, file)
                dest_label_path = os.path.join(dest_dir, label_filename)

                try:
                    shutil.move(image_path, dest_image_path)
                except Exception as e:
                    print(f"[Error] 이미지 이동 실패 ({image_path}): {e}")
                    continue

                try:
                    # 필터링된 내용으로 새 레이블 파일 작성
                    with open(dest_label_path, 'w') as f:
                        f.write("\n".join(filtered_lines) + "\n")
                    # 원본 레이블 파일은 삭제
                    os.remove(label_path)
                except Exception as e:
                    print(f"[Error] 레이블 파일 처리 실패 ({label_path}): {e}")
                    continue

                print(f"[Moved] {image_path}와 필터링된 레이블 파일을 {dest_dir}로 이동함.")

if __name__ == '__main__':
    # 원본 데이터가 있는 상위 폴더 경로 (사용 환경에 맞게 수정)
    input_root = r'C:\Users\JMP\Desktop\data\out_data'
    # 이동할 데이터를 저장할 output 폴더 경로 (사용 환경에 맞게 수정)
    output_root = r'C:\Users\JMP\Desktop\data\filter'
    
    move_files_with_filtered_labels(input_root, output_root, min_pixel=20)
