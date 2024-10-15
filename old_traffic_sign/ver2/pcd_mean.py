import open3d as o3d
import numpy as np
import os

def calculate_average_intensity(pcd_file):
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)
    # Assuming intensity is the fourth column in the PCD file
    if points.shape[1] > 3:
        intensities = points[:, 3]
        average_intensity = np.mean(intensities)
        return average_intensity
    else:
        print(f"The PCD file {pcd_file} does not contain intensity information.")
        return None

# PCD 파일이 저장된 폴더 경로
folder_path = r"/old_traffic_sign/ver2/pcd_file"

# 폴더 내 모든 PCD 파일을 처리
for filename in os.listdir(folder_path):
    if filename.endswith(".pcd"):
        file_path = os.path.join(folder_path, filename)
        avg_intensity = calculate_average_intensity(file_path)
        if avg_intensity is not None:
            print(f"Average Intensity for {filename}: {avg_intensity}")











