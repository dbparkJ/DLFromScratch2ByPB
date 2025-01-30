from time import clock_settime

import rosbag
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from datetime import datetime
import math
import numpy as np
import torch
from scipy.stats import mode

# shape파일 용
import pandas as pd
import geopandas as gpd
from shapely.geometry import Point


# -----------------------------
# 설정 부분
bag_file = "/home/han/GEON/KT/2024-12-17-10-57-07.bag"  # rosbag 파일 경로
output_dir = "/home/han/GEON/KT/output_images2"  # 출력 이미지 폴더
yolo_dir = "/home/han/GEON/KT/output_images2/yolo"
#bag_file = "/media/han/01051872512/2024.12.16/2024-12-16-10-22-42.bag"  # rosbag 파일 경로
#output_dir = "/media/han/01051872512/2024.12.16/output_images"  # 출력 이미지 폴더
#yolo_dir = "/media/han/01051872512/2024.12.16/output_images/yolo"

image_topic = "/oak/rgb/image_raw"  # 이미지 토픽
depth_topic = "/oak/stereo/image_raw"
imu_topic = "/imu/data"  # IMU 토픽
gps_topic = "/fix"  # GPS 토픽

yolo_weights = "/home/han/GEON/KT_Python/yolov5m_1280/weights/best.pt"
# -----------------------------


# OAK-D Pro W 카메라 캘리브레이션 값
'''
fx = 835.0  # 초점 거리 (픽셀 단위)
fy = 835.0  # 초점 거리 (픽셀 단위)
cx = 640.0  # 이미지 중심 x 좌표
cy = 360.0  # 이미지 중심 y 좌표
dist_coeffs = np.array([0, 0, 0, 0, 0])  # 왜곡 계수 (왜곡 없음 가정)
'''
fx = 2286.7216796875  # 초점 거리 (픽셀 단위)
fy = 2285.615966796875  # 초점 거리 (픽셀 단위)
cx = 1920.4844970703125 / 2  # 이미지 중심 x 좌표
cy = 1080.3642578125 / 2  # 이미지 중심 y 좌표
camera_matrix = np.array([
    [fx, 0,cx],
    [0, fy, cy],
    [0, 0, 1]
])
dist_coeffs = np.array([13.189817428588867, 6.496435642242432, -0.0006137333111837506, 0.00034765710006468, -2.9563202857917119])


# -----------------------------


# 출력 폴더 생성
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
if not os.path.exists(yolo_dir):
    os.makedirs(yolo_dir)

bridge = CvBridge()

# 데이터 저장용 리스트
image_data = []
depth_data = []
imu_data = []
gps_data = []
detected_objects = []
root_obj = []

# ROSBag 데이터 읽기
print("Reading rosbag file...")
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if topic == image_topic:
            # 이미지 데이터를 OpenCV 형식으로 변환
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_data.append((t.to_sec(), cv_image))
        elif topic == depth_topic:
            # Depth 이미지를 그대로 저장
            cv_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_data.append((t.to_sec(), cv_depth))

        elif topic == imu_topic:
            x = msg.orientation.x
            y = msg.orientation.y
            z = msg.orientation.z
            w = msg.orientation.w
            roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            pitch = math.asin(2.0 * (w * y - z * x))
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            imu_data.append((t.to_sec(), roll, pitch, yaw))

        elif topic == gps_topic:
            gps_data.append((t.to_sec(), msg.latitude, msg.longitude, msg.altitude))

print("Data extraction complete. Processing images...")

# YOLOv5 모델 로드
print("Loading YOLOv5 model...")
model = torch.hub.load('ultralytics/yolov5', 'custom', path=yolo_weights)

# 이미지 왜곡 보정 함수
def undistort_image(image, camera_matrix, dist_coeffs):
    h, w = image.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted_image

# 가장 가까운 IMU 및 GPS 프레임을 찾는 함수
def find_closest_data(target_time, data_list):
    closest = min(data_list, key=lambda x: abs(x[0] - target_time))
    return closest

def find_previous_data(target_time, data_list):
    previous_frames = [data for data in data_list if data[0] < target_time]

    if not previous_frames:
        return None
    return max(previous_frames, key=lambda x: x[0])

# z 값을 방위각(0-360도)으로 변환하는 함수
def convert_to_heading(yaw):
    # Yaw를 0도(북쪽) 기준으로 보정
    heading = (360-math.degrees(yaw)) % 360 # 0~360도로 변환
    return heading

def convert_depth_to_grayscale(depth_img):
    depth_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
    grayscale_img = np.uint8(depth_normalized)
    return grayscale_img

def calculate_gps_heading(lat1, lon1, lat2, lon2):
    # 위도와 경도를 라디안으로 변환
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    d_lon = lon2 - lon1
    x = math.sin(d_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    initial_heading = math.atan2(x, y)
    heading = (math.degrees(initial_heading) + 360) % 360  # 0~360도 변환
    return heading

def calculate_absolute_coordinates(center_x, center_y, depth, roll, pitch, yaw, gps_lat, gps_lon, gps_alt):
    # 중심 좌표를 카메라 프레임에서 월드 프레임으로 변환
    # depth는 카메라에서 박스까지의 거리 (m)
    # roll, pitch, yaw는 라디안 값
    fx = 2286.7216796875  # 초점 거리 (픽셀 단위)
    fy = 2285.615966796875  # 초점 거리 (픽셀 단위)
    cx = 1920.4844970703125 / 2  # 이미지 중심 x 좌표
    cy = 1080.3642578125 / 2  # 이미지 중심 y 좌표

    # 픽셀 좌표 → 카메라 좌표 변환
    X_camera = (center_x - cx) * depth / fx
    Y_camera = (center_y - cy) * depth / fy
    Z_camera = depth

    # 카메라 좌표 → 월드 좌표 변환
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]])
    R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                    [0, 1, 0],
                    [-math.sin(pitch), 0, math.cos(pitch)]])
    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])

    rotation_matrix = np.dot(R_z, np.dot(R_y, R_x))
    camera_coords = np.array([X_camera, Y_camera, Z_camera])
    world_coords = np.dot(rotation_matrix, camera_coords)

    X_world, Y_world, Z_world = world_coords

    # GPS 좌표에 변환된 값을 더해 절대 좌표 계산
    absolute_lat = gps_lat + (Y_world / 111320)  # 1미터당 위도 변경량
    absolute_lon = gps_lon + (X_world / (40075000 * math.cos(math.radians(gps_lat)) / 360))  # 1미터당 경도 변경량
    absolute_alt = gps_alt + Z_world

    return absolute_lat, absolute_lon, absolute_alt




def detect_and_draw_boxes(image, gps_data, imu_data, depth_image):
    results = model(image)
    detections = results.xyxy[0].cpu().numpy()  # 탐지된 객체

    if detections.shape[0] == 0:
        return image, None, None, None, None, None, depth_image

    for *box, conf, cls in detections:
        x1, y1, x2, y2 = map(int, box)
        label = f"{int(cls)} {conf:.2f}"
        # RGB 이미지에 박스 그리기
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        print("XXXXXXXXXXXXXX", center_x, "////////////", center_y)
        d_value = depth_image[center_y, center_x]
        # Depth를 5m로 일괄 설정
        if d_value == 0 or np.isnan(d_value) or np.isinf(d_value):
            depth = 1
        else:
            depth = 882.5 * 7.5 / float(d_value)  # 숫자 계산

        depth_normal = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colormap = cv2.applyColorMap(depth_normal, cv2.COLORMAP_JET)

        # Depth 이미지에 박스 그리기
        #box_depth = depth_image[y1:y2, x1:x2]
        #avg_depth = np.mean(box_depth) if box_depth.size > 0 else 0
        #avg_depth_label = f"{avg_depth:.2f}m"
        cv2.rectangle(depth_colormap, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #cv2.putText(depth_image, avg_depth_label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)



        # Depth 이미지의 박스 내 평균값 계산 및 RGB에 출력
        # Depth 이미지의 박스 내 중심값 및 최빈값 계산
        #box_depth = depth_image[y1:y2, x1:x2]
        #center_depth = box_depth[box_depth.shape[0] // 2, box_depth.shape[
        #    1] // 2] / 1000 if box_depth.size > 0 else 0  # mm → m 변환
        """
        mode_depth = mode(box_depth.flatten())[0][0] / 1000 if box_depth.size > 0 else 0  # mm → m 변환
        depth_label = f"C:{center_depth:.2f}m M:{mode_depth:.2f}m"
        cv2.putText(image, depth_label, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        """

        # 박스 중심 좌표 계산


        print("SSSSSSSSSSSSSSSSSSSSS", depth)
        depth_text = f"{depth:.2f}m"
        cv2.putText(image, depth_text, (x1, y2 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)



        # 가장 가까운 GPS 및 IMU 데이터 가져오기
        closest_gps = find_closest_data(img_time, gps_data)
        closest_imu = find_closest_data(img_time, imu_data)

        #print(closest_imu)

        roll, pitch, yaw = closest_imu[1], closest_imu[2], closest_imu[3]
        gps_lat, gps_lon, gps_alt = closest_gps[1], closest_gps[2], closest_gps[3]

        # 절대 좌표 계산
        abs_lat, abs_lon, abs_alt = calculate_absolute_coordinates(
            center_x, center_y, float(depth), roll, pitch, yaw, gps_lat, gps_lon, gps_alt
        )

        abs_coords_label = f"Lat:{abs_lat:.6f} Lon:{abs_lon:.6f} Alt:{abs_alt:.2f}"
        cv2.putText(image, abs_coords_label, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)



    #return image, depth_image
    return image, int(cls), float(conf), abs_lat, abs_lon, abs_alt, depth_colormap


# 이미지 저장
for img_time, img in image_data:
    # 가장 가까운 IMU 및 GPS 데이터 찾기
    closest_imu = find_closest_data(img_time, imu_data)
    closest_gps = find_closest_data(img_time, gps_data)
    closest_depth = find_closest_data(img_time, depth_data)


    imu_heading = convert_to_heading(closest_imu[3])
    imu_time = closest_imu[0]
    gps_x = closest_gps[1]
    gps_y = closest_gps[2]
    gps_z = closest_gps[3]
    gps_time = closest_gps[0]

    previous_gps = find_previous_data(gps_time, gps_data)
    # GPS 기반 방위각 계산 (이전 GPS 데이터가 있을 때만)
    if previous_gps:
        gps_heading = calculate_gps_heading(previous_gps[1], previous_gps[2], gps_x, gps_y)
    else:
        gps_heading = 0.0  # 이전 GPS 데이터가 없으면 0으로 설정

    # 파일명 생성
    filename_rgb = f"{img_time:.6f}_{imu_heading:.3f}_{gps_heading:.3f}_{imu_time:.6f}_{gps_x:.6f}_{gps_y:.6f}_{gps_time:.6f}.jpg"
    filename_depth = f"{img_time:.6f}_{imu_heading:.3f}_{gps_heading:.3f}_{imu_time:.6f}_{gps_x:.6f}_{gps_y:.6f}_{gps_time:.6f}_D.jpg"
    filepath_rgb = os.path.join(output_dir, filename_rgb)
    filepath_depth = os.path.join(output_dir, filename_depth)


    # 왜곡 보정
    re_img = cv2.rotate(img, cv2.ROTATE_180).copy()
    undistorted_img = undistort_image(re_img,camera_matrix, dist_coeffs)
    # YOLOv5 탐지 및 박스 그리기
    #detected_img, detected_depth = detect_and_draw_boxes(cv2.rotate(img, cv2.ROTATE_180).copy(), closest_depth[1])
    detected_img, cls, conf, abs_lat, abs_lon, abs_alt, detected_depth = detect_and_draw_boxes(re_img, gps_data, imu_data, closest_depth[1].copy())

    if cls is not None :
        #탐지 shape 만들기
        detected_objects.append({
            "Time": img_time,
            "ClassID": int(cls),
            "Confidence": float(conf),
            "Latitude": abs_lat,
            "Longitude": abs_lon,
            "Altitude": abs_alt
        })

    root_obj.append({
        "Time": img_time,
        "Latitude": gps_x,
        "Longitude": gps_y,
        "Altitude": gps_z
    })

    # YOLO 탐지된 이미지 저장
    yolo_rgb_path = os.path.join(output_dir, "yolo", "Y_" + filename_rgb)
    yolo_depth_path = os.path.join(output_dir, "yolo", "Y_" + filename_depth)
    cv2.imwrite(yolo_rgb_path, detected_img)
    cv2.imwrite(yolo_depth_path, detected_depth)
    #print(f"YOLO Saved: {yolo_rgb_path}, {yolo_depth_path}")

    # 이미지 저장
    cv2.imwrite(filepath_rgb, cv2.rotate(img, cv2.ROTATE_180))
    #print(f"Saved: {filename_rgb}")

    #if closest_depth:
    #    cv2.imwrite(filepath_depth,  convert_depth_to_grayscale(cv2.rotate(closest_depth, cv2.ROTATE_180)))

df = pd.DataFrame(detected_objects)
root_df = pd.DataFrame(root_obj)
csv_path = os.path.join(output_dir, "detected_objects.csv")
root_path = os.path.join(output_dir, "root_objects.csv")
df.to_csv(csv_path, index=False)
root_df.to_csv(root_path, index=False)
print(f"CSV 파일 저장 완료: {csv_path}")

# 2. Shapefile 생성
gdf = gpd.GeoDataFrame(
    df, geometry=[Point(lon, lat) for lat, lon in zip(df["Latitude"], df["Longitude"])],
    crs="EPSG:4326"  # WGS84 좌표계
)
r_gdf = gpd.GeoDataFrame(
    root_df, geometry=[Point(lon, lat) for lat, lon in zip(root_df["Latitude"], root_df["Longitude"])],
    crs="EPSG:4326"  # WGS84 좌표계
)
shapefile_path = os.path.join(output_dir, "detected_objects.shp")
gdf.to_file(shapefile_path, driver="ESRI Shapefile")

r_shapefile_path = os.path.join(output_dir, "root_objects.shp")
r_gdf.to_file(r_shapefile_path, driver="ESRI Shapefile")


print("Image processing complete. Files saved in:", output_dir)
