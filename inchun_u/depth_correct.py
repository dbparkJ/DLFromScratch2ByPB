import numpy as np
import math
from imu_angle import mahony_filter


def euler_to_rotation_matrix(roll, pitch, yaw):
    """change the roll pitch yaw value to rotated matrix"""
    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    R_y = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    R_z = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    R = R_z @ R_y @ R_x
    return R


def correct_depth_vectorized(depth_map, K, roll, pitch, yaw):
    h, w = depth_map.shape

    # create coordinate grid
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    # depth value
    Z_c = depth_map
    f_x, f_y = K[0, 0], K[1, 1]
    c_x, c_y = K[0, 2], K[1, 2]

    # calculate the values in the camera coordinate
    X_c = (u - c_x) * Z_c / f_x
    Y_c = (v - c_y) * Z_c / f_y

    R = euler_to_rotation_matrix(roll, pitch, yaw)

    # change the camera coordinate to world coordinate
    points_camera = np.stack((X_c, Y_c, Z_c), axis=-1).reshape(-1, 3)  # (h*w, 3)
    points_world = points_camera @ R.T  # (h*w, 3)

    # correct depth value
    corrected_depth = points_world[:, 2].reshape(h, w)

    return corrected_depth


# examples
depth_map = np.ones((480, 640)) * 2.0
K = np.array([[525, 0, 320], [0, 525, 240], [0, 0, 1]])  # intrinsic matrix
angular_velocity = [0.01, 0.02, 0.03]
linear_acceleration = [0.0, 0.0, -9.81]
dt = 0.01  # time stamp
q = [1.0, 0.0, 0.0, 0.0] #orientation value
roll, pitch, yaw = mahony_filter(angular_velocity, linear_acceleration, q)
# 修正深度图
corrected_depth = correct_depth_vectorized(depth_map, K, roll, pitch, yaw)
print("修正后的深度图：", corrected_depth)
