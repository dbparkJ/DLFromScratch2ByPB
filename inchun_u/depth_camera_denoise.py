import cv2
import numpy as np

import matplotlib.pyplot as plt


def sobel_smooth_filter(depth_image):
    # egde detection using sobel
    sobel_x = cv2.Sobel(depth_image, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(depth_image, cv2.CV_64F, 0, 1, ksize=3)

    # calculate the intensity of the edge
    edges = cv2.magnitude(sobel_x, sobel_y)

    # using average filter to smooth the edge images
    kernel = np.ones((5, 5), np.float32) / 25
    mean_filtered_edges = cv2.filter2D(edges, -1, kernel)

    return mean_filtered_edges



# Using non-loca mean to denoise
def nlm_filter(depth_image):
    nlm_filtered_depth = cv2.fastNlMeansDenoising(depth_image, None, 30, 7, 21)
    return nlm_filtered_depth


def interpolate_zero(depth_image):

    # using the average pixel of the surrounding pixels
    # 使用临近像素进行插值（近邻插值或者填充方法）
    depth_image[depth_image == 0] = np.nan  # setting 0 to Nan
    nan_mask = np.isnan(depth_image)

    # interpolate the zero points with nearby pixels
    depth_image = cv2.inpaint(depth_image.astype(np.float32), nan_mask.astype(np.uint8), 3, cv2.INPAINT_TELEA)

    return depth_image




# to fit the linear compensate model
def compensate_depth_simple(depth_image):
    a = 1.2  # 系数a
    b = 5    # 系数b
    compensated_depth = a * depth_image + b
    return compensated_depth


# # to fit the poly compensate model
# def compensate_depth_poly(depth_image):
#     real_depth = np.array([100, 200, 300, 400, 500])  # real d（单位：mm）
#     sensor_depth = np.array([105, 210, 310, 420, 520])  # 传感器测得的深度（单位：mm）
#
#     # 拟合一个多项式模型，拟合度数为2的多项式
#     coeffs = np.polyfit(sensor_depth, real_depth, 2)  # polyfit返回的系数是从高到低的
#     compensated_depth = np.polyval(coeffs, depth_image)  # 使用拟合的多项式模型进行补偿
#     return compensated_depth

depth_image = cv2.imread('depth_image.png', cv2.IMREAD_UNCHANGED)


compensated_depth = compensate_depth_simple(depth_image)


