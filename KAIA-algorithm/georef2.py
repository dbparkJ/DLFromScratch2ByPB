import numpy as np
import os

def define_sym_type(points):
    Y_min = np.min(points[:, 1])
    Y_max = np.max(points[:, 1])
    Z_min = np.min(points[:, 2])
    Z_max = np.max(points[:, 2])

    Y_axis_length = Y_max - Y_min
    Z_axis_length = Z_max - Z_min

    if Y_axis_length < 0.25 and Z_axis_length < 0.4:
        sym_type = 'F'
    else:
        z_values = points[:, 2]
        lower_bound_threshold = Z_min + 0.3 * Z_axis_length
        upper_bound_threshold = Z_max - 0.3 * Z_axis_length

        lower_bound_count = np.sum(z_values < lower_bound_threshold)
        upper_bound_count = np.sum(z_values > upper_bound_threshold)

        total_points = len(points)
        lower_bound_ratio = lower_bound_count / total_points
        upper_bound_ratio = upper_bound_count / total_points

        is_sym = upper_bound_ratio - lower_bound_ratio

        if abs(is_sym) <= 0.15:
            sym_type = 'S'
        elif is_sym < 0 and abs(is_sym) > 0.15:
            sym_type = 'U'
        elif is_sym > 0 and is_sym > 0.15:
            sym_type = 'L'

    return sym_type


def find_Y_min_max_point(points):
    all_points = np.vstack(points)

    max_Y_point = all_points[np.argmax(all_points[:, 1])]
    min_Y_point = all_points[np.argmin(all_points[:, 1])]

    return max_Y_point, min_Y_point


def list_YZ_lines(points, z_threshold, y_threshold):
    minY, maxY = np.min(points[:, 1]), np.max(points[:, 1])
    minYZ_lines, maxYZ_lines = [], []
    checked = np.zeros(len(points), dtype=bool)

    for i, base_point in enumerate(points):
        if checked[i]:
            continue

        group = [base_point]
        checked[i] = True

        for j, point in enumerate(points):
            if checked[j]:
                continue
            if abs(point[2] - base_point[2]) <= z_threshold:
                group.append(point)
                checked[j] = True

        if len(group) >= 2:
            group = np.array(group)
            min_y_point = group[np.argmin(group[:, 1])]
            max_y_point = group[np.argmax(group[:, 1])]

            line = np.array([min_y_point, max_y_point])
            if np.any((group[:, 1] >= minY - y_threshold) & (group[:, 1] <= minY + y_threshold)):
                minYZ_lines.append(line)
            if np.any((group[:, 1] >= maxY - y_threshold * 1.5) & (group[:, 1] <= maxY + y_threshold * 1.5)):
                maxYZ_lines.append(line)

    return minYZ_lines, maxYZ_lines


def cal_y_axis_longest_line(line1, line2, Y_max_point, Y_min_point):
    direction = line2 - line1
    direction /= np.linalg.norm(direction)

    min_point = line1 + direction * (Y_min_point[1] - line1[1])
    max_point = line2 + direction * (Y_max_point[1] - line2[1])

    return np.array([min_point, max_point])


def Y_Axis_tran(points, rotation_axis, rotation_center):
    #line_points = np.array(rotation_axis)
    line_points = rotation_axis

    if rotation_center == 'U':
        Y_rot_index = np.argmin(line_points[:, 1])
        Y_rot_point = line_points[Y_rot_index]
        rotation_axis = line_points[1] - line_points[0]
    elif rotation_center == 'L':
        Y_rot_index = np.argmax(line_points[:, 1])
        Y_rot_point = line_points[Y_rot_index]
        rotation_axis = line_points[0] - line_points[1]
    elif rotation_center == 'S':
        Y_rot_point = np.median(line_points, axis=0)
        rotation_axis = line_points[0] - line_points[1]
    else:
        raise ValueError("회전 중심점을 확인하세요!")

    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    center_of_rotation = Y_rot_point
    rotation_angle = np.pi
    R_z = np.array([
        [np.cos(rotation_angle), -np.sin(rotation_angle), 0],
        [np.sin(rotation_angle),  np.cos(rotation_angle), 0],
        [0, 0, 1]
    ])

    T = np.eye(4)
    T[:3, :3] = R_z
    T[:3, 3] = center_of_rotation - np.dot(R_z, center_of_rotation)

    points_h = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed_points = (T @ points_h.T).T[:, :3]

    return transformed_points


def Z_Axis_tran(points, rotation_axis, rotation_center):
    #line_points = np.array(rotation_axis)
    line_points = rotation_axis

    if rotation_center == 'U':
        Y_rot_index = np.argmin(line_points[:, 1])
        Y_rot_point = line_points[Y_rot_index]
        rotation_axis = line_points[1] - line_points[0]
    elif rotation_center == 'L':
        Y_rot_index = np.argmax(line_points[:, 1])
        Y_rot_point = line_points[Y_rot_index]
        rotation_axis = line_points[0] - line_points[1]
    elif rotation_center == 'S':
        Y_rot_point = np.median(line_points, axis=0)
        rotation_axis = line_points[0] - line_points[1]
    else:
        raise ValueError("회전 중심점을 확인하세요!")

    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    center_of_rotation = Y_rot_point
    rotation_angle = np.pi
    R_z = np.array([
        [np.cos(rotation_angle), 0, np.sin(rotation_angle)],
        [0, 1, 0],
        [-np.sin(rotation_angle), 0, np.cos(rotation_angle)]
    ])

    """
    R_z = np.array([
        [np.cos(rotation_angle), 0, -np.sin(rotation_angle)],
        [0, 1, 0],
        [np.sin(rotation_angle), 0, np.cos(rotation_angle)]
    ])
    """
    points_h = np.hstack([points, np.ones((points.shape[0], 1))])
    T = np.eye(4)
    T[:3, :3] = R_z
    T[:3, 3] = center_of_rotation - np.dot(R_z, center_of_rotation)

    transformed_points = (T @ points_h.T).T[:, :3]

    return transformed_points


"""
def ob_c3_sym(points):
    aabb_center = np.mean(points, axis=0)
    obb_center = np.median(points, axis=0)
    return aabb_center, obb_center
"""

def ob_c3_sym(points):
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    aabb_center = (min_vals + max_vals) / 2

    obb_center = np.median(points, axis=0)
    #obb_center = np.mean(points, axis=0)

    return aabb_center, obb_center


def are_lines_equal(line1, line2, tolerance=1e-6):
    line1_points = np.asarray(line1.points)
    line2_points = np.asarray(line2.points)

    return (
        np.allclose(line1_points[0], line2_points[0], atol=tolerance) and
        np.allclose(line1_points[1], line2_points[1], atol=tolerance)
    ) or (
        np.allclose(line1_points[0], line2_points[1], atol=tolerance) and
        np.allclose(line1_points[1], line2_points[0], atol=tolerance)
    )


def cal_rotation_axis_YZ_lines(minYZ_lines, maxYZ_lines, points, sym_type):
    Y_max_point = points[np.argmax(points[:, 1])]
    Y_min_point = points[np.argmin(points[:, 1])]

    minY_maxZ_line = max(minYZ_lines, key=lambda l: np.max(l[:, 2]), default=None)
    maxY_minZ_line = min(maxYZ_lines, key=lambda l: np.min(l[:, 2]), default=None)

    if maxY_minZ_line is None and minY_maxZ_line is None:
        print("Error: 유효한 회전 축을 찾을 수 없습니다.")
        return None, None

    elif maxY_minZ_line is None:
        r_axis = minY_maxZ_line
        r_center = 'L'
    elif minY_maxZ_line is None:
        r_axis = maxY_minZ_line
        r_center = 'U'
    else:
        if np.array_equal(maxY_minZ_line, minY_maxZ_line):
            r_axis = np.array([Y_min_point, Y_max_point])
            r_center = 'S'
        else:
            r_axis = maxY_minZ_line
            r_center = 'U'

    return r_axis, r_center


def process_georef_2(points):
    sym_type = define_sym_type(points)

    minYZ_lines, maxYZ_lines = list_YZ_lines(points, 0.01, 0.05)

    rotation_axis, rotation_center = cal_rotation_axis_YZ_lines(minYZ_lines, maxYZ_lines, points, sym_type)

    if (rotation_axis is not None) and (rotation_center is not None):
        rotated_points_y = Y_Axis_tran(points, rotation_axis, rotation_center)

        temp_points = np.vstack([points, rotated_points_y])
        rotated_points_z = Z_Axis_tran(temp_points, rotation_axis, rotation_center)

        sym_points = np.vstack([points, rotated_points_z])
        aab_center, ob_center = ob_c3_sym(sym_points)

        #print(f"\t{aab_center = }\n\t{ob_center = }")
        return aab_center, ob_center
        #return aab_center, None
    else:
        return None, None

