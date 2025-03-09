import rospy
import numpy as np

from geon.features import extract_feature_points
from geon.utils import get_feature_factors, adjust_data_size
from geon.georef2 import process_georef_2

import torch


def process_module_1(points):
    min_points = 5
    constraints = ((3.0, 18.0), (-1.0, 2.0))
    NPC = 10

    cond_result = 0

    feature_points = extract_feature_points(data=points, consts=constraints, min_points=min_points)

    est_x = ext_npc = cond_result = 0
    msg = "Nothing!"

    # condition 1. feature points are not found.
    if (feature_points is None) or (feature_points.shape[0] == 0):
        msg = "featrure ponits are not found!"
        cond_result = 1
    else:
        # condition 2. the # of feature
        avg_val = feature_points[:,-1].mean()
        est_x = feature_points[:,0].mean()
        ext_npc = feature_points.shape[0]
        if ext_npc < NPC:
            #msg = f"the # of feature points is less than {NPC}"
            cond_result = 2
        # condition 3. X coord. is not in 3m < x < 18m
        elif (constraints[0][0] >= est_x) or (constraints[0][1] <= est_x):
            #msg = f"out of detection bound => {est_x}"
            cond_result = 3
        else:
            #msg = f"feature points: est_x = {est_x:.2f}m, {ext_npc = }, avg_intensity = {avg_val:.2f}"
            cond_result = 4

        #log_msg = f"\tcond. {cond_result} : {msg}"
        #rospy.loginfo(log_msg)

    return feature_points, cond_result


def process_module_2(feature_values, model, scaler):
    feature_factors = get_feature_factors(feature_values)
    input_data = np.array([list(feature_factors.values())])
    input_data_scaled = scaler.transform(input_data)

    predict = model.predict(input_data_scaled)

    return predict


def process_module_3(feature_points, model):
    NUM_POINTS = 128

    ft_points_min = np.min(feature_points, axis=0)
    ft_points_max = np.max(feature_points, axis=0)
    norm_ft_points = (feature_points - ft_points_min) / (ft_points_max - ft_points_min)

    adj_norm_data = adjust_data_size(norm_ft_points, num_points=NUM_POINTS)

    #log_msg = f"\tadjusting feature points: {feature_points.shape} -> {adj_norm_data.shape}"
    #rospy.loginfo(log_msg)

    try:
        input_data = adj_norm_data.T
        input_data = input_data[np.newaxis, :]
        input_tensor = torch.tensor(input_data, dtype=torch.float32)
    except Exception as e:
        rospy.logwarn(f"\tError in preparing input data: {str(e)}")

    predicted_class = -1
    try:
        with torch.no_grad():
            output = model(input_tensor)
            predicted_class = torch.argmax(output, dim=1).item()
            #log_msg = f"\tPredicted class: {predicted_class + 1}"
            #ospy.loginfo(log_msg)
    except Exception as e:
        rospy.logerr(f"Error during model inference: {str(e)}")

    return predicted_class + 1


def process_module_4(feature_points):
    avg_point = np.mean(feature_points[:, 0:3], axis=0)
    med_point = np.median(feature_points[:, 0:3], axis=0)
    est_point1, est_point2 = process_georef_2(feature_points[:, 0:3])
    #est_point = process_georef_3(feature_points[:, 0:3])

    #rospy.loginfo(f"{avg_point = }, {med_point =}")
    if est_point1 is not None:
        return est_point1
    else:
        return avg_point
