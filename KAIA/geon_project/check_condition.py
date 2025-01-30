#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import datetime

import os
import numpy as np

from geon.processings import process_module_2
from geon.utils import load_model1, load_scaler1

QUEUE_SIZE = 10

def callback(cloud, model, scaler):
    seq = cloud.header.seq

    pcd = pc2.read_points(cloud, skip_nans=True, field_names = ("x", "y", "z", "intensity"))

    feature_points = np.array([[x, y, z, intensity] for x, y, z, intensity in pcd])
    #rospy.loginfo(f"[{seq}] received points: {feature_points.shape}")

    cond = process_module_2(feature_points[:,-1], model, scaler)
    rospy.loginfo(f"[#3]-[{seq}] results of checked condition: {feature_points.shape} -> {cond}")

    if cond == 1:
        pub.publish(cloud)


def points_subscriber(model, scaler):
    global pub

    rospy.init_node('check_condition', anonymous=True)
    rospy.Subscriber('/features', PointCloud2, lambda msg: callback(msg, model, scaler))
    pub = rospy.Publisher('features_1', PointCloud2, queue_size=QUEUE_SIZE)
    rospy.spin()


if __name__ == "__main__":
    model_file = os.path.join(os.path.dirname(__file__), "../../../lib/stat_model.joblib")
    scaler_file = os.path.join(os.path.dirname(__file__), "../../../lib/stat_model_scaler.joblib")

    model = load_model1(model_file)
    scaler = load_scaler1(scaler_file)

    if any(x is None for x in [model, scaler]):
        rospy.logerr("**** Model or Scaler loading failed! ****")
        exit

    try:
        points_subscriber(model, scaler)
    except rospy.ROSInterruptException:
        pass
