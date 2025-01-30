#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import datetime

import os
import numpy as np

from geon.processings import process_module_3
from geon.utils import load_model2

QUEUE_SIZE = 10

def callback(cloud, model):
    seq = cloud.header.seq

    pcd = pc2.read_points(cloud, skip_nans=True, field_names = ("x", "y", "z", "intensity"))

    feature_points = np.array([[x, y, z, intensity] for x, y, z, intensity in pcd])
    #rospy.loginfo(f"[{seq}] received points: {feature_points.shape}")

    class_id = process_module_3(feature_points, model)
    avg_axis = np.mean(feature_points[:,0:3], axis=0)
    avg_str = f"({avg_axis[0]:.2f}, {avg_axis[1]:.2f}, {avg_axis[2]:.2f})"
    rospy.loginfo(f"[#4]-[{seq}] ===@> prediction result: class C{class_id} @ {avg_str}")

    if class_id == 3:
        pub.publish(cloud)


def points_subscriber(model):
    global pub

    rospy.init_node('check_condition', anonymous=True)
    rospy.Subscriber('/features_1', PointCloud2, lambda msg: callback(msg, model))
    pub = rospy.Publisher('features_c3', PointCloud2, queue_size=QUEUE_SIZE)
    rospy.spin()


if __name__ == "__main__":
    model_file = os.path.join(os.path.dirname(__file__), "../../../lib/geon_model_norm_T.pth")

    model = load_model2(model_file)
    if model is None:
        rospy.logerr("**** model loading failed!! ****")
        exit

    try:
        points_subscriber(model)
    except rospy.ROSInterruptException:
        pass
