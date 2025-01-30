#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import datetime
import numpy as np

from geon.processings import process_module_4

def callback(cloud):
    seq = cloud.header.seq

    pcd = pc2.read_points(cloud, skip_nans=True, field_names = ("x", "y", "z", "intensity"))

    points = np.array([[x, y, z, intensity] for x, y, z, intensity in pcd])

    coord = process_module_4(points)

    rospy.loginfo(f"[#5]-[{seq}] =====@=> estimate coords: {coord}")


def points_subscriber():
    rospy.init_node('estimate_georef', anonymous=True)
    rospy.Subscriber('/features_c3', PointCloud2, callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        points_subscriber()
    except rospy.ROSInterruptException:
        pass
