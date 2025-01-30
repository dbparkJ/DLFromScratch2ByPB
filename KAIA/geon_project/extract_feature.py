#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Header
import datetime
import numpy as np

from geon.processings import process_module_1

QUEUE_SIZE = 10

def callback(cloud):
    seq = cloud.header.seq
    #stamp = cloud.header.stamp.secs
    #width = cloud.width

    pcd = pc2.read_points(cloud, skip_nans=True, field_names = ("x", "y", "z", "intensity"))

    points = np.array([[x, y, z, intensity] for x, y, z, intensity in pcd])
    #rospy.loginfo(f"[seq:{seq}] received points: {points.shape}")

    feature_points, cond = process_module_1(points)
    #rospy.loginfo(f"\t{feature_points.shape = }, {cond = }")

    if cond == 4:
        feature_points_list = feature_points.tolist()

        new_header = Header()
        new_header.seq = cloud.header.seq + 1
        new_header.stamp = rospy.Time.now()
        new_header.frame_id = "feature_points"

        cloud_msg = pc2.create_cloud(new_header, [
            pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
            pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
            pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
            pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1)
        ], feature_points_list)

        pub.publish(cloud_msg)
        rospy.loginfo(f"[#2]-[{seq}] feature points: {feature_points.shape}")


def points_subscriber():
    global pub

    rospy.init_node('extract_feature', anonymous=True)
    rospy.Subscriber('/points', PointCloud2, callback)              # from pcd files
    #rospy.Subscriber('/velodyne_points', PointCloud2, callback)    # from Velodyhe 32C
    pub = rospy.Publisher('features', PointCloud2, queue_size=QUEUE_SIZE)
    rospy.spin()


if __name__ == "__main__":
    try:
        points_subscriber()
    except rospy.ROSInterruptException:
        pass
