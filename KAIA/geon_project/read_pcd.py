#!/usr/bin/env python3

import rospy
import os
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Header
from pathlib import Path

from contextlib import suppress

QUEUE_SIZE = 10

def find_files(in_dir):
    in_dir = Path(in_dir)
    print(f"{in_dir = }")

    for dir in in_dir.glob('**/'):
        if dir.is_dir():
            file_found = False
            for file in dir.glob("*.pcd"):
                file_found = True
                yield file

            if not file_found:
                rospy.logwarn(f"No pcd files in dir:{dir}")


def read_pcd(file_path):
    with suppress(FileNotFoundError):
        with open(file_path, 'r') as f:
            lines = f.readlines()

        data_start_idx = 0
        for i, line in enumerate(lines):
            if line.startswith('DATA'):
                data_start_idx = i + 1
                break
        data = [list(map(float, line.strip().split())) for line in lines[data_start_idx:]]

        return data


def main(pcd_dir):
    # topic name: pcd
    pub = rospy.Publisher('points', PointCloud2, latch=True, queue_size=QUEUE_SIZE)
    rospy.init_node('read_pcd', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]

    header = Header()
    header.seq = 0

    file_generator = find_files(pcd_dir)

    while not rospy.is_shutdown():
        try:
            pcd_file = next(file_generator)
            points = read_pcd(pcd_file)
            if len(points) == 0:
                rospy.logwarn(f"points of {pcd_file}: is None")
                continue

            header.seq += 1
            header.stamp = rospy.Time.now()
            header.frame_id ="points"

            cloud_msg = pc2.create_cloud(header, fields, points)
            pub.publish(cloud_msg)

            rospy.loginfo(f"[#1] Read file: {Path(pcd_file).name}, points: {len(points)}")

        except StopIteration:
            rospy.logerr("No more files to publish")
            break
        except Exception as e:
            rospy.logerr(f"Error processing {pcd_file}:{e}")

        rate.sleep()


if __name__ == "__main__":
    try:
        #pcd_dir = "../pcd32_sample"
        pcd_dir = os.path.join(os.path.dirname(__file__), "../../../pcd32_sample")
        main(pcd_dir)
    except rospy.ROSInterruptException:
        pass




