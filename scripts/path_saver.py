#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path

def path_callback(msg):
    # with open("/home/d1/datasets/Underground/test_frames/path/path_t.txt", "w") as file:
    with open("/home/d1/datasets/gazebo/test_frames/path/path_t.txt", "w") as file:
        for pose in msg.poses:
            timestamp = pose.header.stamp.to_sec()  # 获取时间戳（秒）
            x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            file.write(f"{timestamp:20.8f} {x:24.18f} {y:24.18f} {z:24.18f}\n")
    # with open("/home/d1/datasets/Underground/test_frames/path/path.txt", "w") as file:
    with open("/home/d1/datasets/gazebo/test_frames/path/path.txt", "w") as file:
        for pose in msg.poses:
            x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            file.write(f"{x:24.18f} {y:24.18f} {z:24.18f}\n")
    rospy.loginfo("Path saved to /home/d1/datasets/Underground/test_frames/path/path_t.txt(path.txt).")

if __name__ == "__main__":
    rospy.init_node("path_saver")
    rospy.Subscriber("/path", Path, path_callback)
    rospy.spin()
