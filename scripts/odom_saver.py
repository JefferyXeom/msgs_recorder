#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

# 保存路径
SAVE_PATH = "/home/d1/datasets/gazebo/test_frames/path/path.txt"
index = 1  # 关键帧计数，从 1 开始

# 订阅回调函数
def odometry_callback(msg):
    global index
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    # 追加写入文件
    with open(SAVE_PATH, 'a') as file:
        file.write(f"{index} {x:.6f} {y:.6f} {z:.6f} {qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}\n")

    rospy.loginfo(f"Saved Odometry #{index}: Pos=({x:.3f}, {y:.3f}, {z:.3f}), Quat=({qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f})")

    index += 1  # 递增索引

# 主函数
def main():
    rospy.init_node('save_odometry_txt_node', anonymous=True)
    rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, odometry_callback)
    rospy.loginfo(f"Listening to /lio_sam/mapping/odometry and saving to {SAVE_PATH} ...")
    rospy.spin()

if __name__ == "__main__":
    main()
