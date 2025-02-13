#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl
from pcl import PointCloud
import std_msgs.msg

# 回调函数处理 velodyne_points 点云
def point_cloud_callback(msg):
    # 发布新的点云消息
    cloud_pub.publish(msg)
    # rospy.loginfo("Published transformed cloud message.")

# 主函数
def main():
    rospy.init_node('velodyne_to_cloud', anonymous=True)

    # 订阅 velodyne_points 点云消息
    rospy.Subscriber("/velodyne_points", PointCloud2, point_cloud_callback)

    # 发布转换后的点云消息
    global cloud_pub
    cloud_pub = rospy.Publisher("/cloud", PointCloud2, queue_size=10)

    rospy.loginfo("velodyne_points to cloud node is running...")
    rospy.spin()

if __name__ == "__main__":
    main()
