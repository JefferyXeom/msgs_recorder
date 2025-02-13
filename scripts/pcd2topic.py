#!/usr/bin/env python3

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import pcl.pcl_visualization
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

def load_pcd_and_publish(pcd_file, topic_name):
    # 创建 ROS 节点
    rospy.init_node('pcd_to_pointcloud2', anonymous=True)

    # 创建 PointCloud2 发布者
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

    # 加载 PCD 文件
    cloud = pcl.load(pcd_file)

    # 发布消息
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # 将 PCL 点云转换为 PointCloud2
        pc2_data = pcl_to_ros(cloud)

        pub.publish(pc2_data)
        rate.sleep()

def pcl_to_ros(pcl_cloud):
    """
    将 PCL 点云对象转换为 ROS 点云消息 (sensor_msgs/PointCloud2)
    """
    # 获取 PCL 点云的数据
    pcl_data = pcl_cloud.to_array()

    # 创建 PointCloud2 消息
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "cloud"  # 使用合适的坐标系

    # 将 PCL 数据转换为 PointCloud2 格式
    pc2_msg = pc2.create_cloud_xyz32(header, pcl_data)

    return pc2_msg

if __name__ == '__main__':
    # 这里指定你想发布的 PCD 文件路径和话题名称
    pcd_file = "/home/d1/datasets/Underground/test_frames/pcd/"  
    # pcd_file += "2025-02-04-16-15-11/scans_334.pcd"
    pcd_file += "2025-02-04-16-15-11/scans_334.pcd"
    topic_name = "/cloud"  # 替换为你想发布的 ROS 话题名称

    try:
        load_pcd_and_publish(pcd_file, topic_name)
    except rospy.ROSInterruptException:
        pass
