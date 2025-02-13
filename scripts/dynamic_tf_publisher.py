#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

def publish_transforms():
    # 创建一个ROS节点
    rospy.init_node('dynamic_tf_publisher')

    # 创建 tf2 转换器、广播器
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()

    # 设置发布频率
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        try:
            # 构建从 odom 到 base_link 的变换
            transform_odom_to_base = geometry_msgs.msg.TransformStamped()
            transform_odom_to_base.header.stamp = rospy.Time.now()
            transform_odom_to_base.header.frame_id = "odom"
            transform_odom_to_base.child_frame_id = "base_link"
            transform_odom_to_base.transform.translation.x = 0.0
            transform_odom_to_base.transform.translation.y = 0.0
            transform_odom_to_base.transform.translation.z = 0.0
            transform_odom_to_base.transform.rotation.x = 0.0
            transform_odom_to_base.transform.rotation.y = 0.0
            transform_odom_to_base.transform.rotation.z = 0.0
            transform_odom_to_base.transform.rotation.w = 1.0

            # 广播从 odom 到 base_link 的变换
            broadcaster.sendTransform(transform_odom_to_base)

            # 构建从 base_link 到 cloud 的变换
            transform_base_to_cloud = geometry_msgs.msg.TransformStamped()
            transform_base_to_cloud.header.stamp = rospy.Time.now()
            transform_base_to_cloud.header.frame_id = "base_link"
            transform_base_to_cloud.child_frame_id = "cloud"
            transform_base_to_cloud.transform.translation.x = 0.0
            transform_base_to_cloud.transform.translation.y = 0.0
            transform_base_to_cloud.transform.translation.z = 0.0
            transform_base_to_cloud.transform.rotation.x = 0.0
            transform_base_to_cloud.transform.rotation.y = 0.0
            transform_base_to_cloud.transform.rotation.z = 0.0
            transform_base_to_cloud.transform.rotation.w = 1.0

            # 广播从 base_link 到 cloud 的变换
            broadcaster.sendTransform(transform_base_to_cloud)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF Lookup failed.")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transforms()
    except rospy.ROSInterruptException:
        pass
