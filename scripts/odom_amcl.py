#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist
import tf

class FakeOdomPublisher:
    def __init__(self):
        rospy.init_node('fake_odom_publisher', anonymous=True)
        self.odom_pub = rospy.Publisher('/odom_2', Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)  # 10 Hz
        self.t = 0  # 时间计数器
        self.max_displacement = 0.01  # 最大位移扰动 (单位: 米)
        self.max_rotation = 0.005  # 最大角度扰动 (单位: 弧度)
        self.publish_fake_odom()


    def publish_fake_odom(self):
        while not rospy.is_shutdown():
            
            if self.t % 2 == 0:
                displacement = 0.01
                # rotation = 0.001
                x_offset = 0.0
                theta_offset = 0.0
            else:
                displacement = -0.01
                # rotation = -0.001
                x_offset = 0.01
                theta_offset = 0.01
            # 创建 Odometry 消息
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom_2"
            odom_msg.child_frame_id = "velodyne"

            # 设置位姿信息（pose）
            odom_msg.pose.pose.position.x = displacement
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.orientation.w = 1
            # odom_msg.pose.covariance = [
            #                         1, 0, 0, 0, 0, 0,
            #                         0, 1, 0, 0, 0, 0,
            #                         0, 0, 0, 0, 0, 0,
            #                         0, 0, 0, 0, 0, 0,
            #                         0, 0, 0, 0, 0, 0,
            #                         0, 0, 0, 0, 0, 1
            #                         ]
            odom_msg.pose.covariance = [
                1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.001]

            # 发布 /odom_2
            self.odom_pub.publish(odom_msg)

            # 发布 `odom_2 -> base_link` 变换
            # print("x_offset: ", x_offset)
            # print("theta_offset: ", theta_offset)
            self.br.sendTransform(
                (x_offset, 0, 0),  # 位置 (x, y, z)
                tf.transformations.quaternion_from_euler(0, 0, theta_offset),  # 旋转 (roll, pitch, yaw)
                rospy.Time.now(),
                "velodyne",  # 目标坐标系（机器人本体）
                "odom_2"  # 里程计坐标系
            )

            # 增加时间步长
            self.t += 1  # 控制扰动频率
            self.rate.sleep()

    # def publish_fake_odom(self):
    #     while not rospy.is_shutdown():
    #         # 生成一个微小的前后往复运动（正弦波扰动）
    #         displacement = self.max_displacement * math.sin(self.t)
    #         rotation = self.max_rotation * math.sin(self.t)

    #         # 创建 Odometry 消息
    #         odom_msg = Odometry()
    #         odom_msg.header.stamp = rospy.Time.now()
    #         odom_msg.header.frame_id = "odom_2"
    #         odom_msg.child_frame_id = "base_link"

    #         # 设置位姿信息（pose）
    #         odom_msg.pose.pose.position.x = displacement
    #         odom_msg.pose.pose.position.y = 0.0
    #         odom_msg.pose.pose.position.z = 0.0
    #         odom_msg.pose.pose.orientation.w = math.cos(rotation / 2.0)
    #         odom_msg.pose.pose.orientation.z = math.sin(rotation / 2.0)

    #         # 设置线速度扰动
    #         odom_msg.twist.twist.linear.x = 0.01 * math.cos(self.t)
    #         odom_msg.twist.twist.angular.z = 0.002 * math.sin(self.t)

    #         # 发布 /odom_2
    #         self.odom_pub.publish(odom_msg)

    #         # 增加时间步长
    #         self.t += 0.1  # 控制扰动频率
    #         self.rate.sleep()

if __name__ == '__main__':
    try:
        FakeOdomPublisher()
    except rospy.ROSInterruptException:
        pass
