#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPoseModifier:
    def __init__(self):
        rospy.init_node("initialpose_modifier", anonymous=True)

        # 订阅 Rviz 发送的 /initialpose 话题
        self.sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.callback)

        # 重新发布到 /amcl_initialpose
        self.pub = rospy.Publisher("/amcl_initialpose", PoseWithCovarianceStamped, queue_size=10)

        # 设置自定义协方差
        self.cov_xx = 20  # X 方向的方差
        self.cov_yy = 20  # Y 方向的方差
        self.cov_aa = 20  # 角度的方差 (单位: rad)

    def callback(self, msg):
        rospy.loginfo("Received initial pose from Rviz. Modifying covariance.")

        # 修改协方差
        msg.pose.covariance = [
            self.cov_xx, 0, 0, 0, 0, 0,
            0, self.cov_yy, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, self.cov_aa
        ]

        # 重新发布修改后的消息
        self.pub.publish(msg)
        rospy.loginfo("Published modified initial pose.")

if __name__ == "__main__":
    try:
        InitialPoseModifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
