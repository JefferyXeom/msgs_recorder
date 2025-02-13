import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np

# 存储的点云帧计数
frame_count = 0
save_interval = 1  # 每10帧保存一次
# output_dir = "/home/d1/datasets/Underground/test_frames/pcd"  # 修改为你的保存路径
output_dir = "/home/d1/datasets/gazebo/test_frames/pcd"  # 修改为你的保存路径

def pointcloud_callback(msg):
    global frame_count
    frame_count += 1

    # 只保存部分帧
    if frame_count % save_interval != 0:
        return

    # 解析 ROS 点云消息
    cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    
    if len(cloud_points) == 0:
        return

    # 转换为 NumPy 数组
    cloud_np = np.array(cloud_points, dtype=np.float32)

    # 使用 Open3D 处理点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_np)

    # 保存为 PCD 文件
    # save_path = f"{output_dir}/livox_frame_{frame_count}.pcd"
    save_path = f"{output_dir}/velodyne_frame_{frame_count}.pcd"
    o3d.io.write_point_cloud(save_path, pcd)
    rospy.loginfo(f"Saved {save_path}")

def main():
    rospy.init_node("livox_pcd_saver", anonymous=True)
    # rospy.Subscriber("/livox/points", PointCloud2, pointcloud_callback)
    # rospy.Subscriber("/lio_sam/mapping/cloud_registered", PointCloud2, pointcloud_callback)
    rospy.Subscriber("/lio_sam/deskew/cloud_deskewed", PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
