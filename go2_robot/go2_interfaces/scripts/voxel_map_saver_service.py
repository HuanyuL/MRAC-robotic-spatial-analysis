#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import struct


class VoxelMapSaver(Node):
    def __init__(self):
        super().__init__("voxel_map_saver")
        self.subscription = self.create_subscription(
            PointCloud2, "/voxel_map", self.pointcloud_callback, 10
        )

    def pointcloud_callback(self, msg):
        self.get_logger().info("Received voxel map, saving...")
        pc = self.pointcloud2_to_array(msg)
        self.save_to_pcd(pc, "voxel_map.pcd")

    def pointcloud2_to_array(self, cloud_msg):
        point_step = cloud_msg.point_step
        data = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        points = np.reshape(data, (-1, point_step))
        x = np.frombuffer(points[:, 0:4], dtype=np.float32)
        y = np.frombuffer(points[:, 4:8], dtype=np.float32)
        z = np.frombuffer(points[:, 8:12], dtype=np.float32)
        return np.vstack((x, y, z)).T

    def save_to_pcd(self, pc, filename):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"Saved voxel map to: {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = VoxelMapSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
