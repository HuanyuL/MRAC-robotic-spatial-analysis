#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from go2_interfaces.srv import SaveVoxelCloud
import open3d as o3d
import numpy as np


class VoxelMapSaver(Node):
    def __init__(self):
        super().__init__("voxel_map_saver")
        self.subscription = self.create_subscription(
            PointCloud2, "/voxel_point_cloud", self.pointcloud_callback, 10
        )
        self.service = self.create_service(
            SaveVoxelCloud, "save_voxel_cloud", self.save_voxel_cloud_callback
        )

        self.latest_pointcloud = None

    def pointcloud_callback(self, msg):
        self.get_logger().info("Received voxel map.")
        self.latest_pointcloud = msg

    def save_voxel_cloud_callback(self, request, response):
        if self.latest_pointcloud is None:
            response.success = False
            self.get_logger().warn("No point cloud data received yet.")
            return response

        try:
            pc = self.pointcloud2_to_array(self.latest_pointcloud)
            self.save_to_pcd(pc, request.filename)

            response.success = True
            self.get_logger().info(f"Saved voxel map to: {request.filename}")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Error saving voxel map: {str(e)}")

        return response

    def pointcloud2_to_array(self, cloud_msg):
        point_step = cloud_msg.point_step
        data = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        data = np.ascontiguousarray(data)
        points = data.reshape((-1, point_step))

        x = np.frombuffer(points[:, 0:4].tobytes(), dtype=np.float32)
        y = np.frombuffer(points[:, 4:8].tobytes(), dtype=np.float32)
        z = np.frombuffer(points[:, 8:12].tobytes(), dtype=np.float32)
        pc = np.column_stack((x, y, z)).astype(np.float32)

        return np.ascontiguousarray(pc)

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
