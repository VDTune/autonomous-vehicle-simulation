#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class AvoidObstacleNode(Node):
    def __init__(self):
        super().__init__('avoid_obstacle_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',  # đảm bảo khớp với topic thực tế
            self.lidar_callback,
            10
        )
        self.get_logger().info('Avoid obstacle node started')

    def lidar_callback(self, msg: PointCloud2):
        # đọc points (x,y,z)
        points = list(point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        if not points:
            return

        arr = np.array(points, dtype=np.float32)  # shape (N,3)

        # filter: phía trước robot x in (0, 1.0m) và y in (-0.5, 0.5m)
        mask = (arr[:,0] > 0.0) & (arr[:,0] < 1.0) & (arr[:,1] > -0.5) & (arr[:,1] < 0.5)
        close_count = np.count_nonzero(mask)

        cmd = Twist()
        if close_count > 20:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.6
            self.get_logger().info(f'Obstacle detected, turning left (count={close_count})')
        else:
            cmd.linear.x = 0.35
            cmd.angular.z = 0.0
            self.get_logger().info(f'Path clear, moving forward (count={close_count})')

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AvoidObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
