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
            '/velodyne_points',
            self.lidar_callback,
            10
        )
        self.get_logger().info('Avoid obstacle node started')

    def lidar_callback(self, msg: PointCloud2):
        points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not points:
            return

        arr = np.array(points, dtype=np.float32)  # shape (N,3)

        # Filter ground (z > -0.1 và z < 1.0, giả sử robot cao 0.18m)
        mask_ground = (arr[:,2] > -0.1) & (arr[:,2] < 1.0)
        arr = arr[mask_ground]

        # Regions (góc nhìn 90 độ trước, khoảng cách 0-1.5m)
        # Phía trước: x >0, |y| <0.5, |angle| <45 deg
        mask_front = (arr[:,0] > 0.0) & (arr[:,0] < 1.5) & (np.abs(arr[:,1]) < 0.5)
        count_front = np.count_nonzero(mask_front)

        # Bên trái: y >0.5, x >0, x <1.5
        mask_left = (arr[:,0] > 0.0) & (arr[:,0] < 1.5) & (arr[:,1] > 0.5) & (arr[:,1] < 1.0)
        count_left = np.count_nonzero(mask_left)

        # Bên phải: y <-0.5, x >0, x <1.5
        mask_right = (arr[:,0] > 0.0) & (arr[:,0] < 1.5) & (arr[:,1] < -0.5) & (arr[:,1] > -1.0)
        count_right = np.count_nonzero(mask_right)

        cmd = Twist()
        threshold = 20  # Adjust dựa trên test

        if count_front > threshold:
            if count_left < count_right:  # Quay trái nếu trái ít vật cản hơn
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6
                self.get_logger().info(f'Obstacle front, turning left (front={count_front}, left={count_left}, right={count_right})')
            elif count_right < count_left:
                cmd.linear.x = 0.0
                cmd.angular.z = -0.6
                self.get_logger().info(f'Obstacle front, turning right (front={count_front}, left={count_left}, right={count_right})')
            else:
                cmd.linear.x = -0.2  # Lùi nếu cả hai bên blocked
                cmd.angular.z = 0.0
                self.get_logger().info(f'Blocked everywhere, backing up')
        else:
            cmd.linear.x = 0.35
            cmd.angular.z = 0.0
            self.get_logger().info(f'Path clear, moving forward')

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AvoidObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
