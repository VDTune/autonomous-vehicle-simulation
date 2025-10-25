import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import pcl
from pcl import PointCloud
from pcl.point_types import PointXYZ
from sensor_msgs.msg import PointField  # Import for PointField if needed
import struct  # For unpacking data

class AvoidObstacleNode(Node):
    def __init__(self):
        super().__init__('avoid_obstacle_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne2/velodyne_points2',
            self.lidar_callback,
            10
        )
        self.get_logger().info('Avoid obstacle node started')

    def lidar_callback(self, msg):
        # Chuyển PointCloud2 sang PCL PointCloud
        pcl_cloud = self.pointcloud2_to_pcl(msg)

        # Lọc point cloud để phát hiện vật cản (x: 0 đến 1m, y: -0.5 đến 0.5m)
        passthrough = pcl_cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("x")
        passthrough.set_filter_limits(0.0, 1.0)
        passthrough.set_filter_field_name("y")
        passthrough.set_filter_limits(-0.5, 0.5)
        filtered_cloud = passthrough.filter()

        # Nếu có points trong vùng (vật cản gần), quay trái
        cmd = Twist()
        if filtered_cloud.size > 10:  # Ngưỡng phát hiện
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Quay trái (rad/s)
            self.get_logger().info('Obstacle detected, turning left')
        else:
            cmd.linear.x = 0.5  # Đi thẳng
            cmd.angular.z = 0.0
            self.get_logger().info('No obstacle, moving forward')

        self.publisher_.publish(cmd)

    def pointcloud2_to_pcl(self, cloud_msg):
        # Hàm tùy chỉnh để chuyển PointCloud2 sang PCL PointCloud
        points_list = []
        for data in cloud_msg.data:
            # Giả định format XYZ (float32 x3)
            points_list.append(struct.unpack('fff', data))
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_list(points_list)
        return pcl_cloud

def main(args=None):
    rclpy.init(args=args)
    node = AvoidObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
