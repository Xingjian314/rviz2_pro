import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection


class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')
        self.projector = LaserProjection()
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/pointcloud', 10)

    def scan_callback(self, msg):
        cloud = self.projector.projectLaser(msg)
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
