import rclpy
from rclpy.node import Node

from kiss_slam.slam import KissSLAM
from kiss_slam.config import load_config

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class KissSLAMNode(Node):

    def __init__(self):
        super().__init__('kiss_slam_node')
        self.kiss = KissSLAM(load_config(None))
        self.subscription = self.create_subscription(
            PointCloud2,
            'points',
            self.cloud_callback,
            10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

    def cloud_callback(self, in_msg):
        out_msg = Odometry()
        self.odom_publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    kiss = KissSLAMNode()

    rclpy.spin(kiss)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kiss.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()