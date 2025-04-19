import numpy as np

# Scipy feels like a really heavy dependency just for rot/quat conversion
# but it's being included anyway from KISS
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from kiss_slam.slam import KissSLAM
from kiss_slam.config import load_config

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker


def message_to_numpy(msg: PointCloud2):
    # We do this instead of using read_points_numpy() to handle
    # cases where intensity channel might be different type than xyz types
    fields = ['x', 'y', 'z']
    structured = point_cloud2.read_points(msg, field_names=fields)
    unstructured = point_cloud2.structured_to_unstructured(structured)
    return unstructured

def create_marker(x: float, y: float, z: float, q: np.ndarray):
    msg = Marker()
    msg.header.frame_id = "map"
    msg.id = 0
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    msg.scale.x = 1.0
    msg.scale.y = 0.2
    msg.scale.z = 0.2
    msg.color.g = 1.0
    msg.color.a = 1.0
    return msg

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
        self.marker_publisher = self.create_publisher(Marker, 'marker', 10)

    def cloud_callback(self, in_msg: PointCloud2):
        self.kiss.process_scan(message_to_numpy(in_msg), np.empty((0,)))
        pose = self.kiss.poses[-1]
        R = Rotation.from_matrix(pose[:3,:3])
        q = R.as_quat()
        marker = create_marker(pose[0,3], pose[1,3], pose[2,3], q)
        marker.header.stamp = in_msg.header.stamp
        self.marker_publisher.publish(marker)
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