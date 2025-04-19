import numpy as np

# Scipy feels like a really heavy dependency just for rot/quat conversion
# but it's being included anyway from KISS
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from kiss_slam.slam import KissSLAM
from kiss_slam.config import load_config

from kiss_slam_ros.kiss_conversions import pc2_to_numpy, create_marker, create_transform
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

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
        self.tf_broadcaster = TransformBroadcaster(self)

    def cloud_callback(self, in_msg: PointCloud2):
        self.kiss.process_scan(pc2_to_numpy(in_msg), np.empty((0,)))

        pose = self.kiss.poses[-1]
        R = Rotation.from_matrix(pose[:3,:3])
        q = R.as_quat()

        marker = create_marker(pose[0,3], pose[1,3], pose[2,3], q)
        marker.header.stamp = in_msg.header.stamp
        self.marker_publisher.publish(marker)

        t = create_transform(pose[0,3], pose[1,3], pose[2,3], q)
        t.header.stamp = in_msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = in_msg.header.frame_id
        self.tf_broadcaster.sendTransform(t)

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