import numpy as np

# Scipy feels like a really heavy dependency just for rot/quat conversion
# but it's being included anyway from KISS
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from kiss_slam.slam import KissSLAM
from kiss_slam.config import load_config

from kiss_slam_ros.conversions import pc2_to_numpy, build_transform, build_odometry, matrix_to_pose
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class KissSLAMNode(Node):

    def __init__(self):
        """Create parameters, subscriptions, publishers, and set up SLAM
        """
        super().__init__('kiss_slam_node')

        # TODO an argument to set the SLAM config, with a sensible default value

        points_topic_desc = ParameterDescriptor(description='What topic to listen on for PointCloud2 messages.')
        self.declare_parameter('points_topic', '/points', points_topic_desc)
        self.points_topic = self.get_parameter('points_topic').value

        map_frame_desc = ParameterDescriptor(description='Frame ID to use as parent for SLAM estimated transform. Dynamic.')
        self.declare_parameter('map_frame', 'map', map_frame_desc)
        self.map_frame = self.get_parameter('map_frame').value

        position_covariance_desc = ParameterDescriptor(description='Covariance value for position terms of odometry. Dynamic.')
        self.declare_parameter('position_covariance', 0.1, position_covariance_desc)
        self.position_covariance = self.get_parameter('position_covariance').value

        orientation_covariance_desc = ParameterDescriptor(description='Covariance value for orientation terms of odometry. Dynamic.')
        self.declare_parameter('orientation_covariance', 0.1, orientation_covariance_desc)
        self.orientation_covariance = self.get_parameter('orientation_covariance').value

        # Main subscription to drive the SLAM process
        self.subscription = self.create_subscription(
            PointCloud2,
            self.points_topic,
            self.cloud_callback,
            10)
        
        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.slam = KissSLAM(load_config(None))
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params: list):
        """Callback to handle dynamic parameters for runtime reconfiguration

        :param params: List of params changed that triggered this callback
        :type params: list
        :return: Success regardless of whether a param was updated
        :rtype: SetParametersResult
        """
        for param in params:
            if param.name == 'map_frame' and param.type_ == Parameter.Type.STRING:
                self.map_frame = param.value
            if param.name == 'position_covariance' and param.type_ == Parameter.Type.DOUBLE:
                self.position_covariance = param.value
            if param.name == 'orientation_covariance' and param.type_ == Parameter.Type.DOUBLE:
                self.orientation_covariance = param.value
        return SetParametersResult(successful=True)
    
    def cloud_callback(self, in_msg: PointCloud2):
        """When receiving a new point cloud, run KISS-SLAM and produce a tf and odometry
        estimate in the same frame as the point source

        :param in_msg: New points to injest
        :type in_msg: PointCloud2
        """
        self.slam.process_scan(pc2_to_numpy(in_msg), np.empty((0,)))

        pose = matrix_to_pose(self.slam.poses[-1])
        header = Header()
        header.stamp = in_msg.header.stamp
        header.frame_id = self.map_frame

        t = build_transform(header, pose, in_msg.header.frame_id)
        self.tf_broadcaster.sendTransform(t)

        odom = build_odometry(header, pose, self.position_covariance, self.orientation_covariance)
        self.odom_publisher.publish(odom)


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