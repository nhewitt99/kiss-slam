import numpy as np

from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker


def pc2_to_numpy(msg: PointCloud2):
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

def create_transform(x: float, y: float, z: float, q: np.ndarray):
    t = TransformStamped()
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t
