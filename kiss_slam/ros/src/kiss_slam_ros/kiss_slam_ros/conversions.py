import numpy as np

# Scipy is a really heavy dependency just for rot/quat conversion
# but it's being included anyway from KISS
from scipy.spatial.transform import Rotation

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry


def pc2_to_numpy(msg: PointCloud2):
    # We do this instead of using read_points_numpy() to handle
    # cases where intensity channel might be different type than xyz types
    fields = ['x', 'y', 'z']
    structured = point_cloud2.read_points(msg, field_names=fields)
    unstructured = point_cloud2.structured_to_unstructured(structured)
    return unstructured

def matrix_to_pose(mtx: np.ndarray):
    pose = Pose()

    pose.position.x = mtx[0,3]
    pose.position.y = mtx[1,3]
    pose.position.z = mtx[2,3]

    R = Rotation.from_matrix(mtx[:3,:3])
    q = R.as_quat()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def build_marker(header: Header, pose: Pose):
    msg = Marker()
    msg.header = header
    msg.id = 0
    msg.pose = pose
    msg.scale.x = 1.0
    msg.scale.y = 0.2
    msg.scale.z = 0.2
    msg.color.g = 1.0
    msg.color.a = 1.0
    return msg

def build_transform(header: Header, pose: Pose, child_frame_id: str):
    t = TransformStamped()
    t.header = header
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = pose.orientation
    t.child_frame_id = child_frame_id
    return t

def build_odometry(header: Header, pose: Pose, position_cov: float, orientation_cov: float):
    odom = Odometry()
    odom.header = header
    odom.pose.pose = pose
    odom.pose.covariance[0] = position_cov
    odom.pose.covariance[7] = position_cov
    odom.pose.covariance[14] = position_cov
    odom.pose.covariance[21] = orientation_cov
    odom.pose.covariance[28] = orientation_cov
    odom.pose.covariance[35] = orientation_cov
    return odom