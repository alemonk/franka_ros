import math
import numpy as np
import tf.transformations as tf_trans
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from franka_msgs.msg import FrankaState
import time

def quaternion_from_matrix(matrix):
    """Convert a 3x3 rotation matrix to a quaternion."""
    # Ensure the matrix is a valid 3x3 matrix
    assert matrix.shape == (3, 3), "Input matrix must be 3x3"

    # Create a 4x4 identity matrix
    m = np.eye(4)
    
    # Replace the top-left 3x3 part with the rotation matrix
    m[:3, :3] = matrix[:3, :3]

    # Convert to quaternion using the proper 4x4 matrix
    q = tf_trans.quaternion_from_matrix(m)
    
    # Normalize the quaternion to avoid any potential drift
    q = q / np.linalg.norm(q)

    return q


def slerp(q0, q1, t):
    """Spherical Linear Interpolation between two quaternions."""
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = np.dot(q0, q1)

    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.95:
        result = q0 + t * (q1 - q0)
        result /= np.linalg.norm(result)
        return result

    theta_0 = np.arccos(dot)
    theta = theta_0 * t

    q2 = q1 - q0 * dot
    q2 /= np.linalg.norm(q2)

    result = q0 * np.cos(theta) + q2 * np.sin(theta)
    result /= np.linalg.norm(result)
    return result

def arctan_interpolation(t, total_steps, scaling_factor=5):
    return (math.atan(scaling_factor * (t / total_steps - 0.5)) + math.atan(scaling_factor / 2)) / (2 * math.atan(scaling_factor / 2))

def move_robot_from_A_to_B(end_pose, frequency=5, duration=2):
    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=10)
    rate = rospy.Rate(frequency)

    start_pose, _ = get_current_pose()
 
    steps = int(frequency * duration)
    for i in range(steps):
        alpha = arctan_interpolation(i, steps)
        intermediate_pose = PoseStamped()
        intermediate_pose.header.frame_id = start_pose.header.frame_id
        intermediate_pose.pose.position.x = (1 - alpha) * start_pose.pose.position.x + alpha * end_pose.pose.position.x
        intermediate_pose.pose.position.y = (1 - alpha) * start_pose.pose.position.y + alpha * end_pose.pose.position.y
        intermediate_pose.pose.position.z = (1 - alpha) * start_pose.pose.position.z + alpha * end_pose.pose.position.z

        start_quat = np.array([start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w])
        end_quat = np.array([end_pose.pose.orientation.x, end_pose.pose.orientation.y, end_pose.pose.orientation.z, end_pose.pose.orientation.w])
        interp_quat = slerp(start_quat, end_quat, alpha)

        # Normalize the interpolated quaternion
        interp_quat /= np.linalg.norm(interp_quat)

        intermediate_pose.pose.orientation.x = interp_quat[0]
        intermediate_pose.pose.orientation.y = interp_quat[1]
        intermediate_pose.pose.orientation.z = interp_quat[2]
        intermediate_pose.pose.orientation.w = interp_quat[3]
        
        intermediate_pose.header.stamp = rospy.Time.now()
        pose_pub.publish(intermediate_pose)
        contact_pub.publish(False)
        rate.sleep()

def get_current_pose():
    msg = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState)
    current_pose = PoseStamped()
    current_pose.header = msg.header
    current_pose.pose.position.x = msg.O_T_EE[12]
    current_pose.pose.position.y = msg.O_T_EE[13]
    current_pose.pose.position.z = msg.O_T_EE[14]

    # Extract rotation matrix from O_T_EE    
    R = np.array([
        [msg.O_T_EE[0], msg.O_T_EE[4], msg.O_T_EE[8]],
        [msg.O_T_EE[1], msg.O_T_EE[5], msg.O_T_EE[9]],
        [msg.O_T_EE[2], msg.O_T_EE[6], msg.O_T_EE[10]]
    ])

    # Convert rotation matrix to quaternion
    q = quaternion_from_matrix(R)

    current_pose.pose.orientation.x = q[0]
    current_pose.pose.orientation.y = q[1]
    current_pose.pose.orientation.z = q[2]
    current_pose.pose.orientation.w = q[3]

    # determine_rotation(R)

    return current_pose, R

def get_rotation_matrix(axis, angle):
    axis = np.asarray(axis)
    axis = axis / np.linalg.norm(axis)
    a = math.cos(angle / 2.0)
    b, c, d = -axis * math.sin(angle / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    R = np.array([
        [aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]
    ])
    return R

def determine_rotation(R):

    # Calculate the angle of rotation
    theta = np.arccos((np.trace(R) - 1) / 2)

    # Calculate the axis of rotation
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2 * np.sin(theta))

    print('......................................')
    print(f'Rotation matrix:\n{R}')
    print(f'Rotation: {np.degrees(theta)}° around the axis: {axis / np.linalg.norm(axis)}')
    print('......................................')
