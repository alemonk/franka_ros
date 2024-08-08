import math
import numpy as np
import tf.transformations as tf_trans
import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
import time
from std_msgs.msg import Header, Bool
import getpass

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

def rotation_motion(distance, angle, axis):
    print(f'Performing a {angle} degrees rotation around {axis} axis')

    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=100)
    contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=100)
    rate = rospy.Rate(10)

    # Starting position for the rotation motion
    rotation_pose = PoseStamped()
    rotation_pose.header = Header()
    rotation_pose.header.frame_id = "panda_link0"

    # Define the initial position in the plane
    current_pose, R_in = get_current_pose()
    initial_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z])
    position_vector_ee = np.array([0, 0, -distance])
    position_vector_base = np.dot(R_in, position_vector_ee)
    pivot_point = initial_position - position_vector_base

    # Convert the initial orientation to a quaternion
    quat_in = np.array([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])

    # Define the quaternion representing the rotation around the given axis
    axis_normalized = np.array(axis) / np.linalg.norm(axis)
    
    num_steps = abs(angle)
    for step in range(num_steps + 1):
        angle_step = (math.radians(angle) / num_steps) * step

        # Create a quaternion representing the rotation for this step
        quat_rotation = tf_trans.quaternion_about_axis(angle_step, axis_normalized)
        
        # Multiply the quaternions to get the new orientation
        quat_out = tf_trans.quaternion_multiply(quat_rotation, quat_in)
        
        # Create the rotation matrix from the quaternion
        rotation_matrix = tf_trans.quaternion_matrix(quat_rotation)[:3, :3]
        
        # Rotate the position vector relative to the initial position
        rotated_position = rotation_matrix.dot(position_vector_base)
        
        # Combine the initial position with the rotated offset
        new_position = pivot_point + rotated_position
        
        # Update the rotation_pose with the new position and orientation
        rotation_pose.pose.position.x = new_position[0]
        rotation_pose.pose.position.y = new_position[1]
        rotation_pose.pose.position.z = new_position[2]
        rotation_pose.pose.orientation.x = quat_out[0]
        rotation_pose.pose.orientation.y = quat_out[1]
        rotation_pose.pose.orientation.z = quat_out[2]
        rotation_pose.pose.orientation.w = quat_out[3]
        
        # Update the header timestamp
        rotation_pose.header.stamp = rospy.Time.now()
        
        # Publish the pose
        pose_pub.publish(rotation_pose)
        contact_pub.publish(False)

        # Sleep to maintain the rate
        rate.sleep()

    # Move back to the initial position
    getpass.getpass("Press Enter to continue...")

    return rotation_pose