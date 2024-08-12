import math
import numpy as np
import tf.transformations as tf_trans
import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from std_msgs.msg import Header, Bool
import time
import copy

def move_robot_to_pose(start_pose, end_pose, duration=1, frequency=10):
    """
    Moves the robot's end-effector smoothly from a start pose to an end pose over a specified duration.

    Args:
        start_pose (PoseStamped): The initial pose of the robot's end-effector in the base frame.
        end_pose (PoseStamped): The target pose of the robot's end-effector in the base frame.
        duration (float, optional): The duration of the motion in seconds. Default is 1 second.
        frequency (int, optional): The control loop frequency in Hz. Default is 10 Hz.

    Returns:
        PoseStamped: The final pose of the robot's end-effector after the motion.

    Notes:
        - The `start_pose` and `end_pose` must be provided in the base frame of the robot.
        - The function performs linear interpolation of the position and spherical linear interpolation (slerp) of the orientation (quaternion) between the start and end poses.
        - The function will wait until the robot has reached the final pose before returning.
    """

    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=10)
    rate = rospy.Rate(frequency)

    desired_pose = copy.deepcopy(start_pose)
    steps = int(duration * frequency)

    for i in range(steps):
        alpha = float(i) / float(steps)
        # Position interpolation
        desired_pose.pose.position.x = (1 - alpha) * start_pose.pose.position.x + alpha * end_pose.pose.position.x
        desired_pose.pose.position.y = (1 - alpha) * start_pose.pose.position.y + alpha * end_pose.pose.position.y
        desired_pose.pose.position.z = (1 - alpha) * start_pose.pose.position.z + alpha * end_pose.pose.position.z

        # Quaternion (Orientation) interpolation
        start_quat = np.array([start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w])
        end_quat = np.array([end_pose.pose.orientation.x, end_pose.pose.orientation.y, end_pose.pose.orientation.z, end_pose.pose.orientation.w])
        desired_quat = slerp(start_quat, end_quat, alpha)
        desired_quat /= np.linalg.norm(desired_quat)

        desired_pose.pose.orientation.x = desired_quat[0]
        desired_pose.pose.orientation.y = desired_quat[1]
        desired_pose.pose.orientation.z = desired_quat[2]
        desired_pose.pose.orientation.w = desired_quat[3]

        # Update timestamp and publish
        desired_pose.header.stamp = rospy.Time.now()
        pose_pub.publish(desired_pose)
        contact_pub.publish(False)

        rate.sleep()

    # Wait until the final pose is reached
    wait_until_reached(end_pose)
    return desired_pose

def rotation_motion(start_pose, position_vector_ee, angle_degrees, axis, frequency=10):
    """
    Rotates the robot's end-effector around a specified axis in the base frame while maintaining a fixed distance from the rotation point.

    Args:
        start_pose (PoseStamped): The initial pose of the robot's end-effector in the base frame.
        position_vector_ee (numpy.ndarray): A 3D vector representing the offset from the end-effector to the rotation point, specified in the end-effector's frame.
        angle_degrees (float): The total angle of rotation in degrees.
        axis (list or numpy.ndarray): The axis of rotation, specified as a 3D vector in the base frame.
        frequency (int, optional): The control loop frequency in Hz. Default is 10 Hz.

    Returns:
        PoseStamped: The final pose of the robot's end-effector after completing the rotation.

    Notes:
        - The `start_pose` must be provided in the base frame of the robot.
        - The `position_vector_ee` specifies the offset from the end-effector to the rotation point. It should be defined in the end-effector's frame.
        - The function performs a rotation of the end-effector around the specified axis while keeping the distance to the rotation point constant.
        - The orientation of the end-effector is updated during the rotation to ensure alignment with the new position.
        - The rotation is performed incrementally over the specified number of steps, defined by the `frequency` and the `angle_degrees`.
    """

    print(f'Performing a {angle_degrees} degrees rotation around {axis} axis')

    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=100)
    contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=100)
    rate = rospy.Rate(frequency)

    # Starting position for the rotation motion
    rotation_pose = PoseStamped()
    rotation_pose.header = Header()
    rotation_pose.header.frame_id = "panda_link0"

    # Define the initial position in the plane
    _, R_in = get_current_pose()
    start_position = np.array([start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z])
    position_vector_base = np.dot(R_in, position_vector_ee)
    rotation_point = start_position - position_vector_base

    quat_in = np.array([start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w])
    axis_normalized = np.array(axis) / np.linalg.norm(axis)
    
    num_steps = abs(angle_degrees)
    for step in range(num_steps + 1):
        angle_step = (math.radians(angle_degrees) / num_steps) * step

        quat_rotation = tf_trans.quaternion_about_axis(angle_step, axis_normalized)
        quat_out = tf_trans.quaternion_multiply(quat_rotation, quat_in)
        
        rotation_matrix = tf_trans.quaternion_matrix(quat_rotation)[:3, :3]
        rotated_position = rotation_matrix.dot(position_vector_base)
        new_position = rotation_point + rotated_position
        
        rotation_pose.pose.position.x = new_position[0]
        rotation_pose.pose.position.y = new_position[1]
        rotation_pose.pose.position.z = new_position[2]
        rotation_pose.pose.orientation.x = quat_out[0]
        rotation_pose.pose.orientation.y = quat_out[1]
        rotation_pose.pose.orientation.z = quat_out[2]
        rotation_pose.pose.orientation.w = quat_out[3]
        
        rotation_pose.header.stamp = rospy.Time.now()
        pose_pub.publish(rotation_pose)
        contact_pub.publish(False)

        rate.sleep()

    wait_until_reached(rotation_pose)
    return rotation_pose

def linear_motion_z_axis(start_pose, distance):
    """
    Performs a linear motion along the end-effector's Z-axis for a specified distance.

    Args:
        start_pose (PoseStamped): The initial pose of the robot's end-effector in the base frame.
        distance (float): The distance to move along the end-effector's Z-axis, specified in meters. Positive values move the end-effector forward along its Z-axis, while negative values move it backward.

    Returns:
        PoseStamped: The final pose of the robot's end-effector after completing the linear motion.

    Notes:
        - The `start_pose` must be provided in the base frame of the robot.
        - The function calculates the end position by transforming the motion vector from the end-effector's frame to the base frame using the current rotation matrix.
        - The motion is executed by calling `move_robot_to_pose`, which handles the interpolation of the position and orientation during the movement.
    """

    print(f"Performing a {distance}m linear motion along the end effector's z axis")

    _, R = get_current_pose()
    start_position = np.array([start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z])
    position_vector_ee = np.array([0, 0, distance])
    position_vector_base = np.dot(R, position_vector_ee)

    end_position = start_position + position_vector_base

    end_pose = copy.deepcopy(start_pose)
    end_pose.pose.position.x = end_position[0]
    end_pose.pose.position.y = end_position[1]
    end_pose.pose.position.z = end_position[2]

    _ = move_robot_to_pose(start_pose, end_pose)
    return end_pose

def quaternion_from_matrix(matrix):
    """Convert a 3x3 rotation matrix to a quaternion."""

    m = np.eye(4)
    m[:3, :3] = matrix[:3, :3]
    q = tf_trans.quaternion_from_matrix(m)
    q = q / np.linalg.norm(q)
    return q

def slerp(q0, q1, t, threshold=0.1):
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
    
    # Directly set the final quaternion if the interpolation is near completion
    if t > 1.0 - threshold:
        return q1
    
    return result

def arctan_interpolation(t, total_steps, scaling_factor=5):
    """Computes a smooth interpolation value using an arctangent function, useful for generating non-linear motion profiles."""

    return (math.atan(scaling_factor * (t / total_steps - 0.5)) + math.atan(scaling_factor / 2)) / (2 * math.atan(scaling_factor / 2))

def get_current_pose():
    """
    Retrieves the current pose of the robot's end-effector, including position and orientation, and returns the corresponding rotation matrix.

    Returns:
        tuple:
            - PoseStamped: The current pose of the robot's end-effector in the base frame.
            - numpy.ndarray: The 3x3 rotation matrix representing the orientation of the end-effector.

    Notes:
        - The function waits for the latest message from the `/franka_state_controller/franka_states` topic to retrieve the current state.
        - The position is extracted from the transformation matrix `O_T_EE`.
        - The rotation matrix is converted to a quaternion, which is then used to populate the orientation of the `PoseStamped` message.
    """

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
    return copy.deepcopy(current_pose), R

def get_rotation_matrix(axis, angle):
    """Computes a rotation matrix for a given axis and angle using the Rodrigues' rotation formula."""

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
    """Analyzes a rotation matrix to determine the angle and axis of rotation, and prints the results."""

    # Calculate the angle of rotation
    theta = np.arccos((np.trace(R) - 1) / 2)

    # Calculate the axis of rotation
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2 * np.sin(theta))

    print(f'Rotation matrix:\n{R}')
    print(f'Rotation: {np.degrees(theta)}° around the axis: {axis / np.linalg.norm(axis)}')

def wait_until_reached(end_pose, position_tolerance=0.01, orientation_tolerance=0.05, frequency=10):
    """
    Waits until the end effector reaches the specified pose within the given tolerances.

    :param end_pose: The target pose (of type PoseStamped) the end effector should reach.
    :param position_tolerance: The allowable difference between the current and target positions (in meters).
    :param orientation_tolerance: The allowable difference between the current and target orientations (in radians).
    :param frequency: The rate at which to check the current pose (in Hz).
    """
    rate = rospy.Rate(frequency)
    t1 = time.time()
    
    while True:
        current_pose, _ = get_current_pose()
        current_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z])
        target_position = np.array([end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z])
        
        current_orientation = np.array([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])
        target_orientation = np.array([end_pose.pose.orientation.x, end_pose.pose.orientation.y, end_pose.pose.orientation.z, end_pose.pose.orientation.w])
        
        position_difference = np.linalg.norm(current_position - target_position)
        orientation_difference = 2 * math.acos(np.clip(np.dot(current_orientation, target_orientation), -1.0, 1.0))

        t2 = time.time()
        delta_t = t2 - t1
        
        if (position_difference < position_tolerance and orientation_difference < orientation_tolerance) or delta_t > 3:
            time.sleep(1)
            print("End effector has reached the target pose.")
            break
        
        rate.sleep()
