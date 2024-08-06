import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
from franka_msgs.msg import FrankaState
import math
import numpy as np
import tf.transformations as tf_trans

current_pose = None

def franka_state_callback(msg):
    global current_pose
    current_pose = PoseStamped()
    current_pose.header = msg.header
    current_pose.pose.position.x = msg.O_T_EE[12]
    current_pose.pose.position.y = msg.O_T_EE[13]
    current_pose.pose.position.z = msg.O_T_EE[14]

    # Extract rotation matrix from O_T_EE
    R = np.array([
        [msg.O_T_EE[0], msg.O_T_EE[1], msg.O_T_EE[2]],
        [msg.O_T_EE[4], msg.O_T_EE[5], msg.O_T_EE[6]],
        [msg.O_T_EE[8], msg.O_T_EE[9], msg.O_T_EE[10]]
    ])

    # Convert rotation matrix to quaternion
    q = quaternion_from_matrix(R)

    current_pose.pose.orientation.x = q[0]
    current_pose.pose.orientation.y = q[1]
    current_pose.pose.orientation.z = q[2]
    current_pose.pose.orientation.w = q[3]

def quaternion_from_matrix(matrix):
    """Convert a rotation matrix to a quaternion."""
    return tf_trans.quaternion_from_matrix(np.pad(matrix, ((0, 1), (0, 1)), mode='constant', constant_values=0) + np.eye(4))

def slerp(q0, q1, t):
    """Spherical Linear Interpolation between two quaternions."""
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

    return q0 * np.cos(theta) + q2 * np.sin(theta)

def arctan_interpolation(t, total_steps, scaling_factor=5):
    return (math.atan(scaling_factor * (t / total_steps - 0.5)) + math.atan(scaling_factor / 2)) / (2 * math.atan(scaling_factor / 2))

def move_robot_smoothly(start_pose, end_pose, frequency, duration):
    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=10)
    rate = rospy.Rate(frequency)
    
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

        intermediate_pose.pose.orientation.x = interp_quat[0]
        intermediate_pose.pose.orientation.y = interp_quat[1]
        intermediate_pose.pose.orientation.z = interp_quat[2]
        intermediate_pose.pose.orientation.w = interp_quat[3]
        
        intermediate_pose.header.stamp = rospy.Time.now()
        pose_pub.publish(intermediate_pose)
        contact_pub.publish(False)
        rate.sleep()

def publish_circle(radius, center_x, center_y, center_z, speed_mm_s, frequency):
    rospy.init_node('circle_pose_publisher', anonymous=True)
    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=10)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, franka_state_callback)
    rate = rospy.Rate(frequency)

    # Wait until we have a valid current pose
    while current_pose is None:
        rospy.loginfo("Waiting for the current pose...")
        rospy.sleep(0.1)

    initial_pose = current_pose

    start_pose = PoseStamped()
    start_pose.header = Header()
    start_pose.header.frame_id = "panda_link0"
    start_pose.pose.position.x = center_x + radius
    start_pose.pose.position.y = center_y
    start_pose.pose.position.z = center_z

    # Create a quaternion representing a 90-degree
    quat = tf_trans.quaternion_from_euler(-math.pi / 2, 0, 0)
    start_pose.pose.orientation.x = quat[0]
    start_pose.pose.orientation.y = quat[1]
    start_pose.pose.orientation.z = quat[2]
    start_pose.pose.orientation.w = quat[3]

    # Move to the circle starting point
    rospy.loginfo("Moving to the circle starting point...")
    move_robot_smoothly(initial_pose, start_pose, frequency, duration=5.0)
    t = start_pose
    start_pose.pose.position.y = start_pose.pose.position.y + 0.25
    move_robot_smoothly(t, start_pose, frequency, duration=5.0)

    # Convert speed from mm/s to m/s
    speed_m_s = speed_mm_s / 1000.0

    # Calculate the circumference of the circle
    circumference = 2 * math.pi * radius

    # Calculate the time to complete one full circle based on the desired speed
    time_to_complete_circle = circumference / speed_m_s

    # Calculate the number of points based on the time and frequency
    points = int(time_to_complete_circle * frequency)

    for _ in range(2):  # Complete two circles
        for i in range(points):
            # Calculate the angle for the current point
            angle = 2 * math.pi * i / points
            
            # Set the desired position
            start_pose.pose.position.x = center_x + radius * math.cos(angle)
            start_pose.pose.position.y = center_y + 0.25
            start_pose.pose.position.z = center_z + radius * math.sin(angle)
            
            # Update the header timestamp
            start_pose.header.stamp = rospy.Time.now()
            
            # Publish the pose
            pose_pub.publish(start_pose)
            contact_pub.publish(True)

            # Sleep to maintain the rate
            rate.sleep()

    # Move back to the initial position
    rospy.loginfo("Returning to the initial position...")
    move_robot_smoothly(start_pose, initial_pose, frequency, duration=5.0)

if __name__ == '__main__':
    try:
        # Parameters for the circle
        radius = 0.1  # Radius of the circle in meters
        center_x = 0.5  # X-coordinate of the circle center in meters
        center_y = 0.0  # Y-coordinate of the circle center in meters
        center_z = 0.25  # Z-coordinate of the circle center in meters
        speed_mm_s = 40  # Desired linear speed in mm/s
        frequency = 2  # Publishing frequency in Hz

        publish_circle(radius, center_x, center_y, center_z, speed_mm_s, frequency)
    except rospy.ROSInterruptException:
        pass
