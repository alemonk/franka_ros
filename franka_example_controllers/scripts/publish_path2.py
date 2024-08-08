import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
from franka_msgs.msg import FrankaState
import math
import numpy as np
import tf.transformations as tf_trans
from utils import *

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
    move_robot_from_A_to_B(start_pose, frequency, duration=5.0)

    # Convert speed from mm/s to m/s
    speed_m_s = speed_mm_s / 1000.0

    # Calculate the circumference of the circle
    circumference = 2 * math.pi * radius

    # Calculate the time to complete one full circle based on the desired speed
    time_to_complete_circle = circumference / speed_m_s

    # Calculate the number of points based on the time and frequency
    points = int(time_to_complete_circle * frequency)

    for _ in range(1):
        for i in range(points):
            # Calculate the angle for the current point
            angle = 2 * math.pi * i / points
            
            # Set the desired position
            start_pose.pose.position.x = center_x + radius * math.cos(angle)
            start_pose.pose.position.y = center_y
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
    move_robot_from_A_to_B(initial_pose, frequency, duration=5.0)

if __name__ == '__main__':
    try:
        # Parameters for the circle
        radius = 0.1  # Radius of the circle in meters
        center_x = 0.5  # X-coordinate of the circle center in meters
        center_y = 0.25  # Y-coordinate of the circle center in meters
        center_z = 0.25  # Z-coordinate of the circle center in meters
        speed_mm_s = 40  # Desired linear speed in mm/s
        frequency = 2  # Publishing frequency in Hz

        publish_circle(radius, center_x, center_y, center_z, speed_mm_s, frequency)
    except rospy.ROSInterruptException:
        pass
