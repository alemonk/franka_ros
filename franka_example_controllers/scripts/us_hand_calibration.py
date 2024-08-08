import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
import math
import numpy as np
import tf.transformations as tf_trans
from utils import get_current_pose
import time
import getpass

def perform_rotation_motion(initial_pose, distance, angle, frequency, axis, num_steps=10):
    print(f'Performing a {angle} degrees rotation around {axis} axis')

    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=10)
    rate = rospy.Rate(frequency)

    # Starting position for the rotation motion
    rotation_pose = PoseStamped()
    rotation_pose.header = Header()
    rotation_pose.header.frame_id = "panda_link0"
    rotation_pose.pose.position = initial_pose.pose.position
    rotation_pose.pose.orientation = initial_pose.pose.orientation

    # Define the initial position in the plane
    initial_position = np.array([initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z])
    _, R = get_current_pose()
    position_vector_ee = np.array([0, 0, -distance])
    position_vector_base = np.dot(R, position_vector_ee)
    pivot_point = initial_position - position_vector_base

    # Convert the initial orientation to a quaternion
    quat_in = np.array([initial_pose.pose.orientation.x, initial_pose.pose.orientation.y, initial_pose.pose.orientation.z, initial_pose.pose.orientation.w])

    # Define the quaternion representing the rotation around the given axis
    axis_normalized = np.array(axis) / np.linalg.norm(axis)
    
    for step in range(num_steps + 1):
        angle_step = (angle / num_steps) * step

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

    return initial_pose

if __name__ == '__main__':
    try:
        # Parameters for the screw motion
        distance = 0.1  # Distance perpendicular to the end effector's original position in meters
        frequency = 10  # Publishing frequency in Hz

        rospy.init_node('screw_pose_publisher', anonymous=True)

        getpass.getpass("Press Enter to start...")

        initial_pose, _ = get_current_pose()
        initial_pose = perform_rotation_motion(initial_pose, distance, math.radians(90), frequency, [1, 0, 0])
        initial_pose = perform_rotation_motion(initial_pose, distance, math.radians(-90), frequency, [1, 0, 0])

        initial_pose = perform_rotation_motion(initial_pose, distance, math.radians(90), frequency, [0, 1, 0])
        initial_pose = perform_rotation_motion(initial_pose, distance, math.radians(-90), frequency, [0, 1, 0])

        initial_pose = perform_rotation_motion(initial_pose, distance, math.radians(90), frequency, [0, 0, 1])
        initial_pose = perform_rotation_motion(initial_pose, distance, math.radians(-90), frequency, [0, 0, 1])

    except rospy.ROSInterruptException:
        pass