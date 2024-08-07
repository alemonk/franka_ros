import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
import math
import numpy as np
import tf.transformations as tf_trans
from help import *
import time
from copy import deepcopy

def perform_rotation_motion(pose_pub, contact_pub, initial_pose, distance, total_angle_degrees, frequency, axis, step_angle=1):
    print(f'Performing rotation around {axis} for {total_angle_degrees} degrees')
    rate = rospy.Rate(frequency)

    # Starting position for the screw motion
    rotation_pose = PoseStamped()
    rotation_pose.header = Header()
    rotation_pose.header.frame_id = "panda_link0"
    rotation_pose.pose.position = deepcopy(initial_pose.pose.position)  # Make a deep copy
    rotation_pose.pose.orientation = deepcopy(initial_pose.pose.orientation)  # Make a deep copy

    # Convert initial orientation to quaternion
    initial_quat = [
        initial_pose.pose.orientation.x,
        initial_pose.pose.orientation.y,
        initial_pose.pose.orientation.z,
        initial_pose.pose.orientation.w
    ]

    # Calculate the total steps
    total_steps = int(abs(total_angle_degrees) / step_angle)
    angle_step_radians = math.radians(step_angle) * np.sign(total_angle_degrees)

    # Define the initial position in the y-z plane
    initial_position = np.array([initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z])
    position_vector = np.array([0, 0, distance])

    for step in range(total_steps + 1):  # Including the final position
        angle = angle_step_radians * step
        
        # Get the rotation matrix for the current step
        R = get_rotation_matrix(axis, angle)
        
        # Calculate the desired position by applying the rotation matrix
        rotated_position = np.dot(R, position_vector)
        
        # Convert rotated_position to the base frame
        rotated_position_base_frame = initial_position + rotated_position - position_vector

        rotation_pose.pose.position.x = rotated_position_base_frame[0]
        rotation_pose.pose.position.y = rotated_position_base_frame[1]
        rotation_pose.pose.position.z = rotated_position_base_frame[2]
        
        # Calculate the desired orientation (rotate around the given axis)
        if axis == 'x':
            quat = tf_trans.quaternion_multiply(tf_trans.quaternion_from_euler(angle, 0, 0), initial_quat)
        elif axis == 'y':
            quat = tf_trans.quaternion_multiply(tf_trans.quaternion_from_euler(0, angle, 0), initial_quat)
        elif axis == 'z':
            quat = tf_trans.quaternion_multiply(tf_trans.quaternion_from_euler(0, 0, angle), initial_quat)
        
        rotation_pose.pose.orientation.x = quat[0]
        rotation_pose.pose.orientation.y = quat[1]
        rotation_pose.pose.orientation.z = quat[2]
        rotation_pose.pose.orientation.w = quat[3]
        
        # Update the header timestamp
        rotation_pose.header.stamp = rospy.Time.now()
        
        # Publish the pose
        pose_pub.publish(rotation_pose)
        contact_pub.publish(False)

        # Sleep to maintain the rate
        rate.sleep()

    # Move back to the initial position
    time.sleep(5)
    rospy.loginfo("Returning to the initial position...")
    move_robot_from_A_to_B(end_pose=initial_pose, frequency=frequency, duration=3.0)

if __name__ == '__main__':
    try:
        # Parameters for the screw motion
        distance = 0.1  # Distance perpendicular to the end effector's original position in meters
        frequency = 50  # Publishing frequency in Hz

        rospy.init_node('screw_pose_publisher', anonymous=True)
        pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
        contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=10)

        initial_pose, _ = get_current_pose()
        angle_degrees = 30
        axis = 'x'
        perform_rotation_motion(pose_pub, contact_pub, initial_pose, distance, angle_degrees, frequency, axis)

        time.sleep(5)

        initial_pose, _ = get_current_pose()
        angle_degrees = -30
        axis = 'x'
        perform_rotation_motion(pose_pub, contact_pub, initial_pose, distance, angle_degrees, frequency, axis)
        
        time.sleep(5)

        initial_pose, _ = get_current_pose()
        angle_degrees = 30
        axis = 'y'
        perform_rotation_motion(pose_pub, contact_pub, initial_pose, distance, angle_degrees, frequency, axis)

        time.sleep(5)

        initial_pose, _ = get_current_pose()
        angle_degrees = -30
        axis = 'y'
        perform_rotation_motion(pose_pub, contact_pub, initial_pose, distance, angle_degrees, frequency, axis)
        
        time.sleep(5)

        initial_pose, _ = get_current_pose()
        angle_degrees = 30
        axis = 'z'
        perform_rotation_motion(pose_pub, contact_pub, initial_pose, distance, angle_degrees, frequency, axis)

        time.sleep(5)

        initial_pose, _ = get_current_pose()
        angle_degrees = -30
        axis = 'z'
        perform_rotation_motion(pose_pub, contact_pub, initial_pose, distance, angle_degrees, frequency, axis)

    except rospy.ROSInterruptException:
        pass
