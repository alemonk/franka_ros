#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
from franka_msgs.msg import FrankaState
import math
import numpy as np
import copy
from utils import get_current_pose, move_robot_to_pose

def publish_circle(radius, center_x, center_y, center_z, speed_mm_s, frequency):
    rospy.init_node('circle_pose_publisher', anonymous=True)
    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=10)
    rate = rospy.Rate(frequency)

    initial_pose, _ = get_current_pose()

    start_pose = PoseStamped()
    start_pose.header = Header()
    start_pose.header.frame_id = "panda_link0"
    start_pose.pose.position.x = center_x + radius
    start_pose.pose.position.y = center_y
    start_pose.pose.position.z = center_z

    start_pose.pose.orientation.x = 1.0
    start_pose.pose.orientation.y = 0.0
    start_pose.pose.orientation.z = 0.0
    start_pose.pose.orientation.w = 0.0

    # Move to the circle starting point
    rospy.loginfo("Moving to the circle starting point...")
    _ = move_robot_to_pose(copy.deepcopy(initial_pose), start_pose, duration=5)

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
            start_pose.pose.position.y = center_y + radius * math.sin(angle)
            start_pose.pose.position.z = center_z
            
            # Update the header timestamp
            start_pose.header.stamp = rospy.Time.now()
            
            # Publish the pose
            pose_pub.publish(start_pose)
            contact_pub.publish(True)

            # Sleep to maintain the rate
            rate.sleep()

    # Move back to the initial position
    rospy.loginfo("Returning to the initial position...")
    _ = move_robot_to_pose(start_pose, initial_pose, duration=15)

if __name__ == '__main__':
    try:
        # Parameters for the circle
        radius = 0.1  # Radius of the circle in meters
        center_x = 0.5  # X-coordinate of the circle center in meters
        center_y = 0.0  # Y-coordinate of the circle center in meters
        center_z = 0.0  # Z-coordinate of the circle center in meters
        speed_mm_s = 40  # Desired linear speed in mm/s
        frequency = 2  # Publishing frequency in Hz

        publish_circle(radius, center_x, center_y, center_z, speed_mm_s, frequency)
    except rospy.ROSInterruptException:
        pass
