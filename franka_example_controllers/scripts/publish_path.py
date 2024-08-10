#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
from franka_msgs.msg import FrankaState
import math
import numpy as np
import copy
from utils import get_current_pose, move_robot_to_pose

def publish_circle(radius, center_x, center_y, center_z, speed_mm_s, frequency=1000):
    rospy.init_node('circle_pose_publisher', anonymous=True)
    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=100)
    contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=100)
    rate = rospy.Rate(frequency)

    start_pose, _ = get_current_pose()
    circle_pose = copy.deepcopy(start_pose)
    circle_pose.pose.position.x = center_x + radius
    circle_pose.pose.position.y = center_y
    circle_pose.pose.position.z = center_z
    _ = move_robot_to_pose(start_pose, circle_pose, duration=10)

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
            circle_pose.pose.position.x = center_x + radius * math.cos(angle)
            circle_pose.pose.position.y = center_y + radius * math.sin(angle)
            circle_pose.pose.position.z = center_z
            
            # Update the header timestamp
            circle_pose.header.stamp = rospy.Time.now()
            
            # Publish the pose
            pose_pub.publish(circle_pose)
            contact_pub.publish(True)

            # Sleep to maintain the rate
            rate.sleep()

    # Move back to the initial position
    rospy.loginfo("Returning to the initial position...")
    _ = move_robot_to_pose(circle_pose, start_pose, duration=5)

if __name__ == '__main__':
    try:
        # Parameters for the circle
        radius = 0.1  # Radius of the circle in meters
        center_x = 0.5  # X-coordinate of the circle center in meters
        center_y = 0.0  # Y-coordinate of the circle center in meters
        center_z = 0.0  # Z-coordinate of the circle center in meters
        speed_mm_s = 10  # Desired linear speed in mm/s

        publish_circle(radius, center_x, center_y, center_z, speed_mm_s)
    except rospy.ROSInterruptException:
        pass
