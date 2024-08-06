#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
import math

def publish_circle(radius, center_x, center_y, center_z, speed_mm_s, frequency):
    rospy.init_node('circle_pose_publisher', anonymous=True)
    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(frequency)  # Set the rate to publish the poses

    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = "panda_link0"  # Change if necessary

    # Set the desired orientation (60 degrees from the xy plane)
    angle_rad = math.radians(90 + 90)
    pose.pose.orientation.x = math.sin(angle_rad / 2)
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = math.cos(angle_rad / 2)

    # Convert speed from mm/s to m/s
    speed_m_s = speed_mm_s / 1000.0

    # Calculate the circumference of the circle
    circumference = 2 * math.pi * radius

    # Calculate the time to complete one full circle based on the desired speed
    time_to_complete_circle = circumference / speed_m_s

    # Calculate the number of points based on the time and frequency
    points = int(time_to_complete_circle * frequency)

    while not rospy.is_shutdown():
        for i in range(points):
            # Calculate the angle for the current point
            angle = 2 * math.pi * i / points
            
            # Set the desired position
            pose.pose.position.x = center_x + radius * math.cos(angle)
            pose.pose.position.y = center_y + radius * math.sin(angle)
            pose.pose.position.z = center_z
            
            # Update the header timestamp
            pose.header.stamp = rospy.Time.now()
            
            # Publish the pose
            pose_pub.publish(pose)

            # Sleep to maintain the rate
            rate.sleep()
    
if __name__ == '__main__':
    try:
        # Parameters for the circle
        radius = 0.1  # Radius of the circle in meters
        center_x = 0.5  # X-coordinate of the circle center in meters
        center_y = 0.0  # Y-coordinate of the circle center in meters
        center_z = 0.0  # Z-coordinate of the circle center in meters
        speed_mm_s = 40  # Desired linear speed in mm/s
        frequency = 10  # Publishing frequency in Hz

        publish_circle(radius, center_x, center_y, center_z, speed_mm_s, frequency)
    except rospy.ROSInterruptException:
        pass
