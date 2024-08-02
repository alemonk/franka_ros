#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
import math

def publish_line(start_x, start_y, start_z, range_y, speed_mm_s, frequency):
    rospy.init_node('line_pose_publisher', anonymous=True)
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

    # Calculate the distance to travel along the y-axis
    total_distance = 2 * range_y

    # Calculate the time to complete the movement based on the desired speed
    time_to_complete_line = total_distance / speed_m_s

    # Calculate the number of points based on the time and frequency
    points = int(time_to_complete_line * frequency)

    while not rospy.is_shutdown():
        for i in range(points):
            # Calculate the position along the y-axis for the current point
            y_position = start_y + (range_y * math.sin(2 * math.pi * i / points))
            
            # Set the desired position
            pose.pose.position.x = start_x
            pose.pose.position.y = y_position
            pose.pose.position.z = start_z
            
            # Update the header timestamp
            pose.header.stamp = rospy.Time.now()
            
            # Publish the pose
            pose_pub.publish(pose)

            # Sleep to maintain the rate
            rate.sleep()
    
if __name__ == '__main__':
    try:
        # Parameters for the line
        start_x = 0.5  # Starting x-coordinate in meters
        start_y = 0.0  # Starting y-coordinate in meters
        start_z = 0.0  # Starting z-coordinate in meters (0 for the xy plane)
        range_y = 0.2  # Range of motion along the y-axis in meters (20 cm)
        speed_mm_s = 10  # Desired linear speed in mm/s
        frequency = 10  # Publishing frequency in Hz

        publish_line(start_x, start_y, start_z, range_y, speed_mm_s, frequency)
    except rospy.ROSInterruptException:
        pass
