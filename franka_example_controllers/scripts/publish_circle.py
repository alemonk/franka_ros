#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math

def publish_circle(radius, center_x, center_y, center_z, points, rate_hz):
    rospy.init_node('circle_pose_publisher', anonymous=True)
    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(rate_hz)  # Set the rate to publish the poses

    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = "panda_link0"  # Change if necessary

    # Set the desired orientation (60 degrees from the xy plane)
    angle_rad = math.radians(90 + 90)
    pose.pose.orientation.x = math.sin(angle_rad / 2)
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = math.cos(angle_rad / 2)

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
        radius = 0.15  # Radius of the circle
        center_x = 0.5  # X-coordinate of the circle center
        center_y = 0.0  # Y-coordinate of the circle center
        center_z = 0.0  # Z-coordinate of the circle center
        points = 200  # Number of points to define the circle
        rate_hz = 10  # Publishing rate in Hz

        publish_circle(radius, center_x, center_y, center_z, points, rate_hz)
    except rospy.ROSInterruptException:
        pass
