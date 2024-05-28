#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

def publish_pose():
    rospy.init_node('pose_publisher', anonymous=True)
    pose_pub = rospy.Publisher('/my_controller/equilibrium_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        pose = PoseStamped()

        # Fill in the header
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "panda_link0"  # Change if necessary

        # Set the desired position
        pose.pose.position.x = 0.75
        pose.pose.position.y = 0.25
        pose.pose.position.z = 0.2
        # Set the desired orientation (quaternion)
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        # Publish the pose
        pose_pub.publish(pose)

        time.sleep(5)

        # Set the desired position
        pose.pose.position.x = 0.75
        pose.pose.position.y = -0.25
        pose.pose.position.z = 0.2
        # Set the desired orientation (quaternion)
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        # Publish the pose
        pose_pub.publish(pose)

        time.sleep(5)

        # Set the desired position
        pose.pose.position.x = 0.25
        pose.pose.position.y = -0.25
        pose.pose.position.z = 0.2
        # Set the desired orientation (quaternion)
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        # Publish the pose
        pose_pub.publish(pose)

        time.sleep(5)

        # Set the desired position
        pose.pose.position.x = 0.25
        pose.pose.position.y = 0.25
        pose.pose.position.z = 0.2
        # Set the desired orientation (quaternion)
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        # Publish the pose
        pose_pub.publish(pose)

        time.sleep(5)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
