#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

def publish_pose():
    rospy.init_node('pose_publisher', anonymous=True)
    # pose_pub = rospy.Publisher('/my_controller/waypoint', PoseStamped, queue_size=10)
    # pose_pub = rospy.Publisher('/hybrid_controller/waypoint', PoseStamped, queue_size=10)
    pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    pose = PoseStamped()

    # Fill in the header
    pose.header = Header()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "panda_link0"  # Change if necessary

    # Set the desired position
    pose.pose.position.x = 0.5
    pose.pose.position.y = 0.2
    pose.pose.position.z = 0.5
    # Set the desired orientation (quaternion)
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    # Publish the pose
    pose_pub.publish(pose)

    time.sleep(10)

    # Set the desired position
    pose.pose.position.x = 0.5
    pose.pose.position.y = -0.2
    pose.pose.position.z = -0.5
    # Set the desired orientation (quaternion)
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    # Publish the pose
    pose_pub.publish(pose)

    time.sleep(10)

    # Set the desired position
    pose.pose.position.x = 0.2
    pose.pose.position.y = -0.2
    pose.pose.position.z = 0.5
    # Set the desired orientation (quaternion)
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    # Publish the pose
    pose_pub.publish(pose)

    time.sleep(10)

    # Set the desired position
    pose.pose.position.x = 0.2
    pose.pose.position.y = 0.2
    pose.pose.position.z = 0.5
    # Set the desired orientation (quaternion)
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    # Publish the pose
    pose_pub.publish(pose)

    time.sleep(10)


if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
