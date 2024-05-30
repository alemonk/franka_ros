#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from std_msgs.msg import Header
import numpy as np

class SmoothTransition:
    def __init__(self):
        rospy.init_node('smooth_transition_node', anonymous=True)

        # Initialize publishers and subscribers
        self.waypoint_sub = rospy.Subscriber('/hybrid_controller/waypoint', PoseStamped, self.waypoint_callback)
        self.state_sub = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.state_callback)
        self.equilibrium_pub = rospy.Publisher('/hybrid_controller/equilibrium_pose', PoseStamped, queue_size=10)

        self.target_pose = None
        self.current_pose = None
        self.rate = rospy.Rate(100)  # Control loop rate in Hz

    def waypoint_callback(self, msg):
        self.target_pose = msg.pose
        rospy.loginfo("Received new target pose")

    def state_callback(self, msg):
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = msg.O_T_EE_d[12]
        self.current_pose.pose.position.y = msg.O_T_EE_d[13]
        self.current_pose.pose.position.z = msg.O_T_EE_d[14]
        self.current_pose.pose.orientation.x = msg.O_T_EE_d[0]
        self.current_pose.pose.orientation.y = msg.O_T_EE_d[1]
        self.current_pose.pose.orientation.z = msg.O_T_EE_d[2]
        self.current_pose.pose.orientation.w = msg.O_T_EE_d[3]

    def publish_equilibrium_pose(self):
        if self.target_pose is None or self.current_pose is None:
            return

        # Define transition parameters
        transition_time = 5.0  # seconds
        start_time = rospy.Time.now()

        # Initialize linear interpolation factors
        start_pos = np.array([self.current_pose.pose.position.x,
                              self.current_pose.pose.position.y,
                              self.current_pose.pose.position.z])
        target_pos = np.array([self.target_pose.position.x,
                               self.target_pose.position.y,
                               self.target_pose.position.z])
        
        start_ori = np.array([self.current_pose.pose.orientation.x,
                              self.current_pose.pose.orientation.y,
                              self.current_pose.pose.orientation.z,
                              self.current_pose.pose.orientation.w])
        target_ori = np.array([self.target_pose.orientation.x,
                               self.target_pose.orientation.y,
                               self.target_pose.orientation.z,
                               self.target_pose.orientation.w])
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = (current_time - start_time).to_sec()

            if elapsed_time > transition_time:
                break

            t = elapsed_time / transition_time

            # Linear interpolation for position
            interp_pos = (1 - t) * start_pos + t * target_pos
            # Spherical linear interpolation for orientation (quaternion)
            interp_ori = self.slerp(start_ori, target_ori, t)

            equilibrium_pose = PoseStamped()
            equilibrium_pose.header = Header()
            equilibrium_pose.header.stamp = rospy.Time.now()
            equilibrium_pose.header.frame_id = "panda_link0"
            equilibrium_pose.pose.position.x = interp_pos[0]
            equilibrium_pose.pose.position.y = interp_pos[1]
            equilibrium_pose.pose.position.z = interp_pos[2]
            equilibrium_pose.pose.orientation.x = interp_ori[0]
            equilibrium_pose.pose.orientation.y = interp_ori[1]
            equilibrium_pose.pose.orientation.z = interp_ori[2]
            equilibrium_pose.pose.orientation.w = interp_ori[3]

            self.equilibrium_pub.publish(equilibrium_pose)
            self.rate.sleep()

    def slerp(self, start, target, t):
        """Spherical linear interpolation of quaternions."""
        dot = np.dot(start, target)

        if dot < 0.0:
            target = -target
            dot = -dot

        DOT_THRESHOLD = 0.9995
        if dot > DOT_THRESHOLD:
            result = start + t * (target - start)
            return result / np.linalg.norm(result)

        theta_0 = np.arccos(dot)
        theta = theta_0 * t

        sin_theta = np.sin(theta)
        sin_theta_0 = np.sin(theta_0)

        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return s0 * start + s1 * target

if __name__ == '__main__':
    try:
        smooth_transition = SmoothTransition()
        while not rospy.is_shutdown():
            smooth_transition.publish_equilibrium_pose()
    except rospy.ROSInterruptException:
        pass
