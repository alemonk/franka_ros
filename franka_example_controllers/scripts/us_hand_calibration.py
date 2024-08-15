import rospy
from utils import rotation_motion, linear_motion_z_axis, get_current_pose, move_robot_to_pose
import getpass
import copy
import numpy as np

if __name__ == '__main__':
    try:
        distance = 0.1  # Distance perpendicular to the end effector's original position in meters
        distance_from_axis = np.array([0, 0, -distance])

        rospy.init_node('screw_pose_publisher', anonymous=True)
        getpass.getpass("Press Enter to start...")

        current_pose, _ = get_current_pose()

        # x axis
        p = rotation_motion(current_pose, distance_from_axis, 30, [1, 0, 0])
        p = linear_motion_z_axis(p, -0.01)
        p = linear_motion_z_axis(p, 0.01)
        p = rotation_motion(p, distance_from_axis, -60, [1, 0, 0])
        p = linear_motion_z_axis(p, -0.01)
        p = linear_motion_z_axis(p, 0.01)
        p = rotation_motion(p, distance_from_axis, 30, [1, 0, 0])

        # y axis
        p = rotation_motion(p, distance_from_axis, 30, [0, 1, 0])
        p = linear_motion_z_axis(p, -0.01)
        p = linear_motion_z_axis(p, 0.01)
        p = rotation_motion(p, distance_from_axis, -60, [0, 1, 0])
        p = linear_motion_z_axis(p, -0.01)
        p = linear_motion_z_axis(p, 0.01)
        p = rotation_motion(p, distance_from_axis, 30, [0, 1, 0])

        # z axis
        p = rotation_motion(p, distance_from_axis, 30, [0, 0, 1])
        p = linear_motion_z_axis(p, -0.01)
        p = linear_motion_z_axis(p, 0.01)
        p = rotation_motion(p, distance_from_axis, -60, [0, 0, 1])
        p = linear_motion_z_axis(p, -0.01)
        p = linear_motion_z_axis(p, 0.01)
        p = rotation_motion(p, distance_from_axis, 30, [0, 0, 1])

    except rospy.ROSInterruptException:
        pass