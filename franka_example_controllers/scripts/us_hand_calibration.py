import rospy
from utils import get_current_pose, rotation_motion, linear_motion_z_axis
import time
import getpass

if __name__ == '__main__':
    try:
        # Parameters for the screw motion
        distance = 0.2  # Distance perpendicular to the end effector's original position in meters

        rospy.init_node('screw_pose_publisher', anonymous=True)
        getpass.getpass("Press Enter to start...")

        initial_pose, _ = get_current_pose()
        initial_pose = rotation_motion(distance, 30, [1, 0, 0])
        initial_pose = linear_motion_z_axis(distance)
        initial_pose = linear_motion_z_axis(-distance)
        initial_pose = rotation_motion(distance, -60, [1, 0, 0])
        initial_pose = linear_motion_z_axis(distance)
        initial_pose = linear_motion_z_axis(-distance)
        initial_pose = rotation_motion(distance, 30, [1, 0, 0])

        initial_pose = rotation_motion(distance, 30, [0, 1, 0])
        initial_pose = linear_motion_z_axis(distance)
        initial_pose = linear_motion_z_axis(-distance)
        initial_pose = rotation_motion(distance, -60, [0, 1, 0])
        initial_pose = linear_motion_z_axis(distance)
        initial_pose = linear_motion_z_axis(-distance)
        initial_pose = rotation_motion(distance, 30, [0, 1, 0])

        initial_pose = rotation_motion(distance, 30, [0, 0, 1])
        initial_pose = rotation_motion(distance, -60, [0, 0, 1])
        initial_pose = rotation_motion(distance, 30, [0, 0, 1])

    except rospy.ROSInterruptException:
        pass