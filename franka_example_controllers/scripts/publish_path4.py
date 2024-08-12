import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
import math
import tf.transformations as tf_trans
from utils import *
import copy
import time

if __name__ == '__main__':
    try:
        rospy.init_node('circle_pose_publisher', anonymous=True)
        pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
        contact_pub = rospy.Publisher('/cartesian_impedance_example_controller/target_contact', Bool, queue_size=10)
        rate = rospy.Rate(10)

        initial_pose, R = get_current_pose()

        des_pose = PoseStamped()
        des_pose.header = Header()
        des_pose.header.frame_id = "panda_link0"
        des_pose.pose.position = initial_pose.pose.position
        des_pose.pose.orientation.x = 0
        des_pose.pose.orientation.y = 0
        des_pose.pose.orientation.z = 0
        des_pose.pose.orientation.w = 1
        
        time.sleep(3)
        p = move_robot_to_pose(copy.deepcopy(initial_pose), des_pose)
        print(p.pose.orientation)
        
        quat_rotation = tf_trans.quaternion_from_euler(math.pi/2, 0, 0)
        quat_in = np.array([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])
        quat = tf_trans.quaternion_multiply(quat_in, quat_rotation)
        des_pose.pose.orientation.x = quat[0]
        des_pose.pose.orientation.y = quat[1]
        des_pose.pose.orientation.z = quat[2]
        des_pose.pose.orientation.w = quat[3]

        time.sleep(3)
        p = move_robot_to_pose(p, des_pose, duration=3)     
        print(p.pose.orientation)

        quat_rotation = tf_trans.quaternion_from_euler(0, math.pi/2, 0)
        quat_in = np.array([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])
        quat = tf_trans.quaternion_multiply(quat_in, quat_rotation)
        des_pose.pose.orientation.x = quat[0]
        des_pose.pose.orientation.y = quat[1]
        des_pose.pose.orientation.z = quat[2]
        des_pose.pose.orientation.w = quat[3]

        time.sleep(3)
        p = move_robot_to_pose(p, des_pose, duration=10)
        print(p.pose.orientation)

        # # Move back to the initial position
        # rospy.loginfo("Returning to the initial position...")
        # _ = move_robot_to_pose(p, initial_pose, duration=5.0)

    except rospy.ROSInterruptException:
        pass
