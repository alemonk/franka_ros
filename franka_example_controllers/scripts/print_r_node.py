import rospy
import numpy as np
from franka_msgs.msg import FrankaState

def determine_rotation(R):
    """ Function to interpret and print the rotation matrix. """
    print("Rotation Matrix R:")
    print(R)

def franka_state_callback(msg):
    """ Callback function for Franka state subscriber. """
    # Extract the rotation matrix from O_T_EE
    R = np.array([
        [msg.O_T_EE[0], msg.O_T_EE[1], msg.O_T_EE[2]],
        [msg.O_T_EE[4], msg.O_T_EE[5], msg.O_T_EE[6]],
        [msg.O_T_EE[8], msg.O_T_EE[9], msg.O_T_EE[10]]
    ])
    
    # Print the rotation matrix
    determine_rotation(R)

def franka_state_subscriber():
    """ Initializes the ROS node and subscriber. """
    rospy.init_node('franka_state_printer', anonymous=True)
    
    # Subscribe to the Franka state topic
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, franka_state_callback)
    
    # Spin to keep the script running and processing callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        franka_state_subscriber()
    except rospy.ROSInterruptException:
        pass
