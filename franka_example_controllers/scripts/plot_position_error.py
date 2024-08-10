#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ErrorPlotter:
    def __init__(self):
        self.time_data = []

        self.des_x = 0.0
        self.des_y = 0.0
        self.des_z = 0.0

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = 0.0

        self.err_x_data = []
        self.err_y_data = []
        self.err_z_data = []
        
        rospy.init_node('position_error_plotter', anonymous=True)
        rospy.Subscriber('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, self.desired_pose_callback)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.current_pose_callback)

        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title('Position Error Plotter')
        
        # Create lines for x, y, and z errors with different colors
        self.line_x, = self.ax.plot([], [], 'r-', label='Error X')
        self.line_y, = self.ax.plot([], [], 'g-', label='Error Y')
        self.line_z, = self.ax.plot([], [], 'b-', label='Error Z')

        self.ax.set_xlim(0, 60)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Position Error (cm)')

        # Add a legend to distinguish the different lines
        self.ax.legend()

        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True)
        plt.show()

    def desired_pose_callback(self, msg):
        self.des_x = msg.pose.position.x
        self.des_y = msg.pose.position.y
        self.des_z = msg.pose.position.z

    def current_pose_callback(self, msg):
        self.cur_x = msg.O_T_EE[12]
        self.cur_y = msg.O_T_EE[13]
        self.cur_z = msg.O_T_EE[14]

    def init_plot(self):
        # Initialize the lines for x, y, and z with empty data
        self.line_x.set_data([], [])
        self.line_y.set_data([], [])
        self.line_z.set_data([], [])
        return self.line_x, self.line_y, self.line_z

    def update_plot(self, frame):
        current_time = rospy.get_time()
        self.time_data.append(current_time)
        
        # Append the latest errors to the respective lists
        err_x = 100 * (self.des_x - self.cur_x)
        err_y = 100 * (self.des_y - self.cur_y)
        err_z = 100 * (self.des_z - self.cur_z)
        self.err_x_data.append(err_x)
        self.err_y_data.append(err_y)
        self.err_z_data.append(err_z)

        # Keep only the last 60 seconds of data
        if len(self.time_data) > 1:
            while self.time_data[-1] - self.time_data[0] > 60:
                self.time_data.pop(0)
                self.err_x_data.pop(0)
                self.err_y_data.pop(0)
                self.err_z_data.pop(0)

        # Update the data for each line
        self.line_x.set_data(self.time_data, self.err_x_data)
        self.line_y.set_data(self.time_data, self.err_y_data)
        self.line_z.set_data(self.time_data, self.err_z_data)
        
        # Update x-axis limits dynamically
        self.ax.set_xlim(self.time_data[0], self.time_data[-1])

        return self.line_x, self.line_y, self.line_z

if __name__ == '__main__':
    try:
        ErrorPlotter()
    except rospy.ROSInterruptException:
        pass
