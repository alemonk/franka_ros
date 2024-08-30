#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ErrorPlotter:
    def __init__(self):
        self.time_data = []

        # Position data
        self.des_x = 0.0
        self.des_y = 0.0
        self.des_z = 0.0
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = 0.0
        self.err_x_data = []
        self.err_y_data = []
        self.err_z_data = []

        # Orientation data (quaternion)
        self.des_qx = 0.0
        self.des_qy = 0.0
        self.des_qz = 0.0
        self.des_qw = 0.0
        self.cur_qx = 0.0
        self.cur_qy = 0.0
        self.cur_qz = 0.0
        self.cur_qw = 0.0
        self.err_qx_data = []
        self.err_qy_data = []
        self.err_qz_data = []
        self.err_qw_data = []

        rospy.init_node('position_orientation_error_plotter', anonymous=True)
        rospy.Subscriber('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, self.desired_pose_callback)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.current_pose_callback)

        self.fig, (self.ax_pos, self.ax_orient) = plt.subplots(2, 1)
        self.fig.canvas.manager.set_window_title('Position and Orientation Error Plotter')
        
        # Create lines for position errors
        self.line_x, = self.ax_pos.plot([], [], 'r-', label='Error X')
        self.line_y, = self.ax_pos.plot([], [], 'g-', label='Error Y')
        self.line_z, = self.ax_pos.plot([], [], 'b-', label='Error Z')
        
        self.ax_pos.set_xlim(0, 60)
        self.ax_pos.set_ylim(-5, 5)
        self.ax_pos.set_xlabel('Time (s)')
        self.ax_pos.set_ylabel('Position Error (cm)')
        self.ax_pos.legend()

        # Create lines for orientation errors
        self.line_qx, = self.ax_orient.plot([], [], 'r-', label='Error QX')
        self.line_qy, = self.ax_orient.plot([], [], 'g-', label='Error QY')
        self.line_qz, = self.ax_orient.plot([], [], 'b-', label='Error QZ')
        self.line_qw, = self.ax_orient.plot([], [], 'y-', label='Error QW')

        self.ax_orient.set_xlim(0, 60)
        self.ax_orient.set_ylim(-1, 1)
        self.ax_orient.set_xlabel('Time (s)')
        self.ax_orient.set_ylabel('Orientation Error')
        self.ax_orient.legend()

        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True)
        plt.tight_layout()
        plt.show()

    def desired_pose_callback(self, msg):
        # Desired position
        self.des_x = msg.pose.position.x
        self.des_y = msg.pose.position.y
        self.des_z = msg.pose.position.z
        
        # Desired orientation
        self.des_qx = msg.pose.orientation.x
        self.des_qy = msg.pose.orientation.y
        self.des_qz = msg.pose.orientation.z
        self.des_qw = msg.pose.orientation.w

    def current_pose_callback(self, msg):
        # Current position
        self.cur_x = msg.O_T_EE[12]
        self.cur_y = msg.O_T_EE[13]
        self.cur_z = msg.O_T_EE[14]
        
        # Current orientation (extracting quaternion from the homogeneous transformation matrix)
        self.cur_qx = msg.O_T_EE[0]  # Assuming O_T_EE stores the rotation as a quaternion or similar
        self.cur_qy = msg.O_T_EE[1]
        self.cur_qz = msg.O_T_EE[2]
        self.cur_qw = msg.O_T_EE[3]

    def init_plot(self):
        # Initialize position error lines
        self.line_x.set_data([], [])
        self.line_y.set_data([], [])
        self.line_z.set_data([], [])
        # Initialize orientation error lines
        self.line_qx.set_data([], [])
        self.line_qy.set_data([], [])
        self.line_qz.set_data([], [])
        self.line_qw.set_data([], [])
        return self.line_x, self.line_y, self.line_z, self.line_qx, self.line_qy, self.line_qz, self.line_qw

    def update_plot(self, frame):
        current_time = rospy.get_time()
        self.time_data.append(current_time)
        
        # Position error calculation
        err_x = 100 * (self.des_x - self.cur_x)
        err_y = 100 * (self.des_y - self.cur_y)
        err_z = 100 * (self.des_z - self.cur_z)
        self.err_x_data.append(err_x)
        self.err_y_data.append(err_y)
        self.err_z_data.append(err_z)

        # Orientation error calculation (simple subtraction, more complex methods can be used)
        err_qx = self.des_qx - self.cur_qx
        err_qy = self.des_qy - self.cur_qy
        err_qz = self.des_qz - self.cur_qz
        err_qw = self.des_qw - self.cur_qw
        self.err_qx_data.append(err_qx)
        self.err_qy_data.append(err_qy)
        self.err_qz_data.append(err_qz)
        self.err_qw_data.append(err_qw)

        print(f"Position Errors: {err_x}, {err_y}, {err_z}")
        print(f"Orientation Errors: {err_qx}, {err_qy}, {err_qz}, {err_qw}")
        print()

        # Keep only the last 60 seconds of data
        if len(self.time_data) > 1:
            while self.time_data[-1] - self.time_data[0] > 60:
                self.time_data.pop(0)
                self.err_x_data.pop(0)
                self.err_y_data.pop(0)
                self.err_z_data.pop(0)
                self.err_qx_data.pop(0)
                self.err_qy_data.pop(0)
                self.err_qz_data.pop(0)
                self.err_qw_data.pop(0)

        # Update the data for position lines
        self.line_x.set_data(self.time_data, self.err_x_data)
        self.line_y.set_data(self.time_data, self.err_y_data)
        self.line_z.set_data(self.time_data, self.err_z_data)
        
        # Update the data for orientation lines
        self.line_qx.set_data(self.time_data, self.err_qx_data)
        self.line_qy.set_data(self.time_data, self.err_qy_data)
        self.line_qz.set_data(self.time_data, self.err_qz_data)
        self.line_qw.set_data(self.time_data, self.err_qw_data)
        
        # Update x-axis limits dynamically for both plots
        self.ax_pos.set_xlim(self.time_data[0], self.time_data[-1])
        self.ax_orient.set_xlim(self.time_data[0], self.time_data[-1])

        return self.line_x, self.line_y, self.line_z, self.line_qx, self.line_qy, self.line_qz, self.line_qw

if __name__ == '__main__':
    try:
        ErrorPlotter()
    except rospy.ROSInterruptException:
        pass
