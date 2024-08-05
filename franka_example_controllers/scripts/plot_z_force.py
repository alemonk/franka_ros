#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ForcePlotter:
    def __init__(self):
        self.force_z = 0.0
        self.time_data = []
        self.force_data = []
        
        rospy.init_node('force_z_plotter', anonymous=True)
        rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self.callback)

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-')
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-8, 2)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Force Z (N)')

        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True)
        plt.show()

    def callback(self, msg):
        self.force_z = msg.wrench.force.z  # Z-axis force component

    def init_plot(self):
        self.line.set_data([], [])
        return self.line,

    def update_plot(self, frame):
        current_time = rospy.get_time()
        self.time_data.append(current_time)
        self.force_data.append(self.force_z)

        # Keep only the last 10 seconds of data
        if len(self.time_data) > 1:
            while self.time_data[-1] - self.time_data[0] > 10:
                self.time_data.pop(0)
                self.force_data.pop(0)

        self.line.set_data(self.time_data, self.force_data)
        self.ax.set_xlim(self.time_data[0], self.time_data[-1])

        return self.line,

if __name__ == '__main__':
    try:
        ForcePlotter()
    except rospy.ROSInterruptException:
        pass
