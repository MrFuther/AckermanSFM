#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Buffer size for plotting
BUFFER_SIZE = 200

class VelocityVisualizer:
    def __init__(self):
        rospy.init_node('velocity_visualizer', anonymous=True)
        self.sfm_vt = 0.0
        self.encoder_vt = 0.0
        self.cmd_vel = 0.0
        self.sfm_buffer = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.encoder_buffer = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.cmdvel_buffer = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)

        rospy.Subscriber('/fnav', Point, self.sfm_callback)
        rospy.Subscriber('/vehicle_speed', Float64, self.encoder_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback)

        # Setup matplotlib
        self.fig, self.ax = plt.subplots()
        self.line_sfm, = self.ax.plot([], [], label='FNAV')
        self.line_encoder, = self.ax.plot([], [], label='Encoder Velocity')
        self.line_cmdvel, = self.ax.plot([], [], label='SFM Commanded Velocity')
        self.ax.set_ylim(-1, 3)
        self.ax.set_xlim(0, BUFFER_SIZE)
        self.ax.set_xlabel('Time (samples)')
        self.ax.set_ylabel('Velocity (m/s)')
        self.ax.legend()

    def sfm_callback(self, msg):
        self.sfm_vt = msg.x
        self.sfm_buffer.append(self.sfm_vt)

    def encoder_callback(self, msg):
        self.encoder_vt = msg.data
        self.encoder_buffer.append(self.encoder_vt)

    def cmdvel_callback(self, msg):
        self.cmd_vel = msg.linear.x
        self.cmdvel_buffer.append(self.cmd_vel)

    def update_plot(self, frame):
        x = list(range(BUFFER_SIZE))
        self.line_sfm.set_data(x, list(self.sfm_buffer))
        self.line_encoder.set_data(x, list(self.encoder_buffer))
        self.line_cmdvel.set_data(x, list(self.cmdvel_buffer))
        return self.line_sfm, self.line_encoder, self.line_cmdvel

    def run(self):
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=True)
        plt.show()

if __name__ == '__main__':
    try:
        visualizer = VelocityVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass