#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.near_threshold = 0.2  # Adjust this threshold according to your needs
        self.fig, self.ax = plt.subplots()
        self.scan_line, = self.ax.plot([], [], 'bo', markersize=2)  # Plot LaserScan data
        self.ax.set_xlim(-5, 5)  # Set x-axis limits
        self.ax.set_ylim(-5, 5)   # Set y-axis limits
        self.ax.set_aspect('equal', adjustable='box')  # Equal aspect ratio
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('LaserScan Data')
        self.scan_data = None

    def scan_callback(self, data):
        # Update scan data
        self.scan_data = data

    def update_plot(self, frame):
        if self.scan_data:
            # angles = np.arange(self.scan_data.angle_min, self.scan_data.angle_max, self.scan_data.angle_increment)
            angles = np.arange(0.0, 1.0, self.scan_data.angle_increment)
            
            xs = [distance * np.cos(angle) for distance, angle in zip(self.scan_data.ranges, angles)]
            ys = [distance * np.sin(angle) for distance, angle in zip(self.scan_data.ranges, angles)]

            self.scan_line.set_data(xs, ys)
        return self.scan_line,

    def start(self):
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=True)
        plt.show()

if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        obstacle_detector.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
