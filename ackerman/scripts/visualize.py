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
        self.distance_min = 0.0  # Define minimum distance
        self.distance_max = 1.0  # Define maximum distance
        self.fig, self.ax = plt.subplots()
        self.scan_line, = self.ax.plot([], [], 'bo', markersize=2)  # Plot LaserScan data
        self.ax.set_xlim(-2, 2)  # Set x-axis limits
        self.ax.set_ylim(-2, 2)  # Set y-axis limits
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
            angles = np.arange(self.scan_data.angle_min, self.scan_data.angle_max, self.scan_data.angle_increment)

            # Filter distances based on the defined range
            filtered_data = [(angle, distance) for angle, distance in zip(angles, self.scan_data.ranges) 
                            if self.distance_min <= distance <= self.distance_max]

            # Convert polar coordinates to Cartesian coordinates
            xs = [distance * np.cos(angle) for angle, distance in filtered_data]
            ys = [distance * np.sin(angle) for angle, distance in filtered_data]

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
