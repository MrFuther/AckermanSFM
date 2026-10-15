#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.distance_min = 0.0  # Define minimum distance
        self.distance_max = 1.0  # Define maximum distance
        self.scan_data = None

    def scan_callback(self, data):
        # Update scan data
        self.scan_data = data

    def process_scan_data(self):
        if self.scan_data:
            angles = np.arange(self.scan_data.angle_min, self.scan_data.angle_max, self.scan_data.angle_increment)

            # Filter distances based on the defined range
            filtered_data = [(angle, distance) for angle, distance in zip(angles, self.scan_data.ranges) 
                            if self.distance_min <= distance <= self.distance_max]

            # Print the angle and distance for each point within the specified range
            for angle, distance in filtered_data:
                rospy.loginfo(f"Angle: {np.degrees(angle):.2f} degrees, Distance: {distance:.2f} meters")

if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        while not rospy.is_shutdown():
            obstacle_detector.process_scan_data()
            rospy.sleep(1)  # Adjust the sleep duration as needed
    except rospy.ROSInterruptException:
        pass
