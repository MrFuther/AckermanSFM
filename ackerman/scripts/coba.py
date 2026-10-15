#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.near_threshold = 0.5  # Adjust this threshold according to your needs

    def scan_callback(self, data):
        # Loop through the laser scan data and measure distances to detected obstacles
        for i, distance in enumerate(data.ranges):
            if distance < 1.0 and distance > 0:
                print(f"Detected obstacle at angle {data.angle_min + i * data.angle_increment} with distance {distance:.2f} meters")

if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
