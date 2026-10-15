#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math

class AngleMeasurement:
    def __init__(self):
        rospy.init_node('angle_measurement', anonymous=True)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.target_angle_degrees = 360  # Adjust this angle according to your needs
        self.target_angle_radians = math.radians(self.target_angle_degrees)
        self.distance = None

    def scan_callback(self, data):
        # Calculate the index corresponding to the target angle
        angle_increment = data.angle_increment
        target_index = int(round(self.target_angle_radians / angle_increment))

        # Check if the target index is within the valid range
        if 0 <= target_index < len(data.ranges):
            self.distance = data.ranges[target_index]
            rospy.loginfo(f"Distance at {self.target_angle_degrees} degrees: {self.distance:.2f} meters")
        else:
            rospy.logwarn("Target angle is out of range.")

if __name__ == '__main__':
    try:
        angle_measurement = AngleMeasurement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
