#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    # Print the range readings and their corresponding angles
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    ranges = data.ranges

    for i, distance in enumerate(ranges):
        angle = angle_min + i * angle_increment
        print(f"Angle: {angle:.2f} radians, Distance: {distance:.2f} meters")

def listener():
    rospy.init_node('hokuyo_subscriber', anonymous=True)
    rospy.Subscriber('/hokuyo', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
