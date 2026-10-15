#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def converted_scan_callback(scan_msg):
    rospy.loginfo("Received converted_scan:")
    rospy.loginfo(f"Distance: {scan_msg.range_min} meters")
    rospy.loginfo(f"Angle: {scan_msg.angle_min} radians")

def converted_scan_subscriber():
    rospy.init_node('converted_scan_subscriber', anonymous=True)
    rospy.Subscriber('/converted_scan', LaserScan, converted_scan_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        converted_scan_subscriber()
    except rospy.ROSInterruptException:
        pass
