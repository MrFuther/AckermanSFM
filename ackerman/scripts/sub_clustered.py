#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np

class ClusteredPointsSubscriber:
    def __init__(self):
        rospy.init_node('clustered_points_subscriber', anonymous=True)
        self.clustered_points_subscriber = rospy.Subscriber('/clustered_points', Point, self.clustered_points_callback)
        self.scan_publisher = rospy.Publisher('/converted_scan', LaserScan, queue_size=10)

    def clustered_points_callback(self, point_msg):
        distance = np.sqrt(point_msg.x**2 + point_msg.y**2)
        angle = np.arctan2(point_msg.y, point_msg.x)
        
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "laser_frame"  # Adjust frame_id according to your setup
        scan_msg.angle_min = angle
        scan_msg.angle_max = angle
        scan_msg.angle_increment = 0  # For single point
        scan_msg.time_increment = 0
        scan_msg.scan_time = 0
        scan_msg.range_min = distance
        scan_msg.range_max = distance
        scan_msg.ranges = [distance]

        self.scan_publisher.publish(scan_msg)

if __name__ == '__main__':
    try:
        clustered_points_subscriber = ClusteredPointsSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
