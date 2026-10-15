#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, Twist
import math

class AckermanSteering:
    def __init__(self):
        rospy.init_node('ackerman_steering_node', anonymous=True)
        
        self.l = rospy.get_param('~wheel_base', 1.0)
        self.velocity = rospy.get_param('~velocity', 1.0)

        self.pose = Pose2D()
        
        rospy.Subscriber('/ackerman_cmd', Twist, self.ackerman_callback)
        self.pose_pub = rospy.Publisher('/pose', Pose2D, queue_size=10)

        self.rate = rospy.Rate(10)
        self.run()

    def ackerman_callback(self, msg):
        rho = msg.angular.z
        V = self.velocity
        
        self.pose.theta += (V * math.sin(rho)) / self.l
        self.pose.x += V * math.cos(rho + self.pose.theta)
        self.pose.y += V * math.sin(rho + self.pose.theta)

    def run(self):
        while not rospy.is_shutdown():
            self.pose_pub.publish(self.pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        AckermanSteering()
    except rospy.ROSInterruptException:
        pass
