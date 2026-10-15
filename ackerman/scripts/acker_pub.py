#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point

# Global variables to store the latest values of velocity and rho
latest_velocity = None
latest_rho = None

# Conversion functions
def degrees_to_radians(degrees):
    return degrees * (3.141592653589793 / 180.0)

def map_rho_to_steering_angle(rho):
    # Map the rho value from degrees to radians
    return degrees_to_radians(rho)

# Callback functions for the subscribers
def velocity_callback(data):
    global latest_velocity
    latest_velocity = data.x
    rospy.loginfo("Received velocity: %f", latest_velocity)
    publish_ackermann_drive()

def rho_callback(data):
    global latest_rho
    latest_rho = data.x
    rospy.loginfo("Received rho: %f", latest_rho)
    publish_ackermann_drive()

def publish_ackermann_drive():
    if latest_velocity is not None and latest_rho is not None:
        # Convert rho to steering angle
        steering_angle = map_rho_to_steering_angle(latest_rho)
        
        # Create and publish AckermannDrive message
        drive_msg = AckermannDrive()
        drive_msg.steering_angle = steering_angle
        drive_msg.speed = latest_velocity
        
        pub.publish(drive_msg)
        rospy.loginfo("Published AckermannDrive message with steering_angle: %f, speed: %f", steering_angle, latest_velocity)

def listener():
    rospy.init_node('ackermann_drive_node', anonymous=True)
    
    rospy.Subscriber('/fnav', Point, velocity_callback)
    rospy.Subscriber('/rho', Point, rho_callback)
    
    global pub
    pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
    
    rospy.loginfo("AckermannDrive node started. Waiting for messages on /velocity and /rho...")
    
    rospy.spin()

if __name__ == '__main__':
    listener()
