#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
    # Initialize ROS node
    rospy.init_node('velocity_publisher', anonymous=True)

    # Create publisher object
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Set the rate at which to publish messages (10 Hz in this case)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        # Create Twist message and set linear and angular velocities
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # linear velocity in m/s
        twist_msg.angular.z = 0.2  # angular velocity in rad/s

        # Publish the message
        pub.publish(twist_msg)

        # Sleep to maintain the specified rate
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
