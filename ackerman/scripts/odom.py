#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import WheelEncoder, SteeringAngle  # Import custom message types
import math

class OdometryNode:
    def __init__(self):
        rospy.init_node('odometry_node', anonymous=True)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.encoder_sub = rospy.Subscriber("wheel_encoder", WheelEncoder, self.encoder_callback)
        self.steering_sub = rospy.Subscriber("steering_angle", SteeringAngle, self.steering_callback)
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheelbase = 0.5  # Wheelbase of the robot in meters
        self.steering_ratio = 0.5  # Steering ratio
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def encoder_callback(self, data):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Calculate linear velocity for each wheel
        left_wheel_velocity = data.left_wheel_velocity
        right_wheel_velocity = data.right_wheel_velocity

        # Convert wheel velocities to robot velocity (linear and angular)
        self.linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0
        self.angular_velocity = (right_wheel_velocity - left_wheel_velocity) / self.wheelbase

        # Update robot pose using velocities
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        # Publish odometry message
        self.publish_odometry(current_time)

        self.last_time = current_time

    def steering_callback(self, data):
        # Adjust robot orientation based on steering angle
        self.theta += data.steering_angle * self.steering_ratio

    def publish_odometry(self, current_time):
        odom_quat = quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.linear_velocity, 0, 0), Vector3(0, 0, self.angular_velocity))

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        odometry_node = OdometryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
