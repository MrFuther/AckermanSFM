#!/usr/bin/env python3
import serial
import rospy
import math

from std_msgs.msg import Int32, Float32, Float64

# Define serial port settings
port = '/dev/blackpill'
baudrate = 115200

# Open serial port
ser = serial.Serial(port, baudrate)

# Initialize ROS node
rospy.init_node('serial_to_ros_publisher')

# Create ROS publishers
pos_pub = rospy.Publisher('/read_pos', Int32, queue_size=100)
rpm_pub = rospy.Publisher('/read_rpm', Float32, queue_size=100)
odom_pub = rospy.Publisher('/read_odometry', Float64, queue_size=100)
velocity_pub = rospy.Publisher('/read_velocity', Float32, queue_size=100)  # New publisher for velocity

def calculate_odometry(position):
    # Gear reduction
    # 22T/70T * 14T/40T
    # 11/35 * 7/20 = 77/700
    # = 9.09
    encoder_resolution = 14.0
    wheel_diameter = 8.24  # in cm
    gear_reduction = 9.09

    wheel_revolutions = (position / encoder_resolution) / gear_reduction
    wheel_circumference = wheel_diameter * math.pi
    distance_traveled = wheel_revolutions * wheel_circumference
    
    return distance_traveled, wheel_revolutions

def calculate_velocity(rpm_motor):
    # Convert motor RPM to wheel RPM
    gear_reduction = 9.09
    rpm_wheel = rpm_motor / gear_reduction
    
    # Calculate wheel speed in m/s from wheel RPM
    wheel_diameter_m = 8.24 / 100  # Convert cm to meters
    wheel_circumference_m = wheel_diameter_m * math.pi
    wheel_speed_m_per_min = rpm_wheel * wheel_circumference_m  # Speed in meters per minute
    wheel_speed_m_per_s = wheel_speed_m_per_min / 60  # Convert to meters per second
    
    return wheel_speed_m_per_s

try:
    while not rospy.is_shutdown():
        dummy = "1"
        ser.write(dummy.encode())
        data = ser.readline()
        
        data = data.split(b';')
        pos = int(float(data[0]))  
        rpm = float(data[1])
        
        distance_traveled, wheel_revolutions = calculate_odometry(pos)
        velocity = calculate_velocity(rpm)
        
        # Print statements for debugging
        # print("Position:", pos)
        # print("ppr:", wheel_revolutions)
        # print("Odometry:", distance_traveled, "cm")
        # print("Velocity:", velocity, "m/s")
        
        # Publish data to ROS topics
        pos_pub.publish(pos)
        rpm_pub.publish(rpm)
        odom_pub.publish(distance_traveled)
        velocity_pub.publish(velocity)  # Publish velocity
        
except rospy.ROSInterruptException:
    # Close the serial port on ROS interrupt
    ser.close()
    rospy.loginfo("Serial port closed.")
