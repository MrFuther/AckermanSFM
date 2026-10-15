#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16, Float64
import sys
import select
import tty
import termios

# Function to get keyboard input without blocking
def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    # Initialize the ROS node
    rospy.init_node('multi_publisher', anonymous=True)

    # Create publishers for both topics
    pub_v1 = rospy.Publisher('v1', UInt16, queue_size=10)
    pub_velocity2 = rospy.Publisher('velocity2', Float64, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize the values to be published
    v1_value = 100  # For 'v1' topic
    velocity2_value = 2000.0  # For 'velocity2' topic, starting at 2000

    print("Controls:")
    print("  'w'/'s': Increase/Decrease v1 (starts at 100, cannot go below 0)")
    print("  'a'/'d': Decrease/Increase velocity2 (starts at 2000, range: 1800 to 2300)")
    print("  'q': Quit")

    while not rospy.is_shutdown():
        # Get keyboard input
        key = getKey()

        # Handle 'v1' topic ('w'/'s')
        if key == 'w':
            v1_value += 1
            print(f"v1: {v1_value}")
        elif key == 's':
            if v1_value > 0:  # Prevent going below 0
                v1_value -= 1
            print(f"v1: {v1_value}")

        # Handle 'velocity2' topic ('a'/'d')
        elif key == 'a':
            if velocity2_value > 1800:  # Prevent going below 1800
                velocity2_value -= 10
            print(f"velocity2: {velocity2_value}")
        elif key == 'd':
            if velocity2_value < 2300:  # Prevent going above 2300
                velocity2_value += 10
            print(f"velocity2: {velocity2_value}")

        # Quit when 'q' is pressed
        elif key == 'q':
            v1_value = 0
            velocity2_value = 2000.0
            msg_v1 = UInt16()
            msg_v1.data = v1_value
            pub_v1.publish(msg_v1)

            msg_velocity2 = Float64()
            msg_velocity2.data = velocity2_value
            pub_velocity2.publish(msg_velocity2)
            print(f"v1: {v1_value}, velocity2: {velocity2_value}")
            print("Exiting...")
            break

        # Publish the current values as messages
        msg_v1 = UInt16()
        msg_v1.data = v1_value
        pub_v1.publish(msg_v1)

        msg_velocity2 = Float64()
        msg_velocity2.data = velocity2_value
        pub_velocity2.publish(msg_velocity2)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass