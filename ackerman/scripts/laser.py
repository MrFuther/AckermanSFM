#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math

def calculate_resultant_vector(angles, distances):
    if len(angles) != len(distances):
        raise ValueError("The lengths of angles and distances arrays must be the same.")

    angles_rad = [math.radians(angle) for angle in angles]

    x_components = [distance * math.cos(angle) for angle, distance in zip(angles_rad, distances)]
    y_components = [distance * math.sin(angle) for angle, distance in zip(angles_rad, distances)]

    resultant_x = sum(x_components)
    resultant_y = sum(y_components)

    resultant_magnitude = math.sqrt(resultant_x**2 + resultant_y**2)
    resultant_direction = math.degrees(math.atan2(resultant_y, resultant_x))

    return resultant_x, resultant_y, resultant_magnitude, resultant_direction

def laser_callback(msg):
    ranges = msg.ranges
    angle_increment = msg.angle_increment
    angle_min = msg.angle_min

    distances = []
    angle_degrees_array = []

    for i, distance in enumerate(ranges):
        angle = angle_min + i * angle_increment

        # Limit distance to 1 meter
        if distance <= 1.0:
            distances.append(distance)
            angle_degrees_array.append(math.degrees(angle))

    if not distances:
        print("No object detected within 1 meter.")
        return

    resultant_x, resultant_y, magnitude, direction = calculate_resultant_vector(angle_degrees_array, distances)

    print("Resultant X :", resultant_x)
    print("Resultant Y :", resultant_y)
    print("Magnitude :", magnitude)
    print("Direction :", direction)
    print("====================================================")
    print("")

def laser_subscriber():
    rospy.init_node('laser_subscriber', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        laser_subscriber()
    except rospy.ROSInterruptException:
        pass
