#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from std_msgs.msg import Float32

class LaserForceCalculator:
    def __init__(self):
        rospy.init_node('laser_force_calculator', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.resultant_magnitude_publisher = rospy.Publisher('/resultant_magnitude', Float32, queue_size=10)
        self.resultant_direction_publisher = rospy.Publisher('/resultant_direction', Float32, queue_size=10)

        # Social force parameters
        self.Fsos = 0.0
        self.Fphy = 0.0
        self.k = 1.0  # Gain
        self.d = 0.0 
        self.psi = 0.20  # Reactive distance in meters
        self.front_degree = 180
        self.a = 1.0  # Semi-major axis
        self.b = 0.5  # Semi-minor axis

    def calculate_force(self, distances, angles):
        total_force_x = 0
        total_force_y = 0

        for distance, angle in zip(distances, angles):
            angle_rad = math.radians(angle)

            # Calculate r for the given angle using the ellipse formula
            r = (self.a * self.b) / np.sqrt((self.b * np.cos(angle_rad))**2 + (self.a * np.sin(angle_rad))**2)

            # Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
            x = r * math.cos(angle_rad)
            y = r * math.sin(angle_rad)

            # Calculate the social force
            fsos = self.k * np.exp((r - distance) / self.psi)

            # Calculate the x and y components of the force
            force_x = fsos * math.cos(angle_rad)
            force_y = fsos * math.sin(angle_rad)

            # Add the components to the total force
            total_force_x += force_x
            total_force_y += force_y

        # Calculate the magnitude and direction of the resultant force
        magnitude = math.sqrt(total_force_x**2 + total_force_y**2)
        direction = math.atan2(total_force_y, total_force_x)

        return magnitude, math.degrees(direction)

    def laser_callback(self, msg):
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        distances = []
        angles = []

        for i, distance in enumerate(ranges):
            if distance > 1.0:  # Filter out distances greater than 1 meter
                continue

            angle = angle_min + i * angle_increment
            angle_degrees = math.degrees(angle)

            distances.append(distance)
            angles.append(angle_degrees)

        magnitude, direction = self.calculate_force(distances, angles)

        # Print the magnitude and direction
        rospy.loginfo(f"Resultant Force Magnitude: {magnitude:.2f} N")
        rospy.loginfo(f"Resultant Force Direction: {direction:.2f} degrees")

        # Publish the resultant force magnitude
        resultant_magnitude_msg = Float32()
        resultant_magnitude_msg.data = magnitude
        self.resultant_magnitude_publisher.publish(resultant_magnitude_msg)

        # Publish the resultant force direction
        resultant_direction_msg = Float32()
        resultant_direction_msg.data = direction
        self.resultant_direction_publisher.publish(resultant_direction_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        calculator = LaserForceCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        pass
