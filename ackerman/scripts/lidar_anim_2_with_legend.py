#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


class LaserVisualizer:
    def __init__(self):
        self.distances_90_to_180 = []
        self.angles_90_to_180 = []
        self.distances_minus180_to_minus90 = []
        self.angles_minus180_to_minus90 = []

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)

        self.li, = self.ax.plot(0, 0, 'ko')  # LiDAR center point (black dot)
        self.fixed_goal, = self.ax.plot(2, 0, 'go')  # Fixed goal point (green dot)
        self.nearest_obstacle_90_to_180, = self.ax.plot([], [], 'ro')  # Nearest obstacle (red dot for right side 90 to 180 degrees)
        self.nearest_obstacle_minus180_to_minus90, = self.ax.plot([], [], 'bo')  # Nearest obstacle (blue dot for left side -180 to -90 degrees)
        self.resultant_vector, = self.ax.plot([], [], 'g-')  # Resultant vector line (green line)
        self.ellipse, = self.ax.plot([], [], 'b--')  # Ellipse line (blue dashed line)

        self.annotation_90_to_180 = None
        self.annotation_minus180_to_minus90 = None

        rospy.init_node('laser_visualizer', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=1)  # Set interval to 1 ms for high-speed updates

        # Social force
        self.Fsos = 0.0
        self.Fphy = 0.0
        self.k = 1.0  # Gain
        self.d = 0.0 
        self.psi = 0.20  # Reactive distance in meters
        self.front_degree = 180
        self.a = 1.0  # Semi-major axis
        self.b = 0.5  # Semi-minor axis

        # Initialize the ellipse coordinates
        self.ellipse_x, self.ellipse_y = self.calculate_ellipse_coordinates()

        # Create legends for each line or color
        self.ax.legend(['LiDAR center', 'Fixed goal', 'Nearest obstacle (right side)', 'Nearest obstacle (left side)', 'Resultant vector', 'Ellipse'])

    def calculate_ellipse_coordinates(self):
        angles = np.linspace(0, 2 * np.pi, 100)
        ellipse_x = self.a * np.cos(angles)
        ellipse_y = self.b * np.sin(angles)
        return ellipse_x, ellipse_y

    def calculate_coordinates(self, degree):
        # Convert degree to radians
        angle = np.deg2rad(degree)
        
        # Calculate r for the given degree using the ellipse formula
        r = (self.a * self.b) / np.sqrt((self.b * np.cos(angle))**2 + (self.a * np.sin(angle))**2)
        
        # Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        
        # Calculate distance from the origin
        distance = np.sqrt(x**2 + y**2)    
        return distance

    def force_social(self, distances, degree):
        r_values = self.calculate_coordinates(degree)
        Fsos = self.k * np.exp((r_values - distances) / self.psi)
        return Fsos

    def force_physical(self, distances, degree):
        r_values = self.calculate_coordinates(degree)
        Fphy = self.k * (r_values - distances)
        return Fphy
    
    def laser_callback(self, msg):
        self.distances_90_to_180.clear()
        self.angles_90_to_180.clear()
        self.distances_minus180_to_minus90.clear()
        self.angles_minus180_to_minus90.clear()

        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        for i, distance in enumerate(ranges):
            if distance > 1.0:  # Filter out distances greater than 1 meter
                continue

            angle = angle_min + i * angle_increment
            angle_degrees = math.degrees(angle)

            if 90 <= angle_degrees <= 180:
                self.distances_90_to_180.append((distance, angle_degrees))
            elif -180 <= angle_degrees <= -90:
                self.distances_minus180_to_minus90.append((distance, angle_degrees))

    def update_plot(self, frame):
        nearest_90_to_180 = min(self.distances_90_to_180, key=lambda x: x[0], default=None)
        nearest_minus180_to_minus90 = min(self.distances_minus180_to_minus90, key=lambda x: x[0], default=None)

        if nearest_90_to_180:
            nearest_distance_90_to_180, nearest_angle_90_to_180 = nearest_90_to_180
            x_90_to_180 = nearest_distance_90_to_180 * math.cos(math.radians(nearest_angle_90_to_180))
            y_90_to_180 = nearest_distance_90_to_180 * math.sin(math.radians(nearest_angle_90_to_180))
            sosial = self.force_social(nearest_distance_90_to_180, nearest_angle_90_to_180)
            physical = self.force_physical(nearest_distance_90_to_180, nearest_angle_90_to_180)
            force_static = sosial + physical
            angle_reverse = nearest_angle_90_to_180 * -1
            print(f"Right side (90 to 180 degrees) - Distance: {nearest_distance_90_to_180:.2f}, Force: {force_static:.2f} N")
            
            self.nearest_obstacle_90_to_180.set_data(x_90_to_180, y_90_to_180)
            self.annotation_90_to_180 = self.annotate_obstacle(self.annotation_90_to_180, x_90_to_180, y_90_to_180, f'Distance: {nearest_distance_90_to_180:.2f}\nAngle: {nearest_angle_90_to_180:.2f}')
        else:
            self.nearest_obstacle_90_to_180.set_data([], [])
            if self.annotation_90_to_180:
                self.annotation_90_to_180.remove()
                self.annotation_90_to_180 = None

        if nearest_minus180_to_minus90:
            nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90 = nearest_minus180_to_minus90
            x_minus180_to_minus90 = nearest_distance_minus180_to_minus90 * math.cos(math.radians(nearest_angle_minus180_to_minus90))
            y_minus180_to_minus90 = nearest_distance_minus180_to_minus90 * math.sin(math.radians(nearest_angle_minus180_to_minus90))
            
            rsosial = self.force_social(nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90)
            rphysical = self.force_physical(nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90)
            r_force_static = rsosial + rphysical
            r_angle_reverse = nearest_angle_minus180_to_minus90 * -1
            
            print(f"Left side (-180 to -90 degrees) - Distance: {nearest_distance_minus180_to_minus90:.2f}, Force: {r_force_static:.2f} N")
            
            self.nearest_obstacle_minus180_to_minus90.set_data(x_minus180_to_minus90, y_minus180_to_minus90)
            self.annotation_minus180_to_minus90 = self.annotate_obstacle(self.annotation_minus180_to_minus90, x_minus180_to_minus90, y_minus180_to_minus90, f'Distance: {nearest_distance_minus180_to_minus90:.2f}\nAngle: {nearest_angle_minus180_to_minus90:.2f}')
        else:
            self.nearest_obstacle_minus180_to_minus90.set_data([], [])
            if self.annotation_minus180_to_minus90:
                self.annotation_minus180_to_minus90.remove()
                self.annotation_minus180_to_minus90 = None

        distances = [2]  # Custom distance
        angles = [180]  # Custom angle

        if nearest_90_to_180:
            distances.append(force_static)
            angles.append(angle_reverse)
        if nearest_minus180_to_minus90:
            distances.append(r_force_static)
            angles.append(r_angle_reverse)

        resultant_x, resultant_y, magnitude, direction = self.calculate_resultant_vector(distances, angles)
        self.resultant_vector.set_data([0, resultant_x], [0, resultant_y])

        self.ellipse.set_data(self.ellipse_x, self.ellipse_y)
        print(f"Resultant Force (Fnav): {magnitude:.2f} N")
        print(f"Resultant Direction (rho): {direction:.2f} degrees")
        print("====================================================")
        print("")

        return self.nearest_obstacle_90_to_180, self.nearest_obstacle_minus180_to_minus90, self.resultant_vector, self.ellipse

    def annotate_obstacle(self, annotation, x, y, text):
        if annotation:
            annotation.remove()
        return self.ax.text(x, y, text, fontsize=8, ha='right', va='bottom')

    def calculate_resultant_vector(self, distances, angles):
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

    def run(self):
        plt.show()

if __name__ == '__main__':
    try:
        visualizer = LaserVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
