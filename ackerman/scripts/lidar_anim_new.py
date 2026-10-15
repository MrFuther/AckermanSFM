#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

        self.li, = self.ax.plot(0, 0, 'ko')  # LiDAR center point
        self.fixed_goal, = self.ax.plot(2, 0, 'go')  # Fixed goal point
        self.nearest_obstacle_90_to_180, = self.ax.plot([], [], 'ro')  # Nearest obstacle (90 to 180)
        self.nearest_obstacle_minus180_to_minus90, = self.ax.plot([], [], 'bo')  # Nearest obstacle (-180 to -90)
        self.resultant_vector, = self.ax.plot([], [], 'g-')  # Resultant vector line
        self.all_obstacles_90_to_180, = self.ax.plot([], [], 'r.')  # All obstacles (90 to 180)
        self.all_obstacles_minus180_to_minus90, = self.ax.plot([], [], 'b.')  # All obstacles (-180 to -90)
        self.other_obstacles_90_to_180, = self.ax.plot([], [], 'm.')  # Other obstacles (90 to 180)
        self.other_obstacles_minus180_to_minus90, = self.ax.plot([], [], 'c.')  # Other obstacles (-180 to -90)

        self.annotation_90_to_180 = None
        self.annotation_minus180_to_minus90 = None

        rospy.init_node('laser_visualizer', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=10)

    def laser_callback(self, msg):
        self.distances_90_to_180.clear()
        self.angles_90_to_180.clear()
        self.distances_minus180_to_minus90.clear()
        self.angles_minus180_to_minus90.clear()

        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            angle_degrees = math.degrees(angle)

            if 90 <= angle_degrees <= 180:
                self.distances_90_to_180.append((distance, angle_degrees))
            elif -180 <= angle_degrees <= -90:
                self.distances_minus180_to_minus90.append((distance, angle_degrees))

    def update_plot(self, frame):
        nearest_90_to_180 = min(self.distances_90_to_180, key=lambda x: x[0], default=None)
        nearest_minus180_to_minus90 = min(self.distances_minus180_to_minus90, key=lambda x: x[0], default=None)

        x_data_90_to_180 = []
        y_data_90_to_180 = []
        x_data_minus180_to_minus90 = []
        y_data_minus180_to_minus90 = []

        if nearest_90_to_180:
            nearest_distance_90_to_180, nearest_angle_90_to_180 = nearest_90_to_180
            x_nearest_90_to_180 = nearest_distance_90_to_180 * math.cos(math.radians(nearest_angle_90_to_180))
            y_nearest_90_to_180 = nearest_distance_90_to_180 * math.sin(math.radians(nearest_angle_90_to_180))
            self.nearest_obstacle_90_to_180.set_data(x_nearest_90_to_180, y_nearest_90_to_180)
            self.annotation_90_to_180 = self.annotate_obstacle(self.annotation_90_to_180, x_nearest_90_to_180, y_nearest_90_to_180, f'Distance: {nearest_distance_90_to_180:.2f}\nAngle: {nearest_angle_90_to_180:.2f}')
        
        if nearest_minus180_to_minus90:
            nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90 = nearest_minus180_to_minus90
            x_nearest_minus180_to_minus90 = nearest_distance_minus180_to_minus90 * math.cos(math.radians(nearest_angle_minus180_to_minus90))
            y_nearest_minus180_to_minus90 = nearest_distance_minus180_to_minus90 * math.sin(math.radians(nearest_angle_minus180_to_minus90))
            self.nearest_obstacle_minus180_to_minus90.set_data(x_nearest_minus180_to_minus90, y_nearest_minus180_to_minus90)
            self.annotation_minus180_to_minus90 = self.annotate_obstacle(self.annotation_minus180_to_minus90, x_nearest_minus180_to_minus90, y_nearest_minus180_to_minus90, f'Distance: {nearest_distance_minus180_to_minus90:.2f}\nAngle: {nearest_angle_minus180_to_minus90:.2f}')

        resultant_x, resultant_y, magnitude, direction = self.calculate_resultant_vector(nearest_90_to_180, nearest_minus180_to_minus90)
        self.resultant_vector.set_data([0, resultant_x], [0, resultant_y])

        # Plot all obstacles
        x_all_90_to_180 = []
        y_all_90_to_180 = []
        x_other_90_to_180 = []
        y_other_90_to_180 = []
        x_all_minus180_to_minus90 = []
        y_all_minus180_to_minus90 = []
        x_other_minus180_to_minus90 = []
        y_other_minus180_to_minus90 = []

        for distance, angle in self.distances_90_to_180:
            x = distance * math.cos(math.radians(angle))
            y = distance * math.sin(math.radians(angle))
            if (x, y) != (x_nearest_90_to_180, y_nearest_90_to_180):
                x_other_90_to_180.append(x)
                y_other_90_to_180.append(y)
            x_all_90_to_180.append(x)
            y_all_90_to_180.append(y)
        
        for distance, angle in self.distances_minus180_to_minus90:
            x = distance * math.cos(math.radians(angle))
            y = distance * math.sin(math.radians(angle))
            if (x, y) != (x_nearest_minus180_to_minus90, y_nearest_minus180_to_minus90):
                x_other_minus180_to_minus90.append(x)
                y_other_minus180_to_minus90.append(y)
            x_all_minus180_to_minus90.append(x)
            y_all_minus180_to_minus90.append(y)

        self.all_obstacles_90_to_180.set_data(x_all_90_to_180, y_all_90_to_180)
        self.all_obstacles_minus180_to_minus90.set_data(x_all_minus180_to_minus90, y_all_minus180_to_minus90)
        self.other_obstacles_90_to_180.set_data(x_other_90_to_180, y_other_90_to_180)
        self.other_obstacles_minus180_to_minus90.set_data(x_other_minus180_to_minus90, y_other_minus180_to_minus90)

        print("Resultant Vector:")
        print("Magnitude:", magnitude)
        print("Direction:", direction)

        return self.nearest_obstacle_90_to_180, self.nearest_obstacle_minus180_to_minus90, self.resultant_vector, self.all_obstacles_90_to_180, self.all_obstacles_minus180_to_minus90, self.other_obstacles_90_to_180, self.other_obstacles_minus180_to_minus90

    def annotate_obstacle(self, annotation, x, y, text):
        if annotation:
            annotation.remove()
        return self.ax.text(x, y, text, fontsize=8, ha='right', va='bottom')

    def clear_annotations(self):
        if self.annotation_90_to_180:
            self.annotation_90_to_180.remove()
            self.annotation_90_to_180 = None
        if self.annotation_minus180_to_minus90:
            self.annotation_minus180_to_minus90.remove()
            self.annotation_minus180_to_minus90 = None

    def calculate_resultant_vector(self, nearest_90_to_180, nearest_minus180_to_minus90):
        distances = []
        angles = []

        if nearest_90_to_180:
            distances.append(nearest_90_to_180[0])
            angles.append(nearest_90_to_180[1])
        if nearest_minus180_to_minus90:
            distances.append(nearest_minus180_to_minus90[0])
            angles.append(nearest_minus180_to_minus90[1])

        if distances and angles:
            resultant_x, resultant_y, magnitude, direction = self._calculate_resultant_vector(angles, distances)
            return resultant_x, resultant_y, magnitude, direction
        else:
            return 0, 0, 0, 0

    def _calculate_resultant_vector(self, angles, distances):
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
