#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import csv

class LaserVisualizer:
    def __init__(self):
        self.distances_90_to_180 = []
        self.angles_90_to_180 = []
        self.distances_minus180_to_minus90 = []
        self.angles_minus180_to_minus90 = []

        self.all_distances = []
        self.all_angles = []

        # CSV data storage
        self.csv_data = []

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-2.1, 2.1)
        self.ax.set_ylim(-2.1, 2.1)

        self.li, = self.ax.plot(0, 0, 'ko')  # LiDAR center point (black dot)
        self.fixed_goal, = self.ax.plot(-2, 0, 'go')  # Fixed goal point (green dot)
        self.nearest_obstacle_90_to_180, = self.ax.plot([], [], 'ro')  # Nearest obstacle (red dot for right side 90 to 180 degrees)
        self.nearest_obstacle_minus180_to_minus90, = self.ax.plot([], [], 'bo')  # Nearest obstacle (blue dot for left side -180 to -90 degrees)
        self.resultant_vector, = self.ax.plot([], [], 'g-')  # Resultant vector line (green line)
        self.ellipse, = self.ax.plot([], [], 'b--')  # Ellipse line (blue dashed line)
        self.scan_points, = self.ax.plot([], [], 'k.', markersize=1)  # All scan points (gray dots)

        # Lines from LiDAR center to the nearest obstacles
        self.line_to_obstacle_90_to_180, = self.ax.plot([], [], 'r-')
        self.line_to_obstacle_minus180_to_minus90, = self.ax.plot([], [], 'b-')

        # Add annotation for the fixed goal
        self.ax.text(-2, 0, "Distance: 2m \n", fontsize=8, ha='left', va='top')
        
        self.annotation_90_to_180 = None
        self.annotation_minus180_to_minus90 = None

        # Add text elements for resultant magnitude and direction in the upper left corner
        self.resultant_magnitude_text = self.ax.text(0.05, 0.95, '', fontsize=10, ha='left', va='top', color='green', transform=self.ax.transAxes)
        self.resultant_direction_text = self.ax.text(0.05, 0.90, '', fontsize=10, ha='left', va='top', color='green', transform=self.ax.transAxes)

        rospy.init_node('laser_visualizer', anonymous=True)
        rospy.Subscriber('/hokuyo', LaserScan, self.laser_callback)
        
        # Publishers for the magnitude and direction as Point
        self.magnitude_publisher = rospy.Publisher('/fnav', Point, queue_size=10)
        self.direction_publisher = rospy.Publisher('/rho', Point, queue_size=10)

        self.ani = FuncAnimation(self.fig, self.update_plot, interval=1)  # Set interval to 1 ms for high-speed updates

        # Social force
        self.Fsos = 0.0
        self.Fphy = 0.0
        self.k = 1.0  # Gain
        self.d = 0.0 
        self.psi =1.5  # Reactive distance in meters
        self.front_degree = 180
        self.a = 2.0  # Semi-major axis
        self.b = 1.5  # Semi-minor axis
        
        self.v0 = 2
        self.m = 2.6 #kg
        self.dt = 0.1 #second
        
        self.r_kanan = 0.0
        self.r_kiri = 0.0

        # Initialize the ellipse coordinates
        self.ellipse_x, self.ellipse_y = self.calculate_ellipse_coordinates()

        # Create legends for each line or color
        self.ax.legend(['LiDAR center', 'Fgoal', 'Right Obstacle', 'Left obstacle', 'Fnav', 'Robot Interaction Space'])

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
        self.all_distances.clear()
        self.all_angles.clear()

        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        for i, distance in enumerate(ranges):
            if distance > 3.0:
                continue

            angle = angle_min + i * angle_increment
            angle_degrees = math.degrees(angle)
            
            #comment for real robot 
            
            # if angle_degrees <= 0:
            #     angle_degrees += 180
            # elif angle_degrees >= 0:
            #     angle_degrees -= 180

            self.all_distances.append(distance)
            self.all_angles.append(angle_degrees)

            if 90 <= angle_degrees <= 180:
                self.distances_90_to_180.append((distance, angle_degrees))
            elif -180 <= angle_degrees <= -90:
                self.distances_minus180_to_minus90.append((distance, angle_degrees))

    def update_plot(self, frame):
        # Update scan points in gray
        x_all = [d * math.cos(math.radians(a)) for d, a in zip(self.all_distances, self.all_angles)]
        y_all = [d * math.sin(math.radians(a)) for d, a in zip(self.all_distances, self.all_angles)]
        self.scan_points.set_data(x_all, y_all)

        nearest_90_to_180 = min(self.distances_90_to_180, key=lambda x: x[0], default=None)
        nearest_minus180_to_minus90 = min(self.distances_minus180_to_minus90, key=lambda x: x[0], default=None)

        if nearest_90_to_180:
            nearest_distance_90_to_180, nearest_angle_90_to_180 = nearest_90_to_180
            x_90_to_180 = nearest_distance_90_to_180 * math.cos(math.radians(nearest_angle_90_to_180))
            y_90_to_180 = nearest_distance_90_to_180 * math.sin(math.radians(nearest_angle_90_to_180))
            sosial = self.force_social(nearest_distance_90_to_180, nearest_angle_90_to_180)
            physical = self.force_physical(nearest_distance_90_to_180, nearest_angle_90_to_180)
            
            if(self.calculate_coordinates(nearest_angle_90_to_180) <= nearest_distance_90_to_180):
                force_static = 0.0
                self.r_kanan = 0
            else:
                force_static = sosial + physical
                self.r_kanan = self.calculate_coordinates(nearest_angle_90_to_180)
            
            angle_reverse = nearest_angle_90_to_180 * -1
            print(f"Obstacle kanan (degree): {nearest_angle_90_to_180:.2f}, Distance: {nearest_distance_90_to_180:.2f} m, Force: {force_static:.2f} N")
            
            self.nearest_obstacle_90_to_180.set_data(x_90_to_180, y_90_to_180)
            self.annotation_90_to_180 = self.annotate_obstacle(self.annotation_90_to_180, x_90_to_180, y_90_to_180, f'Distance: {nearest_distance_90_to_180:.2f}\nAngle: {nearest_angle_90_to_180:.2f}')
            
            self.line_to_obstacle_90_to_180.set_data([0, x_90_to_180], [0, y_90_to_180])
        else:
            self.nearest_obstacle_90_to_180.set_data([], [])
            self.line_to_obstacle_90_to_180.set_data([], [])
            if self.annotation_90_to_180:
                self.annotation_90_to_180.remove()
                self.annotation_90_to_180 = None

        if nearest_minus180_to_minus90:
            nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90 = nearest_minus180_to_minus90
            x_minus180_to_minus90 = nearest_distance_minus180_to_minus90 * math.cos(math.radians(nearest_angle_minus180_to_minus90))
            y_minus180_to_minus90 = nearest_distance_minus180_to_minus90 * math.sin(math.radians(nearest_angle_minus180_to_minus90))
            
            rsosial = self.force_social(nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90)
            rphysical = self.force_physical(nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90)
            
            
            if(self.calculate_coordinates(nearest_angle_minus180_to_minus90) <= nearest_distance_minus180_to_minus90):
                r_force_static = 0.0
                self.r_kiri = 0
            else:
                r_force_static = rsosial + rphysical
                self.r_kiri = self.calculate_coordinates(nearest_angle_minus180_to_minus90)
                

            r_angle_reverse = nearest_angle_minus180_to_minus90 * -1
            
            print(f"Obstacle kiri (degree): {nearest_angle_minus180_to_minus90:.2f}, Distance: {nearest_distance_minus180_to_minus90:.2f} m, Force: {r_force_static:.2f} N")
            
            self.nearest_obstacle_minus180_to_minus90.set_data(x_minus180_to_minus90, y_minus180_to_minus90)
            self.annotation_minus180_to_minus90 = self.annotate_obstacle(self.annotation_minus180_to_minus90, x_minus180_to_minus90, y_minus180_to_minus90, f'Distance: {nearest_distance_minus180_to_minus90:.2f}\nAngle: {nearest_angle_minus180_to_minus90:.2f}')
            
            self.line_to_obstacle_minus180_to_minus90.set_data([0, x_minus180_to_minus90], [0, y_minus180_to_minus90])
        else:
            self.nearest_obstacle_minus180_to_minus90.set_data([], [])
            self.line_to_obstacle_minus180_to_minus90.set_data([], [])
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
            
        print("Dis :", distances)
        print("Ang :", angles)

        distance_array = np.array(distances)
        angle_array = np.deg2rad(angles)

        Fx = distance_array * np.cos(angle_array)
        Fy = distance_array * np.sin(angle_array)
        
        resultant_vector_x = np.sum(Fx)
        resultant_vector_y = np.sum(Fy)
        
        resultant_magnitude = np.sqrt(resultant_vector_x**2 + resultant_vector_y**2)
        resultant_angle = np.arctan2(resultant_vector_y, resultant_vector_x)
        
        velocity = self.v0 + (resultant_magnitude/self.m)* self.dt
        print("velocity :", velocity)
        print("Fnav :", resultant_magnitude)
        print("Rho :", np.degrees(resultant_angle))
        print("")

        # Publish the resultant magnitude
        magnitude_msg = Point(x=velocity, y=0, z=0)
        self.magnitude_publisher.publish(magnitude_msg)

        # Publish the resultant direction
        direction_msg = Point(x=np.degrees(resultant_angle), y=0, z=0)
        self.direction_publisher.publish(direction_msg)

        self.resultant_vector.set_data([0, resultant_vector_x], [0, resultant_vector_y])
        self.ellipse.set_data(self.ellipse_x, self.ellipse_y)

        # Update text elements with the resultant magnitude and direction
        self.resultant_magnitude_text.set_text(f'Fnav: {resultant_magnitude:.2f} N')
        self.resultant_direction_text.set_text(f'Rho: {np.degrees(resultant_angle):.2f}°')

        # Collect data for CSV
        self.csv_data.append([
            # kanan
            nearest_distance_90_to_180 if nearest_90_to_180 else '',
            nearest_angle_90_to_180 if nearest_90_to_180 else '',
            force_static if nearest_90_to_180 else '',
            sosial if nearest_90_to_180 else '',
            physical if nearest_90_to_180 else '',
            self.r_kanan,
            # kiri
            nearest_distance_minus180_to_minus90 if nearest_minus180_to_minus90 else '',
            nearest_angle_minus180_to_minus90 if nearest_minus180_to_minus90 else '',
            r_force_static if nearest_minus180_to_minus90 else '',
            rsosial if nearest_minus180_to_minus90 else '',
            rphysical if nearest_minus180_to_minus90 else '',
            self.r_kiri,
            resultant_magnitude,
            np.degrees(resultant_angle),
            velocity,
            self.v0
        ])

        # Export data to CSV periodically
        if len(self.csv_data) % 100 == 0:  # Adjust the condition as needed
            self.export_csv()

        return self.nearest_obstacle_90_to_180, self.nearest_obstacle_minus180_to_minus90, self.resultant_vector, self.ellipse, self.line_to_obstacle_90_to_180, self.line_to_obstacle_minus180_to_minus90, self.scan_points

    def annotate_obstacle(self, annotation, x, y, text):
        if annotation:
            annotation.remove()
        annotation = self.ax.annotate(text, xy=(x, y), xytext=(x + 0.1, y + 0.1), fontsize=8, ha='left', va='top')
        return annotation

    def export_csv(self):
        with open('laser_data.csv', 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([
                'Right Obstacle Distance', 'Right Obstacle Angle',
                'Force Static', 'Sosial', 'Physical', 'r_kanan',
                'Left Obstacle Distance', 'Left Obstacle Angle',
                'R Force Static', 'RSosial', 'RPhysical', 'r_kiri',
                'Resultant Magnitude', 'Resultant Angle', 'Velocity', 'v0'
            ])
            csv_writer.writerows(self.csv_data)

    def run(self):
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    visualizer = LaserVisualizer()
    visualizer.run()
