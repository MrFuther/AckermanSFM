import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math
import numpy as np
import threading

class LaserVisualizer:
    def __init__(self):
        self.k = 1.0  # Gain
        self.psi = 0.20  # Reactive distance in meters
        self.a = 1.0  # Semi-major axis
        self.b = 0.5  # Semi-minor axis
        
        rospy.init_node('laser_visualizer', anonymous=True)
        rospy.Subscriber('/hokuyo', LaserScan, self.laser_callback)
        
        self.steering_pub = rospy.Publisher('/rho', Point, queue_size=100)
        self.magnitude_pub = rospy.Publisher('/fnav', Point, queue_size=100)

        self.last_resultant_magnitude = 0.0
        self.last_resultant_angle = 0.0

    def calculate_coordinates(self, degree):
        angle = np.deg2rad(degree)
        r = (self.a * self.b) / np.sqrt((self.b * np.cos(angle))**2 + (self.a * np.sin(angle))**2)
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        return np.sqrt(x**2 + y**2)    

    def calculate_forces(self, distances, degree):
        r_values = self.calculate_coordinates(degree)
        sosial = self.k * np.exp((r_values - distances) / self.psi)
        physical = self.k * (r_values - distances)
        return sosial + physical

    def calculate_resultant_vector(self, angles, distances):
        angle_array = np.deg2rad(angles)
        Fx = distances * np.cos(angle_array)
        Fy = distances * np.sin(angle_array)
        resultant_vector_x = np.sum(Fx)
        resultant_vector_y = np.sum(Fy)
        return np.sqrt(resultant_vector_x**2 + resultant_vector_y**2), np.arctan2(resultant_vector_y, resultant_vector_x)

    def publish_results(self, magnitude, direction):
        steering_msg = Point()
        steering_msg.x = direction
        self.steering_pub.publish(steering_msg)
        
        magnitude_msg = Point()
        magnitude_msg.x = magnitude
        self.magnitude_pub.publish(magnitude_msg)

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        valid_indices = np.where(ranges <= 1.0)[0]
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        # Right obstacle
        right_obstacle = valid_ranges[(valid_angles >= np.deg2rad(-90)) & (valid_angles <= np.deg2rad(0))]
        if len(right_obstacle) > 0:
            force_right = self.calculate_forces(right_obstacle.min(), np.rad2deg(valid_angles[np.argmin(right_obstacle)]))
        else:
            force_right = 0.0

        # Left obstacle
        left_obstacle = valid_ranges[(valid_angles >= np.deg2rad(0)) & (valid_angles <= np.deg2rad(90))]
        if len(left_obstacle) > 0:
            force_left = self.calculate_forces(left_obstacle.min(), np.rad2deg(valid_angles[np.argmin(left_obstacle)]))
        else:
            force_left = 0.0

        distances = np.array([2.0])  # Custom distance
        angles = np.array([180.0])  # Custom angle
        if force_right > 0:
            distances = np.append(distances, force_right)
            angles = np.append(angles, -np.rad2deg(valid_angles[np.argmin(right_obstacle)]))
        if force_left > 0:
            distances = np.append(distances, force_left)
            angles = np.append(angles, np.rad2deg(valid_angles[np.argmin(left_obstacle)]))

        resultant_magnitude, resultant_angle = self.calculate_resultant_vector(angles, distances)

        # Check if the current magnitude and angle are consistent with the previous ones
        if abs(resultant_magnitude - self.last_resultant_magnitude) > 0.01 or abs(resultant_angle - self.last_resultant_angle) > 0.01:
            # Publish only if there is a significant change
            threading.Thread(target=self.publish_results, args=(resultant_magnitude, np.rad2deg(resultant_angle))).start()
            self.last_resultant_magnitude = resultant_magnitude
            self.last_resultant_angle = resultant_angle
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    visualizer = LaserVisualizer()
    visualizer.run()
