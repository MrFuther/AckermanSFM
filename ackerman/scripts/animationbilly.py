#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class LaserVisualizer:
    def __init__(self):
        self.distances_90_to_180 = []
        self.distances_minus180_to_minus90 = []
        self.distances_minus150_to_150= []

        self.all_distances = []
        self.all_angles = []

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-4.1,4.1)
        self.ax.set_ylim(-4.1, 4.1)

        self.li, = self.ax.plot(0, 0, 'ko')  # LiDAR center point (black dot)
        #self.fixed_goal, = self.ax.plot(-2, 0, 'go')  # Fixed goal point (green dot)
        self.nearest_obstacle_90_to_180, = self.ax.plot([], [], 'ro')  # Nearest obstacle (red dot for right side 90 to 180 degrees)
        self.nearest_obstacle_minus180_to_minus90, = self.ax.plot([], [], 'bo')  # Nearest obstacle (blue dot for left side -180 to -90 degrees)
        self.nearest_obstacle_minus150_to_150, = self.ax.plot([], [], 'yo')  # Nearest obstacle (blue dot for left side -180 to -90 degrees)
        self.resultant_vector, = self.ax.plot([], [], 'g-')  # Resultant vector line (green line)
        self.ellipse, = self.ax.plot([], [], 'b--')  # Ellipse line (blue dashed line)
        self.scan_points, = self.ax.plot([], [], 'k.', markersize=1)  # All scan points (gray dots)

        # Lines from LiDAR center to the nearest obstacles
        self.line_to_obstacle_90_to_180, = self.ax.plot([], [], 'r-')
        self.line_to_obstacle_minus180_to_minus90, = self.ax.plot([], [], 'b-')
        self.line_to_obstacle_minus150_to_150, = self.ax.plot([], [], 'y-')

        # Add annotation for the fixed goal
        #self.ax.text(-2, 0, "Distance: 2m \n", fontsize=8, ha='left', va='top')
        
        self.annotation_90_to_180 = None
        self.annotation_minus180_to_minus90 = None
        self.annotation_minus150_to_150 = None
        
        # Add text elements for resultant magnitude and direction in the upper left corner
        self.resultant_magnitude_text = self.ax.text(0.05, 0.95, '', fontsize=10, ha='left', va='top', color='green', transform=self.ax.transAxes)
        self.resultant_direction_text = self.ax.text(0.05, 0.90, '', fontsize=10, ha='left', va='top', color='green', transform=self.ax.transAxes)

        rospy.init_node('animation', anonymous=True)

        rospy.Subscriber('/fnav', Point, self.magnitude_callback)
        rospy.Subscriber('/rho', Point, self.direction_callback)
        rospy.Subscriber("/resultant_vector_y", Float32, self.vector_y_callback)
        rospy.Subscriber("/resultant_vector_x", Float32, self.vector_x_callback)

        rospy.Subscriber("/all_distances", Float32MultiArray, self.all_distances_callback)
        rospy.Subscriber("/all_angles", Float32MultiArray, self.all_angles_callback)
        rospy.Subscriber("/distances_90_to_180", Float32MultiArray, self.distances_90_to_180_callback)
        rospy.Subscriber("/distances_minus180_to_minus90", Float32MultiArray, self.distances_minus180_to_minus90_callback)
        rospy.Subscriber("/distances_minus150_to_150", Float32MultiArray, self.distances_minus150_to_150_callback)
        rospy.Subscriber("/vehicle_speed", Float64, self.vehicle_speed_callback)
        rospy.Subscriber("/v0_target", Float64, self.v0_callback)

        self.ani = FuncAnimation(self.fig, self.update_plot, interval=1)  # Set interval to 1 ms for high-speed updates

        self.d = 0.0 
        self.h = 0  # x-coordinate of the ellipse center
        self.k = 0   # y-coordinate of the ellipse center
        self.current_vehicle_speed = 0.0
        self.current_v0 = 1.6  # Nilai default v0 dari keyboard_handler
        self.base_a = 1.5  # Sesuai dengan sfm.cpp
        self.base_b = 0.5  # Sesuai dengan sfm.cpp
        self.resultant_magnitude = 0
        self.resultant_angle = 0
        self.resultant_vector_y = 0
        self.resultant_vector_x = 0

        # Static ellipse (centered at 0,0)
        self.robot_ellipse_x, self.robot_ellipse_y = self.calculate_ellipse_coordinates(
            h=0, k=0, a=self.base_a, b=self.base_b
        )
        self.robot_ellipse, = self.ax.plot(
            self.robot_ellipse_x,
            self.robot_ellipse_y,
            'r--',  # Red dashed line
            label='Robot Ellipse'
        )

        # Create legends for each line or color
        #self.ax.legend(['LiDAR center', 'Fgoal', 'Right Obstacle', 'Left obstacle', 'Front Obstacle', 'Fnav', 'Robot Interaction Space'])

        # Register the shutdown function for ROS
        rospy.on_shutdown(self.shutdown)

        # Register close event for the matplotlib window
        self.fig.canvas.mpl_connect('close_event', self.on_close)

    def calculate_ellipse_coordinates(self, h, k, a, b):
        angles = np.linspace(0, 2 * np.pi, 100)
        ellipse_x = h + a * np.cos(angles)
        ellipse_y = k + b * np.sin(angles)
        return ellipse_x, ellipse_y
        
    def vector_y_callback(self, data):
        self.resultant_vector_y = data.data

    def vector_x_callback(self, data):
        self.resultant_vector_x = data.data
    
    def vehicle_speed_callback(self, msg):
        self.current_vehicle_speed = msg.data
        if self.current_vehicle_speed < 0:
            self.current_vehicle_speed = 0.0
    
    def v0_callback(self, msg):
        # Gunakan 0.1 sebagai batas minimum
        self.current_v0 = msg.data if msg.data > 0.1 else 0.1
    
    def magnitude_callback(self, msg):
        self.resultant_magnitude = msg.x

    def direction_callback(self, msg):
        self.resultant_angle = msg.x

    def all_distances_callback(self, msg):
        self.all_distances = msg.data

    def all_angles_callback(self, msg):
        self.all_angles = msg.data

    def distances_90_to_180_callback(self, msg):
        self.distances_90_to_180 = []
        for i in range(0, len(msg.data), 2):
            distance = msg.data[i]
            angle = msg.data[i + 1]
            self.distances_90_to_180.append((distance, angle))

    def distances_minus180_to_minus90_callback(self, msg):
        self.distances_minus180_to_minus90 = []
        for i in range(0, len(msg.data), 2):
            distance = msg.data[i]
            angle = msg.data[i + 1]
            self.distances_minus180_to_minus90.append((distance, angle))

    def distances_minus150_to_150_callback(self, msg):
        self.distances_minus150_to_150 = []
        for i in range(0, len(msg.data), 2):
            distance = msg.data[i]
            angle = msg.data[i + 1]
            self.distances_minus150_to_150.append((distance, angle))            

    def update_plot(self, frame):
        # Update scan points in gray
        x_all = [d * math.cos(math.radians(a)) for d, a in zip(self.all_distances, self.all_angles)]
        y_all = [d * math.sin(math.radians(a)) for d, a in zip(self.all_distances, self.all_angles)]
        self.scan_points.set_data(x_all, y_all)

        # --- Blok Logika Dinamis Baru ---
        MAX_SCALE_INCREASE = 1.0  # Membesar 100% (2x lipat)
        
        speed_factor = 1.0 + MAX_SCALE_INCREASE * (self.current_vehicle_speed / max(self.current_v0, 1e-6))
        
        # Hitung a dan b dinamis untuk elips ROBOT
        dynamic_a = self.base_a * speed_factor
        dynamic_b = self.base_b 
        
        # Hitung dan gambar ulang elips ROBOT (merah)
        robot_x, robot_y = self.calculate_ellipse_coordinates(
            h=0, k=0, a=dynamic_a, b=dynamic_b
        )
        self.robot_ellipse.set_data(robot_x, robot_y)
        # --- Akhir Blok Baru ---

        nearest_90_to_180 = min(self.distances_90_to_180, key=lambda x: x[0], default=None)
        nearest_minus180_to_minus90 = min(self.distances_minus180_to_minus90, key=lambda x: x[0], default=None)
        nearest_minus150_to_150 = min(self.distances_minus150_to_150, key=lambda x: x[0], default=None)

        """if nearest_90_to_180:
            nearest_distance_90_to_180, nearest_angle_90_to_180 = nearest_90_to_180
            x_90_to_180 = nearest_distance_90_to_180 * math.cos(math.radians(nearest_angle_90_to_180))
            y_90_to_180 = nearest_distance_90_to_180 * math.sin(math.radians(nearest_angle_90_to_180))
            
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
            
            self.nearest_obstacle_minus180_to_minus90.set_data(x_minus180_to_minus90, y_minus180_to_minus90)
            self.annotation_minus180_to_minus90 = self.annotate_obstacle(self.annotation_minus180_to_minus90, x_minus180_to_minus90, y_minus180_to_minus90, f'Distance: {nearest_distance_minus180_to_minus90:.2f}\nAngle: {nearest_angle_minus180_to_minus90:.2f}')
            
            self.line_to_obstacle_minus180_to_minus90.set_data([0, x_minus180_to_minus90], [0, y_minus180_to_minus90])

        else:
            self.nearest_obstacle_minus180_to_minus90.set_data([], [])
            self.line_to_obstacle_minus180_to_minus90.set_data([], [])
            if self.annotation_minus180_to_minus90:
                self.annotation_minus180_to_minus90.remove()
                self.annotation_minus180_to_minus90 = None"""

        if nearest_minus150_to_150:
            nearest_distance_minus150_to_150, nearest_angle_minus150_to_150 = nearest_minus150_to_150
            x_minus150_to_150 = nearest_distance_minus150_to_150 * math.cos(math.radians(nearest_angle_minus150_to_150))
            y_minus150_to_150 = nearest_distance_minus150_to_150 * math.sin(math.radians(nearest_angle_minus150_to_150))
            
            self.nearest_obstacle_minus150_to_150.set_data(x_minus150_to_150, y_minus150_to_150)
            self.annotation_minus150_to_150 = self.annotate_obstacle(self.annotation_minus150_to_150, x_minus150_to_150, y_minus150_to_150, f'Distance: {nearest_distance_minus150_to_150:.2f}\nAngle: {nearest_angle_minus150_to_150:.2f}')

            self.line_to_obstacle_minus150_to_150.set_data([0, x_minus150_to_150], [0, y_minus150_to_150])
            self.h = x_minus150_to_150
            self.k = y_minus150_to_150
            
            # Hitung dan gambar ulang elips OBSTACLE (biru)
            # Gunakan base_a dan base_b agar ukurannya STATIS
            obstacle_x, obstacle_y = self.calculate_ellipse_coordinates(
                h=self.h, k=self.k, a=self.base_a, b=self.base_b
            )
            self.ellipse.set_data(obstacle_x, obstacle_y)

        else:
            self.nearest_obstacle_minus150_to_150.set_data([], [])
            self.line_to_obstacle_minus150_to_150.set_data([], [])
            if self.annotation_minus150_to_150:
                self.annotation_minus150_to_150.remove()
                self.annotation_minus150_to_150 = None

        self.resultant_vector.set_data([0, self.resultant_vector_x], [0, self.resultant_vector_y])

        self.resultant_magnitude_text.set_text(f'Fnav: {self.resultant_magnitude:.2f} N')
        self.resultant_direction_text.set_text(f'Rho: {self.resultant_angle:.2f}°')

    def annotate_obstacle(self, annotation, x, y, text):
        if annotation:
            annotation.remove()
        annotation = self.ax.annotate(text, xy=(x, y), xytext=(x + 0.1, y + 0.1), fontsize=8, ha='left', va='top')
        return annotation

    def on_close(self, event):
        rospy.signal_shutdown('Window closed')

    def shutdown(self):
        rospy.loginfo("Shutting down...")
        plt.close(self.fig)

    def run(self):
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    visualizer = LaserVisualizer()
    visualizer.run()
