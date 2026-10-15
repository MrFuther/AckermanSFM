#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sklearn.cluster import DBSCAN
import math

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.additional_distance = 35.0  # meters
        self.additional_angle_degrees = 180  # degrees
        self.additional_angle_radians = math.radians(self.additional_angle_degrees)
        
        self.distance_min = 0.0  # Define minimum distance
        self.distance_max = 1  # Define maximum distance
        self.cluster_eps = 0.1  # Define DBSCAN epsilon parameter for clustering
        self.cluster_min_samples = 5  # Define DBSCAN min_samples parameter for clustering
        self.fig, self.ax = plt.subplots()
        self.scan_line, = self.ax.plot([], [], 'bo', markersize=2)  # Plot original LaserScan data
        self.cluster_line, = self.ax.plot([], [], 'ro', markersize=5)  # Plot clustered points
        self.resultant_line, = self.ax.plot([], [], 'go', markersize=10)  # Plot resultant vector
        self.ax.set_xlim(-2, 2)  # Set x-axis limits
        self.ax.set_ylim(-2, 2)  # Set y-axis limits
        self.ax.set_aspect('equal', adjustable='box')  # Equal aspect ratio
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Ackerman Mobile Robot')
        self.scan_data = None
        self.resultant_vector_arrow = None
        self.lines = []
        self.reset_lines()
        self.annotations = []
        self.reset_annotations()
        self.custom_annotation = None
        
        self.custom_distances_fgoal = []
        self.custom_angles_fgoal = []
        
        # Social force
        self.Fsos = 0.0
        self.Fphy = 0.0
        self.k = 1.0 #gain
        self.r = 1.0 #radius max jangkauan lidar
        self.d = 0.0 
        self.psi = 0.20 #jarak reaktif in m
        
        
        
        # self.fig, self.ax = plt.subplots()
        self.annotations = []

        self.steering_pub = rospy.Publisher('/nav/steering', Point, queue_size=10)
        self.magnitude_pub = rospy.Publisher('/nav/magnitude', Point, queue_size=10)


    def reset_lines(self):
        for line in self.lines:
            line.remove()
        self.lines = []
        
    def reset_annotations(self):
        for line in self.lines:
            line.remove()
        self.lines = []
        for annotation in self.annotations:
            annotation.remove()
        self.annotations = []
        
    def update_custom_annotation(self, custom_x, custom_y):
        # Update the position of the custom annotation
        if self.custom_annotation:
            self.custom_annotation.set_position((custom_x, custom_y))
        else:
            # Create the custom annotation if it doesn't exist
            self.custom_annotation = self.ax.annotate('', xy=(custom_x, custom_y),
                                                    xytext=(5, -5),
                                                    textcoords='offset points',
                                                    ha='left',
                                                    va='top')
        
    def scan_callback(self, data):
        # Update scan data
        self.scan_data = data
        
    def dynamic_scaling_factors(self, distances):
        epsilon = 0.1  # Small constant to avoid division by zero
        scaled_distances = distances + epsilon
        scaling_factors = 1 / scaled_distances
        return scaling_factors
    
    def force_social(self, distances):
        # Calculate the force based on the defined equation
        self.Fsos = self.k * np.exp((self.r - distances) / self.psi)
        return self.Fsos
    
    def force_physical(self, distances):
        self.Fphy = self.k * (self.r - distances)

        return self.Fphy

    def update_plot(self, frame):
        if self.scan_data:
            angles = np.arange(self.scan_data.angle_min, self.scan_data.angle_max, self.scan_data.angle_increment)

            #scan hanya bagian depan saja
            angle_range_min = 90  # Minimum angle in degrees
            angle_range_max = 180  # Maximum angle in degrees
            
            angle_range_rmin = -180  # Minimum angle in degrees
            angle_range_rmax = -90  # Maximum angle in degrees
            
            # Filter distances and angles based on the defined range
            filtered_data_kiri = [(angle, distance) for angle, distance in zip(angles, self.scan_data.ranges) 
                            if (self.distance_min <= distance <= self.distance_max) and
                            ((angle >= np.radians(angle_range_rmin)) and (angle <= np.radians(angle_range_rmax)))
        
                            ]
            filtered_data_kanan = [(angle, distance) for angle, distance in zip(angles, self.scan_data.ranges) 
                if (self.distance_min <= distance <= self.distance_max) and
                ((angle >= np.radians(angle_range_min)) and (angle <= np.radians(angle_range_max)))

                ]
            
            filtered_data = filtered_data_kiri + filtered_data_kanan

            
            # Convert polar coordinates to Cartesian coordinates
            xs = [distance * np.cos(angle) for angle, distance in filtered_data]
            ys = [distance * np.sin(angle) for angle, distance in filtered_data]

            # Append the reference point (0.5, 0.0) as an additional point
            xs.append(0.5)
            ys.append(0.0)

            # Plot original LaserScan data
            self.scan_line.set_data(xs, ys)

            # Find the nearest point to the robot
            nearest_distance = float('inf')
            nearest_point = None
            for x, y in zip(xs, ys):
                distance = np.sqrt((x - 0.5)**2 + (y - 0.0)**2)
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_point = (x, y)

            # Plot the nearest point in red
            if nearest_point:
                self.cluster_line.set_data([nearest_point[0]], [nearest_point[1]])
            
            # Publish distance and angle
            if nearest_point:
                nearest_angle = np.arctan2(nearest_point[1], nearest_point[0])
                nearest_angle_degrees = np.degrees(nearest_angle)
                steering_msg = Point()
                steering_msg.x = nearest_angle_degrees
                self.steering_pub.publish(steering_msg)
                
                magnitude_msg = Point()
                magnitude_msg.x = nearest_distance
                self.magnitude_pub.publish(magnitude_msg)
                
                rospy.loginfo(f"Nearest Distance = {nearest_distance:.2f}, Nearest Angle = {nearest_angle_degrees:.2f}")

        
            

            
            rcluster_xs = (nearest_point[0]*-1)
            rcluster_ys = (nearest_point[1]*-1)
            
            force_distances = np.sqrt(rcluster_xs**2 + rcluster_ys**2)
            
            scaling_factors = self.dynamic_scaling_factors(force_distances)
            
            sosial = self.force_social(force_distances)
            physical = self.force_physical(force_distances)
            force_static = sosial + physical
            
            rcluster_xs_scaled = rcluster_xs*force_static
            rcluster_ys_scaled = rcluster_ys*force_static
                        
            # print(f"dis {force_distances}")
            # print(f"sos {sosial}")
            # print(f"phy {physical}")
            # print(f"static {force_static}")
            
            
            # for x, y in zip(rcluster_xs, rcluster_ys):
            #     force_distance = np.sqrt(x**2 + y**2)
            #     rospy.loginfo(f"dis {force_distance:.2f} m")
            #     scaling_factor = self.dynamic_scaling_factor(force_distance)
            #     rcluster_xs= rcluster_xs*scaling_factor
            #     rcluster_ys= rcluster_ys*scaling_factor

            # Plot clustered points obtained from DBSCAN in red
            self.cluster_line.set_data(rcluster_xs, rcluster_ys)
            # self.cluster_line.set_data(cluster_xs, cluster_ys)
            
            for line in self.lines:
                line.remove()
            self.lines = []
            
            self.reset_annotations()
            
            # FORCE merah
            for x, y in zip(rcluster_xs, rcluster_ys):
                distance = np.sqrt(x**2 + y**2)
                
                sosial = self.force_social(distance)
                physical = self.force_physical(distance)
                force_static = sosial + physical

                # distance = force_static
                
                angle = np.arctan2(y, x)
                angle_degrees = np.degrees(angle)
                
                rospy.loginfo(f"distance {distance:.2f} m, Force {force_static:2f} N.")
                
                # Plot the line
                line_force = self.ax.plot([0, x], [0, y], color='red')[0]
                self.lines.append(line_force)
                
                # Annotate the line with distance and angle information
                annotation = self.ax.annotate(f'{force_static:.2f} m\n{angle_degrees:.2f}°',
                                            xy=(x, y),
                                            xytext=(5, -5),
                                            textcoords='offset points',
                                            ha='left',
                                            va='top')
                self.annotations.append(annotation)
                
            # DISTANCE HIJAU
            for x, y in zip(cluster_xs, cluster_ys):
                distance = np.sqrt(x**2 + y**2)
                angle = np.arctan2(y, x)
                angle_degrees = np.degrees(angle)
                
                # rospy.loginfo(f"dis {distance:.2f} m, angle {angle_degrees:2f} dgre.")
                
                # Plot the line
                line = self.ax.plot([0, x], [0, y], color='green')[0]
                self.lines.append(line)
                
                # Annotate the line with distance and angle information
                annotation = self.ax.annotate(f'{distance:.2f} m\n{angle_degrees:.2f}°',
                                            xy=(x, y),
                                            xytext=(5, -5),
                                            textcoords='offset points',
                                            ha='left',
                                            va='top')
                self.annotations.append(annotation)
            
            
            # Append F navigation
            rcluster_xs_scaled = np.append(rcluster_xs_scaled, self.additional_distance * np.cos(self.additional_angle_radians))
            rcluster_ys_scaled = np.append(rcluster_ys_scaled, self.additional_distance * np.sin(self.additional_angle_radians))
            
            # Plot the line for F navigation
            line_f_navigation = self.ax.plot([0, rcluster_xs_scaled[-1]], [0, rcluster_ys_scaled[-1]], color='yellow')[0]
            self.lines.append(line_f_navigation)

            # Calculate resultant vector of distances and angles
            resultant_x = np.sum(rcluster_xs_scaled)
            resultant_y = np.sum(rcluster_ys_scaled)
            resultant_magnitude = np.sqrt(resultant_x**2 + resultant_y**2)
            resultant_direction = np.arctan2(resultant_y, resultant_x)
            
            resultant_direction = np.degrees(resultant_direction)

            # Plot or update resultant vector
            if self.resultant_vector_arrow is None:
                self.resultant_vector_arrow = self.ax.quiver(0, 0, resultant_x, resultant_y, angles='xy', scale_units='xy', scale=1, color='blue')
            else:
                self.resultant_vector_arrow.set_UVC(resultant_x, resultant_y)
                self.resultant_vector_arrow.set_offsets([0, 0])

            # Publish distance
            steering_msg = Point()
            steering_msg.x = resultant_direction
            self.steering_pub.publish(steering_msg)
            
            magnitude_msg = Point()
            magnitude_msg.x = (resultant_magnitude)
            self.magnitude_pub.publish(magnitude_msg)
            
            rospy.loginfo(f"Magnitude = {resultant_magnitude}, Direction = {resultant_direction}")
            rospy.loginfo("====================================================")

        return self.scan_line, self.cluster_line, self.resultant_vector_arrow, *self.lines, *self.annotations

    def start(self):
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=5, blit=True)
        plt.show()

if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        obstacle_detector.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
