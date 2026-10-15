#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sklearn.cluster import DBSCAN
import math

from sklearn.neighbors import NearestNeighbors
from collections import defaultdict


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
        self.ax.set_xlim(-1, 1)  # Set x-axis limits
        self.ax.set_ylim(-1, 1)  # Set y-axis limits
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
            
            
            # Custom angle ranges
            custom_ranges = [
                # (90, 100),
                # (110, 115),
                # (125, 130), 
                # (135, 145),
                # (150,165), 
                # (175,180),
                
                # (-95, -90),
                # (-120, -115),
                # (-135, -130), 
                # (-145, -140),
                # (-165,-160), 
                # (-180,-175)
                
                (90, 180),
                (-180, -90),
            
                ]
            
            filtered_data = []
            
            # Filter distances and angles based on the defined range
            filtered_data_left = [(angle, distance) for angle, distance in zip(angles, self.scan_data.ranges) 
                            if (self.distance_min <= distance <= self.distance_max) and
                            ((angle >= np.radians(angle_range_rmin)) and (angle <= np.radians(angle_range_rmax)))
        
                            ]
            
            filtered_data_right = [(angle, distance) for angle, distance in zip(angles, self.scan_data.ranges) 
                if (self.distance_min <= distance <= self.distance_max) and
                ((angle >= np.radians(angle_range_min)) and (angle <= np.radians(angle_range_max)))

                ]
            
            
            
            # filtered_data = filtered_data_left + filtered_data_right
            
            # for angle, distance in zip(angles, self.scan_data.ranges):
            #     if (self.distance_min <= distance <= self.distance_max) and (angle >= np.radians(angle_range_min)) and (angle <= np.radians(angle_range_max)):
            #         # Check if angle falls within custom ranges
            #         for custom_range in custom_ranges:
            #             if (angle >= np.radians(custom_range[0])) and (angle <= np.radians(custom_range[1])):
            #                 filtered_data.append((angle, distance))
            #                 break  # Move to the next angle if this one is included in a custom range
            
            
            for angle, distance in zip(angles, self.scan_data.ranges):
                # left
                if (self.distance_min <= distance <= self.distance_max) and (angle >= np.radians(angle_range_rmin)) and (angle <= np.radians(angle_range_rmax)):
                    # Check if angle falls within custom ranges
                    for custom_range in custom_ranges:
                        if (angle >= np.radians(custom_range[0])) and (angle <= np.radians(custom_range[1])):
                            filtered_data.append((angle, distance))
                            break  # Move to the next angle if this one is included in a custom range
                # right
                if (self.distance_min <= distance <= self.distance_max) and (angle >= np.radians(angle_range_min)) and (angle <= np.radians(angle_range_max)):
                        # Check if angle falls within custom ranges
                        for custom_range in custom_ranges:
                            if (angle >= np.radians(custom_range[0])) and (angle <= np.radians(custom_range[1])):
                                filtered_data.append((angle, distance))
                                break  # Move to the next angle if this one is included in a custom range

            
            # Convert polar coordinates to Cartesian coordinates
            xs = [distance * np.cos(angle) for angle, distance in filtered_data]
            ys = [distance * np.sin(angle) for angle, distance in filtered_data]

            # Append the reference point (0.5, 0.0) as an additional point
            xs.append(0.5)
            ys.append(0.0)

            # Plot original LaserScan data
            self.scan_line.set_data(xs, ys)

            # Apply DBSCAN clustering to group nearby points
            points = np.array(list(zip(xs, ys)))
            db = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples).fit(points)
            labels = db.labels_
            unique_labels = set(labels)

            clustered_points = []
            
            # Iterate over each unique label
            for k in unique_labels:
                if k != -1:  # Ignore noise points
                    class_member_mask = (labels == k)
                    xy = points[class_member_mask]
                    
                    # Calculate distances from each point within the group to the reference point
                    distances_to_ref = [np.sqrt((x - 0.5)**2 + (y - 0.0)**2) for x, y in xy]
                    
                    # Get the index of the nearest point within the group
                    nearest_index = np.argmin(distances_to_ref)
                    
                    # Select the nearest point as the centroid for the group
                    centroid = xy[nearest_index]
                    
                    # Append the centroid to clustered_points
                    clustered_points.append(centroid)
                    
                    # Check if the group contains 5 or more points
                    if len(xy) >= 2:
                        # Calculate the centroid of the cluster group
                        centroid_x, centroid_y = centroid
                        
                        # Find the point on the edge of the cluster group closest to the centroid
                        edge_point_index = np.argmax(distances_to_ref)  # Index of the point farthest from the centroid
                        edge_point = xy[edge_point_index]
                        
                        # Append the edge point to clustered_points
                        clustered_points.append(edge_point)


            # Convert clustered_points to a NumPy array
            clustered_points = np.array(clustered_points)

            # Initialize cluster_xs and cluster_ys
            cluster_xs = []
            cluster_ys = []
            rcluster_xs = []
            rcluster_ys = []
            
            rcluster_xs_scaled = []
            rcluster_ys_scaled = []
            
            # Check if there are any clustered points
            if len(clustered_points) > 0:
                # Extract cluster_xs and cluster_ys
                cluster_xs = clustered_points[:, 0]
                cluster_ys = clustered_points[:, 1]
                
                rcluster_xs = (cluster_xs*-1)
                rcluster_ys = (cluster_ys*-1)
                
                force_distances = np.sqrt(rcluster_xs**2 + rcluster_ys**2)
                
                # scaling_factors = self.dynamic_scaling_factors(force_distances)
                
                sosial = self.force_social(force_distances)
                physical = self.force_physical(force_distances)
                force_static = sosial + physical
                
                rcluster_xs_scaled = rcluster_xs*force_static
                rcluster_ys_scaled = rcluster_ys*force_static
            else:
                # No clustered points to plot
                pass
            

            
            
                        
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
                annotation = self.ax.annotate(f'{force_static:.2f} N',
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
