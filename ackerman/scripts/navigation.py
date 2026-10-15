#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
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
        
        self.steering_pub = rospy.Publisher('/nav/steering', Point, queue_size=10)
        self.magnitude_pub = rospy.Publisher('/nav/magnitude', Point, queue_size=10)


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
        k = 1.0  # gain
        r = 1.0  # radius max jangkauan lidar
        psi = 0.20  # jarak reaktif in m
        Fsos = k * np.exp((r - distances) / psi)
        return Fsos
    
    def force_physical(self, distances):
        k = 1.0  # gain
        r = 1.0  # radius max jangkauan lidar
        Fphy = k * (r - distances)
        return Fphy

    def update_plot(self):
        if hasattr(self, 'scan_data'):
            angles = np.arange(self.scan_data.angle_min, self.scan_data.angle_max, self.scan_data.angle_increment)

            #scan hanya bagian depan saja
            angle_range_min = 90  # Minimum angle in degrees
            angle_range_max = 180  # Maximum angle in degrees
            
            angle_range_rmin = -180  # Minimum angle in degrees
            angle_range_rmax = -90  # Maximum angle in degrees
            
            # Custom angle ranges
            custom_ranges = [
                (90, 180),
                (-180, -90)
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
            # self.scan_line.set_data(xs, ys)

            # Apply DBSCAN clustering to group nearby points
            points = np.array(list(zip(xs, ys)))
            db = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples).fit(points)
            labels = db.labels_
            unique_labels = set(labels)

            clustered_points = []
            
            for k in unique_labels:
                if k != -1:
                    class_member_mask = (labels == k)
                    xy = points[class_member_mask]
                    
                    distances_to_ref = [np.sqrt((x - 0.5)**2 + (y - 0.0)**2) for x, y in xy]
                    
                    nearest_index = np.argmin(distances_to_ref)
                    
                    centroid = xy[nearest_index]
                    
                    clustered_points.append(centroid)
                    
                    if len(xy) >= 2:
                        centroid_x, centroid_y = centroid
                        
                        edge_point_index = np.argmax(distances_to_ref)
                        edge_point = xy[edge_point_index]
                        
                        clustered_points.append(edge_point)

            clustered_points = np.array(clustered_points)

            cluster_xs = []
            cluster_ys = []
            rcluster_xs = []
            rcluster_ys = []
            
            rcluster_xs_scaled = []
            rcluster_ys_scaled = []
            
            if len(clustered_points) > 0:
                cluster_xs = clustered_points[:, 0]
                cluster_ys = clustered_points[:, 1]
                
                rcluster_xs = (cluster_xs*-1)
                rcluster_ys = (cluster_ys*-1)
                
                force_distances = np.sqrt(rcluster_xs**2 + rcluster_ys**2)
                
                sosial = self.force_social(force_distances)
                physical = self.force_physical(force_distances)
                force_static = sosial + physical
                
                rcluster_xs_scaled = rcluster_xs*force_static
                rcluster_ys_scaled = rcluster_ys*force_static
            else:
                pass

            for x, y in zip(rcluster_xs, rcluster_ys):
                distance = np.sqrt(x**2 + y**2)
                
                sosial = self.force_social(distance)
                physical = self.force_physical(distance)
                force_static = sosial + physical

                angle = np.arctan2(y, x)
                angle_degrees = np.degrees(angle)
                
                rospy.loginfo(f"distance {distance:.2f} m, Force {force_static:2f} N.")
                
            for x, y in zip(cluster_xs, cluster_ys):
                distance = np.sqrt(x**2 + y**2)
                angle = np.arctan2(y, x)
                angle_degrees = np.degrees(angle)

            # Append F navigation
            rcluster_xs_scaled = np.append(rcluster_xs_scaled, self.additional_distance * np.cos(self.additional_angle_radians))
            rcluster_ys_scaled = np.append(rcluster_ys_scaled, self.additional_distance * np.sin(self.additional_angle_radians))
            
            resultant_x = np.sum(rcluster_xs_scaled)
            resultant_y = np.sum(rcluster_ys_scaled)
            resultant_magnitude = np.sqrt(resultant_x**2 + resultant_y**2)
            resultant_direction = np.arctan2(resultant_y, resultant_x)
            
            resultant_direction = np.degrees(resultant_direction)

            # Publish distance
            steering_msg = Point()
            steering_msg.x = resultant_direction
            self.steering_pub.publish(steering_msg)
            
            magnitude_msg = Point()
            magnitude_msg.x = (resultant_magnitude)
            self.magnitude_pub.publish(magnitude_msg)
            
            # rospy.loginfo(f"Magnitude = {resultant_magnitude}, Direction = {resultant_direction}")
            # rospy.loginfo("====================================================")

if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        while not rospy.is_shutdown():
            obstacle_detector.update_plot()
            rospy.sleep(0.00001)  # Sleep to control the loop rate
    except rospy.ROSInterruptException:
        pass
