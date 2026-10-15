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
        
        self.additional_distance = 2.0  # meters
        self.additional_angle_degrees = 0  # degrees
        self.additional_angle_radians = math.radians(self.additional_angle_degrees)
        
        self.distance_min = 0.0  # Define minimum distance
        self.distance_max = 1.0  # Define maximum distance
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
            # rospy.loginfo(f"fil {filtered_data.distance}")

            # Convert polar coordinates to Cartesian coordinates
            xs = [distance * np.cos(angle) for angle, distance in filtered_data]
            ys = [distance * np.sin(angle) for angle, distance in filtered_data]

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
            for k in unique_labels:
                if k != -1:  # Ignore noise points
                    class_member_mask = (labels == k)
                    xy = points[class_member_mask]
                    centroid = np.mean(xy, axis=0)
                    clustered_points.append(centroid)

            clustered_points = np.array(clustered_points)
            cluster_xs = clustered_points[:, 0]
            cluster_ys = clustered_points[:, 1]
            
            # Append the additional point
            cluster_xs = np.append(cluster_xs, self.additional_distance * np.cos(self.additional_angle_radians))
            cluster_ys = np.append(cluster_ys, self.additional_distance * np.sin(self.additional_angle_radians))
                
            rcluster_xs = cluster_xs*-1
            rcluster_ys = cluster_ys*-1
            
            # Plot clustered points obtained from DBSCAN in red
            self.cluster_line.set_data(rcluster_xs, rcluster_ys)
            # self.cluster_line.set_data(cluster_xs, cluster_ys)
            
            for line in self.lines:
                line.remove()
            self.lines = []
            
            self.reset_annotations()
            
            for x, y in zip(rcluster_xs, rcluster_ys):
                distance = np.sqrt(x**2 + y**2)
                angle = np.arctan2(y, x)
                angle_degrees = np.degrees(angle)
                
                rospy.loginfo(f"dis {distance:.2f} m, angle {angle_degrees:2f} dgre.")
                
                # Plot the line
                line = self.ax.plot([0, x], [0, y], color='green')[0]
                line_force = self.ax.plot([0, x], [0, y], color='red')[0]
                self.lines.append(line)
                self.lines.append(line_force)
                
                # Annotate the line with distance and angle information
                annotation = self.ax.annotate(f'{distance:.2f} m\n{angle_degrees:.2f}°',
                                            xy=(x, y),
                                            xytext=(5, -5),
                                            textcoords='offset points',
                                            ha='left',
                                            va='top')
                self.annotations.append(annotation)
                
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
            
            # Calculate resultant vector of distances and angles
            resultant_x = np.sum(rcluster_xs)
            resultant_y = np.sum(rcluster_ys)
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
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=True)
        plt.show()

if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        obstacle_detector.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
