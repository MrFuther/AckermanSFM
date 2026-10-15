#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sklearn.cluster import DBSCAN

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.distance_min = 0.0  # Define minimum distance
        self.distance_max = 1.0  # Define maximum distance
        self.cluster_eps = 0.1  # Define DBSCAN epsilon parameter for clustering
        self.cluster_min_samples = 5  # Define DBSCAN min_samples parameter for clustering
        self.fig, self.ax = plt.subplots()
        self.scan_line, = self.ax.plot([], [], 'bo', markersize=2)  # Plot original LaserScan data
        self.cluster_line, = self.ax.plot([], [], 'ro', markersize=5)  # Plot clustered points
        self.ax.set_xlim(-2, 2)  # Set x-axis limits
        self.ax.set_ylim(-2, 2)  # Set y-axis limits
        self.ax.set_aspect('equal', adjustable='box')  # Equal aspect ratio
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('LaserScan Data')
        self.scan_data = None

    def scan_callback(self, data):
        # Update scan data
        self.scan_data = data

    def update_plot(self, frame):
        if self.scan_data:
            angles = np.arange(self.scan_data.angle_min, self.scan_data.angle_max, self.scan_data.angle_increment)

            # Filter distances based on the defined range
            filtered_data = [(angle, distance) for angle, distance in zip(angles, self.scan_data.ranges) 
                            if self.distance_min <= distance <= self.distance_max]

            # Convert polar coordinates to Cartesian coordinates
            xs = [distance * np.cos(angle) for angle, distance in filtered_data]
            ys = [distance * np.sin(angle) for angle, distance in filtered_data]

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

            # Plot clustered points obtained from DBSCAN in red
            self.cluster_line.set_data(cluster_xs, cluster_ys)
            
            # Calculate distance and angle of clustered points
            for x, y in zip(cluster_xs, cluster_ys):
                distance = np.sqrt(x**2 + y**2)
                angle = np.arctan2(y, x)
                rospy.loginfo(f"Clustered point at distance {distance:.2f} meters and angle {np.degrees(angle):.2f} degrees.")
                rospy.loginfo(f"-------")

        return self.scan_line, self.cluster_line

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
