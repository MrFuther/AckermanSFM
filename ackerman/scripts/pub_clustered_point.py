#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np
from sklearn.cluster import DBSCAN

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.distance_min = 0.0  # Define minimum distance
        self.distance_max = 1.0  # Define maximum distance
        self.cluster_eps = 0.1  # Define DBSCAN epsilon parameter for clustering
        self.cluster_min_samples = 5  # Define DBSCAN min_samples parameter for clustering
        self.clustered_point_publisher = rospy.Publisher('/clustered_points', Point, queue_size=10)
        self.scan_data = None

    def scan_callback(self, data):
        # Update scan data
        self.scan_data = data

    def publish_clustered_points(self, clustered_points):
        # Publish clustered points as Point messages
        for point in clustered_points:
            p = Point()
            p.x, p.y = point[0], point[1]
            self.clustered_point_publisher.publish(p)

    def detect_obstacles(self):
        if self.scan_data:
            angles = np.arange(self.scan_data.angle_min, self.scan_data.angle_max, self.scan_data.angle_increment)

            # Filter distances based on the defined range
            filtered_data = [(angle, distance) for angle, distance in zip(angles, self.scan_data.ranges)
                            if self.distance_min <= distance <= self.distance_max]

            # Convert polar coordinates to Cartesian coordinates
            xs = [distance * np.cos(angle) for angle, distance in filtered_data]
            ys = [distance * np.sin(angle) for angle, distance in filtered_data]

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
                    distance = np.linalg.norm(centroid)
                    angle = np.arctan2(centroid[1], centroid[0])
                    clustered_points.append((distance, angle))

            clustered_points = np.array(clustered_points)
            
            self.publish_clustered_points(clustered_points)

if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            obstacle_detector.detect_obstacles()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
