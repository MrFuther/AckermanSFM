import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.distance_min = 0.0  # Define minimum distance
        self.distance_max = 1.0  # Define maximum distance

    def scan_callback(self, data):
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        
        # Define the specific range of angles you want to consider
        angle_range_min = 90  # Minimum angle in degrees
        angle_range_max = 180  # Maximum angle in degrees
        
        # Filter distances and angles based on the defined range
        filtered_data = [(angle, distance) for angle, distance in zip(angles, data.ranges) 
                        if (self.distance_min <= distance <= self.distance_max) and
                           (angle >= np.radians(angle_range_min)) and
                           (angle <= np.radians(angle_range_max))]
        
        # Print the filtered data
        for angle, distance in filtered_data:
            rospy.loginfo(f"Angle: {np.degrees(angle)}, Distance: {distance}")
        
if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
